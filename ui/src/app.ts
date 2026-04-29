import {
  Vec2,
  EdgeArc,
  Polygon,
  DriveLine,
  DriveAisleGraph,
  ParkingParams,
  ParkingLot,
  GenerateInput,
  DebugToggles,
  GridStop,
  Target,
  AisleDirection,
  StallKind,
  arcApex,
  bulgeFromApex,
  computeBoundaryPin,
  evalBoundaryEdge,
  annotationWorldPos,
  projectToArc,
  splitArcAt,
  worldToPerimeterPos,
  worldToAbstractVertex,
  worldToSpliceVertex,
  effectiveDepth,
  ensurePolygonIds,
  nextVertexId,
} from "./types";
import { SnapGuide, SnapState, emptySnapState } from "./snap";
import { showToast } from "./toast";
import {
  resolve_grid_target_js,
  targets_equal_js,
} from "./wasm/parking_lot_engine";

/** Default bulge applied when the user first turns an edge into an arc. */
function defaultArcForEdge(): EdgeArc {
  // +0.3 is a visible-but-gentle bulge (≈34° sweep, sagitta ≈ 15% of
  // chord length). Sign follows arcol convention: positive = CCW-perp
  // side of start→end.
  return { bulge: 0.3 };
}

export interface Camera {
  offsetX: number;
  offsetY: number;
  zoom: number;
}

export interface VertexRef {
  type: "boundary-outer" | "boundary-hole" | "aisle" | "drive-line" | "stall-line" | "annotation" | "aisle-vector" | "region-vector" | "edge-midpoint";
  index: number;
  holeIndex?: number;
  endpoint?: "start" | "end" | "body";
  cpTarget?: "outer" | "hole";
}

export interface EdgeRef {
  index: number; // seed edge index
  chain: number[]; // all deduplicated edge indices in the collinear chain
  mode: "chain" | "segment"; // chain = full line, segment = single edge
}

export type EditMode =
  | "select"
  | "add-hole"
  | "add-drive-line"
  | "add-stall-line"
  | "add-boundary";

export interface LayerVisibility {
  stalls: boolean;
  customStalls: boolean;
  vertices: boolean;
  /** Drive lines and all annotation markers (Direction / DeleteVertex /
   *  DeleteEdge). Grouped because they're all user-drawn substrate
   *  overlays — toggling one without the others rarely makes sense. */
  annotations: boolean;
  spines: boolean;
  faces: boolean;
  islands: boolean;
  regions: boolean;
  paintLines: boolean;
}

export interface AppState {
  lot: ParkingLot;
  params: ParkingParams;
  debug: DebugToggles;
  selectedVertex: VertexRef | null;
  hoveredVertex: VertexRef | null;
  selectedEdge: EdgeRef | null;
  isDragging: boolean;
  camera: Camera;
  editMode: EditMode;
  // For add-hole mode: vertices being placed
  pendingHole: Vec2[];
  // For add-boundary mode: vertices being placed
  pendingBoundary: Vec2[];
  // For add-drive-line mode
  pendingDriveLine: Vec2 | null;
  pendingDriveLinePreview: Vec2 | null;
  // For add-stall-line mode
  pendingStallLine: Vec2 | null;
  pendingStallLinePreview: Vec2 | null;
  layers: LayerVisibility;
  snapGuides: SnapGuide[];
  snapState: SnapState;
}

export type GenerateFn = (input: string) => string;

export class App {
  state: AppState;
  private generateFn: GenerateFn;
  private onUpdate: () => void;
  private nextDriveLineId = 1;

  /** Mint a fresh drive-line id. Splice annotations key off this. */
  newDriveLineId(): number {
    return this.nextDriveLineId++;
  }

  /** Advance the id allocator past a specific value — used when loading
   *  a fixture that assigns explicit ids, so future auto-mints don't
   *  collide with them. */
  bumpDriveLineId(past: number): void {
    if (past >= this.nextDriveLineId) {
      this.nextDriveLineId = past + 1;
    }
  }

  constructor(generateFn: GenerateFn, onUpdate: () => void) {
    this.generateFn = generateFn;
    this.onUpdate = onUpdate;

    const defaultLot: ParkingLot = {
      boundary: {
        outer: [
          { x: -11.97, y: 49.12 },
          { x: 692.14, y: 0 },
          { x: 782.80, y: 654.85 },
          { x: -166.61, y: 638.35 },
        ],
        holes: [
          [
            { x: 394.53, y: 251.80 },
            { x: 611.14, y: 251.80 },
            { x: 613.91, y: 512.33 },
            { x: 394.53, y: 512.33 },
          ],
        ],
        outer_arcs: [null, { bulge: -0.524 }, null, null],
        hole_arcs: [[{ bulge: -0.796 }, null, { bulge: 0.400 }, null]],
        outer_ids: [1, 2, 3, 4],
        hole_ids: [[5, 6, 7, 8]],
      },
      annotations: [
        {
          kind: "Direction",
          target: {
            on: "Grid",
            region: 84978695160110,
            axis: "X",
            coord: -5,
            range: [{ at: "Lattice", other: -3 }, { at: "Lattice", other: -1 }],
          },
          traffic: "OneWay",
        },
        {
          kind: "DeleteEdge",
          target: {
            on: "Grid",
            region: 84978695160110,
            axis: "Y",
            coord: -2,
            range: [{ at: "Lattice", other: -9 }, { at: "Lattice", other: -4 }],
          },
        },
        {
          kind: "DeleteEdge",
          target: { on: "DriveLine", id: 1, t: 0.80 },
        },
        {
          kind: "DeleteEdge",
          target: {
            on: "Grid",
            region: 42210882214992,
            axis: "X",
            coord: -5,
            range: [{ at: "Lattice", other: 0 }, { at: "Lattice", other: 2 }],
          },
        },
        {
          kind: "DeleteVertex",
          target: {
            on: "Perimeter",
            loop: { kind: "Outer" },
            start: 2,
            end: 3,
            t: 0.63,
          },
        },
        {
          kind: "DeleteEdge",
          target: {
            on: "Grid",
            region: 42210882214992,
            axis: "X",
            coord: -16,
            range: [{ at: "Lattice", other: 0 }, { at: "Lattice", other: 2 }],
          },
        },
        {
          kind: "DeleteEdge",
          target: {
            on: "Grid",
            region: 42210882214992,
            axis: "Y",
            coord: 1,
            range: [{ at: "Lattice", other: -16 }, { at: "Lattice", other: -15 }],
          },
        },
        {
          kind: "Direction",
          target: {
            on: "Grid",
            region: 42210882214992,
            axis: "X",
            coord: -4,
            range: [{ at: "Lattice", other: 0 }, { at: "Lattice", other: 2 }],
          },
          traffic: "OneWayReverse",
        },
        {
          kind: "Direction",
          target: {
            on: "Grid",
            region: 42210882214992,
            axis: "X",
            coord: -3,
            range: [{ at: "Lattice", other: 0 }, { at: "Lattice", other: 2 }],
          },
          traffic: "TwoWayReverse",
        },
        {
          kind: "Direction",
          target: {
            on: "Grid",
            region: 42210882214992,
            axis: "X",
            coord: -1,
            range: [{ at: "Lattice", other: 0 }, { at: "Lattice", other: 2 }],
          },
          traffic: "OneWayReverse",
        },
        {
          kind: "Direction",
          target: {
            on: "Grid",
            region: 42210882214992,
            axis: "X",
            coord: -12,
            range: [{ at: "Lattice", other: 0 }, { at: "Lattice", other: 2 }],
          },
          traffic: "TwoWayReverse",
        },
      ],
      aisleVector: aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
      aisleOffsetBaseline: midpointPerpProj(
        aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
        90,
      ),
      driveLines: [
        { id: 1, start: { x: 312.28, y: 518.60 }, end: { x: 144.72, y: 515.73 }, partitions: false },
        { id: 2, start: { x: 1186.57, y: 153.66 }, end: { x: 475.79, y: 349.25 }, partitions: true },
        { id: 3, start: { x: 446.56, y: 335.61 }, end: { x: 454.87, y: -53.66 }, partitions: true },
      ],
      stallModifiers: [
        {
          polyline: [
            { x: 205.37, y: 39.08 },
            { x: 244.63, y: 39.08 },
          ],
          kind: "Suppressed",
        },
        {
          polyline: [
            { x: 414.14, y: 434.72 },
            { x: 412.17, y: 298.61 },
          ],
          kind: "Ada",
        },
        {
          polyline: [
            { x: 502.84, y: 638.35 },
            { x: 704.25, y: 646.60 },
          ],
          kind: "Compact",
        },
        {
          polyline: [
            { x: 817.96, y: 446.80 },
            { x: 712.33, y: 536.65 },
          ],
          kind: "Island",
        },
        {
          polyline: [
            { x: 201.87, y: 127.27 },
            { x: 201.87, y: 169.58 },
          ],
          kind: "Island",
        },
        {
          polyline: [
            { x: 21.54, y: 434.72 },
            { x: 21.54, y: 489.73 },
          ],
          kind: "Suppressed",
        },
      ],
      regionOverrides: {
        "221108085334326": { offset: 154.58 },
        "5138299582397": { angle: 2, offset: -79.50 },
        "42210882214992": { angle: 92 },
      },
      layout: null,
    };

    this.state = {
      lot: defaultLot,
      params: {
        stall_width: 9,
        stall_depth: 18,
        aisle_width: 12,
        stall_angle: 45,
        aisle_angle: 90,
        aisle_offset: 0,
        site_offset: 0,
        stalls_per_face: 29,
        use_regions: true,
        island_stall_interval: 12,
        min_stalls_per_spine: 3,
        arc_discretize_tolerance: 5,
        spine_merge_angle_deg: 8.1,
        spine_merge_endpoint_tol: 1,
        island_corner_radius: 6,
        ada_stall_width: 8,
        compact_stall_width: 7.5,
        ada_buffer_width: 5,
        ada_buffer_shared: true,
      },
      debug: {
        spine_end_trim: true,
        spine_merging: true,
        spine_extensions: true,
        stall_face_clipping: true,
        entrance_on_face_filter: true,
        conflict_removal: true,
        island_stall_dilation: true,
      },
      selectedVertex: null,
      hoveredVertex: null,
      selectedEdge: null,
      isDragging: false,
      camera: { offsetX: 30, offsetY: 60, zoom: 1.3 },
      editMode: "select",
      pendingHole: [],
      pendingBoundary: [],
      pendingDriveLine: null,
      pendingDriveLinePreview: null,
      pendingStallLine: null,
      pendingStallLinePreview: null,
      snapGuides: [],
      snapState: emptySnapState(),
      layers: {
        stalls: false,
        customStalls: true,
        vertices: true,
        annotations: true,
        spines: false,
        faces: false,
        islands: false,
        regions: false,
        paintLines: true,
      },
    };
  }

  /** Replace the lot's boundary (used by add-boundary mode). Resets
   *  driveLines, annotations, region overrides, and reorients the
   *  aisle vector over the new boundary's centroid. */
  replaceBoundary(boundary: Polygon): void {
    const cx = boundary.outer.reduce((s, v) => s + v.x, 0) / boundary.outer.length;
    const cy = boundary.outer.reduce((s, v) => s + v.y, 0) / boundary.outer.length;
    const initialVector = aisleVectorFromAngle(90, 0, { x: cx, y: cy });
    this.state.lot = {
      boundary,
      driveLines: [],
      stallModifiers: [],
      annotations: [],
      aisleVector: initialVector,
      aisleOffsetBaseline: midpointPerpProj(initialVector, 90),
      regionOverrides: {},
      layout: null,
    };
  }

  generate(): void {
    const lot = this.state.lot;
    // Defensive id sync — catches any boundary mutation site that
    // didn't keep outer_ids/hole_ids parallel.
    ensurePolygonIds(lot.boundary);
    const input: GenerateInput = {
      boundary: lot.boundary,
      drive_lines: lot.driveLines,
      annotations: lot.annotations.filter((a) => a._active !== false),
      params: this.state.params,
      debug: this.state.debug,
      regionOverrides: Object.entries(lot.regionOverrides).map(([k, v]) => ({
        region_id: Number(k),
        aisle_angle: v.angle,
        aisle_offset: v.offset,
      })),
      stall_modifiers: lot.stallModifiers,
    };
    const inputJson = JSON.stringify(input);
    (window as any).__parkingInput = inputJson;
    try {
      const resultJson = this.generateFn(inputJson);
      lot.layout = JSON.parse(resultJson);
      const dormant = lot.layout?.dormant_annotations ?? [];
      if (dormant.length > 0) {
        console.groupCollapsed(
          `[generate] ${dormant.length} dormant annotation(s)`,
        );
        for (const d of dormant) {
          const a = lot.annotations[d.index];
          console.log(`#${d.index} ${a?.kind ?? "?"} — ${d.reason}`, a);
        }
        console.groupEnd();
        const which = dormant
          .map((d) => {
            const a = lot.annotations[d.index];
            return `#${d.index} ${a?.kind ?? "?"}: ${d.reason}`;
          })
          .join("; ");
        showToast(`${dormant.length} annotation(s) dormant — ${which}`);
      }
    } catch (e) {
      console.error("Generate failed:", e);
    }
    this.onUpdate();
  }

  setParam(key: keyof ParkingParams, value: number): void {
    (this.state.params as any)[key] = value;
    if (key === "aisle_angle" || key === "aisle_offset") {
      this.syncAisleVector(this.state.lot);
    }
    this.generate();
  }

  private syncAisleVector(lot: ParkingLot): void {
    // Rebuild the vector at its current midpoint with the new angle,
    // then reposition it perpendicular to the aisle so its midpoint
    // projection equals `baseline + aisle_offset`. Preserving the
    // along-aisle component keeps the vector visually anchored
    // alongside the same rough region of the lot even as it slides
    // perpendicular with aisle_offset changes.
    const vec = lot.aisleVector;
    const center = {
      x: (vec.start.x + vec.end.x) / 2,
      y: (vec.start.y + vec.end.y) / 2,
    };
    const angleDeg = this.state.params.aisle_angle;
    const angleRad = angleDeg * (Math.PI / 180);
    const dirX = Math.cos(angleRad);
    const dirY = Math.sin(angleRad);
    const perpX = -Math.sin(angleRad);
    const perpY = Math.cos(angleRad);
    const alongProj = center.x * dirX + center.y * dirY;
    const targetPerpProj =
      lot.aisleOffsetBaseline + this.state.params.aisle_offset;
    const newCenter: Vec2 = {
      x: alongProj * dirX + targetPerpProj * perpX,
      y: alongProj * dirY + targetPerpProj * perpY,
    };
    lot.aisleVector = aisleVectorFromAngle(angleDeg, 0, newCenter);
  }

  getEffectiveAisleGraph(): DriveAisleGraph | null {
    return this.state.lot.layout?.resolved_graph ?? null;
  }

  getAllVertices(): { ref: VertexRef; pos: Vec2 }[] {
    const result: { ref: VertexRef; pos: Vec2 }[] = [];
    const lot = this.state.lot;

    // Region vector endpoints — highest hit-test priority.
    const rd = lot.layout?.region_debug;
    if (rd) {
      const halfLen = 30;
      for (let i = 0; i < rd.regions.length; i++) {
        const region = rd.regions[i];
        const angleRad = region.aisle_angle * (Math.PI / 180);
        const dirX = Math.cos(angleRad);
        const dirY = Math.sin(angleRad);
        const cx = region.center.x;
        const cy = region.center.y;
        result.push({
          ref: { type: "region-vector", index: i, endpoint: "start" },
          pos: { x: cx - dirX * halfLen, y: cy - dirY * halfLen },
        });
        result.push({
          ref: { type: "region-vector", index: i, endpoint: "end" },
          pos: { x: cx + dirX * halfLen, y: cy + dirY * halfLen },
        });
      }
    }
    // Annotation anchors: resolve each annotation's target to its
    // current world position. Dormant annotations (substrate missing)
    // return null and are skipped.
    lot.annotations.forEach((ann, i) => {
      const pos = annotationWorldPos(ann, lot, this.state.params);
      if (pos) {
        result.push({ ref: { type: "annotation", index: i }, pos });
      }
    });
    // Drive-line vertices.
    lot.driveLines.forEach((dl, i) => {
      result.push({ ref: { type: "drive-line", index: i, endpoint: "start" }, pos: dl.start });
      result.push({ ref: { type: "drive-line", index: i, endpoint: "end" }, pos: dl.end });
    });
    // Stall-modifier line vertices.
    lot.stallModifiers.forEach((sm, i) => {
      const pl = sm.polyline;
      if (pl.length >= 1) {
        result.push({ ref: { type: "stall-line", index: i, endpoint: "start" }, pos: pl[0] });
      }
      if (pl.length >= 2) {
        result.push({ ref: { type: "stall-line", index: i, endpoint: "end" }, pos: pl[pl.length - 1] });
      }
    });
    // Edge midpoint handles — highest priority, shown on every edge.
    // Dragging creates or adjusts the edge's arc (apex follows the
    // drag; snapping back to the chord removes the arc).
    for (let i = 0; i < lot.boundary.outer.length; i++) {
      const a = lot.boundary.outer[i];
      const b = lot.boundary.outer[(i + 1) % lot.boundary.outer.length];
      const arc = lot.boundary.outer_arcs?.[i];
      const pos = arc
        ? arcApex(a, b, arc.bulge)
        : { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
      result.push({ ref: { type: "edge-midpoint", index: i, cpTarget: "outer" }, pos });
    }
    // Boundary vertices.
    lot.boundary.outer.forEach((v, i) => {
      result.push({ ref: { type: "boundary-outer", index: i }, pos: v });
    });
    lot.boundary.holes.forEach((hole, hi) => {
      // Edge midpoint handles for hole edges.
      for (let i = 0; i < hole.length; i++) {
        const a = hole[i];
        const b = hole[(i + 1) % hole.length];
        const arc = lot.boundary.hole_arcs?.[hi]?.[i];
        const pos = arc
          ? arcApex(a, b, arc.bulge)
          : { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
        result.push({
          ref: { type: "edge-midpoint", index: i, cpTarget: "hole", holeIndex: hi },
          pos,
        });
      }
      // Hole vertices.
      hole.forEach((v, vi) => {
        result.push({ ref: { type: "boundary-hole", index: vi, holeIndex: hi }, pos: v });
      });
    });
    const graph = this.getEffectiveAisleGraph();
    if (graph) {
      // Skip orphan vertices (no incident edges) — left behind by
      // delete-vertex annotations since the engine doesn't compact
      // graph.vertices. The renderer hides them; hit-testing must too,
      // otherwise an invisible orphan can steal a click meant for the
      // annotation marker drawn at the same spot.
      const degree = new Int32Array(graph.vertices.length);
      for (const e of graph.edges) {
        degree[e.start]++;
        degree[e.end]++;
      }
      graph.vertices.forEach((v, i) => {
        if (degree[i] === 0) return;
        result.push({ ref: { type: "aisle", index: i }, pos: v });
      });
    }
    return result;
  }

  moveVertex(ref: VertexRef, pos: Vec2): void {
    const lot = this.state.lot;
    if (ref.type === "edge-midpoint") {
      // Drag edge midpoint → update the edge's arc bulge so its apex
      // lands at `pos` (projected onto the chord-perpendicular bisector;
      // off-axis drag is ignored). Snap back to straight if the apex
      // returns near the chord.
      const verts = ref.cpTarget === "outer" ? lot.boundary.outer : lot.boundary.holes[ref.holeIndex!];
      const n = verts.length;
      const a = verts[ref.index];
      const b = verts[(ref.index + 1) % n];
      const bulge = bulgeFromApex(a, b, pos);
      const SNAP_EPS = 0.02;

      if (ref.cpTarget === "outer") {
        if (Math.abs(bulge) < SNAP_EPS) {
          if (lot.boundary.outer_arcs?.[ref.index]) lot.boundary.outer_arcs[ref.index] = null;
        } else {
          if (!lot.boundary.outer_arcs) lot.boundary.outer_arcs = new Array(n).fill(null);
          lot.boundary.outer_arcs[ref.index] = { bulge };
        }
      } else {
        const hi = ref.holeIndex!;
        if (Math.abs(bulge) < SNAP_EPS) {
          const ha = lot.boundary.hole_arcs?.[hi];
          if (ha?.[ref.index]) ha[ref.index] = null;
        } else {
          if (!lot.boundary.hole_arcs) lot.boundary.hole_arcs = [];
          if (!lot.boundary.hole_arcs[hi]) {
            lot.boundary.hole_arcs[hi] = new Array(n).fill(null);
          }
          lot.boundary.hole_arcs[hi][ref.index] = { bulge };
        }
      }
    } else if (ref.type === "boundary-outer") {
      // Arc bulges are stored as a signed scalar independent of
      // endpoint positions, so moving an endpoint automatically
      // reshapes adjacent arcs — no bookkeeping needed here.
      lot.boundary.outer[ref.index] = pos;
      for (const dl of lot.driveLines) {
        this.resolveBoundaryPin(dl, lot);
      }
    } else if (ref.type === "boundary-hole" && ref.holeIndex !== undefined) {
      lot.boundary.holes[ref.holeIndex][ref.index] = pos;
      for (const dl of lot.driveLines) {
        if (dl.holePin?.holeIndex === ref.holeIndex && dl.holePin?.vertexIndex === ref.index) {
          dl.start = pos;
        }
      }
    } else if (ref.type === "annotation") {
      // Annotations are anchored to stable identities (abstract grid
      // coords or drive-line splice positions), not draggable in world
      // space. Drag is a no-op.
      return;
    } else if (ref.type === "drive-line" && ref.endpoint) {
      const dl = lot.driveLines[ref.index];
      if (dl.holePin && ref.endpoint === "start") {
        this.moveVertex(
          { type: "boundary-hole", index: dl.holePin.vertexIndex, holeIndex: dl.holePin.holeIndex },
          pos,
        );
        return;
      }
      if (dl.holePin && ref.endpoint === "end") {
        const proj = this.nearestBoundaryProjection(pos, lot);
        dl.end = proj.pos;
        dl.boundaryPin = { edgeIndex: proj.edgeIndex, t: proj.t };
      } else if (ref.endpoint === "start" || ref.endpoint === "end") {
        dl[ref.endpoint] = pos;
      }
    } else if (ref.type === "stall-line" && ref.endpoint) {
      const sm = lot.stallModifiers[ref.index];
      if (!sm) return;
      const i = ref.endpoint === "start" ? 0 : sm.polyline.length - 1;
      sm.polyline[i] = pos;
    } else if (ref.type === "region-vector") {
      const rd = lot.layout?.region_debug;
      const region = rd?.regions[ref.index];
      if (!region) return;
      const regionKey = String(region.id);
      const cx = region.center.x;
      const cy = region.center.y;

      if (ref.endpoint === "start" || ref.endpoint === "end") {
        const dx = pos.x - cx;
        const dy = pos.y - cy;
        const len = Math.sqrt(dx * dx + dy * dy);
        if (len > 1) {
          let angleDeg = Math.atan2(dy, dx) * (180 / Math.PI);
          angleDeg = ((angleDeg % 180) + 180) % 180;
          if (!lot.regionOverrides[regionKey]) {
            lot.regionOverrides[regionKey] = {};
          }
          lot.regionOverrides[regionKey].angle = Math.round(angleDeg);
        }
      } else if (ref.endpoint === "body") {
        const angleRad = region.aisle_angle * (Math.PI / 180);
        const perpX = -Math.sin(angleRad);
        const perpY = Math.cos(angleRad);
        const offset = pos.x * perpX + pos.y * perpY;
        if (!lot.regionOverrides[regionKey]) {
          lot.regionOverrides[regionKey] = {};
        }
        lot.regionOverrides[regionKey].offset = offset;
      }
    } else if (ref.type === "aisle-vector") {
      const vec = lot.aisleVector;
      if (ref.endpoint === "start") {
        vec.start = pos;
      } else {
        vec.end = pos;
      }
      const dx = vec.end.x - vec.start.x;
      const dy = vec.end.y - vec.start.y;
      const len = Math.sqrt(dx * dx + dy * dy);
      if (len > 1) {
        let angleDeg = Math.atan2(dy, dx) * (180 / Math.PI);
        angleDeg = ((angleDeg % 180) + 180) % 180;
        this.state.params.aisle_angle = Math.round(angleDeg);
      }
      // aisle_offset = midpoint perpendicular projection minus the
      // baseline projection captured at lot creation. Using the
      // midpoint instead of a single endpoint means rotating around
      // the midpoint (dragging one end) doesn't change the offset.
      // Subtracting the baseline means the first drag doesn't jump
      // the grid — initial offset stays 0 even though the vector is
      // placed at an arbitrary world position.
      const curProj = midpointPerpProj(vec, this.state.params.aisle_angle);
      this.state.params.aisle_offset = curProj - lot.aisleOffsetBaseline;
    }
    this.generate();
  }

  addHole(vertices: Vec2[]): void {
    const lot = this.state.lot;
    lot.boundary.holes.push(vertices);
    let next = nextVertexId(lot.boundary);
    lot.boundary.hole_ids.push(vertices.map(() => next++));
    this.generate();
  }

  toggleEdgeArc(edgeIndex: number, target: "outer" | "hole", holeIndex?: number): void {
    const lot = this.state.lot;
    const verts = target === "outer" ? lot.boundary.outer : lot.boundary.holes[holeIndex!];
    if (target === "outer") {
      if (!lot.boundary.outer_arcs) lot.boundary.outer_arcs = new Array(verts.length).fill(null);
      const arcs = lot.boundary.outer_arcs;
      arcs[edgeIndex] = arcs[edgeIndex] ? null : defaultArcForEdge();
    } else {
      if (!lot.boundary.hole_arcs) lot.boundary.hole_arcs = [];
      if (!lot.boundary.hole_arcs[holeIndex!]) {
        lot.boundary.hole_arcs[holeIndex!] = new Array(verts.length).fill(null);
      }
      const arcs = lot.boundary.hole_arcs[holeIndex!];
      arcs[edgeIndex] = arcs[edgeIndex] ? null : defaultArcForEdge();
    }
    this.generate();
  }

  insertBoundaryVertex(index: number, pos: Vec2): void {
    const lot = this.state.lot;
    lot.boundary.outer.splice(index + 1, 0, pos);
    // Maintain id array: the new vertex gets a fresh id; existing ids
    // upstream and downstream stay put. The original sketch edge
    // (outer_ids[index] → outer_ids[index+1]) is now split, so any
    // perimeter annotation referencing that pair will go dormant on
    // resolve — by design.
    lot.boundary.outer_ids.splice(index + 1, 0, nextVertexId(lot.boundary));
    // Maintain arc array: split the edge's arc at the insertion point,
    // or insert a straight-edge slot.
    if (lot.boundary.outer_arcs && lot.boundary.outer_arcs.length > 0) {
      const oldArc = lot.boundary.outer_arcs[index];
      if (oldArc) {
        const a = lot.boundary.outer[index];
        const b = lot.boundary.outer[(index + 2) % lot.boundary.outer.length];
        const { t } = projectToArc(a, b, oldArc.bulge, pos);
        const [left, right] = splitArcAt(oldArc.bulge, t);
        lot.boundary.outer_arcs.splice(index, 1, left, right);
      } else {
        lot.boundary.outer_arcs.splice(index + 1, 0, null);
      }
    }
    this.generate();
  }

  deleteBoundaryVertex(index: number): boolean {
    const lot = this.state.lot;
    if (lot.boundary.outer.length <= 3) return false;
    lot.boundary.outer.splice(index, 1);
    lot.boundary.outer_ids.splice(index, 1);
    // Maintain arc array: remove the edge entry and revert the merged
    // edge to straight (the pre-deletion bulge on the prior edge is no
    // longer meaningful — its chord just changed).
    if (lot.boundary.outer_arcs && lot.boundary.outer_arcs.length > 0) {
      const n = lot.boundary.outer_arcs.length;
      const prevEdge = (index - 1 + n) % n;
      lot.boundary.outer_arcs.splice(index, 1);
      if (lot.boundary.outer_arcs[prevEdge % lot.boundary.outer_arcs.length]) {
        lot.boundary.outer_arcs[prevEdge % lot.boundary.outer_arcs.length] = null;
      }
    }
    this.generate();
    return true;
  }

  deleteHoleVertex(holeIndex: number, vertexIndex: number): void {
    const lot = this.state.lot;
    const hole = lot.boundary.holes[holeIndex];
    if (hole.length <= 3) {
      lot.boundary.holes.splice(holeIndex, 1);
      lot.boundary.hole_ids.splice(holeIndex, 1);
      lot.boundary.hole_arcs?.splice(holeIndex, 1);
    } else {
      hole.splice(vertexIndex, 1);
      lot.boundary.hole_ids[holeIndex]?.splice(vertexIndex, 1);
      const hc = lot.boundary.hole_arcs?.[holeIndex];
      if (hc && hc.length > 0) {
        const n = hc.length;
        const prevEdge = (vertexIndex - 1 + n) % n;
        hc.splice(vertexIndex, 1);
        if (hc[prevEdge % hc.length]) {
          hc[prevEdge % hc.length] = null;
        }
      }
    }
    this.state.selectedVertex = null;
    this.generate();
  }

  insertHoleVertex(holeIndex: number, afterIndex: number, pos: Vec2): void {
    const lot = this.state.lot;
    lot.boundary.holes[holeIndex].splice(afterIndex + 1, 0, pos);
    if (!lot.boundary.hole_ids[holeIndex]) lot.boundary.hole_ids[holeIndex] = [];
    lot.boundary.hole_ids[holeIndex].splice(
      afterIndex + 1,
      0,
      nextVertexId(lot.boundary),
    );
    const hc = lot.boundary.hole_arcs?.[holeIndex];
    if (hc && hc.length > 0) {
      const oldArc = hc[afterIndex];
      if (oldArc) {
        const hole = lot.boundary.holes[holeIndex];
        const a = hole[afterIndex];
        const b = hole[(afterIndex + 2) % hole.length];
        const { t } = projectToArc(a, b, oldArc.bulge, pos);
        const [left, right] = splitArcAt(oldArc.bulge, t);
        hc.splice(afterIndex, 1, left, right);
      } else {
        hc.splice(afterIndex + 1, 0, null);
      }
    }
    this.generate();
  }

  commitPendingHole(): void {
    if (this.state.pendingHole.length >= 3) {
      const lot = this.state.lot;
      const verts = [...this.state.pendingHole];
      lot.boundary.holes.push(verts);
      let next = nextVertexId(lot.boundary);
      lot.boundary.hole_ids.push(verts.map(() => next++));
      this.generate();
    }
    this.state.pendingHole = [];
  }

  commitPendingBoundary(): void {
    if (this.state.pendingBoundary.length >= 3) {
      const next: Polygon = {
        outer: [...this.state.pendingBoundary],
        holes: [],
        outer_arcs: [],
        hole_arcs: [],
        outer_ids: [],
        hole_ids: [],
      };
      ensurePolygonIds(next);
      this.replaceBoundary(next);
      this.generate();
    }
    this.state.pendingBoundary = [];
  }

  addDriveLine(start: Vec2, end: Vec2): void {
    this.state.lot.driveLines.push({
      id: this.newDriveLineId(),
      start,
      end,
      partitions: false,
    });
    this.generate();
  }

  /** Append a stall-modifier line (`Suppressed` kind by default). */
  addStallLine(start: Vec2, end: Vec2): void {
    this.state.lot.stallModifiers.push({
      polyline: [start, end],
      kind: "Suppressed",
    });
    this.generate();
  }

  /** Cycle a stall-modifier line's kind:
   *  Suppressed → Ada → Compact → Island → Suppressed. */
  cycleStallLineKind(index: number): void {
    const sm = this.state.lot.stallModifiers[index];
    if (!sm) return;
    const next: { [k: string]: StallKind } = {
      Suppressed: "Ada",
      Ada: "Compact",
      Compact: "Island",
      Island: "Suppressed",
    };
    sm.kind = next[sm.kind] ?? "Suppressed";
    this.generate();
  }

  /** Drop the stall-modifier at `index`. */
  deleteStallLine(index: number): void {
    this.state.lot.stallModifiers.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  deleteAnnotation(index: number): void {
    const lot = this.state.lot;
    lot.annotations.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  toggleDriveLinePartitions(index: number): void {
    const lot = this.state.lot;
    const dl = lot.driveLines[index];
    if (!dl) return;
    dl.partitions = !(dl.partitions ?? false);
    lot.regionOverrides = {};
    this.generate();
  }

  toggleSeparator(holeIndex: number, vertexIndex: number): void {
    const lot = this.state.lot;
    const idx = lot.driveLines.findIndex(
      (dl) => dl.holePin?.holeIndex === holeIndex && dl.holePin?.vertexIndex === vertexIndex,
    );
    if (idx >= 0) {
      lot.driveLines.splice(idx, 1);
    } else {
      const hole = lot.boundary.holes[holeIndex];
      if (!hole || vertexIndex >= hole.length) return;
      const vertex = hole[vertexIndex];
      const proj = this.nearestBoundaryProjection(vertex, lot);
      lot.driveLines.push({
        id: this.newDriveLineId(),
        start: vertex,
        end: proj.pos,
        holePin: { holeIndex, vertexIndex },
        boundaryPin: { edgeIndex: proj.edgeIndex, t: proj.t },
        partitions: true,
      });
    }
    lot.regionOverrides = {};
    this.generate();
  }

  /** Project a point onto the nearest outer boundary edge. */
  private nearestBoundaryProjection(pt: Vec2, lot?: ParkingLot): { pos: Vec2; edgeIndex: number; t: number } {
    const l = lot ?? this.state.lot;
    return computeBoundaryPin(pt, l.boundary.outer, l.boundary.outer_arcs);
  }

  /** Recompute the end position of a drive line from its boundaryPin. */
  private resolveBoundaryPin(dl: DriveLine, lot: ParkingLot): void {
    if (!dl.holePin) return;
    if (!dl.boundaryPin) {
      // Migrate: compute boundaryPin from current end position.
      const proj = this.nearestBoundaryProjection(dl.end, lot);
      dl.boundaryPin = { edgeIndex: proj.edgeIndex, t: proj.t };
    }
    const { edgeIndex, t } = dl.boundaryPin;
    dl.end = evalBoundaryEdge(lot.boundary.outer, edgeIndex, t, lot.boundary.outer_arcs);
  }

  cycleAnnotationDirection(index: number): void {
    const lot = this.state.lot;
    const ann = lot.annotations[index];
    if (!ann) return;
    if (ann.kind !== "Direction") return;

    // Cycle: TwoWayReverse → OneWay → OneWayReverse → inactive
    //        (tombstone) → TwoWayReverse.
    // TwoWay isn't a valid annotation alphabet — the unannotated
    // default already produces a two-way layout, so we tombstone the
    // annotation instead of cycling through TwoWay.
    if (ann._active === false) {
      ann.traffic = "TwoWayReverse";
      ann._active = true;
    } else {
      const next: Record<AisleDirection, { next: AisleDirection; tombstone: boolean }> = {
        TwoWayReverse: { next: "OneWay", tombstone: false },
        OneWay: { next: "OneWayReverse", tombstone: false },
        OneWayReverse: { next: "TwoWayReverse", tombstone: true },
      };
      const step = next[ann.traffic];
      ann.traffic = step.next;
      if (step.tombstone) {
        ann._active = false;
      }
    }
    this.generate();
  }

  cleanupTombstones(): void {
    const lot = this.state.lot;
    const before = lot.annotations.length;
    lot.annotations = lot.annotations.filter((a) => a._active !== false);
    if (lot.annotations.length !== before) this.generate();
  }

  deleteAisleVertexByAnnotation(vertexIndex: number): void {
    const lot = this.state.lot;
    const graph = this.getEffectiveAisleGraph();
    if (!graph) return;
    const v = graph.vertices[vertexIndex];
    if (!v) return;

    // Three addressing schemes in priority order. The semantic question
    // is "what does this vertex belong to" — anchor by the most
    // explicit container the click is on:
    //   1. Perimeter (loop + arc) — if the vertex lies on a boundary
    //      loop. The perimeter is what the user drew; the vertex is at
    //      this position *because* the perimeter passes through here.
    //      Wins over grid even when a grid line also terminates here:
    //      that crossing is incidental to the vertex's perimeter
    //      identity. Deletion stays bound to the boundary the user
    //      sees, regardless of grid-parameter changes.
    //   2. Abstract grid (region + xi + yi) — for purely interior
    //      vertices that snap to an integer lattice point. Stable under
    //      grid-parameter changes within the same region.
    //   3. Drive-line splice (id + t) — for drive × interior crossings
    //      that aren't on the perimeter and aren't lattice-aligned.
    // Defensive: ensure ids are populated and parallel to outer/holes
    // before any wasm bridge that needs them. Cheap when already
    // synced; corrects any drift from unwatched mutation paths.
    ensurePolygonIds(lot.boundary);
    // Project the click against the sketch boundary to see how far
    // off it is — diagnoses "click too far from any sketch edge"
    // failures vs. resolver-side problems.
    const pin = computeBoundaryPin(
      v,
      lot.boundary.outer,
      lot.boundary.outer_arcs,
    );
    const pinDist = Math.hypot(pin.pos.x - v.x, pin.pos.y - v.y);
    const perim_v_count = graph.perim_vertex_count ?? 0;
    const arcsSummary = (lot.boundary.outer_arcs ?? [])
      .map((a, i) => (a ? `${i}:b=${a.bulge.toFixed(2)}` : `${i}:line`))
      .join(",");
    console.log(
      `[delete-vertex] vi=${vertexIndex} ` +
        `isPerim=${vertexIndex < perim_v_count} (perim_n=${perim_v_count}) ` +
        `world=(${v.x.toFixed(2)},${v.y.toFixed(2)}) ` +
        `pin=(${pin.pos.x.toFixed(2)},${pin.pos.y.toFixed(2)}) ` +
        `pinDist=${pinDist.toFixed(3)} pinEdge=${pin.edgeIndex} pinT=${pin.t.toFixed(3)} ` +
        `outerLen=${lot.boundary.outer.length} ` +
        `outerArcsLen=${lot.boundary.outer_arcs?.length} ` +
        `outerIds=[${lot.boundary.outer_ids?.join(",")}] ` +
        `arcs=[${arcsSummary}]`,
    );
    // The user clicked on a graph vertex; for perim vertices that
    // lives on the aisle-edge inset (`inset_d = effective_depth +
    // aisle_width` ≈ 28 ft inside the sketch deed line), so the
    // projection onto the sketch needs that distance plus slack for
    // chord deflection on discretized arcs (~`arc_discretize_tolerance`).
    // Without it, perim-vertex deletes silently fall through to
    // grid/drive-line and error out.
    const insetD = effectiveDepth(this.state.params) + this.state.params.aisle_width;
    const perimTol = insetD + (this.state.params.arc_discretize_tolerance ?? 0.5) + 1.0;
    const perim = worldToPerimeterPos(v, lot.boundary, perimTol);
    console.log("[delete-vertex] worldToPerimeterPos result:", perim);
    if (perim) {
      showToast(`delete: anchored on perimeter (${perim.start}→${perim.end} t=${perim.t.toFixed(3)})`);
      lot.annotations.push({
        kind: "DeleteVertex",
        target: {
          on: "Perimeter",
          loop: perim.loop,
          start: perim.start,
          end: perim.end,
          t: perim.t,
        },
      });
    } else {
      const abs = lot.layout?.region_debug
        ? worldToAbstractVertex(v, this.state.params, lot.layout.region_debug)
        : null;
      console.log("[delete-vertex] worldToAbstractVertex result:", abs);
      if (abs) {
        const stop: GridStop = { at: "Lattice", other: abs.yi };
        lot.annotations.push({
          kind: "DeleteVertex",
          target: {
            on: "Grid",
            region: abs.region,
            axis: "X",
            coord: abs.xi,
            range: [stop, stop],
          },
        });
      } else {
        const sp = worldToSpliceVertex(v, lot.driveLines);
        console.log("[delete-vertex] worldToSpliceVertex result:", sp);
        if (sp) {
          lot.annotations.push({
            kind: "DeleteVertex",
            target: { on: "DriveLine", id: sp.drive_line_id, t: sp.t },
          });
        } else {
          console.log("[delete-vertex] FALLTHROUGH — no annotation created");
          showToast("can't anchor delete: vertex isn't on perimeter, grid, or drive-line");
          return; // no stable anchor for this vertex
        }
      }
    }
    this.state.selectedVertex = null;
    this.generate();
  }

  /**
   * Resolve an aisle-edge selection to a `Target::Grid` that covers
   * the selection. Delegates to `resolve::resolve_grid_target` in the
   * engine — which handles both the on-grid seed + chain-widening
   * path and the off-grid seed + chain-junction fallback.
   */
  private resolveGridTarget(
    sel: EdgeRef,
    _graph: DriveAisleGraph,
    _seed: { start: number; end: number },
    lot: ParkingLot,
  ): Target | null {
    const regionDebug = lot.layout?.region_debug;
    if (!regionDebug) return null;
    const graph = this.getEffectiveAisleGraph();
    if (!graph) return null;
    return resolve_grid_target_js(
      sel.index,
      new Uint32Array(sel.chain),
      sel.mode === "chain",
      graph,
      this.state.params,
      regionDebug,
    ) as Target | null;
  }

  deleteSelectedEdge(): void {
    const sel = this.state.selectedEdge;
    if (!sel) return;
    const lot = this.state.lot;
    const graph = this.getEffectiveAisleGraph();
    if (!graph) return;
    const seedEdge = graph.edges[sel.index];
    if (!seedEdge) return;

    const target = this.resolveGridTarget(sel, graph, seedEdge, lot);
    if (target) {
      lot.annotations.push({ kind: "DeleteEdge", target });
      this.state.selectedEdge = null;
      this.generate();
      return;
    }

    // Splice path: chain lies on a single drive line and isn't
    // grid-addressable. The drive-line edge target is a single point;
    // we use the midpoint of the (chain-extended) t-range so the post-
    // pass picks the sub-edge the user clicked.
    const s = graph.vertices[seedEdge.start];
    const e = graph.vertices[seedEdge.end];
    const sa = worldToSpliceVertex(s, lot.driveLines);
    const sb = worldToSpliceVertex(e, lot.driveLines);
    if (!(sa && sb && sa.drive_line_id === sb.drive_line_id)) {
      showToast("delete: no stable anchor for this sub-edge");
      return;
    }
    let ta = sa.t;
    let tb = sb.t;
    if (sel.mode === "chain" && sel.chain.length > 1) {
      const ts: number[] = [];
      for (const ei of sel.chain) {
        const ce = graph.edges[ei];
        if (!ce) continue;
        const p1 = worldToSpliceVertex(graph.vertices[ce.start], lot.driveLines);
        const p2 = worldToSpliceVertex(graph.vertices[ce.end], lot.driveLines);
        if (p1 && p1.drive_line_id === sa.drive_line_id) ts.push(p1.t);
        if (p2 && p2.drive_line_id === sa.drive_line_id) ts.push(p2.t);
      }
      if (ts.length >= 2) {
        ta = Math.min(...ts);
        tb = Math.max(...ts);
      }
    }
    lot.annotations.push({
      kind: "DeleteEdge",
      target: { on: "DriveLine", id: sa.drive_line_id, t: (ta + tb) / 2 },
    });
    this.state.selectedEdge = null;
    this.generate();
  }

  deleteDriveLine(index: number): void {
    const lot = this.state.lot;
    lot.driveLines.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  cycleEdgeDirection(sel: EdgeRef): void {
    const lot = this.state.lot;
    const graph = this.getEffectiveAisleGraph();
    if (!graph) return;
    const seed = graph.edges[sel.index];
    if (!seed) return;

    const gridTarget = this.resolveGridTarget(sel, graph, seed, lot);
    if (gridTarget) {
      const existing = lot.annotations.findIndex(
        (ann) => ann.kind === "Direction" && targets_equal_js(ann.target, gridTarget),
      );
      if (existing < 0) {
        const annIdx = lot.annotations.length;
        lot.annotations.push({
          kind: "Direction",
          target: gridTarget,
          traffic: "TwoWayReverse",
          _active: true,
        });
        this.state.selectedVertex = { type: "annotation", index: annIdx };
        this.generate();
      } else {
        this.cycleAnnotationDirection(existing);
        this.state.selectedVertex = { type: "annotation", index: existing };
      }
      return;
    }

    // Splice path: both endpoints on the same drive line.
    const s = graph.vertices[seed.start];
    const e = graph.vertices[seed.end];
    const sa = worldToSpliceVertex(s, lot.driveLines);
    const sb = worldToSpliceVertex(e, lot.driveLines);
    if (!(sa && sb && sa.drive_line_id === sb.drive_line_id)) {
      showToast("f: no stable anchor for this sub-edge");
      return;
    }
    const dlTarget: Target = {
      on: "DriveLine",
      id: sa.drive_line_id,
      t: (sa.t + sb.t) / 2,
    };
    const sliceTol = 1e-3;
    const existing = lot.annotations.findIndex((ann) => {
      if (ann.kind !== "Direction") return false;
      if (ann.target.on !== "DriveLine") return false;
      if (ann.target.id !== sa.drive_line_id) return false;
      return Math.abs(ann.target.t - dlTarget.t) < sliceTol;
    });
    if (existing < 0) {
      const annIdx = lot.annotations.length;
      lot.annotations.push({
        kind: "Direction",
        target: dlTarget,
        traffic: "TwoWayReverse",
        _active: true,
      });
      this.state.selectedVertex = { type: "annotation", index: annIdx };
      this.generate();
    } else {
      this.cycleAnnotationDirection(existing);
      this.state.selectedVertex = { type: "annotation", index: existing };
    }
  }
}

function aisleVectorFromAngle(angleDeg: number, _offset: number, center: Vec2): { start: Vec2; end: Vec2 } {
  const angleRad = angleDeg * (Math.PI / 180);
  const dirX = Math.cos(angleRad);
  const dirY = Math.sin(angleRad);
  const halfLen = 30;
  return {
    start: { x: center.x - dirX * halfLen, y: center.y - dirY * halfLen },
    end: { x: center.x + dirX * halfLen, y: center.y + dirY * halfLen },
  };
}

/**
 * Perpendicular projection of an aisle vector's midpoint onto the
 * perp_dir implied by `aisleAngleDeg`. Used to maintain the
 * aisleOffsetBaseline invariant when the lot is created and when the
 * vector is dragged.
 */
function midpointPerpProj(
  vec: { start: Vec2; end: Vec2 },
  aisleAngleDeg: number,
): number {
  const angleRad = aisleAngleDeg * (Math.PI / 180);
  const perpX = -Math.sin(angleRad);
  const perpY = Math.cos(angleRad);
  const mx = (vec.start.x + vec.end.x) / 2;
  const my = (vec.start.y + vec.end.y) / 2;
  return mx * perpX + my * perpY;
}
