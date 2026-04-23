import {
  Vec2,
  EdgeArc,
  Polygon,
  DriveLine,
  DriveAisleGraph,
  ParkingParams,
  ParkingLayout,
  ParkingLot,
  GenerateInput,
  DebugToggles,
  Annotation,
  GridStop,
  Target,
  TrafficDirection,
  arcApex,
  bulgeFromApex,
  computeBoundaryPin,
  evalBoundaryEdge,
  annotationWorldPos,
  projectToArc,
  worldToPerimeterPos,
  worldToAbstractVertex,
  worldToSpliceVertex,
} from "./types";
import { SnapGuide, SnapState, emptySnapState } from "./snap";
import { showToast } from "./toast";
import {
  resolve_grid_target_js,
  targets_equal_js,
} from "./wasm/parking_lot_engine";

// ---------------------------------------------------------------------------
// Arc helpers for boundary-edge editing
// ---------------------------------------------------------------------------

/**
 * Split an arc at parameter `t` into two sub-arcs that together cover
 * the original. Uses tan-half-angle identities: sweep = −4·atan(b), so a
 * fraction-t sub-arc has bulge = tan(t · atan(b)).
 */
function splitArcAt(bulge: number, t: number): [EdgeArc | null, EdgeArc | null] {
  const half = Math.atan(bulge);
  const b1 = Math.tan(t * half);
  const b2 = Math.tan((1 - t) * half);
  return [
    Math.abs(b1) < 1e-6 ? null : { bulge: b1 },
    Math.abs(b2) < 1e-6 ? null : { bulge: b2 },
  ];
}

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
  type: "boundary-outer" | "boundary-hole" | "aisle" | "drive-line" | "annotation" | "aisle-vector" | "region-vector" | "edge-midpoint";
  index: number;
  holeIndex?: number;
  endpoint?: "start" | "end" | "body";
  lotId?: string;
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
  | "add-boundary";

export interface LayerVisibility {
  stalls: boolean;
  aisles: boolean;
  vertices: boolean;
  driveLines: boolean;
  spines: boolean;
  faces: boolean;
  faceColors: boolean;
  miterFills: boolean;
  skeletonDebug: boolean;
  islands: boolean;
  extensionStalls: boolean;
  regions: boolean;
  paintLines: boolean;
}

export interface AppState {
  lots: ParkingLot[];
  activeLotId: string;
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
  layers: LayerVisibility;
  snapGuides: SnapGuide[];
  snapState: SnapState;
}

export type GenerateFn = (input: string) => string;

export class App {
  state: AppState;
  private generateFn: GenerateFn;
  private onUpdate: () => void;
  private nextLotId = 1;
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
      id: "lot-0",
      boundary: {
        outer: [
          { x: 76.30, y: 0 },
          { x: 692.14, y: 0 },
          { x: 782.80, y: 654.85 },
          { x: 168.78, y: 654.85 },
        ],
        holes: [
          [
            { x: 394.53, y: 251.80 },
            { x: 611.14, y: 251.80 },
            { x: 613.91, y: 512.33 },
            { x: 394.53, y: 512.33 },
          ],
        ],
        outer_arcs: [],
        hole_arcs: [],
      },
      aisleGraph: null,
      annotations: [
        {
          kind: "DeleteEdge",
          target: {
            on: "Grid",
            region: 38732162085172,
            axis: "X",
            coord: -2,
            range: [{ at: "Lattice", other: 0 }, { at: "Lattice", other: 1 }],
          },
        },
        {
          kind: "DeleteEdge",
          target: {
            on: "Grid",
            region: 38732162085172,
            axis: "X",
            coord: -2,
            range: [{ at: "Lattice", other: 1 }, { at: "Lattice", other: 2 }],
          },
        },
      ],
      aisleVector: aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
      aisleOffsetBaseline: midpointPerpProj(
        aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
        90,
      ),
      driveLines: [
        { id: 1, start: { x: 930.76, y: 381.06 }, end: { x: -42.60, y: 512.33 }, partitions: false },
        { id: 2, start: { x: 198.60, y: -53.66 }, end: { x: 475.79, y: 349.25 }, partitions: true },
        { id: 3, start: { x: 446.56, y: 335.61 }, end: { x: 930.76, y: 135.52 }, partitions: true },
      ],
      regionOverrides: {
        "221108085334326": { offset: 154.58 },
      },
      layout: null,
    };

    this.state = {
      lots: [defaultLot],
      activeLotId: "lot-0",
      params: {
        stall_width: 9,
        stall_depth: 18,
        aisle_width: 12,
        stall_angle_deg: 45,
        aisle_angle_deg: 90,
        aisle_offset: 0,
        site_offset: 0,
        stalls_per_face: 29,
        use_regions: true,
        island_stall_interval: 12,
      },
      debug: {
        miter_fills: true,
        boundary_only_miters: true,
        spike_removal: true,
        contour_simplification: true,
        hole_filtering: false,
        edge_provenance: true,
        face_simplification: false,
        edge_classification: true,
        spine_clipping: true,
        spine_dedup: true,
        spine_merging: true,
        short_spine_filter: false,
        spine_extensions: true,
        stall_centering: true,
        stall_face_clipping: true,
        boundary_clipping: false,
        conflict_removal: true,
        short_segment_filter: true,
        skeleton_debug: false,
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
      snapGuides: [],
      snapState: emptySnapState(),
      layers: {
        stalls: false,
        aisles: false,
        vertices: false,
        driveLines: false,
        spines: false,
        faces: false,
        faceColors: false,
        miterFills: false,
        skeletonDebug: false,
        islands: true,
        extensionStalls: false,
        regions: false,
        paintLines: true,
      },
    };
  }

  // ── Lot helpers ──────────────────────────────────────────────────────

  activeLot(): ParkingLot {
    return this.state.lots.find((l) => l.id === this.state.activeLotId)!;
  }

  lotById(id: string): ParkingLot | undefined {
    return this.state.lots.find((l) => l.id === id);
  }

  /** Resolve the lot a VertexRef belongs to. Falls back to active lot. */
  lotForRef(ref: VertexRef): ParkingLot {
    if (ref.lotId) {
      return this.lotById(ref.lotId) ?? this.activeLot();
    }
    return this.activeLot();
  }

  private newLotId(): string {
    return `lot-${this.nextLotId++}`;
  }

  addLot(boundary: Polygon): ParkingLot {
    const id = this.newLotId();
    // Compute centroid for aisle vector placement.
    const cx = boundary.outer.reduce((s, v) => s + v.x, 0) / boundary.outer.length;
    const cy = boundary.outer.reduce((s, v) => s + v.y, 0) / boundary.outer.length;
    const initialVector = aisleVectorFromAngle(90, 0, { x: cx, y: cy });
    const lot: ParkingLot = {
      id,
      boundary,
      aisleGraph: null,
      driveLines: [],
      annotations: [],
      aisleVector: initialVector,
      aisleOffsetBaseline: midpointPerpProj(initialVector, 90),
      regionOverrides: {},
      layout: null,
    };
    this.state.lots.push(lot);
    this.state.activeLotId = id;
    return lot;
  }

  deleteLot(id: string): void {
    this.state.lots = this.state.lots.filter((l) => l.id !== id);
    if (this.state.activeLotId === id) {
      this.state.activeLotId = this.state.lots[0]?.id ?? "";
    }
  }

  generate(): void {
    for (const lot of this.state.lots) {
      this.generateLot(lot);
    }
    this.onUpdate();
  }

  generateLot(lot: ParkingLot): void {
    const input: GenerateInput = {
      boundary: lot.boundary,
      aisle_graph: lot.aisleGraph,
      drive_lines: lot.driveLines,
      annotations: lot.annotations.filter((a) => a._active !== false),
      params: this.state.params,
      debug: this.state.debug,
      regionOverrides: Object.entries(lot.regionOverrides).map(([k, v]) => ({
        region_id: Number(k),
        aisle_angle_deg: v.angle,
        aisle_offset: v.offset,
      })),
      stall_modifiers: [],
    };
    const inputJson = JSON.stringify(input);
    if (lot.id === this.state.activeLotId) {
      (window as any).__parkingInput = inputJson;
    }
    try {
      const resultJson = this.generateFn(inputJson);
      lot.layout = JSON.parse(resultJson);
    } catch (e) {
      console.error(`Generate failed for lot ${lot.id}:`, e);
    }
  }

  setParam(key: keyof ParkingParams, value: number): void {
    (this.state.params as any)[key] = value;
    for (const lot of this.state.lots) {
      lot.aisleGraph = null;
    }
    if (key === "aisle_angle_deg" || key === "aisle_offset") {
      this.syncAisleVector(this.activeLot());
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
    const angleDeg = this.state.params.aisle_angle_deg;
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

  // Returns the effective aisle graph for a lot (default: active lot).
  getEffectiveAisleGraph(lot?: ParkingLot): DriveAisleGraph | null {
    const l = lot ?? this.activeLot();
    return l.aisleGraph ?? l.layout?.resolved_graph ?? null;
  }

  // Promote the auto-generated (resolved) graph into the editable
  // aisleGraph so it can be dragged/modified.
  materializeAisleGraph(lot?: ParkingLot): void {
    const l = lot ?? this.activeLot();
    if (l.aisleGraph) return;
    const resolved = l.layout?.resolved_graph;
    if (!resolved) return;
    l.aisleGraph = {
      vertices: resolved.vertices.map((v) => ({ ...v })),
      edges: resolved.edges.map((e) => ({ ...e })),
      perim_vertex_count: resolved.perim_vertex_count,
    };
  }

  getAllVertices(): { ref: VertexRef; pos: Vec2 }[] {
    const result: { ref: VertexRef; pos: Vec2 }[] = [];

    for (const lot of this.state.lots) {
      const lid = lot.id;

      // Region vector endpoints — highest hit-test priority.
      const rd = lot.layout?.region_debug;
      if (rd) {
        const halfLen = 30;
        for (let i = 0; i < rd.regions.length; i++) {
          const region = rd.regions[i];
          const angleRad = region.aisle_angle_deg * (Math.PI / 180);
          const dirX = Math.cos(angleRad);
          const dirY = Math.sin(angleRad);
          const cx = region.center.x;
          const cy = region.center.y;
          result.push({
            ref: { type: "region-vector", index: i, endpoint: "start", lotId: lid },
            pos: { x: cx - dirX * halfLen, y: cy - dirY * halfLen },
          });
          result.push({
            ref: { type: "region-vector", index: i, endpoint: "end", lotId: lid },
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
          result.push({ ref: { type: "annotation", index: i, lotId: lid }, pos });
        }
      });
      // Drive-line vertices.
      lot.driveLines.forEach((dl, i) => {
        result.push({
          ref: { type: "drive-line", index: i, endpoint: "start", lotId: lid },
          pos: dl.start,
        });
        result.push({
          ref: { type: "drive-line", index: i, endpoint: "end", lotId: lid },
          pos: dl.end,
        });
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
        result.push({
          ref: { type: "edge-midpoint", index: i, cpTarget: "outer", lotId: lid },
          pos,
        });
      }
      // Boundary vertices.
      lot.boundary.outer.forEach((v, i) => {
        result.push({ ref: { type: "boundary-outer", index: i, lotId: lid }, pos: v });
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
            ref: { type: "edge-midpoint", index: i, cpTarget: "hole", holeIndex: hi, lotId: lid },
            pos,
          });
        }
        // Hole vertices.
        hole.forEach((v, vi) => {
          result.push({
            ref: { type: "boundary-hole", index: vi, holeIndex: hi, lotId: lid },
            pos: v,
          });
        });
      });
      const graph = this.getEffectiveAisleGraph(lot);
      if (graph) {
        graph.vertices.forEach((v, i) => {
          result.push({ ref: { type: "aisle", index: i, lotId: lid }, pos: v });
        });
      }
    }
    return result;
  }

  moveVertex(ref: VertexRef, pos: Vec2): void {
    const lot = this.lotForRef(ref);
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
      lot.aisleGraph = null;
    } else if (ref.type === "boundary-outer") {
      // Arc bulges are stored as a signed scalar independent of
      // endpoint positions, so moving an endpoint automatically
      // reshapes adjacent arcs — no bookkeeping needed here.
      lot.boundary.outer[ref.index] = pos;
      for (const dl of lot.driveLines) {
        this.resolveBoundaryPin(dl, lot);
      }
      lot.aisleGraph = null;
    } else if (ref.type === "boundary-hole" && ref.holeIndex !== undefined) {
      lot.boundary.holes[ref.holeIndex][ref.index] = pos;
      for (const dl of lot.driveLines) {
        if (dl.holePin?.holeIndex === ref.holeIndex && dl.holePin?.vertexIndex === ref.index) {
          dl.start = pos;
        }
      }
      lot.aisleGraph = null;
    } else if (ref.type === "annotation") {
      // Annotations are anchored to stable identities (abstract grid
      // coords or drive-line splice positions), not draggable in world
      // space. Drag is a no-op.
      return;
    } else if (ref.type === "drive-line" && ref.endpoint) {
      const dl = lot.driveLines[ref.index];
      if (dl.holePin && ref.endpoint === "start") {
        this.moveVertex(
          { type: "boundary-hole", index: dl.holePin.vertexIndex, holeIndex: dl.holePin.holeIndex, lotId: lot.id },
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
      lot.aisleGraph = null;
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
        const angleRad = region.aisle_angle_deg * (Math.PI / 180);
        const perpX = -Math.sin(angleRad);
        const perpY = Math.cos(angleRad);
        const offset = pos.x * perpX + pos.y * perpY;
        if (!lot.regionOverrides[regionKey]) {
          lot.regionOverrides[regionKey] = {};
        }
        lot.regionOverrides[regionKey].offset = offset;
      }
      lot.aisleGraph = null;
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
        this.state.params.aisle_angle_deg = Math.round(angleDeg);
      }
      // aisle_offset = midpoint perpendicular projection minus the
      // baseline projection captured at lot creation. Using the
      // midpoint instead of a single endpoint means rotating around
      // the midpoint (dragging one end) doesn't change the offset.
      // Subtracting the baseline means the first drag doesn't jump
      // the grid — initial offset stays 0 even though the vector is
      // placed at an arbitrary world position.
      const curProj = midpointPerpProj(vec, this.state.params.aisle_angle_deg);
      this.state.params.aisle_offset = curProj - lot.aisleOffsetBaseline;
      lot.aisleGraph = null;
    }
    this.generate();
  }

  addHole(vertices: Vec2[]): void {
    const lot = this.activeLot();
    lot.boundary.holes.push(vertices);
    lot.aisleGraph = null;
    this.generate();
  }

  toggleEdgeArc(edgeIndex: number, target: "outer" | "hole", holeIndex?: number): void {
    const lot = this.activeLot();
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
    lot.aisleGraph = null;
    this.generate();
  }

  insertBoundaryVertex(index: number, pos: Vec2, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.boundary.outer.splice(index + 1, 0, pos);
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
    lot.aisleGraph = null;
    this.generate();
  }

  deleteBoundaryVertex(index: number, lotId?: string): boolean {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    if (lot.boundary.outer.length <= 3) return false;
    lot.boundary.outer.splice(index, 1);
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
    lot.aisleGraph = null;
    this.generate();
    return true;
  }

  deleteHoleVertex(holeIndex: number, vertexIndex: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    const hole = lot.boundary.holes[holeIndex];
    if (hole.length <= 3) {
      lot.boundary.holes.splice(holeIndex, 1);
      lot.boundary.hole_arcs?.splice(holeIndex, 1);
    } else {
      hole.splice(vertexIndex, 1);
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
    lot.aisleGraph = null;
    this.generate();
  }

  insertHoleVertex(holeIndex: number, afterIndex: number, pos: Vec2, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.boundary.holes[holeIndex].splice(afterIndex + 1, 0, pos);
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
    lot.aisleGraph = null;
    this.generate();
  }

  commitPendingHole(): void {
    if (this.state.pendingHole.length >= 3) {
      const lot = this.activeLot();
      lot.boundary.holes.push([...this.state.pendingHole]);
      lot.aisleGraph = null;
      this.generate();
    }
    this.state.pendingHole = [];
  }

  commitPendingBoundary(): void {
    if (this.state.pendingBoundary.length >= 3) {
      this.addLot({
        outer: [...this.state.pendingBoundary],
        holes: [],
        outer_arcs: [],
        hole_arcs: [],
      });
      this.generate();
    }
    this.state.pendingBoundary = [];
  }

  addDriveLine(start: Vec2, end: Vec2): void {
    this.activeLot().driveLines.push({
      id: this.newDriveLineId(),
      start,
      end,
      partitions: false,
    });
    this.generate();
  }

  deleteAnnotation(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.annotations.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  toggleDriveLinePartitions(index: number, lotId?: string): void {
    const lot = (lotId && this.state.lots.find((l) => l.id === lotId)) || this.activeLot();
    const dl = lot.driveLines[index];
    if (!dl) return;
    dl.partitions = !(dl.partitions ?? false);
    lot.regionOverrides = {};
    lot.aisleGraph = null;
    this.generate();
  }

  toggleSeparator(holeIndex: number, vertexIndex: number): void {
    const lot = this.activeLot();
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
    lot.aisleGraph = null;
    this.generate();
  }

  /** Project a point onto the nearest outer boundary edge. */
  private nearestBoundaryProjection(pt: Vec2, lot?: ParkingLot): { pos: Vec2; edgeIndex: number; t: number } {
    const l = lot ?? this.activeLot();
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

  cycleAnnotationDirection(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    const ann = lot.annotations[index];
    if (!ann) return;
    if (ann.kind !== "Direction") return;

    // Cycle: TwoWayOriented → TwoWayOrientedReverse → OneWay →
    //        OneWayReverse → inactive (tombstone) → TwoWayOriented.
    if (ann._active === false) {
      ann.traffic = "TwoWayOriented";
      ann._active = true;
    } else {
      const next: Record<TrafficDirection, { next: TrafficDirection; tombstone: boolean }> = {
        TwoWayOriented: { next: "TwoWayOrientedReverse", tombstone: false },
        TwoWayOrientedReverse: { next: "OneWay", tombstone: false },
        OneWay: { next: "OneWayReverse", tombstone: false },
        OneWayReverse: { next: "TwoWayOriented", tombstone: true },
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
    let changed = false;
    for (const lot of this.state.lots) {
      const before = lot.annotations.length;
      lot.annotations = lot.annotations.filter((a) => a._active !== false);
      if (lot.annotations.length !== before) changed = true;
    }
    if (changed) this.generate();
  }

  deleteAisleVertexByAnnotation(vertexIndex: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    const graph = this.getEffectiveAisleGraph(lot);
    if (!graph) return;
    const v = graph.vertices[vertexIndex];
    if (!v) return;

    // Three addressing schemes in priority order. Try the MOST stable
    // first so that parameter changes (aisle angle, stall depth) don't
    // silently re-target a different vertex:
    //   1. Abstract grid (region + xi + yi) — for vertices that snap to
    //      an integer lattice point. Stable under boundary moves AND
    //      grid-parameter changes.
    //   2. Drive-line splice (id + t) — for drive × anything crossings.
    //      Stable under grid changes (drive line doesn't move with
    //      grid), drifts only when the drive line itself is edited.
    //   3. Perimeter (loop + arc) — fallback for vertices that sit on
    //      a boundary loop but aren't at integer lattice (e.g. grid ×
    //      perimeter crossings at non-lattice positions). Stable under
    //      boundary-geometry edits but drifts when the grid rotates and
    //      the crossing sweeps along the loop — acceptable for the
    //      cases this catches, which the other two don't.
    const abs = lot.layout?.region_debug
      ? worldToAbstractVertex(v, this.state.params, lot.layout.region_debug)
      : null;
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
      if (sp) {
        lot.annotations.push({
          kind: "DeleteVertex",
          target: { on: "DriveLine", id: sp.drive_line_id, t: sp.t },
        });
      } else {
        const perim = worldToPerimeterPos(v, lot.boundary);
        if (perim) {
          lot.annotations.push({
            kind: "DeleteVertex",
            target: { on: "Perimeter", loop: perim.loop, arc: perim.arc },
          });
        } else {
          return; // no stable anchor for this vertex
        }
      }
    }
    this.state.selectedVertex = null;
    lot.aisleGraph = null;
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
    const graph = this.getEffectiveAisleGraph(lot);
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
    const lot = this.activeLot();
    const graph = this.getEffectiveAisleGraph(lot);
    if (!graph) return;
    const seedEdge = graph.edges[sel.index];
    if (!seedEdge) return;

    const target = this.resolveGridTarget(sel, graph, seedEdge, lot);
    if (target) {
      lot.annotations.push({ kind: "DeleteEdge", target });
      this.state.selectedEdge = null;
      lot.aisleGraph = null;
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
    lot.aisleGraph = null;
    this.generate();
  }

  deleteDriveLine(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.driveLines.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  cycleEdgeDirection(sel: EdgeRef): void {
    const lot = this.activeLot();
    const graph = this.getEffectiveAisleGraph(lot);
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
          traffic: "TwoWayOriented",
          _active: true,
        });
        this.state.selectedVertex = { type: "annotation", index: annIdx, lotId: lot.id };
        this.generate();
      } else {
        this.cycleAnnotationDirection(existing, lot.id);
        this.state.selectedVertex = { type: "annotation", index: existing, lotId: lot.id };
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
        traffic: "TwoWayOriented",
        _active: true,
      });
      this.state.selectedVertex = { type: "annotation", index: annIdx, lotId: lot.id };
      this.generate();
    } else {
      this.cycleAnnotationDirection(existing, lot.id);
      this.state.selectedVertex = { type: "annotation", index: existing, lotId: lot.id };
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
