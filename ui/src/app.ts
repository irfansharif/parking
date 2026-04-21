import {
  Vec2,
  EdgeCurve,
  Polygon,
  DriveLine,
  DriveAisleGraph,
  ParkingParams,
  ParkingLayout,
  ParkingLot,
  GenerateInput,
  DebugToggles,
  Annotation,
  computeBoundaryPin,
  evalBoundaryEdge,
  isAbstractAnnotation,
  abstractAnnotationWorldPos,
  chainExtentsInRegion,
  worldToAbstractVertex,
  worldToSpliceVertex,
} from "./types";
import { SnapGuide, SnapState, emptySnapState } from "./snap";
import { findCollinearChain } from "./interaction";

// ---------------------------------------------------------------------------
// Bezier helpers for curve editing
// ---------------------------------------------------------------------------

function evalCubic(p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2, t: number): Vec2 {
  const s = 1 - t;
  return {
    x: s * s * s * p0.x + 3 * s * s * t * cp1.x + 3 * s * t * t * cp2.x + t * t * t * p3.x,
    y: s * s * s * p0.y + 3 * s * s * t * cp1.y + 3 * s * t * t * cp2.y + t * t * t * p3.y,
  };
}

function splitCubicAt(
  p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2, t: number,
): [EdgeCurve | null, EdgeCurve | null] {
  const lerp = (a: Vec2, b: Vec2, t: number): Vec2 => ({
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
  });
  const m01 = lerp(p0, cp1, t);
  const m12 = lerp(cp1, cp2, t);
  const m23 = lerp(cp2, p3, t);
  const m012 = lerp(m01, m12, t);
  const m123 = lerp(m12, m23, t);
  return [
    { cp1: m01, cp2: m012 },
    { cp1: m123, cp2: m23 },
  ];
}

function nearestTOnCubic(p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2, pt: Vec2): number {
  // Sample the curve at N points, find closest, then refine with bisection.
  const N = 20;
  let bestT = 0;
  let bestDist = Infinity;
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    const p = evalCubic(p0, cp1, cp2, p3, t);
    const d = (p.x - pt.x) ** 2 + (p.y - pt.y) ** 2;
    if (d < bestDist) { bestDist = d; bestT = t; }
  }
  // Refine with bisection.
  let lo = Math.max(0, bestT - 1 / N);
  let hi = Math.min(1, bestT + 1 / N);
  for (let i = 0; i < 16; i++) {
    const t1 = (2 * lo + hi) / 3;
    const t2 = (lo + 2 * hi) / 3;
    const p1 = evalCubic(p0, cp1, cp2, p3, t1);
    const p2 = evalCubic(p0, cp1, cp2, p3, t2);
    const d1 = (p1.x - pt.x) ** 2 + (p1.y - pt.y) ** 2;
    const d2 = (p2.x - pt.x) ** 2 + (p2.y - pt.y) ** 2;
    if (d1 < d2) hi = t2; else lo = t1;
  }
  return (lo + hi) / 2;
}

function defaultCurveForEdge(verts: Vec2[], edgeIndex: number): EdgeCurve {
  const a = verts[edgeIndex];
  const b = verts[(edgeIndex + 1) % verts.length];
  const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
  const dx = b.x - a.x, dy = b.y - a.y;
  // Perpendicular offset scaled to 30% of edge length — gives a visible
  // outward bulge. Both control points share the same offset so the
  // default is a symmetric C-shape, not an S.
  const nx = -dy * 0.3, ny = dx * 0.3;
  return {
    cp1: { x: mx + nx - dx * 0.15, y: my + ny - dy * 0.15 },
    cp2: { x: mx + nx + dx * 0.15, y: my + ny + dy * 0.15 },
  };
}

export interface Camera {
  offsetX: number;
  offsetY: number;
  zoom: number;
}

export interface VertexRef {
  type: "boundary-outer" | "boundary-hole" | "aisle" | "drive-line" | "annotation" | "aisle-vector" | "region-vector" | "curve-cp" | "edge-midpoint";
  index: number;
  holeIndex?: number;
  endpoint?: "start" | "end" | "body";
  lotId?: string;
  cpIndex?: 0 | 1;
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
      },
      aisleGraph: null,
      annotations: [
        { kind: "AbstractDeleteEdge", region: 38732162085172, xa: -2, ya: 0, xb: -2, yb: 1 },
        { kind: "AbstractDeleteEdge", region: 38732162085172, xa: -2, ya: 1, xb: -2, yb: 2 },
      ],
      aisleVector: aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
      aisleOffsetBaseline: midpointPerpProj(
        aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
        90,
      ),
      driveLines: [
        { id: 1, start: { x: 930.76, y: 381.06 }, end: { x: -42.60, y: 512.33 } },
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
      // Annotation anchors. Abstract annotations are integer grid
      // handles with no world-space anchor; the renderer resolves them
      // via the engine. Splice annotations live on a drive line, so we
      // compute their world position from (drive_line_id, t) here.
      lot.annotations.forEach((ann, i) => {
        if (isAbstractAnnotation(ann)) {
          // Resolve abstract grid coords to a world anchor via the
          // host region's AbstractFrame. Dormant annotations (region
          // gone) return null and are skipped — nothing to click.
          const pos = abstractAnnotationWorldPos(
            ann,
            lot.layout?.region_debug,
            this.state.params,
          );
          if (pos) {
            result.push({ ref: { type: "annotation", index: i, lotId: lid }, pos });
          }
          return;
        }
        // Splice annotation: world pos = midpoint of (ta, tb) along the
        // line, or just `t` for the vertex variant.
        const dl = lot.driveLines.find((d) => d.id === ann.drive_line_id);
        if (!dl) return;
        const t = ann.kind === "SpliceDeleteVertex"
          ? ann.t
          : (ann.ta + ann.tb) / 2;
        const pos = {
          x: dl.start.x + (dl.end.x - dl.start.x) * t,
          y: dl.start.y + (dl.end.y - dl.start.y) * t,
        };
        result.push({ ref: { type: "annotation", index: i, lotId: lid }, pos });
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
      // Dragging creates/adjusts curves.
      for (let i = 0; i < lot.boundary.outer.length; i++) {
        const a = lot.boundary.outer[i];
        const b = lot.boundary.outer[(i + 1) % lot.boundary.outer.length];
        const c = lot.boundary.outer_curves?.[i];
        const pos = c
          ? evalCubic(a, c.cp1, c.cp2, b, 0.5)
          : { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
        result.push({
          ref: { type: "edge-midpoint", index: i, cpTarget: "outer", lotId: lid },
          pos,
        });
      }
      // Curve control points (ghost handles, only for curved edges).
      lot.boundary.outer_curves?.forEach((c, i) => {
        if (!c) return;
        result.push({
          ref: { type: "curve-cp", index: i, cpIndex: 0, cpTarget: "outer", lotId: lid },
          pos: c.cp1,
        });
        result.push({
          ref: { type: "curve-cp", index: i, cpIndex: 1, cpTarget: "outer", lotId: lid },
          pos: c.cp2,
        });
      });
      // Boundary vertices.
      lot.boundary.outer.forEach((v, i) => {
        result.push({ ref: { type: "boundary-outer", index: i, lotId: lid }, pos: v });
      });
      lot.boundary.holes.forEach((hole, hi) => {
        // Edge midpoint handles for hole edges.
        for (let i = 0; i < hole.length; i++) {
          const a = hole[i];
          const b = hole[(i + 1) % hole.length];
          const c = lot.boundary.hole_curves?.[hi]?.[i];
          const pos = c
            ? evalCubic(a, c.cp1, c.cp2, b, 0.5)
            : { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
          result.push({
            ref: { type: "edge-midpoint", index: i, cpTarget: "hole", holeIndex: hi, lotId: lid },
            pos,
          });
        }
        // Curve control points for hole edges.
        lot.boundary.hole_curves?.[hi]?.forEach((c, i) => {
          if (!c) return;
          result.push({
            ref: { type: "curve-cp", index: i, cpIndex: 0, cpTarget: "hole", holeIndex: hi, lotId: lid },
            pos: c.cp1,
          });
          result.push({
            ref: { type: "curve-cp", index: i, cpIndex: 1, cpTarget: "hole", holeIndex: hi, lotId: lid },
            pos: c.cp2,
          });
        });
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
      // Drag edge midpoint → create or adjust curve so bezier(0.5) = pos.
      // For a cubic bezier, B(0.5) = (p0 + 3*cp1 + 3*cp2 + p3) / 8.
      // We want B(0.5) = pos, with cp1 and cp2 symmetric around the
      // chord midpoint's perpendicular. We solve for the shared offset.
      const verts = ref.cpTarget === "outer" ? lot.boundary.outer : lot.boundary.holes[ref.holeIndex!];
      const n = verts.length;
      const a = verts[ref.index];
      const b = verts[(ref.index + 1) % n];
      const chordMid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
      const distFromChord = Math.sqrt((pos.x - chordMid.x) ** 2 + (pos.y - chordMid.y) ** 2);

      // Snap to straight if dragged back near the chord midpoint.
      if (distFromChord < 2) {
        if (ref.cpTarget === "outer") {
          lot.boundary.outer_curves?.[ref.index] && (lot.boundary.outer_curves[ref.index] = null);
        } else {
          const hc = lot.boundary.hole_curves?.[ref.holeIndex!];
          if (hc?.[ref.index]) hc[ref.index] = null;
        }
      } else {
        // Position CPs so bezier(0.5) = pos with symmetric C-shape.
        // B(0.5) = (p0 + 3*cp1 + 3*cp2 + p3) / 8 = chordMid + 0.75*offset.
        const scale = 1 / 0.75;
        const ox = (pos.x - chordMid.x) * scale;
        const oy = (pos.y - chordMid.y) * scale;
        const dx = (b.x - a.x) * 0.15;
        const dy = (b.y - a.y) * 0.15;
        const curve: EdgeCurve = {
          cp1: { x: chordMid.x + ox - dx, y: chordMid.y + oy - dy },
          cp2: { x: chordMid.x + ox + dx, y: chordMid.y + oy + dy },
        };
        if (ref.cpTarget === "outer") {
          if (!lot.boundary.outer_curves) lot.boundary.outer_curves = new Array(n).fill(null);
          lot.boundary.outer_curves[ref.index] = curve;
        } else {
          if (!lot.boundary.hole_curves) lot.boundary.hole_curves = [];
          if (!lot.boundary.hole_curves[ref.holeIndex!]) {
            lot.boundary.hole_curves[ref.holeIndex!] = new Array(n).fill(null);
          }
          lot.boundary.hole_curves[ref.holeIndex!][ref.index] = curve;
        }
      }
      lot.aisleGraph = null;
    } else if (ref.type === "boundary-outer") {
      const old = lot.boundary.outer[ref.index];
      const dx = pos.x - old.x;
      const dy = pos.y - old.y;
      lot.boundary.outer[ref.index] = pos;
      // Drag adjacent curve control points with the vertex.
      const oc = lot.boundary.outer_curves;
      if (oc) {
        const n = lot.boundary.outer.length;
        const prevEdge = (ref.index - 1 + n) % n;
        if (oc[prevEdge]) {
          oc[prevEdge]!.cp2 = { x: oc[prevEdge]!.cp2.x + dx, y: oc[prevEdge]!.cp2.y + dy };
        }
        if (oc[ref.index]) {
          oc[ref.index]!.cp1 = { x: oc[ref.index]!.cp1.x + dx, y: oc[ref.index]!.cp1.y + dy };
        }
      }
      for (const dl of lot.driveLines) {
        this.resolveBoundaryPin(dl, lot);
      }
      lot.aisleGraph = null;
    } else if (ref.type === "boundary-hole" && ref.holeIndex !== undefined) {
      const old = lot.boundary.holes[ref.holeIndex][ref.index];
      const dx = pos.x - old.x;
      const dy = pos.y - old.y;
      lot.boundary.holes[ref.holeIndex][ref.index] = pos;
      // Drag adjacent hole curve control points with the vertex.
      const hc = lot.boundary.hole_curves?.[ref.holeIndex];
      if (hc) {
        const n = lot.boundary.holes[ref.holeIndex].length;
        const prevEdge = (ref.index - 1 + n) % n;
        if (hc[prevEdge]) {
          hc[prevEdge]!.cp2 = { x: hc[prevEdge]!.cp2.x + dx, y: hc[prevEdge]!.cp2.y + dy };
        }
        if (hc[ref.index]) {
          hc[ref.index]!.cp1 = { x: hc[ref.index]!.cp1.x + dx, y: hc[ref.index]!.cp1.y + dy };
        }
      }
      for (const dl of lot.driveLines) {
        if (dl.holePin?.holeIndex === ref.holeIndex && dl.holePin?.vertexIndex === ref.index) {
          dl.start = pos;
        }
      }
      lot.aisleGraph = null;
    } else if (ref.type === "curve-cp") {
      const curves = ref.cpTarget === "outer"
        ? lot.boundary.outer_curves
        : lot.boundary.hole_curves?.[ref.holeIndex!];
      if (curves?.[ref.index]) {
        if (ref.cpIndex === 0) curves[ref.index]!.cp1 = pos;
        else curves[ref.index]!.cp2 = pos;
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

  toggleEdgeCurve(edgeIndex: number, target: "outer" | "hole", holeIndex?: number): void {
    const lot = this.activeLot();
    const verts = target === "outer" ? lot.boundary.outer : lot.boundary.holes[holeIndex!];
    if (target === "outer") {
      if (!lot.boundary.outer_curves) lot.boundary.outer_curves = new Array(verts.length).fill(null);
      const curves = lot.boundary.outer_curves;
      if (curves[edgeIndex]) {
        curves[edgeIndex] = null;
      } else {
        curves[edgeIndex] = defaultCurveForEdge(verts, edgeIndex);
      }
    } else {
      if (!lot.boundary.hole_curves) lot.boundary.hole_curves = [];
      if (!lot.boundary.hole_curves[holeIndex!]) {
        lot.boundary.hole_curves[holeIndex!] = new Array(verts.length).fill(null);
      }
      const curves = lot.boundary.hole_curves[holeIndex!];
      if (curves[edgeIndex]) {
        curves[edgeIndex] = null;
      } else {
        curves[edgeIndex] = defaultCurveForEdge(verts, edgeIndex);
      }
    }
    lot.aisleGraph = null;
    this.generate();
  }

  insertBoundaryVertex(index: number, pos: Vec2, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.boundary.outer.splice(index + 1, 0, pos);
    // Maintain curve array: split the edge's curve or insert null.
    if (lot.boundary.outer_curves && lot.boundary.outer_curves.length > 0) {
      const oldCurve = lot.boundary.outer_curves[index];
      if (oldCurve) {
        // Split the bezier at the nearest t to the insertion point.
        const a = lot.boundary.outer[index];
        const b = lot.boundary.outer[(index + 2) % lot.boundary.outer.length];
        const t = nearestTOnCubic(a, oldCurve.cp1, oldCurve.cp2, b, pos);
        const [left, right] = splitCubicAt(a, oldCurve.cp1, oldCurve.cp2, b, t);
        lot.boundary.outer_curves.splice(index, 1, left, right);
      } else {
        lot.boundary.outer_curves.splice(index + 1, 0, null);
      }
    }
    lot.aisleGraph = null;
    this.generate();
  }

  deleteBoundaryVertex(index: number, lotId?: string): boolean {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    if (lot.boundary.outer.length <= 3) return false;
    lot.boundary.outer.splice(index, 1);
    // Maintain curve array: remove the edge entry and revert the merged edge to straight.
    if (lot.boundary.outer_curves && lot.boundary.outer_curves.length > 0) {
      const n = lot.boundary.outer_curves.length;
      const prevEdge = (index - 1 + n) % n;
      // Remove curve at the deleted vertex's edge and revert the previous edge to straight.
      lot.boundary.outer_curves.splice(index, 1);
      if (lot.boundary.outer_curves[prevEdge % lot.boundary.outer_curves.length]) {
        lot.boundary.outer_curves[prevEdge % lot.boundary.outer_curves.length] = null;
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
      lot.boundary.hole_curves?.splice(holeIndex, 1);
    } else {
      hole.splice(vertexIndex, 1);
      const hc = lot.boundary.hole_curves?.[holeIndex];
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
    const hc = lot.boundary.hole_curves?.[holeIndex];
    if (hc && hc.length > 0) {
      const oldCurve = hc[afterIndex];
      if (oldCurve) {
        const hole = lot.boundary.holes[holeIndex];
        const a = hole[afterIndex];
        const b = hole[(afterIndex + 2) % hole.length];
        const t = nearestTOnCubic(a, oldCurve.cp1, oldCurve.cp2, b, pos);
        const [left, right] = splitCubicAt(a, oldCurve.cp1, oldCurve.cp2, b, t);
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
      this.addLot({ outer: [...this.state.pendingBoundary], holes: [] });
      this.generate();
    }
    this.state.pendingBoundary = [];
  }

  addDriveLine(start: Vec2, end: Vec2): void {
    this.activeLot().driveLines.push({ id: this.newDriveLineId(), start, end });
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
    return computeBoundaryPin(pt, l.boundary.outer, l.boundary.outer_curves);
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
    dl.end = evalBoundaryEdge(lot.boundary.outer, edgeIndex, t, lot.boundary.outer_curves);
  }

  cycleAnnotationDirection(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    const ann = lot.annotations[index];
    if (!ann) return;

    // Abstract variants: cycle through
    //   TwoWayOriented(a→b) → TwoWayOriented(b→a) → OneWay(a→b) →
    //   OneWay(b→a) → inactive (tombstone) → back to TwoWayOriented(a→b)
    if (
      ann.kind === "AbstractTwoWayOriented" ||
      ann.kind === "AbstractOneWay"
    ) {
      const swap = () => {
        const oxa = ann.xa, oya = ann.ya;
        ann.xa = ann.xb;
        ann.ya = ann.yb;
        ann.xb = oxa;
        ann.yb = oya;
      };
      if (ann._active === false) {
        (ann as any).kind = "AbstractTwoWayOriented";
        ann._active = true;
      } else if (ann.kind === "AbstractTwoWayOriented") {
        // Flip, then downgrade to OneWay on the second press.
        // We detect "already flipped once" by a sentinel field? Simpler:
        // just downgrade every second press. No persistent _origDir
        // here, since the abstract identity is the (xa,ya)↔(xb,yb)
        // pair — after one swap we're back to equivalent.
        (ann as any).kind = "AbstractOneWay";
      } else {
        // AbstractOneWay active: swap, then on next press tombstone.
        // Detect "just swapped" via a simple flag.
        if ((ann as any)._swapped) {
          ann._active = false;
          (ann as any)._swapped = false;
        } else {
          swap();
          (ann as any)._swapped = true;
        }
      }
      this.generate();
      return;
    }

    // Splice variants: same cycle as abstract, swapping ta/tb.
    if (ann.kind === "SpliceTwoWayOriented" || ann.kind === "SpliceOneWay") {
      const swap = () => {
        const ota = ann.ta;
        ann.ta = ann.tb;
        ann.tb = ota;
      };
      if (ann._active === false) {
        (ann as any).kind = "SpliceTwoWayOriented";
        ann._active = true;
      } else if (ann.kind === "SpliceTwoWayOriented") {
        (ann as any).kind = "SpliceOneWay";
      } else {
        if ((ann as any)._swapped) {
          ann._active = false;
          (ann as any)._swapped = false;
        } else {
          swap();
          (ann as any)._swapped = true;
        }
      }
      this.generate();
      return;
    }

    // Anything else (Abstract*Delete or unknown) — no cycle.
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

    // Try abstract grid first (regular grid intersections), then
    // splice (drive-line crossings). Both addressing schemes survive
    // parameter changes; the splice scheme survives drive-line edits
    // too. If neither resolves (e.g., a boundary corner that's neither
    // on the grid nor on a drive line), bail — those vertices have no
    // stable identity yet.
    const abs = lot.layout?.region_debug
      ? worldToAbstractVertex(v, this.state.params, lot.layout.region_debug)
      : null;
    if (abs) {
      lot.annotations.push({
        kind: "AbstractDeleteVertex",
        region: abs.region,
        xi: abs.xi,
        yi: abs.yi,
      });
    } else {
      const sp = worldToSpliceVertex(v, lot.driveLines);
      if (sp) {
        lot.annotations.push({
          kind: "SpliceDeleteVertex",
          drive_line_id: sp.drive_line_id,
          t: sp.t,
        });
      } else {
        return; // no stable anchor for this vertex
      }
    }
    this.state.selectedVertex = null;
    lot.aisleGraph = null;
    this.generate();
  }

  /**
   * Compute the abstract-lattice scope for an aisle-edge selection.
   * Segment mode returns the seed sub-edge's extents. Chain mode
   * widens to the full collinear run, projected into the seed's
   * region frame (no tolerance rejection — unlike
   * `chainToAbstractLatticeEdge`, this always succeeds on a multi-edge
   * chain so chain mode never silently collapses into segment mode).
   */
  private computeChainScope(
    sel: EdgeRef,
    graph: DriveAisleGraph,
    lot: ParkingLot,
    regionId: number,
    seedScope: { xa: number; ya: number; xb: number; yb: number },
  ): { xa: number; ya: number; xb: number; yb: number } {
    if (sel.mode !== "chain" || sel.chain.length <= 1) return seedScope;
    const region = lot.layout?.region_debug?.regions.find((r) => r.id === regionId);
    if (!region) return seedScope;
    const chainPts: Vec2[] = [];
    for (const ei of sel.chain) {
      const ce = graph.edges[ei];
      if (!ce) continue;
      chainPts.push(graph.vertices[ce.start]);
      chainPts.push(graph.vertices[ce.end]);
    }
    const extents = chainExtentsInRegion(chainPts, this.state.params, region);
    if (!extents) return seedScope;
    return { xa: extents.xa, ya: extents.ya, xb: extents.xb, yb: extents.yb };
  }

  deleteSelectedEdge(): void {
    const sel = this.state.selectedEdge;
    if (!sel) return;
    const lot = this.activeLot();
    const graph = this.getEffectiveAisleGraph(lot);
    if (!graph) return;

    // Seed sub-edge — the clicked one. Its abstract-lattice extents
    // are the segment-mode scope AND, when chain mode widens the
    // scope, the anchor that pins the marker to the click.
    const seedEdge = graph.edges[sel.index];
    if (!seedEdge) return;
    const s = graph.vertices[seedEdge.start];
    const e = graph.vertices[seedEdge.end];
    const absA = lot.layout?.region_debug
      ? worldToAbstractVertex(s, this.state.params, lot.layout.region_debug)
      : null;
    const absB = lot.layout?.region_debug
      ? worldToAbstractVertex(e, this.state.params, lot.layout.region_debug)
      : null;

    if (absA && absB && absA.region === absB.region) {
      const scope = this.computeChainScope(
        sel, graph, lot, absA.region,
        { xa: absA.xi, ya: absA.yi, xb: absB.xi, yb: absB.yi },
      );
      const anchor = (scope.xa !== absA.xi || scope.ya !== absA.yi ||
                      scope.xb !== absB.xi || scope.yb !== absB.yi)
        ? { xa: absA.xi, ya: absA.yi, xb: absB.xi, yb: absB.yi }
        : undefined;
      lot.annotations.push({
        kind: "AbstractDeleteEdge",
        region: absA.region,
        xa: scope.xa, ya: scope.ya,
        xb: scope.xb, yb: scope.yb,
        ...(anchor ? { anchor } : {}),
      });
      this.state.selectedEdge = null;
      lot.aisleGraph = null;
      this.generate();
      return;
    }

    // Splice path: chain lies on a single drive line and isn't
    // grid-addressable. Segment mode deletes just the seed edge;
    // chain mode spans the whole collinear run's t-range.
    const sa = worldToSpliceVertex(s, lot.driveLines);
    const sb = worldToSpliceVertex(e, lot.driveLines);
    if (!(sa && sb && sa.drive_line_id === sb.drive_line_id)) return;
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
      kind: "SpliceDeleteEdge",
      drive_line_id: sa.drive_line_id,
      ta,
      tb,
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
    const s = graph.vertices[seed.start];
    const e = graph.vertices[seed.end];

    // Resolve the seed sub-edge's endpoints to abstract grid points.
    // This is both the segment-mode scope AND (when chain mode) the
    // click anchor that pins the marker.
    const absA = lot.layout?.region_debug
      ? worldToAbstractVertex(s, this.state.params, lot.layout.region_debug)
      : null;
    const absB = lot.layout?.region_debug
      ? worldToAbstractVertex(e, this.state.params, lot.layout.region_debug)
      : null;
    if (absA && absB && absA.region === absB.region) {
      const scope = this.computeChainScope(
        sel, graph, lot, absA.region,
        { xa: absA.xi, ya: absA.yi, xb: absB.xi, yb: absB.yi },
      );
      // Anchor is the clicked seed sub-edge; only meaningful when the
      // scope extends past it (chain mode spanning multiple cells).
      const anchor = (scope.xa !== absA.xi || scope.ya !== absA.yi ||
                      scope.xb !== absB.xi || scope.yb !== absB.yi)
        ? { xa: absA.xi, ya: absA.yi, xb: absB.xi, yb: absB.yi }
        : undefined;

      // Match existing direction annotation by scope (not anchor).
      // Either orientation matches — cycle flips orientation.
      const existing = lot.annotations.findIndex((ann) => {
        if (
          ann.kind !== "AbstractOneWay" &&
          ann.kind !== "AbstractTwoWayOriented"
        ) {
          return false;
        }
        if (ann.region !== absA.region) return false;
        const matchFwd =
          ann.xa === scope.xa && ann.ya === scope.ya &&
          ann.xb === scope.xb && ann.yb === scope.yb;
        const matchRev =
          ann.xa === scope.xb && ann.ya === scope.yb &&
          ann.xb === scope.xa && ann.yb === scope.ya;
        return matchFwd || matchRev;
      });
      if (existing < 0) {
        const annIdx = lot.annotations.length;
        lot.annotations.push({
          kind: "AbstractTwoWayOriented",
          region: absA.region,
          xa: scope.xa, ya: scope.ya,
          xb: scope.xb, yb: scope.yb,
          ...(anchor ? { anchor } : {}),
          _active: true,
        });
        this.state.selectedVertex = { type: "annotation", index: annIdx, lotId: lot.id };
      } else {
        this.cycleAnnotationDirection(existing, lot.id);
        this.state.selectedVertex = { type: "annotation", index: existing, lotId: lot.id };
        return;
      }
      this.generate();
      return;
    }

    // Splice path: both endpoints on the same drive line.
    const sa = worldToSpliceVertex(s, lot.driveLines);
    const sb = worldToSpliceVertex(e, lot.driveLines);
    if (!(sa && sb && sa.drive_line_id === sb.drive_line_id)) {
      // No stable anchor — skip silently.
      return;
    }
    const sliceTol = 1e-3;
    const existing = lot.annotations.findIndex((ann) => {
      if (ann.kind !== "SpliceOneWay" && ann.kind !== "SpliceTwoWayOriented") return false;
      if (ann.drive_line_id !== sa.drive_line_id) return false;
      const fwd = Math.abs(ann.ta - sa.t) < sliceTol && Math.abs(ann.tb - sb.t) < sliceTol;
      const rev = Math.abs(ann.ta - sb.t) < sliceTol && Math.abs(ann.tb - sa.t) < sliceTol;
      return fwd || rev;
    });
    if (existing < 0) {
      const annIdx = lot.annotations.length;
      lot.annotations.push({
        kind: "SpliceTwoWayOriented",
        drive_line_id: sa.drive_line_id,
        ta: sa.t,
        tb: sb.t,
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

function pointToSegmentDist(p: Vec2, a: Vec2, b: Vec2): number {
  const dx = b.x - a.x, dy = b.y - a.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return Math.sqrt((p.x - a.x) ** 2 + (p.y - a.y) ** 2);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq));
  const px = a.x + t * dx, py = a.y + t * dy;
  return Math.sqrt((p.x - px) ** 2 + (p.y - py) ** 2);
}
