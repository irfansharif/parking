export interface Vec2 {
  x: number;
  y: number;
}

export interface StallQuad {
  corners: [Vec2, Vec2, Vec2, Vec2];
  kind: "Standard" | "Compact" | "Ev" | "Extension" | "Island";
}

export interface Metrics {
  total_stalls: number;
}

export interface SpineLine {
  start: Vec2;
  end: Vec2;
  normal: Vec2;
  is_extension?: boolean;
}

export interface Face {
  contour: Vec2[];
  holes?: Vec2[][];
  is_boundary?: boolean;
  edge_sources?: string[];
  hole_edge_sources?: string[][];
}

export interface Island {
  contour: Vec2[];
  holes?: Vec2[][];
  face_idx: number;
}

export interface ParkingLayout {
  aisle_polygons: Vec2[][];
  stalls: StallQuad[];
  metrics: Metrics;
  resolved_graph: DriveAisleGraph;
  spines: SpineLine[];
  faces: Face[];
  miter_fills: Vec2[][];
  skeleton_debug: { arcs: [Vec2, Vec2][]; nodes: Vec2[]; split_nodes: Vec2[]; sources: Vec2[] }[];
  islands: Island[];
  derived_outer?: Vec2[];
  derived_holes?: Vec2[][];
  region_debug?: RegionDebug;
  /**
   * Indices into the input's annotation list for annotations that
   * didn't resolve to any graph feature this generate. For abstract
   * annotations: the (region, xi, yi) lookup missed. For legacy
   * proximity annotations: currently always empty (legacy silently
   * no-ops on a miss).
   */
  dormant_annotations?: number[];
}

export interface RegionDebug {
  regions: RegionInfo[];
  separators: [Vec2, Vec2][];
}

export interface RegionInfo {
  id: RegionId;
  clip_poly: Vec2[];
  aisle_angle_deg: number;
  aisle_offset: number;
  center: Vec2;
}

export type AisleDirection = "TwoWay" | "TwoWayOriented" | "OneWay";

export interface AisleEdge {
  start: number;
  end: number;
  width: number;
  interior?: boolean;
  direction?: AisleDirection;
}

export interface DriveAisleGraph {
  vertices: Vec2[];
  edges: AisleEdge[];
  perim_vertex_count?: number;
}

export interface EdgeCurve {
  cp1: Vec2;
  cp2: Vec2;
}

export interface Polygon {
  outer: Vec2[];
  holes: Vec2[][];
  outer_curves?: (EdgeCurve | null)[];
  hole_curves?: (EdgeCurve | null)[][];
}

export interface ParkingParams {
  stall_width: number;
  stall_depth: number;
  aisle_width: number;
  stall_angle_deg: number;
  aisle_angle_deg: number;
  aisle_offset: number;
  site_offset: number;
  /**
   * Number of stalls along the aisle direction in one face (between
   * adjacent cross driving aisles). Integer. Replaces the old
   * `cross_aisle_max_run` float.
   */
  stalls_per_face: number;
  use_regions?: boolean;
  island_stall_interval?: number;
}

// ---------------------------------------------------------------------------
// AbstractFrame — mirrors engine/src/types.rs::AbstractFrame.
//
// Per-region transformation from the abstract integer grid into world
// space. Derived fresh on every generate, never serialized.
//
//     world(x, y) = origin_world + x * dx * x_dir + y * dy * y_dir
//
// ---------------------------------------------------------------------------

export interface AbstractFrame {
  origin_world: Vec2;
  x_dir: Vec2;           // unit, perpendicular to parallel aisle
  y_dir: Vec2;           // unit, along parallel aisle
  dx: number;            // 2*effective_depth + 2*aisle_width
  dy: number;            // stalls_per_face * stall_pitch
  stalls_per_face: number;
}

/** Real-valued point in an abstract frame. */
export interface AbstractPoint2 {
  x: number;
  y: number;
}

function stallPitch(p: ParkingParams): number {
  const sinA = Math.sin((p.stall_angle_deg * Math.PI) / 180);
  return Math.abs(sinA) > 1e-12 ? p.stall_width / sinA : p.stall_width;
}

function effectiveDepth(p: ParkingParams): number {
  const rad = (p.stall_angle_deg * Math.PI) / 180;
  return p.stall_depth * Math.sin(rad) + Math.cos(rad) * p.stall_width / 2;
}

/** Compute the root (lot-wide) abstract frame from params. */
export function computeRootFrame(params: ParkingParams): AbstractFrame {
  return computeFrameForAngle(params, params.aisle_angle_deg, params.aisle_offset);
}

/**
 * Compute a region frame with a possibly-overridden aisle angle and
 * offset. Used for regions whose aisle angle or offset differs from
 * the lot-wide base.
 */
export function computeRegionFrame(
  params: ParkingParams,
  aisleAngleDeg: number,
  aisleOffset: number = params.aisle_offset,
): AbstractFrame {
  return computeFrameForAngle(params, aisleAngleDeg, aisleOffset);
}

function computeFrameForAngle(
  params: ParkingParams,
  aisleAngleDeg: number,
  aisleOffset: number,
): AbstractFrame {
  const rad = (aisleAngleDeg * Math.PI) / 180;
  const y_dir: Vec2 = { x: Math.cos(rad), y: Math.sin(rad) };
  const x_dir: Vec2 = { x: -Math.sin(rad), y: Math.cos(rad) };
  const dx = 2 * effectiveDepth(params) + 2 * params.aisle_width;
  const stalls_per_face = Math.max(1, Math.round(params.stalls_per_face));
  const dy = stalls_per_face * stallPitch(params);
  // Shift canvas-anchored origin along the perpendicular axis by
  // aisle_offset. Dragging the aisle vector in the UI slides the
  // entire grid (and abstract annotations with it).
  const origin_world: Vec2 = {
    x: x_dir.x * aisleOffset,
    y: x_dir.y * aisleOffset,
  };
  return {
    origin_world,
    x_dir,
    y_dir,
    dx,
    dy,
    stalls_per_face,
  };
}

/**
 * Point-in-polygon test. Even-odd rule. Used by the abstract-handle
 * converter below to figure out which region a world point sits in.
 */
export function pointInPolygon(p: Vec2, poly: Vec2[]): boolean {
  let inside = false;
  const n = poly.length;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const a = poly[i];
    const b = poly[j];
    if (
      a.y > p.y !== b.y > p.y &&
      p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y + 1e-12) + a.x
    ) {
      inside = !inside;
    }
  }
  return inside;
}

/**
 * Try to convert a world position into a `(region, xi, yi)` triple
 * that names a grid intersection in a region's abstract frame. Used by
 * the UI when placing annotations under the abstract-stamp path — a
 * click on a grid vertex becomes an abstract annotation that survives
 * future parameter changes.
 *
 * Returns null if no region contains the point, or if the nearest
 * abstract (xi, yi) is farther than `snap_tol_world` from the point
 * (i.e., the click didn't land on the grid).
 */
export function worldToAbstractVertex(
  world: Vec2,
  params: ParkingParams,
  regionDebug: RegionDebug | undefined,
  snapTolWorld: number = 0.5,
): { region: RegionId; xi: number; yi: number } | null {
  if (!regionDebug) return null;
  for (const region of regionDebug.regions) {
    if (!pointInPolygon(world, region.clip_poly)) continue;
    const frame = computeRegionFrame(
      params,
      region.aisle_angle_deg,
      region.aisle_offset,
    );
    const abs = frameInverse(frame, world);
    const xi = Math.round(abs.x);
    const yi = Math.round(abs.y);
    const dx = Math.abs(abs.x - xi) * frame.dx;
    const dy = Math.abs(abs.y - yi) * frame.dy;
    if (dx > snapTolWorld || dy > snapTolWorld) continue;
    return { region: region.id, xi, yi };
  }
  return null;
}

/**
 * Resolve a world-space chain of collinear aisle sub-edges to the
 * **lattice-edge extents** in some region's abstract frame. Returns
 * (region, xa, ya, xb, yb) describing the unit/multi-cell lattice edge
 * that bounds the chain.
 *
 * Used to address chains that have been split by drive-line or boundary
 * intersections: the chain endpoints land on integer lattice
 * intersections (junctions), while interior break points are fractional.
 * The bounding lattice extents are the rounded min/max of all chain
 * endpoints along the varying axis.
 *
 * Returns null if the chain isn't collinear, no region contains its
 * midpoint, or the rounded extents collapse (degenerate edge).
 */
export function chainToAbstractLatticeEdge(
  chainPoints: Vec2[],
  params: ParkingParams,
  regionDebug: RegionDebug | undefined,
): { region: RegionId; xa: number; ya: number; xb: number; yb: number } | null {
  if (!regionDebug || chainPoints.length < 2) return null;
  const mid = {
    x: chainPoints.reduce((a, p) => a + p.x, 0) / chainPoints.length,
    y: chainPoints.reduce((a, p) => a + p.y, 0) / chainPoints.length,
  };
  for (const region of regionDebug.regions) {
    if (!pointInPolygon(mid, region.clip_poly)) continue;
    const frame = computeRegionFrame(
      params,
      region.aisle_angle_deg,
      region.aisle_offset,
    );
    const abs = chainPoints.map((p) => frameInverse(frame, p));
    const xs = abs.map((p) => p.x);
    const ys = abs.map((p) => p.y);
    const xRange = Math.max(...xs) - Math.min(...xs);
    const yRange = Math.max(...ys) - Math.min(...ys);
    // Reject chains that aren't actually axis-aligned in this frame
    // (e.g. drive-line chains at arbitrary angles). The "constant"
    // axis must lie within snap tolerance of an integer grid line,
    // otherwise we'd silently project the chain onto the nearest
    // lattice edge and anchor the annotation there.
    const snapTol = 0.5;
    if (xRange > yRange) {
      const yi = Math.round(ys.reduce((a, b) => a + b, 0) / ys.length);
      const yDev = Math.max(...ys.map((y) => Math.abs(y - yi))) * frame.dy;
      if (yDev > snapTol) return null;
      const xa = Math.floor(Math.min(...xs));
      const xb = Math.ceil(Math.max(...xs));
      if (xa === xb) return null;
      return { region: region.id, xa, ya: yi, xb, yb: yi };
    } else {
      const xi = Math.round(xs.reduce((a, b) => a + b, 0) / xs.length);
      const xDev = Math.max(...xs.map((x) => Math.abs(x - xi))) * frame.dx;
      if (xDev > snapTol) return null;
      const ya = Math.floor(Math.min(...ys));
      const yb = Math.ceil(Math.max(...ys));
      if (ya === yb) return null;
      return { region: region.id, xa: xi, ya, xb: xi, yb };
    }
  }
  return null;
}

/**
 * Compute the lattice-edge extents of a world-space chain in a *given*
 * region's abstract frame — like `chainToAbstractLatticeEdge` but
 * without the tolerance rejection. The caller has already decided
 * which region the scope should live in (typically the seed sub-edge's
 * region); this helper just floors/ceils the chain's min/max along
 * the varying axis and snaps the fixed axis to the nearest integer.
 *
 * Used for chain-mode annotations that cross region boundaries — the
 * strict variant returns null in that case, silently collapsing chain
 * mode into segment mode.
 */
export function chainExtentsInRegion(
  chainPoints: Vec2[],
  params: ParkingParams,
  region: { id: RegionId; aisle_angle_deg: number; aisle_offset: number },
): { region: RegionId; xa: number; ya: number; xb: number; yb: number } | null {
  if (chainPoints.length < 2) return null;
  const frame = computeRegionFrame(params, region.aisle_angle_deg, region.aisle_offset);
  const abs = chainPoints.map((p) => frameInverse(frame, p));
  const xs = abs.map((p) => p.x);
  const ys = abs.map((p) => p.y);
  const xRange = Math.max(...xs) - Math.min(...xs);
  const yRange = Math.max(...ys) - Math.min(...ys);
  if (xRange > yRange) {
    const yi = Math.round(ys.reduce((a, b) => a + b, 0) / ys.length);
    const xa = Math.floor(Math.min(...xs));
    const xb = Math.ceil(Math.max(...xs));
    if (xa === xb) return null;
    return { region: region.id, xa, ya: yi, xb, yb: yi };
  }
  const xi = Math.round(xs.reduce((a, b) => a + b, 0) / xs.length);
  const ya = Math.floor(Math.min(...ys));
  const yb = Math.ceil(Math.max(...ys));
  if (ya === yb) return null;
  return { region: region.id, xa: xi, ya, xb: xi, yb };
}

/**
 * Project a world-space point onto the closest drive line and return its
 * stable splice anchor — `(drive_line_id, t)` where `t ∈ [0, 1]` is the
 * fractional position along the line. Returns null if no drive line is
 * within `snap_tol_world` of the point. Mirrors `worldToAbstractVertex`
 * for the splice axis.
 */
export function worldToSpliceVertex(
  world: Vec2,
  driveLines: DriveLine[],
  snapTolWorld: number = 0.5,
): { drive_line_id: number; t: number } | null {
  let best: { id: number; t: number; dist: number } | null = null;
  for (const dl of driveLines) {
    const dx = dl.end.x - dl.start.x;
    const dy = dl.end.y - dl.start.y;
    const len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) continue;
    // Project onto infinite line; t may fall outside [0, 1] if the
    // splice extends past the user-drawn endpoints.
    const tRaw = ((world.x - dl.start.x) * dx + (world.y - dl.start.y) * dy) / len2;
    const px = dl.start.x + dx * tRaw;
    const py = dl.start.y + dy * tRaw;
    const dist = Math.hypot(world.x - px, world.y - py);
    if (dist > snapTolWorld) continue;
    if (best === null || dist < best.dist) {
      best = { id: dl.id, t: tRaw, dist };
    }
  }
  return best ? { drive_line_id: best.id, t: best.t } : null;
}

/** Forward transform: abstract → world. */
export function frameForward(frame: AbstractFrame, p: AbstractPoint2): Vec2 {
  return {
    x:
      frame.origin_world.x +
      p.x * frame.dx * frame.x_dir.x +
      p.y * frame.dy * frame.y_dir.x,
    y:
      frame.origin_world.y +
      p.x * frame.dx * frame.x_dir.y +
      p.y * frame.dy * frame.y_dir.y,
  };
}

/**
 * Inverse transform: world → abstract. Relies on x_dir and y_dir being
 * orthonormal, so the inverse is just a projection onto each axis
 * followed by a division by the corresponding scale.
 */
export function frameInverse(frame: AbstractFrame, w: Vec2): AbstractPoint2 {
  const rx = w.x - frame.origin_world.x;
  const ry = w.y - frame.origin_world.y;
  return {
    x: (rx * frame.x_dir.x + ry * frame.x_dir.y) / frame.dx,
    y: (rx * frame.y_dir.x + ry * frame.y_dir.y) / frame.dy,
  };
}

export interface DebugToggles {
  // Corridor merging
  miter_fills: boolean;
  boundary_only_miters: boolean;
  spike_removal: boolean;
  contour_simplification: boolean;
  hole_filtering: boolean;
  // Spine generation
  face_simplification: boolean;
  edge_classification: boolean;
  spine_clipping: boolean;
  // Spine post-processing
  spine_dedup: boolean;
  spine_merging: boolean;
  short_spine_filter: boolean;
  // Spine extensions
  spine_extensions: boolean;
  // Stall placement
  stall_centering: boolean;
  stall_face_clipping: boolean;
  // Boundary
  boundary_clipping: boolean;
  // Conflict removal
  conflict_removal: boolean;
  // Short segment filter
  short_segment_filter: boolean;
  // Edge provenance
  edge_provenance: boolean;
  // Skeleton debug visualization
  skeleton_debug: boolean;
}

export interface DriveLine {
  start: Vec2;
  end: Vec2;
  /** Stable identifier assigned by the UI at creation time. Used by
   *  Splice* annotations as the addressing axis. */
  id: number;
  /** When set, start is pinned to a hole vertex (separator). */
  holePin?: { holeIndex: number; vertexIndex: number };
  /** When set, end is pinned to a boundary edge at parameter t. */
  boundaryPin?: { edgeIndex: number; t: number };
  /** When true, participates in face enumeration and partitions the lot
   *  into regions. Otherwise, the line is a corridor-only drive line
   *  (aisle graph edge, no partitioning effect). */
  partitions?: boolean;
}

/** Project a point onto the nearest outer boundary edge. */
function evalCubicPin(p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2, t: number): Vec2 {
  const s = 1 - t;
  return {
    x: s * s * s * p0.x + 3 * s * s * t * cp1.x + 3 * s * t * t * cp2.x + t * t * t * p3.x,
    y: s * s * s * p0.y + 3 * s * s * t * cp1.y + 3 * s * t * t * cp2.y + t * t * t * p3.y,
  };
}

export function evalBoundaryEdge(
  outer: Vec2[], edgeIndex: number, t: number, curves?: (EdgeCurve | null)[],
): Vec2 {
  const a = outer[edgeIndex];
  const b = outer[(edgeIndex + 1) % outer.length];
  const curve = curves?.[edgeIndex];
  if (curve) {
    return evalCubicPin(a, curve.cp1, curve.cp2, b, t);
  }
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

export function computeBoundaryPin(
  pt: Vec2, outer: Vec2[], curves?: (EdgeCurve | null)[],
): { pos: Vec2; edgeIndex: number; t: number } {
  let bestDist = Infinity;
  let bestProj = pt;
  let bestEdge = 0;
  let bestT = 0;
  for (let i = 0; i < outer.length; i++) {
    const a = outer[i];
    const b = outer[(i + 1) % outer.length];
    const curve = curves?.[i];
    if (curve) {
      // Sample the bezier to find the closest point.
      const N = 20;
      for (let j = 0; j <= N; j++) {
        const t = j / N;
        const p = evalCubicPin(a, curve.cp1, curve.cp2, b, t);
        const dx = pt.x - p.x, dy = pt.y - p.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < bestDist) {
          bestDist = dist;
          bestProj = p;
          bestEdge = i;
          bestT = t;
        }
      }
      // Refine with ternary search around best sample.
      let lo = Math.max(0, bestT - 1 / N);
      let hi = Math.min(1, bestT + 1 / N);
      for (let k = 0; k < 12; k++) {
        const t1 = (2 * lo + hi) / 3;
        const t2 = (lo + 2 * hi) / 3;
        const p1 = evalCubicPin(a, curve.cp1, curve.cp2, b, t1);
        const p2 = evalCubicPin(a, curve.cp1, curve.cp2, b, t2);
        const d1 = (pt.x - p1.x) ** 2 + (pt.y - p1.y) ** 2;
        const d2 = (pt.x - p2.x) ** 2 + (pt.y - p2.y) ** 2;
        if (d1 < d2) hi = t2; else lo = t1;
      }
      const tFinal = (lo + hi) / 2;
      const pFinal = evalCubicPin(a, curve.cp1, curve.cp2, b, tFinal);
      const dFinal = Math.sqrt((pt.x - pFinal.x) ** 2 + (pt.y - pFinal.y) ** 2);
      if (dFinal < bestDist) {
        bestDist = dFinal;
        bestProj = pFinal;
        bestEdge = i;
        bestT = tFinal;
      }
    } else {
      const ab = { x: b.x - a.x, y: b.y - a.y };
      const lenSq = ab.x * ab.x + ab.y * ab.y;
      if (lenSq < 1e-12) continue;
      const t = Math.max(0, Math.min(1, ((pt.x - a.x) * ab.x + (pt.y - a.y) * ab.y) / lenSq));
      const proj = { x: a.x + ab.x * t, y: a.y + ab.y * t };
      const dx = pt.x - proj.x, dy = pt.y - proj.y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < bestDist) {
        bestDist = dist;
        bestProj = proj;
        bestEdge = i;
        bestT = t;
      }
    }
  }
  return { pos: bestProj, edgeIndex: bestEdge, t: bestT };
}

/**
 * A spatial intent that survives graph regeneration. The annotation targets
 * a vertex or one-or-more sub-edges in one substrate's own coord system
 * (grid, drive-line, or perimeter). Single dormancy rule: any referenced
 * substrate or stop missing from the current graph → dormant this regen.
 */
export type Annotation = {
  _active?: boolean;
} & (
  | { kind: "DeleteVertex"; target: Target }
  | { kind: "DeleteEdge"; target: Target }
  | { kind: "Direction"; target: Target; traffic: TrafficDirection }
);

/**
 * Traffic direction relative to the target carrier's canonical direction.
 * Grid line: canonical = +other-axis (low → high). Drive line: canonical =
 * +t (start → end). Perimeter: canonical = CCW (Outer) / CW (Hole).
 */
export type TrafficDirection =
  | "OneWay"
  | "OneWayReverse"
  | "TwoWayOriented"
  | "TwoWayOrientedReverse";

/**
 * A referenceable region in one substrate's coord system.
 * - Grid.range = null → whole grid line (edge annotations only).
 * - Grid.range = [s, s] → a vertex at that stop.
 * - Grid.range = [s1, s2] (s1 ≠ s2) → a span of sub-edges.
 * - DriveLine / Perimeter: a single parametric point; resolution finds
 *   the graph vertex at the point (for vertex annotations) or the
 *   sub-edge containing the point (for edge annotations).
 */
export type Target =
  | {
      on: "Grid";
      region: RegionId;
      axis: "X" | "Y";
      coord: number;
      range: [GridStop, GridStop] | null;
    }
  | { on: "DriveLine"; id: number; t: number }
  | { on: "Perimeter"; loop: PerimeterLoop; arc: number };

export type GridStop =
  | { at: "Lattice"; other: number }
  | { at: "CrossesDriveLine"; id: number }
  | { at: "CrossesPerimeter"; loop: PerimeterLoop };

export type PerimeterLoop =
  | { kind: "Outer" }
  | { kind: "Hole"; index: number };

/** True if the annotation is a direction annotation. */
export function isDirectionAnnotation(
  ann: Annotation,
): ann is Extract<Annotation, { kind: "Direction" }> {
  return ann.kind === "Direction";
}

/**
 * Resolve an annotation's target to its current world-space anchor point,
 * for rendering the marker. Returns null when the target's substrate or
 * stops don't resolve — i.e. the annotation is dormant / off-screen.
 */
export function annotationWorldPos(
  ann: Annotation,
  lot: ParkingLot,
  params: ParkingParams,
): Vec2 | null {
  return targetWorldPos(ann.target, lot, params);
}

export function targetWorldPos(
  target: Target,
  lot: ParkingLot,
  params: ParkingParams,
): Vec2 | null {
  if (target.on === "DriveLine") {
    const dl = lot.driveLines.find((d) => d.id === target.id);
    if (!dl) return null;
    return {
      x: dl.start.x + (dl.end.x - dl.start.x) * target.t,
      y: dl.start.y + (dl.end.y - dl.start.y) * target.t,
    };
  }
  if (target.on === "Grid") {
    const rd = lot.layout?.region_debug;
    if (!rd) return null;
    const region = rd.regions.find((r) => r.id === target.region);
    if (!region) return null;
    const frame = computeRegionFrame(
      params,
      region.aisle_angle_deg,
      region.aisle_offset,
    );
    if (target.range == null) {
      // Whole line — anchor at (coord, 0) on the fixed axis.
      const abs = target.axis === "X"
        ? { x: target.coord, y: 0 }
        : { x: 0, y: target.coord };
      return frameForward(frame, abs);
    }
    const [s1, s2] = target.range;
    const c1 = gridStopCoord(s1);
    const c2 = gridStopCoord(s2);
    if (c1 == null || c2 == null) return null;
    const mid = (c1 + c2) / 2;
    const abs = target.axis === "X"
      ? { x: target.coord, y: mid }
      : { x: mid, y: target.coord };
    return frameForward(frame, abs);
  }
  // Perimeter: not rendered until the engine resolves perimeter targets.
  return null;
}

function gridStopCoord(s: GridStop): number | null {
  return s.at === "Lattice" ? s.other : null;
}

export interface GenerateInput {
  boundary: Polygon;
  aisle_graph: DriveAisleGraph | null;
  drive_lines: DriveLine[];
  annotations: Annotation[];
  params: ParkingParams;
  debug: DebugToggles;
  regionOverrides?: RegionOverride[];
}

export interface ParkingLot {
  id: string;
  boundary: Polygon;
  aisleGraph: DriveAisleGraph | null;
  driveLines: DriveLine[];
  annotations: Annotation[];
  aisleVector: { start: Vec2; end: Vec2 };
  /**
   * Perpendicular projection of the aisle vector's midpoint at the
   * time the lot was created (or the last time the vector was
   * reoriented from scratch). Used to decouple the vector's visual
   * world position from `params.aisle_offset`:
   *
   *   aisle_offset = midpoint.perpProj - aisleOffsetBaseline
   *
   * Without this, the first drag of the vector would jump the grid
   * by (midpoint.perpProj - 0) because the vector's initial world
   * position has a non-zero perpendicular projection.
   */
  aisleOffsetBaseline: number;
  /**
   * Region overrides keyed by RegionId (u64 encoded as a JS number —
   * RegionIds are constrained to 53-bit safe integers on the engine
   * side specifically so JS can hold them losslessly). The string key
   * comes from `String(regionId)`.
   */
  regionOverrides: { [regionId: string]: { angle?: number; offset?: number } };
  layout: ParkingLayout | null;
}

/**
 * Stable identifier for a region. Mirrors engine/src/types.rs::RegionId.
 * Values come from the engine; the UI never constructs them from raw
 * fields.
 */
export type RegionId = number;

export interface RegionOverride {
  region_id: RegionId;
  aisle_angle_deg?: number;
  aisle_offset?: number;
}
