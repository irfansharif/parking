// Engine types are generated from Rust via ts-rs — the sole source of
// truth lives in engine/src/types/*.rs. This file re-exports those
// types and adds a thin layer of UI-session glue: `ParkingLot` (the
// lot + its layout + editing state), and intersection types for the
// UI-only fields engine serde tolerates (`DriveLine.boundaryPin`,
// `Annotation._active`).
//
// The math helpers below are TypeScript wrappers around wasm exports.
// Each wrapper preserves the ergonomic `(Vec2, Vec2, bulge)`-style
// signature the rest of the UI expects; the wasm boundary passes flat
// `f64`s to stay cheap in hot render/event paths.

import {
  annotation_world_pos_js,
  arc_apex_js,
  bulge_from_apex_js,
  chain_extents_in_region_js,
  chain_to_abstract_lattice_edge_js,
  compute_boundary_pin_js,
  compute_region_frame_js,
  effective_depth_js,
  eval_arc_at_js,
  eval_boundary_edge_js,
  frame_forward_js,
  frame_inverse_js,
  point_in_polygon_js,
  project_to_arc_js,
  split_arc_at_js,
  stall_pitch_js,
  target_world_pos_js,
  world_to_abstract_vertex_js,
  world_to_perimeter_pos_js,
  world_to_splice_vertex_js,
} from "./wasm/parking_lot_engine";

import type {
  AbstractFrame,
  AbstractPoint2,
  AbstractVertexResult,
  ChainLatticeEdge,
  EdgeArc,
  ParkingLayout,
  ParkingParams,
  Polygon,
  RegionDebug,
  RegionInfo,
  SpliceVertexResult,
  PerimeterPosResult,
  Vec2,
  Annotation as EngineAnnotation,
  DriveLine as EngineDriveLine,
  Target as EngineTarget,
} from "./bindings";

export type {
  AbstractFrame,
  AbstractPoint2,
  AbstractVertexResult,
  AisleDirection,
  AisleEdge,
  Axis,
  ChainLatticeEdge,
  DebugToggles,
  DriveAisleGraph,
  EdgeArc,
  Face,
  GenerateInput,
  GridStop,
  HolePin,
  Island,
  Metrics,
  ParkingLayout,
  ParkingParams,
  PerimeterLoop,
  PerimeterPosResult,
  Polygon,
  RegionDebug,
  RegionId,
  RegionInfo,
  RegionOverride,
  SpineLine,
  SpliceVertexResult,
  StallKind,
  StallModifier,
  StallQuad,
  Vec2,
} from "./bindings";

// ---------------------------------------------------------------------------
// UI extensions of engine types.
//
// `DriveLine.boundaryPin` — UI-only pairing of a hole-pinned drive
// line's far endpoint back to a boundary edge so it tracks when the
// user drags the outer polygon. The engine ignores the field (serde
// tolerates unknown fields by default).
//
// `Annotation._active` — UI-only tombstone flag used while cycling
// direction states before committing or discarding. The UI filters
// inactive annotations out before handing input to the engine.
//
// Both ride on engine shapes via intersection rather than polluting
// the Rust side with UI-session concepts.
// ---------------------------------------------------------------------------

export type DriveLine = EngineDriveLine & {
  boundaryPin?: { edgeIndex: number; t: number };
};

export type Target = EngineTarget;

export type Annotation = EngineAnnotation & { _active?: boolean };

// ---------------------------------------------------------------------------
// ParkingLot — UI session aggregate. Holds one engine input (+ its
// last layout) together with the UI-only state that goes with editing
// it (aisle vector for the drag handle, cached baseline for the
// aisle-offset delta, per-region overrides keyed for lookup).
// Intentionally NOT generated from Rust; the engine has no analog.
// ---------------------------------------------------------------------------

export interface ParkingLot {
  boundary: Polygon;
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

// ---------------------------------------------------------------------------
// Math helpers — wasm wrappers.
//
// Arc primitives pass flat `f64` args through wasm_bindgen and get
// back `[x, y]` / `[t, x, y]` Float64Arrays. Frame math uses a
// serde-wasm-bindgen round-trip for the compound `AbstractFrame` /
// `ParkingParams` structs.
// ---------------------------------------------------------------------------

/** Compute the root (lot-wide) abstract frame from params. */
export function computeRootFrame(params: ParkingParams): AbstractFrame {
  return compute_region_frame_js(params, params.aisle_angle_deg, params.aisle_offset) as AbstractFrame;
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
  return compute_region_frame_js(params, aisleAngleDeg, aisleOffset) as AbstractFrame;
}

/** Forward transform: abstract → world. */
export function frameForward(frame: AbstractFrame, p: AbstractPoint2): Vec2 {
  const [x, y] = frame_forward_js(frame, p.x, p.y);
  return { x, y };
}

/**
 * Inverse transform: world → abstract. Relies on x_dir and y_dir being
 * orthonormal, so the inverse is just a projection onto each axis
 * followed by a division by the corresponding scale.
 */
export function frameInverse(frame: AbstractFrame, w: Vec2): AbstractPoint2 {
  const [x, y] = frame_inverse_js(frame, w.x, w.y);
  return { x, y };
}

export function arcApex(p0: Vec2, p1: Vec2, bulge: number): Vec2 {
  const [x, y] = arc_apex_js(p0.x, p0.y, p1.x, p1.y, bulge);
  return { x, y };
}

export function bulgeFromApex(p0: Vec2, p1: Vec2, apex: Vec2): number {
  return bulge_from_apex_js(p0.x, p0.y, p1.x, p1.y, apex.x, apex.y);
}

/** Sample a point at parameter `t` ∈ [0, 1] along an arc edge. */
export function evalArcAt(p0: Vec2, p1: Vec2, bulge: number, t: number): Vec2 {
  const [x, y] = eval_arc_at_js(p0.x, p0.y, p1.x, p1.y, bulge, t);
  return { x, y };
}

/**
 * Closest point on an arc edge to `pt`, returned as the parameter `t`
 * and the projected position. Closed-form solve in the engine; this
 * wrapper unpacks the flat `[t, x, y]` triple into the struct the rest
 * of the UI expects.
 */
export function projectToArc(
  p0: Vec2, p1: Vec2, bulge: number, pt: Vec2,
): { pos: Vec2; t: number } {
  const [t, x, y] = project_to_arc_js(p0.x, p0.y, p1.x, p1.y, bulge, pt.x, pt.y);
  return { pos: { x, y }, t };
}

/** Evaluate a point at parameter `t` along boundary edge `edgeIndex`. */
export function evalBoundaryEdge(
  outer: Vec2[], edgeIndex: number, t: number, arcs?: (EdgeArc | null)[],
): Vec2 {
  const [x, y] = eval_boundary_edge_js(outer, edgeIndex, t, arcs ?? null);
  return { x, y };
}

/** Project a world point onto the nearest outer boundary edge. */
export function computeBoundaryPin(
  pt: Vec2, outer: Vec2[], arcs?: (EdgeArc | null)[],
): { pos: Vec2; edgeIndex: number; t: number } {
  const [px, py, edgeIndex, t] = compute_boundary_pin_js(
    pt.x, pt.y, outer, arcs ?? null,
  );
  return { pos: { x: px, y: py }, edgeIndex, t };
}

/**
 * Split an arc at parameter `t`. Returns the two sub-arcs that
 * together cover the original; `null` for sub-arcs that are
 * effectively straight.
 */
export function splitArcAt(
  bulge: number, t: number,
): [EdgeArc | null, EdgeArc | null] {
  const [b1, b2] = split_arc_at_js(bulge, t);
  return [
    Number.isNaN(b1) ? null : { bulge: b1 },
    Number.isNaN(b2) ? null : { bulge: b2 },
  ];
}

// ---------------------------------------------------------------------------
// Arc rendering — the one piece of arc math that stays TS-side for
// now. The renderer calls `bulgeToArc` per curved edge per frame to
// draw Canvas arcs (center/radius/startAngle/sweep are the native
// CanvasRenderingContext2D inputs). Moving it through wasm each frame
// would add per-call marshaling cost to the hot path for zero benefit
// — the engine's `bulge_to_arc` and this one produce identical output.
// Step 3+ may revisit.
// ---------------------------------------------------------------------------

export interface ArcDef {
  center: Vec2;
  radius: number;
  startAngle: number;
  sweep: number;
}

export function bulgeToArc(p0: Vec2, p1: Vec2, bulge: number): ArcDef {
  const cx = p1.x - p0.x, cy = p1.y - p0.y;
  const l = Math.sqrt(cx * cx + cy * cy);
  const midX = (p0.x + p1.x) * 0.5;
  const midY = (p0.y + p1.y) * 0.5;
  const perpX = -cy / l;
  const perpY = cx / l;
  const radius = l * (1 + bulge * bulge) / (4 * Math.abs(bulge));
  const offset = (bulge * bulge - 1) * l / (4 * bulge);
  const center = { x: midX + perpX * offset, y: midY + perpY * offset };
  const startAngle = Math.atan2(p0.y - center.y, p0.x - center.x);
  const sweep = -4 * Math.atan(bulge);
  return { center, radius, startAngle, sweep };
}

// ---------------------------------------------------------------------------
// Substrate resolution — all delegate to `engine/src/resolve.rs` via
// wasm. The TS wrappers exist only to preserve the ergonomic
// signatures callers already use (e.g. `worldToPerimeterPos(v,
// boundary)` versus explicit coordinate unpacking on every call).
// ---------------------------------------------------------------------------

/** Point-in-polygon test (ray-casting, even-odd rule). */
export function pointInPolygon(p: Vec2, poly: Vec2[]): boolean {
  return point_in_polygon_js(p.x, p.y, poly);
}

/**
 * Convert a world position into a `(region, xi, yi)` triple naming a
 * grid-lattice intersection. Returns null if the point lies outside
 * every region's clip polygon, or the nearest integer lattice point is
 * farther than `snap_tol_world` from the query in world units.
 */
export function worldToAbstractVertex(
  world: Vec2,
  params: ParkingParams,
  regionDebug: RegionDebug | undefined,
  snapTolWorld: number = 0.5,
): AbstractVertexResult | null {
  return world_to_abstract_vertex_js(
    world.x,
    world.y,
    params,
    regionDebug ?? null,
    snapTolWorld,
  ) as AbstractVertexResult | null;
}

/** Strict chain → lattice-edge extents (see `resolve::chain_to_abstract_lattice_edge`). */
export function chainToAbstractLatticeEdge(
  chainPoints: Vec2[],
  params: ParkingParams,
  regionDebug: RegionDebug | undefined,
): ChainLatticeEdge | null {
  return chain_to_abstract_lattice_edge_js(
    chainPoints,
    params,
    regionDebug ?? null,
  ) as ChainLatticeEdge | null;
}

/** Lenient chain extents in a specified region (see `resolve::chain_extents_in_region`). */
export function chainExtentsInRegion(
  chainPoints: Vec2[],
  params: ParkingParams,
  region: RegionInfo,
): ChainLatticeEdge | null {
  return chain_extents_in_region_js(chainPoints, params, region) as ChainLatticeEdge | null;
}

/** Project a world point onto the closest drive line; returns `(id, t)` or null. */
export function worldToSpliceVertex(
  world: Vec2,
  driveLines: DriveLine[],
  snapTolWorld: number = 0.5,
): SpliceVertexResult | null {
  return world_to_splice_vertex_js(
    world.x,
    world.y,
    driveLines,
    snapTolWorld,
  ) as SpliceVertexResult | null;
}

/** True if the annotation is a direction annotation. */
export function isDirectionAnnotation(
  ann: Annotation,
): ann is Extract<Annotation, { kind: "Direction" }> {
  return ann.kind === "Direction";
}

/**
 * Resolve an annotation's target to its current world-space anchor
 * point. Returns null when the substrate (region / drive line /
 * perimeter loop) isn't present in the current generate — i.e. the
 * annotation is dormant.
 */
export function annotationWorldPos(
  ann: Annotation,
  lot: ParkingLot,
  params: ParkingParams,
): Vec2 | null {
  return annotation_world_pos_js(
    ann,
    lot.boundary,
    lot.driveLines,
    lot.layout?.region_debug ?? null,
    params,
  ) as Vec2 | null;
}

export function targetWorldPos(
  target: Target,
  lot: ParkingLot,
  params: ParkingParams,
): Vec2 | null {
  return target_world_pos_js(
    target,
    lot.boundary,
    lot.driveLines,
    lot.layout?.region_debug ?? null,
    params,
  ) as Vec2 | null;
}

/** Project a world point onto the nearest perimeter loop. The boundary
 * must carry `outer_ids` / `hole_ids` parallel to its vertex arrays —
 * call `ensurePolygonIds` first if you're constructing one fresh. */
export function worldToPerimeterPos(
  v: Vec2,
  boundary: Polygon,
  tol: number = 0.5,
): PerimeterPosResult | null {
  return world_to_perimeter_pos_js(v.x, v.y, boundary, tol) as PerimeterPosResult | null;
}

// ---------------------------------------------------------------------------
// Vertex-id allocation helpers. Boundary annotations reference sketch
// edges by `(start_vid, end_vid)` pairs; the UI is responsible for
// keeping `outer_ids` / `hole_ids` in sync with vertex mutations so
// that ids stay stable across edits unrelated to a given annotation's
// edge. When a vertex is inserted on an annotation's edge or one of
// the endpoints is deleted, the engine returns dormant — the user
// re-adds the annotation. There is no migration / promotion.
// ---------------------------------------------------------------------------

/** Highest VertexId currently in use across the boundary. */
function maxBoundaryVid(boundary: Polygon): number {
  let m = 0;
  if (Array.isArray(boundary.outer_ids)) {
    for (const id of boundary.outer_ids) if (id > m) m = id;
  }
  if (Array.isArray(boundary.hole_ids)) {
    for (const ring of boundary.hole_ids) {
      if (Array.isArray(ring)) {
        for (const id of ring) if (id > m) m = id;
      }
    }
  }
  return m;
}

/** Allocate the next available VertexId on the boundary. */
export function nextVertexId(boundary: Polygon): number {
  return maxBoundaryVid(boundary) + 1;
}

/** Populate any missing `outer_ids` / `hole_ids` so they're parallel
 * to `outer` / `holes`. Idempotent: existing ids are preserved when
 * lengths already match. When a length mismatch is detected (e.g. a
 * vertex was inserted without a paired id), all ids on that ring are
 * reallocated from scratch — annotations targeting that ring go
 * dormant on the next regen, matching the "clear on split" rule. */
export function ensurePolygonIds(boundary: Polygon): void {
  let next = maxBoundaryVid(boundary) + 1;
  if (
    !Array.isArray(boundary.outer_ids) ||
    boundary.outer_ids.length !== boundary.outer.length
  ) {
    boundary.outer_ids = boundary.outer.map(() => next++);
  }
  if (
    !Array.isArray(boundary.hole_ids) ||
    boundary.hole_ids.length !== boundary.holes.length
  ) {
    boundary.hole_ids = boundary.holes.map(() => []);
  }
  for (let i = 0; i < boundary.holes.length; i++) {
    if (
      !Array.isArray(boundary.hole_ids[i]) ||
      boundary.hole_ids[i].length !== boundary.holes[i].length
    ) {
      boundary.hole_ids[i] = boundary.holes[i].map(() => next++);
    }
  }
}

// ---------------------------------------------------------------------------
// `stallPitch` / `effectiveDepth` — thin wasm wrappers over
// `ParkingParams::{stall_pitch, effective_depth}`. Used by the chain/
// frame math above and nowhere else today.
// ---------------------------------------------------------------------------

export function stallPitch(params: ParkingParams): number {
  return stall_pitch_js(params);
}

export function effectiveDepth(params: ParkingParams): number {
  return effective_depth_js(params);
}
