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
  arc_apex_js,
  bulge_from_apex_js,
  compute_region_frame_js,
  effective_depth_js,
  eval_arc_at_js,
  frame_forward_js,
  frame_inverse_js,
  project_to_arc_js,
  stall_pitch_js,
} from "./wasm/parking_lot_engine";

import type {
  AbstractFrame,
  AbstractPoint2,
  EdgeArc,
  GridStop,
  ParkingLayout,
  ParkingParams,
  PerimeterLoop,
  Polygon,
  RegionDebug,
  RegionId,
  Vec2,
  Annotation as EngineAnnotation,
  DriveLine as EngineDriveLine,
  Target as EngineTarget,
} from "./bindings";

export type {
  AbstractFrame,
  AbstractPoint2,
  AisleDirection,
  AisleEdge,
  Axis,
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
  Polygon,
  RegionDebug,
  RegionId,
  RegionInfo,
  RegionOverride,
  SkeletonDebug,
  SpineLine,
  StallKind,
  StallModifier,
  StallQuad,
  TrafficDirection,
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
  id: string;
  boundary: Polygon;
  aisleGraph: import("./bindings").DriveAisleGraph | null;
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
  const a = outer[edgeIndex];
  const b = outer[(edgeIndex + 1) % outer.length];
  const arc = arcs?.[edgeIndex];
  if (arc) return evalArcAt(a, b, arc.bulge, t);
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

/** Project a world point onto the nearest outer boundary edge. */
export function computeBoundaryPin(
  pt: Vec2, outer: Vec2[], arcs?: (EdgeArc | null)[],
): { pos: Vec2; edgeIndex: number; t: number } {
  let bestDist = Infinity;
  let bestProj: Vec2 = pt;
  let bestEdge = 0;
  let bestT = 0;
  for (let i = 0; i < outer.length; i++) {
    const a = outer[i];
    const b = outer[(i + 1) % outer.length];
    const arc = arcs?.[i];
    const proj = arc
      ? projectToArc(a, b, arc.bulge, pt)
      : projectToArc(a, b, 0, pt);
    const dx = pt.x - proj.pos.x, dy = pt.y - proj.pos.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist < bestDist) {
      bestDist = dist;
      bestProj = proj.pos;
      bestEdge = i;
      bestT = proj.t;
    }
  }
  return { pos: bestProj, edgeIndex: bestEdge, t: bestT };
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

const STRAIGHT_EPS = 1e-9;

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
// UI-layer math that still lives TS-side (step 3+ of the audit moves
// these into the engine). Each one operates on world-space inputs +
// engine-produced layout data (region_debug frames, drive-line
// parametric t's, perimeter polygons).
// ---------------------------------------------------------------------------

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
  if (target.on === "Perimeter") {
    const poly = perimeterLoopPolygon(lot.boundary, target.loop);
    if (!poly || poly.length < 3) return null;
    return loopArcToWorld(poly, target.arc);
  }
  return null;
}

function gridStopCoord(s: GridStop): number | null {
  return s.at === "Lattice" ? s.other : null;
}

/**
 * Return the polygon vertices of the named perimeter loop on this lot's
 * boundary. For `Outer`, returns the raw boundary outer. Hole loops come
 * from `boundary.holes[index]`.
 *
 * Note: the engine uses the INSET outer loop (by `site_offset`) as the
 * actual grid-bounding perimeter; for the prototype's rendering we accept
 * that the UI-side arc may drift by up to `site_offset` when the user
 * clicks on the raw polygon vs. the engine-resolved position. Re-anchor on
 * the next regen if needed.
 */
export function perimeterLoopPolygon(
  boundary: { outer: Vec2[]; holes: Vec2[][] },
  loop: PerimeterLoop,
): Vec2[] | null {
  if (loop.kind === "Outer") return boundary.outer;
  return boundary.holes[loop.index] ?? null;
}

/**
 * Compute the world position at a normalized arc length along a loop.
 * `arc ∈ [0, 1]` advances along the polygon in its stored winding.
 */
export function loopArcToWorld(poly: Vec2[], arc: number): Vec2 {
  const n = poly.length;
  if (n < 2) return poly[0] ?? { x: 0, y: 0 };
  const cum: number[] = [0];
  for (let i = 0; i < n; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % n];
    cum.push(cum[cum.length - 1] + Math.hypot(b.x - a.x, b.y - a.y));
  }
  const total = cum[cum.length - 1];
  if (total < 1e-9) return poly[0];
  const target = Math.max(0, Math.min(1, arc)) * total;
  for (let i = 0; i < n; i++) {
    if (cum[i + 1] >= target) {
      const segLen = cum[i + 1] - cum[i];
      const t = segLen > 1e-9 ? (target - cum[i]) / segLen : 0;
      const a = poly[i];
      const b = poly[(i + 1) % n];
      return { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) };
    }
  }
  return poly[0];
}

/**
 * Project a world-space point onto the nearest perimeter loop, within the
 * given world-space tolerance. Returns `(loop, arc)` if successful, null
 * if the point isn't on any loop. Used by the UI to address grid ×
 * perimeter crossings (and any other perimeter-adjacent vertex) via a
 * `Target::Perimeter`.
 */
export function worldToPerimeterPos(
  v: Vec2,
  boundary: { outer: Vec2[]; holes: Vec2[][] },
  tol: number = 0.5,
): { loop: PerimeterLoop; arc: number } | null {
  const candidates: { loop: PerimeterLoop; poly: Vec2[] }[] = [];
  if (boundary.outer.length >= 3) {
    candidates.push({ loop: { kind: "Outer" }, poly: boundary.outer });
  }
  for (let i = 0; i < boundary.holes.length; i++) {
    const h = boundary.holes[i];
    if (h.length >= 3) candidates.push({ loop: { kind: "Hole", index: i }, poly: h });
  }
  let best: { loop: PerimeterLoop; arc: number; dist: number } | null = null;
  for (const { loop, poly } of candidates) {
    const arc = projectOntoLoop(v, poly, tol);
    if (arc == null) continue;
    const worldAt = loopArcToWorld(poly, arc);
    const dist = Math.hypot(v.x - worldAt.x, v.y - worldAt.y);
    if (!best || dist < best.dist) best = { loop, arc, dist };
  }
  return best ? { loop: best.loop, arc: best.arc } : null;
}

function projectOntoLoop(v: Vec2, poly: Vec2[], tol: number): number | null {
  const n = poly.length;
  let totalLen = 0;
  const cum: number[] = [0];
  for (let i = 0; i < n; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % n];
    totalLen += Math.hypot(b.x - a.x, b.y - a.y);
    cum.push(totalLen);
  }
  if (totalLen < 1e-9) return null;
  let best: { arc: number; dist: number } | null = null;
  for (let i = 0; i < n; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % n];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const segLenSq = dx * dx + dy * dy;
    if (segLenSq < 1e-12) continue;
    let t = ((v.x - a.x) * dx + (v.y - a.y) * dy) / segLenSq;
    t = Math.max(0, Math.min(1, t));
    const projX = a.x + t * dx;
    const projY = a.y + t * dy;
    const dist = Math.hypot(v.x - projX, v.y - projY);
    if (dist > tol) continue;
    const arc = (cum[i] + t * (cum[i + 1] - cum[i])) / totalLen;
    if (!best || dist < best.dist) best = { arc, dist };
  }
  return best ? best.arc : null;
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
