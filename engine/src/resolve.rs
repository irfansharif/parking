//! Address-resolution between substrates — translate back and forth
//! between world points, abstract grid lattice coordinates, drive-line
//! splice anchors, and perimeter arc-length positions.
//!
//! Three directions:
//!
//!   forward  (substrate addr → world)  `target_world_pos`
//!   inverse  (world → substrate addr)  `world_to_{abstract_vertex,
//!                                        splice_vertex, perimeter_pos}`
//!   chain    (collinear world run →     `chain_to_abstract_lattice_edge`
//!            bounding lattice edge)     `chain_extents_in_region`
//!
//! All inputs are engine types (ts-rs-exported), so callers on either
//! side of the wasm boundary work with the same shapes. None of the
//! function bodies depend on the UI's canvas or event state.

use serde::{Deserialize, Serialize};
use ts_rs::TS;

use crate::geom::clip::point_in_polygon;
use crate::geom::poly::point_to_segment_dist;
use crate::types::{
    AbstractFrame, Annotation, Axis, DriveAisleGraph, DriveLine, EdgeArc, GridStop,
    ParkingParams, PerimeterLoop, Polygon, RegionDebug, RegionId, RegionInfo, Target, Vec2,
    VertexId,
};

// ---------------------------------------------------------------------------
// Result structs — tiny records the inverse functions return so the
// wasm boundary and TS callers see named fields rather than tuples.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct AbstractVertexResult {
    pub region: RegionId,
    pub xi: i32,
    pub yi: i32,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct SpliceVertexResult {
    pub drive_line_id: u32,
    pub t: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct PerimeterPosResult {
    #[serde(rename = "loop")]
    #[ts(rename = "loop")]
    pub loop_: PerimeterLoop,
    /// Sketch vertex at the start of the addressed edge.
    pub start: VertexId,
    /// Sketch vertex at the end of the addressed edge.
    pub end: VertexId,
    /// Position along the chord chain from `start` to `end` ∈ [0, 1].
    pub t: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct ChainLatticeEdge {
    pub region: RegionId,
    pub xa: i32,
    pub ya: i32,
    pub xb: i32,
    pub yb: i32,
}

// ---------------------------------------------------------------------------
// Forward: target → world.
// ---------------------------------------------------------------------------

/// Resolve an annotation's `Target` to its current world-space anchor
/// point. Returns `None` when the substrate (region, drive line,
/// perimeter loop) isn't present in the current generate — i.e. the
/// annotation is dormant.
pub fn target_world_pos(
    target: &Target,
    boundary: &Polygon,
    drive_lines: &[DriveLine],
    region_debug: Option<&RegionDebug>,
    params: &ParkingParams,
) -> Option<Vec2> {
    match target {
        Target::DriveLine { id, t } => {
            let dl = drive_lines.iter().find(|d| d.id == *id)?;
            Some(Vec2::new(
                dl.start.x + (dl.end.x - dl.start.x) * t,
                dl.start.y + (dl.end.y - dl.start.y) * t,
            ))
        }
        Target::Grid {
            region,
            axis,
            coord,
            range,
        } => {
            let rd = region_debug?;
            let ri = rd.regions.iter().find(|r| r.id == *region)?;
            let frame = AbstractFrame::region(params, ri.aisle_angle_deg, ri.aisle_offset);
            let abs = match range {
                None => match axis {
                    // Whole line — anchor at (coord, 0) on the fixed axis.
                    Axis::X => crate::types::AbstractPoint2 {
                        x: *coord as f64,
                        y: 0.0,
                    },
                    Axis::Y => crate::types::AbstractPoint2 {
                        x: 0.0,
                        y: *coord as f64,
                    },
                },
                Some((s1, s2)) => {
                    let c1 = grid_stop_coord(s1)?;
                    let c2 = grid_stop_coord(s2)?;
                    let mid = (c1 + c2) as f64 * 0.5;
                    match axis {
                        Axis::X => crate::types::AbstractPoint2 {
                            x: *coord as f64,
                            y: mid,
                        },
                        Axis::Y => crate::types::AbstractPoint2 {
                            x: mid,
                            y: *coord as f64,
                        },
                    }
                }
            };
            Some(frame.forward(abs))
        }
        Target::Perimeter { loop_, start, end, t } => {
            let poly = perimeter_loop_polygon(boundary, loop_)?;
            let ids = perimeter_loop_ids(boundary, loop_)?;
            if poly.len() < 3 {
                return None;
            }
            // If the sketch edge `(start, end)` has a bulge, evaluate
            // the world position along the *arc* (not the chord chain)
            // so the annotation marker follows the curve as the bulge
            // changes. Falls back to chord-chain lerp for straight
            // edges and discretized inputs (where the loop has many
            // synthetic intermediates).
            if let Some(pos) = evaluate_perim_edge_arc(boundary, loop_, *start, *end, *t) {
                return Some(pos);
            }
            evaluate_perim_edge(poly, ids, *start, *end, *t)
        }
    }
}

/// Convenience wrapper: `target_world_pos(ann.target, ...)`.
pub fn annotation_world_pos(
    ann: &Annotation,
    boundary: &Polygon,
    drive_lines: &[DriveLine],
    region_debug: Option<&RegionDebug>,
    params: &ParkingParams,
) -> Option<Vec2> {
    let target = match ann {
        Annotation::DeleteVertex { target } => target,
        Annotation::DeleteEdge { target } => target,
        Annotation::Direction { target, .. } => target,
    };
    target_world_pos(target, boundary, drive_lines, region_debug, params)
}

fn grid_stop_coord(s: &GridStop) -> Option<i32> {
    match s {
        GridStop::Lattice { other } => Some(*other),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Inverse: world → substrate address.
// ---------------------------------------------------------------------------

/// Convert a world position into a `(region, xi, yi)` triple naming a
/// grid-lattice intersection. Returns `None` if the point lies outside
/// every region's clip polygon, or the nearest integer lattice point is
/// farther than `snap_tol_world` from the query in world units.
pub fn world_to_abstract_vertex(
    world: Vec2,
    params: &ParkingParams,
    region_debug: Option<&RegionDebug>,
    snap_tol_world: f64,
) -> Option<AbstractVertexResult> {
    let rd = region_debug?;
    for region in &rd.regions {
        if !point_in_polygon(&world, &region.clip_poly) {
            continue;
        }
        let frame = AbstractFrame::region(params, region.aisle_angle_deg, region.aisle_offset);
        let abs = frame.inverse(world);
        let xi = abs.x.round() as i32;
        let yi = abs.y.round() as i32;
        let dx = (abs.x - xi as f64).abs() * frame.dx;
        let dy = (abs.y - yi as f64).abs() * frame.dy;
        if dx > snap_tol_world || dy > snap_tol_world {
            continue;
        }
        return Some(AbstractVertexResult {
            region: region.id,
            xi,
            yi,
        });
    }
    None
}

/// Project a world point onto the closest drive line and return its
/// stable splice anchor `(drive_line_id, t)`. `t` may fall outside
/// `[0, 1]` when the splice extends past the user-drawn endpoints.
pub fn world_to_splice_vertex(
    world: Vec2,
    drive_lines: &[DriveLine],
    snap_tol_world: f64,
) -> Option<SpliceVertexResult> {
    let mut best: Option<(u32, f64, f64)> = None;
    for dl in drive_lines {
        let d = dl.end - dl.start;
        let len2 = d.dot(d);
        if len2 < 1e-12 {
            continue;
        }
        let t_raw = (world - dl.start).dot(d) / len2;
        let p = dl.start + d * t_raw;
        let dist = (world - p).length();
        if dist > snap_tol_world {
            continue;
        }
        if best.map(|(_, _, bd)| dist < bd).unwrap_or(true) {
            best = Some((dl.id, t_raw, dist));
        }
    }
    best.map(|(id, t, _)| SpliceVertexResult {
        drive_line_id: id,
        t,
    })
}

/// Project a world point onto the nearest perimeter loop, within the
/// given world-space tolerance. Returns the addressed sketch edge
/// `(start, end, t)` for the closest projection. Uses arc-aware
/// projection (`compute_boundary_pin`), so clicks on a discretized
/// arc resolve to the correct sketch edge even though the engine's
/// graph stores chord-interior synthetic vertices.
pub fn world_to_perimeter_pos(
    world: Vec2,
    boundary: &Polygon,
    tol: f64,
) -> Option<PerimeterPosResult> {
    let mut best: Option<(PerimeterLoop, VertexId, VertexId, f64, f64)> = None;

    let mut consider = |loop_id: PerimeterLoop,
                        poly: &[Vec2],
                        ids: &[VertexId],
                        arcs: Option<&[Option<EdgeArc>]>| {
        let n = poly.len();
        if n < 3 || ids.len() != n {
            return;
        }
        let (proj, edge_idx, t) =
            crate::geom::arc::compute_boundary_pin(world, poly, arcs);
        let dist = (world - proj).length();
        if dist > tol {
            return;
        }
        let start = ids[edge_idx];
        let end = ids[(edge_idx + 1) % n];
        if best.as_ref().map(|(_, _, _, _, d)| dist < *d).unwrap_or(true) {
            best = Some((loop_id, start, end, t, dist));
        }
    };

    consider(
        PerimeterLoop::Outer,
        &boundary.outer,
        &boundary.outer_ids,
        if boundary.outer_arcs.is_empty() { None } else { Some(&boundary.outer_arcs) },
    );
    for (i, h) in boundary.holes.iter().enumerate() {
        if let Some(ids) = boundary.hole_ids.get(i) {
            let arcs = boundary
                .hole_arcs
                .get(i)
                .filter(|v| !v.is_empty())
                .map(|v| v.as_slice());
            consider(PerimeterLoop::Hole { index: i }, h, ids, arcs);
        }
    }

    best.map(|(loop_, start, end, t, _)| PerimeterPosResult { loop_, start, end, t })
}

// ---------------------------------------------------------------------------
// Chain → lattice-edge extents. The strict variant rejects chains that
// aren't axis-aligned in the containing region's frame; the lenient
// variant trusts the caller on region choice and just floors/ceils
// along the varying axis.
// ---------------------------------------------------------------------------

/// Strict: find the region whose clip polygon contains the chain's
/// midpoint, project chain points into its frame, and emit lattice
/// extents — rejecting when the chain isn't grid-axis-aligned (snap
/// tolerance on the constant axis).
pub fn chain_to_abstract_lattice_edge(
    chain_points: &[Vec2],
    params: &ParkingParams,
    region_debug: Option<&RegionDebug>,
) -> Option<ChainLatticeEdge> {
    let rd = region_debug?;
    if chain_points.len() < 2 {
        return None;
    }
    let mid = Vec2::new(
        chain_points.iter().map(|p| p.x).sum::<f64>() / chain_points.len() as f64,
        chain_points.iter().map(|p| p.y).sum::<f64>() / chain_points.len() as f64,
    );
    for region in &rd.regions {
        if !point_in_polygon(&mid, &region.clip_poly) {
            continue;
        }
        let frame = AbstractFrame::region(params, region.aisle_angle_deg, region.aisle_offset);
        let abs: Vec<_> = chain_points.iter().map(|p| frame.inverse(*p)).collect();
        let (xmin, xmax) = minmax(abs.iter().map(|p| p.x));
        let (ymin, ymax) = minmax(abs.iter().map(|p| p.y));
        let x_range = xmax - xmin;
        let y_range = ymax - ymin;
        // Reject chains that aren't axis-aligned: the "constant" axis
        // must lie within snap tolerance of an integer grid line,
        // else we'd silently project onto the nearest lattice edge.
        let snap_tol = 0.5;
        if x_range > y_range {
            let yi = (abs.iter().map(|p| p.y).sum::<f64>() / abs.len() as f64).round() as i32;
            let y_dev = abs
                .iter()
                .map(|p| (p.y - yi as f64).abs())
                .fold(0.0_f64, f64::max)
                * frame.dy;
            if y_dev > snap_tol {
                return None;
            }
            let xa = xmin.floor() as i32;
            let xb = xmax.ceil() as i32;
            if xa == xb {
                return None;
            }
            return Some(ChainLatticeEdge {
                region: region.id,
                xa,
                ya: yi,
                xb,
                yb: yi,
            });
        }
        let xi = (abs.iter().map(|p| p.x).sum::<f64>() / abs.len() as f64).round() as i32;
        let x_dev = abs
            .iter()
            .map(|p| (p.x - xi as f64).abs())
            .fold(0.0_f64, f64::max)
            * frame.dx;
        if x_dev > snap_tol {
            return None;
        }
        let ya = ymin.floor() as i32;
        let yb = ymax.ceil() as i32;
        if ya == yb {
            return None;
        }
        return Some(ChainLatticeEdge {
            region: region.id,
            xa: xi,
            ya,
            xb: xi,
            yb,
        });
    }
    None
}

/// Lenient: project chain into the passed-in region's frame without
/// the tolerance rejection. Used for chain-mode annotations where the
/// caller already chose the region from the seed sub-edge.
pub fn chain_extents_in_region(
    chain_points: &[Vec2],
    params: &ParkingParams,
    region: &RegionInfo,
) -> Option<ChainLatticeEdge> {
    if chain_points.len() < 2 {
        return None;
    }
    let frame = AbstractFrame::region(params, region.aisle_angle_deg, region.aisle_offset);
    let abs: Vec<_> = chain_points.iter().map(|p| frame.inverse(*p)).collect();
    let (xmin, xmax) = minmax(abs.iter().map(|p| p.x));
    let (ymin, ymax) = minmax(abs.iter().map(|p| p.y));
    let x_range = xmax - xmin;
    let y_range = ymax - ymin;
    if x_range > y_range {
        let yi = (abs.iter().map(|p| p.y).sum::<f64>() / abs.len() as f64).round() as i32;
        let xa = xmin.floor() as i32;
        let xb = xmax.ceil() as i32;
        if xa == xb {
            return None;
        }
        return Some(ChainLatticeEdge {
            region: region.id,
            xa,
            ya: yi,
            xb,
            yb: yi,
        });
    }
    let xi = (abs.iter().map(|p| p.x).sum::<f64>() / abs.len() as f64).round() as i32;
    let ya = ymin.floor() as i32;
    let yb = ymax.ceil() as i32;
    if ya == yb {
        return None;
    }
    Some(ChainLatticeEdge {
        region: region.id,
        xa: xi,
        ya,
        xb: xi,
        yb,
    })
}

// ---------------------------------------------------------------------------
// Perimeter arc-length helpers. Not exposed via wasm directly (the UI
// calls world_to_perimeter_pos / target_world_pos instead), but shared
// between the resolve functions above.
// ---------------------------------------------------------------------------

/// Fetch the vertex ring for a named perimeter loop. The engine uses
/// the inset outer loop for grid bounding; callers of the prototype
/// accept that UI-side rendering may drift by up to `site_offset` on
/// the outer loop (the next regen re-anchors).
pub fn perimeter_loop_polygon<'a>(
    boundary: &'a Polygon,
    loop_: &PerimeterLoop,
) -> Option<&'a [Vec2]> {
    match loop_ {
        PerimeterLoop::Outer => Some(&boundary.outer),
        PerimeterLoop::Hole { index } => boundary.holes.get(*index).map(|h| h.as_slice()),
    }
}

/// Fetch the per-vertex `VertexId` array parallel to the loop's
/// vertices. Returns `None` when the polygon doesn't have ids
/// populated (e.g. test fixtures that bypassed `ensure_ids`).
pub fn perimeter_loop_ids<'a>(
    boundary: &'a Polygon,
    loop_: &PerimeterLoop,
) -> Option<&'a [VertexId]> {
    let (poly, ids) = match loop_ {
        PerimeterLoop::Outer => (boundary.outer.as_slice(), boundary.outer_ids.as_slice()),
        PerimeterLoop::Hole { index } => {
            let h = boundary.holes.get(*index)?;
            let i = boundary.hole_ids.get(*index).map(|v| v.as_slice()).unwrap_or(&[]);
            (h.as_slice(), i)
        }
    };
    if ids.len() == poly.len() && !ids.is_empty() {
        Some(ids)
    } else {
        None
    }
}

/// Walk the loop forward from the chord-chain corresponding to the
/// sketch edge `start → end`, emitting `(a, b, chord_len)` triples
/// for each chord in the chain plus the running total length.
///
/// Returns `None` when:
///   - either id is missing from the loop (sketch vertex deleted),
///   - the polygon is malformed,
///   - or any *non-synthetic* id appears strictly between `start`
///     and `end` on the loop. That case means a real sketch vertex
///     has been inserted on the edge (the user split it), so the
///     annotation's original `(start, end)` no longer names a single
///     sketch edge — caller treats this as dormant. Synthetic ids
///     (chord-interior samples produced by arc discretization) are
///     allowed and walked over without flagging.
fn perimeter_edge_chords(
    poly: &[Vec2],
    ids: &[VertexId],
    start: VertexId,
    end: VertexId,
) -> Option<(Vec<(Vec2, Vec2, f64)>, f64, usize, usize)> {
    let n = ids.len();
    if n < 2 || poly.len() != n {
        return None;
    }
    let is = ids.iter().position(|v| *v == start)?;
    let ie = ids.iter().position(|v| *v == end)?;
    if is == ie {
        return None;
    }
    let mut chords = Vec::new();
    let mut total = 0.0;
    let mut k = is;
    let mut steps = 0usize;
    while k != ie {
        let knext = (k + 1) % n;
        if knext != ie && !ids[knext].is_synthetic() {
            // Sketch vertex inserted between `start` and `end` — the
            // edge has been split. Annotation goes dormant.
            return None;
        }
        let a = poly[k];
        let b = poly[knext];
        let len = (b - a).length();
        chords.push((a, b, len));
        total += len;
        k = knext;
        steps += 1;
        if steps > n {
            return None; // malformed: walked past `end`
        }
    }
    Some((chords, total, is, ie))
}

/// Evaluate position along the sketch edge `(start → end)` using its
/// stored arc data. Returns `Some(pos)` only when the loop carries
/// per-edge arc info (i.e. it's the un-discretized sketch); the
/// position lies on the actual circular arc (or the chord for
/// straight edges). Returns `None` if the edge isn't found, the loop
/// lacks arc data, or `start`/`end` aren't consecutive (which means
/// the input is the post-discretization loop — caller falls back to
/// chord-chain interpolation).
pub fn evaluate_perim_edge_arc(
    boundary: &Polygon,
    loop_: &PerimeterLoop,
    start: VertexId,
    end: VertexId,
    t: f64,
) -> Option<Vec2> {
    let (poly, ids, arcs) = match loop_ {
        PerimeterLoop::Outer => (
            boundary.outer.as_slice(),
            boundary.outer_ids.as_slice(),
            boundary.outer_arcs.as_slice(),
        ),
        PerimeterLoop::Hole { index } => {
            let h = boundary.holes.get(*index)?;
            let i = boundary.hole_ids.get(*index).map(|v| v.as_slice()).unwrap_or(&[]);
            let a = boundary.hole_arcs.get(*index).map(|v| v.as_slice()).unwrap_or(&[]);
            (h.as_slice(), i, a)
        }
    };
    let n = poly.len();
    if n < 2 || ids.len() != n {
        return None;
    }
    let is = ids.iter().position(|v| *v == start)?;
    let ie = ids.iter().position(|v| *v == end)?;
    // Sketch edges are consecutive vertices. If `(start, end)` aren't
    // adjacent in the loop, this isn't a sketch edge — return None
    // so the caller falls back to chord-chain interpolation.
    if (is + 1) % n != ie {
        return None;
    }
    let p0 = poly[is];
    let p1 = poly[ie];
    let bulge = arcs.get(is).and_then(|a| a.as_ref()).map(|a| a.bulge).unwrap_or(0.0);
    Some(crate::geom::arc::eval_arc_at(p0, p1, bulge, t))
}

/// Sample the world position at parameter `t ∈ [0, 1]` along the
/// chord chain that represents the sketch edge `start → end` on the
/// loop. For an un-discretized straight edge this is a single linear
/// interpolation; for a discretized arc edge it walks the chord
/// polyline by length.
pub fn evaluate_perim_edge(
    poly: &[Vec2],
    ids: &[VertexId],
    start: VertexId,
    end: VertexId,
    t: f64,
) -> Option<Vec2> {
    let (chords, total, is, ie) = perimeter_edge_chords(poly, ids, start, end)?;
    if total < 1e-12 {
        return Some(poly[is]);
    }
    let target = t.clamp(0.0, 1.0) * total;
    let mut acc = 0.0;
    for (a, b, len) in &chords {
        if acc + *len >= target {
            let local = if *len > 1e-12 { (target - acc) / *len } else { 0.0 };
            return Some(*a + (*b - *a) * local);
        }
        acc += *len;
    }
    Some(poly[ie])
}

/// Sample the world position at a normalized arc length `arc ∈ [0, 1]`
/// along the loop's stored winding.
pub fn loop_arc_to_world(poly: &[Vec2], arc: f64) -> Vec2 {
    let n = poly.len();
    if n < 2 {
        return *poly.first().unwrap_or(&Vec2::new(0.0, 0.0));
    }
    let mut cum = Vec::with_capacity(n + 1);
    cum.push(0.0_f64);
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let len = (b - a).length();
        cum.push(cum[i] + len);
    }
    let total = *cum.last().unwrap();
    if total < 1e-9 {
        return poly[0];
    }
    let target = arc.clamp(0.0, 1.0) * total;
    for i in 0..n {
        if cum[i + 1] >= target {
            let seg_len = cum[i + 1] - cum[i];
            let t = if seg_len > 1e-9 {
                (target - cum[i]) / seg_len
            } else {
                0.0
            };
            let a = poly[i];
            let b = poly[(i + 1) % n];
            return a + (b - a) * t;
        }
    }
    poly[0]
}

/// Project a world point onto the nearest point of a loop (treated as
/// a closed polygon); returns the normalized arc length if the nearest
/// point is within `tol`.
pub fn project_onto_loop(v: Vec2, poly: &[Vec2], tol: f64) -> Option<f64> {
    let n = poly.len();
    let mut cum = Vec::with_capacity(n + 1);
    cum.push(0.0_f64);
    let mut total = 0.0;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let len = (b - a).length();
        total += len;
        cum.push(total);
    }
    if total < 1e-9 {
        return None;
    }
    let mut best: Option<(f64, f64)> = None;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let seg = b - a;
        let seg_len_sq = seg.dot(seg);
        if seg_len_sq < 1e-12 {
            continue;
        }
        let t = ((v - a).dot(seg) / seg_len_sq).clamp(0.0, 1.0);
        let proj = a + seg * t;
        let dist = (v - proj).length();
        if dist > tol {
            continue;
        }
        let arc = (cum[i] + t * (cum[i + 1] - cum[i])) / total;
        if best.map(|(_, d)| dist < d).unwrap_or(true) {
            best = Some((arc, dist));
        }
    }
    best.map(|(a, _)| a)
}

fn minmax<I: Iterator<Item = f64>>(iter: I) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for v in iter {
        if v < min {
            min = v;
        }
        if v > max {
            max = v;
        }
    }
    (min, max)
}

// ---------------------------------------------------------------------------
// Aisle-graph edge hit-testing and collinear-chain expansion. Both
// operate on the deduplicated edge set — i.e. the canonical edge for
// each unordered `(u, v)` pair, not the two directed halves serde
// emits. A click picks one canonical edge; the chain walks outward
// from it along collinear neighbours.
// ---------------------------------------------------------------------------

/// Index of the canonical (lower-id-first) deduplicated edge nearest
/// to `world` that lies within `world_radius`. Returns `None` if no
/// edge is within radius or the graph is empty.
pub fn hit_test_edge(world: Vec2, graph: &DriveAisleGraph, world_radius: f64) -> Option<usize> {
    let mut seen: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
    let mut best: Option<(usize, f64)> = None;
    for (i, edge) in graph.edges.iter().enumerate() {
        let key = (edge.start.min(edge.end), edge.start.max(edge.end));
        if !seen.insert(key) {
            continue;
        }
        let a = graph.vertices[edge.start];
        let b = graph.vertices[edge.end];
        let dist = point_to_segment_dist(world, a, b);
        if dist < world_radius && best.map(|(_, bd)| dist < bd).unwrap_or(true) {
            best = Some((i, dist));
        }
    }
    best.map(|(i, _)| i)
}

/// All deduplicated edge indices that lie on the same collinear run as
/// `seed_idx`. Starts at the seed and BFS-expands through vertices
/// whose incident edges share the seed's direction (dot ≥ 0.99).
pub fn find_collinear_chain(graph: &DriveAisleGraph, seed_idx: usize) -> Vec<usize> {
    let seed = match graph.edges.get(seed_idx) {
        Some(e) => e,
        None => return vec![],
    };
    let s = graph.vertices[seed.start];
    let e = graph.vertices[seed.end];
    let seed_d = e - s;
    let seed_len = seed_d.length();
    if seed_len < 1e-9 {
        return vec![seed_idx];
    }
    let seed_dir = Vec2::new(seed_d.x / seed_len, seed_d.y / seed_len);

    // Adjacency over deduplicated edges.
    let mut adj: std::collections::HashMap<usize, Vec<(usize, usize, Vec2)>> =
        std::collections::HashMap::new();
    let mut seen: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
    for (i, edge) in graph.edges.iter().enumerate() {
        let key = (edge.start.min(edge.end), edge.start.max(edge.end));
        if !seen.insert(key) {
            continue;
        }
        let a = graph.vertices[edge.start];
        let b = graph.vertices[edge.end];
        let d = b - a;
        let len = d.length();
        if len < 1e-9 {
            continue;
        }
        let n = Vec2::new(d.x / len, d.y / len);
        adj.entry(edge.start).or_default().push((i, edge.end, n));
        adj.entry(edge.end).or_default().push((i, edge.start, n * -1.0));
    }

    let mut chain = vec![seed_idx];
    let mut visited: std::collections::HashSet<usize> = std::collections::HashSet::new();
    visited.insert(seed_idx);
    let mut frontier = vec![seed.start, seed.end];
    while let Some(v) = frontier.pop() {
        if let Some(neighbours) = adj.get(&v) {
            for (ei, other, dir) in neighbours.iter() {
                if visited.contains(ei) {
                    continue;
                }
                if (dir.dot(seed_dir)).abs() < 0.99 {
                    continue;
                }
                visited.insert(*ei);
                chain.push(*ei);
                frontier.push(*other);
            }
        }
    }
    chain
}

// ---------------------------------------------------------------------------
// Chain → grid target. Composes `world_to_abstract_vertex` on the seed
// edge's endpoints with `chain_extents_in_region` / `chain_to_abstract_
// lattice_edge` to produce the `Target::Grid` a chain-mode UI
// selection deletes or cycles direction on.
//
// Two paths:
//
//   1. Seed endpoints snap to lattice junctions in the same region.
//      Scope widens to the chain's extents in that region (lenient —
//      chain crosses region boundaries are silently clamped to the
//      seed's region instead of rejecting).
//
//   2. Seed is an interior break (endpoints off-grid — e.g. a
//      drive-line splice). Fall back to the chain's outer junctions
//      via the strict `chain_to_abstract_lattice_edge`; reject if the
//      chain isn't axis-aligned.
//
// Returns `None` when neither path yields a grid-addressable scope —
// the caller then falls through to a drive-line splice target.
// ---------------------------------------------------------------------------

/// Build a grid `Target` from a lattice scope. Single-point scope
/// maps to a `range = (s, s)` vertex on the X axis by convention;
/// varying-axis scopes set `axis` accordingly.
pub fn grid_target_from_scope(scope: ChainLatticeEdge) -> Option<Target> {
    if scope.xa == scope.xb && scope.ya == scope.yb {
        // Single lattice point — address on the X axis by convention.
        let stop = GridStop::Lattice { other: scope.ya };
        return Some(Target::Grid {
            region: scope.region,
            axis: Axis::X,
            coord: scope.xa,
            range: Some((stop.clone(), stop)),
        });
    }
    if scope.xa == scope.xb {
        // Varying along y, x fixed.
        let s1 = GridStop::Lattice { other: scope.ya };
        let s2 = GridStop::Lattice { other: scope.yb };
        return Some(Target::Grid {
            region: scope.region,
            axis: Axis::X,
            coord: scope.xa,
            range: Some((s1, s2)),
        });
    }
    if scope.ya == scope.yb {
        // Varying along x, y fixed.
        let s1 = GridStop::Lattice { other: scope.xa };
        let s2 = GridStop::Lattice { other: scope.xb };
        return Some(Target::Grid {
            region: scope.region,
            axis: Axis::Y,
            coord: scope.ya,
            range: Some((s1, s2)),
        });
    }
    None
}

/// Resolve an edge-chain selection (seed edge + collinear run) to a
/// `Target::Grid`. See module-level comment for the two paths. A
/// selection with `chain.len() <= 1` or explicit segment mode widens
/// only to the seed sub-edge.
pub fn resolve_grid_target(
    seed_index: usize,
    chain: &[usize],
    is_chain_mode: bool,
    graph: &DriveAisleGraph,
    params: &ParkingParams,
    region_debug: &RegionDebug,
) -> Option<Target> {
    let seed = graph.edges.get(seed_index)?;
    let s = graph.vertices[seed.start];
    let e = graph.vertices[seed.end];

    let abs_a = world_to_abstract_vertex(s, params, Some(region_debug), 0.5);
    let abs_b = world_to_abstract_vertex(e, params, Some(region_debug), 0.5);

    let scope = match (abs_a, abs_b) {
        (Some(a), Some(b)) if a.region == b.region => {
            // Seed on-grid. Default to the seed's own extents, then
            // widen to the full chain within the same region.
            let seed_scope = ChainLatticeEdge {
                region: a.region,
                xa: a.xi,
                ya: a.yi,
                xb: b.xi,
                yb: b.yi,
            };
            if is_chain_mode && chain.len() > 1 {
                let region_info = region_debug.regions.iter().find(|r| r.id == a.region)?;
                let chain_pts = chain_world_points(chain, graph);
                chain_extents_in_region(&chain_pts, params, region_info).unwrap_or(seed_scope)
            } else {
                seed_scope
            }
        }
        _ => {
            // Seed off-grid — fall back to chain junctions via the
            // strict projection.
            let chain_pts = chain_world_points(chain, graph);
            chain_to_abstract_lattice_edge(&chain_pts, params, Some(region_debug))?
        }
    };
    grid_target_from_scope(scope)
}

fn chain_world_points(chain: &[usize], graph: &DriveAisleGraph) -> Vec<Vec2> {
    let mut pts = Vec::with_capacity(chain.len() * 2);
    for ei in chain {
        if let Some(e) = graph.edges.get(*ei) {
            pts.push(graph.vertices[e.start]);
            pts.push(graph.vertices[e.end]);
        }
    }
    pts
}

/// Structural equality of two targets. Compares fields — ignores
/// stop-order (`(s1, s2)` vs `(s2, s1)` match) for grid ranges and
/// uses a small epsilon on parametric `t` / `arc` so floating-point
/// round-trips through JSON stay equal.
pub fn targets_equal(a: &Target, b: &Target) -> bool {
    match (a, b) {
        (
            Target::Grid {
                region: ra,
                axis: aa,
                coord: ca,
                range: rna,
            },
            Target::Grid {
                region: rb,
                axis: ab,
                coord: cb,
                range: rnb,
            },
        ) => {
            if ra != rb || aa != ab || ca != cb {
                return false;
            }
            match (rna, rnb) {
                (None, None) => true,
                (Some((a0, a1)), Some((b0, b1))) => {
                    (grid_stops_equal(a0, b0) && grid_stops_equal(a1, b1))
                        || (grid_stops_equal(a0, b1) && grid_stops_equal(a1, b0))
                }
                _ => false,
            }
        }
        (Target::DriveLine { id: ai, t: at }, Target::DriveLine { id: bi, t: bt }) => {
            ai == bi && (at - bt).abs() < 1e-3
        }
        (
            Target::Perimeter {
                loop_: al,
                start: as_,
                end: ae,
                t: at,
            },
            Target::Perimeter {
                loop_: bl,
                start: bs,
                end: be,
                t: bt,
            },
        ) => loops_equal(al, bl) && as_ == bs && ae == be && (at - bt).abs() < 1e-3,
        _ => false,
    }
}

fn grid_stops_equal(a: &GridStop, b: &GridStop) -> bool {
    match (a, b) {
        (GridStop::Lattice { other: x }, GridStop::Lattice { other: y }) => x == y,
        (GridStop::CrossesDriveLine { id: x }, GridStop::CrossesDriveLine { id: y }) => x == y,
        (GridStop::CrossesPerimeter { loop_: x }, GridStop::CrossesPerimeter { loop_: y }) => {
            loops_equal(x, y)
        }
        _ => false,
    }
}

fn loops_equal(a: &PerimeterLoop, b: &PerimeterLoop) -> bool {
    match (a, b) {
        (PerimeterLoop::Outer, PerimeterLoop::Outer) => true,
        (PerimeterLoop::Hole { index: x }, PerimeterLoop::Hole { index: y }) => x == y,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{AisleEdge, AisleDirection};

    // Build a minimal 2x2 aisle graph: 4 outer vertices at unit coords,
    // plus the four edges wrapping them. Canonical edge indices
    // 0:(0→1), 1:(1→2), 2:(2→3), 3:(3→0) after dedup.
    fn square_graph() -> DriveAisleGraph {
        DriveAisleGraph {
            vertices: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            edges: vec![
                AisleEdge { start: 0, end: 1, width: 1.0, interior: false, direction: AisleDirection::TwoWay },
                AisleEdge { start: 1, end: 2, width: 1.0, interior: false, direction: AisleDirection::TwoWay },
                AisleEdge { start: 2, end: 3, width: 1.0, interior: false, direction: AisleDirection::TwoWay },
                AisleEdge { start: 3, end: 0, width: 1.0, interior: false, direction: AisleDirection::TwoWay },
            ],
            perim_vertex_count: 4,
        }
    }

    #[test]
    fn hit_test_edge_picks_nearest() {
        let g = square_graph();
        // (5, 0.1) sits on the bottom edge (edge 0).
        assert_eq!(hit_test_edge(Vec2::new(5.0, 0.1), &g, 1.0), Some(0));
        // (10.1, 5) on the right edge (edge 1).
        assert_eq!(hit_test_edge(Vec2::new(10.1, 5.0), &g, 1.0), Some(1));
        // (5, 5) inside the square, outside any edge's radius — None.
        assert_eq!(hit_test_edge(Vec2::new(5.0, 5.0), &g, 0.5), None);
    }

    #[test]
    fn find_collinear_chain_collinear_run() {
        // Three colinear segments along y=0: (0,0)-(5,0), (5,0)-(10,0),
        // (10,0)-(15,0). Chain from any seed should return all three.
        let g = DriveAisleGraph {
            vertices: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(5.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(15.0, 0.0),
            ],
            edges: vec![
                AisleEdge { start: 0, end: 1, width: 1.0, interior: false, direction: AisleDirection::TwoWay },
                AisleEdge { start: 1, end: 2, width: 1.0, interior: false, direction: AisleDirection::TwoWay },
                AisleEdge { start: 2, end: 3, width: 1.0, interior: false, direction: AisleDirection::TwoWay },
            ],
            perim_vertex_count: 0,
        };
        let chain = find_collinear_chain(&g, 1);
        assert_eq!(chain.len(), 3);
        // All three deduped indices covered.
        let mut sorted = chain.clone();
        sorted.sort();
        assert_eq!(sorted, vec![0, 1, 2]);
    }

    #[test]
    fn find_collinear_chain_stops_at_corner() {
        let g = square_graph();
        // Seeded on bottom edge (0→1). Right edge (1→2) is
        // perpendicular — shouldn't be in the chain.
        let chain = find_collinear_chain(&g, 0);
        assert_eq!(chain, vec![0]);
    }

    #[test]
    fn loop_arc_to_world_at_endpoints() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        // arc=0 is the first vertex.
        let p0 = loop_arc_to_world(&poly, 0.0);
        assert!((p0 - poly[0]).length() < 1e-9);
        // arc=0.25 on a 40-unit-perimeter square is at (10, 0) — the
        // next vertex.
        let p = loop_arc_to_world(&poly, 0.25);
        assert!((p - poly[1]).length() < 1e-9);
        // arc=0.5 is halfway around — (10, 10).
        let p = loop_arc_to_world(&poly, 0.5);
        assert!((p - poly[2]).length() < 1e-9);
    }

    #[test]
    fn world_to_perimeter_pos_projects_onto_loop() {
        let mut boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            holes: vec![],
            ..Default::default()
        };
        boundary.ensure_ids();
        // Point just above the bottom edge projects onto the bottom edge.
        let r = world_to_perimeter_pos(Vec2::new(5.0, 0.3), &boundary, 1.0).unwrap();
        assert!(matches!(r.loop_, PerimeterLoop::Outer));
        // Bottom edge is outer_ids[0] → outer_ids[1]; t = 0.5 (midpoint).
        assert_eq!(r.start, boundary.outer_ids[0]);
        assert_eq!(r.end, boundary.outer_ids[1]);
        assert!((r.t - 0.5).abs() < 1e-9);
        // Point far from any edge — None within tol=1.
        assert!(world_to_perimeter_pos(Vec2::new(5.0, 5.0), &boundary, 1.0).is_none());
    }

    #[test]
    fn grid_target_single_point() {
        let t = grid_target_from_scope(ChainLatticeEdge {
            region: RegionId::single_region_fallback(),
            xa: 2,
            ya: 3,
            xb: 2,
            yb: 3,
        })
        .unwrap();
        match t {
            Target::Grid { axis, coord, range, .. } => {
                assert_eq!(axis, Axis::X);
                assert_eq!(coord, 2);
                let (s1, s2) = range.unwrap();
                assert!(matches!(s1, GridStop::Lattice { other: 3 }));
                assert!(matches!(s2, GridStop::Lattice { other: 3 }));
            }
            _ => panic!("expected Target::Grid"),
        }
    }

    #[test]
    fn grid_target_span_varies_along_y() {
        // xa == xb, ya != yb — line fixed at x=5, spanning y=0..3.
        let t = grid_target_from_scope(ChainLatticeEdge {
            region: RegionId::single_region_fallback(),
            xa: 5,
            ya: 0,
            xb: 5,
            yb: 3,
        })
        .unwrap();
        match t {
            Target::Grid { axis, coord, range, .. } => {
                assert_eq!(axis, Axis::X);
                assert_eq!(coord, 5);
                let (s1, s2) = range.unwrap();
                assert!(matches!(s1, GridStop::Lattice { other: 0 }));
                assert!(matches!(s2, GridStop::Lattice { other: 3 }));
            }
            _ => panic!("expected Target::Grid"),
        }
    }

    #[test]
    fn targets_equal_ignores_stop_order() {
        let a = Target::Grid {
            region: RegionId::single_region_fallback(),
            axis: Axis::X,
            coord: 2,
            range: Some((
                GridStop::Lattice { other: 0 },
                GridStop::Lattice { other: 6 },
            )),
        };
        let b = Target::Grid {
            region: RegionId::single_region_fallback(),
            axis: Axis::X,
            coord: 2,
            range: Some((
                GridStop::Lattice { other: 6 },
                GridStop::Lattice { other: 0 },
            )),
        };
        assert!(targets_equal(&a, &b));
    }

    #[test]
    fn targets_equal_rejects_different_coord() {
        let a = Target::Grid {
            region: RegionId::single_region_fallback(),
            axis: Axis::X,
            coord: 2,
            range: None,
        };
        let b = Target::Grid {
            region: RegionId::single_region_fallback(),
            axis: Axis::X,
            coord: 3,
            range: None,
        };
        assert!(!targets_equal(&a, &b));
    }

    #[test]
    fn targets_equal_drive_line_epsilon() {
        let a = Target::DriveLine { id: 1, t: 0.5 };
        let b = Target::DriveLine { id: 1, t: 0.5005 };
        assert!(targets_equal(&a, &b)); // within 1e-3
        let c = Target::DriveLine { id: 1, t: 0.52 };
        assert!(!targets_equal(&a, &c)); // outside 1e-3
    }

    #[test]
    fn world_to_splice_vertex_projects_onto_drive_line() {
        let dls = vec![DriveLine {
            start: Vec2::new(0.0, 0.0),
            end: Vec2::new(10.0, 0.0),
            hole_pin: None,
            id: 7,
            partitions: false,
        }];
        let r = world_to_splice_vertex(Vec2::new(5.0, 0.1), &dls, 1.0).unwrap();
        assert_eq!(r.drive_line_id, 7);
        assert!((r.t - 0.5).abs() < 1e-9);
        // Outside tolerance.
        assert!(world_to_splice_vertex(Vec2::new(5.0, 5.0), &dls, 1.0).is_none());
    }
}
