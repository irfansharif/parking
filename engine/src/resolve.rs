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
use crate::types::{
    AbstractFrame, Annotation, Axis, DriveLine, GridStop, ParkingLayout, ParkingParams,
    PerimeterLoop, Polygon, RegionDebug, RegionId, RegionInfo, Target, Vec2,
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
    pub arc: f64,
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
        Target::Perimeter { loop_, arc } => {
            let poly = perimeter_loop_polygon(boundary, loop_)?;
            if poly.len() < 3 {
                return None;
            }
            Some(loop_arc_to_world(poly, *arc))
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
/// given world-space tolerance. Used by the UI to address grid ×
/// perimeter crossings via a `Target::Perimeter`.
pub fn world_to_perimeter_pos(
    world: Vec2,
    boundary: &Polygon,
    tol: f64,
) -> Option<PerimeterPosResult> {
    let mut candidates: Vec<(PerimeterLoop, &[Vec2])> = Vec::new();
    if boundary.outer.len() >= 3 {
        candidates.push((PerimeterLoop::Outer, boundary.outer.as_slice()));
    }
    for (i, h) in boundary.holes.iter().enumerate() {
        if h.len() >= 3 {
            candidates.push((PerimeterLoop::Hole { index: i }, h.as_slice()));
        }
    }
    let mut best: Option<(PerimeterLoop, f64, f64)> = None;
    for (loop_, poly) in candidates {
        let arc = match project_onto_loop(world, poly, tol) {
            Some(a) => a,
            None => continue,
        };
        let world_at = loop_arc_to_world(poly, arc);
        let dist = (world - world_at).length();
        if best.as_ref().map(|(_, _, d)| dist < *d).unwrap_or(true) {
            best = Some((loop_, arc, dist));
        }
    }
    best.map(|(loop_, arc, _)| PerimeterPosResult { loop_, arc })
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

// Unused-import silencer: ParkingLayout isn't referenced yet but the
// resolve module is the natural home for future functions that take a
// whole layout (e.g. hit_test_edge). Keeping the import documents
// intent without slowing the build noticeably.
#[allow(dead_code)]
fn _touch_parking_layout(_l: &ParkingLayout) {}
