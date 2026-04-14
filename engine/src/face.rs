use crate::aisle_graph::{compute_inset_d, derive_raw_holes, derive_raw_outer};
use crate::clip::{clip_stalls_to_boundary, remove_conflicting_stalls};
use crate::inset::signed_area;
use crate::segment::fill_strip;
use crate::skeleton;
use crate::types::*;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

/// Minimum number of stalls a spine (including extensions) must have
/// to survive the short-segment filter.
const MIN_STALLS_PER_SPINE: usize = 3;

use crate::aisle_polygon::{
    deduplicate_corridors, generate_miter_fills, merge_corridor_shapes,
};
#[cfg(test)]
use crate::aisle_polygon::corridor_polygon;

// ---------------------------------------------------------------------------
// Face extraction via boolean overlay
// ---------------------------------------------------------------------------

/// Compute signed area of a path in [f64; 2] form.
pub(crate) fn signed_area_f64(path: &[[f64; 2]]) -> f64 {
    let n = path.len();
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += path[i][0] * path[j][1] - path[j][0] * path[i][1];
    }
    area * 0.5
}

/// Normalize a path to CCW winding (positive signed area).
fn ensure_ccw(path: &mut Vec<[f64; 2]>) {
    if signed_area_f64(path) < 0.0 {
        path.reverse();
    }
}

/// Extract positive-space face polygons by subtracting corridors and holes
/// from the boundary. Returns Vec<Shape> where each shape is Vec<Vec<Vec2>>
/// with shape[0] = outer contour, shape[1..] = holes within that face.
///
/// Done in two steps so that corridor contours (which have consistent
/// relative winding from the union output) never share a clip set with
/// boundary hole contours (which have user-determined winding). Mixing
/// them in a single NonZero operation causes winding interference.
fn extract_faces(
    raw_outer: &[Vec2],
    merged_corridors: &[Vec<Vec<Vec2>>],
    raw_holes: &[Vec<Vec2>],
) -> Vec<Vec<Vec<Vec2>>> {
    let to_path = |pts: &[Vec2]| -> Vec<[f64; 2]> {
        pts.iter().map(|v| [v.x, v.y]).collect()
    };
    let to_vec2 = |shape: Vec<Vec<[f64; 2]>>| -> Vec<Vec<Vec2>> {
        shape
            .into_iter()
            .map(|contour| contour.into_iter().map(|p| Vec2::new(p[0], p[1])).collect())
            .collect()
    };

    // Step 1: boundary MINUS corridors. Corridor outers and holes have
    // consistent relative winding from the boolean union output, so
    // NonZero handles them correctly in isolation.
    let subj = to_path(raw_outer);
    let mut corridor_paths: Vec<Vec<[f64; 2]>> = Vec::new();
    for shape in merged_corridors {
        for contour in shape {
            corridor_paths.push(to_path(contour));
        }
    }
    let after_corridors = subj.overlay(&corridor_paths, OverlayRule::Difference, FillRule::NonZero);

    // Step 2: subtract raw building holes (derived from aisle-edge rings).
    // Kept separate so that hole winding can't interfere with corridor winding.
    if raw_holes.is_empty() {
        return after_corridors.into_iter().map(to_vec2).collect();
    }

    let mut hole_paths: Vec<Vec<[f64; 2]>> = Vec::new();
    for hole in raw_holes {
        let mut path = to_path(hole);
        ensure_ccw(&mut path);
        hole_paths.push(path);
    }

    // Feed step 1 output as subject paths for the second subtraction.
    let subj2: Vec<Vec<[f64; 2]>> = after_corridors
        .into_iter()
        .flat_map(|shape| shape.into_iter())
        .collect();
    let result = subj2.overlay(&hole_paths, OverlayRule::Difference, FillRule::NonZero);

    result.into_iter().map(to_vec2).collect()
}

// ---------------------------------------------------------------------------
// Straight skeleton spine generation
// ---------------------------------------------------------------------------

/// Test if point `p` is inside `polygon` using the winding-number rule.
fn point_in_polygon(p: Vec2, polygon: &[Vec2]) -> bool {
    let n = polygon.len();
    let mut winding = 0i32;
    for i in 0..n {
        let j = (i + 1) % n;
        let a = polygon[i];
        let b = polygon[j];
        if a.y <= p.y {
            if b.y > p.y {
                let cross = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
                if cross > 0.0 {
                    winding += 1;
                }
            }
        } else if b.y <= p.y {
            let cross = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
            if cross < 0.0 {
                winding -= 1;
            }
        }
    }
    winding != 0
}

/// Test if point `p` is inside a face shape (inside outer contour, outside
/// all hole contours).
fn point_in_face(p: Vec2, shape: &[Vec<Vec2>]) -> bool {
    if shape.is_empty() {
        return false;
    }
    if !point_in_polygon(p, &shape[0]) {
        return false;
    }
    for hole in &shape[1..] {
        if point_in_polygon(p, hole) {
            return false;
        }
    }
    true
}

/// Like `point_in_face` but also accepts points within `tol` of any face
/// boundary edge. Needed for skeleton debug filtering where source vertices
/// lie exactly on the face boundary (winding-number test treats them as
/// outside).
fn point_in_or_on_face(p: Vec2, shape: &[Vec<Vec2>], tol: f64) -> bool {
    if point_in_face(p, shape) {
        return true;
    }
    for contour in shape {
        let n = contour.len();
        for i in 0..n {
            let j = (i + 1) % n;
            if point_to_segment_dist(p, contour[i], contour[j]) < tol {
                return true;
            }
        }
    }
    false
}

/// Clip line segment `a→b` to the interior of a face shape (outer contour
/// minus holes).  Returns parameter intervals `[(t0,t1), …]` where
/// `a + t*(b-a)` is inside.
fn clip_segment_to_face(a: Vec2, b: Vec2, shape: &[Vec<Vec2>]) -> Vec<(f64, f64)> {
    let d = b - a;
    if d.length() < 1e-12 {
        return vec![];
    }

    let mut ts = vec![0.0f64, 1.0];

    // Intersect with ALL contour edges (outer + holes).
    for contour in shape.iter() {
        let n = contour.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let p = contour[i];
            let e = contour[j] - contour[i];
            let denom = d.x * e.y - d.y * e.x;
            if denom.abs() < 1e-12 {
                continue;
            }
            let h = p - a;
            let t = (h.x * e.y - h.y * e.x) / denom;
            let s = (h.x * d.y - h.y * d.x) / denom;
            if s >= -1e-9 && s <= 1.0 + 1e-9 {
                ts.push(t.clamp(0.0, 1.0));
            }
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

    let mut segments: Vec<(f64, f64)> = Vec::new();
    for i in 0..ts.len() - 1 {
        let t_mid = (ts[i] + ts[i + 1]) / 2.0;
        let mid = a + d * t_mid;
        if point_in_face(mid, shape) {
            if let Some(last) = segments.last_mut() {
                if (ts[i] - last.1).abs() < 1e-9 {
                    last.1 = ts[i + 1];
                    continue;
                }
            }
            segments.push((ts[i], ts[i + 1]));
        }
    }

    segments
}

/// Shortest distance from point `p` to line segment `a→b`.
fn point_to_segment_dist(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < 1e-12 {
        return (p - a).length();
    }
    let t = (p - a).dot(ab) / len_sq;
    let proj = a + ab * t.clamp(0.0, 1.0);
    (p - proj).length()
}

/// Classify each edge of `contour` as aisle-facing (true) or not.
/// An edge is aisle-facing if its midpoint lies on a corridor polygon
/// edge — meaning it was carved by a corridor during the boolean
/// difference. Uses a tight floating-point epsilon since face edges
/// are exact sub-segments of corridor edges after the boolean overlay.
/// A face is boundary if none of its aisle-facing edges come from an
/// interior aisle. Faces bounded only by perimeter/hole aisles (and
/// walls) get 90° stalls. Any interior aisle edge makes the face interior.
/// Returns (is_boundary, wall_edge_indices).
fn is_boundary_face(
    contour: &[Vec2],
    merged_aisle_shapes: &[Vec<Vec<Vec2>>],
    per_edge_aisles: &[(Vec<Vec2>, bool, Option<Vec2>)],
) -> bool {
    let eps = 0.5;
    let n = contour.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let p0 = contour[i];
        let p1 = contour[j];

        // Condition 1: check against MERGED aisle shapes. Face edges come
        // from the merged union, so this is the right thing to match against.
        // Sample p0, midpoint, and p1: a long face edge may straddle an
        // aisle/wall junction, so requiring both endpoints on one segment
        // fails. Any sample point on a merged aisle means the edge is at
        // least partially aisle-facing (not a pure wall).
        let mid = Vec2 { x: (p0.x + p1.x) * 0.5, y: (p0.y + p1.y) * 0.5 };
        let samples = [p0, mid, p1];
        let mut on_merged = false;
        'merged: for shape in merged_aisle_shapes {
            for ring in shape {
                for ci in 0..ring.len() {
                    let cj = (ci + 1) % ring.len();
                    for &pt in &samples {
                        if point_to_segment_dist(pt, ring[ci], ring[cj]) < eps {
                            on_merged = true;
                            break 'merged;
                        }
                    }
                }
            }
        }
        if !on_merged {
            return true; // Wall edge.
        }

        // Condition 2: check which un-merged aisle this edge belongs to.
        let mut best_dist = f64::INFINITY;
        let mut best_interior = false;
        for (poly, interior, _) in per_edge_aisles {
            for ci in 0..poly.len() {
                let cj = (ci + 1) % poly.len();
                let d = point_to_segment_dist(p0, poly[ci], poly[cj])
                    .max(point_to_segment_dist(p1, poly[ci], poly[cj]));
                if d < best_dist {
                    best_dist = d;
                    best_interior = *interior;
                }
            }
        }
        if best_interior {
            return false; // Has an interior aisle edge.
        }
    }
    true
}

/// Check whether a point is within `eps` of any merged aisle segment.
fn point_on_merged_aisle(pt: Vec2, merged: &[Vec<Vec<Vec2>>], eps: f64) -> bool {
    for shape in merged {
        for ring in shape {
            for ci in 0..ring.len() {
                let cj = (ci + 1) % ring.len();
                if point_to_segment_dist(pt, ring[ci], ring[cj]) < eps {
                    return true;
                }
            }
        }
    }
    false
}

/// Find the best-matching un-merged corridor for a face edge sub-segment.
/// Returns corridor index using segment-overlap scoring.
fn best_corridor_for_edge(
    p0: Vec2,
    p1: Vec2,
    per_edge_aisles: &[(Vec<Vec2>, bool, Option<Vec2>)],
) -> usize {
    let eps = 0.5;
    let mid = Vec2 { x: (p0.x + p1.x) * 0.5, y: (p0.y + p1.y) * 0.5 };
    let face_dir = p1 - p0;
    let face_len = face_dir.length();
    let mut best_score = f64::NEG_INFINITY;
    let mut best_idx = 0usize;
    for (idx, (poly, _, _)) in per_edge_aisles.iter().enumerate() {
        for ci in 0..poly.len() {
            let cj = (ci + 1) % poly.len();
            let ca = poly[ci];
            let cb = poly[cj];
            let perp = point_to_segment_dist(mid, ca, cb);
            if perp > eps * 2.0 {
                continue;
            }
            if face_len > 1e-9 {
                let fd = face_dir * (1.0 / face_len);
                let t0: f64 = 0.0;
                let t1 = face_len;
                let s0 = (ca - p0).dot(fd);
                let s1 = (cb - p0).dot(fd);
                let (smin, smax) = if s0 < s1 { (s0, s1) } else { (s1, s0) };
                let overlap = (t1.min(smax) - t0.max(smin)).max(0.0);
                let score = overlap - perp * 10.0;
                if score > best_score {
                    best_score = score;
                    best_idx = idx;
                }
            } else {
                let score = -perp;
                if score > best_score {
                    best_score = score;
                    best_idx = idx;
                }
            }
        }
    }
    best_idx
}

/// Tag each edge of a face with its source (wall or aisle corridor).
/// Runs once after `extract_faces()` so downstream code can read tags
/// instead of re-doing distance-based matching. Edges that span an
/// aisle/wall junction are split at the transition point.
fn tag_face_edges(
    shape: &[Vec<Vec2>],
    merged_aisle_shapes: &[Vec<Vec<Vec2>>],
    per_edge_aisles: &[(Vec<Vec2>, bool, Option<Vec2>)],
    two_way_oriented_dirs: &[Option<Vec2>],
) -> TaggedFace {
    let tag_contour = |contour: &[Vec2]| -> Vec<FaceEdge> {
        let eps = 0.5;
        let n = contour.len();
        let mut edges = Vec::with_capacity(n);
        for i in 0..n {
            let j = (i + 1) % n;
            let p0 = contour[i];
            let p1 = contour[j];

            let p0_on = point_on_merged_aisle(p0, merged_aisle_shapes, eps);
            let p1_on = point_on_merged_aisle(p1, merged_aisle_shapes, eps);

            if !p0_on && !p1_on {
                // Both endpoints off merged aisle — pure wall.
                let mid = Vec2 { x: (p0.x + p1.x) * 0.5, y: (p0.y + p1.y) * 0.5 };
                if !point_on_merged_aisle(mid, merged_aisle_shapes, eps) {
                    edges.push(FaceEdge { start: p0, end: p1, source: EdgeSource::Wall });
                    continue;
                }
                // Midpoint is on aisle but endpoints aren't — rare, treat as
                // aisle (conservative; could split further but unlikely to matter).
            }

            if p0_on && p1_on {
                // Both endpoints on merged aisle — pure aisle edge.
                let idx = best_corridor_for_edge(p0, p1, per_edge_aisles);
                let (_, interior, travel_dir) = &per_edge_aisles[idx];
                let is_two_way_oriented = two_way_oriented_dirs
                    .get(idx).and_then(|v| *v).is_some();
                edges.push(FaceEdge {
                    start: p0, end: p1,
                    source: EdgeSource::Aisle {
                        corridor_idx: idx, interior: *interior,
                        travel_dir: *travel_dir, is_two_way_oriented,
                    },
                });
                continue;
            }

            // One endpoint on aisle, the other on wall — find the split point
            // via binary search along the edge.
            let (on_pt, off_pt, on_first) = if p0_on { (p0, p1, true) } else { (p1, p0, false) };
            let mut lo = 0.0_f64;
            let mut hi = 1.0_f64;
            for _ in 0..16 {
                let t = (lo + hi) * 0.5;
                let pt = on_pt + (off_pt - on_pt) * t;
                if point_on_merged_aisle(pt, merged_aisle_shapes, eps) {
                    lo = t;
                } else {
                    hi = t;
                }
            }
            let split_t = (lo + hi) * 0.5;
            let split_pt = on_pt + (off_pt - on_pt) * split_t;

            // Create two sub-edges in the correct winding order.
            let (aisle_start, aisle_end, wall_start, wall_end) = if on_first {
                (p0, split_pt, split_pt, p1)
            } else {
                (split_pt, p1, p0, split_pt)
            };

            // Tag aisle sub-edge.
            let idx = best_corridor_for_edge(aisle_start, aisle_end, per_edge_aisles);
            let (_, interior, travel_dir) = &per_edge_aisles[idx];
            let is_two_way_oriented = two_way_oriented_dirs
                .get(idx).and_then(|v| *v).is_some();

            if on_first {
                edges.push(FaceEdge {
                    start: aisle_start, end: aisle_end,
                    source: EdgeSource::Aisle {
                        corridor_idx: idx, interior: *interior,
                        travel_dir: *travel_dir, is_two_way_oriented,
                    },
                });
                edges.push(FaceEdge { start: wall_start, end: wall_end, source: EdgeSource::Wall });
            } else {
                edges.push(FaceEdge { start: wall_start, end: wall_end, source: EdgeSource::Wall });
                edges.push(FaceEdge {
                    start: aisle_start, end: aisle_end,
                    source: EdgeSource::Aisle {
                        corridor_idx: idx, interior: *interior,
                        travel_dir: *travel_dir, is_two_way_oriented,
                    },
                });
            }
        }
        edges
    };

    let edges = if !shape.is_empty() { tag_contour(&shape[0]) } else { vec![] };
    let hole_edges: Vec<Vec<FaceEdge>> = shape[1..].iter().map(|h| tag_contour(h)).collect();

    // Derive is_boundary and wall_edge_indices.
    let wall_edge_indices: Vec<usize> = edges.iter().enumerate()
        .filter(|(_, e)| matches!(e.source, EdgeSource::Wall))
        .map(|(i, _)| i)
        .collect();
    let has_interior_aisle = edges.iter().any(|e| matches!(e.source, EdgeSource::Aisle { interior: true, .. }));
    let is_boundary = !wall_edge_indices.is_empty() || !has_interior_aisle;

    TaggedFace { edges, hole_edges, is_boundary, wall_edge_indices }
}

/// Classify each edge of a face contour as (aisle_facing, is_interior, travel_dir).
///
/// `is_interior` is a per-FACE decision: a face with any non-aisle-facing
/// edge is a boundary face (90° stalls). A face with all aisle-facing edges
/// is interior (angled stalls).
/// from being confused with a horizontal perimeter corridor nearby.
fn classify_face_edges(
    contour: &[Vec2],
    corridor_shapes: &[Vec<Vec<Vec2>>],
    per_edge_corridors: &[(Vec<Vec2>, bool, Option<Vec2>)],
    classify: bool,
) -> Vec<(bool, bool, Option<Vec2>)> {
    classify_face_edges_ext(contour, corridor_shapes, per_edge_corridors, classify, &[])
        .into_iter()
        .map(|(a, b, c, _)| (a, b, c))
        .collect()
}

/// Extended version that also reports which edges are two-way-oriented.
fn classify_face_edges_ext(
    contour: &[Vec2],
    corridor_shapes: &[Vec<Vec<Vec2>>],
    per_edge_corridors: &[(Vec<Vec2>, bool, Option<Vec2>)],
    classify: bool,
    two_way_oriented_dirs: &[Option<Vec2>],
) -> Vec<(bool, bool, Option<Vec2>, bool)> {
    if !classify {
        return vec![(true, true, None, false); contour.len()];
    }

    let is_interior = !is_boundary_face(contour, corridor_shapes, per_edge_corridors);

    let eps = 0.5;
    let n = contour.len();
    let mut result = vec![(false, is_interior, None, false); n];
    for i in 0..n {
        let j = (i + 1) % n;
        let p0 = contour[i];
        let p1 = contour[j];
        let mid = (p0 + p1) * 0.5;

        let mut is_aisle = false;
        'outer: for shape in corridor_shapes {
            for ring in shape {
                for ci in 0..ring.len() {
                    let cj = (ci + 1) % ring.len();
                    let d = point_to_segment_dist(p0, ring[ci], ring[cj])
                        .max(point_to_segment_dist(p1, ring[ci], ring[cj]));
                    if d < eps {
                        is_aisle = true;
                        break 'outer;
                    }
                }
            }
        }
        if !is_aisle {
            continue;
        }

        let mut best_dist = f64::INFINITY;
        let mut best_travel_dir = None;
        let mut best_idx = 0usize;
        for (idx, (poly, _, travel_dir)) in per_edge_corridors.iter().enumerate() {
            for ci in 0..poly.len() {
                let cj = (ci + 1) % poly.len();
                let d = point_to_segment_dist(mid, poly[ci], poly[cj]);
                if d < best_dist {
                    best_dist = d;
                    best_travel_dir = *travel_dir;
                    best_idx = idx;
                }
            }
        }
        let is_two_way_oriented = two_way_oriented_dirs
            .get(best_idx)
            .and_then(|v| *v)
            .is_some();
        result[i] = (true, is_interior, best_travel_dir, is_two_way_oriented);
    }
    result
}

/// Simplify a polygon contour for skeleton computation.
/// Removes duplicate vertices (zero-length edges) and merges nearly-collinear
/// consecutive edges (angle between them smaller than `angle_tol` radians).
fn simplify_contour(pts: &[Vec2], angle_tol: f64) -> Vec<Vec2> {
    if pts.len() < 3 {
        return pts.to_vec();
    }

    // Pass 1: remove duplicate vertices (truly zero-length edges only).
    let dup_tol = 1e-6;
    let mut deduped: Vec<Vec2> = Vec::with_capacity(pts.len());
    for &p in pts {
        if deduped.last().map_or(true, |prev: &Vec2| (*prev - p).length() > dup_tol) {
            deduped.push(p);
        }
    }
    while deduped.len() > 3 {
        if (deduped[0] - *deduped.last().unwrap()).length() <= dup_tol {
            deduped.pop();
        } else {
            break;
        }
    }

    // Pass 2: merge nearly-collinear edges.
    let cos_tol = angle_tol.cos();
    let mut simplified: Vec<Vec2> = Vec::with_capacity(deduped.len());
    let n = deduped.len();
    for i in 0..n {
        let prev = deduped[(i + n - 1) % n];
        let curr = deduped[i];
        let next = deduped[(i + 1) % n];
        let d1 = curr - prev;
        let d2 = next - curr;
        let len1 = d1.length();
        let len2 = d2.length();
        if len1 < 1e-12 || len2 < 1e-12 {
            continue;
        }
        let dot = d1.dot(d2) / (len1 * len2);
        if dot > cos_tol {
            continue;
        }
        simplified.push(curr);
    }

    if simplified.len() < 3 {
        return deduped;
    }
    simplified
}

/// Normalize winding for multi-contour skeleton input.
/// Outer contour (index 0) → CCW (positive signed area).
/// Hole contours (index 1+) → CW (negative signed area).
fn normalize_face_winding(shape: &[Vec<Vec2>]) -> Vec<Vec<Vec2>> {
    shape.iter().enumerate().filter_map(|(ci, c)| {
        if c.len() < 3 { return None; }
        let sa = signed_area(c);
        let needs_reverse = (ci == 0 && sa < 0.0) || (ci > 0 && sa > 0.0);
        Some(if needs_reverse {
            c.iter().rev().copied().collect()
        } else {
            c.clone()
        })
    }).collect()
}

/// Generate spine segments for a face shape (outer contour + holes).
///
/// Computes the multi-contour straight skeleton (outer CCW + holes CW),
/// extracts wavefront loops at `effective_depth`, and emits a spine for
/// each surviving aisle-facing edge. Spines are optionally clipped to
/// the face interior.
///
/// `per_edge_corridors` carries un-merged corridor polygons with their
/// `interior` flag and travel direction so that spines can be tagged
/// as interior vs perimeter with the correct one-way direction.
/// Perimeter aisle-facing edges get a higher skeleton weight so their
/// spines sit at full stall_depth from the corridor (for 90° stalls).
fn compute_face_spines(
    shape: &[Vec<Vec2>],
    effective_depth: f64,
    corridor_shapes: &[Vec<Vec<Vec2>>],
    per_edge_corridors: &[(Vec<Vec2>, bool, Option<Vec2>)],
    face_is_boundary: bool,
    _params: &ParkingParams,
    debug: &DebugToggles,
    two_way_oriented_dirs: &[Option<Vec2>],
    tagged: Option<&TaggedFace>,
) -> Vec<SpineSegment> {
    if shape.is_empty() || shape[0].len() < 3 {
        return vec![];
    }

    // Skip faces that are too narrow to hold a stall strip.
    let outer = &shape[0];
    let area = signed_area(outer).abs();
    let perimeter: f64 = outer.iter().enumerate().map(|(i, v)| {
        let next = &outer[(i + 1) % outer.len()];
        (*next - *v).length()
    }).sum();
    if perimeter > 0.0 && 4.0 * area / perimeter < effective_depth {
        return vec![];
    }

    let mut contours = normalize_face_winding(shape);
    if debug.face_simplification {
        contours = contours.into_iter()
            .map(|c| simplify_contour(&c, 0.035))
            .filter(|c| c.len() >= 3)
            .collect();
        if contours.is_empty() {
            return vec![];
        }
    }
    let ed = effective_depth - 0.05;
    let mut aisle_facing_flat: Vec<bool> = Vec::new();
    let face_interior = !face_is_boundary;
    let mut interior_flat: Vec<bool> = Vec::new();
    let mut travel_dir_flat: Vec<Option<Vec2>> = Vec::new();
    let mut two_way_oriented_flat: Vec<bool> = Vec::new();

    // Use tagged classification when available and face_simplification is off
    // (simplification changes vertex count, breaking tag alignment).
    let use_tags = tagged.is_some() && !debug.face_simplification;
    if use_tags {
        let tf = tagged.unwrap();
        // Build list of (contour, tagged_edges) pairs to classify.
        let mut pairs: Vec<(&[Vec2], &[FaceEdge])> = Vec::new();
        if !contours.is_empty() {
            pairs.push((&contours[0], &tf.edges));
        }
        for (ci, contour) in contours.iter().enumerate().skip(1) {
            let hi = ci - 1;
            if hi < tf.hole_edges.len() {
                pairs.push((contour, &tf.hole_edges[hi]));
            }
        }
        for (contour, tagged_edges) in pairs {
            let n = contour.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let mid = (contour[i] + contour[j]) * 0.5;
                let mut best_dist = f64::INFINITY;
                let mut best_source: &EdgeSource = &EdgeSource::Wall;
                for te in tagged_edges {
                    let te_mid = (te.start + te.end) * 0.5;
                    let d = (te_mid - mid).length();
                    if d < best_dist {
                        best_dist = d;
                        best_source = &te.source;
                    }
                }
                match best_source {
                    EdgeSource::Wall => {
                        aisle_facing_flat.push(false);
                        interior_flat.push(face_interior);
                        travel_dir_flat.push(None);
                        two_way_oriented_flat.push(false);
                    }
                    EdgeSource::Aisle { travel_dir, is_two_way_oriented, .. } => {
                        aisle_facing_flat.push(true);
                        interior_flat.push(face_interior);
                        travel_dir_flat.push(*travel_dir);
                        two_way_oriented_flat.push(*is_two_way_oriented);
                    }
                }
            }
        }
    } else {
        for contour in contours.iter() {
            let classified = classify_face_edges_ext(contour, corridor_shapes, per_edge_corridors, debug.edge_classification, two_way_oriented_dirs);
            for &(facing, _ignored_interior, travel_dir, is_two_way_ori) in classified.iter() {
                aisle_facing_flat.push(facing);
                interior_flat.push(face_interior);
                travel_dir_flat.push(travel_dir);
                two_way_oriented_flat.push(is_two_way_ori);
            }
        }
    }

    // Edge weights for the weighted skeleton: aisle-facing edges shrink at
    // normal speed (1.0), non-aisle-facing edges (boundary walls) stay fixed
    // (0.0). This lets one-sided perimeter faces produce spines even when
    // the face is narrower than 2× effective_depth.
    let edge_weights: Vec<f64> = aisle_facing_flat
        .iter()
        .map(|&af| if af { 1.0 } else { 0.0 })
        .collect();
    let sk = skeleton::compute_skeleton_multi(&contours, &edge_weights);
    let wf_loops = skeleton::wavefront_loops_at(&sk, ed);

    // Collect wavefront loops to process, each with an optional edge to skip.
    // Normal path: no skipped edges.
    // Suppression path (narrow faces): skip the suppressed edge so we don't
    // emit a spine on the face boundary where the edge didn't move.
    let mut wf_to_process: Vec<(Vec<Vec2>, Vec<usize>, Option<usize>)> = Vec::new();
    for (pts, edges) in wf_loops {
        wf_to_process.push((pts, edges, None));
    }

    // If the normal wavefront is empty, the face is too narrow for back-to-back
    // stalls but may fit single-sided rows. Suppress one aisle-facing edge at a
    // time (treat it as a wall) and re-run the skeleton to get spines for the
    // remaining aisle edges.
    if wf_to_process.is_empty() {
        let aisle_indices: Vec<usize> = aisle_facing_flat
            .iter()
            .enumerate()
            .filter(|(_, &af)| af)
            .map(|(i, _)| i)
            .collect();
        if aisle_indices.len() >= 2 {
            for &suppress_idx in &aisle_indices {
                let mut w = edge_weights.clone();
                w[suppress_idx] = 0.0;
                let sk2 = skeleton::compute_skeleton_multi(&contours, &w);
                for (pts, edges) in skeleton::wavefront_loops_at(&sk2, ed) {
                    wf_to_process.push((pts, edges, Some(suppress_idx)));
                }
            }
        }
    }

    let mut all_spines = Vec::new();
    for (wf_pts, active_edges, skip_edge) in &wf_to_process {
        let m = active_edges.len();
        if m < 2 || wf_pts.len() != m {
            continue;
        }
        for idx in 0..m {
            let next_idx = (idx + 1) % m;
            let orig_edge = active_edges[next_idx];
            if orig_edge >= aisle_facing_flat.len() || !aisle_facing_flat[orig_edge] {
                continue;
            }
            if skip_edge.map_or(false, |s| s == orig_edge) {
                continue;
            }

            let spine_start = wf_pts[idx];
            let spine_end = wf_pts[next_idx];
            if (spine_end - spine_start).length() < 1.0 {
                continue;
            }

            // Outward normal = opposite of the edge's inward normal.
            let outward = Vec2::new(-sk.edge_normals[orig_edge].x, -sk.edge_normals[orig_edge].y);
            let is_interior = interior_flat.get(orig_edge).copied().unwrap_or(true);
            let is_two_way_ori = two_way_oriented_flat.get(orig_edge).copied().unwrap_or(false);
            let travel_dir = travel_dir_flat.get(orig_edge).copied().flatten().map(|td| {
                if is_two_way_ori {
                    // For two-way-oriented aisles, flip travel_dir for the
                    // spine on the "canonical negative" side. This condition
                    // is td-INDEPENDENT (based only on outward_normal), so
                    // it always flips the same physical side regardless of
                    // which variant is active. Combined with the standard
                    // flip_angle formula, this makes both sides produce the
                    // same flip_angle, giving correct per-lane stall angles.
                    let neg_side = outward.x < -1e-9
                        || (outward.x.abs() < 1e-9 && outward.y < -1e-9);
                    if neg_side { Vec2::new(-td.x, -td.y) } else { td }
                } else {
                    td
                }
            });

            // Suppress stalls on the left side of one-way aisles.
            if let Some(td) = travel_dir {
                if !is_two_way_ori && td.cross(outward) >= 0.0 {
                    continue;
                }
            }

            if !debug.spine_clipping {
                all_spines.push(SpineSegment {
                    start: spine_start,
                    end: spine_end,
                    outward_normal: outward,
                    face_idx: 0,
                    is_interior,
                    travel_dir,
                });
                continue;
            }

            // Clip spine and stall-reach line to face interior.
            let spine_clips = clip_segment_to_face(spine_start, spine_end, shape);
            let reach = (ed - 0.5).max(0.0);
            let offset_start = spine_start + outward * reach;
            let offset_end = spine_end + outward * reach;
            let offset_clips = clip_segment_to_face(offset_start, offset_end, shape);

            for (st0, st1) in &spine_clips {
                for (ot0, ot1) in &offset_clips {
                    let t0 = st0.max(*ot0);
                    let t1 = st1.min(*ot1);
                    if t1 - t0 < 1e-9 {
                        continue;
                    }
                    let d = spine_end - spine_start;
                    let s = spine_start + d * t0;
                    let e = spine_start + d * t1;
                    if (e - s).length() > 1.0 {
                        all_spines.push(SpineSegment {
                            start: s,
                            end: e,
                            outward_normal: outward,
                            face_idx: 0,
                            is_interior,
                            travel_dir,
                        });
                    }
                }
            }
        }
    }

    all_spines
}


// ---------------------------------------------------------------------------
// Spine merging
// ---------------------------------------------------------------------------

/// Merge collinear spine segments that share an endpoint (within tolerance)
/// and have the same outward normal direction. This eliminates gaps between
/// stall strips that span multiple faces along the same aisle edge.
fn merge_collinear_spines(spines: Vec<SpineSegment>, tolerance: f64) -> Vec<SpineSegment> {
    if spines.is_empty() {
        return spines;
    }

    let mut merged = spines;
    let mut changed = true;

    while changed {
        changed = false;
        let mut result: Vec<SpineSegment> = Vec::new();
        let mut used = vec![false; merged.len()];

        for i in 0..merged.len() {
            if used[i] {
                continue;
            }

            let mut seg = merged[i].clone();
            used[i] = true;

            // Repeatedly try to extend this segment by merging with others.
            let mut extended = true;
            while extended {
                extended = false;
                for j in 0..merged.len() {
                    if used[j] {
                        continue;
                    }

                    if let Some(m) = try_merge_spines(&seg, &merged[j], tolerance) {
                        seg = m;
                        used[j] = true;
                        extended = true;
                        changed = true;
                    }
                }
            }

            result.push(seg);
        }

        merged = result;
    }

    merged
}

/// Try to merge two spine segments. Returns the merged segment if they are
/// collinear (directions within tolerance), have similar outward normals,
/// and share an endpoint.
fn try_merge_spines(a: &SpineSegment, b: &SpineSegment, tolerance: f64) -> Option<SpineSegment> {
    // Only merge spines of the same type (interior/perimeter).
    if a.is_interior != b.is_interior {
        return None;
    }

    // Only merge spines with compatible travel directions.
    match (&a.travel_dir, &b.travel_dir) {
        (None, None) => {}
        (Some(d1), Some(d2)) if d1.dot(*d2) > 0.99 => {}
        _ => return None,
    }

    let dir_a = (a.end - a.start).normalize();
    let dir_b = (b.end - b.start).normalize();

    // Must be collinear: directions parallel (dot ≈ ±1).
    if dir_a.dot(dir_b).abs() < 0.99 {
        return None;
    }

    // Must have (nearly) identical outward normals. Spines from the same
    // corridor edge have exactly equal normals; spines from different edges
    // at the same boundary corner differ by the angle between the edges.
    // A tight threshold (0.9999 ≈ 0.8°) prevents cross-edge merging that
    // would extend one edge's normal over another edge's territory.
    if a.outward_normal.dot(b.outward_normal) < 0.9999 {
        return None;
    }

    // Must share an endpoint (within tolerance).
    let pairs = [
        (a.end, b.start),  // a→b chain
        (a.start, b.end),  // b→a chain
        (a.end, b.end),    // both point away from junction
        (a.start, b.start), // both point toward junction
    ];

    for (p1, p2) in &pairs {
        if (*p1 - *p2).length() > tolerance {
            continue;
        }

        // Project all four endpoints onto the shared line direction
        // and take the full extent.
        let origin = a.start;
        let dots = [
            dir_a.dot(a.start - origin),
            dir_a.dot(a.end - origin),
            dir_a.dot(b.start - origin),
            dir_a.dot(b.end - origin),
        ];
        let min_t = dots.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_t = dots.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        return Some(SpineSegment {
            start: origin + dir_a * min_t,
            end: origin + dir_a * max_t,
            outward_normal: a.outward_normal,
            face_idx: a.face_idx,
            is_interior: a.is_interior,
            travel_dir: a.travel_dir,
        });
    }

    None
}

// ---------------------------------------------------------------------------
// Top-level orchestration
// ---------------------------------------------------------------------------

/// Remove stalls where any shrunk corner falls outside the stall's own
/// face. This catches stalls that protrude past the face boundary into
/// corridors or adjacent faces.
fn clip_stalls_to_faces(
    stalls: Vec<(StallQuad, usize)>,
    faces: &[Vec<Vec<Vec2>>],
) -> Vec<(StallQuad, usize)> {
    stalls
        .into_iter()
        .filter(|(stall, face_idx)| {
            if *face_idx >= faces.len() { return false; }
            let shape = &faces[*face_idx];
            // Shrink 40% toward centroid for tolerance. Angled stalls have
            // corners that extend past the spine into the corridor, so a
            // proportional shrink handles all angles.
            let cx = stall.corners.iter().map(|c| c.x).sum::<f64>() / 4.0;
            let cy = stall.corners.iter().map(|c| c.y).sum::<f64>() / 4.0;
            let centroid = Vec2::new(cx, cy);
            let shrunk: Vec<Vec2> = stall.corners.iter().map(|c| {
                *c + (centroid - *c) * 0.4
            }).collect();
            // Every shrunk corner must be inside the stall's own face.
            if !shrunk.iter().all(|corner| point_in_face(*corner, shape)) {
                return false;
            }
            // Corner ray check: rays from p2 (dir p1→p2) and p3 (dir p0→p3)
            // must hit the same face boundary edge. Reject stalls that
            // straddle a face corner (each depth side projects to a
            // different edge).
            if !shape.is_empty() {
                let origin_a = stall.corners[2] + (centroid - stall.corners[2]) * 0.2;
                let origin_b = stall.corners[3] + (centroid - stall.corners[3]) * 0.2;
                let dir_a = (stall.corners[2] - stall.corners[1]).normalize();
                let dir_b = (stall.corners[3] - stall.corners[0]).normalize();
                let hit_a = ray_hit_face_edge(origin_a, dir_a, shape);
                let hit_b = ray_hit_face_edge(origin_b, dir_b, shape);
                match (hit_a, hit_b) {
                    (Some((ci_a, ei_a)), Some((ci_b, ei_b))) => {
                        if ci_a != ci_b || ei_a != ei_b { return false; }
                    }
                    _ => { return false; }
                }
            }
            true
        })
        .collect()
}


/// Compute the centering shift for a spine: the offset to add to proj_start
/// so that stalls are centered symmetrically within the spine extent.
fn compute_centering_shift(proj_start: f64, edge_len: f64, pitch: f64) -> f64 {
    let k_min = (proj_start / pitch).ceil() as i64;
    let k_max = ((proj_start + edge_len) / pitch - 1.0).floor() as i64;
    if k_max < k_min {
        return 0.0;
    }
    let n = k_max - k_min + 1;
    let total_gap = edge_len - n as f64 * pitch;
    let desired_gap_start = total_gap / 2.0;
    let current_gap_start = k_min as f64 * pitch - proj_start;
    current_gap_start - desired_gap_start
}

/// Place stalls on spine segments, returning stalls tagged with
/// (face_idx, spine_idx) and per-spine centering shifts (indexed by
/// spine position in the input slice). When `extensions` is provided,
/// centering uses the full projected range (primary + extensions) for
/// each spine instead of just the primary length.
/// Compute fill_strip parameters for a spine segment.
fn spine_fill_params(seg: &SpineSegment) -> (Vec2, Vec2, Option<f64>, bool, f64) {
    let (start, end) = seg.oriented_endpoints();
    let oriented_dir = (end - start).normalize();
    let angle_override = if seg.is_interior { None } else { Some(90.0) };
    let flip_angle = seg.travel_dir.map_or(false, |td| oriented_dir.dot(td) > 0.0);
    // Stagger grid by half a pitch for one-way spines whose outward_normal
    // points to the right of the travel direction (cross product >= 0).
    let grid_offset = seg.travel_dir.map_or(0.0, |td| {
        if td.cross(seg.outward_normal) >= 0.0 { 0.5 } else { 0.0 }
    });
    (start, end, angle_override, flip_angle, grid_offset)
}

/// Place stalls on a spine with the given centering shift.
fn fill_spine(seg: &SpineSegment, params: &ParkingParams, centering: f64) -> Vec<StallQuad> {
    let (start, end, angle_override, flip_angle, grid_offset) = spine_fill_params(seg);
    fill_strip(start, end, 1.0, 0.0, params, angle_override, flip_angle, grid_offset, centering)
}

fn place_stalls_on_spines(
    spines: &[SpineSegment],
    params: &ParkingParams,
    center: bool,
    extensions: Option<&[(SpineSegment, usize)]>,
) -> (Vec<(StallQuad, usize, usize)>, Vec<f64>) {
    let stall_pitch = params.stall_pitch();

    // Pre-compute the full projected range along each spine's oriented
    // direction, incorporating any extensions.
    let extended_ranges: Vec<Option<(f64, f64)>> = spines.iter().enumerate().map(|(i, seg)| {
        let exts = extensions?;
        let oriented = seg.oriented_dir();
        let p0 = seg.start.dot(oriented);
        let p1 = seg.end.dot(oriented);
        let mut lo = p0.min(p1);
        let mut hi = p0.max(p1);
        let mut found = false;
        for (ext, src_idx) in exts {
            if *src_idx != i { continue; }
            found = true;
            let e0 = ext.start.dot(oriented);
            let e1 = ext.end.dot(oriented);
            lo = lo.min(e0.min(e1));
            hi = hi.max(e0.max(e1));
        }
        if found { Some((lo, hi)) } else { None }
    }).collect();

    // Sort spines longest-first (using extended range when available).
    let mut order: Vec<(usize, f64)> = spines.iter().enumerate()
        .map(|(i, s)| {
            let len = match &extended_ranges[i] {
                Some((lo, hi)) => hi - lo,
                None => (s.end - s.start).length(),
            };
            (i, len)
        })
        .collect();
    order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    // Track centering shifts. When a spine has an opposing match
    // (collinear + opposite normals), it grid-locks to the existing
    // shift instead of centering independently.
    let mut claimed: Vec<(usize, Vec2, f64)> = Vec::new();
    let mut shifts = vec![0.0_f64; spines.len()];
    let mut stalls = Vec::new();
    for &(spine_idx, _) in &order {
        let seg = &spines[spine_idx];
        let edge_len = (seg.end - seg.start).length();
        let oriented_dir = seg.oriented_dir();
        let (start, _, angle_override, _, _) = spine_fill_params(seg);
        let eff_pitch = if angle_override.is_some() { params.stall_width } else { stall_pitch };
        let proj_start = start.dot(oriented_dir);

        let centering = if !center {
            0.0
        } else if let Some((shift, _partner_idx)) = find_opposing_shift(&claimed, spines, spine_idx, oriented_dir, eff_pitch, params.aisle_width + 2.0 * params.stall_depth) {
            shift
        } else {
            let (center_proj, center_len) = match &extended_ranges[spine_idx] {
                Some((lo, hi)) => (*lo, hi - lo),
                None => (proj_start, edge_len),
            };
            compute_centering_shift(center_proj, center_len, eff_pitch)
        };

        claimed.push((spine_idx, oriented_dir, centering));
        shifts[spine_idx] = centering;

        for quad in fill_spine(seg, params, centering) {
            stalls.push((quad, seg.face_idx, spine_idx));
        }
    }
    (stalls, shifts)
}

/// Find the centering shift to use for grid-locking to an already-processed
/// opposing spine. Opposing means: parallel directions (|dot| > 0.99) and
/// opposite outward normals (dot < -0.99). When opposing spines have the
/// same oriented edge_dir, the shift is reused directly. When opposite,
/// the shift is negated (mod pitch) so stalls land at the same absolute
/// positions.
fn find_opposing_shift(
    claimed: &[(usize, Vec2, f64)],
    spines: &[SpineSegment],
    spine_idx: usize,
    oriented_dir: Vec2,
    pitch: f64,
    max_dist: f64,
) -> Option<(f64, usize)> {
    let seg = &spines[spine_idx];
    let dir = (seg.end - seg.start).normalize();
    let mid = (seg.start + seg.end) * 0.5;

    for &(other_idx, other_oriented_dir, other_shift) in claimed {
        let other = &spines[other_idx];
        let other_dir = (other.end - other.start).normalize();

        // Only grid-lock spines of the same type: interior↔interior or
        // boundary↔boundary. Boundary spines use 90° stalls with a
        // different pitch, so cross-type grid-locking misaligns the grid.
        if seg.is_interior != other.is_interior {
            continue;
        }
        // Collinear: directions parallel (within ~8°).
        if dir.dot(other_dir).abs() < 0.99 {
            continue;
        }
        // Opposite normals (within ~8°).
        if seg.outward_normal.dot(other.outward_normal) > -0.99 {
            continue;
        }
        // Proximity: perpendicular distance between spine midpoints
        // must be small (same aisle, not a distant parallel aisle).
        let other_mid = (other.start + other.end) * 0.5;
        let sep = other_mid - mid;
        let perp_dist = (sep - dir * sep.dot(dir)).length();
        if perp_dist > max_dist {
            continue;
        }

        // Same oriented direction: reuse shift directly.
        // Opposite oriented direction: negate shift (mod pitch).
        if oriented_dir.dot(other_oriented_dir) > 0.0 {
            return Some((other_shift, other_idx));
        } else {
            return Some(((-other_shift).rem_euclid(pitch), other_idx));
        }
    }
    None
}

/// Compute the centering shift for an extension spine that should be
/// contiguous with its source primary spine.
fn compute_extension_shift(ext: &SpineSegment, src: &SpineSegment, src_shift: f64, pitch: f64) -> f64 {
    if src.oriented_dir().dot(ext.oriented_dir()) > 0.0 {
        src_shift
    } else {
        (-src_shift).rem_euclid(pitch)
    }
}

use crate::island::{compute_islands, mark_island_stalls, stall_key};
#[cfg(test)]
use crate::island::stall_center;

/// Deduplicate overlapping collinear spines. When two spines have the same
/// direction and outward normal and overlap along their length, trim the
/// shorter one to only its non-overlapping portion. This prevents
/// miter-fill spines from extending into regions already covered by the
/// main corridor spine, while preserving the unique tail of each spine.
fn dedup_overlapping_spines(spines: Vec<SpineSegment>, tolerance: f64) -> Vec<SpineSegment> {
    let mut result = spines;

    // For each pair of collinear, overlapping spines: trim the shorter one.
    let mut changed = true;
    while changed {
        changed = false;
        let n = result.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let dir_i = (result[i].end - result[i].start).normalize();
                let dir_j = (result[j].end - result[j].start).normalize();

                // Must be collinear and same normal.
                if dir_i.dot(dir_j).abs() < 0.99 {
                    continue;
                }
                if result[i].outward_normal.dot(result[j].outward_normal) < 0.99 {
                    continue;
                }
                let travel_compat = match (&result[i].travel_dir, &result[j].travel_dir) {
                    (None, None) => true,
                    (Some(d1), Some(d2)) => d1.dot(*d2) > 0.99,
                    _ => false,
                };
                if !travel_compat {
                    continue;
                }

                // Must be on the same line (close perpendicularly).
                let perp = Vec2::new(-dir_i.y, dir_i.x);
                if perp.dot(result[j].start - result[i].start).abs() > tolerance {
                    continue;
                }

                // Project both onto the shared direction.
                let origin = result[i].start;
                let ti_s = dir_i.dot(result[i].start - origin);
                let ti_e = dir_i.dot(result[i].end - origin);
                let tj_s = dir_i.dot(result[j].start - origin);
                let tj_e = dir_i.dot(result[j].end - origin);

                let (i_min, i_max) = (ti_s.min(ti_e), ti_s.max(ti_e));
                let (j_min, j_max) = (tj_s.min(tj_e), tj_s.max(tj_e));

                // Check for overlap.
                let overlap_start = i_min.max(j_min);
                let overlap_end = i_max.min(j_max);
                if overlap_end - overlap_start < 1.0 {
                    continue; // No significant overlap.
                }

                // Trim the shorter spine to its non-overlapping portion.
                let len_i = i_max - i_min;
                let len_j = j_max - j_min;
                let (shorter, longer_min, longer_max) = if len_i <= len_j {
                    (i, j_min, j_max)
                } else {
                    (j, i_min, i_max)
                };

                let s_min = if shorter == i { i_min } else { j_min };
                let s_max = if shorter == i { i_max } else { j_max };

                // The non-overlapping portion is [s_min, longer_min) or
                // (longer_max, s_max]. Keep the longer tail.
                let tail_left = (longer_min - s_min).max(0.0);
                let tail_right = (s_max - longer_max).max(0.0);

                if tail_left > tail_right && tail_left > 1.0 {
                    // Keep the left tail.
                    result[shorter].start = origin + dir_i * s_min;
                    result[shorter].end = origin + dir_i * longer_min;
                    changed = true;
                } else if tail_right > 1.0 {
                    // Keep the right tail.
                    result[shorter].start = origin + dir_i * longer_max;
                    result[shorter].end = origin + dir_i * s_max;
                    changed = true;
                } else {
                    // Entirely covered: mark for removal (zero length).
                    result[shorter].end = result[shorter].start;
                    changed = true;
                }

                if changed {
                    break;
                }
            }
            if changed {
                break;
            }
        }

        // Remove zero-length spines.
        result.retain(|s| (s.end - s.start).length() > 1.0);
    }

    result
}

/// Extend each spine colinearly in both directions until it hits the face
/// boundary. Returns (extension SpineSegment, source spine index) pairs.
/// Each extension inherits all properties (normal, travel_dir, etc.) from its
/// source spine. A dual-clip approach validates both the spine position and
/// the stall-reach position, ensuring extension stalls have depth clearance.
fn extend_spines_to_faces(
    spines: &[SpineSegment],
    faces: &[Vec<Vec<Vec2>>],
    effective_depth: f64,
    params: &ParkingParams,
) -> Vec<(SpineSegment, usize)> {
    let mut extensions = Vec::new();

    // Compute stall pitch to use as overlap margin. fill_strip keeps stall
    // centers at least half a pitch from each segment endpoint, so extending
    // the extension segment one pitch into the primary spine ensures the
    // first extension grid cell bridges the gap. remove_extension_conflicts
    // discards duplicates in the overlap zone afterward.
    let stall_pitch = params.stall_pitch();

    for (src_idx, spine) in spines.iter().enumerate() {
        if spine.face_idx >= faces.len() {
            continue;
        }
        // Skip boundary faces — extensions only apply to interior faces
        // where angled stalls leave triangular waste in tapered regions.
        if !spine.is_interior {
            continue;
        }
        let face_shape = &faces[spine.face_idx];
        if face_shape.is_empty() {
            continue;
        }

        let dir = (spine.end - spine.start).normalize();
        let spine_len = (spine.end - spine.start).length();
        if spine_len < 1e-6 {
            continue;
        }

        // Extend the spine line far in both directions.
        let extend_dist = 10000.0;
        let ext_start = spine.start - dir * extend_dist;
        let ext_end = spine.end + dir * extend_dist;
        let ext_vec = ext_end - ext_start;
        let ext_total = ext_vec.length();

        // Clip the extended spine line to the face interior.
        let spine_clips = clip_segment_to_face(ext_start, ext_end, face_shape);

        // Also clip the offset line (stall reach) to ensure depth clearance.
        let reach = (effective_depth - 0.5).max(0.0);
        let off_start = ext_start + spine.outward_normal * reach;
        let off_end = ext_end + spine.outward_normal * reach;
        let offset_clips = clip_segment_to_face(off_start, off_end, face_shape);

        // The original spine occupies a known parameter range on the extended
        // line. Shrink it by one stall_pitch on each side so that extension
        // segments overlap the primary, ensuring contiguous stall placement.
        let overlap_t = stall_pitch / ext_total;
        let orig_t0 = extend_dist / ext_total + overlap_t;
        let orig_t1 = (extend_dist + spine_len) / ext_total - overlap_t;

        // End margin: shorten each extension's outer end (the face-boundary
        // side) so the last stall doesn't create a thin sliver island.
        let end_margin_t = (stall_pitch * 1.5) / ext_total;

        for &(st0, st1) in &spine_clips {
            for &(ot0, ot1) in &offset_clips {
                let t0 = st0.max(ot0);
                let t1 = st1.min(ot1);
                if t1 - t0 < 1e-9 {
                    continue;
                }

                // Extract portions outside the (shrunk) original spine range.
                // Outer ends are pulled inward by end_margin_t to avoid
                // thin sliver islands at face boundaries.
                // Left tail: [t0 + margin, min(t1, orig_t0)]
                if t0 < orig_t0 - 1e-9 {
                    let tail_end = t1.min(orig_t0);
                    let s = ext_start + ext_vec * (t0 + end_margin_t);
                    let e = ext_start + ext_vec * tail_end;
                    if (e - s).length() > 1.0 {
                        extensions.push((SpineSegment {
                            start: s,
                            end: e,
                            outward_normal: spine.outward_normal,
                            face_idx: spine.face_idx,
                            is_interior: spine.is_interior,
                            travel_dir: spine.travel_dir,
                        }, src_idx));
                    }
                }
                // Right tail: [max(t0, orig_t1), t1 - margin]
                if t1 > orig_t1 + 1e-9 {
                    let tail_start = t0.max(orig_t1);
                    let s = ext_start + ext_vec * tail_start;
                    let e = ext_start + ext_vec * (t1 - end_margin_t);
                    if (e - s).length() > 1.0 {
                        extensions.push((SpineSegment {
                            start: s,
                            end: e,
                            outward_normal: spine.outward_normal,
                            face_idx: spine.face_idx,
                            is_interior: spine.is_interior,
                            travel_dir: spine.travel_dir,
                        }, src_idx));
                    }
                }
            }
        }
    }

    extensions
}

/// Cast a ray from `origin` in `dir` and return (contour_index, edge_index)
/// of the nearest face boundary edge it crosses.
fn ray_hit_face_edge(
    origin: Vec2,
    dir: Vec2,
    face_shape: &[Vec<Vec2>],
) -> Option<(usize, usize)> {
    let mut best_t = f64::INFINITY;
    let mut best = None;

    for (ci, contour) in face_shape.iter().enumerate() {
        let n = contour.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let p = contour[i];
            let e = contour[j] - contour[i];
            let denom = dir.x * e.y - dir.y * e.x;
            if denom.abs() < 1e-12 {
                continue;
            }
            let h = p - origin;
            let t = (h.x * e.y - h.y * e.x) / denom;
            let s = (h.x * dir.y - h.y * dir.x) / denom;
            if t > 1e-6 && s >= -1e-9 && s <= 1.0 + 1e-9 && t < best_t {
                best_t = t;
                best = Some((ci, i));
            }
        }
    }

    best
}

/// Test whether a stall quad is mostly contained in a face by computing
/// the boolean intersection `stall ∩ face`. If the intersection area is
/// less than `min_frac` of the stall area, the stall bleeds outside.
/// Angled stall corners that poke slightly into the corridor are tolerated
/// since they represent a small fraction of total area.
fn quad_contained_in_face(
    corners: &[Vec2; 4],
    face_shape: &[Vec<Vec2>],
    min_frac: f64,
) -> bool {
    use crate::inset::signed_area;

    let stall_area = signed_area(corners).abs();
    if stall_area < 1e-6 {
        return false;
    }

    let to_path = |pts: &[Vec2]| -> Vec<[f64; 2]> {
        pts.iter().map(|v| [v.x, v.y]).collect()
    };

    let stall_path = to_path(corners);

    // Build face as multi-contour (outer CCW + holes CW).
    let mut face_paths: Vec<Vec<[f64; 2]>> = Vec::new();
    for (i, contour) in face_shape.iter().enumerate() {
        let mut p = to_path(contour);
        if i == 0 {
            // Outer must be CCW.
            if signed_area_f64(&p) < 0.0 { p.reverse(); }
        } else {
            // Holes must be CW.
            if signed_area_f64(&p) > 0.0 { p.reverse(); }
        }
        face_paths.push(p);
    }

    let intersection = stall_path.overlay(&face_paths, OverlayRule::Intersect, FillRule::NonZero);

    let mut isect_area = 0.0;
    for shape in &intersection {
        for contour in shape {
            isect_area += signed_area_f64(contour).abs();
        }
    }

    isect_area >= min_frac * stall_area
}

/// Test whether a stall quad is fully contained within the site boundary
/// (inside outer, outside all holes). Same geometric approach: centroid
/// inside + no edge crossings.
fn quad_contained_in_boundary(corners: &[Vec2; 4], raw_outer: &[Vec2], raw_holes: &[Vec<Vec2>]) -> bool {
    let cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
    let cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
    let centroid = Vec2::new(cx, cy);

    if !crate::clip::point_in_polygon(&centroid, raw_outer) {
        return false;
    }
    for hole in raw_holes {
        if crate::clip::point_in_polygon(&centroid, hole) {
            return false;
        }
    }

    // No quad edge may cross any boundary edge.
    let raw_outer_vec = raw_outer.to_vec();
    let all_contours = std::iter::once(&raw_outer_vec).chain(raw_holes.iter());
    for contour in all_contours {
        let n = contour.len();
        for i in 0..4 {
            let a = corners[i];
            let b = corners[(i + 1) % 4];
            for j in 0..n {
                let c = contour[j];
                let d = contour[(j + 1) % n];
                if crate::clip::segments_intersect(a, b, c, d) {
                    return false;
                }
            }
        }
    }
    true
}

/// Place extension stalls greedily, prioritised by source primary spine
/// length (longest primary spine's extensions first). Each candidate stall
/// is checked against all already-placed stalls (primary + earlier
/// extensions) and silently skipped on conflict, avoiding the
/// mutual-removal behaviour of remove_conflicting_stalls.
fn place_extension_stalls_greedy(
    ext_spines_with_src: &[(SpineSegment, usize)],
    primary_spines: &[SpineSegment],
    primary_stalls: &[(StallQuad, usize)],
    primary_shifts: &[f64],
    faces: &[Vec<Vec<Vec2>>],
    raw_outer: &[Vec2],
    raw_holes: &[Vec<Vec2>],
    _merged_corridors: &[Vec<Vec<Vec2>>],
    params: &ParkingParams,
    debug: &DebugToggles,
    tagged_faces: &[TaggedFace],
) -> Vec<(StallQuad, usize, usize)> {
    use crate::clip::polygons_overlap;

    // Sort by source primary spine length (longest first), breaking ties
    // by extension spine length.
    let mut ordered: Vec<(usize, f64, f64)> = ext_spines_with_src
        .iter()
        .enumerate()
        .map(|(i, (ext, src_idx))| {
            let src = &primary_spines[*src_idx];
            let src_len = (src.end - src.start).length();
            let ext_len = (ext.end - ext.start).length();
            (i, src_len, ext_len)
        })
        .collect();
    ordered.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap()
        .then(b.2.partial_cmp(&a.2).unwrap()));

    // Seed the occupied set with shrunk primary stall polygons, grouped by face.
    let mut occupied_by_face: std::collections::HashMap<usize, Vec<Vec<Vec2>>> =
        std::collections::HashMap::new();
    for (stall, face_idx) in primary_stalls {
        let shrunk = shrink_toward_centroid(&stall.corners, 0.1);
        occupied_by_face.entry(*face_idx).or_default().push(shrunk);
    }

    let mut result = Vec::new();

    for (spine_idx, _, _) in ordered {
        let (seg, src_idx) = &ext_spines_with_src[spine_idx];

        let ext_shift = compute_extension_shift(seg, &primary_spines[*src_idx], primary_shifts[*src_idx], params.stall_pitch());
        let candidates: Vec<(StallQuad, usize, usize)> =
            fill_spine(seg, params, ext_shift)
                .into_iter().map(|q| (q, seg.face_idx, 0)).collect();

        for (mut quad, face_idx, _) in candidates {
            quad.kind = StallKind::Extension;

            // Face containment: boolean intersect stall with its face.
            // Reject if less than 85% of the stall area is inside.
            if debug.stall_face_clipping && face_idx < faces.len() && !faces[face_idx].is_empty() {
                if !quad_contained_in_face(&quad.corners, &faces[face_idx], 0.85) {
                    continue;
                }
            }

            // Boundary containment: centroid inside outer, no stall edge
            // crosses outer or hole edges.
            if debug.boundary_clipping {
                if !quad_contained_in_boundary(&quad.corners, raw_outer, raw_holes) {
                    continue;
                }
            }
            //
            // Corner check: rays from p2 (dir p1→p2) and p3 (dir p0→p3).
            // Both must hit the same face boundary edge. Origins are
            // pulled 20% inward toward centroid to avoid starting outside
            // the face (angled stall corners can protrude).
            if face_idx < faces.len() && !faces[face_idx].is_empty() {
                let cx = (quad.corners[0].x + quad.corners[1].x + quad.corners[2].x + quad.corners[3].x) / 4.0;
                let cy = (quad.corners[0].y + quad.corners[1].y + quad.corners[2].y + quad.corners[3].y) / 4.0;
                let centroid = Vec2::new(cx, cy);
                let origin_a = quad.corners[2] + (centroid - quad.corners[2]) * 0.2;
                let origin_b = quad.corners[3] + (centroid - quad.corners[3]) * 0.2;
                let dir_a = (quad.corners[2] - quad.corners[1]).normalize();
                let dir_b = (quad.corners[3] - quad.corners[0]).normalize();
                let hit_a = ray_hit_face_edge(origin_a, dir_a, &faces[face_idx]);
                let hit_b = ray_hit_face_edge(origin_b, dir_b, &faces[face_idx]);
                match (hit_a, hit_b) {
                    (Some((ci_a, ei_a)), Some((ci_b, ei_b))) => {
                        if ci_a != ci_b || ei_a != ei_b { continue; }
                        // Reject extensions aimed at wall edges.
                        if face_idx < tagged_faces.len() {
                            let tf = &tagged_faces[face_idx];
                            let edges = if ci_a == 0 { &tf.edges } else if ci_a - 1 < tf.hole_edges.len() { &tf.hole_edges[ci_a - 1] } else { &tf.edges };
                            // Find the tagged edge closest to the hit edge midpoint.
                            if !faces[face_idx].is_empty() && ci_a < faces[face_idx].len() {
                                let contour = &faces[face_idx][ci_a];
                                if ei_a < contour.len() {
                                    let ej = (ei_a + 1) % contour.len();
                                    let mid = (contour[ei_a] + contour[ej]) * 0.5;
                                    let mut best_dist = f64::INFINITY;
                                    let mut is_wall = false;
                                    for te in edges {
                                        let te_mid = (te.start + te.end) * 0.5;
                                        let d = (te_mid - mid).length();
                                        if d < best_dist {
                                            best_dist = d;
                                            is_wall = matches!(te.source, EdgeSource::Wall);
                                        }
                                    }
                                    if is_wall { continue; }
                                }
                            }
                        }
                    }
                    _ => { continue; }
                }
            }

            // Conflict check against all occupied stalls in the same face.
            let shrunk = shrink_toward_centroid(&quad.corners, 0.1);
            let dominated = occupied_by_face
                .get(&face_idx)
                .map_or(false, |occ| occ.iter().any(|p| polygons_overlap(&shrunk, p)));
            if dominated {
                continue;
            }

            // Accept this stall — add to occupied set.
            occupied_by_face.entry(face_idx).or_default().push(shrunk);
            result.push((quad, face_idx, *src_idx));
        }
    }

    result
}

/// Shrink polygon vertices toward centroid by a fixed distance (for overlap testing).
fn shrink_toward_centroid(corners: &[Vec2], amount: f64) -> Vec<Vec2> {
    let n = corners.len() as f64;
    let cx = corners.iter().map(|c| c.x).sum::<f64>() / n;
    let cy = corners.iter().map(|c| c.y).sum::<f64>() / n;
    let centroid = Vec2::new(cx, cy);
    corners
        .iter()
        .map(|c| {
            let d = *c - centroid;
            let len = d.length();
            if len < 1e-12 {
                return *c;
            }
            *c - d * (amount / len)
        })
        .collect()
}

/// Generate stalls from positive-space face extraction + per-edge spine shifting.
/// Returns (stalls, corridor_polygons, spine_lines, faces, miter_fills, skeleton_debugs, islands).
pub fn generate_from_spines(
    graph: &DriveAisleGraph,
    boundary: &Polygon,
    params: &ParkingParams,
    debug: &DebugToggles,
) -> (Vec<StallQuad>, Vec<Vec<Vec2>>, Vec<SpineLine>, Vec<Face>, Vec<Vec<Vec2>>, Vec<SkeletonDebug>, Vec<crate::types::Island>) {
    let stall_angle_rad = params.stall_angle_deg.to_radians();
    let effective_depth = params.stall_depth * stall_angle_rad.sin()
        + stall_angle_rad.cos() * params.stall_width / 2.0;
    let inset_d = compute_inset_d(params);
    let raw_outer = derive_raw_outer(&boundary.outer, inset_d, params.site_offset);
    let raw_holes = derive_raw_holes(&boundary.holes, inset_d);

    // Build deduplicated corridor rectangles + miter wedge fills, then
    // boolean-union them into merged shapes (preserving holes for loops).
    let dedup_corridors_with_flags = deduplicate_corridors(graph);
    let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors_with_flags.iter().map(|(p, _, _)| p.clone()).collect();
    let two_way_oriented_dirs: Vec<Option<Vec2>> = {
        let mut seen = std::collections::HashSet::new();
        let mut dirs = Vec::new();
        for edge in &graph.edges {
            let key = if edge.start < edge.end {
                (edge.start, edge.end)
            } else {
                (edge.end, edge.start)
            };
            if !seen.insert(key) {
                continue;
            }
            if edge.direction == AisleDirection::TwoWayOriented {
                dirs.push(Some((graph.vertices[edge.end] - graph.vertices[edge.start]).normalize()));
            } else {
                dirs.push(None);
            }
        }
        dirs
    };
    let miter_fills = generate_miter_fills(graph, debug);
    let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, graph, debug);

    // Flatten outer contours for rendering (aisle polygon overlay).
    let aisle_polygons: Vec<Vec<Vec2>> = merged_corridors
        .iter()
        .filter(|shape| !shape.is_empty())
        .map(|shape| shape[0].clone())
        .collect();

    // Extract faces by subtracting the merged corridor union from the
    // boundary. Since the union resolves all internal seams between
    // corridor rects and miter fills, no sliver artifacts remain.
    let faces = extract_faces(&raw_outer, &merged_corridors, &raw_holes);

    // Tag each face edge with its source corridor/wall for provenance.
    // Indexed by face_idx (parallel to `faces`); empty/small faces get a
    // default empty TaggedFace.
    let tagged_faces: Vec<TaggedFace> = faces.iter()
        .map(|shape| {
            if !shape.is_empty() && shape[0].len() >= 3 {
                tag_face_edges(shape, &merged_corridors, &dedup_corridors_with_flags, &two_way_oriented_dirs)
            } else {
                TaggedFace { edges: vec![], hole_edges: vec![], is_boundary: true, wall_edge_indices: vec![] }
            }
        })
        .collect();

    // Collect spines from all faces, then merge collinear segments across
    // face boundaries so stalls flow continuously along shared aisle edges.
    let mut raw_spines = Vec::new();
    let mut skeleton_debugs = Vec::new();
    // Track which faces produced any spine candidates (before filtering).
    // Faces with spine candidates were wide enough for stalls; faces without
    // are genuinely too narrow and should become islands if they have no stalls.
    let mut faces_with_spines = std::collections::HashSet::new();
    for (face_idx, shape) in faces.iter().enumerate() {
        // Optionally collect skeleton debug data for visualization.
        // Use multi-contour skeleton so arcs stay within the face.
        if debug.skeleton_debug && !shape.is_empty() && shape[0].len() >= 3 {
            let mut normalized = normalize_face_winding(shape);
            if debug.face_simplification {
                normalized = normalized.into_iter()
                    .map(|c| simplify_contour(&c, 0.035))
                    .filter(|c| c.len() >= 3)
                    .collect();
            }
            // Build edge weights matching the spine computation: aisle-facing
            // edges shrink (1.0), boundary edges stay fixed (0.0).
            let debug_weights: Vec<f64> = if face_idx < tagged_faces.len() && !debug.face_simplification {
                let tf = &tagged_faces[face_idx];
                // Match each normalized contour edge to its closest tagged edge.
                let mut weights = Vec::new();
                let classify_weights = |contour: &[Vec2], tagged_edges: &[FaceEdge]| -> Vec<f64> {
                    let n = contour.len();
                    (0..n).map(|i| {
                        let j = (i + 1) % n;
                        let mid = (contour[i] + contour[j]) * 0.5;
                        let mut best_dist = f64::INFINITY;
                        let mut best_aisle = false;
                        for te in tagged_edges {
                            let te_mid = (te.start + te.end) * 0.5;
                            let d = (te_mid - mid).length();
                            if d < best_dist {
                                best_dist = d;
                                best_aisle = matches!(te.source, EdgeSource::Aisle { .. });
                            }
                        }
                        if best_aisle { 1.0 } else { 0.0 }
                    }).collect()
                };
                if !normalized.is_empty() {
                    weights.extend(classify_weights(&normalized[0], &tf.edges));
                }
                for (ci, contour) in normalized.iter().enumerate().skip(1) {
                    let hi = ci - 1;
                    if hi < tf.hole_edges.len() {
                        weights.extend(classify_weights(contour, &tf.hole_edges[hi]));
                    } else {
                        weights.extend(vec![0.0; contour.len()]);
                    }
                }
                weights
            } else {
                normalized.iter().flat_map(|contour| {
                    let classified = classify_face_edges(contour, &merged_corridors, &dedup_corridors_with_flags, debug.edge_classification);
                    classified.into_iter().map(|(facing, _, _)| {
                        if facing { 1.0 } else { 0.0 }
                    })
                }).collect()
            };
            let sk = skeleton::compute_skeleton_multi(&normalized, &debug_weights);
            let sources: Vec<Vec2> = normalized.iter().flat_map(|c| c.iter().copied()).collect();
            skeleton_debugs.push(SkeletonDebug {
                arcs: sk.arcs.iter()
                    .filter(|&&(a, b)| {
                        point_in_or_on_face(a, shape, 0.5)
                            && point_in_or_on_face(b, shape, 0.5)
                    })
                    .map(|&(a, b)| [a, b])
                    .collect(),
                nodes: sk.nodes.iter().copied()
                    .filter(|n| point_in_or_on_face(*n, shape, 0.5))
                    .collect(),
                split_nodes: sk.split_nodes.iter().copied()
                    .filter(|n| point_in_or_on_face(*n, shape, 0.5))
                    .collect(),
                sources,
            });
        }
        let face_is_boundary = tagged_faces[face_idx].is_boundary;
        let tagged_ref = Some(&tagged_faces[face_idx]);
        let mut face_spines = compute_face_spines(shape, effective_depth, &merged_corridors, &dedup_corridors_with_flags, face_is_boundary, params, debug, &two_way_oriented_dirs, tagged_ref);
        if !face_spines.is_empty() {
            faces_with_spines.insert(face_idx);
        }
        for s in &mut face_spines {
            s.face_idx = face_idx;
        }
        let face_spines = if debug.spine_dedup {
            dedup_overlapping_spines(face_spines, 2.0)
        } else {
            face_spines
        };
        raw_spines.extend(face_spines);
    }
    let all_spines = if debug.spine_merging {
        merge_collinear_spines(raw_spines, 1.0)
    } else {
        raw_spines
    };

    let all_spines: Vec<SpineSegment> = if debug.short_spine_filter {
        let min_spine_len = effective_depth;
        all_spines
            .into_iter()
            .filter(|s| (s.end - s.start).length() >= min_spine_len)
            .collect()
    } else {
        all_spines
    };

    // Compute extension spines early so we know the full extended length
    // of each primary spine for centering. The extensions themselves will
    // be used later for greedy stall placement.
    let ext_spines_with_src = if debug.spine_extensions {
        extend_spines_to_faces(&all_spines, &faces, effective_depth, params)
    } else {
        vec![]
    };

    // Place stalls on merged spines (tagged with face_idx + spine_idx).
    // Pass extension spines so centering uses the full projected range.
    let ext_ref = if ext_spines_with_src.is_empty() { None } else { Some(ext_spines_with_src.as_slice()) };
    let (tagged_stalls_3, spine_shifts) = place_stalls_on_spines(
        &all_spines, params, debug.stall_centering, ext_ref,
    );

    // Convert to (StallQuad, usize) for clipping, preserving spine_idx.
    let tagged_stalls: Vec<(StallQuad, usize)> = tagged_stalls_3.iter()
        .map(|(s, fi, _)| (s.clone(), *fi))
        .collect();

    // Clip stalls to face interiors. A stall that protrudes past the face
    // boundary into a corridor (at miter-fill corners) is removed.
    let tagged_stalls = if debug.stall_face_clipping {
        clip_stalls_to_faces(tagged_stalls, &faces)
    } else {
        tagged_stalls
    };

    // Apply boundary clipping and conflict removal before island computation
    // so that corner gaps from removed conflicting stalls appear as islands.
    let tagged_stalls = if debug.boundary_clipping {
        clip_stalls_to_boundary(tagged_stalls, &raw_outer, &raw_holes)
    } else {
        tagged_stalls
    };
    let tagged_stalls = if debug.conflict_removal {
        // Build per-stall spine lengths by matching surviving stalls back
        // to their spine_idx via corner identity.
        let stall_spine_lengths: Vec<f64> = {
            let key_to_spine_len: std::collections::HashMap<[u64; 8], f64> = tagged_stalls_3
                .iter()
                .map(|(s, _, si)| {
                    let spine = &all_spines[*si];
                    (stall_key(s), (spine.end - spine.start).length())
                })
                .collect();
            tagged_stalls
                .iter()
                .map(|(s, _)| *key_to_spine_len.get(&stall_key(s)).unwrap_or(&0.0))
                .collect()
        };
        // Build per-face boundary flags.
        let face_boundary: Vec<bool> = tagged_faces.iter().map(|tf| tf.is_boundary).collect();
        remove_conflicting_stalls(tagged_stalls, &stall_spine_lengths, &face_boundary)
    } else {
        tagged_stalls
    };

    // Rebuild the 3-tuple list from surviving stalls to compute envelopes.
    // Match surviving stalls back to their spine_idx by corner identity.
    let surviving: std::collections::HashSet<[u64; 8]> = tagged_stalls.iter()
        .map(|(s, _)| stall_key(s))
        .collect();
    let surviving_3: Vec<(StallQuad, usize, usize)> = tagged_stalls_3.into_iter()
        .filter(|(s, _, _)| surviving.contains(&stall_key(s)))
        .collect();

    // --- Spine extension phase ---
    // Extend each spine colinearly to the face boundary and place additional
    // stalls on the extensions. Stalls are placed greedily in longest-spine-
    // first order: each candidate is checked against all already-placed stalls
    // (primary + earlier extensions) and silently skipped on conflict.
    let (ext_stalls_tagged, ext_spines, ext_spine_src_indices) = if debug.spine_extensions {
        let ext_tagged = place_extension_stalls_greedy(
            &ext_spines_with_src, &all_spines, &tagged_stalls, &spine_shifts, &faces, &raw_outer, &raw_holes, &merged_corridors, params, debug, &tagged_faces,
        );
        let (ext_spines, ext_spine_src_indices): (Vec<SpineSegment>, Vec<usize>) =
            ext_spines_with_src.into_iter().unzip();
        (ext_tagged, ext_spines, ext_spine_src_indices)
    } else {
        (vec![], vec![], vec![])
    };

    // Merge primary and extension stalls. Extension stalls carry their
    // source primary spine_idx so they join the same strip envelope.
    let ext_stalls_tagged_2: Vec<(StallQuad, usize)> = ext_stalls_tagged
        .iter()
        .map(|(s, fi, _)| (s.clone(), *fi))
        .collect();
    let mut all_tagged = tagged_stalls;
    all_tagged.extend(ext_stalls_tagged_2);

    // Build strip envelopes from primary + extension stalls grouped by spine.
    let mut all_surviving_3 = surviving_3;
    all_surviving_3.extend(ext_stalls_tagged);

    // --- Short segment filter ---
    // Remove all stalls belonging to spines with fewer than
    // MIN_STALLS_PER_SPINE stalls total (primary + extension combined).
    if debug.short_segment_filter {
        let mut counts: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
        for (_, _, spine_idx) in &all_surviving_3 {
            *counts.entry(*spine_idx).or_insert(0) += 1;
        }
        let keep_spines: std::collections::HashSet<usize> = counts
            .into_iter()
            .filter(|(_, count)| *count >= MIN_STALLS_PER_SPINE)
            .map(|(idx, _)| idx)
            .collect();
        all_surviving_3.retain(|(_, _, spine_idx)| keep_spines.contains(spine_idx));
        let surviving_keys: std::collections::HashSet<[u64; 8]> = all_surviving_3
            .iter()
            .map(|(s, _, _)| stall_key(s))
            .collect();
        all_tagged.retain(|(s, _)| surviving_keys.contains(&stall_key(s)));
    }

    // --- Island stall marking ---
    if params.island_stall_interval > 0 {
        mark_island_stalls(
            &mut all_surviving_3, &mut all_tagged,
            &all_spines, params,
        );
    }

    let all_stalls: Vec<StallQuad> = all_tagged.iter().map(|(s, _)| s.clone()).collect();
    let islands = compute_islands(&faces, &all_stalls, 10.0);

    // Build spine lines for visualization: primary + extension.
    // If the short-segment filter is active, only include spines that
    // still have stalls.
    let kept_spines: std::collections::HashSet<usize> = if debug.short_segment_filter {
        all_surviving_3.iter().map(|(_, _, si)| *si).collect()
    } else {
        (0..all_spines.len()).collect()
    };
    let mut spine_lines: Vec<SpineLine> = all_spines
        .iter()
        .enumerate()
        .filter(|(i, _)| kept_spines.contains(i))
        .map(|(_, s)| SpineLine {
            start: s.start,
            end: s.end,
            normal: s.outward_normal,
            is_extension: false,
        })
        .collect();
    spine_lines.extend(
        ext_spines.iter().zip(ext_spine_src_indices.iter())
            .filter(|(_, src_idx)| kept_spines.contains(src_idx))
            .map(|(s, _)| SpineLine {
                start: s.start,
                end: s.end,
                normal: s.outward_normal,
                is_extension: true,
            })
    );

    let face_list: Vec<Face> = faces
        .iter()
        .enumerate()
        .filter(|(_, shape)| !shape.is_empty() && shape[0].len() >= 3)
        .map(|(face_idx, _shape)| {
            let tf = &tagged_faces[face_idx];
            let source_label = |e: &FaceEdge| -> String {
                match &e.source {
                    EdgeSource::Wall => "wall".to_string(),
                    EdgeSource::Aisle { interior: true, .. } => "interior".to_string(),
                    EdgeSource::Aisle { interior: false, .. } => "perimeter".to_string(),
                }
            };
            let contour: Vec<Vec2> = tf.edges.iter().map(|e| e.start).collect();
            let edge_sources: Vec<String> = tf.edges.iter().map(&source_label).collect();
            let holes: Vec<Vec<Vec2>> = tf.hole_edges.iter()
                .map(|hole| hole.iter().map(|e| e.start).collect())
                .collect();
            let hole_edge_sources: Vec<Vec<String>> = tf.hole_edges.iter()
                .map(|hole| hole.iter().map(&source_label).collect())
                .collect();
            Face {
                contour,
                holes,
                is_boundary: tf.is_boundary,
                edge_sources,
                hole_edge_sources,
            }
        })
        .collect();

    (all_stalls, aisle_polygons, spine_lines, face_list, miter_fills, skeleton_debugs, islands)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aisle_graph::auto_generate;

    /// Test helper: create a corridor rectangle with one side along a→b.
    /// The face edge midpoint will lie exactly on this corridor edge.
    fn test_corridor_along(a: Vec2, b: Vec2) -> Vec<Vec<Vec2>> {
        let dir = (b - a).normalize();
        let perp = Vec2::new(-dir.y, dir.x);
        let w = 24.0;
        vec![vec![a, b, b + perp * w, a + perp * w]]
    }

    #[test]
    fn test_basic_rectangle_debug() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(300.0, 0.0),
                Vec2::new(300.0, 200.0),
                Vec2::new(0.0, 200.0),
            ],
            holes: vec![],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let graph = auto_generate(&boundary, &params, &[], &[]);

        let (stalls, _, spines, _, _, _, _) = generate_from_spines(&graph, &boundary, &params, &DebugToggles::default());
        eprintln!("\nTotal stalls: {}", stalls.len());
        eprintln!("Total spines: {}", spines.len());
        assert!(stalls.len() >= 50);
    }

    #[test]
    fn test_attempt_debug() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(227.0, 24.33),
                Vec2::new(300.0, 200.0),
                Vec2::new(0.0, 200.0),
            ],
            holes: vec![],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let graph = auto_generate(&boundary, &params, &[], &[]);

        eprintln!("\n=== Attempt polygon debug ===");
        eprintln!("Graph: {} vertices, {} edges", graph.vertices.len(), graph.edges.len());
        for (i, v) in graph.vertices.iter().enumerate() {
            eprintln!("  v{}: ({:.1}, {:.1})", i, v.x, v.y);
        }
        for (i, e) in graph.edges.iter().enumerate() {
            let s = graph.vertices[e.start];
            let end = graph.vertices[e.end];
            eprintln!("  e{}: v{}→v{} ({:.1},{:.1})→({:.1},{:.1}) w={:.1}",
                i, e.start, e.end, s.x, s.y, end.x, end.y, e.width);
        }

        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();
        let dedup_corridors = deduplicate_corridors(&graph);

        eprintln!("\nMiter fills:");
        let miter_fills = generate_miter_fills(&graph, &DebugToggles::default());
        for (i, fill) in miter_fills.iter().enumerate() {
            let area = signed_area(fill).abs();
            eprintln!("  fill {}: area={:.1}, {} verts", i, area, fill.len());
            for (vi, v) in fill.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();

        eprintln!("\nCorridor rects:");
        for (i, c) in dedup_corridor_polys.iter().enumerate() {
            eprintln!("  rect {}: {} verts", i, c.len());
            for (vi, v) in c.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &DebugToggles::default());

        eprintln!("\nMerged corridors: {}", merged_corridors.len());
        for (ci, shape) in merged_corridors.iter().enumerate() {
            eprintln!("  corridor shape {}: {} contours", ci, shape.len());
            for (ki, contour) in shape.iter().enumerate() {
                let area = signed_area(contour);
                eprintln!("    contour {}: {} verts, signed_area={:.0}", ki, contour.len(), area);
                for (vi, v) in contour.iter().enumerate() {
                    eprintln!("      v{}: ({:.1}, {:.1})", vi, v.x, v.y);
                }
            }
        }

        let faces = extract_faces(&boundary.outer, &merged_corridors, &[]);
        eprintln!("\nFaces: {}", faces.len());
        for (fi, shape) in faces.iter().enumerate() {
            let outer = &shape[0];
            let area = signed_area(outer).abs();
            eprintln!("  face {}: {} contours, outer={} verts, area={:.0}", fi, shape.len(), outer.len(), area);
            for (ki, contour) in shape.iter().enumerate() {
                let sa = signed_area(contour);
                let label = if ki == 0 { "outer" } else { "hole" };
                eprintln!("    {} {}: {} verts, signed_area={:.0}", label, ki, contour.len(), sa);
                for (vi, v) in contour.iter().enumerate() {
                    eprintln!("      v{}: ({:.1}, {:.1})", vi, v.x, v.y);
                }
            }

            let face_spines = compute_face_spines(shape, effective_depth, &merged_corridors, &dedup_corridors, false, &params, &DebugToggles::default(), &[], None);
            eprintln!("  spines from face {}: {}", fi, face_spines.len());
            for (si, s) in face_spines.iter().enumerate() {
                let len = (s.end - s.start).length();
                eprintln!("    spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} n=({:.2},{:.2})",
                    si, s.start.x, s.start.y, s.end.x, s.end.y, len,
                    s.outward_normal.x, s.outward_normal.y);
            }
        }
    }

    /// Diagnostic: check face 0's boolean Difference for the default rect boundary.
    #[test]
    fn test_face0_difference_shapes() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(750.0, 0.0),
                Vec2::new(750.0, 500.0),
                Vec2::new(0.0, 500.0),
            ],
            holes: vec![vec![
                Vec2::new(275.0, 150.0),
                Vec2::new(475.0, 150.0),
                Vec2::new(375.0, 350.0),
            ]],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let debug = DebugToggles::default();

        let (stalls, _, _, faces_out, _, _, islands) =
            generate_from_spines(
                &crate::aisle_graph::auto_generate(&boundary, &params, &[], &[]),
                &boundary, &params, &debug,
            );

        eprintln!("Total stalls: {}, islands: {}", stalls.len(), islands.len());
        for (ii, isl) in islands.iter().enumerate() {
            let a = signed_area(&isl.contour).abs();
            eprintln!("  island {}: face={}, area={:.0}, verts={}", ii, isl.face_idx, a, isl.contour.len());
        }

        // Check: any island from face 0?
        let face0_islands: Vec<_> = islands.iter().filter(|i| i.face_idx == 0).collect();
        eprintln!("Face 0 islands: {}", face0_islands.len());

        // Check: any island near boundary corners?
        let corners = [(5.0, 5.0), (745.0, 5.0), (745.0, 495.0), (5.0, 495.0)];
        for (cx, cy) in &corners {
            let near = islands.iter().any(|isl|
                isl.contour.iter().any(|v| (v.x - cx).abs() < 20.0 && (v.y - cy).abs() < 20.0)
            );
            eprintln!("  corner ({}, {}): island nearby = {}", cx, cy, near);
        }
    }

    /// Islands test: verify that compute_islands returns small gap polygons,
    /// not the full face, for a face with stalls placed in it.
    #[test]
    fn test_islands_are_gaps_not_full_face() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(750.0, 0.0),
                Vec2::new(602.62, 475.41),
                Vec2::new(0.0, 500.0),
            ],
            holes: vec![vec![
                Vec2::new(275.0, 150.0),
                Vec2::new(475.0, 150.0),
                Vec2::new(375.0, 350.0),
            ]],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let debug = DebugToggles::default();
        let graph = auto_generate(&boundary, &params, &[], &[]);

        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();
        let dedup_corridors = deduplicate_corridors(&graph);
        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();
        let miter_fills = generate_miter_fills(&graph, &debug);
        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &debug);
        let faces = extract_faces(&boundary.outer, &merged_corridors, &[]);

        // Run through the full spine+stall pipeline.
        let mut raw_spines = Vec::new();
        let mut faces_with_spines = std::collections::HashSet::new();
        for (face_idx, shape) in faces.iter().enumerate() {
            let mut face_spines = compute_face_spines(shape, effective_depth, &merged_corridors, &dedup_corridors, false, &params, &debug, &[], None);
            if !face_spines.is_empty() {
                faces_with_spines.insert(face_idx);
            }
            for s in &mut face_spines {
                s.face_idx = face_idx;
            }
            let face_spines = dedup_overlapping_spines(face_spines, 2.0);
            raw_spines.extend(face_spines);
        }
        let all_spines = merge_collinear_spines(raw_spines, 1.0);
        let all_spines: Vec<SpineSegment> = all_spines.into_iter()
            .filter(|s| (s.end - s.start).length() >= effective_depth)
            .collect();

        let (tagged_stalls_3, _) = place_stalls_on_spines(&all_spines, &params, true, None);
        let tagged_stalls: Vec<(StallQuad, usize)> = tagged_stalls_3.iter()
            .map(|(s, fi, _)| (s.clone(), *fi)).collect();
        let tagged_stalls = clip_stalls_to_faces(tagged_stalls, &faces);
        let tagged_stalls = clip_stalls_to_boundary(tagged_stalls, &boundary.outer, &[]);
        let tagged_stalls = remove_conflicting_stalls(tagged_stalls, &[], &[]);
        let all_stalls: Vec<StallQuad> = tagged_stalls.iter().map(|(s, _)| s.clone()).collect();
        let islands = compute_islands(&faces, &all_stalls, 10.0);

        // No island should have area close to the boundary or the hole.
        // No island should be as large as the full boundary or the hole itself.
        let boundary_area = signed_area(&boundary.outer).abs();
        let hole_area = signed_area(&boundary.holes[0]).abs();
        for (ii, island) in islands.iter().enumerate() {
            let outer_a = signed_area(&island.contour).abs();
            let holes_a: f64 = island.holes.iter().map(|h| signed_area(h).abs()).sum();
            let net = outer_a - holes_a;
            assert!(net < boundary_area * 0.25,
                "island {} has net area {:.1} ≥ 25% of boundary area {:.1}",
                ii, net, boundary_area);
        }
    }


    /// Two aisle edges meeting at an acute corner: unmerged corridor
    /// rectangles produce a jagged overlap (8 vertices from 2 rects),
    /// while the boolean union should produce a single merged polygon
    /// with a clean mitered corner (fewer vertices, single shape).
    #[test]
    fn test_corridor_merge_acute_corner() {
        // Two edges meeting at a ~45° angle at vertex 1.
        //   v0 ---- v1
        //             \
        //              v2
        let vertices = vec![
            Vec2::new(0.0, 100.0),   // v0: left
            Vec2::new(100.0, 100.0), // v1: corner
            Vec2::new(200.0, 0.0),   // v2: bottom-right
        ];

        let w = 12.0; // half-width
        let edge_a = AisleEdge {
            start: 0,
            end: 1,
            width: w,
            interior: false,
            direction: AisleDirection::default(),
        };
        let edge_b = AisleEdge {
            start: 1,
            end: 2,
            width: w,
            interior: false,
            direction: AisleDirection::default(),
        };

        let rect_a = corridor_polygon(&vertices, &edge_a);
        let rect_b = corridor_polygon(&vertices, &edge_b);

        // Unmerged: two separate 4-vertex rectangles.
        assert_eq!(rect_a.len(), 4);
        assert_eq!(rect_b.len(), 4);

        // Merged: should produce a single polygon with a clean mitered
        // corner — fewer total vertices than 2 separate rectangles.
        let graph = DriveAisleGraph {
            vertices: vertices.clone(),
            edges: vec![edge_a.clone(), edge_b.clone()],
            perim_vertex_count: 3,
        };
        let merged = merge_corridor_shapes(&[rect_a.clone(), rect_b.clone()], &graph, &DebugToggles::default());

        eprintln!("\nUnmerged: 2 rects × 4 verts = 8 verts");
        eprintln!("Merged: {} shape(s)", merged.len());
        for (i, shape) in merged.iter().enumerate() {
            if shape.is_empty() { continue; }
            let poly = &shape[0];
            let area = signed_area(poly).abs();
            eprintln!(
                "  shape {}: {} verts (outer), {} holes, area={:.0}",
                i,
                poly.len(),
                shape.len() - 1,
                area
            );
            for (vi, v) in poly.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        // The two rectangles overlap at the corner, so the union should
        // produce exactly 1 merged shape (not 2 separate ones).
        assert_eq!(merged.len(), 1, "overlapping corridors should merge into 1 shape");
    }

    /// Trapezoidal face: horizontal bottom (aisle), diagonal top (aisle),
    /// vertical sides (boundary).
    ///
    /// Face shape (CCW):
    ///
    ///   v3 -------- v2        diagonal top edge (aisle-facing)
    ///   |            \
    ///   |             \
    ///   v0 ----------- v1     horizontal bottom edge (aisle-facing)
    ///
    /// The skeleton should partition the face so that each aisle edge's
    /// stalls stay in their own region. The right side of the face is
    /// narrower than the left, so the bottom spine should be clipped
    /// where it meets the diagonal's territory.
    #[test]
    fn test_trapezoid_face_spines() {
        let face_contour = vec![
            Vec2::new(0.0, 0.0),     // v0: bottom-left
            Vec2::new(200.0, 0.0),   // v1: bottom-right
            Vec2::new(200.0, 20.0),  // v2: top-right (lower)
            Vec2::new(0.0, 60.0),    // v3: top-left (higher)
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (bottom and top diagonal).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[1]),
            test_corridor_along(face_contour[2], face_contour[3]),
        ];

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &params, &DebugToggles::default(), &[], None);

        eprintln!("\n=== Trapezoid face spine test ===");
        eprintln!("effective_depth = {:.1}", effective_depth);
        eprintln!("spines generated: {}", spines.len());
        for (i, s) in spines.iter().enumerate() {
            let len = (s.end - s.start).length();
            eprintln!(
                "  spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} normal=({:.2},{:.2})",
                i, s.start.x, s.start.y, s.end.x, s.end.y, len,
                s.outward_normal.x, s.outward_normal.y
            );
        }

        assert!(
            !spines.is_empty(),
            "trapezoid face should produce at least one spine"
        );

        // The two spines should meet at a skeleton node — no overlap.
        // Both spines should exist (face is wide enough on the left for
        // two rows: 60 > 2×18 = 36).
        assert_eq!(
            spines.len(),
            2,
            "should have 2 spines (bottom horizontal + top diagonal)"
        );

        // Place stalls and verify ALL stall corners are inside the face.
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.0.corners.iter().enumerate() {
                if !point_in_face(*corner, &shape) {
                    let min_dist = face_contour
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            let j = (i + 1) % face_contour.len();
                            point_to_segment_dist(*corner, face_contour[i], face_contour[j])
                        })
                        .fold(f64::INFINITY, f64::min);
                    if min_dist > 1.0 {
                        eprintln!(
                            "  OUTSIDE: stall {} corner {} at ({:.1},{:.1}), dist_to_edge={:.1}",
                            si, ci, corner.x, corner.y, min_dist
                        );
                        outside_count += 1;
                    }
                }
            }
        }
        assert_eq!(
            outside_count, 0,
            "all stall corners must be inside the face (or within 1ft tolerance)"
        );
    }

    /// Pentagon face: two parallel aisle edges, flat base, pointed cap.
    /// This is the original flickering case — the skeleton should always
    /// produce spines for both long parallel edges.
    ///
    ///   v0 ——————————— v1
    ///   |                \
    ///   |                 v2    ← cap
    ///   |                /
    ///   v4 ——————————— v3
    #[test]
    fn test_pentagon_face_spines() {
        let face_contour = vec![
            Vec2::new(0.0, 60.0),    // v0: top-left
            Vec2::new(180.0, 60.0),  // v1: top-right
            Vec2::new(200.0, 30.0),  // v2: cap point
            Vec2::new(180.0, 0.0),   // v3: bottom-right
            Vec2::new(0.0, 0.0),     // v4: bottom-left
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (top and bottom).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[1]),
            test_corridor_along(face_contour[3], face_contour[4]),
        ];

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &params, &DebugToggles::default(), &[], None);

        eprintln!("\n=== Pentagon face spine test ===");
        eprintln!("effective_depth = {:.1}", effective_depth);
        eprintln!("spines generated: {}", spines.len());
        for (i, s) in spines.iter().enumerate() {
            let len = (s.end - s.start).length();
            eprintln!(
                "  spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} normal=({:.2},{:.2})",
                i, s.start.x, s.start.y, s.end.x, s.end.y, len,
                s.outward_normal.x, s.outward_normal.y
            );
        }

        // Must always produce spines (no flickering to zero).
        assert!(
            !spines.is_empty(),
            "pentagon face must always produce spines"
        );

        // Should have at least one spine for each parallel aisle edge
        // (top and bottom), meeting at a skeleton node near the cap.
        let top_spines: Vec<&SpineSegment> =
            spines.iter().filter(|s| s.outward_normal.y > 0.5).collect();
        let bottom_spines: Vec<&SpineSegment> =
            spines.iter().filter(|s| s.outward_normal.y < -0.5).collect();

        eprintln!(
            "top spines: {}, bottom spines: {}",
            top_spines.len(),
            bottom_spines.len()
        );

        // The face is 60ft tall and stalls are 18ft deep. Two rows of 18
        // = 36ft < 60ft, so both spines should exist.
        assert!(
            !top_spines.is_empty(),
            "should have a top spine (aisle along v0→v1)"
        );
        assert!(
            !bottom_spines.is_empty(),
            "should have a bottom spine (aisle along v3→v4)"
        );

        // Place stalls and verify containment.
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.0.corners.iter().enumerate() {
                if !point_in_face(*corner, &shape) {
                    let min_dist = face_contour
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            let j = (i + 1) % face_contour.len();
                            point_to_segment_dist(*corner, face_contour[i], face_contour[j])
                        })
                        .fold(f64::INFINITY, f64::min);
                    if min_dist > 1.0 {
                        eprintln!(
                            "  OUTSIDE: stall {} corner {} at ({:.1},{:.1}), dist_to_edge={:.1}",
                            si, ci, corner.x, corner.y, min_dist
                        );
                        outside_count += 1;
                    }
                }
            }
        }
        assert_eq!(
            outside_count, 0,
            "all stall corners must be inside the face (or within 1ft tolerance)"
        );
    }

    /// Narrow face: two parallel aisle edges only 30ft apart (< 2×18=36ft).
    /// The normal skeleton collapses, but edge-suppression produces
    /// single-sided spines — one per aisle-facing edge.
    #[test]
    fn test_narrow_face_single_sided_spines() {
        let face_contour = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(200.0, 0.0),
            Vec2::new(200.0, 30.0),
            Vec2::new(0.0, 30.0),
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (bottom and top).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[1]),
            test_corridor_along(face_contour[2], face_contour[3]),
        ];

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let effective_depth = params.stall_depth
            * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &params, &DebugToggles::default(), &[], None);
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();

        eprintln!("\n=== Narrow face test (30ft between aisles) ===");
        eprintln!("spines: {}, stalls: {}", spines.len(), stalls.len());

        assert_eq!(spines.len(), 2, "edge-suppression should produce one spine per aisle edge");
        assert!(stalls.len() > 0, "narrow face should have single-sided stalls");
    }

    /// Face between left/right aisle corridors with a diagonal boundary
    /// at the top. The vertical spines run between the corridors, with
    /// stalls going left and right. Near the diagonal top, the face
    /// narrows — stalls there must not extend outside the face.
    ///
    ///     v3 --------- v2      ← diagonal boundary (top)
    ///    /               |
    ///   /                |
    ///  v0 ------------- v1     ← horizontal boundary (bottom)
    ///
    /// Left edge (v0→v3): aisle-facing (corridor on the left)
    /// Right edge (v1→v2): aisle-facing (corridor on the right)
    /// Bottom (v0→v1) and top diagonal (v2→v3): boundary
    ///
    /// The face is 80ft wide × 100ft (left) / 120ft (right) tall.
    /// The left spine (x=18) and right spine (x=62) each produce stalls.
    /// Near the top where the diagonal cuts across, the last stalls
    /// must not poke outside the face.
    #[test]
    fn test_vertical_spines_diagonal_top() {
        let face_contour = vec![
            Vec2::new(0.0, 0.0),      // v0: bottom-left
            Vec2::new(80.0, 0.0),     // v1: bottom-right
            Vec2::new(80.0, 120.0),   // v2: top-right
            Vec2::new(0.0, 100.0),    // v3: top-left
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (left and right verticals).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[3]),
            test_corridor_along(face_contour[1], face_contour[2]),
        ];

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let effective_depth =
            params.stall_depth * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &params, &DebugToggles::default(), &[], None);

        eprintln!("\n=== Vertical spines + diagonal top test ===");
        eprintln!("effective_depth = {:.1}", effective_depth);
        eprintln!("spines generated: {}", spines.len());
        for (i, s) in spines.iter().enumerate() {
            let len = (s.end - s.start).length();
            eprintln!(
                "  spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} normal=({:.2},{:.2})",
                i, s.start.x, s.start.y, s.end.x, s.end.y, len,
                s.outward_normal.x, s.outward_normal.y
            );
        }

        assert!(
            !spines.is_empty(),
            "should produce vertical spines from left/right aisle edges"
        );

        // Place stalls and verify ALL corners are inside the face.
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.0.corners.iter().enumerate() {
                if !point_in_face(*corner, &shape) {
                    let min_dist = face_contour
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            let j = (i + 1) % face_contour.len();
                            point_to_segment_dist(*corner, face_contour[i], face_contour[j])
                        })
                        .fold(f64::INFINITY, f64::min);
                    if min_dist > 1.0 {
                        eprintln!(
                            "  OUTSIDE: stall {} corner {} at ({:.1},{:.1}), dist={:.1}",
                            si, ci, corner.x, corner.y, min_dist
                        );
                        outside_count += 1;
                    }
                }
            }
        }
        assert_eq!(
            outside_count, 0,
            "stalls near the diagonal top must not extend outside the face"
        );
    }

    /// Debug: reproduce the slanted-boundary layout from attempt.txt and
    /// trace why the rightmost face has no stalls.
    #[test]
    fn test_attempt_boundary_debug() {
        use crate::generate::generate;
        use crate::types::GenerateInput;

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(130.67, 102.33),
                    Vec2::new(300.0, 200.0),
                    Vec2::new(0.0, 200.0),
                ],
                holes: vec![],
                ..Default::default()
            },
            aisle_graph: None,
            drive_lines: vec![],
            annotations: vec![],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
        };

        let layout = generate(input.clone());
        eprintln!("\n=== attempt boundary debug ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Recompute corridor shapes for debug classification.
        let graph = auto_generate(&input.boundary, &input.params, &[], &[]);
        let dedup_corridors = deduplicate_corridors(&graph);
        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();
        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &DebugToggles::default());

        let effective_depth = input.params.stall_depth
            * input.params.stall_angle_deg.to_radians().sin();

        for (fi, face) in layout.faces.iter().enumerate() {
            let contour = &face.contour;
            let area = crate::inset::signed_area(contour).abs();
            let perimeter: f64 = contour.iter().enumerate().map(|(i, v)| {
                let next = &contour[(i + 1) % contour.len()];
                (*next - *v).length()
            }).sum();
            let min_width = if perimeter > 0.0 { 4.0 * area / perimeter } else { 0.0 };
            eprintln!("\n  face {}: {} vertices, area={:.1}, perimeter={:.1}, min_width={:.1} (ed={:.1})",
                fi, contour.len(), area, perimeter, min_width, effective_depth);
            for (vi, v) in contour.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }

            let classified = classify_face_edges(contour, &merged_corridors, &dedup_corridors, true);
            let ed = effective_depth - 0.05;
            let min_edge_len = effective_depth * 2.0;
            for i in 0..contour.len() {
                let j = (i + 1) % contour.len();
                let edge_len = (contour[j] - contour[i]).length();
                let (af, _, _) = classified[i];
                let dist = if af && edge_len >= min_edge_len { ed } else { 0.0 };
                eprintln!(
                    "    edge {}->{}: len={:.1} aisle_facing={} dist={:.1}{}",
                    i, j, edge_len, af, dist,
                    if !af { " (boundary)" }
                    else if edge_len < min_edge_len { " (too short)" }
                    else { "" }
                );
            }

            for (hi, hole) in face.holes.iter().enumerate() {
                eprintln!("    hole {}: {} verts", hi, hole.len());
                let hole_classified = classify_face_edges(hole, &merged_corridors, &dedup_corridors, true);
                for i in 0..hole.len() {
                    let j = (i + 1) % hole.len();
                    let elen = (hole[j] - hole[i]).length();
                    let (af, _, _) = hole_classified[i];
                    eprintln!("      edge {}->{}: len={:.1} af={} ({:.1},{:.1})->({:.1},{:.1})",
                        i, j, elen, af, hole[i].x, hole[i].y, hole[j].x, hole[j].y);
                }
            }
            let mut shape = vec![contour.clone()];
            shape.extend(face.holes.iter().cloned());
            let spines = compute_face_spines(&shape, effective_depth, &merged_corridors, &dedup_corridors, false, &input.params, &DebugToggles::default(), &[], None);
            eprintln!("    spines: {}", spines.len());
            for (si, s) in spines.iter().enumerate() {
                let len = (s.end - s.start).length();
                eprintln!(
                    "      spine {}: ({:.1},{:.1})->({:.1},{:.1}) len={:.1} n=({:.2},{:.2})",
                    si, s.start.x, s.start.y, s.end.x, s.end.y, len,
                    s.outward_normal.x, s.outward_normal.y
                );
            }
        }
    }

    #[test]
    fn test_slanted_boundary_debug() {
        use crate::generate::generate;
        use crate::types::GenerateInput;

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(429.0, 1.67),
                    Vec2::new(300.0, 200.0),
                    Vec2::new(0.0, 200.0),
                ],
                holes: vec![],
                ..Default::default()
            },
            aisle_graph: None,
            drive_lines: vec![],
            annotations: vec![],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
        };

        let layout = generate(input.clone());
        eprintln!("\n=== slanted boundary debug ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Recompute corridor shapes for debug classification.
        let graph = auto_generate(&input.boundary, &input.params, &[], &[]);
        let dedup_corridors = deduplicate_corridors(&graph);
        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();
        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &DebugToggles::default());

        let effective_depth = input.params.stall_depth
            * input.params.stall_angle_deg.to_radians().sin();

        // Examine each face.
        for (fi, face) in layout.faces.iter().enumerate() {
            let contour = &face.contour;
            let area = crate::inset::signed_area(contour).abs();
            eprintln!("\n  face {}: {} vertices, area={:.1}", fi, contour.len(), area);
            for (vi, v) in contour.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }

            // Classify edges for this face.
            let classified = classify_face_edges(contour, &merged_corridors, &dedup_corridors, true);
            let ed = effective_depth - 0.05;
            let min_edge_len = effective_depth * 2.0;
            for i in 0..contour.len() {
                let j = (i + 1) % contour.len();
                let edge_len = (contour[j] - contour[i]).length();
                let (af, _, _) = classified[i];
                let dist = if af && edge_len >= min_edge_len { ed } else { 0.0 };
                eprintln!(
                    "    edge {}->{}: len={:.1} aisle_facing={} dist={:.1}{}",
                    i, j, edge_len, af, dist,
                    if !af { " (boundary)" }
                    else if edge_len < min_edge_len { " (too short)" }
                    else { "" }
                );
            }

            // Print holes.
            eprintln!("    holes: {}", face.holes.len());
            for (hi, hole) in face.holes.iter().enumerate() {
                eprintln!("    hole {}: {} vertices", hi, hole.len());
                for (vi, v) in hole.iter().enumerate() {
                    eprintln!("      v{}: ({:.1}, {:.1})", vi, v.x, v.y);
                }
            }

            // Compute spines for this face using full shape (contour + holes).
            let mut shape = vec![contour.clone()];
            shape.extend(face.holes.iter().cloned());
            let spines = compute_face_spines(&shape, effective_depth, &merged_corridors, &dedup_corridors, false, &input.params, &DebugToggles::default(), &[], None);
            eprintln!("    spines: {}", spines.len());
            for (si, s) in spines.iter().enumerate() {
                let len = (s.end - s.start).length();
                eprintln!(
                    "      spine {}: ({:.1},{:.1})->({:.1},{:.1}) len={:.1} n=({:.2},{:.2})",
                    si, s.start.x, s.start.y, s.end.x, s.end.y, len,
                    s.outward_normal.x, s.outward_normal.y
                );
            }
        }
    }

    #[test]
    fn test_drive_line_horizontal_with_hole() {
        use crate::generate::generate;
        use crate::types::{GenerateInput, DriveLine};

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(750.0, 0.0),
                    Vec2::new(750.0, 500.0),
                    Vec2::new(0.0, 500.0),
                ],
                holes: vec![vec![
                    Vec2::new(275.0, 150.0),
                    Vec2::new(475.0, 150.0),
                    Vec2::new(375.0, 350.0),
                ]],
                ..Default::default()
            },
            aisle_graph: None,
            drive_lines: vec![
                DriveLine {
                    start: Vec2::new(-50.0, 250.0),
                    end: Vec2::new(800.0, 250.0),
                hole_pin: None, id: 0, partitions: false, },
            ],
            annotations: vec![],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
        };

        let layout = generate(input);
        eprintln!("\n=== drive line horizontal with hole ===");
        eprintln!("stalls: {}, faces: {}, miter_fills: {}",
            layout.stalls.len(), layout.faces.len(), layout.miter_fills.len());
        eprintln!("resolved graph: {} vertices, {} edges",
            layout.resolved_graph.vertices.len(), layout.resolved_graph.edges.len());

        // The drive line should produce interior segments that avoid the hole.
        // There should be corridors and miter fills at the junction points.
        assert!(layout.stalls.len() > 0, "should produce stalls");
        assert!(layout.miter_fills.len() > 0, "should produce miter fills");
        assert!(layout.faces.len() > 0, "should produce faces");
    }

    /// Test that TwoWayOriented aisles assign per-side travel_dir to spines.
    ///
    /// Setup: a 200×300 rectangle with a vertical aisle down the middle.
    /// The aisle edge goes from (100,0) to (100,300) and is marked
    /// TwoWayOriented with travel_dir pointing downward (0,1).
    ///
    /// Expected: the spine on the RIGHT side of travel_dir (outward normal
    /// pointing right, i.e. +x) should get travel_dir = (0,1) (downward,
    /// matching the right lane). The spine on the LEFT side (outward normal
    /// pointing left, i.e. -x) should get travel_dir = (0,-1) (upward,
    /// the oncoming lane).
    ///
    /// For comparison, a OneWay aisle should give BOTH sides the same
    /// travel_dir.
    #[test]
    fn test_two_way_oriented_per_side_travel_dir() {
        // Build a minimal graph: vertical aisle at x=100 in a 200×300 box.
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(200.0, 0.0),
                Vec2::new(200.0, 300.0),
                Vec2::new(0.0, 300.0),
            ],
            holes: vec![],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let hw = params.aisle_width / 2.0;

        // Vertical aisle: 4 perimeter vertices + 2 interior vertices at
        // top and bottom of the aisle centerline.
        let vertices = vec![
            Vec2::new(0.0, 0.0),     // 0: bottom-left
            Vec2::new(200.0, 0.0),   // 1: bottom-right
            Vec2::new(200.0, 300.0), // 2: top-right
            Vec2::new(0.0, 300.0),   // 3: top-left
            Vec2::new(100.0, 0.0),   // 4: aisle bottom
            Vec2::new(100.0, 300.0), // 5: aisle top
        ];

        // Helper to build a graph with a specific direction for the interior edge.
        let make_graph = |direction: AisleDirection| -> DriveAisleGraph {
            DriveAisleGraph {
                vertices: vertices.clone(),
                edges: vec![
                    // Perimeter edges
                    AisleEdge { start: 0, end: 1, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 1, end: 2, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 2, end: 3, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 3, end: 0, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    // Perimeter connections to aisle
                    AisleEdge { start: 0, end: 4, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 1, end: 4, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 2, end: 5, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 3, end: 5, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    // Interior aisle edge: 4→5 (bottom to top, i.e. travel_dir = (0,1) upward)
                    AisleEdge { start: 4, end: 5, width: hw, interior: true, direction: direction.clone() },
                    // Reverse edge for two-way
                    AisleEdge { start: 5, end: 4, width: hw, interior: true, direction: direction },
                ],
                perim_vertex_count: 4,
            }
        };

        let debug = DebugToggles::default();

        // --- Test TwoWayOriented ---
        let graph = make_graph(AisleDirection::TwoWayOriented);
        let (stalls, _, spine_lines, faces, _, _, _) = generate_from_spines(&graph, &boundary, &params, &debug);

        eprintln!("\n=== TwoWayOriented test ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", stalls.len(), spine_lines.len(), faces.len());

        // Rebuild spines with travel_dir info (spine_lines loses it).
        // Use the internal pipeline directly.
        let dedup = deduplicate_corridors(&graph);
        let dedup_polys: Vec<Vec<Vec2>> = dedup.iter().map(|(p, _, _)| p.clone()).collect();
        let merged = merge_corridor_shapes(&dedup_polys, &graph, &debug);
        let two_way_dirs: Vec<Option<Vec2>> = {
            let mut seen = std::collections::HashSet::new();
            let mut dirs = Vec::new();
            for edge in &graph.edges {
                let key = if edge.start < edge.end { (edge.start, edge.end) } else { (edge.end, edge.start) };
                if !seen.insert(key) { continue; }
                if edge.direction == AisleDirection::TwoWayOriented {
                    dirs.push(Some((graph.vertices[edge.end] - graph.vertices[edge.start]).normalize()));
                } else {
                    dirs.push(None);
                }
            }
            dirs
        };
        let extracted_faces = extract_faces(&boundary.outer, &merged, &[]);

        let ed = params.stall_depth
            + params.stall_depth * (params.stall_angle_deg * std::f64::consts::PI / 180.0).sin()
            + (params.stall_angle_deg * std::f64::consts::PI / 180.0).cos() * params.stall_width / 2.0;

        let mut left_spines = Vec::new();
        let mut right_spines = Vec::new();
        for shape in &extracted_faces {
            let face_is_boundary = is_boundary_face(&shape[0], &merged, &dedup);
            let spines = compute_face_spines(shape, ed, &merged, &dedup, face_is_boundary, &params, &debug, &two_way_dirs, None);
            for s in &spines {
                if let Some(td) = s.travel_dir {
                    eprintln!(
                        "  spine: outward=({:.1},{:.1}) travel_dir=({:.1},{:.1}) start=({:.0},{:.0}) end=({:.0},{:.0})",
                        s.outward_normal.x, s.outward_normal.y,
                        td.x, td.y,
                        s.start.x, s.start.y, s.end.x, s.end.y,
                    );
                    // Classify: is outward pointing right (+x) or left (-x)?
                    if s.outward_normal.x > 0.5 {
                        right_spines.push(td);
                    } else if s.outward_normal.x < -0.5 {
                        left_spines.push(td);
                    }
                }
            }
        }

        // The interior aisle goes from vertex 4 (100,0) to vertex 5 (100,300).
        // travel_dir = normalize(v5 - v4) = (0, 1).
        //
        // For TwoWayOriented, both sides get the SAME travel_dir (like
        // OneWay). The existing flip_angle logic in place_stalls_on_spines
        // already differentiates stall lean per-side based on the spine's
        // orientation relative to travel_dir.
        eprintln!("right_spines travel_dirs: {:?}", right_spines);
        eprintln!("left_spines travel_dirs: {:?}", left_spines);

        assert!(!right_spines.is_empty(), "should have spines on right side of aisle");
        assert!(!left_spines.is_empty(), "should have spines on left side of aisle");

        // For TwoWayOriented, the "canonical negative" side (outward.x < 0)
        // gets its travel_dir flipped. The "canonical positive" side keeps it.
        // This makes both sides produce the same flip_angle in
        // place_stalls_on_spines, giving correct per-lane stall angles.
        // Right spines (outward.x > 0 = positive side): keep td = (0,1)
        for td in &right_spines {
            assert!(td.y > 0.5, "positive-side travel_dir should stay (0,1), got ({:.2},{:.2})", td.x, td.y);
        }
        // Left spines (outward.x < 0 = negative side): flipped to (0,-1)
        for td in &left_spines {
            assert!(td.y < -0.5, "negative-side travel_dir should be flipped to (0,-1), got ({:.2},{:.2})", td.x, td.y);
        }
    }

    /// End-to-end test: use the full generate pipeline with a TwoWayOriented
    /// annotation on an auto-generated graph (simulating the UI flow).
    #[test]
    fn test_two_way_oriented_annotation_e2e() {
        use crate::generate::generate;
        use crate::types::{GenerateInput, DriveLine};

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(300.0, 0.0),
                    Vec2::new(300.0, 200.0),
                    Vec2::new(0.0, 200.0),
                ],
                holes: vec![],
                ..Default::default()
            },
            aisle_graph: None,
            drive_lines: vec![
                DriveLine {
                    start: Vec2::new(150.0, -50.0),
                    end: Vec2::new(150.0, 250.0),
                hole_pin: None, id: 1, partitions: false, },
            ],
            annotations: vec![
                // Drive line spans y=-50→y=250 (length 300). The interior
                // splice spans the inset boundary, ~y=0→y=200, i.e.
                // t≈0.166→0.833 in normalized coords.
                Annotation::SpliceTwoWayOriented {
                    drive_line_id: 1,
                    ta: 0.166,
                    tb: 0.833,
                },
            ],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
        };

        let layout = generate(input);
        eprintln!("\n=== TwoWayOriented annotation e2e ===");
        eprintln!("stalls: {}, spines: {}, faces: {}",
            layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Check the resolved graph has TwoWayOriented edges.
        let two_way_ori_count = layout.resolved_graph.edges.iter()
            .filter(|e| e.direction == AisleDirection::TwoWayOriented)
            .count();
        eprintln!("TwoWayOriented edges in resolved graph: {}", two_way_ori_count);
        assert!(two_way_ori_count > 0, "should have TwoWayOriented edges after annotation");

        // Verify stalls exist on both sides.
        assert!(layout.stalls.len() > 0, "should produce stalls");
    }

    /// Comprehensive test for mark_island_stalls covering:
    /// - Various row lengths and intervals
    /// - Multiple independent rows (different spines)
    /// - Angled stalls (45°)
    /// - Invariants: no islands at row edges, every qualifying row gets
    ///   islands, no long runs without an island.
    #[test]
    fn test_mark_island_stalls_invariants() {
        // Helper: create a horizontal spine at y_offset with stalls along
        // the x-axis. Normal points up (+y), so oriented_dir is +x.
        fn make_row(
            n_target: usize,
            spine_idx: usize,
            face_idx: usize,
            y_offset: f64,
            params: &ParkingParams,
        ) -> (Vec<(StallQuad, usize, usize)>, SpineSegment) {
            let pitch = params.stall_pitch();
            let spine_len = n_target as f64 * pitch;
            let seg = SpineSegment {
                start: Vec2::new(0.0, y_offset),
                end: Vec2::new(spine_len, y_offset),
                outward_normal: Vec2::new(0.0, 1.0),
                face_idx,
                is_interior: true,
                travel_dir: None,
            };
            let stalls: Vec<(StallQuad, usize, usize)> = fill_spine(&seg, params, 0.0)
                .into_iter()
                .map(|q| (q, face_idx, spine_idx))
                .collect();
            (stalls, seg)
        }

        // Sort stalls by position along direction for a given spine.
        fn sorted_row(
            stalls: &[(StallQuad, usize, usize)],
            spine_idx: usize,
            dir: Vec2,
        ) -> Vec<(f64, bool)> {
            let mut s: Vec<(f64, bool)> = stalls.iter()
                .filter(|(_, _, si)| *si == spine_idx)
                .map(|(q, _, _)| (stall_center(q).dot(dir), q.kind == StallKind::Island))
                .collect();
            s.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            s
        }

        fn spine_range(spine: &SpineSegment, dir: Vec2) -> (f64, f64) {
            let a = spine.start.dot(dir);
            let b = spine.end.dot(dir);
            (a.min(b), a.max(b))
        }

        // Check all invariants for a single row.
        // spine_proj_range: (proj_min, proj_max) from spine endpoints,
        // matching how mark_island_stalls computes the end margin.
        fn check_invariants(
            stalls: &[(StallQuad, usize, usize)],
            spine_idx: usize,
            dir: Vec2,
            interval: usize,
            spine_proj_range: (f64, f64),
            label: &str,
        ) {
            let row = sorted_row(stalls, spine_idx, dir);
            let count = row.len();

            if count < interval {
                let n_islands = row.iter().filter(|(_, is)| *is).count();
                assert_eq!(n_islands, 0,
                    "{}: row with {} stalls (< interval {}) should have 0 islands, got {}",
                    label, count, interval, n_islands);
                return;
            }

            // A+B: stalls within 1.5 pitches of spine ends are not islands
            // (position-based margin from spine endpoints, matching
            // mark_island_stalls logic).
            let stall_pitch_val = row.get(1).map_or(1.0, |r| r.0 - row[0].0);
            let end_margin = 1.5 * stall_pitch_val;
            let (proj_min, proj_max) = spine_proj_range;
            for (j, &(proj, is_isl)) in row.iter().enumerate() {
                if proj - proj_min < end_margin || proj_max - proj < end_margin {
                    assert!(!is_isl,
                        "{}: stall {} at proj={:.1} should not be island (within end margin, count={}, interval={})",
                        label, j, proj, count, interval);
                }
            }

            // C: at least one island exists when enough interior stalls
            // remain after position-based end margins.
            let n_islands = row.iter().filter(|(_, is)| *is).count();
            let eligible = row.iter().filter(|(proj, _)| {
                proj - proj_min >= end_margin && proj_max - proj >= end_margin
            }).count();
            if eligible >= interval {
                assert!(n_islands >= 1,
                    "{}: row with {} stalls ({} eligible) and interval {} should have >= 1 island, got {}",
                    label, count, eligible, interval, n_islands);
            }

            // D: no run of >= interval consecutive non-island stalls
            // in the eligible interior.
            {
                let mut run = 0usize;
                for (j, &(proj, is_isl)) in row.iter().enumerate() {
                    if proj - proj_min < end_margin || proj_max - proj < end_margin {
                        continue;
                    }
                    if is_isl {
                        run = 0;
                    } else {
                        run += 1;
                        assert!(run < interval,
                            "{}: interior run of {} non-island stalls at position {} exceeds interval {} (count={})",
                            label, run + 1, j, interval, count);
                    }
                }
            }

            // E: reasonable island count — at most count/2.
            assert!(n_islands <= count / 2,
                "{}: too many islands ({}) for {} stalls",
                label, n_islands, count);
        }

        // =========================================================
        // Test 1: Single rows, various lengths, interval=5, 90° stalls
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 90.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };
            let dir = Vec2::new(1.0, 0.0);

            for &n in &[3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 30] {
                let (mut stalls_3, spine) = make_row(n, 0, 0, 0.0, &params);
                let actual_n = stalls_3.len();
                let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                    .map(|(q, fi, _)| (q.clone(), *fi)).collect();
                let spines = vec![spine];

                mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);
                let spr = spine_range(&spines[0], dir);
                check_invariants(&stalls_3, 0, dir, 5, spr, &format!("single_90deg_n{}", actual_n));
            }
        }

        // =========================================================
        // Test 2: Single row, length=20, various intervals
        // =========================================================
        {
            let dir = Vec2::new(1.0, 0.0);

            for &interval in &[2, 3, 4, 5, 6, 8, 10] {
                let params = ParkingParams {
                    stall_angle_deg: 90.0,
                    island_stall_interval: interval,
                    ..ParkingParams::default()
                };
                let (mut stalls_3, spine) = make_row(20, 0, 0, 0.0, &params);
                let actual_n = stalls_3.len();
                let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                    .map(|(q, fi, _)| (q.clone(), *fi)).collect();
                let spines = vec![spine];

                mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);
                let spr = spine_range(&spines[0], dir);
                check_invariants(&stalls_3, 0, dir, interval as usize, spr,
                    &format!("single_interval{}_n{}", interval, actual_n));
            }
        }

        // =========================================================
        // Test 3: 45° angled stalls, various lengths, interval=5
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 45.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };
            let dir = Vec2::new(1.0, 0.0);

            for &n in &[6, 8, 12, 20] {
                let (mut stalls_3, spine) = make_row(n, 0, 0, 0.0, &params);
                let actual_n = stalls_3.len();
                let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                    .map(|(q, fi, _)| (q.clone(), *fi)).collect();
                let spines = vec![spine];

                mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);
                let spr = spine_range(&spines[0], dir);
                check_invariants(&stalls_3, 0, dir, 5, spr, &format!("angled45_n{}", actual_n));
            }
        }

        // =========================================================
        // Test 4: Two independent rows (different spines, same params)
        //         — both should independently get islands
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 90.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };
            let dir = Vec2::new(1.0, 0.0);

            let (mut stalls_a, spine_a) = make_row(20, 0, 0, 0.0, &params);
            let (stalls_b, spine_b) = make_row(12, 1, 0, 50.0, &params);
            stalls_a.extend(stalls_b);
            let mut tagged: Vec<(StallQuad, usize)> = stalls_a.iter()
                .map(|(q, fi, _)| (q.clone(), *fi)).collect();
            let spines = vec![spine_a, spine_b];

            mark_island_stalls(&mut stalls_a, &mut tagged, &spines, &params);
            let spr0 = spine_range(&spines[0], dir);
            let spr1 = spine_range(&spines[1], dir);
            check_invariants(&stalls_a, 0, dir, 5, spr0, "two_rows_spine0");
            check_invariants(&stalls_a, 1, dir, 5, spr1, "two_rows_spine1");
        }

        // =========================================================
        // Test 5: tagged vector is marked in sync with stalls_3
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 90.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };

            let (mut stalls_3, spine) = make_row(20, 0, 0, 0.0, &params);
            let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                .map(|(q, fi, _)| (q.clone(), *fi)).collect();
            let spines = vec![spine];

            mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);

            for (s3, _, _) in &stalls_3 {
                let key = stall_key(s3);
                let tagged_match = tagged.iter().find(|(t, _)| stall_key(t) == key);
                assert!(tagged_match.is_some(), "stalls_3 entry should exist in tagged");
                assert_eq!(s3.kind, tagged_match.unwrap().0.kind,
                    "kind should match between stalls_3 and tagged");
            }
        }
    }
}

