//! §3.4 Edge provenance — the production path.
//!
//! Given extracted face contours and the graph/corridor geometry that
//! carved them, tag each face edge with its source:
//!
//!   - Wall (boundary or hole with no adjacent aisle)
//!   - Aisle { corridor_idx, interior, travel_dir, is_two_way_oriented }
//!
//! Works by point-sampling each face edge against the merged aisle
//! polygons and against each per-edge corridor rectangle.

use crate::geom::poly::point_to_segment_dist;
use crate::types::*;

/// Classify each edge of `contour` as aisle-facing (true) or not.
/// An edge is aisle-facing if its midpoint lies on a corridor polygon
/// edge — meaning it was carved by a corridor during the boolean
/// difference. Uses a tight floating-point epsilon since face edges
/// are exact sub-segments of corridor edges after the boolean overlay.
/// A face is boundary if none of its aisle-facing edges come from an
/// interior aisle. Faces bounded only by perimeter/hole aisles (and
/// walls) get 90° stalls. Any interior aisle edge makes the face interior.
/// Returns (is_boundary, wall_edge_indices).
pub(crate) fn is_boundary_face(
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
pub(crate) fn tag_face_edges(
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
#[cfg(test)]
pub(crate) fn classify_face_edges(
    contour: &[Vec2],
    corridor_shapes: &[Vec<Vec<Vec2>>],
    per_edge_corridors: &[(Vec<Vec2>, bool, Option<Vec2>)],
) -> Vec<(bool, bool, Option<Vec2>)> {
    classify_face_edges_ext(contour, corridor_shapes, per_edge_corridors, &[])
        .into_iter()
        .map(|(a, b, c, _)| (a, b, c))
        .collect()
}

/// Extended version that also reports which edges are two-way-oriented.
pub(crate) fn classify_face_edges_ext(
    contour: &[Vec2],
    corridor_shapes: &[Vec<Vec<Vec2>>],
    per_edge_corridors: &[(Vec<Vec2>, bool, Option<Vec2>)],
    two_way_oriented_dirs: &[Option<Vec2>],
) -> Vec<(bool, bool, Option<Vec2>, bool)> {
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
