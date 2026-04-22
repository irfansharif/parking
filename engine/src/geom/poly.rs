//! Polygon and polyline predicates that don't fit cleanly under
//! `boolean`, `clip`, or `inset`.
//!
//! Canonical home for:
//!
//!   - point-in-polygon variants (faces with holes, tolerance-aware)
//!   - point-to-segment distance
//!   - segment clipping against a face (returns parameter intervals)
//!   - contour simplification (colinearity-based)
//!
//! All functions here are parking-agnostic and take/return `Vec2` +
//! face-shape slices; a future `geom` crate extraction can lift this
//! module wholesale.

use super::clip::point_in_polygon as point_in_loop;
use crate::types::Vec2;

/// Test whether `p` lies inside a face shape: inside the outer contour
/// AND outside every hole.
pub fn point_in_face(p: Vec2, shape: &[Vec<Vec2>]) -> bool {
    if shape.is_empty() {
        return false;
    }
    if !point_in_loop(&p, &shape[0]) {
        return false;
    }
    for hole in &shape[1..] {
        if point_in_loop(&p, hole) {
            return false;
        }
    }
    true
}

/// Like `point_in_face`, but also accepts points within `tol` of any
/// face boundary edge. Needed when the caller's point is a vertex
/// known to lie exactly on the face boundary (the winding-number test
/// treats such points as outside).
pub fn point_in_or_on_face(p: Vec2, shape: &[Vec<Vec2>], tol: f64) -> bool {
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

/// Shortest distance from point `p` to line segment `a→b`.
pub fn point_to_segment_dist(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < 1e-12 {
        return (p - a).length();
    }
    let t = (p - a).dot(ab) / len_sq;
    let proj = a + ab * t.clamp(0.0, 1.0);
    (p - proj).length()
}

/// Clip line segment `a→b` to the interior of a face shape. Returns
/// parameter intervals `[(t0, t1), …]` where `a + t*(b-a)` is inside
/// the face (outer CCW minus CW holes).
pub fn clip_segment_to_face(a: Vec2, b: Vec2, shape: &[Vec<Vec2>]) -> Vec<(f64, f64)> {
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

/// Cast a ray from `origin` in `dir` and return
/// `(contour_index, edge_index)` of the nearest face-shape boundary
/// edge it crosses, or `None` if no edge is hit within the positive
/// ray domain.
pub fn ray_hit_face_edge(
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

/// Simplify a closed contour in two passes:
///
/// 1. Remove duplicate vertices (zero-length edges, tol 1e-6).
/// 2. Drop any vertex whose incoming/outgoing edges are near-collinear
///    — the angle between them smaller than `angle_tol` radians.
///
/// Preserves winding. If simplification would leave fewer than 3
/// vertices, returns the dedup-only result.
pub fn simplify_contour(pts: &[Vec2], angle_tol: f64) -> Vec<Vec2> {
    if pts.len() < 3 {
        return pts.to_vec();
    }

    // Pass 1: remove duplicate vertices (truly zero-length edges only).
    let dup_tol = 1e-6;
    let mut deduped: Vec<Vec2> = Vec::with_capacity(pts.len());
    for &p in pts {
        if deduped
            .last()
            .map_or(true, |prev: &Vec2| (*prev - p).length() > dup_tol)
        {
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
