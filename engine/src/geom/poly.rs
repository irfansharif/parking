//! Polygon and polyline predicates that don't fit cleanly under
//! `boolean`, `clip`, or `inset`.
//!
//! Canonical home for:
//!
//!   - point-in-polygon variants (faces with holes, tolerance-aware)
//!   - point-to-segment distance
//!   - segment clipping against a face (returns parameter intervals)
//!   - signed area + winding orientation helpers
//!   - contour simplification (colinearity-based)
//!
//! All functions here are parking-agnostic and take/return `Vec2` +
//! face-shape slices; a future `geom` crate extraction can lift this
//! module wholesale.

use super::clip::point_in_polygon as point_in_loop;
use crate::types::{Vec2, VertexId};

/// Signed area of a closed polygonal loop. Positive ↔ CCW in a y-up
/// frame (sketch convention); negative ↔ CW. Loops with fewer than 3
/// vertices return 0.
pub fn signed_area(polygon: &[Vec2]) -> f64 {
    let n = polygon.len();
    if n < 3 {
        return 0.0;
    }
    let mut acc = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        acc += polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y;
    }
    acc * 0.5
}

/// Reverse `poly` in place if it's wound CW so the result is CCW.
pub fn ensure_ccw(mut poly: Vec<Vec2>) -> Vec<Vec2> {
    if signed_area(&poly) < 0.0 {
        poly.reverse();
    }
    poly
}

/// Reverse `poly` in place if it's wound CCW so the result is CW.
/// Used when feeding hole contours into a boolean overlay that
/// expects holes to be wound opposite from the outer ring.
pub fn ensure_cw(mut poly: Vec<Vec2>) -> Vec<Vec2> {
    if signed_area(&poly) > 0.0 {
        poly.reverse();
    }
    poly
}

/// `ensure_ccw` paired with a parallel `VertexId` array. Reverses ids
/// alongside so segment-i's start vertex id stays at `ids[i]` after
/// any orientation flip. Caller passes empty `ids` (or a length
/// mismatch) to opt out — the returned ids vec is then untouched
/// alongside the polygon.
pub fn ensure_ccw_with_ids(
    mut poly: Vec<Vec2>,
    mut ids: Vec<VertexId>,
) -> (Vec<Vec2>, Vec<VertexId>) {
    if signed_area(&poly) < 0.0 {
        poly.reverse();
        if ids.len() == poly.len() {
            ids.reverse();
        }
    }
    (poly, ids)
}

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

/// Pole of inaccessibility — the interior point farthest from any
/// boundary edge. Useful as a label-placement anchor for irregular
/// polygons (centroid can fall outside concave shapes). Delegates to
/// the `polylabel` crate (Mapbox-authored Rust port of the original
/// JS implementation).
///
/// `precision` caps the search — refinement stops once the heap's
/// best upper bound is within `precision` of the current best
/// in-cell distance. 1.0 (= 1 ft) is plenty for label placement.
pub fn polygon_pole(outer: &[Vec2], holes: &[Vec<Vec2>], precision: f64) -> Vec2 {
    use geo_types::{Coord, LineString, Polygon};

    if outer.len() < 3 {
        return outer.first().copied().unwrap_or(Vec2::new(0.0, 0.0));
    }
    let to_ls = |ring: &[Vec2]| -> LineString<f64> {
        LineString(ring.iter().map(|v| Coord { x: v.x, y: v.y }).collect())
    };
    let polygon = Polygon::new(
        to_ls(outer),
        holes.iter().filter(|h| h.len() >= 3).map(|h| to_ls(h)).collect(),
    );
    match polylabel::polylabel(&polygon, &precision) {
        Ok(p) => Vec2::new(p.x(), p.y()),
        Err(_) => {
            // Bbox center fallback for degenerate inputs (zero-area
            // polygons or tolerance/sizing pathologies).
            let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
            let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
            for p in outer {
                min_x = min_x.min(p.x);
                max_x = max_x.max(p.x);
                min_y = min_y.min(p.y);
                max_y = max_y.max(p.y);
            }
            Vec2::new((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)
        }
    }
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

