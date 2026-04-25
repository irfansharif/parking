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

/// Pole of inaccessibility — the interior point farthest from any
/// boundary edge. Useful as a label-placement anchor for irregular
/// polygons (centroid can fall outside concave shapes). Implements
/// Mapbox's polylabel algorithm: priority-queue grid subdivision over
/// a max-heap keyed by upper-bound distance per cell.
///
/// `precision` caps the search — refinement stops once the heap's
/// best upper bound is within `precision` of the current best
/// in-cell distance. 1.0 (= 1 ft) is plenty for label placement.
pub fn polygon_pole(outer: &[Vec2], holes: &[Vec<Vec2>], precision: f64) -> Vec2 {
    use std::cmp::Ordering;
    use std::collections::BinaryHeap;

    if outer.len() < 3 {
        return outer.first().copied().unwrap_or(Vec2::new(0.0, 0.0));
    }

    let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
    let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in outer {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
        min_y = min_y.min(p.y);
        max_y = max_y.max(p.y);
    }
    let width = max_x - min_x;
    let height = max_y - min_y;
    let cell_size = width.min(height);
    let center_fallback = Vec2::new((min_x + max_x) * 0.5, (min_y + max_y) * 0.5);
    if cell_size < 1e-9 {
        return center_fallback;
    }

    /// `max_d` = upper bound on signed distance for any point in this
    /// cell (= cell-center distance + half-diagonal). Heap order keys
    /// off this so we always refine the most promising cell next.
    struct Cell {
        center: Vec2,
        half: f64,
        d: f64,
        max_d: f64,
    }
    impl PartialEq for Cell {
        fn eq(&self, o: &Self) -> bool {
            self.max_d == o.max_d
        }
    }
    impl Eq for Cell {}
    impl PartialOrd for Cell {
        fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
            Some(self.cmp(o))
        }
    }
    impl Ord for Cell {
        fn cmp(&self, o: &Self) -> Ordering {
            self.max_d.partial_cmp(&o.max_d).unwrap_or(Ordering::Equal)
        }
    }

    let signed_dist = |p: Vec2| -> f64 {
        let mut min_d = f64::INFINITY;
        for ring in std::iter::once(outer).chain(holes.iter().map(|h| h.as_slice())) {
            let n = ring.len();
            if n < 2 {
                continue;
            }
            for i in 0..n {
                let j = (i + 1) % n;
                let d = point_to_segment_dist(p, ring[i], ring[j]);
                if d < min_d {
                    min_d = d;
                }
            }
        }
        let inside = point_in_loop(&p, outer)
            && !holes.iter().any(|h| point_in_loop(&p, h));
        if inside { min_d } else { -min_d }
    };

    let make_cell = |center: Vec2, half: f64| -> Cell {
        let d = signed_dist(center);
        Cell {
            center,
            half,
            d,
            max_d: d + half * std::f64::consts::SQRT_2,
        }
    };

    let h = cell_size * 0.5;
    let mut queue: BinaryHeap<Cell> = BinaryHeap::new();
    let mut x = min_x;
    while x < max_x {
        let mut y = min_y;
        while y < max_y {
            queue.push(make_cell(Vec2::new(x + h, y + h), h));
            y += cell_size;
        }
        x += cell_size;
    }

    // Seed best with bbox-center and centroid.
    let centroid = {
        let mut acc = Vec2::new(0.0, 0.0);
        let mut a2 = 0.0;
        let n = outer.len();
        for i in 0..n {
            let p0 = outer[i];
            let p1 = outer[(i + 1) % n];
            let cross = p0.x * p1.y - p1.x * p0.y;
            acc.x += (p0.x + p1.x) * cross;
            acc.y += (p0.y + p1.y) * cross;
            a2 += cross;
        }
        if a2.abs() > 1e-9 {
            Vec2::new(acc.x / (3.0 * a2), acc.y / (3.0 * a2))
        } else {
            center_fallback
        }
    };
    let mut best_cell = make_cell(center_fallback, 0.0);
    let centroid_cell = make_cell(centroid, 0.0);
    if centroid_cell.d > best_cell.d {
        best_cell = centroid_cell;
    }

    while let Some(cell) = queue.pop() {
        if cell.d > best_cell.d {
            best_cell = Cell {
                center: cell.center,
                half: cell.half,
                d: cell.d,
                max_d: cell.max_d,
            };
        }
        if cell.max_d - best_cell.d <= precision {
            continue;
        }
        let h2 = cell.half * 0.5;
        for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
            let c = Vec2::new(cell.center.x + sx * h2, cell.center.y + sy * h2);
            queue.push(make_cell(c, h2));
        }
    }

    best_cell.center
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

