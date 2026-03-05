use crate::types::Vec2;

/// Signed area of a polygon (positive = CCW, negative = CW).
pub fn signed_area(polygon: &[Vec2]) -> f64 {
    let n = polygon.len();
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += polygon[i].x * polygon[j].y;
        area -= polygon[j].x * polygon[i].y;
    }
    area * 0.5
}

/// 2D cross product (z-component).
fn cross(a: Vec2, b: Vec2) -> f64 {
    a.x * b.y - a.y * b.x
}

/// Inset a polygon inward by distance `d`. Returns the offset polygon, or an
/// empty vec if the polygon collapses entirely.
pub fn inset_polygon(polygon: &[Vec2], d: f64) -> Vec<Vec2> {
    let n = polygon.len();
    if n < 3 {
        return Vec::new();
    }

    // Determine winding: sign = +1 for CCW, -1 for CW.
    let sign = if signed_area(polygon) >= 0.0 { 1.0 } else { -1.0 };

    // Compute edge directions and inward normals.
    let mut edge_dirs: Vec<Vec2> = Vec::with_capacity(n);
    let mut normals: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let e = polygon[j] - polygon[i];
        edge_dirs.push(e);
        let normal = Vec2::new(-e.y * sign, e.x * sign).normalize();
        normals.push(normal);
    }

    // For each vertex, compute the offset position by intersecting the two
    // adjacent offset lines.
    let mut result: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };

        // Line A: offset of edge (prev), passes through polygon[prev] + normal[prev]*d,
        //         direction = edge_dirs[prev]
        let a = polygon[prev] + normals[prev] * d;
        let da = edge_dirs[prev];

        // Line B: offset of edge i, passes through polygon[i] + normal[i]*d,
        //         direction = edge_dirs[i]
        let b = polygon[i] + normals[i] * d;
        let db = edge_dirs[i];

        // Solve a + t*da = b + s*db via cross product.
        let denom = cross(da, db);
        if denom.abs() < 1e-12 {
            // Parallel edges — just offset the vertex along the shared normal.
            result.push(polygon[i] + normals[i] * d);
        } else {
            let t = cross(b - a, db) / denom;
            result.push(a + da * t);
        }
    }

    // Edge collapse detection: remove vertices where adjacent offset edges
    // have flipped direction relative to the original.
    let mut alive = vec![true; n];
    let mut changed = true;
    while changed {
        changed = false;
        let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
        if live.len() < 3 {
            return Vec::new();
        }
        for idx in 0..live.len() {
            let i = live[idx];
            let j = live[(idx + 1) % live.len()];
            let new_dir = result[j] - result[i];
            let orig_dir = polygon[j] - polygon[i];
            if new_dir.dot(orig_dir) <= 0.0 {
                // This edge has collapsed. Remove the vertex that creates
                // a smaller triangle with its neighbors (heuristic: remove
                // both endpoints of the collapsed edge, then recompute).
                alive[i] = false;
                alive[j] = false;
                changed = true;
                break;
            }
        }

        if changed {
            // Recompute intersections for surviving vertices.
            let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
            if live.len() < 3 {
                return Vec::new();
            }
            for idx in 0..live.len() {
                let i = live[idx];
                let prev_idx = if idx == 0 { live.len() - 1 } else { idx - 1 };
                let prev = live[prev_idx];

                let a = polygon[prev] + normals[prev] * d;
                let da = edge_dirs[prev];
                let b = polygon[i] + normals[i] * d;
                let db = edge_dirs[i];
                let denom = cross(da, db);
                if denom.abs() < 1e-12 {
                    result[i] = polygon[i] + normals[i] * d;
                } else {
                    let t = cross(b - a, db) / denom;
                    result[i] = a + da * t;
                }
            }
        }
    }

    let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
    if live.len() < 3 {
        return Vec::new();
    }

    // Check area — if it's negligible or negative, the polygon has collapsed.
    let out: Vec<Vec2> = live.iter().map(|&i| result[i]).collect();
    if signed_area(&out).abs() < 1.0 {
        return Vec::new();
    }

    out
}

/// Core per-edge inset with miter intersections and edge-collapse detection.
///
/// `sign_override` lets the caller force a winding sign (e.g. use the outer
/// contour's sign for hole contours).  Pass `None` to derive from the polygon.
///
/// Returns `(inset_vertices_indexed_by_original, surviving_indices, normals)`.
/// All three vecs are indexed by original vertex index; only the entries at
/// `surviving_indices` positions are meaningful.
pub fn inset_per_edge_core(
    polygon: &[Vec2],
    distances: &[f64],
    sign_override: Option<f64>,
) -> (Vec<Vec2>, Vec<usize>, Vec<Vec2>) {
    let n = polygon.len();
    if n < 3 {
        return (vec![], vec![], vec![]);
    }
    assert_eq!(distances.len(), n);

    let sign = sign_override
        .unwrap_or_else(|| if signed_area(polygon) >= 0.0 { 1.0 } else { -1.0 });

    let mut edge_dirs: Vec<Vec2> = Vec::with_capacity(n);
    let mut normals: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let e = polygon[j] - polygon[i];
        edge_dirs.push(e);
        normals.push(Vec2::new(-e.y * sign, e.x * sign).normalize());
    }

    // Compute miter vertices for a given alive set.
    let compute_miter = |alive: &[bool], result: &mut Vec<Vec2>| {
        let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
        for idx in 0..live.len() {
            let i = live[idx];
            let prev = live[if idx == 0 { live.len() - 1 } else { idx - 1 }];
            let a = polygon[prev] + normals[prev] * distances[prev];
            let da = edge_dirs[prev];
            let b = polygon[i] + normals[i] * distances[i];
            let db = edge_dirs[i];
            let denom = cross(da, db);
            if denom.abs() < 1e-12 {
                result[i] = polygon[i] + normals[i] * distances[i];
            } else {
                let t = cross(b - a, db) / denom;
                result[i] = a + da * t;
            }
        }
    };

    let mut result = vec![Vec2::new(0.0, 0.0); n];
    let mut alive = vec![true; n];
    compute_miter(&alive, &mut result);

    // Edge-collapse detection: if an inset edge has flipped direction
    // relative to its original, the polygon is too narrow there.
    let mut changed = true;
    while changed {
        changed = false;
        let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
        if live.len() < 3 {
            return (vec![], vec![], normals);
        }
        for idx in 0..live.len() {
            let i = live[idx];
            let j = live[(idx + 1) % live.len()];
            let new_dir = result[j] - result[i];
            let orig_dir = polygon[j] - polygon[i];
            if new_dir.dot(orig_dir) <= 0.0 {
                alive[i] = false;
                alive[j] = false;
                changed = true;
                break;
            }
        }
        if changed {
            let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
            if live.len() < 3 {
                return (vec![], vec![], normals);
            }
            compute_miter(&alive, &mut result);
        }
    }

    let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
    if live.len() < 3 {
        return (vec![], vec![], normals);
    }

    (result, live, normals)
}

/// Inset a polygon with a per-edge distance. `distances[i]` is the inset for
/// edge i (from vertex i to vertex (i+1)%n). Returns the offset polygon, or
/// an empty vec if the polygon collapses entirely.
pub fn inset_polygon_per_edge(polygon: &[Vec2], distances: &[f64]) -> Vec<Vec2> {
    let (result, live, _) = inset_per_edge_core(polygon, distances, None);
    if live.is_empty() {
        return Vec::new();
    }
    let out: Vec<Vec2> = live.iter().map(|&i| result[i]).collect();
    if signed_area(&out).abs() < 1.0 {
        return Vec::new();
    }
    out
}

/// Same as `inset_polygon_per_edge` but also returns the surviving original
/// vertex indices. `result.1[k]` is the original polygon index that produced
/// skeleton vertex `result.0[k]`.
pub fn inset_polygon_per_edge_with_indices(
    polygon: &[Vec2],
    distances: &[f64],
) -> (Vec<Vec2>, Vec<usize>) {
    let (result, live, _) = inset_per_edge_core(polygon, distances, None);
    if live.is_empty() {
        return (vec![], vec![]);
    }
    let out: Vec<Vec2> = live.iter().map(|&i| result[i]).collect();
    if signed_area(&out).abs() < 1.0 {
        return (vec![], vec![]);
    }
    (out, live)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signed_area_ccw() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        assert!((signed_area(&poly) - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_inset_square() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];
        let inset = inset_polygon(&poly, 10.0);
        assert_eq!(inset.len(), 4);
        // Should be an 80x80 square centered at (50,50).
        for v in &inset {
            assert!(v.x >= 9.0 && v.x <= 91.0);
            assert!(v.y >= 9.0 && v.y <= 91.0);
        }
    }

    #[test]
    fn test_inset_collapse() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        // Insetting by more than half the width should collapse.
        let inset = inset_polygon(&poly, 6.0);
        assert!(inset.is_empty());
    }
}
