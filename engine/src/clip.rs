use crate::types::*;

/// Remove stalls that extend outside the outer boundary or into any hole.
/// A stall is removed if any of its corners falls outside the outer boundary,
/// inside a hole, or if any stall edge crosses the boundary/hole edges.
pub fn clip_stalls_to_boundary(
    stalls: Vec<StallQuad>,
    boundary: &Polygon,
) -> Vec<StallQuad> {
    stalls
        .into_iter()
        .filter(|stall| {
            // Shrink toward centroid before testing so stalls whose tips are
            // right at the boundary aren't rejected by floating-point noise.
            // Use a small tolerance (0.1ft) to catch stalls that slightly
            // protrude past the boundary at corners.
            let shrunk = shrink_polygon(&stall.corners, 0.1);
            // Every corner must be inside the outer boundary.
            for c in &shrunk {
                if !point_in_polygon(c, &boundary.outer) {
                    return false;
                }
            }
            // No corner may be inside any hole.
            for hole in &boundary.holes {
                for c in &shrunk {
                    if point_in_polygon(c, hole) {
                        return false;
                    }
                }
            }
            true
        })
        .collect()
}

/// Shrink a polygon inward by moving each vertex toward the centroid by `amount`.
fn shrink_polygon(corners: &[Vec2], amount: f64) -> Vec<Vec2> {
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

/// Test whether two convex polygons overlap. Returns true if any vertex of
/// either polygon is inside the other, or if any pair of edges cross.
pub fn polygons_overlap(a: &[Vec2], b: &[Vec2]) -> bool {
    for v in a {
        if point_in_polygon(v, b) {
            return true;
        }
    }
    for v in b {
        if point_in_polygon(v, a) {
            return true;
        }
    }
    for i in 0..a.len() {
        let a0 = a[i];
        let a1 = a[(i + 1) % a.len()];
        for j in 0..b.len() {
            let b0 = b[j];
            let b1 = b[(j + 1) % b.len()];
            if segments_intersect(a0, a1, b0, b1) {
                return true;
            }
        }
    }
    false
}

/// Test whether two line segments (p0-p1) and (q0-q1) properly intersect
/// (cross each other, not just touch at endpoints).
fn segments_intersect(p0: Vec2, p1: Vec2, q0: Vec2, q1: Vec2) -> bool {
    let d1 = (q1 - q0).cross(p0 - q0);
    let d2 = (q1 - q0).cross(p1 - q0);
    let d3 = (p1 - p0).cross(q0 - p0);
    let d4 = (p1 - p0).cross(q1 - p0);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}


/// Ray-casting point-in-polygon test.
pub fn point_in_polygon(point: &Vec2, polygon: &[Vec2]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = &polygon[i];
        let pj = &polygon[j];
        if ((pi.y > point.y) != (pj.y > point.y))
            && (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}
