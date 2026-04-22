use crate::types::{EdgeCurve, Polygon, Vec2};

/// Evaluate a cubic bezier at parameter t using de Casteljau.
pub fn eval_cubic(p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2, t: f64) -> Vec2 {
    let s = 1.0 - t;
    let a = p0 * (s * s * s);
    let b = cp1 * (3.0 * s * s * t);
    let c = cp2 * (3.0 * s * t * t);
    let d = p3 * (t * t * t);
    a + b + c + d
}

/// Split a cubic bezier at parameter t using de Casteljau, returning
/// the control points for the two sub-curves.
pub fn split_cubic(
    p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2, t: f64,
) -> ((Vec2, Vec2, Vec2, Vec2), (Vec2, Vec2, Vec2, Vec2)) {
    let m01 = p0 + (cp1 - p0) * t;
    let m12 = cp1 + (cp2 - cp1) * t;
    let m23 = cp2 + (p3 - cp2) * t;
    let m012 = m01 + (m12 - m01) * t;
    let m123 = m12 + (m23 - m12) * t;
    let mid = m012 + (m123 - m012) * t;
    ((p0, m01, m012, mid), (mid, m123, m23, p3))
}

/// Adaptively discretize a single cubic bezier edge into a polyline.
/// Returns points starting with p0 but NOT including p3 (it's the next
/// edge's start vertex). Uses a flatness-based subdivision: if both
/// control points are within `tolerance` of the chord, emit p0 only.
fn discretize_edge_recursive(
    p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2,
    tolerance: f64, depth: u32, out: &mut Vec<Vec2>,
) {
    // Flatness test: max distance from control points to the chord
    // line segment p0→p3. Uses full point-to-segment distance (not
    // just perpendicular to chord) so S-curves and along-chord
    // deviation are caught.
    let d1 = point_to_segment_dist(cp1, p0, p3);
    let d2 = point_to_segment_dist(cp2, p0, p3);
    let flat = d1 < tolerance && d2 < tolerance;

    if flat || depth >= 12 {
        out.push(p0);
        return;
    }

    let ((a0, a1, a2, a3), (b0, b1, b2, b3)) = split_cubic(p0, cp1, cp2, p3, 0.5);
    discretize_edge_recursive(a0, a1, a2, a3, tolerance, depth + 1, out);
    discretize_edge_recursive(b0, b1, b2, b3, tolerance, depth + 1, out);
}

/// Distance from point `p` to the line segment `a→b`.
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

/// Discretize a single cubic bezier edge into a polyline.
/// Returns vertices from p0 up to (but not including) p3.
pub fn discretize_edge(p0: Vec2, cp1: Vec2, cp2: Vec2, p3: Vec2, tolerance: f64) -> Vec<Vec2> {
    let mut out = Vec::new();
    discretize_edge_recursive(p0, cp1, cp2, p3, tolerance, 0, &mut out);
    out
}

/// Discretize a single polygon ring: expand curved edges into polylines.
fn discretize_ring(vertices: &[Vec2], curves: &[Option<EdgeCurve>], tolerance: f64) -> Vec<Vec2> {
    let n = vertices.len();
    if n < 2 {
        return vertices.to_vec();
    }
    let mut out = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        let curve = curves.get(i).and_then(|c| c.as_ref());
        if let Some(c) = curve {
            out.extend(discretize_edge(vertices[i], c.cp1, c.cp2, vertices[j], tolerance));
        } else {
            out.push(vertices[i]);
        }
    }
    out
}

/// Default discretization tolerance in world units. Smaller values
/// produce smoother curves at the cost of more vertices. 0.1 gives
/// visually smooth results at typical parking-lot scale.
const DEFAULT_TOLERANCE: f64 = 0.1;

/// Discretize all curved edges in a polygon into dense straight-line
/// polylines. Returns a new polygon with only straight edges (empty
/// curve arrays). Straight edges pass through unchanged.
pub fn discretize_polygon(polygon: &Polygon) -> Polygon {
    let outer = discretize_ring(&polygon.outer, &polygon.outer_curves, DEFAULT_TOLERANCE);

    let holes: Vec<Vec<Vec2>> = polygon.holes.iter().enumerate().map(|(hi, hole)| {
        let curves = polygon.hole_curves.get(hi).cloned().unwrap_or_default();
        discretize_ring(hole, &curves, DEFAULT_TOLERANCE)
    }).collect();

    Polygon {
        outer,
        holes,
        outer_curves: vec![],
        hole_curves: vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eval_cubic_endpoints() {
        let p0 = Vec2::new(0.0, 0.0);
        let cp1 = Vec2::new(1.0, 2.0);
        let cp2 = Vec2::new(3.0, 2.0);
        let p3 = Vec2::new(4.0, 0.0);

        let start = eval_cubic(p0, cp1, cp2, p3, 0.0);
        assert!((start.x - p0.x).abs() < 1e-12);
        assert!((start.y - p0.y).abs() < 1e-12);

        let end = eval_cubic(p0, cp1, cp2, p3, 1.0);
        assert!((end.x - p3.x).abs() < 1e-12);
        assert!((end.y - p3.y).abs() < 1e-12);
    }

    #[test]
    fn eval_cubic_midpoint() {
        let p0 = Vec2::new(0.0, 0.0);
        let cp1 = Vec2::new(0.0, 4.0);
        let cp2 = Vec2::new(4.0, 4.0);
        let p3 = Vec2::new(4.0, 0.0);

        let mid = eval_cubic(p0, cp1, cp2, p3, 0.5);
        // Midpoint of this symmetric S-curve should be at (2, 3).
        assert!((mid.x - 2.0).abs() < 1e-12);
        assert!((mid.y - 3.0).abs() < 1e-12);
    }

    #[test]
    fn split_cubic_produces_same_curve() {
        let p0 = Vec2::new(0.0, 0.0);
        let cp1 = Vec2::new(1.0, 3.0);
        let cp2 = Vec2::new(3.0, 3.0);
        let p3 = Vec2::new(4.0, 0.0);

        let ((a0, a1, a2, a3), (b0, b1, b2, b3)) = split_cubic(p0, cp1, cp2, p3, 0.5);

        // First half at t=0.5 should equal the original at t=0.25.
        let orig = eval_cubic(p0, cp1, cp2, p3, 0.25);
        let from_a = eval_cubic(a0, a1, a2, a3, 0.5);
        assert!((orig.x - from_a.x).abs() < 1e-10);
        assert!((orig.y - from_a.y).abs() < 1e-10);

        // Second half at t=0.5 should equal the original at t=0.75.
        let orig = eval_cubic(p0, cp1, cp2, p3, 0.75);
        let from_b = eval_cubic(b0, b1, b2, b3, 0.5);
        assert!((orig.x - from_b.x).abs() < 1e-10);
        assert!((orig.y - from_b.y).abs() < 1e-10);
    }

    #[test]
    fn discretize_straight_edge_produces_one_point() {
        // Control points on the chord → flat → single point output.
        let p0 = Vec2::new(0.0, 0.0);
        let p3 = Vec2::new(10.0, 0.0);
        let cp1 = Vec2::new(3.0, 0.0);
        let cp2 = Vec2::new(7.0, 0.0);

        let pts = discretize_edge(p0, cp1, cp2, p3, 0.5);
        assert_eq!(pts.len(), 1);
        assert!((pts[0].x - p0.x).abs() < 1e-12);
    }

    #[test]
    fn discretize_curved_edge_produces_multiple_points() {
        let p0 = Vec2::new(0.0, 0.0);
        let cp1 = Vec2::new(0.0, 40.0);
        let cp2 = Vec2::new(40.0, 40.0);
        let p3 = Vec2::new(40.0, 0.0);

        let pts = discretize_edge(p0, cp1, cp2, p3, 0.5);
        // Should have several points for a curve with 40-unit deflection.
        assert!(pts.len() > 5, "expected >5 points, got {}", pts.len());
        // First point should be p0.
        assert!((pts[0].x - p0.x).abs() < 1e-12);
        assert!((pts[0].y - p0.y).abs() < 1e-12);
        // p3 should NOT be included (it's the next edge's start).
        let last = pts.last().unwrap();
        assert!((last.x - p3.x).abs() > 0.1 || (last.y - p3.y).abs() > 0.1);
    }

    #[test]
    fn discretize_polygon_no_curves() {
        let polygon = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            holes: vec![],
            outer_curves: vec![],
            hole_curves: vec![],
        };
        let result = discretize_polygon(&polygon);
        assert_eq!(result.outer.len(), 4);
        assert!(result.outer_curves.is_empty());
    }

    #[test]
    fn discretize_polygon_with_one_curved_edge() {
        let polygon = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(100.0, 0.0),
                Vec2::new(100.0, 100.0),
                Vec2::new(0.0, 100.0),
            ],
            holes: vec![],
            outer_curves: vec![
                None,
                Some(EdgeCurve {
                    cp1: Vec2::new(100.0, 30.0),
                    cp2: Vec2::new(130.0, 70.0), // Bulges outward.
                }),
                None,
                None,
            ],
            hole_curves: vec![],
        };
        let result = discretize_polygon(&polygon);
        // Should have more vertices than the original (edge 1 expanded).
        assert!(result.outer.len() > 4, "expected >4, got {}", result.outer.len());
        // Curves should be cleared.
        assert!(result.outer_curves.is_empty());
        // Original straight edges' vertices should still be present.
        assert!(result.outer.contains(&Vec2::new(0.0, 0.0)));
        assert!(result.outer.contains(&Vec2::new(100.0, 0.0)));
        assert!(result.outer.contains(&Vec2::new(0.0, 100.0)));
    }
}
