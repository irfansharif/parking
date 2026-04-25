//! Circular arc edges, AutoCAD-style bulge parameterization.
//!
//! Each `EdgeArc` describes the curve between two polygon vertices with
//! a single signed scalar, `bulge = sagitta / (chord_length / 2)`.
//!   bulge  = 0  → straight line (prefer `None` in the per-edge array)
//!   bulge  = +1 → semicircle bulging to the CCW side of P0→P1
//!   bulge  = -1 → semicircle bulging to the CW side
//! Everything downstream of `discretize_polygon` sees only straight
//! edges — this module is the single place that turns bulges into
//! polylines for the pipeline to consume.

use crate::types::{EdgeArc, Polygon, Vec2, VertexId};

/// A bulge whose magnitude is below this is treated as a straight line.
/// Keeps `bulge_to_arc` away from the 1/bulge singularity.
const STRAIGHT_EPS: f64 = 1e-9;

/// Concrete arc geometry derived from an (endpoints, bulge) triple.
/// `sweep` is signed — positive means CCW from `start_angle`.
#[derive(Clone, Copy, Debug)]
pub struct ArcDef {
    pub center: Vec2,
    pub radius: f64,
    pub start_angle: f64,
    pub sweep: f64,
}

/// Derive center/radius/start/sweep from endpoints and a non-zero bulge.
/// Callers must gate on `bulge.abs() > STRAIGHT_EPS`.
pub fn bulge_to_arc(p0: Vec2, p1: Vec2, bulge: f64) -> ArcDef {
    debug_assert!(
        bulge.abs() > STRAIGHT_EPS,
        "bulge_to_arc called on a straight edge (bulge ≈ 0)"
    );
    let chord = p1 - p0;
    let l = chord.length();
    let mid = Vec2::new((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
    // CCW 90° rotation of the chord direction (y-up frame).
    let perp = Vec2::new(-chord.y / l, chord.x / l);
    let radius = l * (1.0 + bulge * bulge) / (4.0 * bulge.abs());
    // Derivation: center lies on chord's perpendicular bisector, on the
    // opposite side from the apex, at distance r·cos(θ/2). Substituting
    // sin(θ/2) = 2b/(1+b²) and cos(θ/2) = (1−b²)/(1+b²) collapses the
    // signed offset to (b²−1)L/(4b).
    let center = mid + perp * ((bulge * bulge - 1.0) * l / (4.0 * bulge));
    let start_angle = (p0.y - center.y).atan2(p0.x - center.x);
    // With apex on the `+perp` side, the path from P0 through apex to
    // P1 is angle-decreasing (traverses the short way around the
    // circle). Negate the raw 4·atan(b) sweep so sampling `start_angle
    // + t·sweep` for t ∈ [0, 1] traces that short path.
    let sweep = -4.0 * bulge.atan();
    ArcDef { center, radius, start_angle, sweep }
}

/// Point on the arc equidistant from the two endpoints — the sagitta
/// tip. This is the handle the UI exposes for dragging.
pub fn arc_apex(p0: Vec2, p1: Vec2, bulge: f64) -> Vec2 {
    let chord = p1 - p0;
    let l = chord.length();
    if l < 1e-12 {
        return p0;
    }
    let mid = Vec2::new((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
    let perp = Vec2::new(-chord.y / l, chord.x / l);
    mid + perp * (bulge * l * 0.5)
}

/// Recover bulge from a dragged apex point. The apex is projected onto
/// the chord's perpendicular bisector, so off-axis drag components are
/// silently ignored (the handle can only move sagitta-ward).
pub fn bulge_from_apex(p0: Vec2, p1: Vec2, apex: Vec2) -> f64 {
    let chord = p1 - p0;
    let l = chord.length();
    if l < 1e-12 {
        return 0.0;
    }
    let mid = Vec2::new((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
    let perp = Vec2::new(-chord.y / l, chord.x / l);
    let signed = (apex - mid).dot(perp);
    2.0 * signed / l
}

/// Total arc length. Falls back to chord length for near-straight edges.
pub fn arc_length(p0: Vec2, p1: Vec2, bulge: f64) -> f64 {
    if bulge.abs() < STRAIGHT_EPS {
        return (p1 - p0).length();
    }
    let arc = bulge_to_arc(p0, p1, bulge);
    arc.radius * arc.sweep.abs()
}

/// Sample a point at parameter `t ∈ [0, 1]` along an arc edge. For a
/// near-straight edge this is just linear interpolation; otherwise
/// samples `start_angle + t·sweep` on the derived arc circle.
pub fn eval_arc_at(p0: Vec2, p1: Vec2, bulge: f64, t: f64) -> Vec2 {
    if bulge.abs() < STRAIGHT_EPS {
        return p0 + (p1 - p0) * t;
    }
    let arc = bulge_to_arc(p0, p1, bulge);
    let a = arc.start_angle + arc.sweep * t;
    Vec2::new(
        arc.center.x + arc.radius * a.cos(),
        arc.center.y + arc.radius * a.sin(),
    )
}

/// Split an arc at parameter `t` into two sub-arcs that together
/// cover the original. Uses tan-half-angle identities: the full
/// sweep is `-4·atan(b)`, so a fraction-t sub-arc has
/// `bulge = tan(t · atan(b))`. Near-straight sub-arcs (bulge within
/// `STRAIGHT_EPS`) are returned as `None` to match the "prefer None
/// for straight edges" convention used by the arcs array.
pub fn split_arc_at(bulge: f64, t: f64) -> (Option<EdgeArc>, Option<EdgeArc>) {
    let half = bulge.atan();
    let b1 = (t * half).tan();
    let b2 = ((1.0 - t) * half).tan();
    let a = if b1.abs() < STRAIGHT_EPS {
        None
    } else {
        Some(EdgeArc { bulge: b1 })
    };
    let b = if b2.abs() < STRAIGHT_EPS {
        None
    } else {
        Some(EdgeArc { bulge: b2 })
    };
    (a, b)
}

/// Evaluate a point at parameter `t` along boundary edge `edge_index`.
/// Matches the `outer_arcs[edge_index]` convention: edge i goes from
/// `outer[i]` to `outer[(i+1) % n]` with the optional arc at `arcs[i]`.
pub fn eval_boundary_edge(
    outer: &[Vec2],
    edge_index: usize,
    t: f64,
    arcs: Option<&[Option<EdgeArc>]>,
) -> Vec2 {
    let n = outer.len();
    let a = outer[edge_index];
    let b = outer[(edge_index + 1) % n];
    let bulge = arcs
        .and_then(|arr| arr.get(edge_index))
        .and_then(|e| e.as_ref())
        .map(|e| e.bulge)
        .unwrap_or(0.0);
    if bulge.abs() < STRAIGHT_EPS {
        a + (b - a) * t
    } else {
        eval_arc_at(a, b, bulge, t)
    }
}

/// Project `pt` onto the nearest outer-boundary edge. Each edge is
/// either an arc (when `arcs[i]` is `Some`) or a straight segment
/// (when `None` or `arcs` is `None`). Returns the projected world
/// position, the index of the winning edge, and the parametric `t`
/// along that edge.
pub fn compute_boundary_pin(
    pt: Vec2,
    outer: &[Vec2],
    arcs: Option<&[Option<EdgeArc>]>,
) -> (Vec2, usize, f64) {
    let mut best_dist = f64::INFINITY;
    let mut best_proj = pt;
    let mut best_edge = 0usize;
    let mut best_t = 0.0;
    let n = outer.len();
    for i in 0..n {
        let a = outer[i];
        let b = outer[(i + 1) % n];
        let bulge = arcs
            .and_then(|arr| arr.get(i))
            .and_then(|e| e.as_ref())
            .map(|e| e.bulge)
            .unwrap_or(0.0);
        let (proj, t) = project_to_arc(a, b, bulge, pt);
        let dist = (pt - proj).length();
        if dist < best_dist {
            best_dist = dist;
            best_proj = proj;
            best_edge = i;
            best_t = t;
        }
    }
    (best_proj, best_edge, best_t)
}

/// Closest point on an arc edge to `pt`, returned as the parameter `t`
/// and the projected position. Closed form: project onto the circle,
/// compute the angular offset from `start_angle` along `sweep`, clamp
/// to `[0, 1]`. Falls back to line-segment projection on near-straight
/// edges.
pub fn project_to_arc(p0: Vec2, p1: Vec2, bulge: f64, pt: Vec2) -> (Vec2, f64) {
    if bulge.abs() < STRAIGHT_EPS {
        let ab = p1 - p0;
        let len_sq = ab.dot(ab);
        if len_sq < 1e-24 {
            return (p0, 0.0);
        }
        let t = (((pt - p0).dot(ab)) / len_sq).clamp(0.0, 1.0);
        return (p0 + ab * t, t);
    }
    let arc = bulge_to_arc(p0, p1, bulge);
    let d = pt - arc.center;
    let pt_angle = d.y.atan2(d.x);
    // Angular offset from start_angle in the sweep direction, wrapped
    // to [0, 2π). Divided by |sweep|, the arc occupies t ∈ [0, 1];
    // overshoot clamps to the nearer endpoint.
    let sweep_sign = if arc.sweep >= 0.0 { 1.0 } else { -1.0 };
    let mut delta = (pt_angle - arc.start_angle) * sweep_sign;
    delta = delta.rem_euclid(std::f64::consts::TAU);
    let t_raw = delta / arc.sweep.abs();
    let t = t_raw.clamp(0.0, 1.0);
    (eval_arc_at(p0, p1, bulge, t), t)
}

/// Discretize a single curved edge into a polyline.
/// Returns `[p0, s_1, …, s_{N-1}]` — endpoint `p1` is excluded because
/// it's the next edge's start vertex (matches the ring-walk contract).
pub fn discretize_edge(p0: Vec2, p1: Vec2, bulge: f64, tolerance: f64) -> Vec<Vec2> {
    if bulge.abs() < STRAIGHT_EPS {
        return vec![p0];
    }
    let arc = bulge_to_arc(p0, p1, bulge);
    // Chord deflection = r(1 − cos(Δθ/2)). Inverting: Δθ_max = 2·acos(1 − ε/r).
    // Ratio ≥ 1 (huge ε or tiny r) collapses to a single segment; cap at
    // 1024 to bound absurd inputs from exploding the sample count.
    let ratio = (tolerance / arc.radius).min(1.0);
    let dtheta_max = 2.0 * (1.0 - ratio).acos();
    let n = if dtheta_max <= f64::EPSILON {
        1024
    } else {
        ((arc.sweep.abs() / dtheta_max).ceil() as usize).max(1).min(1024)
    };
    let mut out = Vec::with_capacity(n);
    // First sample is exact p0, not the floating-point cos/sin
    // round-trip through the center. This matches the straight-edge
    // contract and keeps integer-coordinate vertices exact so the
    // downstream pipeline can reason about them without tolerance.
    out.push(p0);
    let step = arc.sweep / n as f64;
    for i in 1..n {
        let a = arc.start_angle + step * i as f64;
        out.push(Vec2::new(
            arc.center.x + arc.radius * a.cos(),
            arc.center.y + arc.radius * a.sin(),
        ));
    }
    out
}

fn discretize_ring(
    vertices: &[Vec2],
    arcs: &[Option<EdgeArc>],
    ids: &[VertexId],
    tolerance: f64,
    next_synthetic: &mut u32,
) -> (Vec<Vec2>, Vec<VertexId>) {
    let n = vertices.len();
    if n < 2 {
        return (vertices.to_vec(), ids.to_vec());
    }
    let want_ids = ids.len() == n;
    let mut out_v = Vec::new();
    let mut out_ids = Vec::new();
    let mut alloc_synthetic = || {
        let id = VertexId(*next_synthetic);
        *next_synthetic += 1;
        id
    };
    for i in 0..n {
        let j = (i + 1) % n;
        let arc = arcs.get(i).and_then(|a| a.as_ref());
        let start_id = if want_ids { ids[i] } else { alloc_synthetic() };
        if let Some(a) = arc {
            let pts = discretize_edge(vertices[i], vertices[j], a.bulge, tolerance);
            // pts[0] is the starting vertex (sketch corner). pts[1..]
            // are interior chord samples with synthetic ids.
            for (k, p) in pts.into_iter().enumerate() {
                out_v.push(p);
                out_ids.push(if k == 0 { start_id } else { alloc_synthetic() });
            }
        } else {
            out_v.push(vertices[i]);
            out_ids.push(start_id);
        }
    }
    (out_v, out_ids)
}

/// Discretize every curved edge in a polygon into a dense straight-line
/// polyline. `tolerance` is the chord deflection in world units (feet) —
/// smaller = smoother arcs but more perimeter vertices. Returns a fresh
/// polygon with empty arc arrays. Straight edges pass through unchanged.
///
/// Vertex id propagation: if the input polygon has ids parallel to its
/// vertex arrays, each sketch corner in the output keeps its original
/// `VertexId`. Chord-interior samples (introduced by arc discretization)
/// carry synthetic ids allocated above `VertexId::SYNTHETIC_BASE`. This
/// lets resolution code recognize "real" sketch addresses vs.
/// discretizer-introduced filler when projecting.
pub fn discretize_polygon(polygon: &Polygon, tolerance: f64) -> Polygon {
    let mut next_synthetic: u32 = VertexId::SYNTHETIC_BASE;
    let (outer, outer_ids) = discretize_ring(
        &polygon.outer,
        &polygon.outer_arcs,
        &polygon.outer_ids,
        tolerance,
        &mut next_synthetic,
    );
    let mut holes: Vec<Vec<Vec2>> = Vec::with_capacity(polygon.holes.len());
    let mut hole_ids: Vec<Vec<VertexId>> = Vec::with_capacity(polygon.holes.len());
    for (hi, hole) in polygon.holes.iter().enumerate() {
        let arcs = polygon.hole_arcs.get(hi).cloned().unwrap_or_default();
        let ids = polygon.hole_ids.get(hi).cloned().unwrap_or_default();
        let (h, h_ids) = discretize_ring(
            hole,
            &arcs,
            &ids,
            tolerance,
            &mut next_synthetic,
        );
        holes.push(h);
        hole_ids.push(h_ids);
    }
    Polygon {
        outer,
        holes,
        outer_arcs: vec![],
        hole_arcs: vec![],
        outer_ids,
        hole_ids,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn straight_edge_discretizes_to_single_point() {
        let p0 = Vec2::new(0.0, 0.0);
        let p1 = Vec2::new(10.0, 0.0);
        let pts = discretize_edge(p0, p1, 0.0, 0.1);
        assert_eq!(pts.len(), 1);
        assert!(close(pts[0].x, p0.x, 1e-12));
    }

    #[test]
    fn semicircle_center_and_radius() {
        // P0=(-1,0), P1=(1,0), bulge=1 → semicircle above, radius 1,
        // center at origin.
        let p0 = Vec2::new(-1.0, 0.0);
        let p1 = Vec2::new(1.0, 0.0);
        let arc = bulge_to_arc(p0, p1, 1.0);
        assert!(close(arc.center.x, 0.0, 1e-10));
        assert!(close(arc.center.y, 0.0, 1e-10));
        assert!(close(arc.radius, 1.0, 1e-10));
        // Semicircle sweep magnitude is π. Sign is negative because
        // the short path from P0 through the +y apex to P1 traverses
        // angles π → π/2 → 0 (decreasing).
        assert!(close(arc.sweep.abs(), std::f64::consts::PI, 1e-10));
        assert!(arc.sweep < 0.0);
    }

    #[test]
    fn apex_round_trips_with_bulge() {
        let p0 = Vec2::new(0.0, 0.0);
        let p1 = Vec2::new(4.0, 0.0);
        for b in [-0.9, -0.3, 0.1, 0.5, 0.85] {
            let apex = arc_apex(p0, p1, b);
            let recovered = bulge_from_apex(p0, p1, apex);
            assert!(close(recovered, b, 1e-10), "bulge {} → {}", b, recovered);
        }
    }

    #[test]
    fn positive_bulge_curves_ccw_of_chord() {
        // Chord along +x; CCW perpendicular is +y. Positive bulge should
        // put the apex above the chord.
        let p0 = Vec2::new(0.0, 0.0);
        let p1 = Vec2::new(2.0, 0.0);
        let apex = arc_apex(p0, p1, 0.5);
        assert!(apex.y > 0.0, "apex.y = {}", apex.y);
        // And negative flips it.
        let apex_neg = arc_apex(p0, p1, -0.5);
        assert!(apex_neg.y < 0.0);
    }

    #[test]
    fn discretized_points_lie_on_circle() {
        let p0 = Vec2::new(-1.0, 0.0);
        let p1 = Vec2::new(1.0, 0.0);
        let b = 0.6;
        let arc = bulge_to_arc(p0, p1, b);
        let pts = discretize_edge(p0, p1, b, 0.01);
        assert!(pts.len() > 4);
        for p in &pts {
            let d = ((p.x - arc.center.x).powi(2) + (p.y - arc.center.y).powi(2)).sqrt();
            assert!(
                close(d, arc.radius, 1e-6),
                "sample {:?} off-circle by {}",
                p,
                d - arc.radius
            );
        }
        // First sample is p0.
        assert!(close(pts[0].x, p0.x, 1e-10));
        assert!(close(pts[0].y, p0.y, 1e-10));
        // p1 is NOT included (next edge owns it).
        let last = pts.last().unwrap();
        assert!((last.x - p1.x).abs() + (last.y - p1.y).abs() > 1e-3);
        // Sampling must traverse the short path through the apex, not
        // the long way around the circle. Every sample should lie on
        // the same side of the chord as the apex (sign of the
        // perpendicular component from the chord midpoint).
        let expected_apex = arc_apex(p0, p1, b);
        let chord_mid = Vec2::new((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
        let perp = Vec2::new(-(p1.y - p0.y), p1.x - p0.x);
        let apex_side = (expected_apex - chord_mid).dot(perp);
        assert!(apex_side > 0.0, "apex side sanity check");
        // First sample is exactly p0 (on the chord); skip it.
        for p in &pts[1..] {
            let side = (*p - chord_mid).dot(perp);
            assert!(
                side > -1e-9,
                "sample {:?} on opposite side of chord from apex",
                p
            );
        }
    }

    #[test]
    fn discretize_polygon_passes_straight_edges_through() {
        let polygon = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            ..Default::default()
        };
        let result = discretize_polygon(&polygon, 5.0);
        assert_eq!(result.outer.len(), 4);
        assert!(result.outer_arcs.is_empty());
    }

    #[test]
    fn discretize_polygon_with_one_arc_expands_that_edge() {
        let polygon = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(100.0, 0.0),
                Vec2::new(100.0, 100.0),
                Vec2::new(0.0, 100.0),
            ],
            outer_arcs: vec![
                None,
                Some(EdgeArc { bulge: 0.3 }),
                None,
                None,
            ],
            ..Default::default()
        };
        let result = discretize_polygon(&polygon, 5.0);
        assert!(result.outer.len() > 4, "expected expansion, got {}", result.outer.len());
        assert!(result.outer_arcs.is_empty());
        // Untouched vertices still present.
        assert!(result.outer.contains(&Vec2::new(0.0, 0.0)));
        assert!(result.outer.contains(&Vec2::new(100.0, 0.0)));
        assert!(result.outer.contains(&Vec2::new(0.0, 100.0)));
    }

    #[test]
    fn arc_length_semicircle() {
        let p0 = Vec2::new(-1.0, 0.0);
        let p1 = Vec2::new(1.0, 0.0);
        let len = arc_length(p0, p1, 1.0);
        assert!(close(len, std::f64::consts::PI, 1e-10));
    }

    #[test]
    fn arc_length_straight_edge_uses_chord() {
        let p0 = Vec2::new(0.0, 0.0);
        let p1 = Vec2::new(3.0, 4.0);
        assert!(close(arc_length(p0, p1, 0.0), 5.0, 1e-12));
    }
}
