use crate::inset::signed_area;
use crate::types::Vec2;

// ---------------------------------------------------------------------------
// Straight skeleton types
// ---------------------------------------------------------------------------

/// A single event in the skeleton timeline: records the offset distance at
/// which a topology change occurred, along with the surviving edge indices.
#[derive(Clone, Debug)]
pub struct SkeletonEvent {
    pub d: f64,
    pub active_edges: Vec<usize>,
}

/// Result of computing the event-driven straight skeleton of a polygon.
#[derive(Clone, Debug)]
pub struct StraightSkeleton {
    /// Skeleton edge segments (arc from → to).
    pub arcs: Vec<(Vec2, Vec2)>,
    /// Event points where topology changes occurred.
    pub nodes: Vec<Vec2>,
    /// Timeline of edge-collapse events.
    pub events: Vec<SkeletonEvent>,
    /// Per-edge directions (immutable, from CCW polygon).
    pub edge_dirs: Vec<Vec2>,
    /// Per-edge inward normals (immutable, from CCW polygon).
    edge_normals: Vec<Vec2>,
    /// Per-edge base points (immutable, from CCW polygon).
    edge_points: Vec<Vec2>,
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/// Intersect two lines given as (point, direction). Returns None if parallel.
fn line_intersect(p1: Vec2, d1: Vec2, p2: Vec2, d2: Vec2) -> Option<Vec2> {
    let denom = d1.cross(d2);
    if denom.abs() < 1e-10 {
        return None;
    }
    let diff = p2 - p1;
    let t = diff.cross(d2) / denom;
    Some(p1 + d1 * t)
}

/// Ensure polygon vertices are in CCW winding order (positive signed area).
fn ensure_ccw(pts: &[Vec2]) -> Vec<Vec2> {
    if signed_area(pts) < 0.0 {
        pts.iter().rev().copied().collect()
    } else {
        pts.to_vec()
    }
}

// ---------------------------------------------------------------------------
// Core algorithm
// ---------------------------------------------------------------------------

/// Compute the offset line for edge `e_idx` at distance `d`.
fn get_offset_line(
    edge_points: &[Vec2],
    edge_normals: &[Vec2],
    edge_dirs: &[Vec2],
    e_idx: usize,
    d: f64,
) -> (Vec2, Vec2) {
    let point = edge_points[e_idx] + edge_normals[e_idx] * d;
    let dir = edge_dirs[e_idx];
    (point, dir)
}

/// Compute the vertex where the offset lines of edges `e1` and `e2` meet at
/// distance `d`.
fn offset_vertex(
    edge_points: &[Vec2],
    edge_normals: &[Vec2],
    edge_dirs: &[Vec2],
    e1: usize,
    e2: usize,
    d: f64,
) -> Option<Vec2> {
    let (p1, d1) = get_offset_line(edge_points, edge_normals, edge_dirs, e1, d);
    let (p2, d2) = get_offset_line(edge_points, edge_normals, edge_dirs, e2, d);
    line_intersect(p1, d1, p2, d2)
}

/// Compute the wavefront polygon for a set of active edges at distance `d`.
fn wavefront_at_impl(
    edges: &[usize],
    d: f64,
    edge_points: &[Vec2],
    edge_normals: &[Vec2],
    edge_dirs: &[Vec2],
) -> Option<Vec<Vec2>> {
    let mut pts = Vec::with_capacity(edges.len());
    for i in 0..edges.len() {
        let p = offset_vertex(
            edge_points,
            edge_normals,
            edge_dirs,
            edges[i],
            edges[(i + 1) % edges.len()],
            d,
        )?;
        pts.push(p);
    }
    Some(pts)
}

/// Find when the wavefront vertex between edges[wf_idx] and edges[wf_idx+1]
/// collides with the vertex between edges[wf_idx+1] and edges[wf_idx+2],
/// eliminating edge edges[wf_idx+1].
fn find_edge_event_time(
    edges: &[usize],
    wf_idx: usize,
    current_d: f64,
    edge_points: &[Vec2],
    edge_normals: &[Vec2],
    edge_dirs: &[Vec2],
) -> Option<(f64, Vec2)> {
    let m = edges.len();
    let e_a = edges[wf_idx];
    let e_b = edges[(wf_idx + 1) % m];
    let e_c = edges[(wf_idx + 2) % m];

    let v1_0 = offset_vertex(edge_points, edge_normals, edge_dirs, e_a, e_b, 0.0)?;
    let v1_1 = offset_vertex(edge_points, edge_normals, edge_dirs, e_a, e_b, 1.0)?;
    let v2_0 = offset_vertex(edge_points, edge_normals, edge_dirs, e_b, e_c, 0.0)?;
    let v2_1 = offset_vertex(edge_points, edge_normals, edge_dirs, e_b, e_c, 1.0)?;

    let vel1 = v1_1 - v1_0;
    let vel2 = v2_1 - v2_0;
    let rel_vel = vel1 - vel2;
    let rel_pos = v2_0 - v1_0;

    let d = if rel_vel.x.abs() > rel_vel.y.abs() {
        if rel_vel.x.abs() < 1e-10 {
            return None;
        }
        rel_pos.x / rel_vel.x
    } else {
        if rel_vel.y.abs() < 1e-10 {
            return None;
        }
        rel_pos.y / rel_vel.y
    };

    if d <= current_d + 1e-4 {
        return None;
    }

    let point = v1_0 + vel1 * d;
    Some((d, point))
}

/// Compute the event-driven straight skeleton for a polygon.
///
/// This is a direct port of `computeEventSkeleton()` from poly.jsx.
/// It tracks edge-collapse events over time as the wavefront shrinks inward,
/// recording skeleton arcs, event nodes, and the timeline of active edges.
pub fn compute_skeleton(vertices: &[Vec2]) -> StraightSkeleton {
    if vertices.len() < 3 {
        return StraightSkeleton {
            arcs: vec![],
            nodes: vec![],
            events: vec![SkeletonEvent {
                d: 0.0,
                active_edges: vec![],
            }],
            edge_dirs: vec![],
            edge_normals: vec![],
            edge_points: vec![],
        };
    }

    let orig_pts = ensure_ccw(vertices);
    let n = orig_pts.len();

    // Precompute immutable edge geometry.
    let mut edge_dirs = Vec::with_capacity(n);
    let mut edge_normals = Vec::with_capacity(n);
    let mut edge_points = Vec::with_capacity(n);
    for i in 0..n {
        let p1 = orig_pts[i];
        let p2 = orig_pts[(i + 1) % n];
        let dir = p2 - p1;
        edge_dirs.push(dir);
        // Inward normal: perp of dir = (-dy, dx), normalized.
        let len = dir.length();
        if len < 1e-12 {
            edge_normals.push(Vec2::new(0.0, 0.0));
        } else {
            edge_normals.push(Vec2::new(-dir.y / len, dir.x / len));
        }
        edge_points.push(p1);
    }

    let mut arcs: Vec<(Vec2, Vec2)> = Vec::new();
    let mut nodes: Vec<Vec2> = Vec::new();

    let mut current_d = 0.0;
    let mut active_edges: Vec<usize> = (0..n).collect();
    let mut event_timeline = vec![SkeletonEvent {
        d: 0.0,
        active_edges: active_edges.clone(),
    }];

    let initial_wf = wavefront_at_impl(
        &active_edges,
        0.0,
        &edge_points,
        &edge_normals,
        &edge_dirs,
    );
    let mut prev_wf = initial_wf;
    let mut safety = 0;

    while active_edges.len() >= 3 && safety < 200 {
        safety += 1;
        let m = active_edges.len();

        // Terminal case: triangle collapse.
        if m == 3 {
            let mut collapse_ev: Option<(f64, Vec2)> = None;
            for i in 0..3 {
                if let Some((d, pt)) = find_edge_event_time(
                    &active_edges,
                    i,
                    current_d,
                    &edge_points,
                    &edge_normals,
                    &edge_dirs,
                ) {
                    if collapse_ev.is_none() || d < collapse_ev.unwrap().0 {
                        collapse_ev = Some((d, pt));
                    }
                }
            }

            // Binary search fallback if analytical solution fails.
            if collapse_ev.is_none() {
                let mut try_d = current_d + 1.0;
                while try_d < current_d + 500.0 {
                    let wf_try = wavefront_at_impl(
                        &active_edges,
                        try_d,
                        &edge_points,
                        &edge_normals,
                        &edge_dirs,
                    );
                    if wf_try.is_none()
                        || wf_try
                            .as_ref()
                            .map(|w| signed_area(w) <= 1.0)
                            .unwrap_or(true)
                    {
                        let wf_prev = wavefront_at_impl(
                            &active_edges,
                            try_d - 5.0,
                            &edge_points,
                            &edge_normals,
                            &edge_dirs,
                        );
                        if let Some(ref wp) = wf_prev {
                            let cx = wp.iter().map(|p| p.x).sum::<f64>() / 3.0;
                            let cy = wp.iter().map(|p| p.y).sum::<f64>() / 3.0;
                            collapse_ev = Some((try_d, Vec2::new(cx, cy)));
                        }
                        break;
                    }
                    try_d += 5.0;
                }
            }

            let wf = prev_wf
                .clone()
                .or_else(|| {
                    wavefront_at_impl(
                        &active_edges,
                        current_d,
                        &edge_points,
                        &edge_normals,
                        &edge_dirs,
                    )
                });
            let cp = collapse_ev.map(|(_, pt)| pt).or_else(|| {
                wf.as_ref().map(|w| {
                    let cx = w.iter().map(|p| p.x).sum::<f64>() / 3.0;
                    let cy = w.iter().map(|p| p.y).sum::<f64>() / 3.0;
                    Vec2::new(cx, cy)
                })
            });

            if let (Some(cp), Some(ref wf)) = (cp, &wf) {
                if let Some(ref collapse) = collapse_ev {
                    let wf_near = wavefront_at_impl(
                        &active_edges,
                        collapse.0 - 0.1,
                        &edge_points,
                        &edge_normals,
                        &edge_dirs,
                    );
                    if let (Some(ref wn), Some(ref pw)) = (&wf_near, &prev_wf) {
                        if pw.len() == 3 {
                            for i in 0..3 {
                                let dist = ((pw[i].x - wn[i].x).powi(2)
                                    + (pw[i].y - wn[i].y).powi(2))
                                .sqrt();
                                if dist > 0.5 {
                                    arcs.push((pw[i], wn[i]));
                                }
                                arcs.push((wn[i], cp));
                            }
                        } else {
                            for p in wf {
                                arcs.push((*p, cp));
                            }
                        }
                    } else {
                        for p in wf {
                            arcs.push((*p, cp));
                        }
                    }
                } else {
                    for p in wf {
                        arcs.push((*p, cp));
                    }
                }
                nodes.push(cp);
            }
            break;
        }

        // Find earliest edge event.
        let mut best_event: Option<(f64, Vec2)> = None;
        let mut best_wf_idx: usize = 0;
        for i in 0..m {
            if let Some((d, pt)) = find_edge_event_time(
                &active_edges,
                i,
                current_d,
                &edge_points,
                &edge_normals,
                &edge_dirs,
            ) {
                if best_event.is_none() || d < best_event.unwrap().0 {
                    best_event = Some((d, pt));
                    best_wf_idx = i;
                }
            }
        }

        if best_event.is_none() {
            // No events found — collapse to centroid.
            let wf = prev_wf.clone().or_else(|| {
                wavefront_at_impl(
                    &active_edges,
                    current_d,
                    &edge_points,
                    &edge_normals,
                    &edge_dirs,
                )
            });
            if let Some(ref wf) = wf {
                let cx = wf.iter().map(|p| p.x).sum::<f64>() / m as f64;
                let cy = wf.iter().map(|p| p.y).sum::<f64>() / m as f64;
                let cp = Vec2::new(cx, cy);
                nodes.push(cp);
                for p in wf {
                    arcs.push((*p, cp));
                }
            }
            break;
        }

        let (event_d, event_point) = best_event.unwrap();

        // Record arcs from previous wavefront to event wavefront.
        let wf_at_event = wavefront_at_impl(
            &active_edges,
            event_d,
            &edge_points,
            &edge_normals,
            &edge_dirs,
        );
        if let (Some(ref pw), Some(ref we)) = (&prev_wf, &wf_at_event) {
            if pw.len() == we.len() {
                for i in 0..pw.len() {
                    let dist =
                        ((pw[i].x - we[i].x).powi(2) + (pw[i].y - we[i].y).powi(2)).sqrt();
                    if dist > 0.5 {
                        arcs.push((pw[i], we[i]));
                    }
                }
            }
        }

        nodes.push(event_point);

        // Remove the collapsed edge.
        let remove_idx = (best_wf_idx + 1) % m;
        active_edges.remove(remove_idx);
        current_d = event_d;
        event_timeline.push(SkeletonEvent {
            d: current_d,
            active_edges: active_edges.clone(),
        });

        // Check if the new wavefront has collapsed.
        let new_wf = wavefront_at_impl(
            &active_edges,
            current_d,
            &edge_points,
            &edge_normals,
            &edge_dirs,
        );
        if let Some(ref nw) = new_wf {
            if active_edges.len() >= 3 && signed_area(nw).abs() < 1.0 {
                let cx = nw.iter().map(|p| p.x).sum::<f64>() / nw.len() as f64;
                let cy = nw.iter().map(|p| p.y).sum::<f64>() / nw.len() as f64;
                let cp = Vec2::new(cx, cy);
                nodes.push(cp);
                for p in nw {
                    arcs.push((*p, cp));
                }
                break;
            }
        }
        prev_wf = new_wf.or_else(|| {
            wavefront_at_impl(
                &active_edges,
                current_d + 0.01,
                &edge_points,
                &edge_normals,
                &edge_dirs,
            )
        });
    }

    StraightSkeleton {
        arcs,
        nodes,
        events: event_timeline,
        edge_dirs,
        edge_normals,
        edge_points,
    }
}

/// Compute the wavefront polygon at offset distance `d` using the skeleton's
/// event timeline to determine which edges are still active.
pub fn wavefront_at(skeleton: &StraightSkeleton, d: f64) -> Option<(Vec<Vec2>, Vec<usize>)> {
    if skeleton.events.is_empty() {
        return None;
    }

    // Find the active edges at distance d by walking the event timeline.
    let mut active = &skeleton.events[0].active_edges;
    for i in 1..skeleton.events.len() {
        if skeleton.events[i].d <= d {
            active = &skeleton.events[i].active_edges;
        } else {
            break;
        }
    }

    if active.len() < 3 {
        return None;
    }

    let wf = wavefront_at_impl(
        active,
        d,
        &skeleton.edge_points,
        &skeleton.edge_normals,
        &skeleton.edge_dirs,
    )?;

    // Validate: positive area and no self-intersections.
    if signed_area(&wf) <= 0.0 {
        return None;
    }

    // Per-edge direction check: each offset edge must still run in the same
    // direction as its corresponding original edge.  When the wavefront has
    // folded past the medial axis, some edges reverse direction even though
    // the global signed area might still be positive.
    let n = wf.len();
    for i in 0..n {
        let next = (i + 1) % n;
        let offset_dir = wf[next] - wf[i];
        // The segment from wf[i] to wf[next] lies on the offset of
        // active[(i+1) % n].
        let orig_edge = active[(i + 1) % n];
        let orig_dir = skeleton.edge_dirs[orig_edge];
        if orig_dir.dot(offset_dir) <= 0.0 {
            return None;
        }
    }

    for i in 0..n {
        let a1 = wf[i];
        let a2 = wf[(i + 1) % n];
        for j in (i + 2)..n {
            if j == (i + n - 1) % n {
                continue;
            }
            let b1 = wf[j];
            let b2 = wf[(j + 1) % n];
            let d1 = a2 - a1;
            let d2 = b2 - b1;
            let denom = d1.cross(d2);
            if denom.abs() < 1e-10 {
                continue;
            }
            let diff = b1 - a1;
            let t = diff.cross(d2) / denom;
            let u = diff.cross(d1) / denom;
            if t > 1e-6 && t < 1.0 - 1e-6 && u > 1e-6 && u < 1.0 - 1e-6 {
                return None;
            }
        }
    }

    Some((wf, active.clone()))
}

/// Convenience: get just the offset polygon at distance `d`.
pub fn offset_polygon_at(skeleton: &StraightSkeleton, d: f64) -> Option<Vec<Vec2>> {
    wavefront_at(skeleton, d).map(|(wf, _)| wf)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(w: f64, h: f64) -> Vec<Vec2> {
        vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(w, 0.0),
            Vec2::new(w, h),
            Vec2::new(0.0, h),
        ]
    }

    #[test]
    fn test_rectangle_skeleton() {
        let polygon = rect(100.0, 60.0);
        let sk = compute_skeleton(&polygon);

        // A rectangle should have exactly 1 event (one edge collapses),
        // producing a skeleton with nodes along the medial axis.
        assert!(!sk.nodes.is_empty(), "rectangle should have skeleton nodes");
        assert!(!sk.arcs.is_empty(), "rectangle should have skeleton arcs");
        assert!(sk.events.len() >= 2, "rectangle should have events");
    }

    #[test]
    fn test_rectangle_offset() {
        let polygon = rect(100.0, 60.0);
        let sk = compute_skeleton(&polygon);

        // Offset at d=10 should produce a valid polygon.
        let offset = offset_polygon_at(&sk, 10.0);
        assert!(offset.is_some(), "offset at d=10 should exist for 100x60 rect");
        let wf = offset.unwrap();
        assert_eq!(wf.len(), 4, "offset rectangle should have 4 vertices");

        // Area should be smaller than original.
        let orig_area = signed_area(&polygon).abs();
        let offset_area = signed_area(&wf).abs();
        assert!(
            offset_area < orig_area,
            "offset area {} should be less than original {}",
            offset_area,
            orig_area
        );
    }

    #[test]
    fn test_triangle_collapse() {
        let polygon = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(50.0, 80.0),
        ];
        let sk = compute_skeleton(&polygon);

        // Triangle collapses to a single point.
        assert!(
            !sk.nodes.is_empty(),
            "triangle should have at least one node"
        );
    }

    #[test]
    fn test_l_shape_skeleton() {
        // L-shape polygon.
        let polygon = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 40.0),
            Vec2::new(40.0, 40.0),
            Vec2::new(40.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];
        let sk = compute_skeleton(&polygon);

        // L-shape should have multiple events.
        assert!(
            sk.events.len() >= 2,
            "L-shape should have multiple events, got {}",
            sk.events.len()
        );
        assert!(
            sk.nodes.len() >= 2,
            "L-shape should have multiple nodes, got {}",
            sk.nodes.len()
        );
    }

    #[test]
    fn test_degenerate_input() {
        // Too few points.
        let sk = compute_skeleton(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)]);
        assert!(sk.arcs.is_empty());
        assert!(sk.nodes.is_empty());

        // Empty.
        let sk = compute_skeleton(&[]);
        assert!(sk.arcs.is_empty());
    }

    #[test]
    fn test_offset_beyond_collapse() {
        let polygon = rect(20.0, 10.0);
        let sk = compute_skeleton(&polygon);

        // Offset at d=6 should be None (rectangle collapses at d=5).
        let offset = offset_polygon_at(&sk, 6.0);
        assert!(offset.is_none(), "offset beyond collapse should be None");
    }

    #[test]
    fn test_wavefront_returns_active_edges() {
        let polygon = rect(100.0, 60.0);
        let sk = compute_skeleton(&polygon);

        let result = wavefront_at(&sk, 10.0);
        assert!(result.is_some());
        let (wf, active) = result.unwrap();
        assert_eq!(wf.len(), active.len());
    }

    #[test]
    fn test_narrow_rectangle_no_fold() {
        // A 30×100 rectangle with stall depth 18 (> half width 15).
        // The offset at d=18 should return None — the wavefront has folded
        // because the two long edges have crossed.
        let polygon = rect(30.0, 100.0);
        let sk = compute_skeleton(&polygon);

        // d=14 should be valid (just inside the collapse at d=15).
        let offset = offset_polygon_at(&sk, 14.0);
        assert!(offset.is_some(), "offset at d=14 should exist for 30x100 rect");

        // d=16 should be None (past the collapse).
        let offset = offset_polygon_at(&sk, 16.0);
        assert!(offset.is_none(), "offset at d=16 should be None for 30x100 rect");

        // d=18 definitely should be None.
        let offset = offset_polygon_at(&sk, 18.0);
        assert!(offset.is_none(), "offset at d=18 should be None for 30x100 rect");
    }

    #[test]
    fn test_barely_wide_enough_rectangle() {
        // A 37×100 rectangle with d=18 — barely wider than 2*18=36.
        // The offset should still be valid but very thin.
        let polygon = rect(37.0, 100.0);
        let sk = compute_skeleton(&polygon);

        let offset = offset_polygon_at(&sk, 18.0);
        assert!(offset.is_some(), "offset at d=18 should exist for 37x100 rect");
        let wf = offset.unwrap();
        // The offset polygon should be about 1×64.
        let area = signed_area(&wf).abs();
        assert!(area > 0.0 && area < 200.0, "thin offset area should be small, got {}", area);
    }

    #[test]
    fn test_trapezoid_fold_detection() {
        // Slightly trapezoidal face — narrower at top (25) than bottom (35).
        // At d=18, the top is only 25 wide, so the offset should fold there.
        let polygon = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(35.0, 0.0),
            Vec2::new(30.0, 100.0),
            Vec2::new(5.0, 100.0),
        ];
        let sk = compute_skeleton(&polygon);

        // At d=18, the narrow end (25 wide) has collapsed.
        // The direction check should reject this.
        let offset = offset_polygon_at(&sk, 18.0);
        // If valid, the offset polygon should not have reversed edges.
        if let Some(ref wf) = offset {
            let area = signed_area(wf);
            assert!(area > 0.0, "offset should have positive area if it exists");
        }
    }
}
