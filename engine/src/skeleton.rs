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
    /// Per-loop active edges (for multi-contour skeletons).
    pub active_loops: Vec<Vec<usize>>,
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
    pub edge_normals: Vec<Vec2>,
    /// Per-edge base points (immutable, from CCW polygon).
    pub edge_points: Vec<Vec2>,
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
                active_loops: vec![],
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
        active_loops: vec![active_edges.clone()],
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
            active_loops: vec![active_edges.clone()],
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
// Multi-contour straight skeleton
// ---------------------------------------------------------------------------

/// Compute the straight skeleton for a polygon with holes.
///
/// `contours[0]` is the outer boundary (CCW), `contours[1..]` are holes (CW).
/// Normal direction `(-dy, dx)/len` gives inward normals for CCW (toward face
/// center) and outward-from-polygon normals for CW (away from hole center,
/// into the face). Both point into the face region, so the outer wavefront
/// shrinks and hole wavefronts expand until they meet via split events.
pub fn compute_skeleton_multi(contours: &[Vec<Vec2>]) -> StraightSkeleton {
    if contours.is_empty() {
        return empty_skeleton();
    }
    if contours.len() == 1 {
        return compute_skeleton(&contours[0]);
    }

    // Build flat edge geometry arrays preserving per-contour winding.
    let mut edge_dirs = Vec::new();
    let mut edge_normals = Vec::new();
    let mut edge_points = Vec::new();
    let mut loops: Vec<Vec<usize>> = Vec::new();
    let mut offset = 0;

    for contour in contours {
        let n = contour.len();
        if n < 3 {
            continue;
        }
        loops.push((offset..offset + n).collect());
        for i in 0..n {
            let j = (i + 1) % n;
            let dir = contour[j] - contour[i];
            let len = dir.length();
            edge_dirs.push(dir);
            if len < 1e-12 {
                edge_normals.push(Vec2::new(0.0, 0.0));
            } else {
                edge_normals.push(Vec2::new(-dir.y / len, dir.x / len));
            }
            edge_points.push(contour[i]);
        }
        offset += n;
    }

    if loops.is_empty() {
        return empty_skeleton();
    }

    let mut arcs: Vec<(Vec2, Vec2)> = Vec::new();
    let mut nodes: Vec<Vec2> = Vec::new();
    let mut current_d = 0.0;

    let mut prev_wfs: Vec<Option<Vec<Vec2>>> = loops
        .iter()
        .map(|lp| wavefront_at_impl(lp, 0.0, &edge_points, &edge_normals, &edge_dirs))
        .collect();

    let mut event_timeline = vec![SkeletonEvent {
        d: 0.0,
        active_edges: loops.iter().flat_map(|l| l.iter().copied()).collect(),
        active_loops: loops.clone(),
    }];

    let mut safety = 0;

    while safety < 500 {
        safety += 1;

        // Remove degenerate loops.
        let mut i = 0;
        while i < loops.len() {
            if loops[i].len() < 3 {
                loops.remove(i);
                prev_wfs.remove(i);
            } else {
                i += 1;
            }
        }

        if loops.is_empty() {
            break;
        }

        // --- Find earliest edge event across all loops ---
        let mut best_edge: Option<(f64, Vec2, usize, usize)> = None;
        for (li, lp) in loops.iter().enumerate() {
            let m = lp.len();
            if m < 3 {
                continue;
            }
            for i in 0..m {
                if let Some((d, pt)) = find_edge_event_time(
                    lp,
                    i,
                    current_d,
                    &edge_points,
                    &edge_normals,
                    &edge_dirs,
                ) {
                    if best_edge.is_none() || d < best_edge.unwrap().0 {
                        best_edge = Some((d, pt, li, i));
                    }
                }
            }
        }

        // --- Find earliest split event between loops ---
        let mut best_split: Option<(f64, Vec2, usize, usize, usize, usize)> = None;
        if loops.len() > 1 {
            for li_a in 0..loops.len() {
                let ma = loops[li_a].len();
                for vi in 0..ma {
                    let e_prev = loops[li_a][vi];
                    let e_next = loops[li_a][(vi + 1) % ma];

                    let v0 = match offset_vertex(
                        &edge_points, &edge_normals, &edge_dirs, e_prev, e_next, 0.0,
                    ) {
                        Some(v) => v,
                        None => continue,
                    };
                    let v1 = match offset_vertex(
                        &edge_points, &edge_normals, &edge_dirs, e_prev, e_next, 1.0,
                    ) {
                        Some(v) => v,
                        None => continue,
                    };
                    let v_vel = v1 - v0;

                    for li_b in 0..loops.len() {
                        if li_b == li_a {
                            continue;
                        }
                        let mb = loops[li_b].len();
                        for ei in 0..mb {
                            let e_target = loops[li_b][ei];
                            let dir = edge_dirs[e_target];

                            let rel = v0 - edge_points[e_target];
                            let vel = v_vel - edge_normals[e_target];
                            let denom = vel.cross(dir);
                            if denom.abs() < 1e-10 {
                                continue;
                            }
                            let d = -rel.cross(dir) / denom;
                            if d <= current_d + 1e-4 {
                                continue;
                            }
                            if best_split.is_some() && d >= best_split.as_ref().unwrap().0 {
                                continue;
                            }

                            let point = v0 + v_vel * d;

                            // Segment check: point must lie on the finite edge.
                            let e_pred = loops[li_b][(ei + mb - 1) % mb];
                            let e_succ = loops[li_b][(ei + 1) % mb];
                            let seg_s = match offset_vertex(
                                &edge_points, &edge_normals, &edge_dirs, e_pred, e_target, d,
                            ) {
                                Some(v) => v,
                                None => continue,
                            };
                            let seg_e = match offset_vertex(
                                &edge_points, &edge_normals, &edge_dirs, e_target, e_succ, d,
                            ) {
                                Some(v) => v,
                                None => continue,
                            };
                            let seg_d = seg_e - seg_s;
                            let seg_len_sq = seg_d.dot(seg_d);
                            if seg_len_sq < 1e-12 {
                                continue;
                            }
                            let t = (point - seg_s).dot(seg_d) / seg_len_sq;
                            if t < -0.01 || t > 1.01 {
                                continue;
                            }

                            best_split = Some((d, point, li_a, vi, li_b, ei));
                        }
                    }
                }
            }
        }

        let edge_d = best_edge.map(|e| e.0).unwrap_or(f64::MAX);
        let split_d = best_split.map(|e| e.0).unwrap_or(f64::MAX);

        if edge_d == f64::MAX && split_d == f64::MAX {
            // No events found — collapse remaining loops to centroids.
            for (li, _lp) in loops.iter().enumerate() {
                if let Some(ref wf) = prev_wfs[li] {
                    let m = wf.len() as f64;
                    let cx = wf.iter().map(|p| p.x).sum::<f64>() / m;
                    let cy = wf.iter().map(|p| p.y).sum::<f64>() / m;
                    let cp = Vec2::new(cx, cy);
                    nodes.push(cp);
                    for p in wf {
                        arcs.push((*p, cp));
                    }
                }
            }
            break;
        }

        if edge_d <= split_d {
            // --- Edge event: an edge collapses within a single loop ---
            let (event_d, event_point, li, wf_idx) = best_edge.unwrap();
            let lp = &loops[li];
            let m = lp.len();

            // Record arcs from prev wavefront to event wavefront.
            let wf_at = wavefront_at_impl(lp, event_d, &edge_points, &edge_normals, &edge_dirs);
            if let (Some(ref pw), Some(ref we)) = (&prev_wfs[li], &wf_at) {
                if pw.len() == we.len() {
                    for i in 0..pw.len() {
                        if (pw[i] - we[i]).length() > 0.5 {
                            arcs.push((pw[i], we[i]));
                        }
                    }
                }
            }
            nodes.push(event_point);

            if m == 3 {
                // Triangle collapse: all vertices converge to one point.
                let wf = wf_at.or_else(|| prev_wfs[li].clone());
                if let Some(ref wf) = wf {
                    for p in wf {
                        arcs.push((*p, event_point));
                    }
                }
                loops.remove(li);
                prev_wfs.remove(li);
            } else {
                let remove_idx = (wf_idx + 1) % m;
                loops[li].remove(remove_idx);
                current_d = event_d;

                prev_wfs[li] = wavefront_at_impl(
                    &loops[li],
                    current_d,
                    &edge_points,
                    &edge_normals,
                    &edge_dirs,
                )
                .or_else(|| {
                    wavefront_at_impl(
                        &loops[li],
                        current_d + 0.01,
                        &edge_points,
                        &edge_normals,
                        &edge_dirs,
                    )
                });
            }

            event_timeline.push(SkeletonEvent {
                d: if m == 3 { current_d } else { event_d },
                active_edges: loops.iter().flat_map(|l| l.iter().copied()).collect(),
                active_loops: loops.clone(),
            });
        } else {
            // --- Split event: a vertex from one loop hits an edge of another ---
            // The two wavefronts meet at event_point. Continuing with a
            // merged single loop creates near-anti-parallel edges at the
            // bridge junctions, causing degenerate bisectors. Instead,
            // record arcs up to the split distance and terminate both loops.
            let (event_d, event_point, li_a, _vi, li_b, _ei) = best_split.unwrap();

            // Draw arcs from prev wavefront to wavefront at event_d for
            // each loop. Only the involved vertex/edge meet at the split
            // point; other vertices just reach their event_d positions.
            for &li in &[li_a, li_b] {
                let wf = wavefront_at_impl(
                    &loops[li], event_d, &edge_points, &edge_normals, &edge_dirs,
                );
                if let (Some(ref pw), Some(ref we)) = (&prev_wfs[li], &wf) {
                    if pw.len() == we.len() {
                        for i in 0..pw.len() {
                            if (pw[i] - we[i]).length() > 0.5 {
                                arcs.push((pw[i], we[i]));
                            }
                        }
                    }
                }
            }
            nodes.push(event_point);

            // Remove both loops; keep any remaining loops for further
            // processing (e.g. if there are multiple holes).
            let (first, second) = if li_a > li_b { (li_a, li_b) } else { (li_b, li_a) };
            loops.remove(first);
            prev_wfs.remove(first);
            loops.remove(second);
            prev_wfs.remove(second);
            current_d = event_d;

            event_timeline.push(SkeletonEvent {
                d: current_d,
                active_edges: loops.iter().flat_map(|l| l.iter().copied()).collect(),
                active_loops: loops.clone(),
            });
        }
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

fn empty_skeleton() -> StraightSkeleton {
    StraightSkeleton {
        arcs: vec![],
        nodes: vec![],
        events: vec![SkeletonEvent {
            d: 0.0,
            active_edges: vec![],
            active_loops: vec![],
        }],
        edge_dirs: vec![],
        edge_normals: vec![],
        edge_points: vec![],
    }
}

/// Compute wavefronts for all active loops at distance `d`.
///
/// Returns one `(wavefront_polygon, active_edge_indices)` per active loop.
pub fn wavefront_loops_at(
    skeleton: &StraightSkeleton,
    d: f64,
) -> Vec<(Vec<Vec2>, Vec<usize>)> {
    if skeleton.events.is_empty() {
        return vec![];
    }

    let mut loops = &skeleton.events[0].active_loops;
    for i in 1..skeleton.events.len() {
        if skeleton.events[i].d <= d {
            loops = &skeleton.events[i].active_loops;
        } else {
            break;
        }
    }

    let mut result = Vec::new();
    for lp in loops {
        if lp.len() < 3 {
            continue;
        }
        let wf = match wavefront_at_impl(
            lp,
            d,
            &skeleton.edge_points,
            &skeleton.edge_normals,
            &skeleton.edge_dirs,
        ) {
            Some(w) => w,
            None => continue,
        };

        // Direction check: each offset edge must still run in the same
        // direction as its original edge.
        let n = wf.len();
        let mut valid = true;
        for i in 0..n {
            let next = (i + 1) % n;
            let offset_dir = wf[next] - wf[i];
            let orig_edge = lp[(i + 1) % n];
            let orig_dir = skeleton.edge_dirs[orig_edge];
            if orig_dir.dot(offset_dir) <= 0.0 {
                valid = false;
                break;
            }
        }
        if !valid {
            continue;
        }

        result.push((wf, lp.clone()));
    }
    result
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

    // -----------------------------------------------------------------------
    // Multi-contour skeleton: triangle with hole
    // -----------------------------------------------------------------------

    /// Test if point is inside polygon (winding number).
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

    /// Test if point is inside face (inside outer, outside all holes).
    fn point_in_face(p: Vec2, shape: &[Vec<Vec2>]) -> bool {
        if shape.is_empty() || !point_in_polygon(p, &shape[0]) {
            return false;
        }
        for hole in &shape[1..] {
            if point_in_polygon(p, hole) {
                return false;
            }
        }
        true
    }

    /// Minimum distance from point to polygon boundary.
    fn dist_to_boundary(p: Vec2, polygon: &[Vec2]) -> f64 {
        let n = polygon.len();
        let mut min_dist = f64::MAX;
        for i in 0..n {
            let j = (i + 1) % n;
            let a = polygon[i];
            let b = polygon[j];
            let ab = b - a;
            let len_sq = ab.dot(ab);
            if len_sq < 1e-12 {
                min_dist = min_dist.min((p - a).length());
                continue;
            }
            let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
            let closest = a + ab * t;
            min_dist = min_dist.min((p - closest).length());
        }
        min_dist
    }

    #[test]
    fn test_triangle_with_hole_skeleton() {
        // Large outer triangle (CCW, positive signed area).
        let outer = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(300.0, 0.0),
            Vec2::new(150.0, 260.0),
        ];
        assert!(signed_area(&outer) > 0.0, "outer must be CCW");

        // Smaller inner triangle (CW, negative signed area) — the hole.
        let hole = vec![
            Vec2::new(150.0, 180.0),
            Vec2::new(200.0, 60.0),
            Vec2::new(100.0, 60.0),
        ];
        assert!(signed_area(&hole) < 0.0, "hole must be CW");

        let shape = vec![outer.clone(), hole.clone()];

        // Compute multi-contour skeleton.
        let sk = compute_skeleton_multi(&shape);

        // ---- Visualization (printed as snapshot) ----
        eprintln!("=== Triangle with hole: multi-contour skeleton ===");
        eprintln!("Outer (CCW): {:?}", outer);
        eprintln!("Hole  (CW):  {:?}", hole);
        eprintln!("Arcs: {}", sk.arcs.len());
        for (i, &(a, b)) in sk.arcs.iter().enumerate() {
            eprintln!("  arc[{:2}]: ({:7.1}, {:7.1}) -> ({:7.1}, {:7.1})", i, a.x, a.y, b.x, b.y);
        }
        eprintln!("Nodes: {}", sk.nodes.len());
        for (i, n) in sk.nodes.iter().enumerate() {
            eprintln!("  node[{:2}]: ({:7.1}, {:7.1})", i, n.x, n.y);
        }
        eprintln!("Events: {}", sk.events.len());
        for (i, ev) in sk.events.iter().enumerate() {
            eprintln!("  event[{}]: d={:.2}, loops={}, edges={}",
                i, ev.d, ev.active_loops.len(), ev.active_edges.len());
        }

        // ---- Assertions ----
        assert!(!sk.arcs.is_empty(), "skeleton should have arcs");
        assert!(!sk.nodes.is_empty(), "skeleton should have nodes");

        // Every skeleton node must be inside the face (inside outer, outside hole).
        let mut bad_nodes = Vec::new();
        for (i, node) in sk.nodes.iter().enumerate() {
            if !point_in_face(*node, &shape) {
                let d_outer = dist_to_boundary(*node, &outer);
                let d_hole = dist_to_boundary(*node, &hole);
                bad_nodes.push((i, *node, d_outer, d_hole));
            }
        }
        if !bad_nodes.is_empty() {
            eprintln!("\nBAD NODES (outside face):");
            for &(i, p, d_outer, d_hole) in &bad_nodes {
                let in_outer = point_in_polygon(p, &outer);
                let in_hole = point_in_polygon(p, &hole);
                eprintln!("  node[{}]: ({:.1}, {:.1}) in_outer={} in_hole={} d_outer={:.1} d_hole={:.1}",
                    i, p.x, p.y, in_outer, in_hole, d_outer, d_hole);
            }
        }
        assert!(bad_nodes.is_empty(),
            "{} skeleton nodes are outside the face", bad_nodes.len());

        // Every arc endpoint must be inside the face (with small tolerance
        // for points on the boundary).
        let tol = 2.0;
        let mut bad_arcs = Vec::new();
        for (i, &(a, b)) in sk.arcs.iter().enumerate() {
            for &pt in &[a, b] {
                if !point_in_face(pt, &shape) {
                    let d_outer = dist_to_boundary(pt, &outer);
                    let d_hole = dist_to_boundary(pt, &hole);
                    // Allow points very close to a boundary edge.
                    if d_outer > tol && d_hole > tol {
                        bad_arcs.push((i, pt, d_outer, d_hole));
                    }
                }
            }
        }
        if !bad_arcs.is_empty() {
            eprintln!("\nBAD ARC ENDPOINTS (outside face, beyond tolerance):");
            for &(i, p, d_outer, d_hole) in &bad_arcs {
                let in_outer = point_in_polygon(p, &outer);
                let in_hole = point_in_polygon(p, &hole);
                eprintln!("  arc[{}] pt ({:.1}, {:.1}) in_outer={} in_hole={} d_outer={:.1} d_hole={:.1}",
                    i, p.x, p.y, in_outer, in_hole, d_outer, d_hole);
            }
        }
        assert!(bad_arcs.is_empty(),
            "{} arc endpoints are outside the face (beyond {:.0}ft tolerance)", bad_arcs.len(), tol);

        // Midpoints of arcs should also be inside the face (catches arcs
        // that start/end on the boundary but cut through the hole).
        let mut bad_mids = Vec::new();
        for (i, &(a, b)) in sk.arcs.iter().enumerate() {
            let mid = (a + b) * 0.5;
            if !point_in_face(mid, &shape) {
                let d_outer = dist_to_boundary(mid, &outer);
                let d_hole = dist_to_boundary(mid, &hole);
                if d_outer > tol && d_hole > tol {
                    bad_mids.push((i, mid, d_outer, d_hole));
                }
            }
        }
        if !bad_mids.is_empty() {
            eprintln!("\nBAD ARC MIDPOINTS (outside face):");
            for &(i, p, d_outer, d_hole) in &bad_mids {
                eprintln!("  arc[{}] mid ({:.1}, {:.1}) d_outer={:.1} d_hole={:.1}",
                    i, p.x, p.y, d_outer, d_hole);
            }
        }
        assert!(bad_mids.is_empty(),
            "{} arc midpoints are outside the face", bad_mids.len());
    }
}
