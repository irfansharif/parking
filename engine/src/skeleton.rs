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
    /// Split event points (subset of nodes where the wavefront split).
    pub split_nodes: Vec<Vec2>,
    /// Timeline of edge-collapse events.
    pub events: Vec<SkeletonEvent>,
    /// Per-edge directions (immutable, from CCW polygon).
    pub edge_dirs: Vec<Vec2>,
    /// Per-edge inward normals (immutable, from CCW polygon).
    pub edge_normals: Vec<Vec2>,
    /// Per-edge base points (immutable, from CCW polygon).
    pub edge_points: Vec<Vec2>,
    /// Per-edge weights for the weighted straight skeleton. Weight 1.0 means
    /// the edge shrinks at normal speed; weight 0.0 means it stays fixed
    /// (e.g. a boundary wall in a one-sided perimeter face).
    pub edge_weights: Vec<f64>,
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
/// The edge weight scales how far the edge moves: weight 1.0 = normal,
/// weight 0.0 = stationary (boundary wall).
fn get_offset_line(
    edge_points: &[Vec2],
    edge_normals: &[Vec2],
    edge_dirs: &[Vec2],
    edge_weights: &[f64],
    e_idx: usize,
    d: f64,
) -> (Vec2, Vec2) {
    let point = edge_points[e_idx] + edge_normals[e_idx] * d * edge_weights[e_idx];
    let dir = edge_dirs[e_idx];
    (point, dir)
}

/// Compute the vertex where the offset lines of edges `e1` and `e2` meet at
/// distance `d`.
fn offset_vertex(
    edge_points: &[Vec2],
    edge_normals: &[Vec2],
    edge_dirs: &[Vec2],
    edge_weights: &[f64],
    e1: usize,
    e2: usize,
    d: f64,
) -> Option<Vec2> {
    let (p1, d1) = get_offset_line(edge_points, edge_normals, edge_dirs, edge_weights, e1, d);
    let (p2, d2) = get_offset_line(edge_points, edge_normals, edge_dirs, edge_weights, e2, d);
    line_intersect(p1, d1, p2, d2)
}

/// Compute the wavefront polygon for a set of active edges at distance `d`.
fn wavefront_at_impl(
    edges: &[usize],
    d: f64,
    edge_points: &[Vec2],
    edge_normals: &[Vec2],
    edge_dirs: &[Vec2],
    edge_weights: &[f64],
) -> Option<Vec<Vec2>> {
    let mut pts = Vec::with_capacity(edges.len());
    for i in 0..edges.len() {
        let p = offset_vertex(
            edge_points,
            edge_normals,
            edge_dirs,
            edge_weights,
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
    edge_weights: &[f64],
) -> Option<(f64, Vec2)> {
    let m = edges.len();
    let e_a = edges[wf_idx];
    let e_b = edges[(wf_idx + 1) % m];
    let e_c = edges[(wf_idx + 2) % m];

    let v1_0 = offset_vertex(edge_points, edge_normals, edge_dirs, edge_weights, e_a, e_b, 0.0)?;
    let v1_1 = offset_vertex(edge_points, edge_normals, edge_dirs, edge_weights, e_a, e_b, 1.0)?;
    let v2_0 = offset_vertex(edge_points, edge_normals, edge_dirs, edge_weights, e_b, e_c, 0.0)?;
    let v2_1 = offset_vertex(edge_points, edge_normals, edge_dirs, edge_weights, e_b, e_c, 1.0)?;

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

    // Verify both vertices actually converge to the same point.
    // The single-component solve above finds when they match in one axis;
    // if they don't also match in the other axis, the vertices pass each
    // other without colliding (no true edge event).
    let p1 = v1_0 + vel1 * d;
    let p2 = v2_0 + vel2 * d;
    if (p1 - p2).length() > 1e-2 {
        return None;
    }

    let point = p1;
    Some((d, point))
}

/// Compute the event-driven straight skeleton for a polygon.
///
/// Delegates to `compute_skeleton_multi` which handles both edge-collapse
/// and split events (where a reflex vertex hits a non-adjacent edge,
/// splitting the wavefront into independent sub-polygons).
pub fn compute_skeleton(vertices: &[Vec2]) -> StraightSkeleton {
    if vertices.len() < 3 {
        return empty_skeleton();
    }
    let ccw = ensure_ccw(vertices);
    let n = ccw.len();
    compute_skeleton_multi(&[ccw], &vec![1.0; n])
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
        &skeleton.edge_weights,
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

/// Split a loop of edge indices into two sub-loops at an intra-loop split event.
///
/// Vertex `vi` (between `loop_edges[vi]` and `loop_edges[(vi+1)%m]`) hits
/// edge `loop_edges[ej]`. Both resulting sub-loops include the hit edge.
fn split_loop_edges(loop_edges: &[usize], vi: usize, ej: usize) -> (Vec<usize>, Vec<usize>) {
    let m = loop_edges.len();

    // Loop A: edges from (vi+1)%m to ej, inclusive.
    let mut a = Vec::new();
    let mut idx = (vi + 1) % m;
    loop {
        a.push(loop_edges[idx]);
        if idx == ej {
            break;
        }
        idx = (idx + 1) % m;
    }

    // Loop B: edges from ej to vi, inclusive.
    let mut b = Vec::new();
    idx = ej;
    loop {
        b.push(loop_edges[idx]);
        if idx == vi {
            break;
        }
        idx = (idx + 1) % m;
    }

    (a, b)
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
///
/// `weights` gives a per-edge weight (flat across all contours, matching the
/// order edges are appended: contours[0] edges, then contours[1], …).
/// Weight 1.0 = normal shrinkage, 0.0 = stationary (boundary wall).
pub fn compute_skeleton_multi(contours: &[Vec<Vec2>], weights: &[f64]) -> StraightSkeleton {
    if contours.is_empty() {
        return empty_skeleton();
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

    // Build edge_weights matching the flat edge layout. If the caller
    // provided fewer weights than edges, default missing ones to 1.0.
    let edge_weights: Vec<f64> = (0..edge_points.len())
        .map(|i| if i < weights.len() { weights[i] } else { 1.0 })
        .collect();

    let mut arcs: Vec<(Vec2, Vec2)> = Vec::new();
    let mut nodes: Vec<Vec2> = Vec::new();
    let mut split_nodes: Vec<Vec2> = Vec::new();
    let mut current_d = 0.0;

    let mut prev_wfs: Vec<Option<Vec<Vec2>>> = loops
        .iter()
        .map(|lp| wavefront_at_impl(lp, 0.0, &edge_points, &edge_normals, &edge_dirs, &edge_weights))
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
        // Collect ALL events at the earliest time (within tolerance) so
        // simultaneous collapses (e.g. both short edges of a rectangle)
        // are handled in a single iteration.
        let mut best_edge: Option<(f64, Vec2, usize, usize)> = None;
        let mut all_edge_events: Vec<(f64, Vec2, usize, usize)> = Vec::new();
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
                    &edge_weights,
                ) {
                    all_edge_events.push((d, pt, li, i));
                    if best_edge.is_none() || d < best_edge.unwrap().0 {
                        best_edge = Some((d, pt, li, i));
                    }
                }
            }
        }

        // --- Find earliest split event (inter-loop and intra-loop) ---
        // Inter-loop: vertex from one loop hits edge of another (e.g. outer
        //   wavefront meets hole wavefront).
        // Intra-loop: reflex vertex hits a non-adjacent edge within the same
        //   loop, splitting it into two independent sub-polygons.
        let mut best_split: Option<(f64, Vec2, usize, usize, usize, usize)> = None;
        for li_a in 0..loops.len() {
            let ma = loops[li_a].len();
            if ma < 3 {
                continue;
            }
            for vi in 0..ma {
                let e_prev = loops[li_a][vi];
                let e_next = loops[li_a][(vi + 1) % ma];

                let v0 = match offset_vertex(
                    &edge_points, &edge_normals, &edge_dirs, &edge_weights, e_prev, e_next, 0.0,
                ) {
                    Some(v) => v,
                    None => continue,
                };
                let v1 = match offset_vertex(
                    &edge_points, &edge_normals, &edge_dirs, &edge_weights, e_prev, e_next, 1.0,
                ) {
                    Some(v) => v,
                    None => continue,
                };
                let v_vel = v1 - v0;

                for li_b in 0..loops.len() {
                    let mb = loops[li_b].len();
                    for ei in 0..mb {
                        if li_a == li_b {
                            // Intra-loop: only reflex vertices produce splits.
                            // For CCW winding, reflex = cross(incoming, outgoing) < 0.
                            if edge_dirs[e_prev].cross(edge_dirs[e_next]) >= 0.0 {
                                continue;
                            }
                            // Skip edges adjacent to vertex vi.
                            if ei == vi || ei == (vi + 1) % ma {
                                continue;
                            }
                            // Both resulting sub-loops must have >= 3 edges.
                            let start = (vi + 1) % ma;
                            let size_a = ((ei as isize - start as isize)
                                .rem_euclid(ma as isize))
                                as usize
                                + 1;
                            let size_b =
                                ((vi as isize - ei as isize).rem_euclid(ma as isize)) as usize
                                    + 1;
                            if size_a < 3 || size_b < 3 {
                                continue;
                            }
                        }

                        let e_target = loops[li_b][ei];
                        let dir = edge_dirs[e_target];

                        let rel = v0 - edge_points[e_target];
                        let vel = v_vel - edge_normals[e_target] * edge_weights[e_target];
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
                            &edge_points, &edge_normals, &edge_dirs, &edge_weights, e_pred, e_target, d,
                        ) {
                            Some(v) => v,
                            None => continue,
                        };
                        let seg_e = match offset_vertex(
                            &edge_points, &edge_normals, &edge_dirs, &edge_weights, e_target, e_succ, d,
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
            // --- Edge event(s): one or more edges collapse simultaneously ---
            let event_d = best_edge.unwrap().0;
            let eps = 1e-3;

            // Collect all edge events at this time, grouped by loop index.
            // Each entry is (wf_idx, event_point) for edges collapsing in that loop.
            let mut events_by_loop: std::collections::BTreeMap<usize, Vec<(usize, Vec2)>> =
                std::collections::BTreeMap::new();
            for &(d, pt, li, wf_idx) in &all_edge_events {
                if (d - event_d).abs() < eps {
                    events_by_loop.entry(li).or_default().push((wf_idx, pt));
                }
            }

            // Process each affected loop.
            // Work in reverse loop-index order so removals don't shift indices.
            let affected: Vec<usize> = events_by_loop.keys().copied().rev().collect();
            for li in affected {
                let evts = &events_by_loop[&li];
                let lp = &loops[li];
                let m = lp.len();

                // Record arcs from prev wavefront to event wavefront.
                let wf_at = wavefront_at_impl(
                    lp, event_d, &edge_points, &edge_normals, &edge_dirs, &edge_weights,
                );
                if let (Some(ref pw), Some(ref we)) = (&prev_wfs[li], &wf_at) {
                    if pw.len() == we.len() {
                        for i in 0..pw.len() {
                            if (pw[i] - we[i]).length() > 0.5 {
                                arcs.push((pw[i], we[i]));
                            }
                        }
                    }
                }

                // Record event nodes.
                for &(_, pt) in evts {
                    nodes.push(pt);
                }

                // Collect edge indices to remove (the edge at (wf_idx+1)%m for
                // each event). Use a set to avoid duplicates.
                let mut remove_set: Vec<usize> = evts
                    .iter()
                    .map(|&(wf_idx, _)| (wf_idx + 1) % m)
                    .collect();
                remove_set.sort_unstable();
                remove_set.dedup();

                let remaining = m - remove_set.len();

                if remaining < 3 {
                    // The loop collapses entirely (or to a degenerate < 3 edges).
                    // Draw medial axis arcs between distinct event points.
                    if let Some(ref we) = wf_at {
                        // Find unique wavefront vertices at the event time.
                        let mut unique_pts: Vec<Vec2> = Vec::new();
                        for &p in we {
                            if !unique_pts.iter().any(|u| (*u - p).length() < 1.0) {
                                unique_pts.push(p);
                            }
                        }
                        // Connect consecutive unique points (medial axis).
                        // Don't wrap around — these are open chains, not cycles.
                        for i in 0..unique_pts.len().saturating_sub(1) {
                            let j = i + 1;
                            if (unique_pts[i] - unique_pts[j]).length() > 1.0 {
                                arcs.push((unique_pts[i], unique_pts[j]));
                            }
                        }
                    }
                    // Also handle triangle-collapse-like final arcs.
                    if remaining <= 0 {
                        let fallback_wf = wf_at.as_ref().or(prev_wfs[li].as_ref());
                        if let Some(wf) = fallback_wf {
                            // For complete collapse, draw arcs from wavefront to centroid.
                            let centroid = Vec2::new(
                                wf.iter().map(|p| p.x).sum::<f64>() / wf.len() as f64,
                                wf.iter().map(|p| p.y).sum::<f64>() / wf.len() as f64,
                            );
                            for p in wf {
                                if (*p - centroid).length() > 0.5 {
                                    arcs.push((*p, centroid));
                                }
                            }
                        }
                    }
                    loops.remove(li);
                    prev_wfs.remove(li);
                } else {
                    // Remove collapsing edges in reverse order to preserve indices.
                    for &idx in remove_set.iter().rev() {
                        loops[li].remove(idx);
                    }

                    prev_wfs[li] = wavefront_at_impl(
                        &loops[li],
                        event_d,
                        &edge_points,
                        &edge_normals,
                        &edge_dirs,
                        &edge_weights,
                    )
                    .or_else(|| {
                        wavefront_at_impl(
                            &loops[li],
                            event_d + 0.01,
                            &edge_points,
                            &edge_normals,
                            &edge_dirs,
                            &edge_weights,
                        )
                    });
                }
            }

            current_d = event_d;
            event_timeline.push(SkeletonEvent {
                d: event_d,
                active_edges: loops.iter().flat_map(|l| l.iter().copied()).collect(),
                active_loops: loops.clone(),
            });
        } else {
            let (event_d, event_point, li_a, vi, li_b, ei) = best_split.unwrap();

            if li_a == li_b {
                // --- Intra-loop split: reflex vertex hits non-adjacent edge ---
                // The wavefront polygon splits into two independent regions.
                let li = li_a;

                // Split the loop into two independent sub-loops.
                let (loop_a, loop_b) = split_loop_edges(&loops[li], vi, ei);

                // Draw arcs per sub-loop. The full pre-split wavefront at
                // event_d is self-intersecting (that's what caused the split),
                // so we use each sub-loop's own wavefront which stays within
                // its valid region.
                let orig = &loops[li];
                let m = orig.len();
                for sub in [&loop_a, &loop_b] {
                    let sw = wavefront_at_impl(
                        sub, event_d, &edge_points, &edge_normals, &edge_dirs, &edge_weights,
                    );
                    if let (Some(ref pw), Some(ref sw)) = (&prev_wfs[li], &sw) {
                        let sl = sub.len();
                        for j in 0..sl {
                            // Find the corresponding prev_wf vertex: the one
                            // at the same edge junction in the original loop.
                            let e1 = sub[j];
                            let e2 = sub[(j + 1) % sl];
                            let prev_idx = (0..m).find(|&k| {
                                orig[k] == e1 && orig[(k + 1) % m] == e2
                            });
                            if let Some(k) = prev_idx {
                                if (pw[k] - sw[j]).length() > 0.5 {
                                    arcs.push((pw[k], sw[j]));
                                }
                            }
                        }
                    }
                }

                nodes.push(event_point);
                split_nodes.push(event_point);

                // Replace the original loop with loop_a.
                loops[li] = loop_a;
                prev_wfs[li] = wavefront_at_impl(
                    &loops[li], event_d, &edge_points, &edge_normals, &edge_dirs, &edge_weights,
                ).or_else(|| wavefront_at_impl(
                    &loops[li], event_d + 0.01, &edge_points, &edge_normals, &edge_dirs, &edge_weights,
                ));

                // Insert loop_b right after.
                let wf_b = wavefront_at_impl(
                    &loop_b, event_d, &edge_points, &edge_normals, &edge_dirs, &edge_weights,
                ).or_else(|| wavefront_at_impl(
                    &loop_b, event_d + 0.01, &edge_points, &edge_normals, &edge_dirs, &edge_weights,
                ));
                loops.insert(li + 1, loop_b);
                prev_wfs.insert(li + 1, wf_b);

                current_d = event_d;
            } else {
                // --- Inter-loop split: vertex from one loop hits edge of another ---
                // The two wavefronts meet at event_point. Continuing with a
                // merged single loop creates near-anti-parallel edges at the
                // bridge junctions, causing degenerate bisectors. Instead,
                // record arcs up to the split distance and terminate both loops.
                for &li in &[li_a, li_b] {
                    let wf = wavefront_at_impl(
                        &loops[li], event_d, &edge_points, &edge_normals, &edge_dirs, &edge_weights,
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
                split_nodes.push(event_point);

                // Remove both loops; keep any remaining loops for further
                // processing (e.g. if there are multiple holes).
                let (first, second) = if li_a > li_b { (li_a, li_b) } else { (li_b, li_a) };
                loops.remove(first);
                prev_wfs.remove(first);
                loops.remove(second);
                prev_wfs.remove(second);
                current_d = event_d;
            }

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
        split_nodes,
        events: event_timeline,
        edge_dirs,
        edge_normals,
        edge_points,
        edge_weights,
    }
}

fn empty_skeleton() -> StraightSkeleton {
    StraightSkeleton {
        arcs: vec![],
        nodes: vec![],
        split_nodes: vec![],
        events: vec![SkeletonEvent {
            d: 0.0,
            active_edges: vec![],
            active_loops: vec![],
        }],
        edge_dirs: vec![],
        edge_normals: vec![],
        edge_points: vec![],
        edge_weights: vec![],
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
            &skeleton.edge_weights,
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
        let total_edges = outer.len() + hole.len();

        // Compute multi-contour skeleton.
        let sk = compute_skeleton_multi(&shape, &vec![1.0; total_edges]);

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

    /// Check that weighted rectangle skeletons don't produce crossing arcs.
    /// This reproduces the "X pattern" bug where skeleton lines from opposite
    /// corners cross past each other instead of meeting at edge events.
    #[test]
    fn test_weighted_rectangle_no_crossing_arcs() {
        // Test several weight configurations for a tall rectangle.
        let w = 60.0;
        let h = 100.0;
        let polygon = rect(w, h);
        let n = polygon.len();

        let configs: Vec<(&str, Vec<f64>)> = vec![
            ("all 1.0", vec![1.0; n]),
            ("top/bottom aisle", vec![1.0, 0.0, 1.0, 0.0]),
            ("left/right aisle", vec![0.0, 1.0, 0.0, 1.0]),
            ("one-sided top", vec![0.0, 0.0, 1.0, 0.0]),
            ("one-sided bottom", vec![1.0, 0.0, 0.0, 0.0]),
        ];

        for (label, weights) in &configs {
            let sk = compute_skeleton_multi(&[polygon.clone()], weights);

            eprintln!("\n=== {} ({}x{}) ===", label, w, h);
            eprintln!("  arcs: {}", sk.arcs.len());
            for (i, &(a, b)) in sk.arcs.iter().enumerate() {
                eprintln!("    arc[{}]: ({:.1},{:.1}) -> ({:.1},{:.1})", i, a.x, a.y, b.x, b.y);
            }
            eprintln!("  nodes: {}", sk.nodes.len());
            for (i, n) in sk.nodes.iter().enumerate() {
                eprintln!("    node[{}]: ({:.1},{:.1})", i, n.x, n.y);
            }
            eprintln!("  events: {}", sk.events.len());
            for (i, ev) in sk.events.iter().enumerate() {
                eprintln!("    event[{}]: d={:.2}, active={:?}", i, ev.d, ev.active_edges);
            }

            // Check for crossing arcs: no two arcs should intersect
            // (other than at shared endpoints).
            let mut crossings = Vec::new();
            for i in 0..sk.arcs.len() {
                let (a1, a2) = sk.arcs[i];
                for j in (i + 1)..sk.arcs.len() {
                    let (b1, b2) = sk.arcs[j];
                    // Skip arcs that share an endpoint.
                    if (a1 - b1).length() < 1e-3
                        || (a1 - b2).length() < 1e-3
                        || (a2 - b1).length() < 1e-3
                        || (a2 - b2).length() < 1e-3
                    {
                        continue;
                    }
                    let d1 = a2 - a1;
                    let d2 = b2 - b1;
                    let denom = d1.cross(d2);
                    if denom.abs() < 1e-10 {
                        continue;
                    }
                    let diff = b1 - a1;
                    let t = diff.cross(d2) / denom;
                    let u = diff.cross(d1) / denom;
                    if t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99 {
                        crossings.push((i, j, t, u));
                    }
                }
            }
            if !crossings.is_empty() {
                for &(i, j, t, u) in &crossings {
                    let (a1, a2) = sk.arcs[i];
                    let (b1, b2) = sk.arcs[j];
                    eprintln!(
                        "  CROSSING: arc[{}] ({:.1},{:.1})->({:.1},{:.1}) x arc[{}] ({:.1},{:.1})->({:.1},{:.1}) at t={:.3}, u={:.3}",
                        i, a1.x, a1.y, a2.x, a2.y,
                        j, b1.x, b1.y, b2.x, b2.y,
                        t, u
                    );
                }
            }
            assert!(
                crossings.is_empty(),
                "[{}] {} crossing arc pairs found",
                label,
                crossings.len()
            );
        }
    }

    /// Test with actual face geometry from layout (trapezoids from angled aisles).
    #[test]
    fn test_actual_face_geometry_skeleton() {
        // Face 1 from test_attempt_debug: a trapezoid between two aisles.
        // Original (CW): (42,46.7), (42,158), (123,158), (123,55.4)
        // After CCW normalization: reversed.
        let face1 = vec![
            Vec2::new(123.0, 55.4),
            Vec2::new(123.0, 158.0),
            Vec2::new(42.0, 158.0),
            Vec2::new(42.0, 46.7),
        ];
        assert!(signed_area(&face1) > 0.0, "face1 must be CCW");

        // Face 2: narrower trapezoid.
        let face2 = vec![
            Vec2::new(183.0, 61.9),
            Vec2::new(183.0, 158.0),
            Vec2::new(147.0, 158.0),
            Vec2::new(147.0, 58.0),
        ];
        assert!(signed_area(&face2) > 0.0, "face2 must be CCW");

        let configs: Vec<(&str, Vec<Vec2>, Vec<f64>)> = vec![
            ("face1 all-1", face1.clone(), vec![1.0; 4]),
            ("face1 LR-aisle", face1.clone(), vec![0.0, 1.0, 0.0, 1.0]),
            ("face1 TB-aisle", face1.clone(), vec![1.0, 0.0, 1.0, 0.0]),
            ("face2 all-1", face2.clone(), vec![1.0; 4]),
            ("face2 LR-aisle", face2.clone(), vec![0.0, 1.0, 0.0, 1.0]),
        ];

        for (label, polygon, weights) in &configs {
            let sk = compute_skeleton_multi(&[polygon.clone()], weights);

            eprintln!("\n=== {} ===", label);
            for (i, v) in polygon.iter().enumerate() {
                eprintln!("  v{}: ({:.1},{:.1})", i, v.x, v.y);
            }
            eprintln!("  weights: {:?}", weights);
            eprintln!("  arcs: {}", sk.arcs.len());
            for (i, &(a, b)) in sk.arcs.iter().enumerate() {
                eprintln!("    arc[{}]: ({:.1},{:.1}) -> ({:.1},{:.1})", i, a.x, a.y, b.x, b.y);
            }
            eprintln!("  nodes: {}", sk.nodes.len());
            for (i, n) in sk.nodes.iter().enumerate() {
                eprintln!("    node[{}]: ({:.1},{:.1})", i, n.x, n.y);
            }

            // Check for crossing arcs.
            let mut crossings = Vec::new();
            for i in 0..sk.arcs.len() {
                let (a1, a2) = sk.arcs[i];
                for j in (i + 1)..sk.arcs.len() {
                    let (b1, b2) = sk.arcs[j];
                    if (a1 - b1).length() < 1e-3
                        || (a1 - b2).length() < 1e-3
                        || (a2 - b1).length() < 1e-3
                        || (a2 - b2).length() < 1e-3
                    {
                        continue;
                    }
                    let d1 = a2 - a1;
                    let d2 = b2 - b1;
                    let denom = d1.cross(d2);
                    if denom.abs() < 1e-10 {
                        continue;
                    }
                    let diff = b1 - a1;
                    let t = diff.cross(d2) / denom;
                    let u = diff.cross(d1) / denom;
                    if t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99 {
                        crossings.push((i, j, t, u));
                    }
                }
            }
            if !crossings.is_empty() {
                for &(i, j, t, u) in &crossings {
                    let (a1, a2) = sk.arcs[i];
                    let (b1, b2) = sk.arcs[j];
                    eprintln!(
                        "  CROSSING: arc[{}] ({:.1},{:.1})->({:.1},{:.1}) x arc[{}] ({:.1},{:.1})->({:.1},{:.1}) t={:.3} u={:.3}",
                        i, a1.x, a1.y, a2.x, a2.y,
                        j, b1.x, b1.y, b2.x, b2.y,
                        t, u
                    );
                }
                panic!("[{}] {} crossing arc pairs found", label, crossings.len());
            }
        }
    }

    /// Test with a slightly perturbed rectangle (simulating boolean overlay output).
    #[test]
    fn test_perturbed_rectangle_skeleton() {
        // Simulate a face polygon from boolean overlay — vertices slightly off
        // from a perfect rectangle.
        let configs: Vec<(&str, Vec<Vec2>, Vec<f64>)> = vec![
            (
                "perturbed rect, all 1.0",
                vec![
                    Vec2::new(0.01, -0.005),
                    Vec2::new(59.99, 0.003),
                    Vec2::new(60.01, 99.997),
                    Vec2::new(-0.01, 100.005),
                ],
                vec![1.0, 1.0, 1.0, 1.0],
            ),
            (
                "perturbed rect, top/bottom aisle",
                vec![
                    Vec2::new(0.01, -0.005),
                    Vec2::new(59.99, 0.003),
                    Vec2::new(60.01, 99.997),
                    Vec2::new(-0.01, 100.005),
                ],
                vec![1.0, 0.0, 1.0, 0.0],
            ),
            (
                "perturbed rect, left/right aisle",
                vec![
                    Vec2::new(0.01, -0.005),
                    Vec2::new(59.99, 0.003),
                    Vec2::new(60.01, 99.997),
                    Vec2::new(-0.01, 100.005),
                ],
                vec![0.0, 1.0, 0.0, 1.0],
            ),
        ];

        for (label, polygon, weights) in &configs {
            let ccw = ensure_ccw(polygon);
            let sk = compute_skeleton_multi(&[ccw], weights);

            eprintln!("\n=== {} ===", label);
            eprintln!("  arcs: {}", sk.arcs.len());
            for (i, &(a, b)) in sk.arcs.iter().enumerate() {
                eprintln!("    arc[{}]: ({:.3},{:.3}) -> ({:.3},{:.3})", i, a.x, a.y, b.x, b.y);
            }
            eprintln!("  nodes: {}", sk.nodes.len());
            for (i, n) in sk.nodes.iter().enumerate() {
                eprintln!("    node[{}]: ({:.3},{:.3})", i, n.x, n.y);
            }

            // Check for crossing arcs.
            let mut crossings = Vec::new();
            for i in 0..sk.arcs.len() {
                let (a1, a2) = sk.arcs[i];
                for j in (i + 1)..sk.arcs.len() {
                    let (b1, b2) = sk.arcs[j];
                    if (a1 - b1).length() < 1e-3
                        || (a1 - b2).length() < 1e-3
                        || (a2 - b1).length() < 1e-3
                        || (a2 - b2).length() < 1e-3
                    {
                        continue;
                    }
                    let d1 = a2 - a1;
                    let d2 = b2 - b1;
                    let denom = d1.cross(d2);
                    if denom.abs() < 1e-10 {
                        continue;
                    }
                    let diff = b1 - a1;
                    let t = diff.cross(d2) / denom;
                    let u = diff.cross(d1) / denom;
                    if t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99 {
                        crossings.push((i, j, t, u));
                    }
                }
            }
            if !crossings.is_empty() {
                for &(i, j, t, u) in &crossings {
                    let (a1, a2) = sk.arcs[i];
                    let (b1, b2) = sk.arcs[j];
                    eprintln!(
                        "  CROSSING: arc[{}] ({:.1},{:.1})->({:.1},{:.1}) x arc[{}] ({:.1},{:.1})->({:.1},{:.1}) t={:.3} u={:.3}",
                        i, a1.x, a1.y, a2.x, a2.y,
                        j, b1.x, b1.y, b2.x, b2.y,
                        t, u
                    );
                }
            }
            assert!(
                crossings.is_empty(),
                "[{}] {} crossing arc pairs found",
                label,
                crossings.len()
            );
        }
    }
}
