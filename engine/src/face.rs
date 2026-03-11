use crate::inset::signed_area;
use crate::segment::fill_strip;
use crate::skeleton;
use crate::types::*;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

// ---------------------------------------------------------------------------
// Corridor polygons
// ---------------------------------------------------------------------------

/// Build corridor rectangle for a single aisle edge.
pub(crate) fn corridor_polygon(vertices: &[Vec2], edge: &AisleEdge) -> Vec<Vec2> {
    let start = vertices[edge.start];
    let end = vertices[edge.end];
    let dir = (end - start).normalize();
    let normal = Vec2::new(-dir.y, dir.x);
    let w = edge.width;
    vec![
        start + normal * w,
        end + normal * w,
        end - normal * w,
        start - normal * w,
    ]
}

// ---------------------------------------------------------------------------
// Face extraction via boolean overlay
// ---------------------------------------------------------------------------

/// Compute signed area of a path in [f64; 2] form.
fn signed_area_f64(path: &[[f64; 2]]) -> f64 {
    let n = path.len();
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += path[i][0] * path[j][1] - path[j][0] * path[i][1];
    }
    area * 0.5
}

/// Normalize a path to CCW winding (positive signed area).
fn ensure_ccw(path: &mut Vec<[f64; 2]>) {
    if signed_area_f64(path) < 0.0 {
        path.reverse();
    }
}

/// Extract positive-space face polygons by subtracting corridors and holes
/// from the boundary. Returns Vec<Shape> where each shape is Vec<Vec<Vec2>>
/// with shape[0] = outer contour, shape[1..] = holes within that face.
///
/// Done in two steps so that corridor contours (which have consistent
/// relative winding from the union output) never share a clip set with
/// boundary hole contours (which have user-determined winding). Mixing
/// them in a single NonZero operation causes winding interference.
fn extract_faces(
    boundary: &Polygon,
    merged_corridors: &[Vec<Vec<Vec2>>],
) -> Vec<Vec<Vec<Vec2>>> {
    let to_path = |pts: &[Vec2]| -> Vec<[f64; 2]> {
        pts.iter().map(|v| [v.x, v.y]).collect()
    };
    let to_vec2 = |shape: Vec<Vec<[f64; 2]>>| -> Vec<Vec<Vec2>> {
        shape
            .into_iter()
            .map(|contour| contour.into_iter().map(|p| Vec2::new(p[0], p[1])).collect())
            .collect()
    };

    // Step 1: boundary MINUS corridors. Corridor outers and holes have
    // consistent relative winding from the boolean union output, so
    // NonZero handles them correctly in isolation.
    let subj = to_path(&boundary.outer);
    let mut corridor_paths: Vec<Vec<[f64; 2]>> = Vec::new();
    for shape in merged_corridors {
        for contour in shape {
            corridor_paths.push(to_path(contour));
        }
    }
    let after_corridors = subj.overlay(&corridor_paths, OverlayRule::Difference, FillRule::NonZero);

    // Step 2: subtract boundary holes. Kept separate so that boundary
    // hole winding can't interfere with corridor hole winding.
    if boundary.holes.is_empty() {
        return after_corridors.into_iter().map(to_vec2).collect();
    }

    let mut hole_paths: Vec<Vec<[f64; 2]>> = Vec::new();
    for hole in &boundary.holes {
        let mut path = to_path(hole);
        ensure_ccw(&mut path);
        hole_paths.push(path);
    }

    // Feed step 1 output as subject paths for the second subtraction.
    let subj2: Vec<Vec<[f64; 2]>> = after_corridors
        .into_iter()
        .flat_map(|shape| shape.into_iter())
        .collect();
    let result = subj2.overlay(&hole_paths, OverlayRule::Difference, FillRule::NonZero);

    result.into_iter().map(to_vec2).collect()
}

// ---------------------------------------------------------------------------
// Straight skeleton spine generation
// ---------------------------------------------------------------------------

/// Test if point `p` is inside `polygon` using the winding-number rule.
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

/// Test if point `p` is inside a face shape (inside outer contour, outside
/// all hole contours).
fn point_in_face(p: Vec2, shape: &[Vec<Vec2>]) -> bool {
    if shape.is_empty() {
        return false;
    }
    if !point_in_polygon(p, &shape[0]) {
        return false;
    }
    for hole in &shape[1..] {
        if point_in_polygon(p, hole) {
            return false;
        }
    }
    true
}

/// Clip line segment `a→b` to the interior of a face shape (outer contour
/// minus holes).  Returns parameter intervals `[(t0,t1), …]` where
/// `a + t*(b-a)` is inside.
fn clip_segment_to_face(a: Vec2, b: Vec2, shape: &[Vec<Vec2>]) -> Vec<(f64, f64)> {
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

/// Shortest distance from point `p` to line segment `a→b`.
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

/// Classify each edge of `contour` as aisle-facing (true) or not.
/// An edge is aisle-facing if its midpoint lies on a corridor polygon
/// edge — meaning it was carved by a corridor during the boolean
/// difference. Uses a tight floating-point epsilon since face edges
/// are exact sub-segments of corridor edges after the boolean overlay.
fn classify_face_edges(contour: &[Vec2], corridor_shapes: &[Vec<Vec<Vec2>>], classify: bool) -> Vec<bool> {
    if !classify {
        return vec![true; contour.len()];
    }
    let eps = 0.1;
    let n = contour.len();
    let mut aisle_facing = vec![false; n];
    for i in 0..n {
        let j = (i + 1) % n;
        let mid = (contour[i] + contour[j]) * 0.5;

        'outer: for shape in corridor_shapes {
            for ring in shape {
                for ci in 0..ring.len() {
                    let cj = (ci + 1) % ring.len();
                    if point_to_segment_dist(mid, ring[ci], ring[cj]) < eps {
                        aisle_facing[i] = true;
                        break 'outer;
                    }
                }
            }
        }
    }
    aisle_facing
}

/// Generate spine segments for a face shape (outer contour + holes).
///
/// For each aisle-facing edge, shift it inward by `effective_depth` to
/// produce an inset line. Adjacent inset lines meet at miter vertices.
/// The segment from one miter vertex to the next, clipped to the face
/// interior, is the spine for that edge. Boundary/hole edges stay put
/// (distance = 0) and don't produce spines.
/// Compute spines for a face using a straight-skeleton-style weighted inset.
///
/// Each aisle-facing edge is offset inward by `effective_depth`; boundary
/// edges stay put (distance 0). The inset polygon is computed with proper
/// edge-collapse handling: when an edge shrinks to zero, only that edge is
/// removed (not both neighbors), preserving adjacent spines. The surviving
/// edges that map to aisle-facing original edges become spines.
///
/// This partitions the face interior among its edges — two opposing aisle
/// edges each get half the face width, and edges at different angles get
/// non-overlapping regions bounded by the skeleton ridges (miter points).
fn compute_face_spines(
    shape: &[Vec<Vec2>],
    effective_depth: f64,
    corridor_shapes: &[Vec<Vec<Vec2>>],
    debug: &DebugToggles,
) -> Vec<SpineSegment> {
    if shape.is_empty() || shape[0].len() < 3 {
        return vec![];
    }

    // Skip faces that are too narrow to hold a stall strip. Approximate
    // minimum width as 4 * area / perimeter; faces narrower than the
    // effective stall depth are corner artifacts from miter fills.
    let outer = &shape[0];
    let area = signed_area(outer).abs();
    let perimeter: f64 = outer.iter().enumerate().map(|(i, v)| {
        let next = &outer[(i + 1) % outer.len()];
        (*next - *v).length()
    }).sum();
    if perimeter > 0.0 && 4.0 * area / perimeter < effective_depth {
        return vec![];
    }

    let mut all_spines = Vec::new();

    for (ci, contour) in shape.iter().enumerate() {
        let n = contour.len();
        if n < 3 {
            continue;
        }

        // Normalize to CCW so edge indices match the skeleton's internal
        // ensure_ccw convention.  All downstream arrays (aisle_facing,
        // distances, normals) are built from this CCW contour.
        let contour = if signed_area(contour) < 0.0 {
            contour.iter().rev().copied().collect::<Vec<_>>()
        } else {
            contour.clone()
        };

        let aisle_facing = classify_face_edges(&contour, corridor_shapes, debug.edge_classification);
        let ed = effective_depth - 0.05;
        let min_edge_len = effective_depth * 2.0;
        let is_hole = ci > 0;

        // Per-edge inset distances: ed for aisle-facing edges, 0 for boundary
        // edges. For hole contours (corridor cutouts in boundary faces), all
        // aisle-facing edges get uniform ed regardless of length; zeroing
        // short connector edges between corridor segments creates
        // mixed-distance transitions that blow up the miter computation.
        let distances: Vec<f64> = (0..n)
            .map(|i| {
                if !aisle_facing[i] {
                    return 0.0;
                }
                if debug.short_edge_zeroing && !is_hole {
                    let j = (i + 1) % n;
                    let edge_len = (contour[j] - contour[i]).length();
                    if edge_len < min_edge_len {
                        return 0.0;
                    }
                }
                ed
            })
            .collect();

        let spines = skeleton_inset_spines(&contour, &distances, &aisle_facing, 1.0, shape, debug.spine_clipping);
        all_spines.extend(spines);
    }

    all_spines
}

/// Event-driven straight skeleton inset to produce spine segments.
///
/// Computes the full straight skeleton for the polygon, then extracts the
/// wavefront at the effective stall depth. Each surviving aisle-facing edge
/// of the offset polygon becomes a spine, clipped to the face interior.
fn skeleton_inset_spines(
    polygon: &[Vec2],
    distances: &[f64],
    aisle_facing: &[bool],
    sign: f64,
    face_shape: &[Vec<Vec2>],
    clip: bool,
) -> Vec<SpineSegment> {
    let n = polygon.len();
    if n < 3 {
        return vec![];
    }

    // Determine the effective inset depth from the distances array.
    // All aisle-facing edges use the same depth; pick the first nonzero one.
    let effective_depth = distances
        .iter()
        .copied()
        .find(|&d| d > 0.0)
        .unwrap_or(0.0);
    if effective_depth <= 0.0 {
        return vec![];
    }

    // Compute the straight skeleton on the polygon.
    let sk = skeleton::compute_skeleton(polygon);

    // Get the offset polygon at the effective depth.
    let (offset_pts, active_edges) = match skeleton::wavefront_at(&sk, effective_depth) {
        Some(result) => result,
        None => return vec![],
    };

    // Precompute inward normals for the original polygon edges (same as
    // before, using the caller's sign convention).
    let mut normals: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let e = polygon[j] - polygon[i];
        let len = e.length();
        if len < 1e-12 {
            normals.push(Vec2::new(0.0, 0.0));
        } else {
            normals.push(Vec2::new(-e.y * sign / len, e.x * sign / len));
        }
    }

    // Each vertex i of the offset polygon sits between active_edges[i] and
    // active_edges[(i+1) % m]. The offset edge from vertex i to vertex i+1
    // corresponds to original edge active_edges[(i+1) % m].
    //
    // Wait — let's be precise about the mapping. The wavefront has m vertices.
    // Vertex i is the intersection of offset lines for active_edges[i] and
    // active_edges[(i+1) % m]. So the edge from vertex i to vertex (i+1) is
    // along the offset of original edge active_edges[(i+1) % m].
    //
    // Actually, looking at wavefront_at_impl: vertex i = intersection of
    // active_edges[i] and active_edges[(i+1) % m]. So the segment from
    // vertex i to vertex (i+1) % m runs along the offset line of
    // active_edges[(i+1) % m].
    let m = active_edges.len();
    let mut spines = Vec::new();

    for idx in 0..m {
        let next_idx = (idx + 1) % m;
        let orig_edge = active_edges[next_idx];

        // Only produce spines for aisle-facing edges with nonzero distance.
        if orig_edge >= n || !aisle_facing[orig_edge] || distances[orig_edge] == 0.0 {
            continue;
        }

        let spine_start = offset_pts[idx];
        let spine_end = offset_pts[next_idx];
        let outward = Vec2::new(-normals[orig_edge].x, -normals[orig_edge].y);

        if !clip {
            if (spine_end - spine_start).length() > 1.0 {
                spines.push(SpineSegment {
                    start: spine_start,
                    end: spine_end,
                    outward_normal: outward,
                });
            }
            continue;
        }

        // Clip both the spine and the "stall reach" line to the face interior.
        let spine_clips = clip_segment_to_face(spine_start, spine_end, face_shape);
        let reach = (distances[orig_edge] - 0.5).max(0.0);
        let offset_start = spine_start + outward * reach;
        let offset_end = spine_end + outward * reach;
        let offset_clips = clip_segment_to_face(offset_start, offset_end, face_shape);

        // Intersect the two sets of t-parameter intervals.
        for (st0, st1) in &spine_clips {
            for (ot0, ot1) in &offset_clips {
                let t0 = st0.max(*ot0);
                let t1 = st1.min(*ot1);
                if t1 - t0 < 1e-9 {
                    continue;
                }
                let d = spine_end - spine_start;
                let s = spine_start + d * t0;
                let e = spine_start + d * t1;
                if (e - s).length() > 1.0 {
                    spines.push(SpineSegment {
                        start: s,
                        end: e,
                        outward_normal: outward,
                    });
                }
            }
        }
    }

    spines
}

// ---------------------------------------------------------------------------
// Corridor merging
// ---------------------------------------------------------------------------

/// Build deduplicated corridor polygons (one rectangle per undirected edge).
fn deduplicate_corridors(graph: &DriveAisleGraph) -> Vec<Vec<Vec2>> {
    let mut seen = std::collections::HashSet::new();
    let mut corridors = Vec::new();
    for edge in &graph.edges {
        let key = if edge.start < edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        if !seen.insert(key) {
            continue;
        }
        corridors.push(corridor_polygon(&graph.vertices, edge));
    }
    corridors
}

/// Generate miter-fill polygons at graph vertices where edges meet.
/// At each vertex with 2+ edges, compute the convex hull of all corridor
/// corner points (left and right sides of each edge at the vertex). This
/// covers all gaps between corridor rectangles in one polygon, regardless
/// of how many edges meet or their angles.
fn generate_miter_fills(graph: &DriveAisleGraph, debug: &DebugToggles) -> Vec<Vec<Vec2>> {
    if !debug.miter_fills {
        return vec![];
    }
    let mut fills = Vec::new();
    let mut seen_edges: std::collections::HashSet<(usize, usize)> =
        std::collections::HashSet::new();

    let nv = graph.vertices.len();
    let mut adj: Vec<Vec<(Vec2, f64)>> = vec![vec![]; nv];

    for edge in &graph.edges {
        let key = if edge.start < edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        if !seen_edges.insert(key) {
            continue;
        }
        let s = graph.vertices[edge.start];
        let e = graph.vertices[edge.end];
        let dir = (e - s).normalize();
        let w = edge.width;

        adj[edge.start].push((dir, w));
        adj[edge.end].push((Vec2::new(-dir.x, -dir.y), w));
    }

    for vi in 0..nv {
        if adj[vi].len() < 2 {
            continue;
        }

        let v = graph.vertices[vi];

        // Sort edges by outgoing angle from the vertex.
        let mut edges_sorted: Vec<(f64, Vec2, f64)> = adj[vi]
            .iter()
            .map(|(d, w)| (d.y.atan2(d.x), *d, *w))
            .collect();
        edges_sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // For each consecutive pair of edges (sorted by angle), create a
        // miter fill wedge. The wedge connects the left side of edge i to
        // the right side of edge j via their miter intersection point.
        let ne = edges_sorted.len();
        for i in 0..ne {
            let j = (i + 1) % ne;
            let (a1, d1, w1) = edges_sorted[i];
            let (a2, d2, w2) = edges_sorted[j];

            let n1 = Vec2::new(-d1.y, d1.x);
            let n2 = Vec2::new(-d2.y, d2.x);

            let p1 = v + n1 * w1;  // left side of edge i
            let p2 = v - n2 * w2;  // right side of edge j

            let denom = d1.cross(d2);
            if denom.abs() < 1e-12 {
                continue;
            }

            let t = (p2 - p1).cross(d2) / denom;
            let miter = p1 + d1 * t;

            // Angular gap from edge i to edge j (going CCW).
            let gap = if j > i {
                a2 - a1
            } else {
                (a2 + std::f64::consts::TAU) - a1
            };

            // For outer/convex gaps (< 180°), cap the miter distance to
            // prevent spikes from acute corners extending across the lot.
            // Inner/concave gaps point inward between corridors and are
            // naturally bounded by neighboring geometry.
            if gap < std::f64::consts::PI {
                let max_w = w1.max(w2);
                if (miter - v).length() > max_w * 4.0 {
                    continue;
                }
            }

            let wedge = vec![v, p1, miter, p2];
            if signed_area(&wedge).abs() < 1.0 {
                continue;
            }
            fills.push(wedge);
        }
    }

    fills
}
/// Boolean-union all corridor rectangles + miter fills into merged
/// shapes. Each shape is Vec<Vec<Vec2>> where [0] = outer contour and
/// [1..] = hole contours (for corridors that form loops enclosing faces).
fn merge_corridor_shapes(
    corridors: &[Vec<Vec2>],
    graph: &DriveAisleGraph,
    debug: &DebugToggles,
) -> Vec<Vec<Vec<Vec2>>> {
    if corridors.is_empty() {
        return vec![];
    }

    let miter_fills = generate_miter_fills(graph, debug);

    // All corridor rects + miter fills as subject paths in a single
    // union operation. Normalize all polygons to CCW winding (positive
    // signed area) so the NonZero fill rule unions them correctly —
    // mixed winding causes overlapping regions to cancel instead of merge.
    let subj: Vec<Vec<[f64; 2]>> = corridors
        .iter()
        .chain(miter_fills.iter())
        .map(|c| {
            let mut pts = c.clone();
            if signed_area(&pts) < 0.0 {
                pts.reverse();
            }
            pts.iter().map(|v| [v.x, v.y]).collect()
        })
        .collect();

    if subj.is_empty() {
        return vec![];
    }

    // Self-union: union all subject paths with an empty clip set.
    // NonZero fill rule merges all overlapping/touching paths into
    // a single outline, preserving holes where corridors form loops.
    let empty_clip: Vec<Vec<[f64; 2]>> = vec![];
    let result = subj.overlay(&empty_clip, OverlayRule::Union, FillRule::NonZero);

    // Compute minimum hole area threshold: holes in the merged corridor
    // smaller than max_width² are junction artifacts, not real face
    // regions. Real face holes (from corridor loops) are much larger.
    let max_width = graph.edges.iter().map(|e| e.width).fold(0.0f64, f64::max);
    let min_hole_area = max_width * max_width;

    // Convert to Vec2, preserving outer contours and large holes.
    // Clean up degenerate spikes and filter out small junction-artifact
    // holes that would otherwise become sliver faces.
    result
        .into_iter()
        .filter(|shape| !shape.is_empty())
        .map(|shape| {
            shape
                .into_iter()
                .enumerate()
                .filter_map(|(i, contour)| {
                    let pts: Vec<Vec2> = contour.into_iter().map(|p| Vec2::new(p[0], p[1])).collect();
                    let pts = if debug.spike_removal {
                        remove_contour_spikes(pts, 2.0)
                    } else {
                        pts
                    };
                    // Keep outer contour (index 0) unconditionally.
                    // Filter hole contours (index 1+) by area.
                    if i > 0 && debug.hole_filtering && signed_area(&pts).abs() < min_hole_area {
                        return None;
                    }
                    Some(pts)
                })
                .collect()
        })
        .collect()
}

/// Remove degenerate spikes from a polygon contour. A spike is a
/// vertex where consecutive edges are anti-parallel (the path doubles
/// back on itself), creating a zero-area protrusion. This happens when
/// boolean union merges thin miter fill wedges with corridor rectangles.
fn remove_contour_spikes(mut contour: Vec<Vec2>, tolerance: f64) -> Vec<Vec2> {
    let mut changed = true;
    while changed {
        changed = false;
        let n = contour.len();
        if n < 4 {
            break;
        }
        for i in 0..n {
            let prev = if i == 0 { n - 1 } else { i - 1 };
            let next = (i + 1) % n;
            let a = contour[prev];
            let b = contour[i];
            let c = contour[next];
            let ab = b - a;
            let bc = c - b;
            let ab_len = ab.length();
            let bc_len = bc.length();
            if ab_len < tolerance || bc_len < tolerance {
                // Degenerate near-zero-length edge; remove the vertex.
                contour.remove(i);
                changed = true;
                break;
            }
            let ab_n = ab * (1.0 / ab_len);
            let bc_n = bc * (1.0 / bc_len);
            // Anti-parallel: dot product ≈ -1.
            if ab_n.dot(bc_n) < -0.95 {
                // Spike at vertex i: the path goes A→B then reverses B→C.
                // Also check if C is close to A (the path doubles back to
                // where it started).
                if (c - a).length() < tolerance {
                    // Remove both the spike tip (i) and the return vertex
                    // (next), collapsing A→B→C→D into A→D.
                    let remove_second = next;
                    if remove_second > i {
                        contour.remove(remove_second);
                        contour.remove(i);
                    } else {
                        contour.remove(i);
                        contour.remove(remove_second);
                    }
                    changed = true;
                    break;
                }
            }
        }
    }
    contour
}

// ---------------------------------------------------------------------------
// Spine merging
// ---------------------------------------------------------------------------

/// Merge collinear spine segments that share an endpoint (within tolerance)
/// and have the same outward normal direction. This eliminates gaps between
/// stall strips that span multiple faces along the same aisle edge.
fn merge_collinear_spines(spines: Vec<SpineSegment>, tolerance: f64) -> Vec<SpineSegment> {
    if spines.is_empty() {
        return spines;
    }

    let mut merged = spines;
    let mut changed = true;

    while changed {
        changed = false;
        let mut result: Vec<SpineSegment> = Vec::new();
        let mut used = vec![false; merged.len()];

        for i in 0..merged.len() {
            if used[i] {
                continue;
            }

            let mut seg = merged[i].clone();
            used[i] = true;

            // Repeatedly try to extend this segment by merging with others.
            let mut extended = true;
            while extended {
                extended = false;
                for j in 0..merged.len() {
                    if used[j] {
                        continue;
                    }

                    if let Some(m) = try_merge_spines(&seg, &merged[j], tolerance) {
                        seg = m;
                        used[j] = true;
                        extended = true;
                        changed = true;
                    }
                }
            }

            result.push(seg);
        }

        merged = result;
    }

    merged
}

/// Try to merge two spine segments. Returns the merged segment if they are
/// collinear (directions within tolerance), have similar outward normals,
/// and share an endpoint.
fn try_merge_spines(a: &SpineSegment, b: &SpineSegment, tolerance: f64) -> Option<SpineSegment> {
    let dir_a = (a.end - a.start).normalize();
    let dir_b = (b.end - b.start).normalize();

    // Must be collinear: directions parallel (dot ≈ ±1).
    if dir_a.dot(dir_b).abs() < 0.99 {
        return None;
    }

    // Must have (nearly) identical outward normals. Spines from the same
    // corridor edge have exactly equal normals; spines from different edges
    // at the same boundary corner differ by the angle between the edges.
    // A tight threshold (0.9999 ≈ 0.8°) prevents cross-edge merging that
    // would extend one edge's normal over another edge's territory.
    if a.outward_normal.dot(b.outward_normal) < 0.9999 {
        return None;
    }

    // Must share an endpoint (within tolerance).
    let pairs = [
        (a.end, b.start),  // a→b chain
        (a.start, b.end),  // b→a chain
        (a.end, b.end),    // both point away from junction
        (a.start, b.start), // both point toward junction
    ];

    for (p1, p2) in &pairs {
        if (*p1 - *p2).length() > tolerance {
            continue;
        }

        // Project all four endpoints onto the shared line direction
        // and take the full extent.
        let origin = a.start;
        let dots = [
            dir_a.dot(a.start - origin),
            dir_a.dot(a.end - origin),
            dir_a.dot(b.start - origin),
            dir_a.dot(b.end - origin),
        ];
        let min_t = dots.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_t = dots.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        return Some(SpineSegment {
            start: origin + dir_a * min_t,
            end: origin + dir_a * max_t,
            outward_normal: a.outward_normal,
        });
    }

    None
}

// ---------------------------------------------------------------------------
// Top-level orchestration
// ---------------------------------------------------------------------------

/// Remove stalls where any shrunk corner falls outside all face shapes.
/// This catches stalls in corner regions that protrude past the face
/// boundary into the corridor.
fn clip_stalls_to_faces(stalls: Vec<StallQuad>, faces: &[Vec<Vec<Vec2>>]) -> Vec<StallQuad> {
    stalls
        .into_iter()
        .filter(|stall| {
            // Shrink 0.5ft toward centroid for tolerance.
            let cx = stall.corners.iter().map(|c| c.x).sum::<f64>() / 4.0;
            let cy = stall.corners.iter().map(|c| c.y).sum::<f64>() / 4.0;
            let centroid = Vec2::new(cx, cy);
            let shrunk: Vec<Vec2> = stall.corners.iter().map(|c| {
                let d = *c - centroid;
                let len = d.length();
                if len < 1e-12 { *c } else { *c - d * (0.5 / len) }
            }).collect();
            // Every shrunk corner must be inside at least one face.
            shrunk.iter().all(|corner| {
                faces.iter().any(|shape| point_in_face(*corner, shape))
            })
        })
        .collect()
}


/// Place stalls on a set of spine segments, returning the stall quads.
fn place_stalls_on_spines(spines: &[SpineSegment], params: &ParkingParams) -> Vec<StallQuad> {
    let mut stalls = Vec::new();
    for seg in spines {
        let edge_dir = (seg.end - seg.start).normalize();
        let left_normal = Vec2::new(-edge_dir.y, edge_dir.x);
        let dot = left_normal.dot(seg.outward_normal);

        // Orient so that fill_strip's side=+1 places stalls toward
        // the outward_normal direction.
        let (start, end) = if dot > 0.0 {
            (seg.start, seg.end)
        } else {
            (seg.end, seg.start)
        };

        stalls.extend(fill_strip(start, end, 1.0, 0.0, params));
    }
    stalls
}

/// Compute endcap islands for a face by subtracting placed stalls from
/// the face polygon. The remaining fragments (filtered by area) are the
/// leftover regions at strip ends, corners, and narrow zones.
fn endcap_islands_for_face(
    shape: &[Vec<Vec2>],
    stalls: &[StallQuad],
    min_area: f64,
) -> Vec<Island> {
    if shape.is_empty() || shape[0].is_empty() || stalls.is_empty() {
        return vec![];
    }

    // Subject: full face shape (outer contour + hole contours).
    let subj: Vec<Vec<[f64; 2]>> = shape
        .iter()
        .map(|contour| contour.iter().map(|v| [v.x, v.y]).collect())
        .collect();

    // Clips: each stall quad.
    let clips: Vec<Vec<[f64; 2]>> = stalls
        .iter()
        .map(|s| s.corners.iter().map(|v| [v.x, v.y]).collect())
        .collect();

    let result = subj.overlay(&clips, OverlayRule::Difference, FillRule::EvenOdd);

    let mut islands = Vec::new();
    for fragment in result {
        // Only keep simple-polygon fragments (no holes). Multi-contour
        // results are structural margins (e.g., boundary-to-stall gaps),
        // not endcap islands.
        if fragment.len() != 1 {
            continue;
        }
        let polygon: Vec<Vec2> = fragment[0]
            .iter()
            .map(|p| Vec2::new(p[0], p[1]))
            .collect();
        if signed_area(&polygon).abs() >= min_area {
            islands.push(Island {
                polygon,
                kind: IslandKind::EndCap,
            });
        }
    }
    islands
}

/// Deduplicate overlapping collinear spines. When two spines have the same
/// direction and outward normal and overlap along their length, trim the
/// shorter one to only its non-overlapping portion. This prevents
/// miter-fill spines from extending into regions already covered by the
/// main corridor spine, while preserving the unique tail of each spine.
fn dedup_overlapping_spines(spines: Vec<SpineSegment>, tolerance: f64) -> Vec<SpineSegment> {
    let mut result = spines;

    // For each pair of collinear, overlapping spines: trim the shorter one.
    let mut changed = true;
    while changed {
        changed = false;
        let n = result.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let dir_i = (result[i].end - result[i].start).normalize();
                let dir_j = (result[j].end - result[j].start).normalize();

                // Must be collinear and same normal.
                if dir_i.dot(dir_j).abs() < 0.99 {
                    continue;
                }
                if result[i].outward_normal.dot(result[j].outward_normal) < 0.99 {
                    continue;
                }

                // Must be on the same line (close perpendicularly).
                let perp = Vec2::new(-dir_i.y, dir_i.x);
                if perp.dot(result[j].start - result[i].start).abs() > tolerance {
                    continue;
                }

                // Project both onto the shared direction.
                let origin = result[i].start;
                let ti_s = dir_i.dot(result[i].start - origin);
                let ti_e = dir_i.dot(result[i].end - origin);
                let tj_s = dir_i.dot(result[j].start - origin);
                let tj_e = dir_i.dot(result[j].end - origin);

                let (i_min, i_max) = (ti_s.min(ti_e), ti_s.max(ti_e));
                let (j_min, j_max) = (tj_s.min(tj_e), tj_s.max(tj_e));

                // Check for overlap.
                let overlap_start = i_min.max(j_min);
                let overlap_end = i_max.min(j_max);
                if overlap_end - overlap_start < 1.0 {
                    continue; // No significant overlap.
                }

                // Trim the shorter spine to its non-overlapping portion.
                let len_i = i_max - i_min;
                let len_j = j_max - j_min;
                let (shorter, longer_min, longer_max) = if len_i <= len_j {
                    (i, j_min, j_max)
                } else {
                    (j, i_min, i_max)
                };

                let s_min = if shorter == i { i_min } else { j_min };
                let s_max = if shorter == i { i_max } else { j_max };

                // The non-overlapping portion is [s_min, longer_min) or
                // (longer_max, s_max]. Keep the longer tail.
                let tail_left = (longer_min - s_min).max(0.0);
                let tail_right = (s_max - longer_max).max(0.0);

                if tail_left > tail_right && tail_left > 1.0 {
                    // Keep the left tail.
                    result[shorter].start = origin + dir_i * s_min;
                    result[shorter].end = origin + dir_i * longer_min;
                    changed = true;
                } else if tail_right > 1.0 {
                    // Keep the right tail.
                    result[shorter].start = origin + dir_i * longer_max;
                    result[shorter].end = origin + dir_i * s_max;
                    changed = true;
                } else {
                    // Entirely covered: mark for removal (zero length).
                    result[shorter].end = result[shorter].start;
                    changed = true;
                }

                if changed {
                    break;
                }
            }
            if changed {
                break;
            }
        }

        // Remove zero-length spines.
        result.retain(|s| (s.end - s.start).length() > 1.0);
    }

    result
}

/// Generate stalls from positive-space face extraction + per-edge spine shifting.
/// Returns (stalls, islands, corridor_polygons, spine_lines, faces, miter_fills).
pub fn generate_from_spines(
    graph: &DriveAisleGraph,
    boundary: &Polygon,
    params: &ParkingParams,
    debug: &DebugToggles,
) -> (Vec<StallQuad>, Vec<Island>, Vec<Vec<Vec2>>, Vec<SpineLine>, Vec<Face>, Vec<Vec<Vec2>>, Vec<SkeletonDebug>) {
    let stall_angle_rad = params.stall_angle_deg.to_radians();
    let effective_depth = params.stall_depth * stall_angle_rad.sin();

    // Build deduplicated corridor rectangles + miter wedge fills, then
    // boolean-union them into merged shapes (preserving holes for loops).
    let dedup_corridors = deduplicate_corridors(graph);
    let miter_fills = generate_miter_fills(graph, debug);
    let merged_corridors = merge_corridor_shapes(&dedup_corridors, graph, debug);

    // Flatten outer contours for rendering (aisle polygon overlay).
    let aisle_polygons: Vec<Vec<Vec2>> = merged_corridors
        .iter()
        .filter(|shape| !shape.is_empty())
        .map(|shape| shape[0].clone())
        .collect();

    // Extract faces by subtracting the merged corridor union from the
    // boundary. Since the union resolves all internal seams between
    // corridor rects and miter fills, no sliver artifacts remain.
    let faces = if debug.face_extraction {
        extract_faces(boundary, &merged_corridors)
    } else {
        vec![]
    };

    // Collect spines from all faces, then merge collinear segments across
    // face boundaries so stalls flow continuously along shared aisle edges.
    let mut raw_spines = Vec::new();
    let mut skeleton_debugs = Vec::new();
    for shape in faces.iter() {
        // Optionally collect skeleton debug data for visualization.
        if debug.skeleton_debug && !shape.is_empty() && shape[0].len() >= 3 {
            let sk = skeleton::compute_skeleton(&shape[0]);
            skeleton_debugs.push(SkeletonDebug {
                arcs: sk.arcs.iter().map(|&(a, b)| [a, b]).collect(),
                nodes: sk.nodes.clone(),
            });
        }
        let face_spines = compute_face_spines(shape, effective_depth, &merged_corridors, debug);
        let face_spines = if debug.spine_dedup {
            dedup_overlapping_spines(face_spines, 2.0)
        } else {
            face_spines
        };
        raw_spines.extend(face_spines);
    }
    let all_spines = if debug.spine_merging {
        merge_collinear_spines(raw_spines, 1.0)
    } else {
        raw_spines
    };

    let all_spines: Vec<SpineSegment> = if debug.short_spine_filter {
        let min_spine_len = effective_depth;
        all_spines
            .into_iter()
            .filter(|s| (s.end - s.start).length() >= min_spine_len)
            .collect()
    } else {
        all_spines
    };

    // Place stalls on merged spines.
    let all_stalls = place_stalls_on_spines(&all_spines, params);

    // Clip stalls to face interiors. A stall that protrudes past the face
    // boundary into a corridor (at miter-fill corners) is removed.
    let all_stalls = if debug.stall_face_clipping {
        clip_stalls_to_faces(all_stalls, &faces)
    } else {
        all_stalls
    };



    // Compute endcap islands per face (face minus stalls placed in that face).
    let mut all_islands = Vec::new();
    let min_island_area = 10.0;
    // Minimum island width: filter thin slivers between close parallel spines.
    let min_island_width = params.stall_width;

    if debug.endcap_islands {
        for shape in &faces {
            let islands = endcap_islands_for_face(shape, &all_stalls, min_island_area);
            for island in islands {
                if !debug.island_width_filter {
                    all_islands.push(island);
                    continue;
                }
                // Approximate minimum width: 4 * area / perimeter.
                let area = signed_area(&island.polygon).abs();
                let perimeter: f64 = island.polygon.iter().enumerate().map(|(i, v)| {
                    let next = &island.polygon[(i + 1) % island.polygon.len()];
                    (*next - *v).length()
                }).sum();
                if perimeter > 0.0 && 4.0 * area / perimeter >= min_island_width {
                    all_islands.push(island);
                }
            }
        }
    }

    let spine_lines: Vec<SpineLine> = all_spines
        .iter()
        .map(|s| SpineLine {
            start: s.start,
            end: s.end,
            normal: s.outward_normal,
        })
        .collect();

    let face_list: Vec<Face> = faces
        .iter()
        .filter(|shape| !shape.is_empty() && shape[0].len() >= 3)
        .map(|shape| Face {
            contour: shape[0].clone(),
            holes: shape[1..].iter().cloned().collect(),
        })
        .collect();

    (all_stalls, all_islands, aisle_polygons, spine_lines, face_list, miter_fills, skeleton_debugs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aisle_graph::auto_generate;

    /// Test helper: create a corridor rectangle with one side along a→b.
    /// The face edge midpoint will lie exactly on this corridor edge.
    fn test_corridor_along(a: Vec2, b: Vec2) -> Vec<Vec<Vec2>> {
        let dir = (b - a).normalize();
        let perp = Vec2::new(-dir.y, dir.x);
        let w = 24.0;
        vec![vec![a, b, b + perp * w, a + perp * w]]
    }

    #[test]
    fn test_basic_rectangle_debug() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(300.0, 0.0),
                Vec2::new(300.0, 200.0),
                Vec2::new(0.0, 200.0),
            ],
            holes: vec![],
        };
        let params = ParkingParams::default();
        let graph = auto_generate(&boundary, &params);

        let (stalls, islands, _, spines, _, _) = generate_from_spines(&graph, &boundary, &params, &DebugToggles::default());
        eprintln!("\nTotal stalls: {}", stalls.len());
        eprintln!("Total spines: {}", spines.len());
        eprintln!("Total islands: {}", islands.len());
        for (i, isl) in islands.iter().enumerate() {
            let isl_area = signed_area(&isl.polygon).abs();
            eprintln!("  island {}: area={:.0} kind={:?}", i, isl_area, isl.kind);
        }
        assert!(stalls.len() > 50);
    }

    #[test]
    fn test_attempt_debug() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(227.0, 24.33),
                Vec2::new(300.0, 200.0),
                Vec2::new(0.0, 200.0),
            ],
            holes: vec![],
        };
        let params = ParkingParams::default();
        let graph = auto_generate(&boundary, &params);

        eprintln!("\n=== Attempt polygon debug ===");
        eprintln!("Graph: {} vertices, {} edges", graph.vertices.len(), graph.edges.len());
        for (i, v) in graph.vertices.iter().enumerate() {
            eprintln!("  v{}: ({:.1}, {:.1})", i, v.x, v.y);
        }
        for (i, e) in graph.edges.iter().enumerate() {
            let s = graph.vertices[e.start];
            let end = graph.vertices[e.end];
            eprintln!("  e{}: v{}→v{} ({:.1},{:.1})→({:.1},{:.1}) w={:.1}",
                i, e.start, e.end, s.x, s.y, end.x, end.y, e.width);
        }

        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();
        let dedup_corridors = deduplicate_corridors(&graph);

        eprintln!("\nMiter fills:");
        let miter_fills = generate_miter_fills(&graph);
        for (i, fill) in miter_fills.iter().enumerate() {
            let area = signed_area(fill).abs();
            eprintln!("  fill {}: area={:.1}, {} verts", i, area, fill.len());
            for (vi, v) in fill.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        eprintln!("\nCorridor rects:");
        for (i, c) in dedup_corridors.iter().enumerate() {
            eprintln!("  rect {}: {} verts", i, c.len());
            for (vi, v) in c.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        let merged_corridors = merge_corridor_shapes(&dedup_corridors, &graph);

        eprintln!("\nMerged corridors: {}", merged_corridors.len());
        for (ci, shape) in merged_corridors.iter().enumerate() {
            eprintln!("  corridor shape {}: {} contours", ci, shape.len());
            for (ki, contour) in shape.iter().enumerate() {
                let area = signed_area(contour);
                eprintln!("    contour {}: {} verts, signed_area={:.0}", ki, contour.len(), area);
                for (vi, v) in contour.iter().enumerate() {
                    eprintln!("      v{}: ({:.1}, {:.1})", vi, v.x, v.y);
                }
            }
        }

        let faces = extract_faces(&boundary, &merged_corridors);
        eprintln!("\nFaces: {}", faces.len());
        for (fi, shape) in faces.iter().enumerate() {
            let outer = &shape[0];
            let area = signed_area(outer).abs();
            eprintln!("  face {}: {} contours, outer={} verts, area={:.0}", fi, shape.len(), outer.len(), area);
            for (ki, contour) in shape.iter().enumerate() {
                let sa = signed_area(contour);
                let label = if ki == 0 { "outer" } else { "hole" };
                eprintln!("    {} {}: {} verts, signed_area={:.0}", label, ki, contour.len(), sa);
                for (vi, v) in contour.iter().enumerate() {
                    eprintln!("      v{}: ({:.1}, {:.1})", vi, v.x, v.y);
                }
            }

            let face_spines = compute_face_spines(shape, effective_depth, &merged_corridors);
            eprintln!("  spines from face {}: {}", fi, face_spines.len());
            for (si, s) in face_spines.iter().enumerate() {
                let len = (s.end - s.start).length();
                eprintln!("    spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} n=({:.2},{:.2})",
                    si, s.start.x, s.start.y, s.end.x, s.end.y, len,
                    s.outward_normal.x, s.outward_normal.y);
            }
        }
    }

    /// Two aisle edges meeting at an acute corner: unmerged corridor
    /// rectangles produce a jagged overlap (8 vertices from 2 rects),
    /// while the boolean union should produce a single merged polygon
    /// with a clean mitered corner (fewer vertices, single shape).
    #[test]
    fn test_corridor_merge_acute_corner() {
        // Two edges meeting at a ~45° angle at vertex 1.
        //   v0 ---- v1
        //             \
        //              v2
        let vertices = vec![
            Vec2::new(0.0, 100.0),   // v0: left
            Vec2::new(100.0, 100.0), // v1: corner
            Vec2::new(200.0, 0.0),   // v2: bottom-right
        ];

        let w = 12.0; // half-width
        let edge_a = AisleEdge {
            start: 0,
            end: 1,
            width: w,
            interior: false,
        };
        let edge_b = AisleEdge {
            start: 1,
            end: 2,
            width: w,
            interior: false,
        };

        let rect_a = corridor_polygon(&vertices, &edge_a);
        let rect_b = corridor_polygon(&vertices, &edge_b);

        // Unmerged: two separate 4-vertex rectangles.
        assert_eq!(rect_a.len(), 4);
        assert_eq!(rect_b.len(), 4);

        // Merged: should produce a single polygon with a clean mitered
        // corner — fewer total vertices than 2 separate rectangles.
        let graph = DriveAisleGraph {
            vertices: vertices.clone(),
            edges: vec![edge_a.clone(), edge_b.clone()],
            perim_vertex_count: 3,
        };
        let merged = merge_corridor_shapes(&[rect_a.clone(), rect_b.clone()], &graph);

        eprintln!("\nUnmerged: 2 rects × 4 verts = 8 verts");
        eprintln!("Merged: {} shape(s)", merged.len());
        for (i, shape) in merged.iter().enumerate() {
            if shape.is_empty() { continue; }
            let poly = &shape[0];
            let area = signed_area(poly).abs();
            eprintln!(
                "  shape {}: {} verts (outer), {} holes, area={:.0}",
                i,
                poly.len(),
                shape.len() - 1,
                area
            );
            for (vi, v) in poly.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        // The two rectangles overlap at the corner, so the union should
        // produce exactly 1 merged shape (not 2 separate ones).
        assert_eq!(merged.len(), 1, "overlapping corridors should merge into 1 shape");
    }

    /// Trapezoidal face: horizontal bottom (aisle), diagonal top (aisle),
    /// vertical sides (boundary).
    ///
    /// Face shape (CCW):
    ///
    ///   v3 -------- v2        diagonal top edge (aisle-facing)
    ///   |            \
    ///   |             \
    ///   v0 ----------- v1     horizontal bottom edge (aisle-facing)
    ///
    /// The skeleton should partition the face so that each aisle edge's
    /// stalls stay in their own region. The right side of the face is
    /// narrower than the left, so the bottom spine should be clipped
    /// where it meets the diagonal's territory.
    #[test]
    fn test_trapezoid_face_spines() {
        let face_contour = vec![
            Vec2::new(0.0, 0.0),     // v0: bottom-left
            Vec2::new(200.0, 0.0),   // v1: bottom-right
            Vec2::new(200.0, 20.0),  // v2: top-right (lower)
            Vec2::new(0.0, 60.0),    // v3: top-left (higher)
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (bottom and top diagonal).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[1]),
            test_corridor_along(face_contour[2], face_contour[3]),
        ];

        let params = ParkingParams::default(); // 90° stalls, 18ft depth
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes);

        eprintln!("\n=== Trapezoid face spine test ===");
        eprintln!("effective_depth = {:.1}", effective_depth);
        eprintln!("spines generated: {}", spines.len());
        for (i, s) in spines.iter().enumerate() {
            let len = (s.end - s.start).length();
            eprintln!(
                "  spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} normal=({:.2},{:.2})",
                i, s.start.x, s.start.y, s.end.x, s.end.y, len,
                s.outward_normal.x, s.outward_normal.y
            );
        }

        assert!(
            !spines.is_empty(),
            "trapezoid face should produce at least one spine"
        );

        // The two spines should meet at a skeleton node — no overlap.
        // Both spines should exist (face is wide enough on the left for
        // two rows: 60 > 2×18 = 36).
        assert_eq!(
            spines.len(),
            2,
            "should have 2 spines (bottom horizontal + top diagonal)"
        );

        // Place stalls and verify ALL stall corners are inside the face.
        let stalls = place_stalls_on_spines(&spines, &params);
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.corners.iter().enumerate() {
                if !point_in_face(*corner, &shape) {
                    let min_dist = face_contour
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            let j = (i + 1) % face_contour.len();
                            point_to_segment_dist(*corner, face_contour[i], face_contour[j])
                        })
                        .fold(f64::INFINITY, f64::min);
                    if min_dist > 1.0 {
                        eprintln!(
                            "  OUTSIDE: stall {} corner {} at ({:.1},{:.1}), dist_to_edge={:.1}",
                            si, ci, corner.x, corner.y, min_dist
                        );
                        outside_count += 1;
                    }
                }
            }
        }
        assert_eq!(
            outside_count, 0,
            "all stall corners must be inside the face (or within 1ft tolerance)"
        );
    }

    /// Pentagon face: two parallel aisle edges, flat base, pointed cap.
    /// This is the original flickering case — the skeleton should always
    /// produce spines for both long parallel edges.
    ///
    ///   v0 ——————————— v1
    ///   |                \
    ///   |                 v2    ← cap
    ///   |                /
    ///   v4 ——————————— v3
    #[test]
    fn test_pentagon_face_spines() {
        let face_contour = vec![
            Vec2::new(0.0, 60.0),    // v0: top-left
            Vec2::new(180.0, 60.0),  // v1: top-right
            Vec2::new(200.0, 30.0),  // v2: cap point
            Vec2::new(180.0, 0.0),   // v3: bottom-right
            Vec2::new(0.0, 0.0),     // v4: bottom-left
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (top and bottom).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[1]),
            test_corridor_along(face_contour[3], face_contour[4]),
        ];

        let params = ParkingParams::default();
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes);

        eprintln!("\n=== Pentagon face spine test ===");
        eprintln!("effective_depth = {:.1}", effective_depth);
        eprintln!("spines generated: {}", spines.len());
        for (i, s) in spines.iter().enumerate() {
            let len = (s.end - s.start).length();
            eprintln!(
                "  spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} normal=({:.2},{:.2})",
                i, s.start.x, s.start.y, s.end.x, s.end.y, len,
                s.outward_normal.x, s.outward_normal.y
            );
        }

        // Must always produce spines (no flickering to zero).
        assert!(
            !spines.is_empty(),
            "pentagon face must always produce spines"
        );

        // Should have at least one spine for each parallel aisle edge
        // (top and bottom), meeting at a skeleton node near the cap.
        let top_spines: Vec<&SpineSegment> =
            spines.iter().filter(|s| s.outward_normal.y > 0.5).collect();
        let bottom_spines: Vec<&SpineSegment> =
            spines.iter().filter(|s| s.outward_normal.y < -0.5).collect();

        eprintln!(
            "top spines: {}, bottom spines: {}",
            top_spines.len(),
            bottom_spines.len()
        );

        // The face is 60ft tall and stalls are 18ft deep. Two rows of 18
        // = 36ft < 60ft, so both spines should exist.
        assert!(
            !top_spines.is_empty(),
            "should have a top spine (aisle along v0→v1)"
        );
        assert!(
            !bottom_spines.is_empty(),
            "should have a bottom spine (aisle along v3→v4)"
        );

        // Place stalls and verify containment.
        let stalls = place_stalls_on_spines(&spines, &params);
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.corners.iter().enumerate() {
                if !point_in_face(*corner, &shape) {
                    let min_dist = face_contour
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            let j = (i + 1) % face_contour.len();
                            point_to_segment_dist(*corner, face_contour[i], face_contour[j])
                        })
                        .fold(f64::INFINITY, f64::min);
                    if min_dist > 1.0 {
                        eprintln!(
                            "  OUTSIDE: stall {} corner {} at ({:.1},{:.1}), dist_to_edge={:.1}",
                            si, ci, corner.x, corner.y, min_dist
                        );
                        outside_count += 1;
                    }
                }
            }
        }
        assert_eq!(
            outside_count, 0,
            "all stall corners must be inside the face (or within 1ft tolerance)"
        );
    }

    /// Narrow face: two parallel aisle edges only 30ft apart (< 2×18=36ft).
    /// The skeleton collapses — both offset lines cross at 15ft, so no
    /// valid inset polygon survives. No stalls are placed (face too narrow
    /// for two opposing rows).
    #[test]
    fn test_narrow_face_no_stalls() {
        let face_contour = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(200.0, 0.0),
            Vec2::new(200.0, 30.0),
            Vec2::new(0.0, 30.0),
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (bottom and top).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[1]),
            test_corridor_along(face_contour[2], face_contour[3]),
        ];

        let params = ParkingParams::default();
        let effective_depth = params.stall_depth
            * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes);
        let stalls = place_stalls_on_spines(&spines, &params);

        eprintln!("\n=== Narrow face test (30ft between aisles) ===");
        eprintln!("spines: {}, stalls: {}", spines.len(), stalls.len());

        assert_eq!(spines.len(), 0, "skeleton should collapse for narrow face");
        assert_eq!(stalls.len(), 0, "no stalls in a face too narrow for two rows");
    }

    /// Face between left/right aisle corridors with a diagonal boundary
    /// at the top. The vertical spines run between the corridors, with
    /// stalls going left and right. Near the diagonal top, the face
    /// narrows — stalls there must not extend outside the face.
    ///
    ///     v3 --------- v2      ← diagonal boundary (top)
    ///    /               |
    ///   /                |
    ///  v0 ------------- v1     ← horizontal boundary (bottom)
    ///
    /// Left edge (v0→v3): aisle-facing (corridor on the left)
    /// Right edge (v1→v2): aisle-facing (corridor on the right)
    /// Bottom (v0→v1) and top diagonal (v2→v3): boundary
    ///
    /// The face is 80ft wide × 100ft (left) / 120ft (right) tall.
    /// The left spine (x=18) and right spine (x=62) each produce stalls.
    /// Near the top where the diagonal cuts across, the last stalls
    /// must not poke outside the face.
    #[test]
    fn test_vertical_spines_diagonal_top() {
        let face_contour = vec![
            Vec2::new(0.0, 0.0),      // v0: bottom-left
            Vec2::new(80.0, 0.0),     // v1: bottom-right
            Vec2::new(80.0, 120.0),   // v2: top-right
            Vec2::new(0.0, 100.0),    // v3: top-left
        ];
        let shape = vec![face_contour.clone()];

        // Corridors along the aisle-facing edges (left and right verticals).
        let corridor_shapes = vec![
            test_corridor_along(face_contour[0], face_contour[3]),
            test_corridor_along(face_contour[1], face_contour[2]),
        ];

        let params = ParkingParams::default();
        let effective_depth =
            params.stall_depth * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes);

        eprintln!("\n=== Vertical spines + diagonal top test ===");
        eprintln!("effective_depth = {:.1}", effective_depth);
        eprintln!("spines generated: {}", spines.len());
        for (i, s) in spines.iter().enumerate() {
            let len = (s.end - s.start).length();
            eprintln!(
                "  spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} normal=({:.2},{:.2})",
                i, s.start.x, s.start.y, s.end.x, s.end.y, len,
                s.outward_normal.x, s.outward_normal.y
            );
        }

        assert!(
            !spines.is_empty(),
            "should produce vertical spines from left/right aisle edges"
        );

        // Place stalls and verify ALL corners are inside the face.
        let stalls = place_stalls_on_spines(&spines, &params);
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.corners.iter().enumerate() {
                if !point_in_face(*corner, &shape) {
                    let min_dist = face_contour
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            let j = (i + 1) % face_contour.len();
                            point_to_segment_dist(*corner, face_contour[i], face_contour[j])
                        })
                        .fold(f64::INFINITY, f64::min);
                    if min_dist > 1.0 {
                        eprintln!(
                            "  OUTSIDE: stall {} corner {} at ({:.1},{:.1}), dist={:.1}",
                            si, ci, corner.x, corner.y, min_dist
                        );
                        outside_count += 1;
                    }
                }
            }
        }
        assert_eq!(
            outside_count, 0,
            "stalls near the diagonal top must not extend outside the face"
        );
    }

    /// Debug: reproduce the slanted-boundary layout from attempt.txt and
    /// trace why the rightmost face has no stalls.
    #[test]
    fn test_attempt_boundary_debug() {
        use crate::generate::generate;
        use crate::types::GenerateInput;

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(130.67, 102.33),
                    Vec2::new(300.0, 200.0),
                    Vec2::new(0.0, 200.0),
                ],
                holes: vec![],
            },
            aisle_graph: None,
            params: ParkingParams::default(),
        };

        let layout = generate(input.clone());
        eprintln!("\n=== attempt boundary debug ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Recompute corridor shapes for debug classification.
        let graph = auto_generate(&input.boundary, &input.params);
        let dedup_corridors = deduplicate_corridors(&graph);
        let merged_corridors = merge_corridor_shapes(&dedup_corridors, &graph);

        let effective_depth = input.params.stall_depth
            * input.params.stall_angle_deg.to_radians().sin();

        for (fi, face) in layout.faces.iter().enumerate() {
            let contour = &face.contour;
            let area = crate::inset::signed_area(contour).abs();
            let perimeter: f64 = contour.iter().enumerate().map(|(i, v)| {
                let next = &contour[(i + 1) % contour.len()];
                (*next - *v).length()
            }).sum();
            let min_width = if perimeter > 0.0 { 4.0 * area / perimeter } else { 0.0 };
            eprintln!("\n  face {}: {} vertices, area={:.1}, perimeter={:.1}, min_width={:.1} (ed={:.1})",
                fi, contour.len(), area, perimeter, min_width, effective_depth);
            for (vi, v) in contour.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }

            let aisle_facing = classify_face_edges(contour, &merged_corridors);
            let ed = effective_depth - 0.05;
            let min_edge_len = effective_depth * 2.0;
            for i in 0..contour.len() {
                let j = (i + 1) % contour.len();
                let edge_len = (contour[j] - contour[i]).length();
                let dist = if aisle_facing[i] && edge_len >= min_edge_len { ed } else { 0.0 };
                eprintln!(
                    "    edge {}->{}: len={:.1} aisle_facing={} dist={:.1}{}",
                    i, j, edge_len, aisle_facing[i], dist,
                    if !aisle_facing[i] { " (boundary)" }
                    else if edge_len < min_edge_len { " (too short)" }
                    else { "" }
                );
            }

            for (hi, hole) in face.holes.iter().enumerate() {
                eprintln!("    hole {}: {} verts", hi, hole.len());
                let hole_af = classify_face_edges(hole, &merged_corridors);
                for i in 0..hole.len() {
                    let j = (i + 1) % hole.len();
                    let elen = (hole[j] - hole[i]).length();
                    eprintln!("      edge {}->{}: len={:.1} af={} ({:.1},{:.1})->({:.1},{:.1})",
                        i, j, elen, hole_af[i], hole[i].x, hole[i].y, hole[j].x, hole[j].y);
                }
            }
            let mut shape = vec![contour.clone()];
            shape.extend(face.holes.iter().cloned());
            let spines = compute_face_spines(&shape, effective_depth, &merged_corridors);
            eprintln!("    spines: {}", spines.len());
            for (si, s) in spines.iter().enumerate() {
                let len = (s.end - s.start).length();
                eprintln!(
                    "      spine {}: ({:.1},{:.1})->({:.1},{:.1}) len={:.1} n=({:.2},{:.2})",
                    si, s.start.x, s.start.y, s.end.x, s.end.y, len,
                    s.outward_normal.x, s.outward_normal.y
                );
            }
        }
    }

    #[test]
    fn test_slanted_boundary_debug() {
        use crate::generate::generate;
        use crate::types::GenerateInput;

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(429.0, 1.67),
                    Vec2::new(300.0, 200.0),
                    Vec2::new(0.0, 200.0),
                ],
                holes: vec![],
            },
            aisle_graph: None,
            params: ParkingParams::default(),
        };

        let layout = generate(input.clone());
        eprintln!("\n=== slanted boundary debug ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Recompute corridor shapes for debug classification.
        let graph = auto_generate(&input.boundary, &input.params);
        let dedup_corridors = deduplicate_corridors(&graph);
        let merged_corridors = merge_corridor_shapes(&dedup_corridors, &graph);

        let effective_depth = input.params.stall_depth
            * input.params.stall_angle_deg.to_radians().sin();

        // Examine each face.
        for (fi, face) in layout.faces.iter().enumerate() {
            let contour = &face.contour;
            let area = crate::inset::signed_area(contour).abs();
            eprintln!("\n  face {}: {} vertices, area={:.1}", fi, contour.len(), area);
            for (vi, v) in contour.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }

            // Classify edges for this face.
            let aisle_facing = classify_face_edges(contour, &merged_corridors);
            let ed = effective_depth - 0.05;
            let min_edge_len = effective_depth * 2.0;
            for i in 0..contour.len() {
                let j = (i + 1) % contour.len();
                let edge_len = (contour[j] - contour[i]).length();
                let dist = if aisle_facing[i] && edge_len >= min_edge_len { ed } else { 0.0 };
                eprintln!(
                    "    edge {}->{}: len={:.1} aisle_facing={} dist={:.1}{}",
                    i, j, edge_len, aisle_facing[i], dist,
                    if !aisle_facing[i] { " (boundary)" }
                    else if edge_len < min_edge_len { " (too short)" }
                    else { "" }
                );
            }

            // Print holes.
            eprintln!("    holes: {}", face.holes.len());
            for (hi, hole) in face.holes.iter().enumerate() {
                eprintln!("    hole {}: {} vertices", hi, hole.len());
                for (vi, v) in hole.iter().enumerate() {
                    eprintln!("      v{}: ({:.1}, {:.1})", vi, v.x, v.y);
                }
            }

            // Compute spines for this face using full shape (contour + holes).
            let mut shape = vec![contour.clone()];
            shape.extend(face.holes.iter().cloned());
            let spines = compute_face_spines(&shape, effective_depth, &merged_corridors);
            eprintln!("    spines: {}", spines.len());
            for (si, s) in spines.iter().enumerate() {
                let len = (s.end - s.start).length();
                eprintln!(
                    "      spine {}: ({:.1},{:.1})->({:.1},{:.1}) len={:.1} n=({:.2},{:.2})",
                    si, s.start.x, s.start.y, s.end.x, s.end.y, len,
                    s.outward_normal.x, s.outward_normal.y
                );
            }
        }
    }
}
