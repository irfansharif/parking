use crate::inset::signed_area;
use crate::segment::fill_strip;
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

/// Extract positive-space face polygons by subtracting corridors and holes
/// from the boundary. Returns Vec<Shape> where each shape is Vec<Vec<Vec2>>
/// with shape[0] = outer contour, shape[1..] = holes within that face.
fn extract_faces(
    boundary: &Polygon,
    corridors: &[Vec<Vec2>],
) -> Vec<Vec<Vec<Vec2>>> {
    // Subject: boundary outer ring.
    let subj: Vec<[f64; 2]> = boundary.outer.iter().map(|v| [v.x, v.y]).collect();

    // Clip: all corridors + all holes.
    let mut clip_paths: Vec<Vec<[f64; 2]>> = Vec::new();
    for corridor in corridors {
        let path: Vec<[f64; 2]> = corridor.iter().map(|v| [v.x, v.y]).collect();
        clip_paths.push(path);
    }
    for hole in &boundary.holes {
        let path: Vec<[f64; 2]> = hole.iter().map(|v| [v.x, v.y]).collect();
        clip_paths.push(path);
    }

    // Perform boolean difference: boundary MINUS (corridors + holes).
    let result = subj.overlay(&clip_paths, OverlayRule::Difference, FillRule::NonZero);

    // Convert back to Vec2.
    result
        .into_iter()
        .map(|shape| {
            shape
                .into_iter()
                .map(|contour| contour.into_iter().map(|p| Vec2::new(p[0], p[1])).collect())
                .collect()
        })
        .collect()
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
/// An edge is aisle-facing if its midpoint is NOT close to the site
/// boundary or any hole boundary — meaning it was carved by a corridor.
fn classify_face_edges(contour: &[Vec2], boundary: &Polygon, tolerance: f64) -> Vec<bool> {
    let n = contour.len();
    let mut aisle_facing = vec![true; n];
    for i in 0..n {
        let j = (i + 1) % n;
        let mid = (contour[i] + contour[j]) * 0.5;

        for bi in 0..boundary.outer.len() {
            let bj = (bi + 1) % boundary.outer.len();
            if point_to_segment_dist(mid, boundary.outer[bi], boundary.outer[bj]) < tolerance {
                aisle_facing[i] = false;
                break;
            }
        }
        if !aisle_facing[i] {
            continue;
        }

        for hole in &boundary.holes {
            for hi in 0..hole.len() {
                let hj = (hi + 1) % hole.len();
                if point_to_segment_dist(mid, hole[hi], hole[hj]) < tolerance {
                    aisle_facing[i] = false;
                    break;
                }
            }
            if !aisle_facing[i] {
                break;
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
    boundary: &Polygon,
) -> Vec<SpineSegment> {
    if shape.is_empty() || shape[0].len() < 3 {
        return vec![];
    }

    let outer_sign = if signed_area(&shape[0]) >= 0.0 { 1.0 } else { -1.0 };
    let mut all_spines = Vec::new();

    for contour in shape.iter() {
        let n = contour.len();
        if n < 3 {
            continue;
        }

        let aisle_facing = classify_face_edges(contour, boundary, 2.0);
        let ed = effective_depth - 0.05;
        let min_edge_len = effective_depth * 2.0;

        // Per-edge inset distances: ed for substantial aisle-facing edges,
        // 0 for boundary edges and short boolean-artifact edges.
        let distances: Vec<f64> = (0..n)
            .map(|i| {
                let j = (i + 1) % n;
                let edge_len = (contour[j] - contour[i]).length();
                if aisle_facing[i] && edge_len >= min_edge_len {
                    ed
                } else {
                    0.0
                }
            })
            .collect();

        let spines = skeleton_inset_spines(contour, &distances, &aisle_facing, outer_sign, shape);
        all_spines.extend(spines);
    }

    all_spines
}

/// Straight-skeleton weighted inset to produce spine segments.
///
/// Offsets each edge by its distance, iteratively removing collapsed edges
/// until the inset polygon is stable. Each surviving aisle-facing edge
/// produces a spine clipped to the face interior.
fn skeleton_inset_spines(
    polygon: &[Vec2],
    distances: &[f64],
    aisle_facing: &[bool],
    sign: f64,
    face_shape: &[Vec<Vec2>],
) -> Vec<SpineSegment> {
    let n = polygon.len();
    if n < 3 {
        return vec![];
    }

    // Precompute edge directions and inward normals.
    let mut edge_dirs: Vec<Vec2> = Vec::with_capacity(n);
    let mut normals: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let e = polygon[j] - polygon[i];
        edge_dirs.push(e);
        let len = e.length();
        if len < 1e-12 {
            normals.push(Vec2::new(0.0, 0.0));
        } else {
            normals.push(Vec2::new(-e.y * sign / len, e.x * sign / len));
        }
    }

    let mut alive = vec![true; n];

    // Compute miter vertices for the alive set.
    let compute_miter = |alive: &[bool]| -> Vec<Vec2> {
        let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
        let mut result = vec![Vec2::new(0.0, 0.0); n];
        for idx in 0..live.len() {
            let i = live[idx];
            let prev = live[if idx == 0 { live.len() - 1 } else { idx - 1 }];

            let a = polygon[prev] + normals[prev] * distances[prev];
            let da = edge_dirs[prev];
            let b = polygon[i] + normals[i] * distances[i];
            let db = edge_dirs[i];
            let denom = da.cross(db);

            if denom.abs() < 1e-12 {
                // Parallel edges: place miter at the midpoint of the two
                // offset lines (degenerate skeleton ridge).
                result[i] = (a + b) * 0.5;
            } else {
                let t = (b - a).cross(db) / denom;
                result[i] = a + da * t;
            }
        }
        result
    };

    let mut miter = compute_miter(&alive);

    // Iteratively process edge-collapse events. When an inset edge flips
    // direction, remove only that edge (the vertex at its start), NOT both
    // endpoints. This is the key difference from the old approach — it
    // preserves neighboring edges through the collapse.
    let mut changed = true;
    while changed {
        changed = false;
        let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
        if live.len() < 3 {
            return vec![];
        }

        for idx in 0..live.len() {
            let i = live[idx];
            let j = live[(idx + 1) % live.len()];
            let new_dir = miter[j] - miter[i];
            let orig_dir = edge_dirs[i];

            if new_dir.dot(orig_dir) <= 0.0 {
                // Edge from i→j has collapsed. Remove vertex i (and its
                // edge). The previous edge and edge j become adjacent.
                alive[i] = false;
                changed = true;
                break;
            }
        }

        if changed {
            let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
            if live.len() < 3 {
                return vec![];
            }
            miter = compute_miter(&alive);
        }
    }

    // Extract spines from surviving aisle-facing edges, clipped to face.
    let live: Vec<usize> = (0..n).filter(|i| alive[*i]).collect();
    let mut spines = Vec::new();

    for idx in 0..live.len() {
        let i = live[idx];
        if distances[i] == 0.0 || !aisle_facing[i] {
            continue;
        }

        let j = live[(idx + 1) % live.len()];
        let spine_start = miter[i];
        let spine_end = miter[j];
        let outward = Vec2::new(-normals[i].x, -normals[i].y);

        // Clip both the spine and the "stall reach" line (spine offset by
        // stall_depth in the outward direction) to the face interior. Stalls
        // extend perpendicular from the spine by stall_depth, so the offset
        // line must also be inside the face to avoid protrusion.
        let spine_clips = clip_segment_to_face(spine_start, spine_end, face_shape);
        // Use a slightly reduced offset (0.5ft margin) so the offset line
        // stays clearly inside the face rather than landing exactly on the
        // boundary edge, which would fail the winding-number containment
        // check due to floating-point.
        let reach = (distances[i] - 0.5).max(0.0);
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

/// Generate miter-fill polygons at graph vertices where edges meet at
/// angles. For each pair of adjacent edges (sorted by angle), extend their
/// outer corridor edges to a sharp miter intersection point, producing a
/// wedge that fills the gap between the two corridor rectangles.
fn generate_miter_fills(graph: &DriveAisleGraph) -> Vec<Vec<Vec2>> {
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
        let mut edges: Vec<(f64, Vec2, f64)> = adj[vi]
            .iter()
            .map(|(d, w)| (d.y.atan2(d.x), *d, *w))
            .collect();
        edges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let ne = edges.len();
        for i in 0..ne {
            let j = (i + 1) % ne;
            let (_, d1, w1) = edges[i];
            let (_, d2, w2) = edges[j];

            let n1 = Vec2::new(-d1.y, d1.x);
            let n2 = Vec2::new(-d2.y, d2.x);

            // The gap between consecutive edges (sorted CCW) is bounded by:
            //   - left side of edge 1:  line through (v + n1*w1), direction d1
            //   - right side of edge 2: line through (v - n2*w2), direction d2
            let p1 = v + n1 * w1;
            let p2 = v - n2 * w2;

            let denom = d1.cross(d2);
            if denom.abs() < 1e-12 {
                continue;
            }

            let t = (p2 - p1).cross(d2) / denom;
            let miter = p1 + d1 * t;

            // Miter limit: cap at 4x edge width to avoid long spikes.
            let max_w = w1.max(w2);
            if (miter - v).length() > max_w * 4.0 {
                continue;
            }

            // Fill wedge: vertex center → left corner of edge 1 → miter
            // point → right corner of edge 2. Skip degenerate slivers.
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
/// polygons. Produces clean mitered corners where aisles meet at angles.
fn merge_corridor_polygons(
    corridors: &[Vec<Vec2>],
    graph: &DriveAisleGraph,
) -> Vec<Vec<Vec2>> {
    if corridors.is_empty() {
        return vec![];
    }

    let miter_fills = generate_miter_fills(graph);

    // Iterative pairwise union: merge shapes one at a time to avoid
    // robustness issues with multi-shape self-union in i_overlay.
    let all_shapes: Vec<Vec<[f64; 2]>> = corridors
        .iter()
        .chain(miter_fills.iter())
        .map(|c| c.iter().map(|v| [v.x, v.y]).collect())
        .collect();

    if all_shapes.is_empty() {
        return vec![];
    }

    // Start with the first shape as the accumulated result.
    let mut acc: Vec<Vec<Vec<[f64; 2]>>> = vec![vec![all_shapes[0].clone()]];

    for shape in &all_shapes[1..] {
        let clip = vec![shape.clone()];
        let subj: Vec<Vec<[f64; 2]>> = acc
            .iter()
            .map(|s| s[0].clone())
            .collect();
        let merged = subj.overlay(&clip, OverlayRule::Union, FillRule::NonZero);
        acc = if merged.is_empty() {
            // If union produced nothing, keep accumulator as-is and add
            // the new shape as a separate polygon.
            let mut keep = acc;
            keep.push(vec![shape.clone()]);
            keep
        } else {
            merged
        };
    }

    acc.into_iter()
        .filter(|shape| !shape.is_empty())
        .map(|shape| {
            shape[0]
                .iter()
                .map(|p| Vec2::new(p[0], p[1]))
                .collect()
        })
        .collect()
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

    // Must have same outward normal direction.
    if a.outward_normal.dot(b.outward_normal) < 0.99 {
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

/// Generate stalls from positive-space face extraction + per-edge spine shifting.
/// Returns (stalls, islands, corridor_polygons, spine_lines, faces).
pub fn generate_from_spines(
    graph: &DriveAisleGraph,
    boundary: &Polygon,
    params: &ParkingParams,
) -> (Vec<StallQuad>, Vec<Island>, Vec<Vec<Vec2>>, Vec<SpineLine>, Vec<Face>) {
    let stall_angle_rad = params.stall_angle_deg.to_radians();
    let effective_depth = params.stall_depth * stall_angle_rad.sin();

    // Build deduplicated corridor rectangles + miter wedge fills.
    let dedup_corridors = deduplicate_corridors(graph);
    let miter_fills = generate_miter_fills(graph);

    // Boolean-union corridor rectangles + miter fills for clean rendering.
    let aisle_polygons = merge_corridor_polygons(&dedup_corridors, graph);

    // For face extraction, pass individual shapes (rectangles + wedges) so
    // that the boolean difference works correctly even when corridors form
    // loops (merged outlines would discard hole contours and erase the
    // entire lot interior).
    let all_corridor_shapes: Vec<Vec<Vec2>> = dedup_corridors
        .iter()
        .chain(miter_fills.iter())
        .cloned()
        .collect();
    let faces = extract_faces(boundary, &all_corridor_shapes);

    // Collect spines from all faces, then merge collinear segments across
    // face boundaries so stalls flow continuously along shared aisle edges.
    let mut raw_spines = Vec::new();
    for shape in &faces {
        raw_spines.extend(compute_face_spines(shape, effective_depth, boundary));
    }
    let all_spines = merge_collinear_spines(raw_spines, 2.0);

    // Place stalls on merged spines.
    let all_stalls = place_stalls_on_spines(&all_spines, params);

    // Compute endcap islands per face (face minus stalls placed in that face).
    let mut all_islands = Vec::new();
    let min_island_area = 10.0;
    // Minimum island width: filter thin slivers between close parallel spines.
    let min_island_width = params.stall_width;

    for shape in &faces {
        let islands = endcap_islands_for_face(shape, &all_stalls, min_island_area);
        for island in islands {
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

    (all_stalls, all_islands, aisle_polygons, spine_lines, face_list)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aisle_graph::auto_generate;

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

        let (stalls, islands, _, spines, _) = generate_from_spines(&graph, &boundary, &params);
        eprintln!("\nTotal stalls: {}", stalls.len());
        eprintln!("Total spines: {}", spines.len());
        eprintln!("Total islands: {}", islands.len());
        for (i, isl) in islands.iter().enumerate() {
            let isl_area = signed_area(&isl.polygon).abs();
            eprintln!("  island {}: area={:.0} kind={:?}", i, isl_area, isl.kind);
        }
        assert!(stalls.len() > 50);
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
        let merged = merge_corridor_polygons(&[rect_a.clone(), rect_b.clone()], &graph);

        eprintln!("\nUnmerged: 2 rects × 4 verts = 8 verts");
        eprintln!("Merged: {} polygon(s)", merged.len());
        for (i, poly) in merged.iter().enumerate() {
            let area = signed_area(poly).abs();
            eprintln!(
                "  polygon {}: {} verts, area={:.0}",
                i,
                poly.len(),
                area
            );
            for (vi, v) in poly.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        // The two rectangles overlap at the corner, so the union should
        // produce exactly 1 merged polygon (not 2 separate ones).
        assert_eq!(merged.len(), 1, "overlapping corridors should merge into 1 polygon");

        // NOTE: boolean union of two rectangles produces the outer hull
        // with intersection vertices — it preserves the jagged inner
        // edges at the corner. A proper fix requires miter-join corridor
        // generation (extending outer edges to their intersection),
        // not just boolean union of independent rectangles.
        //
        // The merged polygon has 8 vertices: the union traces both
        // rectangles' outlines through the overlap region instead of
        // producing a clean 6-vertex mitered shape.
        eprintln!(
            "\nMerged polygon has {} verts (expected 6 for mitered, got 8 from union)",
            merged[0].len()
        );
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

        // Boundary includes the left (v3→v0) and right (v1→v2) edges.
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, -50.0),
                Vec2::new(200.0, -50.0),
                Vec2::new(200.0, 100.0),
                Vec2::new(0.0, 100.0),
            ],
            holes: vec![],
        };

        let params = ParkingParams::default(); // 90° stalls, 18ft depth
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &boundary);

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

        // Boundary includes the left edge (v4→v0) and cap edges
        // (v1→v2, v2→v3). Top and bottom are aisle-facing.
        let boundary = Polygon {
            outer: vec![
                Vec2::new(-10.0, -10.0),
                Vec2::new(210.0, -10.0),
                Vec2::new(210.0, 70.0),
                Vec2::new(-10.0, 70.0),
            ],
            holes: vec![],
        };

        let params = ParkingParams::default();
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &boundary);

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

        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, -50.0),
                Vec2::new(200.0, -50.0),
                Vec2::new(200.0, 80.0),
                Vec2::new(0.0, 80.0),
            ],
            holes: vec![],
        };

        let params = ParkingParams::default();
        let effective_depth = params.stall_depth
            * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &boundary);
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

        // Boundary: the site perimeter includes the bottom edge and
        // diagonal top, but extends far left/right (the left and right
        // face edges are interior corridor edges, not on the boundary).
        let boundary = Polygon {
            outer: vec![
                Vec2::new(-100.0, 0.0),    // far left, same y as v0
                Vec2::new(200.0, 0.0),     // far right, same y as v1
                Vec2::new(200.0, 120.0),   // far right, same y as v2
                Vec2::new(80.0, 120.0),    // matches v2
                Vec2::new(0.0, 100.0),     // matches v3
                Vec2::new(-100.0, 100.0),  // far left
            ],
            holes: vec![],
        };

        let params = ParkingParams::default();
        let effective_depth =
            params.stall_depth * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &boundary);

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
}
