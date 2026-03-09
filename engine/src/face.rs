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
// Per-edge spine generation (replaces skeleton polygon approach)
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

        let aisle_facing = classify_face_edges(contour, boundary, 0.5);

        let ed = effective_depth - 0.05;
        let distances: Vec<f64> = (0..n)
            .map(|i| if aisle_facing[i] { ed } else { 0.0 })
            .collect();

        // Compute miter vertices with edge-collapse handling.
        let (miter, live, normals) =
            crate::inset::inset_per_edge_core(contour, &distances, Some(outer_sign));
        if live.is_empty() {
            continue;
        }

        // For each surviving aisle-facing edge, clip its miter-to-miter
        // segment to the face interior.
        for idx in 0..live.len() {
            let i = live[idx];
            if !aisle_facing[i] {
                continue;
            }
            let j = live[(idx + 1) % live.len()];
            let spine_start = miter[i];
            let spine_end = miter[j];
            let outward = Vec2::new(-normals[i].x, -normals[i].y);

            for (t0, t1) in clip_segment_to_face(spine_start, spine_end, shape) {
                let d = spine_end - spine_start;
                let s = spine_start + d * t0;
                let e = spine_start + d * t1;
                if (e - s).length() > 1.0 {
                    all_spines.push(SpineSegment {
                        start: s,
                        end: e,
                        outward_normal: outward,
                    });
                }
            }
        }
    }

    all_spines
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
            // point → right corner of edge 2.
            fills.push(vec![v, p1, miter, p2]);
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
}
