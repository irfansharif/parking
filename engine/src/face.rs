use crate::inset::{inset_per_edge_core, signed_area};
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

/// Generate spine segments for a face shape (outer contour + holes)
/// using a polygon-inset approach.
///
/// Only aisle-facing edges (those adjacent to corridors) are inset by
/// `effective_depth`. Boundary/hole edges stay put and don't produce
/// spines.
fn compute_face_spines(
    shape: &[Vec<Vec2>],
    effective_depth: f64,
    boundary: &Polygon,
) -> Vec<SpineSegment> {
    if shape.is_empty() || shape[0].len() < 3 {
        return vec![];
    }

    // Use the outer contour's winding for all contours so that hole
    // edges inset away from the hole (toward the boundary).
    let outer_sign = if signed_area(&shape[0]) >= 0.0 { 1.0 } else { -1.0 };

    let mut all_spines = Vec::new();

    for contour in shape.iter() {
        let n = contour.len();
        if n < 3 {
            continue;
        }

        let aisle_facing = classify_face_edges(contour, boundary, 0.5);

        // Per-edge inset: aisle-facing edges move inward by
        // effective_depth, boundary/hole edges stay put.
        let ed = effective_depth - 0.05;
        let distances: Vec<f64> = (0..n)
            .map(|i| if aisle_facing[i] { ed } else { 0.0 })
            .collect();

        // Inset with edge-collapse detection (shared with inset.rs).
        let (inset_verts, live, normals) =
            inset_per_edge_core(contour, &distances, Some(outer_sign));
        if live.is_empty() {
            continue;
        }

        for idx in 0..live.len() {
            let i = live[idx];
            let j = live[(idx + 1) % live.len()];

            // Only aisle-facing edges produce spines.
            if !aisle_facing[i] {
                continue;
            }

            let spine_start = inset_verts[i];
            let spine_end = inset_verts[j];

            // Outward normal: toward the corridor (opposite of inward).
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
// Top-level orchestration
// ---------------------------------------------------------------------------

/// Build deduplicated corridor polygons for overlap checks.
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

    // Build per-directed-edge corridor polygons for the renderer.
    let aisle_polygons: Vec<Vec<Vec2>> = graph
        .edges
        .iter()
        .map(|e| corridor_polygon(&graph.vertices, e))
        .collect();

    // Build deduplicated corridor polygons for face extraction.
    let dedup_corridors = deduplicate_corridors(graph);

    // Extract positive-space faces: boundary MINUS corridors MINUS holes.
    let faces = extract_faces(boundary, &dedup_corridors);

    let mut all_spines = Vec::new();
    let mut all_stalls = Vec::new();
    let mut all_islands = Vec::new();

    let min_island_area = 10.0;

    for shape in &faces {
        let spines = compute_face_spines(shape, effective_depth, boundary);
        let stalls = place_stalls_on_spines(&spines, params);
        let islands = endcap_islands_for_face(shape, &stalls, min_island_area);

        all_spines.extend(spines);
        all_stalls.extend(stalls);
        all_islands.extend(islands);
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
}
