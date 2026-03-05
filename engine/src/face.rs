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
fn corridor_polygon(vertices: &[Vec2], edge: &AisleEdge) -> Vec<Vec2> {
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

/// Perpendicular distance from point `p` to the infinite line through `a` and `b`.
fn point_dist_to_line(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let ab = b - a;
    let len = ab.length();
    if len < 1e-12 {
        return (p - a).length();
    }
    let cross = (p.x - a.x) * ab.y - (p.y - a.y) * ab.x;
    cross.abs() / len
}

/// Classify each edge of a face contour as aisle-facing or not.
/// An edge is aisle-facing if both endpoints are within `tolerance` of some
/// corridor edge AND their directions are nearly parallel (|dot| > 0.99).
fn classify_face_edges(
    contour: &[Vec2],
    corridor_edges: &[(Vec2, Vec2)],
    tolerance: f64,
) -> Vec<bool> {
    let n = contour.len();
    let mut aisle_facing = vec![false; n];

    for i in 0..n {
        let j = (i + 1) % n;
        let fa = contour[i];
        let fb = contour[j];
        let face_dir = (fb - fa).normalize();

        for &(ca, cb) in corridor_edges {
            let corr_dir = (cb - ca).normalize();
            // Check directions are parallel.
            let dot = face_dir.dot(corr_dir).abs();
            if dot < 0.99 {
                continue;
            }
            // Check both face edge endpoints are close to the corridor line.
            let d_a = point_dist_to_line(fa, ca, cb);
            let d_b = point_dist_to_line(fb, ca, cb);
            if d_a < tolerance && d_b < tolerance {
                aisle_facing[i] = true;
                break;
            }
        }
    }

    aisle_facing
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

/// Generate spine segments for a face shape (outer contour + holes)
/// using a polygon-inset approach.
///
/// For each contour, aisle-facing edges are inset by `effective_depth`
/// while non-aisle-facing edges stay at 0.  Miter intersections at
/// vertices naturally handle corners, acute angles, and degenerate
/// (too-narrow) faces.  The resulting inset edges corresponding to
/// aisle-facing originals are the spine segments.
fn compute_face_spines(
    shape: &[Vec<Vec2>],
    corridor_edges: &[(Vec2, Vec2)],
    effective_depth: f64,
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

        let aisle_facing = classify_face_edges(contour, corridor_edges, 0.5);

        // Per-edge inset distances.  The small epsilon keeps spines
        // slightly inside the face boundary, avoiding ambiguous
        // point-in-polygon results for points exactly on an edge.
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

        // Extract spine segments from surviving aisle-facing edges.
        for idx in 0..live.len() {
            let i = live[idx];
            let j = live[(idx + 1) % live.len()];
            if !aisle_facing[i] {
                continue;
            }

            let spine_start = inset_verts[i];
            let spine_end = inset_verts[j];

            if (spine_end - spine_start).length() < 1.0 {
                continue;
            }

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
// End-cap islands at acute spine corners
// ---------------------------------------------------------------------------

fn generate_endcap_islands(segments: &[SpineSegment]) -> Vec<Island> {
    let mut islands = Vec::new();

    if segments.len() < 2 {
        return islands;
    }

    for i in 0..segments.len() {
        for j in (i + 1)..segments.len() {
            let s1 = &segments[i];
            let s2 = &segments[j];

            // Check if any endpoints are close.
            let pairs = [
                (s1.end, s2.start),
                (s1.start, s2.end),
                (s1.end, s2.end),
                (s1.start, s2.start),
            ];

            for (p1, p2) in &pairs {
                let gap = (*p1 - *p2).length();
                if gap > 2.0 {
                    continue;
                }

                let d1 = (s1.end - s1.start).normalize();
                let d2 = (s2.end - s2.start).normalize();
                let angle = d1.dot(d2).abs().acos();

                // Acute corner: angle between directions > 120° (turn < 60°).
                if angle > std::f64::consts::PI * 2.0 / 3.0 {
                    let corner = Vec2::new(
                        (p1.x + p2.x) / 2.0,
                        (p1.y + p2.y) / 2.0,
                    );
                    let tip1 = corner + s1.outward_normal * 5.0;
                    let tip2 = corner + s2.outward_normal * 5.0;
                    islands.push(Island {
                        polygon: vec![corner, tip1, tip2],
                        kind: IslandKind::EndCap,
                    });
                }
            }
        }
    }

    islands
}

// ---------------------------------------------------------------------------
// Top-level orchestration
// ---------------------------------------------------------------------------

/// Build deduplicated corridor polygons for overlap checks.
fn deduplicate_corridors(graph: &DriveAisleGraph) -> Vec<Vec<Vec2>> {
    let mut seen: Vec<(usize, usize)> = Vec::new();
    let mut corridors = Vec::new();
    for edge in &graph.edges {
        let key = if edge.start < edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        if seen.contains(&key) {
            continue;
        }
        seen.push(key);
        corridors.push(corridor_polygon(&graph.vertices, edge));
    }
    corridors
}

/// Generate stalls from positive-space face extraction + per-edge spine shifting.
/// Returns (stalls, spine_islands, corridor_polygons, spine_lines, faces).
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

    // Build flat corridor edges list for classification.
    let corridor_edges: Vec<(Vec2, Vec2)> = dedup_corridors
        .iter()
        .flat_map(|quad| {
            let n = quad.len();
            (0..n).map(move |i| (quad[i], quad[(i + 1) % n]))
        })
        .collect();

    let mut all_spines = Vec::new();

    for shape in &faces {
        let spines = compute_face_spines(shape, &corridor_edges, effective_depth);
        all_spines.extend(spines);
    }

    let endcap_islands = generate_endcap_islands(&all_spines);

    let mut all_stalls = Vec::new();

    for seg in &all_spines {
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

        let stalls = fill_strip(start, end, 1.0, 0.0, params);
        all_stalls.extend(stalls);
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

    (all_stalls, endcap_islands, aisle_polygons, spine_lines, face_list)
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

        let (stalls, _, _, spines, _) = generate_from_spines(&graph, &boundary, &params);
        eprintln!("\nTotal stalls: {}", stalls.len());
        eprintln!("Total spines: {}", spines.len());
        for (i, s) in spines.iter().enumerate() {
            let len = (s.end - s.start).length();
            eprintln!(
                "  spine {}: ({:.0},{:.0})->({:.0},{:.0}) len={:.0} n=({:.1},{:.1})",
                i, s.start.x, s.start.y, s.end.x, s.end.y, len, s.normal.x, s.normal.y
            );
        }
        assert!(stalls.len() > 50);
    }
}
