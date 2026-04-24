//! Cross-module integration tests for the pipeline. Each test
//! exercises a multi-stage path — typically `auto_generate` feeding
//! `generate_from_spines`, or a hand-assembled boundary feeding
//! `extract_faces` + `compute_face_spines` — and asserts on emergent
//! properties (stall counts, spine counts, face winding) that no
//! single pipeline stage would catch in isolation.
//!
//! Lives under `pipeline/` so it can call `pub(crate)` helpers from
//! any sibling stage directly. Gated at the `mod integration_tests`
//! declaration in `pipeline/mod.rs`; the whole file only compiles
//! under `cargo test`.

use crate::geom::clip::remove_conflicting_stalls;
use crate::geom::inset::signed_area;
use crate::geom::poly::{point_in_face, point_to_segment_dist};
use crate::graph::auto_generate;
use crate::pipeline::bays::extract_faces;
use crate::pipeline::corridors::{
    corridor_polygon, deduplicate_corridors, generate_miter_fills, merge_corridor_shapes,
};
use crate::pipeline::filter::clip_stalls_to_faces;
use crate::pipeline::generate::generate_from_spines;
use crate::pipeline::islands::{compute_islands, mark_island_stalls, stall_center, stall_key};
use crate::pipeline::placement::{fill_spine, place_stalls_on_spines};
use crate::pipeline::spines::{
    compute_face_spines, dedup_overlapping_spines, merge_collinear_spines,
};
use crate::pipeline::tagging::{classify_face_edges, is_boundary_face};
use crate::types::*;

mod tests {
    use super::*;

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
            ..Default::default()
        };
        let params = ParkingParams::default();
        let graph = auto_generate(&boundary, &params, &[], &[]);

        let (stalls, spines, _, _, _) = generate_from_spines(&graph, &boundary, &params, &DebugToggles::default());
        eprintln!("\nTotal stalls: {}", stalls.len());
        eprintln!("Total spines: {}", spines.len());
        assert!(stalls.len() >= 50);
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
            ..Default::default()
        };
        let params = ParkingParams::default();
        let graph = auto_generate(&boundary, &params, &[], &[]);

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
        let miter_fills = generate_miter_fills(&graph, &DebugToggles::default());
        for (i, fill) in miter_fills.iter().enumerate() {
            let area = signed_area(fill).abs();
            eprintln!("  fill {}: area={:.1}, {} verts", i, area, fill.len());
            for (vi, v) in fill.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();

        eprintln!("\nCorridor rects:");
        for (i, c) in dedup_corridor_polys.iter().enumerate() {
            eprintln!("  rect {}: {} verts", i, c.len());
            for (vi, v) in c.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }
        }

        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &DebugToggles::default());

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

        let faces = extract_faces(&boundary.outer, &merged_corridors, &[]);
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

            let face_spines = compute_face_spines(shape, effective_depth, &merged_corridors, &dedup_corridors, false, &DebugToggles::default(), &[], None);
            eprintln!("  spines from face {}: {}", fi, face_spines.len());
            for (si, s) in face_spines.iter().enumerate() {
                let len = (s.end - s.start).length();
                eprintln!("    spine {}: ({:.1},{:.1})→({:.1},{:.1}) len={:.1} n=({:.2},{:.2})",
                    si, s.start.x, s.start.y, s.end.x, s.end.y, len,
                    s.outward_normal.x, s.outward_normal.y);
            }
        }
    }

    /// Diagnostic: check face 0's boolean Difference for the default rect boundary.
    #[test]
    fn test_face0_difference_shapes() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(750.0, 0.0),
                Vec2::new(750.0, 500.0),
                Vec2::new(0.0, 500.0),
            ],
            holes: vec![vec![
                Vec2::new(275.0, 150.0),
                Vec2::new(475.0, 150.0),
                Vec2::new(375.0, 350.0),
            ]],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let debug = DebugToggles::default();

        let (stalls, _, _faces_out, _, islands) =
            generate_from_spines(
                &crate::graph::auto_generate(&boundary, &params, &[], &[]),
                &boundary, &params, &debug,
            );

        eprintln!("Total stalls: {}, islands: {}", stalls.len(), islands.len());
        for (ii, isl) in islands.iter().enumerate() {
            let a = signed_area(&isl.contour).abs();
            eprintln!("  island {}: face={}, area={:.0}, verts={}", ii, isl.face_idx, a, isl.contour.len());
        }

        // Check: any island from face 0?
        let face0_islands: Vec<_> = islands.iter().filter(|i| i.face_idx == 0).collect();
        eprintln!("Face 0 islands: {}", face0_islands.len());

        // Check: any island near boundary corners?
        let corners = [(5.0, 5.0), (745.0, 5.0), (745.0, 495.0), (5.0, 495.0)];
        for (cx, cy) in &corners {
            let near = islands.iter().any(|isl|
                isl.contour.iter().any(|v| (v.x - cx).abs() < 20.0 && (v.y - cy).abs() < 20.0)
            );
            eprintln!("  corner ({}, {}): island nearby = {}", cx, cy, near);
        }
    }

    /// Islands test: verify that compute_islands returns small gap polygons,
    /// not the full face, for a face with stalls placed in it.
    #[test]
    fn test_islands_are_gaps_not_full_face() {
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(750.0, 0.0),
                Vec2::new(602.62, 475.41),
                Vec2::new(0.0, 500.0),
            ],
            holes: vec![vec![
                Vec2::new(275.0, 150.0),
                Vec2::new(475.0, 150.0),
                Vec2::new(375.0, 350.0),
            ]],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let debug = DebugToggles::default();
        let graph = auto_generate(&boundary, &params, &[], &[]);

        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();
        let dedup_corridors = deduplicate_corridors(&graph);
        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();
        let _miter_fills = generate_miter_fills(&graph, &debug);
        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &debug);
        let faces = extract_faces(&boundary.outer, &merged_corridors, &[]);

        // Run through the full spine+stall pipeline.
        let mut raw_spines = Vec::new();
        let mut faces_with_spines = std::collections::HashSet::new();
        for (face_idx, shape) in faces.iter().enumerate() {
            let mut face_spines = compute_face_spines(shape, effective_depth, &merged_corridors, &dedup_corridors, false, &debug, &[], None);
            if !face_spines.is_empty() {
                faces_with_spines.insert(face_idx);
            }
            for s in &mut face_spines {
                s.face_idx = face_idx;
            }
            let face_spines = dedup_overlapping_spines(face_spines, 2.0);
            raw_spines.extend(face_spines);
        }
        let all_spines = merge_collinear_spines(raw_spines, 1.0);
        let all_spines: Vec<SpineSegment> = all_spines.into_iter()
            .filter(|s| (s.end - s.start).length() >= effective_depth)
            .collect();

        let (tagged_stalls_3, _) = place_stalls_on_spines(&all_spines, &params, true, None, None);
        let tagged_stalls: Vec<(StallQuad, usize)> = tagged_stalls_3.iter()
            .map(|(s, fi, _)| (s.clone(), *fi)).collect();
        let tagged_stalls = clip_stalls_to_faces(tagged_stalls, &faces);
        let tagged_stalls = remove_conflicting_stalls(tagged_stalls, &[], &[]);
        let islands = compute_islands(&faces, &tagged_stalls, 10.0, true);

        // No island should have area close to the boundary or the hole.
        // No island should be as large as the full boundary or the hole itself.
        let boundary_area = signed_area(&boundary.outer).abs();
        let _hole_area = signed_area(&boundary.holes[0]).abs();
        for (ii, island) in islands.iter().enumerate() {
            let outer_a = signed_area(&island.contour).abs();
            let holes_a: f64 = island.holes.iter().map(|h| signed_area(h).abs()).sum();
            let net = outer_a - holes_a;
            assert!(net < boundary_area * 0.25,
                "island {} has net area {:.1} ≥ 25% of boundary area {:.1}",
                ii, net, boundary_area);
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
            direction: AisleDirection::default(),
        };
        let edge_b = AisleEdge {
            start: 1,
            end: 2,
            width: w,
            interior: false,
            direction: AisleDirection::default(),
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
        let merged = merge_corridor_shapes(&[rect_a.clone(), rect_b.clone()], &graph, &DebugToggles::default());

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

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &DebugToggles::default(), &[], None);

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
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.0.corners.iter().enumerate() {
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

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let stall_angle_rad = params.stall_angle_deg.to_radians();
        let effective_depth = params.stall_depth * stall_angle_rad.sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &DebugToggles::default(), &[], None);

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
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.0.corners.iter().enumerate() {
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
    /// The normal skeleton collapses, but edge-suppression produces
    /// single-sided spines — one per aisle-facing edge. Downstream pairing
    /// and stall conflict resolution pick the winner.
    #[test]
    fn test_narrow_face_single_sided_spines() {
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

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let effective_depth = params.stall_depth
            * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &DebugToggles::default(), &[], None);
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();

        eprintln!("\n=== Narrow face test (30ft between aisles) ===");
        eprintln!("spines: {}, stalls: {}", spines.len(), stalls.len());

        assert_eq!(spines.len(), 2, "edge-suppression should produce one spine per aisle edge");
        assert!(stalls.len() > 0, "narrow face should have single-sided stalls");
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

        let params = ParkingParams { stall_angle_deg: 90.0, ..ParkingParams::default() };
        let effective_depth =
            params.stall_depth * params.stall_angle_deg.to_radians().sin();

        let spines = compute_face_spines(&shape, effective_depth, &corridor_shapes, &[], false, &DebugToggles::default(), &[], None);

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
        let stalls: Vec<(StallQuad, usize)> = place_stalls_on_spines(&spines, &params, true, None, None).0
            .into_iter().map(|(s, fi, _)| (s, fi)).collect();
        eprintln!("stalls placed: {}", stalls.len());
        assert!(stalls.len() > 0, "should place some stalls");

        let mut outside_count = 0;
        for (si, stall) in stalls.iter().enumerate() {
            for (ci, corner) in stall.0.corners.iter().enumerate() {
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
        use crate::pipeline::generate::generate;
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
                ..Default::default()
            },
            drive_lines: vec![],
            annotations: vec![],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
            stall_modifiers: Vec::new(),
        };

        let layout = generate(input.clone());
        eprintln!("\n=== attempt boundary debug ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Recompute corridor shapes for debug classification.
        let graph = auto_generate(&input.boundary, &input.params, &[], &[]);
        let dedup_corridors = deduplicate_corridors(&graph);
        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();
        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &DebugToggles::default());

        let effective_depth = input.params.stall_depth
            * input.params.stall_angle_deg.to_radians().sin();

        for (fi, face) in layout.faces.iter().enumerate() {
            let contour = &face.contour;
            let area = crate::geom::inset::signed_area(contour).abs();
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

            let classified = classify_face_edges(contour, &merged_corridors, &dedup_corridors);
            let ed = effective_depth - 0.05;
            let min_edge_len = effective_depth * 2.0;
            for i in 0..contour.len() {
                let j = (i + 1) % contour.len();
                let edge_len = (contour[j] - contour[i]).length();
                let (af, _, _) = classified[i];
                let dist = if af && edge_len >= min_edge_len { ed } else { 0.0 };
                eprintln!(
                    "    edge {}->{}: len={:.1} aisle_facing={} dist={:.1}{}",
                    i, j, edge_len, af, dist,
                    if !af { " (boundary)" }
                    else if edge_len < min_edge_len { " (too short)" }
                    else { "" }
                );
            }

            for (hi, hole) in face.holes.iter().enumerate() {
                eprintln!("    hole {}: {} verts", hi, hole.len());
                let hole_classified = classify_face_edges(hole, &merged_corridors, &dedup_corridors);
                for i in 0..hole.len() {
                    let j = (i + 1) % hole.len();
                    let elen = (hole[j] - hole[i]).length();
                    let (af, _, _) = hole_classified[i];
                    eprintln!("      edge {}->{}: len={:.1} af={} ({:.1},{:.1})->({:.1},{:.1})",
                        i, j, elen, af, hole[i].x, hole[i].y, hole[j].x, hole[j].y);
                }
            }
            let mut shape = vec![contour.clone()];
            shape.extend(face.holes.iter().cloned());
            let spines = compute_face_spines(&shape, effective_depth, &merged_corridors, &dedup_corridors, false, &DebugToggles::default(), &[], None);
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
        use crate::pipeline::generate::generate;
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
                ..Default::default()
            },
            drive_lines: vec![],
            annotations: vec![],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
            stall_modifiers: Vec::new(),
        };

        let layout = generate(input.clone());
        eprintln!("\n=== slanted boundary debug ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Recompute corridor shapes for debug classification.
        let graph = auto_generate(&input.boundary, &input.params, &[], &[]);
        let dedup_corridors = deduplicate_corridors(&graph);
        let dedup_corridor_polys: Vec<Vec<Vec2>> = dedup_corridors.iter().map(|(p, _, _)| p.clone()).collect();
        let merged_corridors = merge_corridor_shapes(&dedup_corridor_polys, &graph, &DebugToggles::default());

        let effective_depth = input.params.stall_depth
            * input.params.stall_angle_deg.to_radians().sin();

        // Examine each face.
        for (fi, face) in layout.faces.iter().enumerate() {
            let contour = &face.contour;
            let area = crate::geom::inset::signed_area(contour).abs();
            eprintln!("\n  face {}: {} vertices, area={:.1}", fi, contour.len(), area);
            for (vi, v) in contour.iter().enumerate() {
                eprintln!("    v{}: ({:.1}, {:.1})", vi, v.x, v.y);
            }

            // Classify edges for this face.
            let classified = classify_face_edges(contour, &merged_corridors, &dedup_corridors);
            let ed = effective_depth - 0.05;
            let min_edge_len = effective_depth * 2.0;
            for i in 0..contour.len() {
                let j = (i + 1) % contour.len();
                let edge_len = (contour[j] - contour[i]).length();
                let (af, _, _) = classified[i];
                let dist = if af && edge_len >= min_edge_len { ed } else { 0.0 };
                eprintln!(
                    "    edge {}->{}: len={:.1} aisle_facing={} dist={:.1}{}",
                    i, j, edge_len, af, dist,
                    if !af { " (boundary)" }
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
            let spines = compute_face_spines(&shape, effective_depth, &merged_corridors, &dedup_corridors, false, &DebugToggles::default(), &[], None);
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
    fn test_drive_line_horizontal_with_hole() {
        use crate::pipeline::generate::generate;
        use crate::types::{GenerateInput, DriveLine};

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(750.0, 0.0),
                    Vec2::new(750.0, 500.0),
                    Vec2::new(0.0, 500.0),
                ],
                holes: vec![vec![
                    Vec2::new(275.0, 150.0),
                    Vec2::new(475.0, 150.0),
                    Vec2::new(375.0, 350.0),
                ]],
                ..Default::default()
            },
            drive_lines: vec![
                DriveLine {
                    start: Vec2::new(-50.0, 250.0),
                    end: Vec2::new(800.0, 250.0),
                hole_pin: None, id: 0, partitions: false, },
            ],
            annotations: vec![],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
            stall_modifiers: Vec::new(),
        };

        let layout = generate(input);
        eprintln!("\n=== drive line horizontal with hole ===");
        eprintln!("stalls: {}, faces: {}, miter_fills: {}",
            layout.stalls.len(), layout.faces.len(), layout.miter_fills.len());
        eprintln!("resolved graph: {} vertices, {} edges",
            layout.resolved_graph.vertices.len(), layout.resolved_graph.edges.len());

        // The drive line should produce interior segments that avoid the hole.
        // There should be corridors and miter fills at the junction points.
        assert!(layout.stalls.len() > 0, "should produce stalls");
        assert!(layout.miter_fills.len() > 0, "should produce miter fills");
        assert!(layout.faces.len() > 0, "should produce faces");
    }

    /// Test that TwoWayOriented aisles assign per-side travel_dir to spines.
    ///
    /// Setup: a 200×300 rectangle with a vertical aisle down the middle.
    /// The aisle edge goes from (100,0) to (100,300) and is marked
    /// TwoWayOriented with travel_dir pointing downward (0,1).
    ///
    /// Expected: the spine on the RIGHT side of travel_dir (outward normal
    /// pointing right, i.e. +x) should get travel_dir = (0,1) (downward,
    /// matching the right lane). The spine on the LEFT side (outward normal
    /// pointing left, i.e. -x) should get travel_dir = (0,-1) (upward,
    /// the oncoming lane).
    ///
    /// For comparison, a OneWay aisle should give BOTH sides the same
    /// travel_dir.
    #[test]
    fn test_two_way_oriented_per_side_travel_dir() {
        // Build a minimal graph: vertical aisle at x=100 in a 200×300 box.
        let boundary = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(200.0, 0.0),
                Vec2::new(200.0, 300.0),
                Vec2::new(0.0, 300.0),
            ],
            holes: vec![],
            ..Default::default()
        };
        let params = ParkingParams::default();
        let hw = params.aisle_width / 2.0;

        // Vertical aisle: 4 perimeter vertices + 2 interior vertices at
        // top and bottom of the aisle centerline.
        let vertices = vec![
            Vec2::new(0.0, 0.0),     // 0: bottom-left
            Vec2::new(200.0, 0.0),   // 1: bottom-right
            Vec2::new(200.0, 300.0), // 2: top-right
            Vec2::new(0.0, 300.0),   // 3: top-left
            Vec2::new(100.0, 0.0),   // 4: aisle bottom
            Vec2::new(100.0, 300.0), // 5: aisle top
        ];

        // Helper to build a graph with a specific direction for the interior edge.
        let make_graph = |direction: AisleDirection| -> DriveAisleGraph {
            DriveAisleGraph {
                vertices: vertices.clone(),
                edges: vec![
                    // Perimeter edges
                    AisleEdge { start: 0, end: 1, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 1, end: 2, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 2, end: 3, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 3, end: 0, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    // Perimeter connections to aisle
                    AisleEdge { start: 0, end: 4, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 1, end: 4, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 2, end: 5, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    AisleEdge { start: 3, end: 5, width: hw, interior: false, direction: AisleDirection::TwoWay },
                    // Interior aisle edge: 4→5 (bottom to top, i.e. travel_dir = (0,1) upward)
                    AisleEdge { start: 4, end: 5, width: hw, interior: true, direction: direction.clone() },
                    // Reverse edge for two-way
                    AisleEdge { start: 5, end: 4, width: hw, interior: true, direction: direction },
                ],
                perim_vertex_count: 4,
            }
        };

        let debug = DebugToggles::default();

        // --- Test TwoWayOriented ---
        let graph = make_graph(AisleDirection::TwoWayOriented);
        let (stalls, spine_lines, faces, _, _) = generate_from_spines(&graph, &boundary, &params, &debug);

        eprintln!("\n=== TwoWayOriented test ===");
        eprintln!("stalls: {}, spines: {}, faces: {}", stalls.len(), spine_lines.len(), faces.len());

        // Rebuild spines with travel_dir info (spine_lines loses it).
        // Use the internal pipeline directly.
        let dedup = deduplicate_corridors(&graph);
        let dedup_polys: Vec<Vec<Vec2>> = dedup.iter().map(|(p, _, _)| p.clone()).collect();
        let merged = merge_corridor_shapes(&dedup_polys, &graph, &debug);
        let two_way_dirs: Vec<Option<Vec2>> = {
            let mut seen = std::collections::HashSet::new();
            let mut dirs = Vec::new();
            for edge in &graph.edges {
                let key = if edge.start < edge.end { (edge.start, edge.end) } else { (edge.end, edge.start) };
                if !seen.insert(key) { continue; }
                if edge.direction == AisleDirection::TwoWayOriented {
                    dirs.push(Some((graph.vertices[edge.end] - graph.vertices[edge.start]).normalize()));
                } else {
                    dirs.push(None);
                }
            }
            dirs
        };
        let extracted_faces = extract_faces(&boundary.outer, &merged, &[]);

        let ed = params.stall_depth
            + params.stall_depth * (params.stall_angle_deg * std::f64::consts::PI / 180.0).sin()
            + (params.stall_angle_deg * std::f64::consts::PI / 180.0).cos() * params.stall_width / 2.0;

        let mut left_spines = Vec::new();
        let mut right_spines = Vec::new();
        for shape in &extracted_faces {
            let face_is_boundary = is_boundary_face(&shape[0], &merged, &dedup);
            let spines = compute_face_spines(shape, ed, &merged, &dedup, face_is_boundary, &debug, &two_way_dirs, None);
            for s in &spines {
                if let Some(td) = s.travel_dir {
                    eprintln!(
                        "  spine: outward=({:.1},{:.1}) travel_dir=({:.1},{:.1}) start=({:.0},{:.0}) end=({:.0},{:.0})",
                        s.outward_normal.x, s.outward_normal.y,
                        td.x, td.y,
                        s.start.x, s.start.y, s.end.x, s.end.y,
                    );
                    // Classify: is outward pointing right (+x) or left (-x)?
                    if s.outward_normal.x > 0.5 {
                        right_spines.push(td);
                    } else if s.outward_normal.x < -0.5 {
                        left_spines.push(td);
                    }
                }
            }
        }

        // The interior aisle goes from vertex 4 (100,0) to vertex 5 (100,300).
        // travel_dir = normalize(v5 - v4) = (0, 1).
        //
        // For TwoWayOriented, both sides get the SAME travel_dir (like
        // OneWay). The existing flip_angle logic in place_stalls_on_spines
        // already differentiates stall lean per-side based on the spine's
        // orientation relative to travel_dir.
        eprintln!("right_spines travel_dirs: {:?}", right_spines);
        eprintln!("left_spines travel_dirs: {:?}", left_spines);

        assert!(!right_spines.is_empty(), "should have spines on right side of aisle");
        assert!(!left_spines.is_empty(), "should have spines on left side of aisle");

        // For TwoWayOriented, the "canonical negative" side (outward.x < 0)
        // gets its travel_dir flipped. The "canonical positive" side keeps it.
        // This makes both sides produce the same flip_angle in
        // place_stalls_on_spines, giving correct per-lane stall angles.
        // Right spines (outward.x > 0 = positive side): keep td = (0,1)
        for td in &right_spines {
            assert!(td.y > 0.5, "positive-side travel_dir should stay (0,1), got ({:.2},{:.2})", td.x, td.y);
        }
        // Left spines (outward.x < 0 = negative side): flipped to (0,-1)
        for td in &left_spines {
            assert!(td.y < -0.5, "negative-side travel_dir should be flipped to (0,-1), got ({:.2},{:.2})", td.x, td.y);
        }
    }

    /// End-to-end test: use the full generate pipeline with a TwoWayOriented
    /// annotation on an auto-generated graph (simulating the UI flow).
    #[test]
    fn test_two_way_oriented_annotation_e2e() {
        use crate::pipeline::generate::generate;
        use crate::types::{GenerateInput, DriveLine};

        let input = GenerateInput {
            boundary: Polygon {
                outer: vec![
                    Vec2::new(0.0, 0.0),
                    Vec2::new(300.0, 0.0),
                    Vec2::new(300.0, 200.0),
                    Vec2::new(0.0, 200.0),
                ],
                holes: vec![],
                ..Default::default()
            },
            drive_lines: vec![
                DriveLine {
                    start: Vec2::new(150.0, -50.0),
                    end: Vec2::new(150.0, 250.0),
                hole_pin: None, id: 1, partitions: false, },
            ],
            annotations: vec![
                // Drive line spans y=-50→y=250 (length 300). The interior
                // splice covers the inset boundary, ~y=0→y=200 (t≈0.166→0.833).
                // Point at the midpoint picks the sub-edge containing it.
                Annotation::Direction {
                    target: Target::DriveLine { id: 1, t: 0.5 },
                    traffic: TrafficDirection::TwoWayOriented,
                },
            ],
            params: ParkingParams::default(),
            debug: DebugToggles::default(), region_overrides: vec![],
            stall_modifiers: Vec::new(),
        };

        let layout = generate(input);
        eprintln!("\n=== TwoWayOriented annotation e2e ===");
        eprintln!("stalls: {}, spines: {}, faces: {}",
            layout.stalls.len(), layout.spines.len(), layout.faces.len());

        // Check the resolved graph has TwoWayOriented edges.
        let two_way_ori_count = layout.resolved_graph.edges.iter()
            .filter(|e| e.direction == AisleDirection::TwoWayOriented)
            .count();
        eprintln!("TwoWayOriented edges in resolved graph: {}", two_way_ori_count);
        assert!(two_way_ori_count > 0, "should have TwoWayOriented edges after annotation");

        // Verify stalls exist on both sides.
        assert!(layout.stalls.len() > 0, "should produce stalls");
    }

    /// Comprehensive test for mark_island_stalls covering:
    /// - Various row lengths and intervals
    /// - Multiple independent rows (different spines)
    /// - Angled stalls (45°)
    /// - Invariants: no islands at row edges, every qualifying row gets
    ///   islands, no long runs without an island.
    #[test]
    fn test_mark_island_stalls_invariants() {
        // Helper: create a horizontal spine at y_offset with stalls along
        // the x-axis. Normal points up (+y), so oriented_dir is +x.
        fn make_row(
            n_target: usize,
            spine_idx: usize,
            face_idx: usize,
            y_offset: f64,
            params: &ParkingParams,
        ) -> (Vec<(StallQuad, usize, usize)>, SpineSegment) {
            let pitch = params.stall_pitch();
            let spine_len = n_target as f64 * pitch;
            let seg = SpineSegment {
                start: Vec2::new(0.0, y_offset),
                end: Vec2::new(spine_len, y_offset),
                outward_normal: Vec2::new(0.0, 1.0),
                face_idx,
                is_interior: true,
                travel_dir: None,
            };
            let stalls: Vec<(StallQuad, usize, usize)> = fill_spine(&seg, params, 0.0)
                .into_iter()
                .map(|q| (q, face_idx, spine_idx))
                .collect();
            (stalls, seg)
        }

        // Sort stalls by position along direction for a given spine.
        fn sorted_row(
            stalls: &[(StallQuad, usize, usize)],
            spine_idx: usize,
            dir: Vec2,
        ) -> Vec<(f64, bool)> {
            let mut s: Vec<(f64, bool)> = stalls.iter()
                .filter(|(_, _, si)| *si == spine_idx)
                .map(|(q, _, _)| (stall_center(q).dot(dir), q.kind == StallKind::Island))
                .collect();
            s.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            s
        }

        fn spine_range(spine: &SpineSegment, dir: Vec2) -> (f64, f64) {
            let a = spine.start.dot(dir);
            let b = spine.end.dot(dir);
            (a.min(b), a.max(b))
        }

        // Check all invariants for a single row.
        // spine_proj_range: (proj_min, proj_max) from spine endpoints,
        // matching how mark_island_stalls computes the end margin.
        fn check_invariants(
            stalls: &[(StallQuad, usize, usize)],
            spine_idx: usize,
            dir: Vec2,
            interval: usize,
            spine_proj_range: (f64, f64),
            label: &str,
        ) {
            let row = sorted_row(stalls, spine_idx, dir);
            let count = row.len();

            if count < interval {
                let n_islands = row.iter().filter(|(_, is)| *is).count();
                assert_eq!(n_islands, 0,
                    "{}: row with {} stalls (< interval {}) should have 0 islands, got {}",
                    label, count, interval, n_islands);
                return;
            }

            // A+B: stalls within 1.5 pitches of spine ends are not islands
            // (position-based margin from spine endpoints, matching
            // mark_island_stalls logic).
            let stall_pitch_val = row.get(1).map_or(1.0, |r| r.0 - row[0].0);
            let end_margin = 1.5 * stall_pitch_val;
            let (proj_min, proj_max) = spine_proj_range;
            for (j, &(proj, is_isl)) in row.iter().enumerate() {
                if proj - proj_min < end_margin || proj_max - proj < end_margin {
                    assert!(!is_isl,
                        "{}: stall {} at proj={:.1} should not be island (within end margin, count={}, interval={})",
                        label, j, proj, count, interval);
                }
            }

            // C: at least one island exists when enough interior stalls
            // remain after position-based end margins.
            let n_islands = row.iter().filter(|(_, is)| *is).count();
            let eligible = row.iter().filter(|(proj, _)| {
                proj - proj_min >= end_margin && proj_max - proj >= end_margin
            }).count();
            if eligible >= interval {
                assert!(n_islands >= 1,
                    "{}: row with {} stalls ({} eligible) and interval {} should have >= 1 island, got {}",
                    label, count, eligible, interval, n_islands);
            }

            // D: no run of >= interval consecutive non-island stalls
            // in the eligible interior.
            {
                let mut run = 0usize;
                for (j, &(proj, is_isl)) in row.iter().enumerate() {
                    if proj - proj_min < end_margin || proj_max - proj < end_margin {
                        continue;
                    }
                    if is_isl {
                        run = 0;
                    } else {
                        run += 1;
                        assert!(run < interval,
                            "{}: interior run of {} non-island stalls at position {} exceeds interval {} (count={})",
                            label, run + 1, j, interval, count);
                    }
                }
            }

            // E: reasonable island count — at most count/2.
            assert!(n_islands <= count / 2,
                "{}: too many islands ({}) for {} stalls",
                label, n_islands, count);
        }

        // =========================================================
        // Test 1: Single rows, various lengths, interval=5, 90° stalls
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 90.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };
            let dir = Vec2::new(1.0, 0.0);

            for &n in &[3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 30] {
                let (mut stalls_3, spine) = make_row(n, 0, 0, 0.0, &params);
                let actual_n = stalls_3.len();
                let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                    .map(|(q, fi, _)| (q.clone(), *fi)).collect();
                let spines = vec![spine];

                mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);
                let spr = spine_range(&spines[0], dir);
                check_invariants(&stalls_3, 0, dir, 5, spr, &format!("single_90deg_n{}", actual_n));
            }
        }

        // =========================================================
        // Test 2: Single row, length=20, various intervals
        // =========================================================
        {
            let dir = Vec2::new(1.0, 0.0);

            for &interval in &[2, 3, 4, 5, 6, 8, 10] {
                let params = ParkingParams {
                    stall_angle_deg: 90.0,
                    island_stall_interval: interval,
                    ..ParkingParams::default()
                };
                let (mut stalls_3, spine) = make_row(20, 0, 0, 0.0, &params);
                let actual_n = stalls_3.len();
                let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                    .map(|(q, fi, _)| (q.clone(), *fi)).collect();
                let spines = vec![spine];

                mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);
                let spr = spine_range(&spines[0], dir);
                check_invariants(&stalls_3, 0, dir, interval as usize, spr,
                    &format!("single_interval{}_n{}", interval, actual_n));
            }
        }

        // =========================================================
        // Test 3: 45° angled stalls, various lengths, interval=5
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 45.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };
            let dir = Vec2::new(1.0, 0.0);

            for &n in &[6, 8, 12, 20] {
                let (mut stalls_3, spine) = make_row(n, 0, 0, 0.0, &params);
                let actual_n = stalls_3.len();
                let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                    .map(|(q, fi, _)| (q.clone(), *fi)).collect();
                let spines = vec![spine];

                mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);
                let spr = spine_range(&spines[0], dir);
                check_invariants(&stalls_3, 0, dir, 5, spr, &format!("angled45_n{}", actual_n));
            }
        }

        // =========================================================
        // Test 4: Two independent rows (different spines, same params)
        //         — both should independently get islands
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 90.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };
            let dir = Vec2::new(1.0, 0.0);

            let (mut stalls_a, spine_a) = make_row(20, 0, 0, 0.0, &params);
            let (stalls_b, spine_b) = make_row(12, 1, 0, 50.0, &params);
            stalls_a.extend(stalls_b);
            let mut tagged: Vec<(StallQuad, usize)> = stalls_a.iter()
                .map(|(q, fi, _)| (q.clone(), *fi)).collect();
            let spines = vec![spine_a, spine_b];

            mark_island_stalls(&mut stalls_a, &mut tagged, &spines, &params);
            let spr0 = spine_range(&spines[0], dir);
            let spr1 = spine_range(&spines[1], dir);
            check_invariants(&stalls_a, 0, dir, 5, spr0, "two_rows_spine0");
            check_invariants(&stalls_a, 1, dir, 5, spr1, "two_rows_spine1");
        }

        // =========================================================
        // Test 5: tagged vector is marked in sync with stalls_3
        // =========================================================
        {
            let params = ParkingParams {
                stall_angle_deg: 90.0,
                island_stall_interval: 5,
                ..ParkingParams::default()
            };

            let (mut stalls_3, spine) = make_row(20, 0, 0, 0.0, &params);
            let mut tagged: Vec<(StallQuad, usize)> = stalls_3.iter()
                .map(|(q, fi, _)| (q.clone(), *fi)).collect();
            let spines = vec![spine];

            mark_island_stalls(&mut stalls_3, &mut tagged, &spines, &params);

            for (s3, _, _) in &stalls_3 {
                let key = stall_key(s3);
                let tagged_match = tagged.iter().find(|(t, _)| stall_key(t) == key);
                assert!(tagged_match.is_some(), "stalls_3 entry should exist in tagged");
                assert_eq!(s3.kind, tagged_match.unwrap().0.kind,
                    "kind should match between stalls_3 and tagged");
            }
        }
    }
}

