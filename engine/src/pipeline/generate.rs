use crate::annotations::{apply_annotations, resolve_regions_for_frames, ResolvedRegion};
use crate::geom::clip::remove_conflicting_stalls;
use crate::geom::poly::{ensure_ccw, signed_area};
use crate::graph::{aisle_edge_perim, auto_generate, compute_inset_d, decompose_regions, derive_raw_holes, derive_raw_outer, intersect_line_polygon, subtract_intervals, Region};
use crate::pipeline::bays::extract_faces;
use crate::pipeline::corridors::{corridor_polygons, merge_corridor_surface};
use crate::pipeline::filter::{clip_stalls_to_faces, filter_stalls_by_entrance_coverage};
use crate::pipeline::islands::{compute_islands, mark_island_stalls, stall_key};
use crate::pipeline::placement::place_stalls_on_spines;
use crate::pipeline::spines::{
    compute_face_spines, extend_primary_with_extensions,
    extend_spines_to_faces, merge_collinear_spines,
};
use crate::pipeline::tagging::tag_face_edges;
use crate::types::*;

fn region_pole(poly: &[Vec2], holes: &[Vec<Vec2>]) -> Vec2 {
    let cleaned: Vec<Vec<Vec2>> = holes.iter().filter(|h| h.len() >= 3).cloned().collect();
    crate::geom::poly::polygon_pole(poly, &cleaned, 1.0)
}

pub fn generate(input: GenerateInput) -> ParkingLayout {
    // Discretize curved boundary edges into dense polylines so the
    // entire downstream pipeline works on straight-line segments.
    let mut input = input;
    // Ensure every sketch vertex has a stable `VertexId` before we
    // discretize — perimeter annotations address sketch edges by
    // `(start_vid, end_vid)` pairs and need the ids to be present
    // both at annotation-creation time (UI side) and at resolve time
    // (here). The UI normally pre-populates these; ensure_ids is a
    // no-op when ids are already aligned.
    input.boundary.ensure_ids();
    input.boundary = crate::geom::arc::discretize_polygon(
        &input.boundary,
        input.params.arc_discretize_tolerance,
    );

    // Partitioning drive lines contribute to the planar-arrangement
    // face enumeration (regions). During the migration we accept
    // either the new `partitions` flag or the legacy `hole_pin` as the
    // signal — either one means "this line divides space".
    let partitioning_lines: Vec<(u32, Vec2, Vec2)> = input.drive_lines.iter()
        .filter(|dl| dl.partitions || dl.hole_pin.is_some())
        .map(|dl| (dl.id, dl.start, dl.end))
        .collect();

    let mut graph = auto_generate(
        &input.boundary,
        &input.params,
        &partitioning_lines,
        &input.region_overrides,
    );

    // Append drive line edges on top — they're additive and should
    // not cause auto edges to be filtered out. Drive-line vertices
    // are placed at arbitrary world coordinates (not on the integer
    // grid), so they're naturally invisible to abstract annotation
    // lookups that only match integer grid points. Legacy proximity
    // annotations intentionally run AFTER this so they can target
    // drive-line edges.
    let (drive_graph, drive_anchors) =
        clip_drive_lines(&input.drive_lines, &input.boundary, &input.params, &graph);
    let mut splice_anchors: Vec<Option<(u32, f64)>> = vec![None; graph.vertices.len()];
    if !drive_graph.vertices.is_empty() {
        let (merged, merged_anchors) = append_graph(graph, drive_graph, drive_anchors);
        graph = merged;
        splice_anchors = merged_anchors;
    }

    // Build the per-region frame list once, for both abstract annotation
    // lookup and the debug info returned to the UI below. Decomposition is
    // cheap enough to compute here again (auto_generate also runs it
    // internally) — a future cleanup could share the result.
    let resolved_regions: Vec<ResolvedRegion> =
        resolve_regions_for_frames(&input, &partitioning_lines);

    // Apply spatial annotations to the resolved graph. Each target is
    // resolved in its own substrate's coordinate system: grid lattice
    // for interior crossings, drive-line parametric t for splices, and
    // perimeter arc length for boundary vertices / sub-edges.
    let dormant_annotations = apply_annotations(
        &mut graph, &input.annotations, &resolved_regions,
        &splice_anchors, &input.drive_lines, &input.boundary, &input.params,
    );

    let inset_d = compute_inset_d(&input.params);
    let derived_outer = derive_raw_outer(&input.boundary.outer, inset_d, input.params.site_offset);
    let derived_holes = derive_raw_holes(&input.boundary.holes, inset_d);

    let (stalls, spines, faces, islands) =
        generate_from_spines(&graph, &input.boundary, &input.params, &input.debug, &input.stall_modifiers);

    let mut stalls = stalls;

    // §1.4 stall modifiers post-pass: retype overlapping stalls. The
    // match radius scales with stall footprint so a polyline drawn over
    // a row of stalls reliably catches them without forcing users to
    // draw pixel-perfect lines.
    if !input.stall_modifiers.is_empty() {
        let radius = input.params.stall_depth * 0.5;
        crate::pipeline::filter::apply_stall_modifiers(&mut stalls, &input.stall_modifiers, radius, &input.params);
    }

    // Suppressed and Island stalls stay in the layout (so downstream
    // tools can see them) but don't count toward total_stalls — they
    // aren't usable parking.
    let total = stalls
        .iter()
        .filter(|s| {
            s.kind != StallKind::Suppressed
                && s.kind != StallKind::Island
                && s.kind != StallKind::Buffer
        })
        .count();

    // Always compute region debug info. When no separators exist, the
    // entire lot is a single region with the global aisle angle/offset.
    // Decomposition runs on the raw sketch so region IDs are tied to
    // sketch corners (not perturbed by inset distance changes).
    let region_debug = {
        let raw_outer_ids: &[VertexId] =
            if input.boundary.outer_ids.len() == input.boundary.outer.len() {
                &input.boundary.outer_ids
            } else {
                &[]
            };

        let mut hole_loops: Vec<Vec<Vec2>> = Vec::new();
        let mut hole_loop_ids: Vec<Vec<VertexId>> = Vec::new();
        for (i, h) in input.boundary.holes.iter().enumerate() {
            if h.is_empty() {
                continue;
            }
            hole_loops.push(h.clone());
            let ids = input
                .boundary
                .hole_ids
                .get(i)
                .filter(|v| v.len() == h.len())
                .cloned()
                .unwrap_or_default();
            hole_loop_ids.push(ids);
        }

        let mut region_list = if !partitioning_lines.is_empty() {
            decompose_regions(
                &input.boundary.outer,
                raw_outer_ids,
                &hole_loops,
                &hole_loop_ids,
                &partitioning_lines,
                input.params.aisle_angle,
                input.params.aisle_offset,
            )
        } else {
            vec![]
        };

        // If no regions formed (0 separators or only 1 per hole), fall
        // back to a single region covering the whole lot, tagged with
        // the reserved single-region-fallback ID.
        if region_list.is_empty() {
            region_list.push(Region {
                id: RegionId::single_region_fallback(),
                clip_poly: input.boundary.outer.clone(),
                aisle_angle: input.params.aisle_angle,
                aisle_offset: input.params.aisle_offset,
            });
        }

        // Apply per-region overrides.
        for ov in &input.region_overrides {
            if let Some(r) = region_list.iter_mut().find(|r| r.id == ov.region_id) {
                if let Some(a) = ov.aisle_angle { r.aisle_angle = a; }
                if let Some(o) = ov.aisle_offset { r.aisle_offset = o; }
            }
        }

        Some(RegionDebug {
            regions: region_list.into_iter().map(|r| {
                let center = region_pole(&r.clip_poly, &hole_loops);
                RegionInfo {
                    id: r.id,
                    clip_poly: r.clip_poly,
                    aisle_angle: r.aisle_angle,
                    aisle_offset: r.aisle_offset,
                    center,
                }
            }).collect(),
            separators: vec![],
        })
    };

    ParkingLayout {
        stalls,
        metrics: Metrics {
            total_stalls: total,
        },
        resolved_graph: graph,
        spines,
        faces,
        islands,
        derived_outer,
        derived_holes,
        region_debug,
        dormant_annotations,
    }
}

/// Clip drive lines against the inset perimeter/expanded holes and clamp to
/// the nearest existing aisle graph edge from each control point. This means
/// a short segment between two inner aisles only extends to those aisles,
/// while a long segment spanning the lot still clips to the perimeter.
/// Returns (graph, splice_anchors) where splice_anchors[i] = Some((drive_line_id, t))
/// for any vertex placed by the drive-line splice (t is normalized [0, 1] along
/// the user-drawn line), and None otherwise. Anchors are parallel to graph.vertices.
fn clip_drive_lines(
    lines: &[DriveLine],
    boundary: &Polygon,
    params: &ParkingParams,
    auto_graph: &DriveAisleGraph,
) -> (DriveAisleGraph, Vec<Option<(u32, f64)>>) {
    if lines.is_empty() {
        return (
            DriveAisleGraph { vertices: vec![], edges: vec![], perim_vertex_count: 0 },
            vec![],
        );
    }

    let hw = params.aisle_width;

    // Drive lines clip to the aisle-edge perimeter (the same ring
    // `auto_generate` placed graph perim vertices on), so a drive line
    // dropped at the lot edge naturally extends in to meet the perim
    // aisle and not the deed line.
    let p = aisle_edge_perim(boundary, params);
    if p.is_empty() {
        return (DriveAisleGraph { vertices: vec![], edges: vec![], perim_vertex_count: 0 }, vec![]);
    }
    let outer_loop = ensure_ccw(p);

    // boundary.holes are aisle-edge rings — use directly.
    let hole_loops: Vec<&[Vec2]> = boundary.holes.iter()
        .filter(|h| !h.is_empty())
        .map(|h| h.as_slice())
        .collect();

    let mut vertices: Vec<Vec2> = Vec::new();
    let mut edges: Vec<AisleEdge> = Vec::new();
    let mut anchors: Vec<Option<(u32, f64)>> = Vec::new();

    for dl in lines {
        let dir = dl.end - dl.start;
        if dir.length() < 1e-9 {
            continue;
        }
        let dir_norm = Vec2::new(dir.x / dir.length(), dir.y / dir.length());
        let origin = dl.start;
        let seg_len = dir.length();

        // Find nearest auto graph edge intersections to clamp the extent.
        let graph_hits = intersect_line_with_graph_edges(origin, dir_norm, auto_graph);
        // t_start: nearest hit at or before control point A (t <= 0)
        let t_start = graph_hits.iter().rev()
            .find(|&&t| t <= 0.0)
            .copied()
            .unwrap_or(f64::NEG_INFINITY);
        // t_end: nearest hit at or after control point B (t >= seg_len)
        let t_end = graph_hits.iter()
            .find(|&&t| t >= seg_len)
            .copied()
            .unwrap_or(f64::INFINITY);

        // Intersect with inset perimeter (not raw boundary).
        let outer_hits = intersect_line_polygon(origin, dir_norm, &outer_loop);
        let mut intervals: Vec<(f64, f64)> = Vec::new();
        for pair in outer_hits.chunks(2) {
            if pair.len() == 2 {
                intervals.push((pair[0].t_line, pair[1].t_line));
            }
        }

        // Subtract expanded hole interiors.
        for hl in &hole_loops {
            let hole_hits = intersect_line_polygon(origin, dir_norm, hl);
            let mut hole_ivs: Vec<(f64, f64)> = Vec::new();
            for pair in hole_hits.chunks(2) {
                if pair.len() == 2 {
                    hole_ivs.push((pair[0].t_line, pair[1].t_line));
                }
            }
            intervals = subtract_intervals(&intervals, &hole_ivs);
        }

        // Clamp intervals to [t_start, t_end] — the nearest graph edges.
        intervals = intervals.iter()
            .filter_map(|&(a, b)| {
                let a = a.max(t_start);
                let b = b.min(t_end);
                if a < b { Some((a, b)) } else { None }
            })
            .collect();

        // Create edges for surviving intervals. Drive lines are always
        // two-way; direction is applied later via annotations.
        // Split each interval at interior graph edge crossings so vertices
        // land exactly on graph edges regardless of control point placement.
        for &(ta, tb) in &intervals {
            // Use graph edge crossings as split points — both interior
            // and perimeter. This avoids polygon intersection positions
            // which can differ slightly from graph edges.
            let mut splits: Vec<f64> = Vec::new();
            for &t in &graph_hits {
                if t >= ta - 3.0 && t <= tb + 3.0 {
                    splits.push(t);
                }
            }
            if splits.is_empty() { continue; }
            splits.sort_by(|a, b| a.partial_cmp(b).unwrap());
            // Deduplicate nearby splits.
            let mut deduped = vec![splits[0]];
            for &s in &splits[1..] {
                if s - *deduped.last().unwrap() > 1.0 {
                    deduped.push(s);
                }
            }

            for pair in deduped.windows(2) {
                let pa = origin + dir_norm * pair[0];
                let pb = origin + dir_norm * pair[1];
                let via = find_or_add_vertex_anchored(
                    &mut vertices, &mut anchors, pa, 1.0, dl.id, pair[0] / seg_len);
                let vib = find_or_add_vertex_anchored(
                    &mut vertices, &mut anchors, pb, 1.0, dl.id, pair[1] / seg_len);
                if via != vib {
                    edges.push(AisleEdge { start: via, end: vib, width: hw, interior: true, direction: None });
                }
            }
        }
    }

    let graph = DriveAisleGraph {
        vertices,
        edges,
        perim_vertex_count: 0,
    };
    debug_assert_eq!(graph.vertices.len(), anchors.len());
    (graph, anchors)
}

/// Like find_or_add_vertex but also tracks the splice anchor for the
/// returned vertex. If the vertex already exists (by proximity), the
/// existing anchor wins (drive lines drawn earlier take precedence).
fn find_or_add_vertex_anchored(
    vertices: &mut Vec<Vec2>,
    anchors: &mut Vec<Option<(u32, f64)>>,
    p: Vec2,
    tol: f64,
    drive_line_id: u32,
    t: f64,
) -> usize {
    for (i, v) in vertices.iter().enumerate() {
        if (*v - p).length() < tol {
            return i;
        }
    }
    vertices.push(p);
    anchors.push(Some((drive_line_id, t)));
    vertices.len() - 1
}

/// Find all intersection t-values of an infinite line with graph edge segments.
/// Returns sorted t values.
fn intersect_line_with_graph_edges(
    origin: Vec2,
    dir: Vec2,
    graph: &DriveAisleGraph,
) -> Vec<f64> {
    let mut hits: Vec<f64> = Vec::new();

    for edge in &graph.edges {
        let s = graph.vertices[edge.start];
        let e = graph.vertices[edge.end];
        let seg = e - s;
        let denom = dir.cross(seg);
        if denom.abs() < 1e-12 { continue; }

        let h = s - origin;
        let t = h.cross(seg) / denom;
        let u = -(dir.cross(h)) / denom;

        if u >= 0.0 && u <= 1.0 {
            hits.push(t);
        }
    }

    hits.sort_by(|a, b| a.partial_cmp(b).unwrap());
    hits
}

/// Append graph b into graph a. For each b vertex, either merge with a nearby
/// existing vertex OR split the nearest a-edge that passes through it. This
/// ensures drive line endpoints on the perimeter/hole loops create proper
/// junctions for miter fill computation.
/// Merge graph `b` (with parallel splice anchors) into `a`. Returns the
/// merged graph and a splice-anchor vector parallel to its vertices: a's
/// vertices contribute `None` (auto-graph never has splice anchors), and
/// b's anchors are remapped to merged indices. When a b-vertex collapses
/// onto an existing a-vertex, the b anchor wins so the merged vertex
/// remains addressable by drive-line annotations.
fn append_graph(
    a: DriveAisleGraph,
    b: DriveAisleGraph,
    b_anchors: Vec<Option<(u32, f64)>>,
) -> (DriveAisleGraph, Vec<Option<(u32, f64)>>) {
    let vtx_tolerance = 1.0;
    let edge_tolerance = 2.0;
    let a_vertex_count = a.vertices.len();
    let mut vertices = a.vertices;
    let base_edge_count = a.edges.len();
    let mut edges = a.edges;
    let mut anchors: Vec<Option<(u32, f64)>> = vec![None; a_vertex_count];

    let mut b_to_merged: Vec<usize> = Vec::with_capacity(b.vertices.len());
    for (bi, bv) in b.vertices.iter().enumerate() {
        // First try: merge with an existing vertex.
        let existing = vertices
            .iter()
            .position(|mv| (*mv - *bv).length() < vtx_tolerance);
        if let Some(mi) = existing {
            b_to_merged.push(mi);
            // Promote: if the b-vertex carried a splice anchor, it
            // wins over the (None) auto-graph anchor at the merged
            // slot — otherwise the splice annotation has nothing to
            // resolve to.
            if anchors[mi].is_none() {
                anchors[mi] = b_anchors[bi];
            }
            continue;
        }

        // Second try: find an a-edge this vertex lies on, and split it.
        let new_vi = vertices.len();
        vertices.push(*bv);
        anchors.push(b_anchors[bi]);

        let mut best_split: Option<(usize, f64)> = None;
        let mut best_dist = edge_tolerance;
        for ei in 0..base_edge_count {
            let e = &edges[ei];
            let s = vertices[e.start];
            let ev = vertices[e.end];
            let seg = ev - s;
            let seg_len_sq = seg.x * seg.x + seg.y * seg.y;
            if seg_len_sq < 1e-12 { continue; }
            let t = ((*bv - s).dot(seg)) / seg_len_sq;
            if t <= 0.01 || t >= 0.99 { continue; }
            let proj = s + seg * t;
            let dist = (*bv - proj).length();
            if dist < best_dist {
                best_dist = dist;
                best_split = Some((ei, t));
            }
        }

        if let Some((ei, _t)) = best_split {
            // Split edge ei at new_vi: replace edge (s->e) with (s->new_vi) + (new_vi->e).
            let s = edges[ei].start;
            let e = edges[ei].end;
            let w = edges[ei].width;
            let interior = edges[ei].interior;
            let direction = edges[ei].direction.clone();

            edges[ei] = AisleEdge { start: s, end: new_vi, width: w, interior, direction: direction.clone() };
            edges.push(AisleEdge { start: new_vi, end: e, width: w, interior, direction });
        }

        b_to_merged.push(new_vi);
    }

    // Skip b-edges whose merged endpoints already share an edge in a.
    // This happens when a drive-line vertex lands on an existing a-edge:
    // the split spawns a half-edge between the same merged endpoints
    // that the b-edge would otherwise contribute.
    let mut existing_keys: std::collections::HashSet<(usize, usize)> =
        std::collections::HashSet::with_capacity(edges.len());
    for ex in &edges {
        existing_keys.insert((ex.start.min(ex.end), ex.start.max(ex.end)));
    }
    for e in &b.edges {
        let s = b_to_merged[e.start];
        let t = b_to_merged[e.end];
        if s == t {
            continue;
        }
        let key = (s.min(t), s.max(t));
        if !existing_keys.insert(key) {
            continue;
        }
        edges.push(AisleEdge {
            start: s,
            end: t,
            width: e.width,
            interior: e.interior,
            direction: e.direction.clone(),
        });
    }

    let merged = DriveAisleGraph {
        vertices,
        edges,
        perim_vertex_count: a.perim_vertex_count,
    };
    debug_assert_eq!(merged.vertices.len(), anchors.len());
    (merged, anchors)
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::clip::point_in_polygon;

    #[test]
    fn pole_u_shape_no_holes() {
        // U-shaped polygon (opening at top).
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(80.0, 100.0),
            Vec2::new(80.0, 40.0),
            Vec2::new(20.0, 40.0),
            Vec2::new(20.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];
        let center = region_pole(&poly, &[]);
        assert!(point_in_polygon(&center, &poly),
            "pole ({}, {}) must be inside U-shape", center.x, center.y);
        // Must be in the bottom bar, not the empty opening.
        assert!(center.y < 40.0,
            "pole ({}, {}) should be in the bottom bar (y < 40)", center.x, center.y);
    }

    #[test]
    fn pole_avoids_hole() {
        // Region polygon that encompasses a building hole (from real debug data).
        // The polygon traces the outer boundary and the building's right/bottom/left
        // edges, so the building interior is inside the polygon.
        let poly = vec![
            Vec2::new(394.5, 251.8),
            Vec2::new(752.0, 39.2),
            Vec2::new(762.2, 244.2),
            Vec2::new(611.1, 251.8),
            Vec2::new(613.9, 512.3),
            Vec2::new(394.5, 512.3),
        ];
        // The building hole (approximate rectangle).
        let hole = vec![
            Vec2::new(394.5, 251.8),
            Vec2::new(611.1, 251.8),
            Vec2::new(613.9, 512.3),
            Vec2::new(394.5, 512.3),
        ];

        // Without the hole, polylabel lands inside the building.
        let center_no_hole = region_pole(&poly, &[]);
        assert!(point_in_polygon(&center_no_hole, &hole),
            "without hole, pole ({}, {}) should land inside building",
            center_no_hole.x, center_no_hole.y);

        // With the hole, polylabel must avoid the building.
        let center = region_pole(&poly, &[hole.clone()]);
        assert!(point_in_polygon(&center, &poly),
            "pole ({}, {}) must be inside region polygon", center.x, center.y);
        assert!(!point_in_polygon(&center, &hole),
            "pole ({}, {}) must NOT be inside building hole", center.x, center.y);
    }

    #[test]
    fn pole_region1_avoids_hole() {
        // Second region from the same debug session.
        let poly = vec![
            Vec2::new(611.1, 251.8),
            Vec2::new(762.2, 244.2),
            Vec2::new(782.8, 654.9),
            Vec2::new(168.8, 654.9),
            Vec2::new(76.3, 0.0),
            Vec2::new(750.0, 0.0),
            Vec2::new(752.0, 39.2),
            Vec2::new(394.5, 251.8),
        ];
        let hole = vec![
            Vec2::new(394.5, 251.8),
            Vec2::new(611.1, 251.8),
            Vec2::new(613.9, 512.3),
            Vec2::new(394.5, 512.3),
        ];
        let center = region_pole(&poly, &[hole.clone()]);
        assert!(point_in_polygon(&center, &poly),
            "pole ({}, {}) must be inside region polygon", center.x, center.y);
        assert!(!point_in_polygon(&center, &hole),
            "pole ({}, {}) must NOT be inside building hole", center.x, center.y);
    }
}



/// Generate stalls from positive-space face extraction + per-edge spine shifting.
/// Returns (stalls, spine_lines, faces, islands).
pub fn generate_from_spines(
    graph: &DriveAisleGraph,
    boundary: &Polygon,
    params: &ParkingParams,
    debug: &DebugToggles,
    stall_modifiers: &[crate::types::StallModifier],
) -> (Vec<StallQuad>, Vec<SpineLine>, Vec<Face>, Vec<crate::types::Island>) {
    let stall_angle_rad = params.stall_angle.to_radians();
    let effective_depth = params.stall_depth * stall_angle_rad.sin()
        + stall_angle_rad.cos() * params.stall_width / 2.0;
    let inset_d = compute_inset_d(params);
    let raw_outer = derive_raw_outer(&boundary.outer, inset_d, params.site_offset);
    let raw_holes = derive_raw_holes(&boundary.holes, inset_d);

    // Per-edge corridor rectangles (used by face-edge tagging only —
    // the merged driveable surface comes straight from a stroke of the
    // centerline graph below).
    let per_edge_corridors = corridor_polygons(graph);
    // Per-corridor "reverse lean" flag, encoded in the legacy
    // Option<Vec2> slot (Some = reverse, None = normal). Only
    // TwoWayReverse populates it, so placement can negate cos_a on
    // the affected spines and produce the mirrored slash pattern.
    let two_way_oriented_dirs: Vec<Option<Vec2>> = graph
        .edges
        .iter()
        .map(|edge| {
            if edge.direction == Some(AisleDirection::TwoWayReverse) {
                Some(Vec2::new(0.0, 0.0))
            } else {
                None
            }
        })
        .collect();
    let merged_corridors = merge_corridor_surface(graph, params.aisle_width);

    // Extract faces by subtracting the merged corridor union from the
    // boundary. Since the union resolves all internal seams between
    // corridor rects and miter fills, no sliver artifacts remain.
    let faces = extract_faces(&raw_outer, &merged_corridors, &raw_holes);

    // When island corner rounding is on, morph-open each face polygon
    // upstream of every stall-vs-face check. This way clipping,
    // entrance coverage, and the island residual all see the same
    // rounded "vehicle-accessible" boundary — stalls don't get placed
    // and then visually overshoot the rounded contour. Spine
    // generation and face-edge tagging keep using the raw `faces`
    // because they reason about original aisle-edge provenance, not
    // turn-radius geometry.
    //
    // If opening severs a face into multiple pieces (thin neck < 2r),
    // we keep just the largest piece per face_idx so the per-face
    // indexing throughout the pipeline stays 1:1. An emptied face
    // (collapsed entirely) becomes an empty shape; downstream
    // `if shape[0].len() < 3` guards skip it cleanly.
    let effective_faces: Vec<Vec<Vec<Vec2>>> = if debug.island_corner_rounding
        && params.island_corner_radius > 0.0
    {
        faces
            .iter()
            .map(|face| {
                if face.is_empty() || face[0].len() < 3 {
                    return face.clone();
                }
                crate::geom::offset::morph_open_round(face, params.island_corner_radius, 0.1)
                    .into_iter()
                    .filter(|s| !s.is_empty() && s[0].len() >= 3)
                    .max_by(|a, b| {
                        let aa = signed_area(&a[0]).abs();
                        let ba = signed_area(&b[0]).abs();
                        aa.partial_cmp(&ba).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .unwrap_or_else(Vec::new)
            })
            .collect()
    } else {
        faces.clone()
    };

    // Tag each face edge with its source corridor/wall for provenance.
    // Indexed by face_idx (parallel to `faces`); empty/small faces get a
    // default empty TaggedFace.
    let tagged_faces: Vec<TaggedFace> = faces.iter()
        .map(|shape| {
            if !shape.is_empty() && shape[0].len() >= 3 {
                tag_face_edges(shape, &merged_corridors, &per_edge_corridors, &two_way_oriented_dirs)
            } else {
                TaggedFace { edges: vec![], hole_edges: vec![], is_boundary: true, wall_edge_indices: vec![] }
            }
        })
        .collect();

    // Collect spines from all faces, then merge collinear segments across
    // face boundaries so stalls flow continuously along shared aisle edges.
    let spine_merge_cos_tol = params.spine_merge_angle_deg.to_radians().cos();
    let mut raw_spines = Vec::new();
    for (face_idx, shape) in faces.iter().enumerate() {
        let face_is_boundary = tagged_faces[face_idx].is_boundary;
        let tagged_ref = Some(&tagged_faces[face_idx]);
        let mut face_spines = compute_face_spines(shape, effective_depth, &merged_corridors, &per_edge_corridors, face_is_boundary, debug, &two_way_oriented_dirs, tagged_ref);
        for s in &mut face_spines {
            s.face_idx = face_idx;
        }
        raw_spines.extend(face_spines);
    }
    let all_spines = if debug.spine_merging {
        merge_collinear_spines(raw_spines, params.spine_merge_endpoint_tol, spine_merge_cos_tol)
    } else {
        raw_spines
    };

    // Compute extensions and fold them into their source primary's extent
    // so each primary becomes a single longer SpineSegment covering its
    // full reach. Downstream — including the snap pass — sees one flat
    // list of spines and is oblivious to whether any of them was extended.
    let all_spines: Vec<SpineSegment> = if debug.spine_extensions {
        let exts = extend_spines_to_faces(&all_spines, &effective_faces, effective_depth, params);
        let mut exts_by_primary: Vec<Vec<SpineSegment>> = vec![Vec::new(); all_spines.len()];
        for (ext, src_idx) in exts {
            exts_by_primary[src_idx].push(ext);
        }
        all_spines
            .into_iter()
            .enumerate()
            .map(|(i, primary)| extend_primary_with_extensions(primary, &exts_by_primary[i]))
            .collect()
    } else {
        all_spines
    };

    // Place stalls on merged spines (tagged with face_idx + spine_idx).
    let tagged_stalls_3 = place_stalls_on_spines(&all_spines, params);

    // Drop boundary-spine stalls whose aisle-facing edge doesn't hug the
    // face boundary. Catches corner overruns that quad_fully_in_face
    // misses because the body is still inside the face even when the
    // entrance dangles off.
    let tagged_stalls_3 = if debug.entrance_on_face_filter {
        filter_stalls_by_entrance_coverage(tagged_stalls_3, &effective_faces)
    } else {
        tagged_stalls_3
    };

    // Convert to (StallQuad, usize) for clipping, preserving spine_idx.
    let tagged_stalls: Vec<(StallQuad, usize)> = tagged_stalls_3.iter()
        .map(|(s, fi, _)| (s.clone(), *fi))
        .collect();

    // Resolve stall conflicts BEFORE face-overhang clipping. When
    // multiple candidate spines exist for the same narrow face (e.g.
    // opposing offset spines in a sub-2*ed bay), their stalls overlap
    // spatially and need to fight it out on spine length. Running
    // clip_stalls_to_faces first would asymmetrically drop the
    // candidate whose stalls hang off a slanted face edge, silently
    // handing the contest to the shorter spine.
    let tagged_stalls = if debug.conflict_removal {
        // Build per-stall spine lengths by matching surviving stalls back
        // to their spine_idx via corner identity. Spines on direction-
        // annotated aisles get a small length bonus so they outrank a
        // TwoWay neighbor of identical geometry — without this, the
        // longer-spine tiebreak collapses on equal-length back-to-back
        // spines and the annotated side's distinctive lean gets eaten
        // by the unannotated default.
        let stall_spine_lengths: Vec<f64> = {
            let key_to_spine_len: std::collections::HashMap<[u64; 8], f64> = tagged_stalls_3
                .iter()
                .map(|(s, _, si)| {
                    let spine = &all_spines[*si];
                    let mut len = (spine.end - spine.start).length();
                    if spine.is_annotated {
                        len += 0.5;
                    }
                    (stall_key(s), len)
                })
                .collect();
            tagged_stalls
                .iter()
                .map(|(s, _)| *key_to_spine_len.get(&stall_key(s)).unwrap_or(&0.0))
                .collect()
        };
        // Build per-face boundary flags.
        let face_boundary: Vec<bool> = tagged_faces.iter().map(|tf| tf.is_boundary).collect();
        remove_conflicting_stalls(tagged_stalls, &stall_spine_lengths, &face_boundary)
    } else {
        tagged_stalls
    };

    // Clip stalls to face interiors. A stall that protrudes past the face
    // boundary into a corridor (at miter-fill corners) is removed.
    let tagged_stalls = if debug.stall_face_clipping {
        clip_stalls_to_faces(tagged_stalls, &effective_faces)
    } else {
        tagged_stalls
    };

    // Rebuild the 3-tuple list from surviving stalls to compute envelopes.
    // Match surviving stalls back to their spine_idx by corner identity.
    let surviving: std::collections::HashSet<[u64; 8]> = tagged_stalls.iter()
        .map(|(s, _)| stall_key(s))
        .collect();
    let mut all_surviving_3: Vec<(StallQuad, usize, usize)> = tagged_stalls_3.into_iter()
        .filter(|(s, _, _)| surviving.contains(&stall_key(s)))
        .collect();

    let mut all_tagged: Vec<(StallQuad, usize)> = all_surviving_3
        .iter()
        .map(|(s, fi, _)| (s.clone(), *fi))
        .collect();

    // --- Short segment filter ---
    // Remove all stalls belonging to spines with fewer than
    // params.min_stalls_per_spine stalls. min_stalls_per_spine=1 is the
    // natural "off" — every spine with any stall has ≥1.
    {
        let min_stalls = params.min_stalls_per_spine as usize;
        let mut counts: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
        for (_, _, spine_idx) in &all_surviving_3 {
            *counts.entry(*spine_idx).or_insert(0) += 1;
        }
        let keep_spines: std::collections::HashSet<usize> = counts
            .into_iter()
            .filter(|(_, count)| *count >= min_stalls)
            .map(|(idx, _)| idx)
            .collect();
        all_surviving_3.retain(|(_, _, spine_idx)| keep_spines.contains(spine_idx));
        let surviving_keys: std::collections::HashSet<[u64; 8]> = all_surviving_3
            .iter()
            .map(|(s, _, _)| stall_key(s))
            .collect();
        all_tagged.retain(|(s, _)| surviving_keys.contains(&stall_key(s)));
    }

    // --- Island stall marking ---
    if params.island_stall_interval > 0 {
        let modifier_radius = params.stall_depth * 0.5;
        mark_island_stalls(
            &mut all_surviving_3, &mut all_tagged,
            &all_spines, params,
            stall_modifiers, modifier_radius,
        );
    }

    let islands = compute_islands(
        &effective_faces,
        &all_tagged,
        10.0,
        debug.island_stall_dilation,
    );
    let all_stalls: Vec<StallQuad> = all_tagged.iter().map(|(s, _)| s.clone()).collect();

    // Build spine lines for visualization: emit every spine that spine
    // generation produced, independent of downstream stall-level filters
    // (face clipping, boundary clipping, conflict removal, short-segment
    // filter). The spines layer reflects spine generation, not stall
    // survival.
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
        .enumerate()
        .filter(|(_, shape)| !shape.is_empty() && shape[0].len() >= 3)
        .map(|(face_idx, _shape)| {
            let tf = &tagged_faces[face_idx];
            let source_label = |e: &FaceEdge| -> String {
                match &e.source {
                    EdgeSource::Wall => "wall".to_string(),
                    EdgeSource::Aisle { interior: true, .. } => "interior".to_string(),
                    EdgeSource::Aisle { interior: false, .. } => "perimeter".to_string(),
                }
            };
            let contour: Vec<Vec2> = tf.edges.iter().map(|e| e.start).collect();
            let edge_sources: Vec<String> = tf.edges.iter().map(&source_label).collect();
            let holes: Vec<Vec<Vec2>> = tf.hole_edges.iter()
                .map(|hole| hole.iter().map(|e| e.start).collect())
                .collect();
            let hole_edge_sources: Vec<Vec<String>> = tf.hole_edges.iter()
                .map(|hole| hole.iter().map(&source_label).collect())
                .collect();
            Face {
                contour,
                holes,
                is_boundary: tf.is_boundary,
                edge_sources,
                hole_edge_sources,
            }
        })
        .collect();

    (all_stalls, spine_lines, face_list, islands)
}
