use crate::annotations::{apply_annotations, resolve_regions_for_frames, ResolvedRegion};
use crate::graph::{auto_generate, compute_inset_d, decompose_regions, derive_raw_holes, derive_raw_outer, intersect_line_polygon, merge_with_auto, subtract_intervals, Region};
use crate::geom::clip::point_in_polygon;
use crate::face::generate_from_spines;
use crate::geom::inset::{inset_polygon, signed_area};
use crate::types::*;
use geo::{Coord, LineString};

fn region_pole(poly: &[Vec2], holes: &[Vec<Vec2>]) -> Vec2 {
    if poly.len() < 3 {
        return if poly.is_empty() { Vec2::new(0.0, 0.0) } else { poly[0] };
    }
    let to_ls = |pts: &[Vec2]| -> LineString<f64> {
        LineString::new(pts.iter().map(|v| Coord { x: v.x, y: v.y }).collect())
    };
    let inner: Vec<LineString<f64>> = holes.iter()
        .filter(|h| h.len() >= 3)
        .map(|h| to_ls(h))
        .collect();
    let geo_poly = geo::Polygon::new(to_ls(poly), inner);
    match polylabel::polylabel(&geo_poly, &1.0) {
        Ok(pt) => Vec2::new(pt.x(), pt.y()),
        Err(_) => poly[0],
    }
}

pub fn generate(input: GenerateInput) -> ParkingLayout {
    // Discretize curved boundary edges into dense polylines so the
    // entire downstream pipeline works on straight-line segments.
    let mut input = input;
    input.boundary = crate::geom::bezier::discretize_polygon(&input.boundary);

    // Partitioning drive lines contribute to the planar-arrangement
    // face enumeration (regions). During the migration we accept
    // either the new `partitions` flag or the legacy `hole_pin` as the
    // signal — either one means "this line divides space".
    let partitioning_lines: Vec<(u32, Vec2, Vec2)> = input.drive_lines.iter()
        .filter(|dl| dl.partitions || dl.hole_pin.is_some())
        .map(|dl| (dl.id, dl.start, dl.end))
        .collect();

    // First, resolve the aisle graph from manual + auto as usual.
    let mut graph = match input.aisle_graph.clone() {
        Some(manual) => merge_with_auto(manual, &input.boundary, &input.params),
        None => auto_generate(
            &input.boundary,
            &input.params,
            &partitioning_lines,
            &input.region_overrides,
        ),
    };

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

    let (stalls, aisle_polygons, spines, faces, miter_fills, skeleton_debug, islands) =
        generate_from_spines(&graph, &input.boundary, &input.params, &input.debug);

    // Remove stalls that fall inside raw drive line corridors. The raw
    // corridors extend from the user-specified endpoints (which lie outside
    // the boundary) through the perimeter aisle to the interior, so they
    // cover the boundary-face stalls at each entrance point.
    let stalls = if !input.drive_lines.is_empty() {
        let hw = input.params.aisle_width;
        let drive_corridors: Vec<Vec<Vec2>> = input.drive_lines.iter()
            .filter_map(|dl| {
                let dir = dl.end - dl.start;
                if dir.length() < 1e-9 { return None; }
                let n = Vec2::new(-dir.y, dir.x).normalize() * hw;
                Some(vec![
                    dl.start + n,
                    dl.end + n,
                    dl.end - n,
                    dl.start - n,
                ])
            })
            .collect();
        stalls.into_iter()
            .filter(|stall| {
                let c = Vec2::new(
                    (stall.corners[0].x + stall.corners[1].x + stall.corners[2].x + stall.corners[3].x) / 4.0,
                    (stall.corners[0].y + stall.corners[1].y + stall.corners[2].y + stall.corners[3].y) / 4.0,
                );
                !drive_corridors.iter().any(|corridor| point_in_polygon(&c, corridor))
            })
            .collect()
    } else {
        stalls
    };

    let total = stalls.len();

    // Always compute region debug info. When no separators exist, the
    // entire lot is a single region with the global aisle angle/offset.
    let region_debug = {
        let outer_loop = if input.params.site_offset > 0.0 {
            let p = inset_polygon(&input.boundary.outer, input.params.site_offset);
            if p.is_empty() { input.boundary.outer.clone() } else { ensure_ccw(p) }
        } else {
            ensure_ccw(input.boundary.outer.clone())
        };

        let hole_loops: Vec<Vec<Vec2>> = input.boundary.holes.iter()
            .filter(|h| !h.is_empty())
            .cloned()
            .collect();

        let mut region_list = if !partitioning_lines.is_empty() {
            decompose_regions(
                &outer_loop,
                &hole_loops,
                &partitioning_lines,
                input.params.aisle_angle_deg,
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
                clip_poly: outer_loop.clone(),
                aisle_angle_deg: input.params.aisle_angle_deg,
                aisle_offset: input.params.aisle_offset,
            });
        }

        // Apply per-region overrides.
        for ov in &input.region_overrides {
            if let Some(r) = region_list.iter_mut().find(|r| r.id == ov.region_id) {
                if let Some(a) = ov.aisle_angle_deg { r.aisle_angle_deg = a; }
                if let Some(o) = ov.aisle_offset { r.aisle_offset = o; }
            }
        }

        Some(RegionDebug {
            regions: region_list.into_iter().map(|r| {
                let center = region_pole(&r.clip_poly, &hole_loops);
                RegionInfo {
                    id: r.id,
                    clip_poly: r.clip_poly,
                    aisle_angle_deg: r.aisle_angle_deg,
                    aisle_offset: r.aisle_offset,
                    center,
                }
            }).collect(),
            separators: vec![],
        })
    };

    ParkingLayout {
        aisle_polygons,
        stalls,
        metrics: Metrics {
            total_stalls: total,
        },
        resolved_graph: graph,
        spines,
        faces,
        miter_fills,
        skeleton_debug,
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

    // boundary.outer is the aisle-edge perimeter — use directly (with site_offset).
    let outer_loop = if params.site_offset > 0.0 {
        let p = inset_polygon(&boundary.outer, params.site_offset);
        if p.is_empty() { return (DriveAisleGraph { vertices: vec![], edges: vec![], perim_vertex_count: 0 }, vec![]); }
        ensure_ccw(p)
    } else {
        ensure_ccw(boundary.outer.clone())
    };

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
                    edges.push(AisleEdge { start: via, end: vib, width: hw, interior: true, direction: AisleDirection::TwoWay });
                    edges.push(AisleEdge { start: vib, end: via, width: hw, interior: true, direction: AisleDirection::TwoWay });
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
/// Returns sorted t values. Deduplicates bidirectional edges.
fn intersect_line_with_graph_edges(
    origin: Vec2,
    dir: Vec2,
    graph: &DriveAisleGraph,
) -> Vec<f64> {
    let mut hits: Vec<f64> = Vec::new();
    let mut seen = std::collections::HashSet::new();

    for edge in &graph.edges {
        let key = (edge.start.min(edge.end), edge.start.max(edge.end));
        if !seen.insert(key) { continue; }

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

fn ensure_ccw(mut poly: Vec<Vec2>) -> Vec<Vec2> {
    if signed_area(&poly) < 0.0 {
        poly.reverse();
    }
    poly
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
            // Since edges are stored bidirectionally, find and split the reverse too.
            let s = edges[ei].start;
            let e = edges[ei].end;
            let w = edges[ei].width;
            let interior = edges[ei].interior;
            let direction = edges[ei].direction.clone();

            // Replace the forward edge with s->new_vi, add new_vi->e.
            edges[ei] = AisleEdge { start: s, end: new_vi, width: w, interior, direction: direction.clone() };
            edges.push(AisleEdge { start: new_vi, end: e, width: w, interior, direction: direction.clone() });

            // Find and split the reverse edge (e->s).
            for ri in 0..edges.len() {
                if ri == ei { continue; }
                if edges[ri].start == e && edges[ri].end == s {
                    let rd = edges[ri].direction.clone();
                    edges[ri] = AisleEdge { start: e, end: new_vi, width: w, interior, direction: rd.clone() };
                    edges.push(AisleEdge { start: new_vi, end: s, width: w, interior, direction: rd });
                    break;
                }
            }
        }

        b_to_merged.push(new_vi);
    }

    for e in &b.edges {
        edges.push(AisleEdge {
            start: b_to_merged[e.start],
            end: b_to_merged[e.end],
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
