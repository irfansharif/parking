use crate::aisle_graph::{auto_generate, compute_inset_d, decompose_regions, derive_raw_holes, derive_raw_outer, intersect_line_polygon, merge_with_auto, subtract_intervals, Region};
use crate::clip::point_in_polygon;
use crate::face::generate_from_spines;
use crate::inset::{inset_polygon, signed_area};
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
    input.boundary = crate::bezier::discretize_polygon(&input.boundary);

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

/// One region's abstract frame plus its clip polygon — what
/// `apply_annotations` needs to resolve Grid-target annotations.
struct ResolvedRegion {
    id: RegionId,
    clip_poly: Vec<Vec2>,
    frame: AbstractFrame,
}

/// Build the (id, frame, clip_poly) triples for the current generate
/// input. Mirrors the decomposition logic in the region_debug block
/// below but returns the frames for annotation resolution.
fn resolve_regions_for_frames(
    input: &GenerateInput,
    partitioning_lines: &[(u32, Vec2, Vec2)],
) -> Vec<ResolvedRegion> {
    let outer_loop = if input.params.site_offset > 0.0 {
        let p = inset_polygon(&input.boundary.outer, input.params.site_offset);
        if p.is_empty() {
            input.boundary.outer.clone()
        } else {
            ensure_ccw(p)
        }
    } else {
        ensure_ccw(input.boundary.outer.clone())
    };
    let hole_loops: Vec<Vec<Vec2>> = input
        .boundary
        .holes
        .iter()
        .filter(|h| !h.is_empty())
        .cloned()
        .collect();

    let mut region_list = if !partitioning_lines.is_empty() {
        decompose_regions(
            &outer_loop,
            &hole_loops,
            partitioning_lines,
            input.params.aisle_angle_deg,
            input.params.aisle_offset,
        )
    } else {
        vec![]
    };
    if region_list.is_empty() {
        region_list.push(Region {
            id: RegionId::single_region_fallback(),
            clip_poly: outer_loop.clone(),
            aisle_angle_deg: input.params.aisle_angle_deg,
            aisle_offset: input.params.aisle_offset,
        });
    }
    for ov in &input.region_overrides {
        if let Some(r) = region_list.iter_mut().find(|r| r.id == ov.region_id) {
            if let Some(a) = ov.aisle_angle_deg {
                r.aisle_angle_deg = a;
            }
            if let Some(o) = ov.aisle_offset {
                r.aisle_offset = o;
            }
        }
    }
    region_list
        .into_iter()
        .map(|r| {
            let frame = AbstractFrame::region(&input.params, r.aisle_angle_deg, r.aisle_offset);
            ResolvedRegion {
                id: r.id,
                clip_poly: r.clip_poly,
                frame,
            }
        })
        .collect()
}

/// Snap tolerance for treating a vertex as "on" an integer abstract
/// grid point. 0.1 world units ≈ 1.2 inches — tight enough to catch
/// numerical drift but loose enough to tolerate it.
const ABSTRACT_SNAP_TOL: f64 = 0.1;

/// For each vertex in the graph, find the region frame (if any) in
/// which the vertex sits exactly on an integer grid point. Returns a
/// lookup `(region, xi, yi) → vertex_index`.
fn build_abstract_vertex_lookup(
    graph: &DriveAisleGraph,
    regions: &[ResolvedRegion],
) -> std::collections::HashMap<(RegionId, i32, i32), usize> {
    use std::collections::HashMap;
    let mut lookup: HashMap<(RegionId, i32, i32), usize> = HashMap::new();
    for (vi, v) in graph.vertices.iter().enumerate() {
        for region in regions {
            if !point_in_polygon(v, &region.clip_poly) {
                continue;
            }
            let abs = region.frame.inverse(*v);
            let xi_round = abs.x.round();
            let yi_round = abs.y.round();
            // Snap tolerance in abstract units: divide world tolerance by
            // the axis scale so 0.1 world units maps to an
            // axis-independent abstract tolerance.
            let dx_abs = (abs.x - xi_round).abs();
            let dy_abs = (abs.y - yi_round).abs();
            if dx_abs * region.frame.dx > ABSTRACT_SNAP_TOL
                || dy_abs * region.frame.dy > ABSTRACT_SNAP_TOL
            {
                continue;
            }
            lookup.insert((region.id, xi_round as i32, yi_round as i32), vi);
            break;
        }
    }
    lookup
}

/// Grid-lattice abstract tolerance (fraction of a cell). Used both for
/// "edge midpoint on this lattice line" and "endpoint sits on this lattice
/// line" checks.
const GRID_ABS_TOL: f64 = 0.05;

/// Resolve the set of graph-edge indices covered by a `Target::Grid`.
/// Returns `None` if the target addresses a vertex (range = Some((s, s))),
/// if the range's stops can't be resolved, or if no edges match.
fn resolve_grid_edges(
    graph: &DriveAisleGraph,
    regions: &[ResolvedRegion],
    region: RegionId,
    axis: Axis,
    coord: i32,
    range: &Option<(GridStop, GridStop)>,
) -> Option<Vec<usize>> {
    let region_data = regions.iter().find(|r| r.id == region)?;
    let frame = &region_data.frame;
    // axis=X → line fixed at x=coord, runs along Y (varying axis = y).
    // axis=Y → line fixed at y=coord, runs along X (varying axis = x).
    let along_x = axis == Axis::Y;
    let fixed = coord;

    let (lo, hi) = match range {
        None => (i32::MIN, i32::MAX), // whole line
        Some((a, b)) => {
            let pa = grid_stop_to_other_axis(a)?;
            let pb = grid_stop_to_other_axis(b)?;
            if pa == pb {
                return None; // point (vertex), not an edge target
            }
            (pa.min(pb), pa.max(pb))
        }
    };

    let mut hits = Vec::new();
    for (i, edge) in graph.edges.iter().enumerate() {
        let s = graph.vertices[edge.start];
        let e = graph.vertices[edge.end];
        let abs_s = frame.inverse(s);
        let abs_e = frame.inverse(e);
        let edge_along_x = (abs_e.x - abs_s.x).abs() > (abs_e.y - abs_s.y).abs();
        if edge_along_x != along_x {
            continue;
        }
        let (mid_fixed, mid_var) = if along_x {
            ((abs_s.y + abs_e.y) * 0.5, (abs_s.x + abs_e.x) * 0.5)
        } else {
            ((abs_s.x + abs_e.x) * 0.5, (abs_s.y + abs_e.y) * 0.5)
        };
        if (mid_fixed - fixed as f64).abs() > GRID_ABS_TOL {
            continue;
        }
        // Span filter (open-ended for whole-line targets).
        if lo != i32::MIN && mid_var < lo as f64 - GRID_ABS_TOL {
            continue;
        }
        if hi != i32::MAX && mid_var > hi as f64 + GRID_ABS_TOL {
            continue;
        }
        hits.push(i);
    }
    if hits.is_empty() { None } else { Some(hits) }
}

/// Convert a GridStop to its coordinate along the grid line's varying axis.
/// Only `Lattice` is supported in this prototype; `CrossesDriveLine` and
/// `CrossesPerimeter` stops return `None` (dormant) until their resolution
/// is implemented.
fn grid_stop_to_other_axis(s: &GridStop) -> Option<i32> {
    match s {
        GridStop::Lattice { other } => Some(*other),
        _ => None, // Cross* stops not yet implemented; annotation goes dormant.
    }
}

/// Apply a direction to every edge index in `hits`, flipping `start`/`end`
/// so the stored orientation matches the carrier's canonical direction
/// (or its reverse, per `traffic`).
fn apply_direction_to_edges(
    graph: &mut DriveAisleGraph,
    hits: &[usize],
    canonical_sign_along_axis: f64, // +1 if edge's other-axis coord should increase start→end to match canonical
    along_x: bool,
    frame_opt: Option<&AbstractFrame>,
    traffic: TrafficDirection,
) {
    let want_canonical = matches!(
        traffic,
        TrafficDirection::OneWay | TrafficDirection::TwoWayOriented
    );
    let aisle_dir = match traffic {
        TrafficDirection::OneWay | TrafficDirection::OneWayReverse => AisleDirection::OneWay,
        TrafficDirection::TwoWayOriented | TrafficDirection::TwoWayOrientedReverse => {
            AisleDirection::TwoWayOriented
        }
    };
    for &i in hits {
        let s = graph.vertices[graph.edges[i].start];
        let e = graph.vertices[graph.edges[i].end];
        let edge_var_sign = if let Some(frame) = frame_opt {
            let abs_s = frame.inverse(s);
            let abs_e = frame.inverse(e);
            let d = if along_x { abs_e.x - abs_s.x } else { abs_e.y - abs_s.y };
            d.signum()
        } else {
            // No frame → fall back to world-axis comparison (used for
            // DriveLine carrier where canonical_sign_along_axis is
            // encoded via pre-aligned direction vector; caller handles).
            1.0
        };
        let edge_aligned_with_canonical = edge_var_sign * canonical_sign_along_axis > 0.0;
        if edge_aligned_with_canonical != want_canonical {
            let t = graph.edges[i].start;
            graph.edges[i].start = graph.edges[i].end;
            graph.edges[i].end = t;
        }
        graph.edges[i].direction = aisle_dir.clone();
    }
}

/// Apply a direction on a drive-line sub-edge identified by parametric
/// point `t`. Uses the splice_lookup to find the sub-edge whose t-range
/// contains `t`, then flips/sets direction according to `traffic` relative
/// to the drive line's canonical direction (+t, start → end).
fn apply_drive_line_direction(
    graph: &mut DriveAisleGraph,
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    drive_line_id: u32,
    t: f64,
    pitch: f64,
    traffic: TrafficDirection,
) -> bool {
    let Some(hits) = resolve_drive_line_edges_containing(graph, splice_lookup, line_lengths, drive_line_id, t, pitch) else {
        return false;
    };
    if hits.is_empty() {
        return false;
    }
    let want_canonical = matches!(
        traffic,
        TrafficDirection::OneWay | TrafficDirection::TwoWayOriented
    );
    let aisle_dir = match traffic {
        TrafficDirection::OneWay | TrafficDirection::OneWayReverse => AisleDirection::OneWay,
        TrafficDirection::TwoWayOriented | TrafficDirection::TwoWayOrientedReverse => {
            AisleDirection::TwoWayOriented
        }
    };
    // For drive-line edges we orient using the splice-anchored t of each endpoint.
    let Some(entries) = splice_lookup.get(&drive_line_id) else { return false };
    let mut v_to_t: std::collections::HashMap<usize, f64> =
        std::collections::HashMap::with_capacity(entries.len());
    for &(et, vi) in entries {
        v_to_t.insert(vi, et);
    }
    for i in hits {
        let (Some(&ts), Some(&te)) = (
            v_to_t.get(&graph.edges[i].start),
            v_to_t.get(&graph.edges[i].end),
        ) else {
            continue;
        };
        let edge_aligned_with_canonical = te > ts;
        if edge_aligned_with_canonical != want_canonical {
            let tmp = graph.edges[i].start;
            graph.edges[i].start = graph.edges[i].end;
            graph.edges[i].end = tmp;
        }
        graph.edges[i].direction = aisle_dir.clone();
    }
    true
}

// ---------------------------------------------------------------------------
// Perimeter-target resolution
// ---------------------------------------------------------------------------
//
// Graph vertices that sit on a perimeter loop (outer or hole) are indexed by
// their normalized arc length along that loop. `Target::Perimeter` then
// names a parametric `arc ∈ [0, 1]`; resolution picks the graph vertex
// nearest that arc (for vertex annotations) or the sub-edge whose arc range
// contains it (for edge annotations).

pub(crate) struct PerimeterLookupEntry {
    /// (normalized arc ∈ [0,1), graph vertex index), sorted by arc.
    entries: Vec<(f64, usize)>,
    /// Total world-space length of the loop.
    total_length: f64,
}

/// World-space tolerance for considering a graph vertex "on" a perimeter
/// loop (via nearest-edge projection).
const PERIM_VTX_ON_LOOP_TOL: f64 = 0.5;

fn build_perimeter_lookup(
    graph: &DriveAisleGraph,
    boundary: &Polygon,
    params: &ParkingParams,
) -> std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry> {
    let mut out: std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry> =
        std::collections::HashMap::new();

    // Effective outer loop: inset by site_offset to match auto_generate's
    // perimeter. Holes are used directly (they're already the aisle-edge
    // rings stored on the Polygon).
    let outer_loop = if params.site_offset > 0.0 {
        let p = inset_polygon(&boundary.outer, params.site_offset);
        if p.is_empty() { boundary.outer.clone() } else { ensure_ccw(p) }
    } else {
        ensure_ccw(boundary.outer.clone())
    };

    // Collect (PerimeterLoop, polygon) pairs.
    let mut loops: Vec<(PerimeterLoop, Vec<Vec2>)> = Vec::new();
    if outer_loop.len() >= 3 {
        loops.push((PerimeterLoop::Outer, outer_loop));
    }
    for (i, h) in boundary.holes.iter().enumerate() {
        if h.len() >= 3 {
            loops.push((PerimeterLoop::Hole { index: i }, h.clone()));
        }
    }

    for (loop_id, poly) in &loops {
        let n = poly.len();
        let mut cum: Vec<f64> = Vec::with_capacity(n + 1);
        cum.push(0.0);
        for i in 0..n {
            let a = poly[i];
            let b = poly[(i + 1) % n];
            cum.push(cum[i] + (b - a).length());
        }
        let total = *cum.last().unwrap();
        if total < 1e-9 {
            continue;
        }

        let mut entries = Vec::new();
        for (vi, &v) in graph.vertices.iter().enumerate() {
            let mut best: Option<(f64, f64)> = None;
            for i in 0..n {
                let a = poly[i];
                let b = poly[(i + 1) % n];
                let seg = b - a;
                let seg_len_sq = seg.x * seg.x + seg.y * seg.y;
                if seg_len_sq < 1e-12 {
                    continue;
                }
                let t = (v - a).dot(seg) / seg_len_sq;
                if t < -0.01 || t > 1.01 {
                    continue;
                }
                let t_c = t.max(0.0).min(1.0);
                let proj = a + seg * t_c;
                let dist = (v - proj).length();
                if dist > PERIM_VTX_ON_LOOP_TOL {
                    continue;
                }
                let arc = (cum[i] + t_c * (cum[i + 1] - cum[i])) / total;
                if best.is_none() || dist < best.unwrap().1 {
                    best = Some((arc, dist));
                }
            }
            if let Some((arc, _)) = best {
                entries.push((arc, vi));
            }
        }
        entries.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        out.insert(
            *loop_id,
            PerimeterLookupEntry { entries, total_length: total },
        );
    }

    out
}

/// Find the graph vertex nearest the given arc-length on `loop_`, within
/// world-space tolerance `world_tol`. Arc-distance wrapping across the
/// 0↔1 seam is handled.
fn resolve_perimeter_vertex(
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    loop_id: PerimeterLoop,
    arc: f64,
    world_tol: f64,
) -> Option<usize> {
    let entry = perim_lookup.get(&loop_id)?;
    if entry.total_length < 1e-9 {
        return None;
    }
    let tol = world_tol / entry.total_length;
    let mut best: Option<(f64, usize)> = None;
    for &(e_arc, vi) in &entry.entries {
        let mut d = (e_arc - arc).abs();
        if d > 0.5 {
            d = 1.0 - d;
        }
        if d > tol {
            continue;
        }
        if best.is_none() || d < best.unwrap().0 {
            best = Some((d, vi));
        }
    }
    best.map(|(_, vi)| vi)
}

/// Find the sub-edges of `loop_` whose arc range contains `arc`. Handles
/// wrap-around for sub-edges that span the 0↔1 seam.
fn resolve_perimeter_edges_containing(
    graph: &DriveAisleGraph,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    loop_id: PerimeterLoop,
    arc: f64,
) -> Option<Vec<usize>> {
    let entry = perim_lookup.get(&loop_id)?;
    let v_to_arc: std::collections::HashMap<usize, f64> =
        entry.entries.iter().map(|&(a, v)| (v, a)).collect();

    let mut hits = Vec::new();
    const TOL: f64 = 0.001;
    for (i, edge) in graph.edges.iter().enumerate() {
        let (Some(&as_), Some(&ae)) =
            (v_to_arc.get(&edge.start), v_to_arc.get(&edge.end))
        else {
            continue;
        };
        let lo = as_.min(ae);
        let hi = as_.max(ae);
        // If the naive span is > 0.5, the edge wraps across the seam —
        // its actual arc range is [hi, 1] ∪ [0, lo].
        let contains = if (hi - lo) > 0.5 {
            arc >= hi - TOL || arc <= lo + TOL
        } else {
            lo - TOL <= arc && arc <= hi + TOL
        };
        if contains {
            hits.push(i);
        }
    }
    if hits.is_empty() { None } else { Some(hits) }
}

/// Apply a `TrafficDirection` to the perimeter sub-edge(s) containing
/// `arc`, aligning each edge's stored `start→end` with the carrier's
/// canonical +arc direction (or its reverse, per `traffic`).
fn apply_perimeter_direction(
    graph: &mut DriveAisleGraph,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    loop_id: PerimeterLoop,
    arc: f64,
    traffic: TrafficDirection,
) -> bool {
    let Some(hits) = resolve_perimeter_edges_containing(graph, perim_lookup, loop_id, arc) else {
        return false;
    };
    if hits.is_empty() {
        return false;
    }
    let Some(entry) = perim_lookup.get(&loop_id) else { return false };
    let v_to_arc: std::collections::HashMap<usize, f64> =
        entry.entries.iter().map(|&(a, v)| (v, a)).collect();

    let want_canonical = matches!(
        traffic,
        TrafficDirection::OneWay | TrafficDirection::TwoWayOriented
    );
    let aisle_dir = match traffic {
        TrafficDirection::OneWay | TrafficDirection::OneWayReverse => AisleDirection::OneWay,
        _ => AisleDirection::TwoWayOriented,
    };
    for i in hits {
        let (Some(&as_), Some(&ae)) = (
            v_to_arc.get(&graph.edges[i].start),
            v_to_arc.get(&graph.edges[i].end),
        ) else {
            continue;
        };
        let naive_diff = ae - as_;
        // Canonical direction = +arc. If the naive diff is > 0.5 in
        // magnitude, the edge wraps the 0↔1 seam; a wrapping edge with
        // start-arc > end-arc is actually going +arc (crossing the seam).
        let edge_aligned_with_canonical = if naive_diff.abs() > 0.5 {
            naive_diff < 0.0
        } else {
            naive_diff > 0.0
        };
        if edge_aligned_with_canonical != want_canonical {
            let tmp = graph.edges[i].start;
            graph.edges[i].start = graph.edges[i].end;
            graph.edges[i].end = tmp;
        }
        graph.edges[i].direction = aisle_dir.clone();
    }
    true
}

/// Find the drive-line sub-edges whose stored t-range contains the given
/// `t` value, using `splice_lookup`. Returns graph edge indices.
fn resolve_drive_line_edges_containing(
    graph: &DriveAisleGraph,
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    drive_line_id: u32,
    t: f64,
    pitch: f64,
) -> Option<Vec<usize>> {
    let line_len = line_lengths.get(&drive_line_id).copied()?;
    if line_len < 1e-9 {
        return None;
    }
    let entries = splice_lookup.get(&drive_line_id)?;
    let tol_t = pitch / line_len;
    let mut v_to_t: std::collections::HashMap<usize, f64> =
        std::collections::HashMap::with_capacity(entries.len());
    for &(et, vi) in entries {
        v_to_t.insert(vi, et);
    }
    let mut hits = Vec::new();
    for (i, ei) in graph.edges.iter().enumerate() {
        let (Some(&ts), Some(&te)) = (v_to_t.get(&ei.start), v_to_t.get(&ei.end)) else {
            continue;
        };
        let lo = ts.min(te);
        let hi_ = ts.max(te);
        if lo - tol_t <= t && t <= hi_ + tol_t {
            hits.push(i);
        }
    }
    if hits.is_empty() { None } else { Some(hits) }
}

/// Apply spatial annotations to a resolved graph. Each annotation carries a
/// target in one substrate's own coord system (grid / drive-line / perimeter)
/// plus an intent (delete-vertex / delete-edge / direction). Resolution is
/// substrate-local; single dormancy rule — any referenced substrate or stop
/// missing from the current graph makes the annotation dormant this regen.
fn apply_annotations(
    graph: &mut DriveAisleGraph,
    annotations: &[Annotation],
    regions: &[ResolvedRegion],
    splice_anchors: &[Option<(u32, f64)>],
    drive_lines: &[DriveLine],
    boundary: &Polygon,
    params: &ParkingParams,
) -> Vec<usize> {
    if annotations.is_empty() {
        return Vec::new();
    }

    let abstract_lookup = build_abstract_vertex_lookup(graph, regions);
    let splice_lookup = build_splice_lookup(splice_anchors);
    let line_lengths: std::collections::HashMap<u32, f64> = drive_lines
        .iter()
        .map(|dl| (dl.id, (dl.end - dl.start).length()))
        .collect();
    let pitch = params.stall_pitch();

    let has_perim = annotations.iter().any(|a| match a {
        Annotation::DeleteVertex { target } | Annotation::DeleteEdge { target } => {
            matches!(target, Target::Perimeter { .. })
        }
        Annotation::Direction { target, .. } => matches!(target, Target::Perimeter { .. }),
    });
    let perim_lookup = if has_perim {
        build_perimeter_lookup(graph, boundary, params)
    } else {
        std::collections::HashMap::new()
    };

    let mut dormant: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();

    // First pass: directions.
    for (ann_idx, ann) in annotations.iter().enumerate() {
        let Annotation::Direction { target, traffic } = ann else { continue };
        let ok = apply_direction_target(
            graph, target, *traffic, regions,
            &splice_lookup, &line_lengths, &perim_lookup, pitch,
        );
        if !ok {
            dormant.insert(ann_idx);
        }
    }

    // Second pass: collect deletions.
    let mut vertices_to_remove = std::collections::HashSet::new();
    let mut edges_to_remove = std::collections::HashSet::new();

    for (ann_idx, ann) in annotations.iter().enumerate() {
        match ann {
            Annotation::DeleteVertex { target } => {
                let vi = resolve_target_vertex(
                    target, &abstract_lookup, &splice_lookup, &line_lengths, &perim_lookup, pitch,
                );
                match vi {
                    Some(v) => { vertices_to_remove.insert(v); }
                    None => { dormant.insert(ann_idx); }
                }
            }
            Annotation::DeleteEdge { target } => {
                let hits = resolve_target_edges(
                    target, graph, regions,
                    &splice_lookup, &line_lengths, &perim_lookup, pitch,
                );
                match hits {
                    Some(hs) if !hs.is_empty() => {
                        for h in hs { edges_to_remove.insert(h); }
                    }
                    _ => { dormant.insert(ann_idx); }
                }
            }
            Annotation::Direction { .. } => {} // handled in first pass
        }
    }

    // Apply deletions: remove edges incident to deleted vertices, then
    // remove explicitly deleted edges. Rebuild with compacted indices.
    if !vertices_to_remove.is_empty() || !edges_to_remove.is_empty() {
        // Also remove edges touching deleted vertices.
        for (i, edge) in graph.edges.iter().enumerate() {
            if vertices_to_remove.contains(&edge.start) || vertices_to_remove.contains(&edge.end) {
                edges_to_remove.insert(i);
            }
        }

        // Filter edges.
        let new_edges: Vec<AisleEdge> = graph.edges.iter().enumerate()
            .filter(|(i, _)| !edges_to_remove.contains(i))
            .map(|(_, e)| e.clone())
            .collect();
        graph.edges = new_edges;

        // Simplify: any vertex left with degree-2 collinear is a vestige
        // (e.g., a former row×column intersection where the row was
        // deleted). Merge its two edges into one and orphan the vertex.
        // Run iteratively in case merges create new degree-2 candidates.
        // Skip perimeter vertices (boundary inset corners are kept).
        simplify_collinear_degree2(graph);

        // Note: we don't compact vertices (remove unused ones) since vertex
        // indices are referenced by edges and the perimeter_vertex_count.
        // Orphaned vertices are harmless.
    }

    dormant.into_iter().collect()
}

/// Iteratively merge degree-2 collinear interior vertices. After deleting
/// a row of edges, the vertices where the row crossed columns become
/// degree-2 (just the two column halves). Those aren't real junctions —
/// the column should be a single straight edge through them. Replaces
/// the two incident edges with one spanning edge and leaves the vertex
/// orphaned (rendering filters orphans).
fn simplify_collinear_degree2(graph: &mut crate::types::DriveAisleGraph) {
    use std::collections::HashMap;
    let perim_n = graph.perim_vertex_count;
    loop {
        // Adjacency: vertex_idx -> Vec<edge_idx>. Deduplicate
        // bidirectional edges by canonical (min, max) endpoint key.
        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut canon_seen: std::collections::HashSet<(usize, usize)> =
            std::collections::HashSet::new();
        let mut canon_edges: Vec<usize> = Vec::new();
        for (i, e) in graph.edges.iter().enumerate() {
            let key = (e.start.min(e.end), e.start.max(e.end));
            if canon_seen.insert(key) {
                canon_edges.push(i);
                adj.entry(e.start).or_default().push(i);
                adj.entry(e.end).or_default().push(i);
            }
        }

        let mut merged_any = false;
        for (&vi, edges) in &adj {
            if vi < perim_n { continue; } // keep boundary vertices
            if edges.len() != 2 { continue; }
            let e1 = &graph.edges[edges[0]];
            let e2 = &graph.edges[edges[1]];
            let other1 = if e1.start == vi { e1.end } else { e1.start };
            let other2 = if e2.start == vi { e2.end } else { e2.start };
            if other1 == other2 { continue; } // degenerate loop
            let p = graph.vertices[vi];
            let a = graph.vertices[other1];
            let b = graph.vertices[other2];
            // Check collinearity: a→p direction matches p→b direction.
            let d1x = p.x - a.x;
            let d1y = p.y - a.y;
            let d2x = b.x - p.x;
            let d2y = b.y - p.y;
            let l1 = (d1x * d1x + d1y * d1y).sqrt();
            let l2 = (d2x * d2x + d2y * d2y).sqrt();
            if l1 < 1e-9 || l2 < 1e-9 { continue; }
            let dot = (d1x * d2x + d1y * d2y) / (l1 * l2);
            if dot < 0.999 { continue; } // not collinear
            // Direction must also match (no U-turns).
            // Already implied by dot>0.999.

            // Merge: remove e1 and e2 (and their reverse twins, if any),
            // add a single edge other1 → other2 with merged direction.
            let key1 = (e1.start.min(e1.end), e1.start.max(e1.end));
            let key2 = (e2.start.min(e2.end), e2.start.max(e2.end));
            let direction = e1.direction.clone();
            let interior = e1.interior && e2.interior;
            let width = e1.width.max(e2.width);
            let new_edges: Vec<crate::types::AisleEdge> = graph.edges.iter()
                .filter(|e| {
                    let k = (e.start.min(e.end), e.start.max(e.end));
                    k != key1 && k != key2
                })
                .cloned()
                .collect();
            graph.edges = new_edges;
            graph.edges.push(crate::types::AisleEdge {
                start: other1,
                end: other2,
                width,
                direction,
                interior,
            });
            merged_any = true;
            break; // restart since adjacency changed
        }
        if !merged_any { break; }
    }
}

/// Build a per-drive-line lookup of (t, vertex_idx) sorted by t. Vertices
/// without a splice anchor are ignored.
fn build_splice_lookup(
    splice_anchors: &[Option<(u32, f64)>],
) -> std::collections::HashMap<u32, Vec<(f64, usize)>> {
    let mut lookup: std::collections::HashMap<u32, Vec<(f64, usize)>> =
        std::collections::HashMap::new();
    for (vi, anchor) in splice_anchors.iter().enumerate() {
        if let Some((id, t)) = anchor {
            lookup.entry(*id).or_default().push((*t, vi));
        }
    }
    for entries in lookup.values_mut() {
        entries.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    }
    lookup
}

/// Find the splice vertex on `drive_line_id` whose stored `t` is closest
/// to the requested `t`, accepting it only if the world-space distance
/// (|Δt| × line_length) is within `pitch`. Returns the graph vertex index.
fn resolve_splice_vertex(
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    drive_line_id: u32,
    t: f64,
    pitch: f64,
) -> Option<usize> {
    let entries = splice_lookup.get(&drive_line_id)?;
    let line_len = *line_lengths.get(&drive_line_id)?;
    if line_len < 1e-9 { return None; }
    let tol_t = pitch / line_len;
    let mut best: Option<(f64, usize)> = None;
    for &(et, vi) in entries {
        let d = (et - t).abs();
        if d > tol_t { continue; }
        if best.is_none() || d < best.unwrap().0 {
            best = Some((d, vi));
        }
    }
    best.map(|(_, vi)| vi)
}

/// Resolve a single-vertex `Target` to a graph vertex index.
fn resolve_target_vertex(
    target: &Target,
    abstract_lookup: &std::collections::HashMap<(RegionId, i32, i32), usize>,
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    pitch: f64,
) -> Option<usize> {
    match target {
        Target::Grid { region, axis, coord, range } => {
            let (s1, s2) = range.as_ref()?;
            let p1 = grid_stop_to_other_axis(s1)?;
            let p2 = grid_stop_to_other_axis(s2)?;
            if p1 != p2 {
                return None; // span, not a point
            }
            // axis=X: line x=coord, varying y → (xi=coord, yi=p1).
            // axis=Y: line y=coord, varying x → (xi=p1, yi=coord).
            let (xi, yi) = if *axis == Axis::X { (*coord, p1) } else { (p1, *coord) };
            abstract_lookup.get(&(*region, xi, yi)).copied()
        }
        Target::DriveLine { id, t } => {
            resolve_splice_vertex(splice_lookup, line_lengths, *id, *t, pitch)
        }
        Target::Perimeter { loop_, arc } => {
            resolve_perimeter_vertex(perim_lookup, *loop_, *arc, pitch)
        }
    }
}

/// Resolve a `Target` to a set of graph edge indices.
fn resolve_target_edges(
    target: &Target,
    graph: &DriveAisleGraph,
    regions: &[ResolvedRegion],
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    pitch: f64,
) -> Option<Vec<usize>> {
    match target {
        Target::Grid { region, axis, coord, range } => {
            resolve_grid_edges(graph, regions, *region, *axis, *coord, range)
        }
        Target::DriveLine { id, t } => {
            resolve_drive_line_edges_containing(graph, splice_lookup, line_lengths, *id, *t, pitch)
        }
        Target::Perimeter { loop_, arc } => {
            resolve_perimeter_edges_containing(graph, perim_lookup, *loop_, *arc)
        }
    }
}

/// Apply a `TrafficDirection` to every edge the target resolves to.
fn apply_direction_target(
    graph: &mut DriveAisleGraph,
    target: &Target,
    traffic: TrafficDirection,
    regions: &[ResolvedRegion],
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    pitch: f64,
) -> bool {
    match target {
        Target::Grid { region, axis, coord, range } => {
            let Some(hits) = resolve_grid_edges(graph, regions, *region, *axis, *coord, range) else {
                return false;
            };
            let Some(region_data) = regions.iter().find(|r| r.id == *region) else {
                return false;
            };
            let along_x = *axis == Axis::Y;
            apply_direction_to_edges(graph, &hits, 1.0, along_x, Some(&region_data.frame), traffic);
            true
        }
        Target::DriveLine { id, t } => apply_drive_line_direction(
            graph,
            splice_lookup,
            line_lengths,
            *id,
            *t,
            pitch,
            traffic,
        ),
        Target::Perimeter { loop_, arc } => apply_perimeter_direction(
            graph,
            perim_lookup,
            *loop_,
            *arc,
            traffic,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clip::point_in_polygon;

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
