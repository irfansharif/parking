use crate::aisle_graph::{auto_generate, compute_inset_d, decompose_regions, derive_raw_holes, derive_raw_outer, find_or_add_vertex, intersect_line_polygon, merge_with_auto, subtract_intervals, Region};
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

    // Extract separator lines from pinned drive lines: (hole_index, vertex_index, boundary_endpoint).
    let separator_lines: Vec<(usize, usize, Vec2)> = input.drive_lines.iter()
        .filter_map(|dl| dl.hole_pin.as_ref().map(|p| (p.hole_index, p.vertex_index, dl.end)))
        .collect();

    // First, resolve the aisle graph from manual + auto as usual.
    let mut graph = match input.aisle_graph.clone() {
        Some(manual) => merge_with_auto(manual, &input.boundary, &input.params),
        None => auto_generate(
            &input.boundary,
            &input.params,
            &separator_lines,
            &input.region_overrides,
            input.debug.use_abstract_stamp,
        ),
    };

    // Append drive line edges on top — they're additive and should
    // not cause auto edges to be filtered out. Drive-line vertices
    // are placed at arbitrary world coordinates (not on the integer
    // grid), so they're naturally invisible to abstract annotation
    // lookups that only match integer grid points. Legacy proximity
    // annotations intentionally run AFTER this so they can target
    // drive-line edges.
    let drive_graph = clip_drive_lines(&input.drive_lines, &input.boundary, &input.params, &graph);
    if !drive_graph.vertices.is_empty() {
        graph = append_graph(graph, drive_graph);
    }

    // Build the per-region frame list once, for both abstract annotation
    // lookup and the debug info returned to the UI below. Decomposition is
    // cheap enough to compute here again (auto_generate also runs it
    // internally) — a future cleanup could share the result.
    let resolved_regions: Vec<ResolvedRegion> =
        resolve_regions_for_frames(&input, &separator_lines);

    // Apply spatial annotations to the resolved graph. Abstract
    // annotations resolve by integer grid lookup into the region
    // frames; the legacy variants fall through to proximity matching
    // and can target both grid edges and drive-line-spliced edges.
    let dormant_annotations =
        apply_annotations(&mut graph, &input.annotations, &resolved_regions);

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

        let mut region_list = if !separator_lines.is_empty() {
            decompose_regions(&outer_loop, &hole_loops, &separator_lines)
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
fn clip_drive_lines(
    lines: &[DriveLine],
    boundary: &Polygon,
    params: &ParkingParams,
    auto_graph: &DriveAisleGraph,
) -> DriveAisleGraph {
    if lines.is_empty() {
        return DriveAisleGraph { vertices: vec![], edges: vec![], perim_vertex_count: 0 };
    }

    let hw = params.aisle_width;

    // boundary.outer is the aisle-edge perimeter — use directly (with site_offset).
    let outer_loop = if params.site_offset > 0.0 {
        let p = inset_polygon(&boundary.outer, params.site_offset);
        if p.is_empty() { return DriveAisleGraph { vertices: vec![], edges: vec![], perim_vertex_count: 0 }; }
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
                let via = find_or_add_vertex(&mut vertices, pa, 1.0);
                let vib = find_or_add_vertex(&mut vertices, pb, 1.0);
                if via != vib {
                    edges.push(AisleEdge { start: via, end: vib, width: hw, interior: true, direction: AisleDirection::TwoWay });
                    edges.push(AisleEdge { start: vib, end: via, width: hw, interior: true, direction: AisleDirection::TwoWay });
                }
            }
        }
    }

    DriveAisleGraph {
        vertices,
        edges,
        perim_vertex_count: 0,
    }
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
fn append_graph(a: DriveAisleGraph, b: DriveAisleGraph) -> DriveAisleGraph {
    let vtx_tolerance = 1.0;
    let edge_tolerance = 2.0;
    let mut vertices = a.vertices;
    let base_edge_count = a.edges.len();
    let mut edges = a.edges;

    let mut b_to_merged: Vec<usize> = Vec::with_capacity(b.vertices.len());
    for bv in &b.vertices {
        // First try: merge with an existing vertex.
        let existing = vertices
            .iter()
            .position(|mv| (*mv - *bv).length() < vtx_tolerance);
        if let Some(mi) = existing {
            b_to_merged.push(mi);
            continue;
        }

        // Second try: find an a-edge this vertex lies on, and split it.
        let new_vi = vertices.len();
        vertices.push(*bv);

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

    DriveAisleGraph {
        vertices,
        edges,
        perim_vertex_count: a.perim_vertex_count,
    }
}

/// One region's abstract frame plus its clip polygon — what
/// `apply_annotations` needs to resolve AbstractDelete* annotations.
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
    separator_lines: &[(usize, usize, Vec2)],
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

    let mut region_list = if !separator_lines.is_empty() {
        decompose_regions(&outer_loop, &hole_loops, separator_lines)
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
            let frame = AbstractFrame::region(&input.params, r.aisle_angle_deg);
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

/// Apply an AbstractOneWay / AbstractTwoWayOriented direction to every
/// graph edge connecting the two abstract grid points. Flips
/// `start`/`end` so the stored ordering points from `a` to `b`,
/// matching the user's declared travel direction. Returns true if at
/// least one graph edge was touched — false means the annotation
/// target didn't resolve and the annotation should be reported as
/// dormant.
fn apply_abstract_direction(
    graph: &mut DriveAisleGraph,
    abstract_lookup: &std::collections::HashMap<(RegionId, i32, i32), usize>,
    region: RegionId,
    a: (i32, i32),
    b: (i32, i32),
    direction: AisleDirection,
) -> bool {
    let Some(&via) = abstract_lookup.get(&(region, a.0, a.1)) else { return false };
    let Some(&vib) = abstract_lookup.get(&(region, b.0, b.1)) else { return false };
    let mut touched = false;
    for i in 0..graph.edges.len() {
        let ei = &graph.edges[i];
        let is_forward = ei.start == via && ei.end == vib;
        let is_reverse = ei.start == vib && ei.end == via;
        if !is_forward && !is_reverse {
            continue;
        }
        graph.edges[i].direction = direction.clone();
        if is_reverse {
            graph.edges[i].start = via;
            graph.edges[i].end = vib;
        }
        touched = true;
    }
    touched
}

/// Apply spatial annotations to a resolved graph. Each annotation is matched
/// to the edge that passes closest to the annotation point (point-to-segment
/// distance), with a tight threshold so annotations only bind to edges that
/// geometrically pass through them. Returns the indices (into the
/// `annotations` slice) of annotations whose abstract handle didn't
/// resolve to any graph feature — the UI surfaces these as "dormant."
fn apply_annotations(
    graph: &mut DriveAisleGraph,
    annotations: &[Annotation],
    regions: &[ResolvedRegion],
) -> Vec<usize> {
    if annotations.is_empty() {
        return Vec::new();
    }

    // Pre-compute the abstract-grid vertex lookup lazily — we only
    // need it if there's at least one abstract annotation in the list.
    let has_abstract = annotations.iter().any(|a| {
        matches!(
            a,
            Annotation::AbstractDeleteVertex { .. }
                | Annotation::AbstractDeleteEdge { .. }
                | Annotation::AbstractOneWay { .. }
                | Annotation::AbstractTwoWayOriented { .. }
        )
    });
    let abstract_lookup = if has_abstract {
        build_abstract_vertex_lookup(graph, regions)
    } else {
        std::collections::HashMap::new()
    };

    // Precompute edge endpoints and directions (one per undirected edge).
    struct EdgeInfo {
        start: Vec2,
        end: Vec2,
        dir: Vec2,
        idx: usize,
    }
    let mut edge_infos: Vec<EdgeInfo> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for (i, edge) in graph.edges.iter().enumerate() {
        let key = if edge.start < edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        if !seen.insert(key) {
            continue;
        }
        let s = graph.vertices[edge.start];
        let e = graph.vertices[edge.end];
        let dir = (e - s).normalize();
        edge_infos.push(EdgeInfo { start: s, end: e, dir, idx: i });
    }

    // Tight threshold: annotation must be essentially on top of the edge.
    let max_dist = 3.0;

    // Indices of annotations whose abstract handle didn't resolve this
    // pass. Populated by the two match blocks below and returned at
    // the bottom so the caller can surface them as dormant.
    let mut dormant: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();

    for (ann_idx, ann) in annotations.iter().enumerate() {
        match ann {
            Annotation::OneWay { midpoint, travel_dir, chain: expand_chain } => {
                // Find the edge that passes closest to the annotation point.
                let mut best: Option<(usize, f64)> = None;
                for info in &edge_infos {
                    // Edge must be roughly parallel to travel direction.
                    if info.dir.dot(*travel_dir).abs() < 0.7 {
                        continue;
                    }
                    let dist = point_to_segment_dist(*midpoint, info.start, info.end);
                    if dist > max_dist {
                        continue;
                    }
                    if best.is_none() || dist < best.unwrap().1 {
                        best = Some((info.idx, dist));
                    }
                }
                let Some((matched_idx, _dist)) = best else { continue };

                // Expand to the full collinear chain, or stay on single segment.
                let chain = if *expand_chain {
                    find_collinear_chain(graph, matched_idx)
                } else {
                    vec![matched_idx]
                };

                // Apply direction to every edge in the chain.
                for &chain_idx in &chain {
                    let edge = &graph.edges[chain_idx];
                    let s = edge.start;
                    let e = edge.end;
                    let edge_dir = (graph.vertices[e] - graph.vertices[s]).normalize();
                    let needs_flip = edge_dir.dot(*travel_dir) < 0.0;

                    for i in 0..graph.edges.len() {
                        let ei = &graph.edges[i];
                        let is_forward = ei.start == s && ei.end == e;
                        let is_reverse = ei.start == e && ei.end == s;
                        if !is_forward && !is_reverse {
                            continue;
                        }
                        graph.edges[i].direction = AisleDirection::OneWay;
                        if needs_flip && is_forward {
                            graph.edges[i].start = e;
                            graph.edges[i].end = s;
                        } else if needs_flip && is_reverse {
                            graph.edges[i].start = s;
                            graph.edges[i].end = e;
                        }
                    }
                }
            }
            Annotation::TwoWayOriented { midpoint, travel_dir, chain: expand_chain } => {
                // Same edge-matching logic as OneWay, but we preserve full
                // aisle width and set TwoWayOriented direction.
                let mut best: Option<(usize, f64)> = None;
                for info in &edge_infos {
                    if info.dir.dot(*travel_dir).abs() < 0.7 {
                        continue;
                    }
                    let dist = point_to_segment_dist(*midpoint, info.start, info.end);
                    if dist > max_dist {
                        continue;
                    }
                    if best.is_none() || dist < best.unwrap().1 {
                        best = Some((info.idx, dist));
                    }
                }
                let Some((matched_idx, _dist)) = best else { continue };

                let chain = if *expand_chain {
                    find_collinear_chain(graph, matched_idx)
                } else {
                    vec![matched_idx]
                };

                for &chain_idx in &chain {
                    let edge = &graph.edges[chain_idx];
                    let s = edge.start;
                    let e = edge.end;
                    let edge_dir = (graph.vertices[e] - graph.vertices[s]).normalize();
                    let needs_flip = edge_dir.dot(*travel_dir) < 0.0;

                    for i in 0..graph.edges.len() {
                        let ei = &graph.edges[i];
                        let is_forward = ei.start == s && ei.end == e;
                        let is_reverse = ei.start == e && ei.end == s;
                        if !is_forward && !is_reverse {
                            continue;
                        }
                        graph.edges[i].direction = AisleDirection::TwoWayOriented;
                        // Keep full width — don't halve like OneWay.
                        if needs_flip && is_forward {
                            graph.edges[i].start = e;
                            graph.edges[i].end = s;
                        } else if needs_flip && is_reverse {
                            graph.edges[i].start = s;
                            graph.edges[i].end = e;
                        }
                    }
                }
            }
            Annotation::AbstractOneWay { region, xa, ya, xb, yb } => {
                let touched = apply_abstract_direction(
                    graph,
                    &abstract_lookup,
                    *region,
                    (*xa, *ya),
                    (*xb, *yb),
                    AisleDirection::OneWay,
                );
                if !touched {
                    dormant.insert(ann_idx);
                }
            }
            Annotation::AbstractTwoWayOriented { region, xa, ya, xb, yb } => {
                let touched = apply_abstract_direction(
                    graph,
                    &abstract_lookup,
                    *region,
                    (*xa, *ya),
                    (*xb, *yb),
                    AisleDirection::TwoWayOriented,
                );
                if !touched {
                    dormant.insert(ann_idx);
                }
            }
            _ => {} // DeleteVertex/DeleteEdge/AbstractDelete* handled in second pass
        }
    }

    // Second pass: collect vertices and edges to delete.
    let max_vtx_dist = 5.0;
    let mut vertices_to_remove = std::collections::HashSet::new();
    let mut edges_to_remove = std::collections::HashSet::new();

    for (ann_idx, ann) in annotations.iter().enumerate() {
        match ann {
            Annotation::DeleteVertex { point } => {
                // Find nearest vertex.
                let mut best: Option<(usize, f64)> = None;
                for (i, v) in graph.vertices.iter().enumerate() {
                    let dist = (*v - *point).length();
                    if dist < max_vtx_dist && (best.is_none() || dist < best.unwrap().1) {
                        best = Some((i, dist));
                    }
                }
                if let Some((vi, _)) = best {
                    vertices_to_remove.insert(vi);
                }
            }
            Annotation::DeleteEdge { midpoint, edge_dir, chain: expand_chain } => {
                let mut best: Option<(usize, f64)> = None;
                for info in &edge_infos {
                    if info.dir.dot(*edge_dir).abs() < 0.7 {
                        continue;
                    }
                    let dist = point_to_segment_dist(*midpoint, info.start, info.end);
                    if dist > max_dist {
                        continue;
                    }
                    if best.is_none() || dist < best.unwrap().1 {
                        best = Some((info.idx, dist));
                    }
                }
                if let Some((matched_idx, _)) = best {
                    let chain = if *expand_chain {
                        find_collinear_chain(graph, matched_idx)
                    } else {
                        vec![matched_idx]
                    };
                    for &ci in &chain {
                        let edge = &graph.edges[ci];
                        let s = edge.start;
                        let e = edge.end;
                        // Mark both forward and reverse edges.
                        for (i, ei) in graph.edges.iter().enumerate() {
                            if (ei.start == s && ei.end == e) || (ei.start == e && ei.end == s) {
                                edges_to_remove.insert(i);
                            }
                        }
                    }
                }
            }
            Annotation::AbstractDeleteVertex { region, xi, yi } => {
                if let Some(&vi) = abstract_lookup.get(&(*region, *xi, *yi)) {
                    vertices_to_remove.insert(vi);
                } else {
                    dormant.insert(ann_idx);
                }
            }
            Annotation::AbstractDeleteEdge { region, xa, ya, xb, yb } => {
                let Some(&via) = abstract_lookup.get(&(*region, *xa, *ya)) else {
                    dormant.insert(ann_idx);
                    continue;
                };
                let Some(&vib) = abstract_lookup.get(&(*region, *xb, *yb)) else {
                    dormant.insert(ann_idx);
                    continue;
                };
                // Find all graph edges that connect via↔vib in either
                // direction (the stamp emits bidirectional edges, and
                // drive-line splicing may introduce mid-edge vertices we
                // don't yet handle). Delete each matching edge.
                let mut touched = false;
                for (i, ei) in graph.edges.iter().enumerate() {
                    if (ei.start == via && ei.end == vib)
                        || (ei.start == vib && ei.end == via)
                    {
                        edges_to_remove.insert(i);
                        touched = true;
                    }
                }
                if !touched {
                    dormant.insert(ann_idx);
                }
            }
            _ => {}
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

        // Note: we don't compact vertices (remove unused ones) since vertex
        // indices are referenced by edges and the perimeter_vertex_count.
        // Orphaned vertices are harmless.
    }

    dormant.into_iter().collect()
}

/// Find all edges in the same collinear chain as the seed edge. Two edges
/// chain if they share a vertex and are collinear (direction dot > 0.99).
/// Returns deduplicated edge indices (one per undirected edge pair).
fn find_collinear_chain(graph: &DriveAisleGraph, seed_idx: usize) -> Vec<usize> {
    let seed = &graph.edges[seed_idx];
    let seed_dir = (graph.vertices[seed.end] - graph.vertices[seed.start]).normalize();

    // Build adjacency: vertex → list of (edge_idx, other_vertex, direction).
    let mut adj: std::collections::HashMap<usize, Vec<(usize, usize, Vec2)>> =
        std::collections::HashMap::new();
    let mut seen_edges = std::collections::HashSet::new();
    for (i, edge) in graph.edges.iter().enumerate() {
        let key = if edge.start < edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        if !seen_edges.insert(key) {
            continue;
        }
        let dir = (graph.vertices[edge.end] - graph.vertices[edge.start]).normalize();
        adj.entry(edge.start).or_default().push((i, edge.end, dir));
        adj.entry(edge.end).or_default().push((i, edge.start, Vec2::new(-dir.x, -dir.y)));
    }

    // BFS/walk from seed in both directions along collinear edges.
    let mut chain = vec![seed_idx];
    let mut visited = std::collections::HashSet::new();
    visited.insert(seed_idx);

    let mut frontier = vec![seed.start, seed.end];
    while let Some(v) = frontier.pop() {
        if let Some(neighbors) = adj.get(&v) {
            for &(ei, other_v, dir) in neighbors {
                if visited.contains(&ei) {
                    continue;
                }
                // Must be collinear with seed direction.
                if dir.dot(seed_dir).abs() < 0.99 {
                    continue;
                }
                visited.insert(ei);
                chain.push(ei);
                frontier.push(other_v);
            }
        }
    }

    chain
}

/// Perpendicular distance from point `p` to the line segment `a`–`b`.
fn point_to_segment_dist(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < 1e-12 {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    let proj = a + ab * t;
    (p - proj).length()
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
