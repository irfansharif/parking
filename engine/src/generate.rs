use crate::aisle_graph::{auto_generate, find_or_add_vertex, intersect_line_polygon, merge_with_auto, subtract_intervals};
use crate::face::generate_from_spines;
use crate::inset::{inset_polygon, signed_area};
use crate::types::*;

pub fn generate(input: GenerateInput) -> ParkingLayout {
    // First, resolve the aisle graph from manual + auto as usual.
    let mut graph = match input.aisle_graph {
        Some(manual) => merge_with_auto(manual, &input.boundary, &input.params),
        None => auto_generate(&input.boundary, &input.params),
    };

    // Then append drive line edges on top — they're additive and should not
    // cause auto edges to be filtered out.
    let drive_graph = clip_drive_lines(&input.drive_lines, &input.boundary, &input.params, &graph);
    if !drive_graph.vertices.is_empty() {
        graph = append_graph(graph, drive_graph);
    }

    // Apply spatial annotations to the resolved graph. Annotations are
    // matched by proximity so they survive graph regeneration.
    apply_annotations(&mut graph, &input.annotations, &input.params);

    let (stalls, aisle_polygons, spines, faces, miter_fills, skeleton_debug, islands) =
        generate_from_spines(&graph, &input.boundary, &input.params, &input.debug);

    let total = stalls.len();

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

    let stall_angle_rad = params.stall_angle_deg.to_radians();
    let effective_depth = params.stall_depth * stall_angle_rad.sin();
    // Use two-way width (widest) for clipping perimeter computation.
    let hw = params.aisle_width;
    let inset_d = effective_depth + hw;

    // Compute the same inset perimeter and expanded holes as auto_generate.
    let effective = if params.site_offset > 0.0 {
        let p = inset_polygon(&boundary.outer, params.site_offset);
        if p.is_empty() { return DriveAisleGraph { vertices: vec![], edges: vec![], perim_vertex_count: 0 }; }
        p
    } else {
        boundary.outer.clone()
    };
    let effective = ensure_ccw(effective);
    let outer_loop = inset_polygon(&effective, inset_d);
    if outer_loop.is_empty() {
        return DriveAisleGraph { vertices: vec![], edges: vec![], perim_vertex_count: 0 };
    }

    let mut hole_loops: Vec<Vec<Vec2>> = Vec::new();
    for hole in &boundary.holes {
        let expanded = inset_polygon(hole, -inset_d);
        if !expanded.is_empty() {
            hole_loops.push(expanded);
        }
    }

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

/// Apply spatial annotations to a resolved graph. Each annotation is matched
/// to the edge that passes closest to the annotation point (point-to-segment
/// distance), with a tight threshold so annotations only bind to edges that
/// geometrically pass through them.
fn apply_annotations(
    graph: &mut DriveAisleGraph,
    annotations: &[Annotation],
    params: &ParkingParams,
) {
    if annotations.is_empty() {
        return;
    }

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

    let one_way_hw = params.aisle_width / 2.0;
    // Tight threshold: annotation must be essentially on top of the edge.
    let max_dist = 3.0;

    for ann in annotations {
        match ann {
            Annotation::OneWay { midpoint, travel_dir } => {
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

                // Orient the matched edge so start→end aligns with travel_dir.
                let edge = &graph.edges[matched_idx];
                let s = edge.start;
                let e = edge.end;
                let edge_dir = (graph.vertices[e] - graph.vertices[s]).normalize();
                let needs_flip = edge_dir.dot(*travel_dir) < 0.0;

                // Apply to the matched edge and its reverse.
                for i in 0..graph.edges.len() {
                    let ei = &graph.edges[i];
                    let is_forward = ei.start == s && ei.end == e;
                    let is_reverse = ei.start == e && ei.end == s;
                    if !is_forward && !is_reverse {
                        continue;
                    }
                    graph.edges[i].direction = AisleDirection::OneWay;
                    graph.edges[i].width = one_way_hw;
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
    }
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
