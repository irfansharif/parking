//! Spatial annotation resolution and application (DESIGN.md §1.5 + §3.1
//! Phase B step 3).
//!
//! Annotations are substrate-keyed edits that survive graph regen:
//! `DeleteVertex`, `DeleteEdge`, and `Direction`. Each carries a
//! `Target` addressing a vertex or sub-edge in one of three substrates
//! — the per-region abstract grid, a drive line, or a perimeter loop
//! — and this module is responsible for resolving each target to
//! concrete graph elements on the current regen, then applying the
//! intent.
//!
//! Single dormancy rule: any target whose substrate or stop is missing
//! in the current graph is recorded as dormant and left untouched.
//!
//! Public surface:
//!
//!   resolve_regions_for_frames — per-region `AbstractFrame` lookup
//!   ResolvedRegion              — id + frame + clip polygon triple
//!   apply_annotations           — resolve and mutate in one pass

use crate::geom::clip::point_in_polygon;
use crate::geom::inset::{inset_polygon, signed_area};
use crate::graph::{decompose_regions, Region};
use crate::types::*;

fn ensure_ccw(mut poly: Vec<Vec2>) -> Vec<Vec2> {
    if signed_area(&poly) < 0.0 {
        poly.reverse();
    }
    poly
}

/// One region's abstract frame plus its clip polygon — what
/// `apply_annotations` needs to resolve Grid-target annotations.
pub(crate) struct ResolvedRegion {
    id: RegionId,
    clip_poly: Vec<Vec2>,
    frame: AbstractFrame,
}

/// Build the (id, frame, clip_poly) triples for the current generate
/// input. Mirrors the decomposition logic in the region_debug block
/// below but returns the frames for annotation resolution.
pub(crate) fn resolve_regions_for_frames(
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

struct PerimeterLookupEntry {
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
pub(crate) fn apply_annotations(
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
