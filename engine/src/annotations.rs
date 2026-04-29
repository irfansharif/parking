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
use crate::geom::poly::ensure_ccw;
use crate::graph::{aisle_edge_perim, decompose_regions, Region};
use crate::types::*;

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
    // Region IDs come from the raw sketch boundary so they're invariant
    // to params that change the inset distance. Each region's clip_poly
    // is in raw-sketch coordinates.
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
            partitioning_lines,
            input.params.aisle_angle,
            input.params.aisle_offset,
        )
    } else {
        vec![]
    };
    if region_list.is_empty() {
        region_list.push(Region {
            id: RegionId::single_region_fallback(),
            clip_poly: input.boundary.outer.clone(),
            aisle_angle: input.params.aisle_angle,
            aisle_offset: input.params.aisle_offset,
        });
    }
    for ov in &input.region_overrides {
        if let Some(r) = region_list.iter_mut().find(|r| r.id == ov.region_id) {
            if let Some(a) = ov.aisle_angle {
                r.aisle_angle = a;
            }
            if let Some(o) = ov.aisle_offset {
                r.aisle_offset = o;
            }
        }
    }
    region_list
        .into_iter()
        .map(|r| {
            let frame = AbstractFrame::region(&input.params, r.aisle_angle, r.aisle_offset);
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
/// Returns an error string explaining why if the target addresses a
/// vertex (range = Some((s, s))), if the range's stops can't be resolved,
/// or if no edges match.
fn resolve_grid_edges(
    graph: &DriveAisleGraph,
    regions: &[ResolvedRegion],
    region: RegionId,
    axis: Axis,
    coord: i32,
    range: &Option<(GridStop, GridStop)>,
) -> Result<Vec<usize>, String> {
    let region_data = regions.iter().find(|r| r.id == region).ok_or_else(|| {
        format!("region {region:?} not in current decomposition")
    })?;
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
                return Err(format!(
                    "grid range collapses to a single lattice point ({pa}); use vertex addressing"
                ));
            }
            (pa.min(pb), pa.max(pb))
        }
    };

    let mut hits = Vec::new();
    for (i, edge) in graph.edges.iter().enumerate() {
        // Grid targets address only the interior abstract grid. Perimeter
        // edges have their own addressing scheme via Target::Perimeter, and
        // letting them match here would let a wide grid-axis range punch
        // holes in the perimeter aisle when the boundary is grid-aligned.
        if !edge.interior { continue; }
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
    if hits.is_empty() {
        Err(format!(
            "no graph edge along grid line {axis:?}={fixed} in region {region:?}"
        ))
    } else {
        Ok(hits)
    }
}

/// Convert a GridStop to its coordinate along the grid line's varying axis.
/// Only `Lattice` is supported in this prototype; `CrossesDriveLine` and
/// `CrossesPerimeter` stops return an error (dormant) until their
/// resolution is implemented.
fn grid_stop_to_other_axis(s: &GridStop) -> Result<i32, String> {
    match s {
        GridStop::Lattice { other } => Ok(*other),
        GridStop::CrossesDriveLine { .. } => {
            Err("grid stop kind CrossesDriveLine not yet supported".to_string())
        }
        GridStop::CrossesPerimeter { .. } => {
            Err("grid stop kind CrossesPerimeter not yet supported".to_string())
        }
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
    traffic: AisleDirection,
) {
    // TwoWayReverse keeps edge orientation untouched and just tags
    // the edge — the per-face flip in placement does the work.
    if matches!(traffic, AisleDirection::TwoWayReverse) {
        for &i in hits {
            graph.edges[i].direction = Some(AisleDirection::TwoWayReverse);
        }
        return;
    }
    let want_canonical = matches!(traffic, AisleDirection::OneWay);
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
        graph.edges[i].direction = Some(if edge_aligned_with_canonical == want_canonical {
            AisleDirection::OneWay
        } else {
            AisleDirection::OneWayReverse
        });
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
    traffic: AisleDirection,
) -> Result<(), String> {
    let hits = resolve_drive_line_edges_containing(
        graph, splice_lookup, line_lengths, drive_line_id, t, pitch,
    )?;
    if hits.is_empty() {
        return Err(format!(
            "no drive-line {drive_line_id} sub-edge contains t={t:.4}"
        ));
    }
    if matches!(traffic, AisleDirection::TwoWayReverse) {
        for i in hits {
            graph.edges[i].direction = Some(AisleDirection::TwoWayReverse);
        }
        return Ok(());
    }
    let want_canonical = matches!(traffic, AisleDirection::OneWay);
    // For drive-line edges we orient using the splice-anchored t of each endpoint.
    let entries = splice_lookup
        .get(&drive_line_id)
        .ok_or_else(|| format!("drive line {drive_line_id} has no splice anchors"))?;
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
        graph.edges[i].direction = Some(if edge_aligned_with_canonical == want_canonical {
            AisleDirection::OneWay
        } else {
            AisleDirection::OneWayReverse
        });
    }
    Ok(())
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
    /// The loop polygon the graph perim was built against (the
    /// aisle-edge inset for the outer; the hole sketch directly for
    /// holes). Used as the projection target when converting a
    /// sketch-edge anchor to an arc on this loop.
    poly: Vec<Vec2>,
    /// Maximum perpendicular distance allowed when projecting an
    /// evaluated sketch position onto `poly`. For the outer loop the
    /// graph perim sits on the inset (~`inset_d` from the deed line),
    /// so this is `inset_d` plus arc/discretization slack. For hole
    /// loops the sketch *is* the aisle-edge ring (graph perim lives
    /// on it directly), so just arc slack.
    sketch_to_loop_tol: f64,
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

    // Outer loop = aisle-edge perim (where graph perim vertices live).
    // Falls back to the raw sketch only when the inset would collapse.
    // Holes are used directly (the hole sketch IS the aisle-edge ring).
    let outer = aisle_edge_perim(boundary, params);
    let outer_collapsed = outer.is_empty();
    let outer_loop = if outer_collapsed {
        ensure_ccw(boundary.outer.clone())
    } else {
        ensure_ccw(outer)
    };

    // Tolerance for projecting an evaluated sketch position onto each
    // loop. The arc slack accommodates chord deflection on
    // discretized arc edges; the inset distance covers the
    // perpendicular gap between the sketch deed line and the
    // aisle-edge ring on the outer loop.
    let inset_d = crate::graph::compute_inset_d(params) + params.site_offset;
    let arc_slack = params.arc_discretize_tolerance + 1.0;
    let outer_tol = if outer_collapsed { arc_slack } else { inset_d + arc_slack };

    let mut loops: Vec<(PerimeterLoop, Vec<Vec2>, f64)> = Vec::new();
    if outer_loop.len() >= 3 {
        loops.push((PerimeterLoop::Outer, outer_loop, outer_tol));
    }
    for (i, h) in boundary.holes.iter().enumerate() {
        if h.len() >= 3 {
            loops.push((PerimeterLoop::Hole { index: i }, h.clone(), arc_slack));
        }
    }

    for (loop_id, poly, sketch_to_loop_tol) in &loops {
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
            PerimeterLookupEntry {
                entries,
                total_length: total,
                poly: poly.clone(),
                sketch_to_loop_tol: *sketch_to_loop_tol,
            },
        );
    }

    out
}

/// Convert a sketch-edge anchor `(start, end, t)` to a normalized arc
/// length on `entry.poly` (the loop the graph perim was built on).
///
/// Evaluates the sketch position from the boundary's raw vertices/ids
/// (arc-aware when the edge has stored bulge data) and projects onto
/// the loop. Returns an error if the sketch edge can't be addressed
/// in the current sketch (vertex deleted or edge split — annotation
/// goes dormant).
fn perim_target_to_arc(
    entry: &PerimeterLookupEntry,
    boundary: &Polygon,
    loop_id: PerimeterLoop,
    start: VertexId,
    end: VertexId,
    t: f64,
) -> Result<f64, String> {
    // Try arc-aware evaluation first (handles bulged sketch edges);
    // fall back to chord-chain interpolation for straight or
    // pre-discretized inputs.
    let world = crate::resolve::evaluate_perim_edge_arc(boundary, &loop_id, start, end, t)
        .or_else(|| {
            let (poly, ids) = match loop_id {
                PerimeterLoop::Outer => (
                    boundary.outer.as_slice(),
                    boundary.outer_ids.as_slice(),
                ),
                PerimeterLoop::Hole { index } => {
                    let h = boundary.holes.get(index)?;
                    let i = boundary
                        .hole_ids
                        .get(index)
                        .map(|v| v.as_slice())
                        .unwrap_or(&[]);
                    (h.as_slice(), i)
                }
            };
            crate::resolve::evaluate_perim_edge(poly, ids, start, end, t)
        })
        .ok_or_else(|| {
            format!(
                "perimeter edge {start:?}→{end:?} not addressable (vertex deleted or edge split)"
            )
        })?;
    crate::resolve::project_onto_loop(world, &entry.poly, entry.sketch_to_loop_tol).ok_or_else(
        || format!("perimeter projection failed for edge {start:?}→{end:?} at t={t:.4}"),
    )
}

/// Find the graph vertex nearest the addressed sketch edge anchor
/// `(start, end, t)` on `loop_`, within world-space tolerance
/// `world_tol`. Arc-distance wrapping across the 0↔1 seam is handled.
fn resolve_perimeter_vertex(
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    boundary: &Polygon,
    loop_id: PerimeterLoop,
    start: VertexId,
    end: VertexId,
    t: f64,
    world_tol: f64,
) -> Result<usize, String> {
    let entry = perim_lookup
        .get(&loop_id)
        .ok_or_else(|| format!("perimeter loop {loop_id:?} not in current sketch"))?;
    if entry.total_length < 1e-9 {
        return Err(format!("perimeter loop {loop_id:?} has zero length"));
    }
    let arc = perim_target_to_arc(entry, boundary, loop_id, start, end, t)?;
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
    best.map(|(_, vi)| vi).ok_or_else(|| {
        format!(
            "no graph vertex on perimeter loop {loop_id:?} within {world_tol} of arc {arc:.4}"
        )
    })
}

/// Find the sub-edges of `loop_` whose arc range contains the
/// addressed sketch edge anchor `(start, end, t)`. Handles
/// wrap-around for sub-edges that span the 0↔1 seam.
fn resolve_perimeter_edges_containing(
    graph: &DriveAisleGraph,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    boundary: &Polygon,
    loop_id: PerimeterLoop,
    start: VertexId,
    end: VertexId,
    t: f64,
) -> Result<Vec<usize>, String> {
    let entry = perim_lookup
        .get(&loop_id)
        .ok_or_else(|| format!("perimeter loop {loop_id:?} not in current sketch"))?;
    let arc = perim_target_to_arc(entry, boundary, loop_id, start, end, t)?;
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
    if hits.is_empty() {
        Err(format!(
            "no graph edge on perimeter loop {loop_id:?} contains arc derived from edge {start:?}→{end:?} at t={t:.4}"
        ))
    } else {
        Ok(hits)
    }
}

/// Apply a `AisleDirection` to the perimeter sub-edge(s) containing
/// the addressed sketch edge anchor `(start, end, t)`, aligning each
/// edge's stored `start→end` with the carrier's canonical +arc
/// direction (or its reverse, per `traffic`).
fn apply_perimeter_direction(
    graph: &mut DriveAisleGraph,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    boundary: &Polygon,
    loop_id: PerimeterLoop,
    start: VertexId,
    end: VertexId,
    t: f64,
    traffic: AisleDirection,
) -> Result<(), String> {
    let hits =
        resolve_perimeter_edges_containing(graph, perim_lookup, boundary, loop_id, start, end, t)?;
    if hits.is_empty() {
        return Err(format!(
            "no graph edge on perimeter loop {loop_id:?} contains arc derived from edge {start:?}→{end:?} at t={t:.4}"
        ));
    }
    let entry = perim_lookup
        .get(&loop_id)
        .ok_or_else(|| format!("perimeter loop {loop_id:?} not in current sketch"))?;
    let v_to_arc: std::collections::HashMap<usize, f64> =
        entry.entries.iter().map(|&(a, v)| (v, a)).collect();

    // TwoWay variants — see apply_direction_to_edges. Tag the edges
    // (without flipping start/end) so placement can flip stall lean
    // for the *Reverse variant.
    if matches!(traffic, AisleDirection::TwoWayReverse) {
        for i in hits {
            graph.edges[i].direction = Some(AisleDirection::TwoWayReverse);
        }
        return Ok(());
    }
    let want_canonical = matches!(traffic, AisleDirection::OneWay);
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
        graph.edges[i].direction = Some(if edge_aligned_with_canonical == want_canonical {
            AisleDirection::OneWay
        } else {
            AisleDirection::OneWayReverse
        });
    }
    Ok(())
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
) -> Result<Vec<usize>, String> {
    let line_len = line_lengths
        .get(&drive_line_id)
        .copied()
        .ok_or_else(|| format!("drive line {drive_line_id} not present in current sketch"))?;
    if line_len < 1e-9 {
        return Err(format!("drive line {drive_line_id} has zero length"));
    }
    let entries = splice_lookup
        .get(&drive_line_id)
        .ok_or_else(|| format!("drive line {drive_line_id} has no splice anchors"))?;
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
    if hits.is_empty() {
        Err(format!(
            "no drive-line {drive_line_id} sub-edge contains t={t:.4}"
        ))
    } else {
        Ok(hits)
    }
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
) -> Vec<DormantAnnotation> {
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

    // index → reason. BTreeMap keeps the output sorted by index. The
    // first failure for a given annotation wins (the only way to record
    // a second one would be an annotation appearing in both passes,
    // which doesn't happen — Direction is first-pass-only and the
    // delete variants are second-pass-only).
    let mut dormant: std::collections::BTreeMap<usize, String> =
        std::collections::BTreeMap::new();

    // First pass: directions.
    for (ann_idx, ann) in annotations.iter().enumerate() {
        let Annotation::Direction { target, traffic } = ann else { continue };
        if let Err(reason) = apply_direction_target(
            graph, target, *traffic, regions,
            &splice_lookup, &line_lengths, &perim_lookup, boundary, pitch,
        ) {
            dormant.insert(ann_idx, reason);
        }
    }

    // Second pass: collect deletions.
    let mut vertices_to_remove = std::collections::HashSet::new();
    let mut edges_to_remove = std::collections::HashSet::new();

    for (ann_idx, ann) in annotations.iter().enumerate() {
        match ann {
            Annotation::DeleteVertex { target } => {
                match resolve_target_vertex(
                    target, &abstract_lookup, &splice_lookup, &line_lengths, &perim_lookup, boundary, pitch,
                ) {
                    Ok(v) => { vertices_to_remove.insert(v); }
                    Err(reason) => { dormant.insert(ann_idx, reason); }
                }
            }
            Annotation::DeleteEdge { target } => {
                match resolve_target_edges(
                    target, graph, regions,
                    &splice_lookup, &line_lengths, &perim_lookup, boundary, pitch,
                ) {
                    Ok(hs) if !hs.is_empty() => {
                        for h in hs { edges_to_remove.insert(h); }
                    }
                    Ok(_) => {
                        dormant.insert(
                            ann_idx,
                            "target resolved to no graph edges".to_string(),
                        );
                    }
                    Err(reason) => { dormant.insert(ann_idx, reason); }
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

    debug_assert!(
        edges_are_unique(graph),
        "DriveAisleGraph carries duplicate undirected edges after apply_annotations"
    );

    dormant
        .into_iter()
        .map(|(index, reason)| DormantAnnotation { index, reason })
        .collect()
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
        // Adjacency: vertex_idx -> Vec<edge_idx>. Each undirected edge
        // contributes one entry at each endpoint.
        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        for (i, e) in graph.edges.iter().enumerate() {
            adj.entry(e.start).or_default().push(i);
            adj.entry(e.end).or_default().push(i);
        }

        // Iterate vertex indices in sorted order so the first eligible
        // merge is deterministic. HashMap iteration is randomized per
        // process, so without sorting the merge sequence — and the
        // resulting edge order in `graph.edges` — would vary across
        // runs even for identical input.
        let mut vis: Vec<usize> = adj.keys().copied().collect();
        vis.sort_unstable();

        let mut merged_any = false;
        for vi in vis {
            let edges = &adj[&vi];
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

            // Merge: remove e1 and e2, add a single edge
            // other1 → other2 with merged direction.
            let i1 = edges[0];
            let i2 = edges[1];
            let direction = e1.direction.clone();
            let interior = e1.interior && e2.interior;
            let width = e1.width.max(e2.width);
            let new_edges: Vec<crate::types::AisleEdge> = graph.edges.iter()
                .enumerate()
                .filter(|(i, _)| *i != i1 && *i != i2)
                .map(|(_, e)| e.clone())
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

/// Debug invariant: every aisle edge has a unique unordered endpoint
/// pair. Catches a producer that accidentally re-introduces the
/// forward+reverse duplication this pipeline used to carry.
fn edges_are_unique(graph: &crate::types::DriveAisleGraph) -> bool {
    let mut seen = std::collections::HashSet::new();
    for e in &graph.edges {
        if e.start == e.end {
            continue;
        }
        let key = (e.start.min(e.end), e.start.max(e.end));
        if !seen.insert(key) {
            return false;
        }
    }
    true
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
) -> Result<usize, String> {
    let entries = splice_lookup
        .get(&drive_line_id)
        .ok_or_else(|| format!("drive line {drive_line_id} has no splice anchors"))?;
    let line_len = *line_lengths
        .get(&drive_line_id)
        .ok_or_else(|| format!("drive line {drive_line_id} not present in current sketch"))?;
    if line_len < 1e-9 {
        return Err(format!("drive line {drive_line_id} has zero length"));
    }
    let tol_t = pitch / line_len;
    let mut best: Option<(f64, usize)> = None;
    for &(et, vi) in entries {
        let d = (et - t).abs();
        if d > tol_t { continue; }
        if best.is_none() || d < best.unwrap().0 {
            best = Some((d, vi));
        }
    }
    best.map(|(_, vi)| vi).ok_or_else(|| {
        format!(
            "no splice vertex on drive line {drive_line_id} within {pitch} of t={t:.4}"
        )
    })
}

/// Resolve a single-vertex `Target` to a graph vertex index.
fn resolve_target_vertex(
    target: &Target,
    abstract_lookup: &std::collections::HashMap<(RegionId, i32, i32), usize>,
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    boundary: &Polygon,
    pitch: f64,
) -> Result<usize, String> {
    match target {
        Target::Grid { region, axis, coord, range } => {
            let (s1, s2) = range
                .as_ref()
                .ok_or_else(|| "grid vertex target missing range".to_string())?;
            let p1 = grid_stop_to_other_axis(s1)?;
            let p2 = grid_stop_to_other_axis(s2)?;
            if p1 != p2 {
                return Err(format!(
                    "grid range covers a span ({p1}..{p2}), not a single vertex"
                ));
            }
            // axis=X: line x=coord, varying y → (xi=coord, yi=p1).
            // axis=Y: line y=coord, varying x → (xi=p1, yi=coord).
            let (xi, yi) = if *axis == Axis::X { (*coord, p1) } else { (p1, *coord) };
            abstract_lookup.get(&(*region, xi, yi)).copied().ok_or_else(|| {
                format!("no graph vertex at region {region:?} grid ({xi},{yi})")
            })
        }
        Target::DriveLine { id, t } => {
            resolve_splice_vertex(splice_lookup, line_lengths, *id, *t, pitch)
        }
        Target::Perimeter { loop_, start, end, t } => {
            resolve_perimeter_vertex(perim_lookup, boundary, *loop_, *start, *end, *t, pitch)
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
    boundary: &Polygon,
    pitch: f64,
) -> Result<Vec<usize>, String> {
    match target {
        Target::Grid { region, axis, coord, range } => {
            resolve_grid_edges(graph, regions, *region, *axis, *coord, range)
        }
        Target::DriveLine { id, t } => {
            resolve_drive_line_edges_containing(graph, splice_lookup, line_lengths, *id, *t, pitch)
        }
        Target::Perimeter { loop_, start, end, t } => {
            resolve_perimeter_edges_containing(graph, perim_lookup, boundary, *loop_, *start, *end, *t)
        }
    }
}

/// Apply a `AisleDirection` to every edge the target resolves to.
fn apply_direction_target(
    graph: &mut DriveAisleGraph,
    target: &Target,
    traffic: AisleDirection,
    regions: &[ResolvedRegion],
    splice_lookup: &std::collections::HashMap<u32, Vec<(f64, usize)>>,
    line_lengths: &std::collections::HashMap<u32, f64>,
    perim_lookup: &std::collections::HashMap<PerimeterLoop, PerimeterLookupEntry>,
    boundary: &Polygon,
    pitch: f64,
) -> Result<(), String> {
    match target {
        Target::Grid { region, axis, coord, range } => {
            let hits =
                resolve_grid_edges(graph, regions, *region, *axis, *coord, range)?;
            let region_data = regions
                .iter()
                .find(|r| r.id == *region)
                .ok_or_else(|| format!("region {region:?} not in current decomposition"))?;
            let along_x = *axis == Axis::Y;
            apply_direction_to_edges(graph, &hits, 1.0, along_x, Some(&region_data.frame), traffic);
            Ok(())
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
        Target::Perimeter { loop_, start, end, t } => apply_perimeter_direction(
            graph,
            perim_lookup,
            boundary,
            *loop_,
            *start,
            *end,
            *t,
            traffic,
        ),
    }
}
