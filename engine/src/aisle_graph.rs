use crate::clip::polygons_overlap;
use crate::face::corridor_polygon;
use crate::inset::{inset_polygon, signed_area};
use crate::types::*;

/// Compute the inset distance for a given set of parking parameters.
/// This is the offset from the aisle-edge ring to the raw building boundary,
/// or equivalently, the offset from the outer boundary to the perimeter loop.
pub fn compute_inset_d(params: &ParkingParams) -> f64 {
    let stall_angle_rad = params.stall_angle_deg.to_radians();
    let effective_depth = params.stall_depth * stall_angle_rad.sin()
        + stall_angle_rad.cos() * params.stall_width / 2.0;
    effective_depth + params.aisle_width
}

/// Expand the aisle-edge perimeter outward by `inset_d` (+ `site_offset`)
/// to recover the raw lot boundary for clipping and face extraction.
pub fn derive_raw_outer(outer: &[Vec2], inset_d: f64, site_offset: f64) -> Vec<Vec2> {
    // Expand outward by inset_d (negative inset = expand).
    let expanded = inset_polygon(outer, -inset_d);
    if expanded.is_empty() {
        return outer.to_vec();
    }
    // Then expand further by site_offset if any.
    if site_offset > 0.0 {
        let with_offset = inset_polygon(&expanded, -site_offset);
        if with_offset.is_empty() {
            return expanded;
        }
        return with_offset;
    }
    expanded
}

/// Shrink each aisle-edge ring inward by `inset_d` to recover the raw
/// building footprint. Rings that collapse to nothing are omitted.
pub fn derive_raw_holes(holes: &[Vec<Vec2>], inset_d: f64) -> Vec<Vec<Vec2>> {
    holes
        .iter()
        .filter_map(|ring| {
            let shrunk = inset_polygon(ring, inset_d);
            if shrunk.is_empty() {
                None
            } else {
                Some(shrunk)
            }
        })
        .collect()
}

/// Automatically generate a connected drive-aisle graph:
/// 1. Perimeter loop (single inset of outer boundary) — stalls face outward
/// 2. Hole loops (aisle-edge rings around each hole) — stalls face away from hole
/// 3. Interior parallel aisles at stall_angle_deg, clipped to perimeter minus holes
/// 4. Interior aisle endpoints spliced into perimeter edges for full connectivity
pub fn auto_generate(boundary: &Polygon, params: &ParkingParams) -> DriveAisleGraph {
    // aisle_width is one driving lane. Auto-generated edges are two-way
    // (two lanes), so their half-width is one full lane width.
    let hw = params.aisle_width;
    let inset_d = compute_inset_d(params);
    let effective_depth = inset_d - hw;
    let row_spacing = 2.0 * effective_depth + 2.0 * params.aisle_width;

    // boundary.outer is the aisle-edge perimeter. Apply site_offset if any.
    let outer_loop = if params.site_offset > 0.0 {
        let p = inset_polygon(&boundary.outer, params.site_offset);
        if p.is_empty() {
            return empty_graph();
        }
        ensure_ccw(p)
    } else {
        ensure_ccw(boundary.outer.clone())
    };

    let mut vertices: Vec<Vec2> = Vec::new();
    let mut edges: Vec<AisleEdge> = Vec::new();

    // Add perimeter polygon vertices.
    let perim_base = vertices.len();
    for v in &outer_loop {
        vertices.push(*v);
    }
    let perim_n = outer_loop.len();

    // 2. Hole loops: boundary.holes are aisle-edge rings — use directly.
    let mut hole_loops: Vec<Vec<Vec2>> = Vec::new();
    let mut hole_bases: Vec<usize> = Vec::new();
    for hole in &boundary.holes {
        if hole.is_empty() {
            continue;
        }
        let base = vertices.len();
        for v in hole {
            vertices.push(*v);
        }
        hole_bases.push(base);
        hole_loops.push(hole.clone());
    }

    // Snap hole loop vertices onto nearby outer perimeter edges. When a hole
    // is close to the boundary, the expanded hole loop and the inset perimeter
    // produce near-duplicate parallel edges. Snapping collapses them.
    let snap_tol = hw; // aisle half-width — generous enough to catch overlaps.
    for hi in 0..hole_loops.len() {
        let base = hole_bases[hi];
        let hn = hole_loops[hi].len();
        for hvi in 0..hn {
            let hp = hole_loops[hi][hvi];
            // Find the closest outer perimeter edge.
            let mut best_dist = f64::INFINITY;
            let mut _best_edge = 0usize;
            let mut best_proj = hp;
            for pi in 0..perim_n {
                let pj = (pi + 1) % perim_n;
                let a = outer_loop[pi];
                let b = outer_loop[pj];
                let d = point_to_segment_dist_with_proj(hp, a, b);
                if d.0 < best_dist {
                    best_dist = d.0;
                    _best_edge = pi;
                    best_proj = d.1;
                }
            }
            if best_dist < snap_tol {
                // Snap: reuse or create a vertex at the projected point on the
                // perimeter edge. Update both the hole loop and the vertex list.
                let vi = find_or_add_vertex(&mut vertices, best_proj, 0.5);
                vertices[base + hvi] = best_proj;
                // If find_or_add_vertex returned a different index, record a
                // mapping so that hole edge building can remap.
                if vi != base + hvi {
                    // Point the hole vertex at the shared location.
                    vertices[base + hvi] = vertices[vi];
                }
                hole_loops[hi][hvi] = best_proj;
            }
        }
    }

    // Per-edge split tracking for hole loops (mirrors perim_splits).
    let mut hole_splits: Vec<Vec<Vec<(f64, usize)>>> = hole_loops
        .iter()
        .map(|hl| vec![Vec::new(); hl.len()])
        .collect();

    // 3. Interior parallel aisles.
    let angle_rad = params.aisle_angle_deg.to_radians();
    let aisle_dir = Vec2::new(angle_rad.cos(), angle_rad.sin());
    let perp_dir = Vec2::new(-angle_rad.sin(), angle_rad.cos());

    // Project perimeter vertices onto perpendicular direction.
    let min_proj = outer_loop
        .iter()
        .map(|v| v.dot(perp_dir))
        .fold(f64::INFINITY, f64::min);
    let max_proj = outer_loop
        .iter()
        .map(|v| v.dot(perp_dir))
        .fold(f64::NEG_INFINITY, f64::max);

    // Collect splits on the outer perimeter edges.
    // Each split: (edge_idx, t_along_edge, global_vertex_index).
    let mut perim_splits: Vec<Vec<(f64, usize)>> = vec![Vec::new(); perim_n];

    // Interior aisle edge pairs: (vertex_idx_start, vertex_idx_end).
    let mut interior_pairs: Vec<(usize, usize)> = Vec::new();

    // Align the aisle grid so that one line passes through aisle_offset
    // (the perpendicular projection of the drag anchor). When offset is 0,
    // the grid starts at min_proj + row_spacing as before.
    let grid_start = min_proj + row_spacing;
    let grid_end = max_proj - row_spacing;
    let first = if params.aisle_offset != 0.0 {
        // Shift grid_start so that (grid_start + k*row_spacing) == aisle_offset
        // for some integer k, keeping the result within [grid_start, grid_end].
        let rem = (params.aisle_offset - grid_start) % row_spacing;
        let aligned = grid_start + if rem < 0.0 { rem + row_spacing } else { rem };
        // If aligned is past grid_end, step back one row.
        if aligned > grid_end { aligned - row_spacing } else { aligned }
    } else {
        grid_start
    };

    let mut t = first;
    while t <= grid_end {
        let origin = perp_dir * t;

        // Intersect with outer perimeter.
        let outer_hits = intersect_line_polygon(origin, aisle_dir, &outer_loop);

        // Convert to intervals (enter/exit pairs).
        let mut intervals: Vec<(f64, f64)> = Vec::new();
        for pair in outer_hits.chunks(2) {
            if pair.len() == 2 {
                intervals.push((pair[0].t_line, pair[1].t_line));
            }
        }

        // Record ALL outer perimeter intersection points as splits (even if
        // holes later trim some segments — extra vertices on the perimeter
        // are harmless). Use find_or_add_vertex so that snapped hole vertices
        // and nearby split points get merged.
        for hit in &outer_hits {
            let pt = origin + aisle_dir * hit.t_line;
            let vi = find_or_add_vertex(&mut vertices, pt, 0.5);
            perim_splits[hit.edge_idx].push((hit.t_edge, vi));
        }

        // Subtract hole interiors from intervals and record intersection
        // points as hole edge splits (mirrors perim_splits).
        for (hi, hl) in hole_loops.iter().enumerate() {
            let hole_hits = intersect_line_polygon(origin, aisle_dir, hl);
            let mut hole_ivs: Vec<(f64, f64)> = Vec::new();
            for pair in hole_hits.chunks(2) {
                if pair.len() == 2 {
                    hole_ivs.push((pair[0].t_line, pair[1].t_line));
                }
            }
            for hit in &hole_hits {
                let pt = origin + aisle_dir * hit.t_line;
                let vi = find_or_add_vertex(&mut vertices, pt, 0.1);
                hole_splits[hi][hit.edge_idx].push((hit.t_edge, vi));
            }
            intervals = subtract_intervals(&intervals, &hole_ivs);
        }

        // Create interior aisle segments from surviving intervals.
        // Match each interval endpoint to the vertex we already created.
        // The vertices were created from outer_hits, so we need to find
        // or create vertices for each interval endpoint.
        for &(ta, tb) in &intervals {
            let pa = origin + aisle_dir * ta;
            let pb = origin + aisle_dir * tb;

            // Find existing vertex or create new one.
            let via = find_or_add_vertex(&mut vertices, pa, 0.1);
            let vib = find_or_add_vertex(&mut vertices, pb, 0.1);
            interior_pairs.push((via, vib));
        }

        t += row_spacing;
    }

    // 3b. Cross-aisles (perpendicular to main aisles).
    let mut cross_pairs: Vec<(usize, usize)> = Vec::new();
    let stall_pitch = params.stall_width / params.stall_angle_deg.to_radians().sin();
    let col_spacing = params.cross_aisle_max_run * stall_pitch;
    if col_spacing >= row_spacing {

        // Project perimeter onto aisle_dir to find extent.
        let min_along = outer_loop
            .iter()
            .map(|v| v.dot(aisle_dir))
            .fold(f64::INFINITY, f64::min);
        let max_along = outer_loop
            .iter()
            .map(|v| v.dot(aisle_dir))
            .fold(f64::NEG_INFINITY, f64::max);

        let col_start = min_along + col_spacing;
        let col_end = max_along - col_spacing;

        let mut s = col_start;
        while s <= col_end {
            let origin = aisle_dir * s;

            let outer_hits = intersect_line_polygon(origin, perp_dir, &outer_loop);

            // Record perimeter splits.
            for hit in &outer_hits {
                let pt = origin + perp_dir * hit.t_line;
                let vi = vertices.len();
                vertices.push(pt);
                perim_splits[hit.edge_idx].push((hit.t_edge, vi));
            }

            let mut intervals: Vec<(f64, f64)> = Vec::new();
            for pair in outer_hits.chunks(2) {
                if pair.len() == 2 {
                    intervals.push((pair[0].t_line, pair[1].t_line));
                }
            }

            // Subtract hole interiors.
            for hl in &hole_loops {
                let hole_hits = intersect_line_polygon(origin, perp_dir, hl);
                let mut hole_ivs: Vec<(f64, f64)> = Vec::new();
                for pair in hole_hits.chunks(2) {
                    if pair.len() == 2 {
                        hole_ivs.push((pair[0].t_line, pair[1].t_line));
                    }
                }
                intervals = subtract_intervals(&intervals, &hole_ivs);
            }

            for &(ta, tb) in &intervals {
                let pa = origin + perp_dir * ta;
                let pb = origin + perp_dir * tb;
                let via = find_or_add_vertex(&mut vertices, pa, 0.1);
                let vib = find_or_add_vertex(&mut vertices, pb, 0.1);
                cross_pairs.push((via, vib));
            }

            s += col_spacing;
        }

        // Split main aisle segments at intersections with cross-aisle segments
        // (and vice versa) so the graph is fully connected at every crossing.
        split_at_crossings(&mut vertices, &mut interior_pairs, &mut cross_pairs, 0.1);
    }

    // 4. Build perimeter edges with splits spliced in.
    for i in 0..perim_n {
        let j = (i + 1) % perim_n;
        let vi_start = perim_base + i;
        let vi_end = perim_base + j;

        // Sort splits on this edge by parameter.
        perim_splits[i].sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let mut prev = vi_start;
        for &(_, split_vi) in &perim_splits[i] {
            edges.push(AisleEdge { start: prev, end: split_vi, width: hw, interior: false, direction: AisleDirection::TwoWay });
            edges.push(AisleEdge { start: split_vi, end: prev, width: hw, interior: false, direction: AisleDirection::TwoWay });
            prev = split_vi;
        }
        edges.push(AisleEdge { start: prev, end: vi_end, width: hw, interior: false, direction: AisleDirection::TwoWay });
        edges.push(AisleEdge { start: vi_end, end: prev, width: hw, interior: false, direction: AisleDirection::TwoWay });
    }

    // 5. Build hole loop edges with splits spliced in (mirrors perimeter).
    for (hi, hl) in hole_loops.iter().enumerate() {
        let base = hole_bases[hi];
        let hn = hl.len();
        for i in 0..hn {
            let j = (i + 1) % hn;
            let vi_start = base + i;
            let vi_end = base + j;

            hole_splits[hi][i].sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

            let mut prev = vi_start;
            for &(_, split_vi) in &hole_splits[hi][i] {
                edges.push(AisleEdge { start: prev, end: split_vi, width: hw, interior: false, direction: AisleDirection::TwoWay });
                edges.push(AisleEdge { start: split_vi, end: prev, width: hw, interior: false, direction: AisleDirection::TwoWay });
                prev = split_vi;
            }
            edges.push(AisleEdge { start: prev, end: vi_end, width: hw, interior: false, direction: AisleDirection::TwoWay });
            edges.push(AisleEdge { start: vi_end, end: prev, width: hw, interior: false, direction: AisleDirection::TwoWay });
        }
    }

    // 6. Build interior aisle edges (main + cross).
    for &(via, vib) in interior_pairs.iter().chain(cross_pairs.iter()) {
        edges.push(AisleEdge { start: via, end: vib, width: hw, interior: true, direction: AisleDirection::TwoWay });
        edges.push(AisleEdge { start: vib, end: via, width: hw, interior: true, direction: AisleDirection::TwoWay });
    }

    // 7. Hole-to-interior connectivity edges removed — they created
    // diagonal aisle rectangles at arbitrary angles, causing miter fill
    // artifacts, face slivers, and boundary misclassification around holes.
    if false {
    for &(via, vib) in interior_pairs.iter().chain(cross_pairs.iter()) {
        for &vi in &[via, vib] {
            let v = vertices[vi];
            let on_perim = perim_splits.iter().any(|splits| {
                splits.iter().any(|&(_, svi)| svi == vi)
            });
            if on_perim {
                continue;
            }
            let mut best_dist = f64::INFINITY;
            let mut best_vi = 0;
            for (hi, hl) in hole_loops.iter().enumerate() {
                let base = hole_bases[hi];
                for k in 0..hl.len() {
                    let d = (vertices[base + k] - v).length();
                    if d < best_dist {
                        best_dist = d;
                        best_vi = base + k;
                    }
                }
            }
            if best_dist < row_spacing {
                edges.push(AisleEdge { start: vi, end: best_vi, width: hw, interior: true, direction: AisleDirection::TwoWay });
                edges.push(AisleEdge { start: best_vi, end: vi, width: hw, interior: true, direction: AisleDirection::TwoWay });
            }
        }
    }
    } // end if false

    // Perimeter vertices are the first perim_n + all hole loop vertices.
    // Split vertices (interior aisle endpoints on the perimeter) are NOT
    // counted — the user needs to drag those to change the aisle angle.
    let total_perim = perim_n + hole_loops.iter().map(|h| h.len()).sum::<usize>();

    DriveAisleGraph { vertices, edges, perim_vertex_count: total_perim }
}

/// Merge a manually-edited graph with the auto-generated one. Auto edges whose
/// corridor overlaps any manual corridor are removed; the rest fill gaps.
pub fn merge_with_auto(
    manual: DriveAisleGraph,
    boundary: &Polygon,
    params: &ParkingParams,
) -> DriveAisleGraph {
    let auto = auto_generate(boundary, params);

    // Compute corridor polygons for manual edges.
    let manual_corridors: Vec<Vec<Vec2>> = manual
        .edges
        .iter()
        .map(|e| corridor_polygon(&manual.vertices, e))
        .collect();

    // Filter auto edges: keep those whose corridor doesn't overlap any manual corridor.
    let mut surviving_auto_edges: Vec<AisleEdge> = Vec::new();
    let mut auto_vertex_used = vec![false; auto.vertices.len()];

    for e in &auto.edges {
        let auto_corridor = corridor_polygon(&auto.vertices, e);
        let overlaps = manual_corridors
            .iter()
            .any(|mc| polygons_overlap(&auto_corridor, mc));
        if !overlaps {
            surviving_auto_edges.push(e.clone());
            auto_vertex_used[e.start] = true;
            auto_vertex_used[e.end] = true;
        }
    }

    // Merge vertices: manual first, then surviving auto vertices.
    let mut merged_vertices: Vec<Vec2> = manual.vertices.clone();
    let mut merged_edges: Vec<AisleEdge> = manual.edges.clone();

    let tolerance = 1.0;
    let mut auto_to_merged: Vec<Option<usize>> = vec![None; auto.vertices.len()];

    for (ai, av) in auto.vertices.iter().enumerate() {
        if !auto_vertex_used[ai] {
            continue;
        }
        let existing = merged_vertices
            .iter()
            .position(|mv| (*mv - *av).length() < tolerance);
        if let Some(mi) = existing {
            auto_to_merged[ai] = Some(mi);
        } else {
            auto_to_merged[ai] = Some(merged_vertices.len());
            merged_vertices.push(*av);
        }
    }

    for e in &surviving_auto_edges {
        if let (Some(s), Some(t)) = (auto_to_merged[e.start], auto_to_merged[e.end]) {
            merged_edges.push(AisleEdge {
                start: s,
                end: t,
                width: e.width,
                interior: e.interior,
                direction: e.direction.clone(),
            });
        }
    }

    DriveAisleGraph {
        vertices: merged_vertices,
        edges: merged_edges,
        perim_vertex_count: auto.perim_vertex_count,
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn empty_graph() -> DriveAisleGraph {
    DriveAisleGraph {
        vertices: Vec::new(),
        edges: Vec::new(),
        perim_vertex_count: 0,
    }
}

fn ensure_ccw(mut poly: Vec<Vec2>) -> Vec<Vec2> {
    if signed_area(&poly) < 0.0 {
        poly.reverse();
    }
    poly
}


/// Distance from point to segment, plus the closest point on the segment.
fn point_to_segment_dist_with_proj(p: Vec2, a: Vec2, b: Vec2) -> (f64, Vec2) {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < 1e-12 {
        return ((p - a).length(), a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    let proj = a + ab * t;
    ((p - proj).length(), proj)
}

/// Find an existing vertex within tolerance, or add a new one.
pub(crate) fn find_or_add_vertex(vertices: &mut Vec<Vec2>, point: Vec2, tolerance: f64) -> usize {
    for (i, v) in vertices.iter().enumerate() {
        if (*v - point).length() < tolerance {
            return i;
        }
    }
    let idx = vertices.len();
    vertices.push(point);
    idx
}

/// Split two sets of segment pairs at their mutual intersections so the graph
/// is connected at every crossing. Each pair (a, b) is a segment from
/// vertices[a] to vertices[b]. When a segment from `set_a` crosses one from
/// `set_b`, a shared vertex is inserted and both segments are replaced by their
/// two sub-segments.
fn split_at_crossings(
    vertices: &mut Vec<Vec2>,
    set_a: &mut Vec<(usize, usize)>,
    set_b: &mut Vec<(usize, usize)>,
    tol: f64,
) {
    // Collect crossing splits: for each segment index, the t-values where it
    // must be split and the vertex index at that point.
    let mut splits_a: Vec<Vec<(f64, usize)>> = vec![Vec::new(); set_a.len()];
    let mut splits_b: Vec<Vec<(f64, usize)>> = vec![Vec::new(); set_b.len()];

    for (ai, &(a0, a1)) in set_a.iter().enumerate() {
        let pa = vertices[a0];
        let da = vertices[a1] - vertices[a0];
        let la = da.length();
        if la < 1e-12 { continue; }

        for (bi, &(b0, b1)) in set_b.iter().enumerate() {
            let pb = vertices[b0];
            let db = vertices[b1] - vertices[b0];
            let lb = db.length();
            if lb < 1e-12 { continue; }

            let denom = da.cross(db);
            if denom.abs() < 1e-12 { continue; }

            let h = pb - pa;
            let ta = h.cross(db) / denom;
            let tb = h.cross(da) / denom;

            if ta > tol / la && ta < 1.0 - tol / la && tb > tol / lb && tb < 1.0 - tol / lb {
                let pt = pa + da * ta;
                let vi = find_or_add_vertex(vertices, pt, tol);
                splits_a[ai].push((ta, vi));
                splits_b[bi].push((tb, vi));
            }
        }
    }

    // Apply splits to produce sub-segments.
    fn apply_splits(
        pairs: &[(usize, usize)],
        splits: &mut [Vec<(f64, usize)>],
    ) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for (i, &(s, e)) in pairs.iter().enumerate() {
            if splits[i].is_empty() {
                out.push((s, e));
                continue;
            }
            splits[i].sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            let mut prev = s;
            for &(_, vi) in &splits[i] {
                out.push((prev, vi));
                prev = vi;
            }
            out.push((prev, e));
        }
        out
    }

    *set_a = apply_splits(set_a, &mut splits_a);
    *set_b = apply_splits(set_b, &mut splits_b);
}

// ---------------------------------------------------------------------------
// Line–polygon intersection
// ---------------------------------------------------------------------------

pub(crate) struct Hit {
    pub t_line: f64,    // parameter along the line
    pub edge_idx: usize, // which polygon edge
    pub t_edge: f64,    // parameter along that edge (0..1)
}

/// Intersect an infinite line (origin + t*dir) with a polygon.
/// Returns hits sorted by t_line.
pub(crate) fn intersect_line_polygon(origin: Vec2, dir: Vec2, polygon: &[Vec2]) -> Vec<Hit> {
    let n = polygon.len();
    let mut hits = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        let p = polygon[i];
        let e = polygon[j] - polygon[i];
        let denom = dir.cross(e);
        if denom.abs() < 1e-12 {
            continue;
        }
        let h = p - origin;
        let t = h.cross(e) / denom;
        let s = -dir.cross(h) / denom;
        if s >= 0.0 && s < 1.0 {
            hits.push(Hit {
                t_line: t,
                edge_idx: i,
                t_edge: s,
            });
        }
    }
    hits.sort_by(|a, b| a.t_line.partial_cmp(&b.t_line).unwrap());
    hits
}

/// Subtract hole intervals from base intervals.
/// Both inputs must be sorted and non-overlapping.
pub(crate) fn subtract_intervals(base: &[(f64, f64)], holes: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut result = Vec::new();
    for &(a, b) in base {
        let mut segments = vec![(a, b)];
        for &(ha, hb) in holes {
            let mut next = Vec::new();
            for &(s, e) in &segments {
                if hb <= s || ha >= e {
                    // No overlap.
                    next.push((s, e));
                } else {
                    if s < ha {
                        next.push((s, ha));
                    }
                    if hb < e {
                        next.push((hb, e));
                    }
                }
            }
            segments = next;
        }
        result.extend(segments);
    }
    result
}
