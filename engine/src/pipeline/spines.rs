//! §3.5 Spine generation + post-processing.
//!
//!   compute_face_spines   — weighted straight-skeleton wavefront
//!                           extraction; emits one spine per surviving
//!                           aisle-facing edge.
//!   extend_spines_to_faces — colinear extension of interior spines to
//!                            face boundary (dual-clip validated).
//!   merge_collinear_spines / try_merge_spines
//!                         — join compatible spines that span face
//!                           boundaries along the same aisle edge.
//!   dedup_overlapping_spines
//!                         — trim the shorter of two collinear,
//!                           overlapping spines.

use crate::geom::inset::signed_area;
use crate::geom::poly::{clip_segment_to_face, simplify_contour};
use crate::pipeline::bays::normalize_face_winding;
use crate::pipeline::tagging::classify_face_edges_ext;
use crate::skeleton;
use crate::types::{
    DebugToggles, EdgeSource, FaceEdge, ParkingParams, SpineSegment, TaggedFace, Vec2,
};

/// Merge collinear spine segments that share an endpoint (within tolerance)
/// and have the same outward normal direction. This eliminates gaps between
/// stall strips that span multiple faces along the same aisle edge.
pub(crate) fn merge_collinear_spines(
    spines: Vec<SpineSegment>,
    tolerance: f64,
) -> Vec<SpineSegment> {
    if spines.is_empty() {
        return spines;
    }

    let mut merged = spines;
    let mut changed = true;

    while changed {
        changed = false;
        let mut result: Vec<SpineSegment> = Vec::new();
        let mut used = vec![false; merged.len()];

        for i in 0..merged.len() {
            if used[i] {
                continue;
            }

            let mut seg = merged[i].clone();
            used[i] = true;

            // Repeatedly try to extend this segment by merging with others.
            let mut extended = true;
            while extended {
                extended = false;
                for j in 0..merged.len() {
                    if used[j] {
                        continue;
                    }

                    if let Some(m) = try_merge_spines(&seg, &merged[j], tolerance) {
                        seg = m;
                        used[j] = true;
                        extended = true;
                        changed = true;
                    }
                }
            }

            result.push(seg);
        }

        merged = result;
    }

    merged
}

/// Try to merge two spine segments. Returns the merged segment if they are
/// collinear (directions within tolerance), have similar outward normals,
/// and share an endpoint.
pub(crate) fn try_merge_spines(
    a: &SpineSegment,
    b: &SpineSegment,
    tolerance: f64,
) -> Option<SpineSegment> {
    // Only merge spines of the same type (interior/perimeter).
    if a.is_interior != b.is_interior {
        return None;
    }

    // Only merge spines with compatible travel directions.
    match (&a.travel_dir, &b.travel_dir) {
        (None, None) => {}
        (Some(d1), Some(d2)) if d1.dot(*d2) > 0.99 => {}
        _ => return None,
    }

    let dir_a = (a.end - a.start).normalize();
    let dir_b = (b.end - b.start).normalize();

    // Must be collinear: directions parallel (dot ≈ ±1).
    if dir_a.dot(dir_b).abs() < 0.99 {
        return None;
    }

    // Must have (nearly) identical outward normals. Spines from the same
    // corridor edge have exactly equal normals; spines from different edges
    // at the same boundary corner differ by the angle between the edges.
    // A tight threshold (0.9999 ≈ 0.8°) prevents cross-edge merging that
    // would extend one edge's normal over another edge's territory.
    if a.outward_normal.dot(b.outward_normal) < 0.9999 {
        return None;
    }

    // Must share an endpoint (within tolerance).
    let pairs = [
        (a.end, b.start),   // a→b chain
        (a.start, b.end),   // b→a chain
        (a.end, b.end),     // both point away from junction
        (a.start, b.start), // both point toward junction
    ];

    for (p1, p2) in &pairs {
        if (*p1 - *p2).length() > tolerance {
            continue;
        }

        // Project all four endpoints onto the shared line direction
        // and take the full extent.
        let origin = a.start;
        let dots = [
            dir_a.dot(a.start - origin),
            dir_a.dot(a.end - origin),
            dir_a.dot(b.start - origin),
            dir_a.dot(b.end - origin),
        ];
        let min_t = dots.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_t = dots.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        return Some(SpineSegment {
            start: origin + dir_a * min_t,
            end: origin + dir_a * max_t,
            outward_normal: a.outward_normal,
            face_idx: a.face_idx,
            is_interior: a.is_interior,
            travel_dir: a.travel_dir,
        });
    }

    None
}

/// Trim overlapping collinear spines so each world-space region along
/// an aisle edge is covered by exactly one spine. The shorter spine is
/// trimmed to its non-overlapping tail (or removed if fully covered).
pub(crate) fn dedup_overlapping_spines(
    spines: Vec<SpineSegment>,
    tolerance: f64,
) -> Vec<SpineSegment> {
    let mut result = spines;

    // For each pair of collinear, overlapping spines: trim the shorter one.
    let mut changed = true;
    while changed {
        changed = false;
        let n = result.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let dir_i = (result[i].end - result[i].start).normalize();
                let dir_j = (result[j].end - result[j].start).normalize();

                // Must be collinear and same normal.
                if dir_i.dot(dir_j).abs() < 0.99 {
                    continue;
                }
                if result[i].outward_normal.dot(result[j].outward_normal) < 0.99 {
                    continue;
                }
                let travel_compat = match (&result[i].travel_dir, &result[j].travel_dir) {
                    (None, None) => true,
                    (Some(d1), Some(d2)) => d1.dot(*d2) > 0.99,
                    _ => false,
                };
                if !travel_compat {
                    continue;
                }

                // Must be on the same line (close perpendicularly).
                let perp = Vec2::new(-dir_i.y, dir_i.x);
                if perp.dot(result[j].start - result[i].start).abs() > tolerance {
                    continue;
                }

                // Project both onto the shared direction.
                let origin = result[i].start;
                let ti_s = dir_i.dot(result[i].start - origin);
                let ti_e = dir_i.dot(result[i].end - origin);
                let tj_s = dir_i.dot(result[j].start - origin);
                let tj_e = dir_i.dot(result[j].end - origin);

                let (i_min, i_max) = (ti_s.min(ti_e), ti_s.max(ti_e));
                let (j_min, j_max) = (tj_s.min(tj_e), tj_s.max(tj_e));

                // Check for overlap.
                let overlap_start = i_min.max(j_min);
                let overlap_end = i_max.min(j_max);
                if overlap_end - overlap_start < 1.0 {
                    continue; // No significant overlap.
                }

                // Trim the shorter spine to its non-overlapping portion.
                let len_i = i_max - i_min;
                let len_j = j_max - j_min;
                let (shorter, longer_min, longer_max) = if len_i <= len_j {
                    (i, j_min, j_max)
                } else {
                    (j, i_min, i_max)
                };

                let s_min = if shorter == i { i_min } else { j_min };
                let s_max = if shorter == i { i_max } else { j_max };

                // The non-overlapping portion is [s_min, longer_min) or
                // (longer_max, s_max]. Keep the longer tail.
                let tail_left = (longer_min - s_min).max(0.0);
                let tail_right = (s_max - longer_max).max(0.0);

                if tail_left > tail_right && tail_left > 1.0 {
                    // Keep the left tail.
                    result[shorter].start = origin + dir_i * s_min;
                    result[shorter].end = origin + dir_i * longer_min;
                    changed = true;
                } else if tail_right > 1.0 {
                    // Keep the right tail.
                    result[shorter].start = origin + dir_i * longer_max;
                    result[shorter].end = origin + dir_i * s_max;
                    changed = true;
                } else {
                    // Entirely covered: mark for removal (zero length).
                    result[shorter].end = result[shorter].start;
                    changed = true;
                }

                if changed {
                    break;
                }
            }
            if changed {
                break;
            }
        }

        // Remove zero-length spines.
        result.retain(|s| (s.end - s.start).length() > 1.0);
    }

    result
}

/// Generate spine segments for a face shape (outer contour + holes).
///
/// Computes the multi-contour straight skeleton (outer CCW + holes CW),
/// extracts wavefront loops at `effective_depth`, and emits a spine for
/// each surviving aisle-facing edge. Spines are optionally clipped to
/// the face interior.
///
/// `per_edge_corridors` carries un-merged corridor polygons with their
/// `interior` flag and travel direction so that spines can be tagged
/// as interior vs perimeter with the correct one-way direction.
/// Perimeter aisle-facing edges get a higher skeleton weight so their
/// spines sit at full stall_depth from the corridor (for 90° stalls).
pub(crate) fn compute_face_spines(
    shape: &[Vec<Vec2>],
    effective_depth: f64,
    corridor_shapes: &[Vec<Vec<Vec2>>],
    per_edge_corridors: &[(Vec<Vec2>, bool, Option<Vec2>)],
    face_is_boundary: bool,
    _params: &ParkingParams,
    debug: &DebugToggles,
    two_way_oriented_dirs: &[Option<Vec2>],
    tagged: Option<&TaggedFace>,
) -> Vec<SpineSegment> {
    if shape.is_empty() || shape[0].len() < 3 {
        return vec![];
    }

    // Skip faces that are too narrow to hold a stall strip.
    let outer = &shape[0];
    let area = signed_area(outer).abs();
    let perimeter: f64 = outer.iter().enumerate().map(|(i, v)| {
        let next = &outer[(i + 1) % outer.len()];
        (*next - *v).length()
    }).sum();
    if perimeter > 0.0 && 4.0 * area / perimeter < effective_depth {
        return vec![];
    }

    let mut contours = normalize_face_winding(shape);
    if debug.face_simplification {
        contours = contours.into_iter()
            .map(|c| simplify_contour(&c, 0.035))
            .filter(|c| c.len() >= 3)
            .collect();
        if contours.is_empty() {
            return vec![];
        }
    }
    let ed = effective_depth - 0.05;
    let mut aisle_facing_flat: Vec<bool> = Vec::new();
    let face_interior = !face_is_boundary;
    let mut interior_flat: Vec<bool> = Vec::new();
    let mut travel_dir_flat: Vec<Option<Vec2>> = Vec::new();
    let mut two_way_oriented_flat: Vec<bool> = Vec::new();

    // Use tagged classification when available and face_simplification is off
    // (simplification changes vertex count, breaking tag alignment).
    let use_tags = tagged.is_some() && !debug.face_simplification;
    if use_tags {
        let tf = tagged.unwrap();
        // Build list of (contour, tagged_edges) pairs to classify.
        let mut pairs: Vec<(&[Vec2], &[FaceEdge])> = Vec::new();
        if !contours.is_empty() {
            pairs.push((&contours[0], &tf.edges));
        }
        for (ci, contour) in contours.iter().enumerate().skip(1) {
            let hi = ci - 1;
            if hi < tf.hole_edges.len() {
                pairs.push((contour, &tf.hole_edges[hi]));
            }
        }
        for (contour, tagged_edges) in pairs {
            let n = contour.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let mid = (contour[i] + contour[j]) * 0.5;
                let mut best_dist = f64::INFINITY;
                let mut best_source: &EdgeSource = &EdgeSource::Wall;
                for te in tagged_edges {
                    let te_mid = (te.start + te.end) * 0.5;
                    let d = (te_mid - mid).length();
                    if d < best_dist {
                        best_dist = d;
                        best_source = &te.source;
                    }
                }
                match best_source {
                    EdgeSource::Wall => {
                        aisle_facing_flat.push(false);
                        interior_flat.push(face_interior);
                        travel_dir_flat.push(None);
                        two_way_oriented_flat.push(false);
                    }
                    EdgeSource::Aisle { travel_dir, is_two_way_oriented, .. } => {
                        aisle_facing_flat.push(true);
                        interior_flat.push(face_interior);
                        travel_dir_flat.push(*travel_dir);
                        two_way_oriented_flat.push(*is_two_way_oriented);
                    }
                }
            }
        }
    } else {
        for contour in contours.iter() {
            let classified = classify_face_edges_ext(contour, corridor_shapes, per_edge_corridors, debug.edge_classification, two_way_oriented_dirs);
            for &(facing, _ignored_interior, travel_dir, is_two_way_ori) in classified.iter() {
                aisle_facing_flat.push(facing);
                interior_flat.push(face_interior);
                travel_dir_flat.push(travel_dir);
                two_way_oriented_flat.push(is_two_way_ori);
            }
        }
    }

    // Edge weights for the weighted skeleton: aisle-facing edges shrink at
    // normal speed (1.0), non-aisle-facing edges (boundary walls) stay fixed
    // (0.0). This lets one-sided perimeter faces produce spines even when
    // the face is narrower than 2× effective_depth.
    let edge_weights: Vec<f64> = aisle_facing_flat
        .iter()
        .map(|&af| if af { 1.0 } else { 0.0 })
        .collect();
    let sk = skeleton::compute_skeleton_multi(&contours, &edge_weights);
    let wf_loops = skeleton::wavefront_loops_at(&sk, ed);

    // Collect wavefront loops to process, each with an optional edge to skip.
    // Normal path: no skipped edges.
    // Suppression path (narrow faces): skip the suppressed edge so we don't
    // emit a spine on the face boundary where the edge didn't move.
    let mut wf_to_process: Vec<(Vec<Vec2>, Vec<usize>, Option<usize>)> = Vec::new();
    for (pts, edges) in wf_loops {
        wf_to_process.push((pts, edges, None));
    }

    // If the normal wavefront is empty, the face is too narrow for back-to-back
    // stalls but may fit single-sided rows. Suppress one aisle-facing edge at a
    // time (treat it as a wall) and re-run the skeleton to get spines for the
    // remaining aisle edges.
    if wf_to_process.is_empty() {
        let aisle_indices: Vec<usize> = aisle_facing_flat
            .iter()
            .enumerate()
            .filter(|(_, &af)| af)
            .map(|(i, _)| i)
            .collect();
        if aisle_indices.len() >= 2 {
            for &suppress_idx in &aisle_indices {
                let mut w = edge_weights.clone();
                w[suppress_idx] = 0.0;
                let sk2 = skeleton::compute_skeleton_multi(&contours, &w);
                for (pts, edges) in skeleton::wavefront_loops_at(&sk2, ed) {
                    wf_to_process.push((pts, edges, Some(suppress_idx)));
                }
            }
        }
    }

    let mut all_spines = Vec::new();
    for (wf_pts, active_edges, skip_edge) in &wf_to_process {
        let m = active_edges.len();
        if m < 2 || wf_pts.len() != m {
            continue;
        }
        for idx in 0..m {
            let next_idx = (idx + 1) % m;
            let orig_edge = active_edges[next_idx];
            if orig_edge >= aisle_facing_flat.len() || !aisle_facing_flat[orig_edge] {
                continue;
            }
            if skip_edge.map_or(false, |s| s == orig_edge) {
                continue;
            }

            let spine_start = wf_pts[idx];
            let spine_end = wf_pts[next_idx];
            if (spine_end - spine_start).length() < 1.0 {
                continue;
            }

            // Outward normal = opposite of the edge's inward normal.
            let outward = Vec2::new(-sk.edge_normals[orig_edge].x, -sk.edge_normals[orig_edge].y);
            let is_interior = interior_flat.get(orig_edge).copied().unwrap_or(true);
            let is_two_way_ori = two_way_oriented_flat.get(orig_edge).copied().unwrap_or(false);
            let travel_dir = travel_dir_flat.get(orig_edge).copied().flatten().map(|td| {
                if is_two_way_ori {
                    // For two-way-oriented aisles, flip travel_dir for the
                    // spine on the "canonical negative" side. This condition
                    // is td-INDEPENDENT (based only on outward_normal), so
                    // it always flips the same physical side regardless of
                    // which variant is active. Combined with the standard
                    // flip_angle formula, this makes both sides produce the
                    // same flip_angle, giving correct per-lane stall angles.
                    let neg_side = outward.x < -1e-9
                        || (outward.x.abs() < 1e-9 && outward.y < -1e-9);
                    if neg_side { Vec2::new(-td.x, -td.y) } else { td }
                } else {
                    td
                }
            });

            // Suppress stalls on the left side of one-way aisles.
            if let Some(td) = travel_dir {
                if !is_two_way_ori && td.cross(outward) >= 0.0 {
                    continue;
                }
            }

            if !debug.spine_clipping {
                all_spines.push(SpineSegment {
                    start: spine_start,
                    end: spine_end,
                    outward_normal: outward,
                    face_idx: 0,
                    is_interior,
                    travel_dir,
                });
                continue;
            }

            // Clip spine and stall-reach line to face interior.
            let spine_clips = clip_segment_to_face(spine_start, spine_end, shape);
            let reach = (ed - 0.5).max(0.0);
            let offset_start = spine_start + outward * reach;
            let offset_end = spine_end + outward * reach;
            let offset_clips = clip_segment_to_face(offset_start, offset_end, shape);

            for (st0, st1) in &spine_clips {
                for (ot0, ot1) in &offset_clips {
                    let t0 = st0.max(*ot0);
                    let t1 = st1.min(*ot1);
                    if t1 - t0 < 1e-9 {
                        continue;
                    }
                    let d = spine_end - spine_start;
                    let s = spine_start + d * t0;
                    let e = spine_start + d * t1;
                    if (e - s).length() > 1.0 {
                        all_spines.push(SpineSegment {
                            start: s,
                            end: e,
                            outward_normal: outward,
                            face_idx: 0,
                            is_interior,
                            travel_dir,
                        });
                    }
                }
            }
        }
    }

    all_spines
}

/// Extend each spine colinearly in both directions until it hits the face
/// boundary. Returns (extension SpineSegment, source spine index) pairs.
/// Each extension inherits all properties (normal, travel_dir, etc.) from its
/// source spine. A dual-clip approach validates both the spine position and
/// the stall-reach position, ensuring extension stalls have depth clearance.
pub(crate) fn extend_spines_to_faces(
    spines: &[SpineSegment],
    faces: &[Vec<Vec<Vec2>>],
    effective_depth: f64,
    params: &ParkingParams,
) -> Vec<(SpineSegment, usize)> {
    let mut extensions = Vec::new();

    // Compute stall pitch to use as overlap margin. fill_strip keeps stall
    // centers at least half a pitch from each segment endpoint, so extending
    // the extension segment one pitch into the primary spine ensures the
    // first extension grid cell bridges the gap. remove_extension_conflicts
    // discards duplicates in the overlap zone afterward.
    let stall_pitch = params.stall_pitch();

    for (src_idx, spine) in spines.iter().enumerate() {
        if spine.face_idx >= faces.len() {
            continue;
        }
        // Skip boundary faces — extensions only apply to interior faces
        // where angled stalls leave triangular waste in tapered regions.
        if !spine.is_interior {
            continue;
        }
        let face_shape = &faces[spine.face_idx];
        if face_shape.is_empty() {
            continue;
        }

        let dir = (spine.end - spine.start).normalize();
        let spine_len = (spine.end - spine.start).length();
        if spine_len < 1e-6 {
            continue;
        }

        // Extend the spine line far in both directions.
        let extend_dist = 10000.0;
        let ext_start = spine.start - dir * extend_dist;
        let ext_end = spine.end + dir * extend_dist;
        let ext_vec = ext_end - ext_start;
        let ext_total = ext_vec.length();

        // Clip the extended spine line to the face interior.
        let spine_clips = clip_segment_to_face(ext_start, ext_end, face_shape);

        // Also clip the offset line (stall reach) to ensure depth clearance.
        let reach = (effective_depth - 0.5).max(0.0);
        let off_start = ext_start + spine.outward_normal * reach;
        let off_end = ext_end + spine.outward_normal * reach;
        let offset_clips = clip_segment_to_face(off_start, off_end, face_shape);

        // The original spine occupies a known parameter range on the extended
        // line. Shrink it by one stall_pitch on each side so that extension
        // segments overlap the primary, ensuring contiguous stall placement.
        let overlap_t = stall_pitch / ext_total;
        let orig_t0 = extend_dist / ext_total + overlap_t;
        let orig_t1 = (extend_dist + spine_len) / ext_total - overlap_t;

        // End margin: shorten each extension's outer end (the face-boundary
        // side) so the last stall doesn't create a thin sliver island.
        let end_margin_t = (stall_pitch * 1.5) / ext_total;

        for &(st0, st1) in &spine_clips {
            for &(ot0, ot1) in &offset_clips {
                let t0 = st0.max(ot0);
                let t1 = st1.min(ot1);
                if t1 - t0 < 1e-9 {
                    continue;
                }

                // Extract portions outside the (shrunk) original spine range.
                // Outer ends are pulled inward by end_margin_t to avoid
                // thin sliver islands at face boundaries.
                // Left tail: [t0 + margin, min(t1, orig_t0)]
                if t0 < orig_t0 - 1e-9 {
                    let tail_end = t1.min(orig_t0);
                    let s = ext_start + ext_vec * (t0 + end_margin_t);
                    let e = ext_start + ext_vec * tail_end;
                    if (e - s).length() > 1.0 {
                        extensions.push((SpineSegment {
                            start: s,
                            end: e,
                            outward_normal: spine.outward_normal,
                            face_idx: spine.face_idx,
                            is_interior: spine.is_interior,
                            travel_dir: spine.travel_dir,
                        }, src_idx));
                    }
                }
                // Right tail: [max(t0, orig_t1), t1 - margin]
                if t1 > orig_t1 + 1e-9 {
                    let tail_start = t0.max(orig_t1);
                    let s = ext_start + ext_vec * tail_start;
                    let e = ext_start + ext_vec * (t1 - end_margin_t);
                    if (e - s).length() > 1.0 {
                        extensions.push((SpineSegment {
                            start: s,
                            end: e,
                            outward_normal: spine.outward_normal,
                            face_idx: spine.face_idx,
                            is_interior: spine.is_interior,
                            travel_dir: spine.travel_dir,
                        }, src_idx));
                    }
                }
            }
        }
    }

    extensions
}
