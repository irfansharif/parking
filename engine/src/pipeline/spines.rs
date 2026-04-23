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
//!   normalize_paired_spines
//!                         — snap a nearly-parallel opposing spine onto
//!                           the longer partner's direction and line, so
//!                           back-to-back rows tile on a shared grid.
//!   extend_primary_with_extensions
//!                         — fold a primary's extension segments into its
//!                           own extent, producing one longer SpineSegment
//!                           that the snap pass can treat uniformly.

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

/// Fold a primary spine's extensions into its own extent so the result
/// is a single longer `SpineSegment` covering [min, max] along the
/// primary's direction. Extensions are colinear with their primary by
/// construction (see `extend_spines_to_faces`), so this is a pure
/// extent-union: the primary's `outward_normal`, `face_idx`,
/// `is_interior`, and `travel_dir` carry through unchanged.
pub(crate) fn extend_primary_with_extensions(
    primary: SpineSegment,
    exts: &[SpineSegment],
) -> SpineSegment {
    if exts.is_empty() {
        return primary;
    }
    let dir = (primary.end - primary.start).normalize();
    let origin = primary.start;
    let project = |p: Vec2| -> f64 { (p - origin).dot(dir) };

    let mut min_t = project(primary.start).min(project(primary.end));
    let mut max_t = project(primary.start).max(project(primary.end));
    for ext in exts {
        let t1 = project(ext.start);
        let t2 = project(ext.end);
        min_t = min_t.min(t1).min(t2);
        max_t = max_t.max(t1).max(t2);
    }
    SpineSegment {
        start: origin + dir * min_t,
        end: origin + dir * max_t,
        outward_normal: primary.outward_normal,
        face_idx: primary.face_idx,
        is_interior: primary.is_interior,
        travel_dir: primary.travel_dir,
    }
}

/// Snap nearly-parallel opposing spines onto a shared line so back-to-back
/// rows tile on the same grid. For each length-descending pass, the longest
/// unconsumed spine claims the first eligible shorter partner and that
/// partner's geometry is reprojected onto the anchor's direction and line.
///
/// Eligibility: same `is_interior` kind, perpendicular midpoint distance
/// within `stall_width / 4` (tight enough to only admit spines on what's
/// essentially the same underlying edge — true back-to-back pairs — and
/// exclude anything further apart, which isn't a pair at all), and
/// direction / opposite-normal alignment both within `snap_ceiling_deg`
/// of exact. Pairs that exceed the ceiling are left untouched (treated
/// as intentional skew).
///
/// The longer spine stays put; the shorter one loses at most `~length ·
/// (1 - cos(ceiling))` of extent when its endpoints are perpendicular-
/// projected onto the anchor's line. At 15° that's ~3% shrinkage.
pub(crate) fn normalize_paired_spines(
    spines: Vec<SpineSegment>,
    params: &ParkingParams,
    snap_ceiling_deg: f64,
) -> Vec<SpineSegment> {
    let n = spines.len();
    if n < 2 {
        return spines;
    }

    let mut out = spines;
    let mut consumed = vec![false; n];

    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        let la = (out[a].end - out[a].start).length();
        let lb = (out[b].end - out[b].start).length();
        lb.partial_cmp(&la).unwrap_or(std::cmp::Ordering::Equal)
    });

    let cos_ceil = snap_ceiling_deg.to_radians().cos();
    let max_perp = params.stall_width / 4.0;

    for i in 0..n {
        let anchor_idx = order[i];
        if consumed[anchor_idx] {
            continue;
        }
        for j in (i + 1)..n {
            let other_idx = order[j];
            if consumed[other_idx] {
                continue;
            }
            if !pair_eligible(&out[anchor_idx], &out[other_idx], cos_ceil, max_perp) {
                continue;
            }
            out[other_idx] = snap_onto_anchor(&out[anchor_idx], &out[other_idx]);
            consumed[anchor_idx] = true;
            consumed[other_idx] = true;
            break;
        }
    }

    out
}

fn pair_eligible(a: &SpineSegment, b: &SpineSegment, cos_ceil: f64, max_perp: f64) -> bool {
    if a.is_interior != b.is_interior {
        return false;
    }
    let dir_a = (a.end - a.start).normalize();
    let dir_b = (b.end - b.start).normalize();
    if dir_a.dot(dir_b).abs() < cos_ceil {
        return false;
    }
    if a.outward_normal.dot(b.outward_normal) > -cos_ceil {
        return false;
    }
    let mid_a = (a.start + a.end) * 0.5;
    let mid_b = (b.start + b.end) * 0.5;
    let sep = mid_b - mid_a;
    let perp_dist = (sep - dir_a * sep.dot(dir_a)).length();
    perp_dist <= max_perp
}

fn snap_onto_anchor(anchor: &SpineSegment, other: &SpineSegment) -> SpineSegment {
    let dir_a = (anchor.end - anchor.start).normalize();
    let origin = anchor.start;
    let project = |p: Vec2| -> Vec2 {
        let t = (p - origin).dot(dir_a);
        origin + dir_a * t
    };
    let new_start = project(other.start);
    let new_end = project(other.end);

    let n1 = Vec2::new(-dir_a.y, dir_a.x);
    let n2 = Vec2::new(dir_a.y, -dir_a.x);
    let new_normal = if n1.dot(other.outward_normal) >= n2.dot(other.outward_normal) {
        n1
    } else {
        n2
    };

    let new_travel = other.travel_dir.map(|td| {
        if td.dot(dir_a) >= td.dot(dir_a * -1.0) {
            dir_a
        } else {
            dir_a * -1.0
        }
    });

    SpineSegment {
        start: new_start,
        end: new_end,
        outward_normal: new_normal,
        face_idx: other.face_idx,
        is_interior: other.is_interior,
        travel_dir: new_travel,
    }
}

/// Offset-carrier spine emission: for each aisle-facing edge in the
/// (normalized) face contours, emit a spine parallel to the edge at
/// `effective_depth` inward. Bypasses the weighted skeleton entirely —
/// edges are independent; there is no wavefront interaction. Spines
/// are clipped to the face interior when `debug.spine_clipping` is on,
/// which also handles narrowness: on strips thinner than `ed` the
/// offset lies past the opposing wall and is clipped away.
///
/// Inward normal convention: edges walked in the contour's intrinsic
/// direction, with normalized winding (outer CCW / holes CW), put the
/// face interior on the left — so inward = left-perp of edge direction.
fn offset_aisle_edges_to_spines(
    contours: &[Vec<Vec2>],
    aisle_facing_flat: &[bool],
    interior_flat: &[bool],
    travel_dir_flat: &[Option<Vec2>],
    two_way_oriented_flat: &[bool],
    effective_depth: f64,
    debug: &DebugToggles,
) -> Vec<SpineSegment> {
    let ed = effective_depth;
    let mut spines = Vec::new();
    let mut flat_base = 0usize;

    for contour in contours.iter() {
        let n = contour.len();
        for i in 0..n {
            let flat_idx = flat_base + i;
            if flat_idx >= aisle_facing_flat.len() || !aisle_facing_flat[flat_idx] {
                continue;
            }
            let a = contour[i];
            let b = contour[(i + 1) % n];
            let edge = b - a;
            let edge_len = edge.length();
            if edge_len < 1e-9 {
                continue;
            }
            let inv = 1.0 / edge_len;
            let inward = Vec2::new(-edge.y * inv, edge.x * inv);
            let outward = Vec2::new(-inward.x, -inward.y);
            let s = a + inward * ed;
            let e = b + inward * ed;

            let is_interior = interior_flat.get(flat_idx).copied().unwrap_or(true);
            let is_two_way_ori =
                two_way_oriented_flat.get(flat_idx).copied().unwrap_or(false);
            let travel_dir = travel_dir_flat.get(flat_idx).copied().flatten().map(|td| {
                if is_two_way_ori {
                    // Mirrors the skeleton path: flip on the canonical
                    // negative side so both back-to-back spines end up
                    // with matching flip_angle.
                    let neg_side = outward.x < -1e-9
                        || (outward.x.abs() < 1e-9 && outward.y < -1e-9);
                    if neg_side { Vec2::new(-td.x, -td.y) } else { td }
                } else {
                    td
                }
            });

            // One-way aisles: suppress the left-of-travel side.
            if let Some(td) = travel_dir {
                if !is_two_way_ori && td.cross(outward) >= 0.0 {
                    continue;
                }
            }

            let push = |start: Vec2, end: Vec2, spines: &mut Vec<SpineSegment>| {
                if (end - start).length() > 1.0 {
                    spines.push(SpineSegment {
                        start,
                        end,
                        outward_normal: outward,
                        face_idx: 0,
                        is_interior,
                        travel_dir,
                    });
                }
            };

            if !debug.spine_clipping {
                push(s, e, &mut spines);
                continue;
            }

            // Don't clip the spine itself: it sits at `ed` inward from
            // the aisle edge, which on a boundary face with width = ed
            // lands exactly on the opposite wall — strict-interior
            // clipping would erase the straight stretch and leave only
            // the corner overhangs. Instead, clip the stall-reach line
            // (`ed - 0.5` toward the aisle from the spine), which sits
            // 0.5ft inside the face near the aisle edge, and map the
            // clipped parameters back to the spine. Mirrors the
            // skeleton path in `compute_face_spines`.
            let reach = (ed - 0.5).max(0.0);
            let reach_start = s + outward * reach;
            let reach_end = e + outward * reach;
            let d = e - s;
            for (t0, t1) in clip_segment_to_face(reach_start, reach_end, contours) {
                if t1 - t0 < 1e-9 {
                    continue;
                }
                push(s + d * t0, s + d * t1, &mut spines);
            }
        }
        flat_base += n;
    }

    spines
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

    // Skip faces that are too narrow to hold a stall strip. The
    // offset-carrier path handles narrowness via clip_segment_to_face,
    // so bypass this pre-filter when that path will run.
    if !(debug.offset_carriers && face_is_boundary) {
        let outer = &shape[0];
        let area = signed_area(outer).abs();
        let perimeter: f64 = outer.iter().enumerate().map(|(i, v)| {
            let next = &outer[(i + 1) % outer.len()];
            (*next - *v).length()
        }).sum();
        if perimeter > 0.0 && 4.0 * area / perimeter < effective_depth {
            return vec![];
        }
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
    let ed = effective_depth;
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

    // Offset-carrier path: skip the weighted skeleton entirely and
    // emit per-aisle-edge offset spines, clipped to the face. Runs
    // only for boundary faces — interior faces still need the skeleton
    // to place back-to-back spines correctly for medial-type regions.
    if debug.offset_carriers && face_is_boundary {
        return offset_aisle_edges_to_spines(
            &contours,
            &aisle_facing_flat,
            &interior_flat,
            &travel_dir_flat,
            &two_way_oriented_flat,
            ed,
            debug,
        );
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
    // time (treat it as a wall) and re-run the skeleton to get spines inset
    // from each remaining aisle edge. All candidates are emitted — pairing
    // (normalize_paired_spines) and stall-level conflict resolution
    // (remove_conflicting_stalls with longer-spine tiebreak) decide the
    // winner downstream.
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

            // Clip only the stall-reach line; the spine itself comes from
            // the wavefront and may sit on a face boundary (narrow boundary
            // faces), where `point_in_face` would wrongly drop it.
            let reach = (ed - 0.5).max(0.0);
            let offset_start = spine_start + outward * reach;
            let offset_end = spine_end + outward * reach;
            let offset_clips = clip_segment_to_face(offset_start, offset_end, shape);

            for &(t0, t1) in &offset_clips {
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

        // Only the clip intervals abutting the original spine are valid
        // extensions. For concave faces the line may exit and re-enter the
        // face further along; those "back-half" intervals must be ignored so
        // the extension stops at the first face-edge crossing rather than
        // jumping across a gap to a disconnected interval.
        let center_t = (extend_dist + spine_len * 0.5) / ext_total;
        let Some(&(st0, st1)) = spine_clips
            .iter()
            .find(|&&(t0, t1)| center_t >= t0 - 1e-9 && center_t <= t1 + 1e-9)
        else {
            continue;
        };
        let Some(&(ot0, ot1)) = offset_clips
            .iter()
            .find(|&&(t0, t1)| center_t >= t0 - 1e-9 && center_t <= t1 + 1e-9)
        else {
            continue;
        };
        let t0 = st0.max(ot0);
        let t1 = st1.min(ot1);
        if t1 - t0 < 1e-9 {
            continue;
        }

        // The original spine occupies a known parameter range on the extended
        // line. Shrink it by one stall_pitch on each side so that extension
        // segments overlap the primary, ensuring contiguous stall placement.
        let overlap_t = stall_pitch / ext_total;
        let orig_t0 = extend_dist / ext_total + overlap_t;
        let orig_t1 = (extend_dist + spine_len) / ext_total - overlap_t;

        // End margin: shorten each extension's outer end (the face-boundary
        // side) so the last stall doesn't create a thin sliver island.
        let end_margin_t = (stall_pitch * 1.5) / ext_total;

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

    extensions
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> ParkingParams {
        ParkingParams::default()
    }

    fn spine(start: Vec2, end: Vec2, normal: Vec2) -> SpineSegment {
        SpineSegment {
            start,
            end,
            outward_normal: normal.normalize(),
            face_idx: 0,
            is_interior: true,
            travel_dir: None,
        }
    }

    #[test]
    fn paired_spines_within_ceiling_snap_onto_longer() {
        // Anchor: long horizontal spine along y=0, normal +y.
        let anchor = spine(Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), Vec2::new(0.0, 1.0));
        // Other: shorter, rotated 10°, centered around y=0 so midpoint
        // falls well within `stall_width / 4 = 2.25` perp threshold.
        let angle = 10.0_f64.to_radians();
        let dx = angle.cos() * 20.0;
        let dy = angle.sin() * 20.0;
        let other_start = Vec2::new(10.0, -dy / 2.0);
        let other_end = Vec2::new(10.0 + dx, dy / 2.0);
        let other_normal = Vec2::new(-angle.sin(), -angle.cos());
        let other = spine(other_start, other_end, other_normal);

        let out = normalize_paired_spines(vec![anchor.clone(), other], &params(), 15.0);
        // Anchor should be unchanged.
        assert_eq!(out[0].start, anchor.start);
        assert_eq!(out[0].end, anchor.end);
        // Other should now sit on y=0.
        assert!(out[1].start.y.abs() < 1e-9, "other.start.y = {}", out[1].start.y);
        assert!(out[1].end.y.abs() < 1e-9, "other.end.y = {}", out[1].end.y);
        // Other's direction should be parallel to anchor (±x).
        let dir = (out[1].end - out[1].start).normalize();
        assert!(dir.x.abs() > 0.999, "dir = {:?}", dir);
        // Other's normal should still point roughly -y (opposite to anchor).
        assert!(out[1].outward_normal.y < -0.99, "normal = {:?}", out[1].outward_normal);
    }

    #[test]
    fn paired_spines_beyond_ceiling_untouched() {
        // 25° angle delta — exceeds 15° ceiling. Spine is centered around
        // y=0 so proximity passes; ceiling alone must do the rejecting.
        let anchor = spine(Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), Vec2::new(0.0, 1.0));
        let angle = 25.0_f64.to_radians();
        let dx = angle.cos() * 30.0;
        let dy = angle.sin() * 30.0;
        let other = spine(
            Vec2::new(10.0, -dy / 2.0),
            Vec2::new(10.0 + dx, dy / 2.0),
            Vec2::new(-angle.sin(), -angle.cos()),
        );
        let other_clone = other.clone();
        let out = normalize_paired_spines(vec![anchor, other], &params(), 15.0);
        assert_eq!(out[1].start, other_clone.start);
        assert_eq!(out[1].end, other_clone.end);
    }

    #[test]
    fn different_is_interior_never_pair() {
        let anchor = spine(Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), Vec2::new(0.0, 1.0));
        let mut other = spine(Vec2::new(0.0, 3.0), Vec2::new(100.0, 3.0), Vec2::new(0.0, -1.0));
        other.is_interior = false;
        let other_clone = other.clone();
        let out = normalize_paired_spines(vec![anchor, other], &params(), 15.0);
        assert_eq!(out[1].start, other_clone.start);
        assert_eq!(out[1].end, other_clone.end);
    }

    #[test]
    fn non_opposing_normals_never_pair() {
        let anchor = spine(Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), Vec2::new(0.0, 1.0));
        // Same-direction normal (both +y) — not opposing.
        let other = spine(Vec2::new(0.0, 3.0), Vec2::new(80.0, 3.0), Vec2::new(0.0, 1.0));
        let other_clone = other.clone();
        let out = normalize_paired_spines(vec![anchor, other], &params(), 15.0);
        assert_eq!(out[1].start, other_clone.start);
        assert_eq!(out[1].end, other_clone.end);
    }

    #[test]
    fn far_spines_not_paired() {
        let anchor = spine(Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), Vec2::new(0.0, 1.0));
        let p = params();
        // Well beyond aisle_width / 2.
        let far_y = p.aisle_width + 2.0 * p.stall_depth;
        let other = spine(
            Vec2::new(0.0, far_y),
            Vec2::new(80.0, far_y),
            Vec2::new(0.0, -1.0),
        );
        let other_clone = other.clone();
        let out = normalize_paired_spines(vec![anchor, other], &p, 15.0);
        assert_eq!(out[1].start, other_clone.start);
        assert_eq!(out[1].end, other_clone.end);
    }

    #[test]
    fn across_aisle_spines_not_paired() {
        // Two parallel opposing spines separated by aisle_width — these are
        // facing each other across an aisle, NOT back-to-back partners.
        // Snapping would move one row of stalls outside its face.
        let p = params();
        let anchor = spine(Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0), Vec2::new(0.0, 1.0));
        let other = spine(
            Vec2::new(0.0, p.aisle_width),
            Vec2::new(80.0, p.aisle_width),
            Vec2::new(0.0, -1.0),
        );
        let other_clone = other.clone();
        let out = normalize_paired_spines(vec![anchor, other], &p, 15.0);
        assert_eq!(out[1].start, other_clone.start);
        assert_eq!(out[1].end, other_clone.end);
    }
}
