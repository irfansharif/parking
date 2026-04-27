//! §3.5 Spine generation + post-processing.
//!
//!   compute_face_spines   — classify face edges (aisle vs wall) and
//!                           emit one offset spine per aisle-facing
//!                           edge via `offset_aisle_edges_to_spines`.
//!   offset_aisle_edges_to_spines
//!                         — offset each aisle edge inward by
//!                           `effective_depth`, trim each end based on
//!                           the adjacent edge's corner angle, clip the
//!                           stall-reach line to the face.
//!   extend_spines_to_faces — colinear extension of interior spines to
//!                            face boundary (dual-clip validated).
//!   merge_collinear_spines / try_merge_spines
//!                         — join compatible spines that span face
//!                           boundaries along the same aisle edge.
//!   extend_primary_with_extensions
//!                         — fold a primary's extension segments into its
//!                           own extent, producing one longer SpineSegment
//!                           covering its full reach.

use crate::geom::poly::clip_segment_to_face;
use crate::pipeline::bays::normalize_face_winding;
use crate::pipeline::tagging::classify_face_edges_ext;
use crate::types::{
    DebugToggles, EdgeSource, FaceEdge, ParkingParams, SpineSegment, TaggedFace, Vec2,
};

/// Merge collinear spine segments that share an endpoint (within tolerance)
/// and have the same outward normal direction. This eliminates gaps between
/// stall strips that span multiple faces along the same aisle edge.
pub(crate) fn merge_collinear_spines(
    spines: Vec<SpineSegment>,
    tolerance: f64,
    cos_tol: f64,
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

                    if let Some(m) = try_merge_spines(&seg, &merged[j], tolerance, cos_tol) {
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
/// collinear (directions within `cos_tol`), have similar outward normals,
/// and share an endpoint within `tolerance`.
pub(crate) fn try_merge_spines(
    a: &SpineSegment,
    b: &SpineSegment,
    tolerance: f64,
    cos_tol: f64,
) -> Option<SpineSegment> {
    // Only merge spines of the same type (interior/perimeter).
    if a.is_interior != b.is_interior {
        return None;
    }

    // Spines on the same physical aisle (same side) end up with the
    // same flip_angle/staggered/is_annotated bits, so requiring exact
    // equality both rules out cross-aisle merges and admits the
    // intended within-aisle ones.
    if a.flip_angle != b.flip_angle
        || a.staggered != b.staggered
        || a.is_annotated != b.is_annotated
    {
        return None;
    }

    let dir_a = (a.end - a.start).normalize();
    let dir_b = (b.end - b.start).normalize();

    // Must be collinear: directions parallel (dot ≈ ±1).
    if dir_a.dot(dir_b).abs() < cos_tol {
        return None;
    }

    // Outward normals must point the same way (both positive dot).
    // The merged spine keeps a.start's origin and a's direction/normal,
    // so the merge effectively projects b onto a's line — chord chains
    // get straightened, cutting corners by < ε relative to the underlying
    // arc at the per-step angles cos_tol admits.
    if a.outward_normal.dot(b.outward_normal) < cos_tol {
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
            flip_angle: a.flip_angle,
            staggered: a.staggered,
            is_annotated: a.is_annotated,
        });
    }

    None
}

/// Fold a primary spine's extensions into its own extent so the result
/// is a single longer `SpineSegment` covering [min, max] along the
/// primary's direction. Extensions are colinear with their primary by
/// construction (see `extend_spines_to_faces`), so this is a pure
/// extent-union: the primary's `outward_normal`, `face_idx`,
/// `is_interior`, and the precomputed flag bits carry through unchanged.
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
        flip_angle: primary.flip_angle,
        staggered: primary.staggered,
        is_annotated: primary.is_annotated,
    }
}

/// For each aisle-facing edge in the (normalized) face contours, emit a
/// spine parallel to the edge at `effective_depth` inward. Edges are
/// independent — no event queue, no wavefront interaction.
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
            let e_dir = Vec2::new(edge.x * inv, edge.y * inv);

            // End-trim at each spine endpoint, based on the corner
            // angle with the neighboring edge in the same contour.
            //
            //   aisle-wall: trim = ed / tan(θ), where θ is the interior
            //     angle. Zero at perpendicular, positive at acute,
            //     clamped to zero at obtuse (we never push a spine
            //     past its original edge extent). A floor of ed/2
            //     applies when the raw formula would give zero, so
            //     perpendicular corners still leave room for an end
            //     island.
            //
            //   aisle-aisle: trim = ed / tan(θ/2). At a 90°
            //     cross-aisle corner that's exactly ed — matches a
            //     grid bay's natural back-to-back row extent. No
            //     floor, so tangent-continuous neighbors (chord
            //     chains on an arc, interior angle → 180°) trim
            //     by → 0 and stay within path grouping's
            //     endpoint_tol. A floor would trim every chord by
            //     ed/2 and shred the chain.
            //
            // Capped per-side at edge_len/2 so the spine can't invert
            // at very acute corners. cos_theta is clamped to [-1, 1]
            // before use to absorb floating-point noise.
            let pred_flat = flat_base + (i + n - 1) % n;
            let succ_flat = flat_base + (i + 1) % n;
            let min_wall_trim = ed * 0.5;
            let compute_end_trim = |neighbor_vec: Vec2,
                                    edge_vec: Vec2,
                                    neighbor_is_aisle: bool|
             -> f64 {
                let nlen = neighbor_vec.length();
                let elen = edge_vec.length();
                if nlen < 1e-9 || elen < 1e-9 {
                    return if neighbor_is_aisle { 0.0 } else { min_wall_trim };
                }
                let cos_theta = (neighbor_vec.dot(edge_vec) / (nlen * elen)).clamp(-1.0, 1.0);
                if neighbor_is_aisle {
                    // tan(θ/2) via half-angle identity. Tangent-continuous
                    // (cos_theta → -1) makes tan_half → ∞, trim → 0.
                    let one_minus = (1.0 - cos_theta).max(1e-12);
                    let one_plus = (1.0 + cos_theta).max(1e-12);
                    let tan_half = (one_minus / one_plus).sqrt();
                    ed / tan_half
                } else {
                    let base = if cos_theta > 0.0 {
                        let sin_theta = (1.0 - cos_theta * cos_theta).sqrt().max(1e-9);
                        ed * cos_theta / sin_theta
                    } else {
                        0.0
                    };
                    base.max(min_wall_trim)
                }
            };
            let pred_is_aisle = aisle_facing_flat.get(pred_flat).copied().unwrap_or(false);
            let succ_is_aisle = aisle_facing_flat.get(succ_flat).copied().unwrap_or(false);
            let p = contour[(i + n - 1) % n];
            let q = contour[(i + 2) % n];
            let (trim_a, trim_b) = if debug.spine_end_trim {
                (
                    compute_end_trim(p - a, b - a, pred_is_aisle).min(edge_len * 0.5),
                    compute_end_trim(q - b, a - b, succ_is_aisle).min(edge_len * 0.5),
                )
            } else {
                (0.0, 0.0)
            };
            if edge_len - trim_a - trim_b < 1.0 {
                continue;
            }
            let trimmed_a = a + e_dir * trim_a;
            let trimmed_b = b - e_dir * trim_b;

            let s = trimmed_a + inward * ed;
            let e = trimmed_b + inward * ed;

            let is_interior = interior_flat.get(flat_idx).copied().unwrap_or(true);
            let face_reverse =
                two_way_oriented_flat.get(flat_idx).copied().unwrap_or(false);
            let travel_dir = travel_dir_flat.get(flat_idx).copied().flatten();

            if (e - s).length() > 1.0 {
                // Three independent states determine flip + stagger:
                //   1. face_reverse (any TwoWayReverse aisle in the face) →
                //      every spine flips lean, no stagger. Mirror-symmetric
                //      slash pattern, opposite of default.
                //   2. OneWay/OneWayReverse without face_reverse → per-side
                //      asymmetric flip from oriented_dir·travel_dir, plus
                //      half-pitch stagger when outward is right-of-travel.
                //      Produces herringbone across the aisle.
                //   3. Default (TwoWay, no annotation) → no flip, no stagger.
                let (flip_angle, staggered) = if face_reverse {
                    (true, false)
                } else if let Some(td) = travel_dir {
                    let oriented_dir = {
                        let d = (e - s).normalize();
                        let left = Vec2::new(-d.y, d.x);
                        if left.dot(outward) > 0.0 { d } else { d * -1.0 }
                    };
                    (oriented_dir.dot(td) > 0.0, td.cross(outward) >= 0.0)
                } else {
                    (false, false)
                };
                let is_annotated = travel_dir.is_some() || face_reverse;
                spines.push(SpineSegment {
                    start: s,
                    end: e,
                    outward_normal: outward,
                    face_idx: 0,
                    is_interior,
                    flip_angle,
                    staggered,
                    is_annotated,
                });
            }
        }
        flat_base += n;
    }

    spines
}

/// Generate spine segments for a face. Classifies each edge of each
/// contour as aisle-facing or wall (via tagged edges when available,
/// else via corridor overlap), then hands off to the per-aisle-edge
/// offset emission.
///
/// `per_edge_corridors` carries un-merged corridor polygons with their
/// `interior` flag and travel direction so that spines can be tagged
/// as interior vs perimeter with the correct one-way direction.
pub(crate) fn compute_face_spines(
    shape: &[Vec<Vec2>],
    effective_depth: f64,
    corridor_shapes: &[Vec<Vec<Vec2>>],
    per_edge_corridors: &[(Vec<Vec2>, bool, Option<Vec2>)],
    face_is_boundary: bool,
    debug: &DebugToggles,
    two_way_oriented_dirs: &[Option<Vec2>],
    tagged: Option<&TaggedFace>,
) -> Vec<SpineSegment> {
    if shape.is_empty() || shape[0].len() < 3 {
        return vec![];
    }

    let contours = normalize_face_winding(shape);
    let ed = effective_depth;
    let mut aisle_facing_flat: Vec<bool> = Vec::new();
    let face_interior = !face_is_boundary;
    let mut interior_flat: Vec<bool> = Vec::new();
    let mut travel_dir_flat: Vec<Option<Vec2>> = Vec::new();
    let mut two_way_oriented_flat: Vec<bool> = Vec::new();

    // Use tagged classification when available.
    let use_tags = tagged.is_some();
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
            let classified = classify_face_edges_ext(contour, corridor_shapes, per_edge_corridors, two_way_oriented_dirs);
            for &(facing, _ignored_interior, travel_dir, is_two_way_ori) in classified.iter() {
                aisle_facing_flat.push(facing);
                interior_flat.push(face_interior);
                travel_dir_flat.push(travel_dir);
                two_way_oriented_flat.push(is_two_way_ori);
            }
        }
    }

    if !contours.is_empty() && !contours[0].is_empty() {
        let xs: Vec<f64> = contours[0].iter().map(|p| p.x).collect();
        let mn = xs.iter().cloned().fold(f64::INFINITY, f64::min);
        let mx = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        eprintln!("[face] x=[{:.0},{:.0}] aisle_facing={:?} reverse_flags={:?}", mn, mx, aisle_facing_flat, two_way_oriented_flat);
    }
    // `reverse_lean` is a face-wide property: if any aisle edge of the
    // bay borders a `TwoWayReverse` aisle, mirror every stall strip in
    // the bay (so back-to-back rows on each side of the aisle both
    // flip together — what the user means by "two-way reverse angles
    // each stall strip on each side the other way").
    let any_reverse = two_way_oriented_flat.iter().any(|&v| v);
    if any_reverse {
        for v in two_way_oriented_flat.iter_mut() {
            *v = true;
        }
    }
    offset_aisle_edges_to_spines(
        &contours,
        &aisle_facing_flat,
        &interior_flat,
        &travel_dir_flat,
        &two_way_oriented_flat,
        ed,
        debug,
    )
}

/// Extend each spine colinearly in both directions until it hits the face
/// boundary. Returns (extension SpineSegment, source spine index) pairs.
/// Each extension inherits all properties (normal, travel_dir, etc.) from its
/// source spine. A dual-clip approach validates both the spine position and
/// the stall-reach position, ensuring extension geometry has depth clearance
/// once folded into the primary by `extend_primary_with_extensions`.
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
                    flip_angle: spine.flip_angle,
                    staggered: spine.staggered,
                    is_annotated: spine.is_annotated,
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
                    flip_angle: spine.flip_angle,
                    staggered: spine.staggered,
                    is_annotated: spine.is_annotated,
                }, src_idx));
            }
        }
    }

    extensions
}

