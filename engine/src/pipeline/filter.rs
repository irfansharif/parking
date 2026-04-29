//! §3.7 Stall filtering.
//!
//! Post-placement passes that drop stalls which don't belong:
//!
//!   - `clip_stalls_to_faces`  — drop stalls that aren't fully contained
//!     in their tagged face, via `quad_fully_in_face`.
//!   - `quad_fully_in_face` — boolean-difference test: `stall − face`
//!     area must be ≤ `STALL_OVERHANG_TOL × stall_area`.
//!   - `filter_stalls_by_entrance_coverage` — drop stalls whose open
//!     (aisle) edge lies strictly inside the face interior instead of on
//!     its boundary (or outside it). An entrance inside the face means
//!     the stall faces a parking bay instead of a drive aisle — it's
//!     unreachable. Overhang past the boundary is *not* this filter's
//!     concern; `clip_stalls_to_faces` governs that.

use crate::geom::boolean::{self, FillRule};
use crate::geom::clip::{point_in_polygon, segments_intersect};
use crate::geom::poly::{point_to_segment_dist, signed_area};
use crate::pipeline::bays::normalize_face_winding;
use crate::types::{ParkingParams, StallKind, StallModifier, StallQuad, Vec2};

/// Max fraction of stall area allowed outside its tagged face.
const STALL_OVERHANG_TOL: f64 = 2e-2;

/// Tolerance (world units) for treating a sampled entrance point as
/// "on the face edge" rather than strictly inside. Coords are in
/// stall-width units (~9 by default).
const ENTRANCE_EPS: f64 = 0.5;

/// Minimum fraction of the entrance segment that must be on the face
/// boundary or outside the face (i.e., *not* strictly inside the face
/// interior). Entrance points that fall in the interior mean the stall
/// opens into a parking bay rather than a drive aisle, so it's
/// unreachable. Overhang past the boundary — the entrance being
/// slightly outside — is fine here and governed separately by
/// `STALL_OVERHANG_TOL`.
const ENTRANCE_MIN_COVERAGE: f64 = 0.9;

/// Number of samples taken along the entrance segment (endpoints +
/// interior).
const ENTRANCE_SAMPLES: usize = 11;

/// Drop stalls that aren't fully contained in their tagged face.
pub(crate) fn clip_stalls_to_faces(
    stalls: Vec<(StallQuad, usize)>,
    faces: &[Vec<Vec<Vec2>>],
) -> Vec<(StallQuad, usize)> {
    stalls
        .into_iter()
        .filter(|(stall, face_idx)| {
            if *face_idx >= faces.len() {
                return false;
            }
            quad_fully_in_face(&stall.corners, &faces[*face_idx])
        })
        .collect()
}

/// True when the stall quad is contained in the face: the boolean
/// difference `stall − face` has area ≤ `STALL_OVERHANG_TOL × stall_area`.
pub(crate) fn quad_fully_in_face(corners: &[Vec2; 4], face_shape: &[Vec<Vec2>]) -> bool {
    let stall_area = signed_area(corners).abs();
    if stall_area < 1e-9 || face_shape.is_empty() {
        return false;
    }
    let stall_subj = vec![corners.to_vec()];
    let face_paths = normalize_face_winding(face_shape);
    let leftover = boolean::difference(&stall_subj, &face_paths, FillRule::NonZero);
    let mut leftover_area = 0.0;
    for shape in &leftover {
        for contour in shape {
            leftover_area += signed_area(contour).abs();
        }
    }
    leftover_area <= STALL_OVERHANG_TOL * stall_area
}

/// Drop stalls whose aisle-facing (entrance) edge falls strictly inside
/// the tagged face's interior. The entrance segment runs from
/// `corners[3]` (aisle_left) to `corners[2]` (aisle_right); cars enter
/// through this edge, so it must sit on the face boundary or on the
/// aisle side of it. If the entrance is in the face interior the stall
/// opens into an adjacent parking bay — unreachable. Overhang past the
/// boundary (entrance slightly outside) is tolerated here; that's what
/// `STALL_OVERHANG_TOL` is for.
pub(crate) fn filter_stalls_by_entrance_coverage(
    stalls: Vec<(StallQuad, usize, usize)>,
    faces: &[Vec<Vec<Vec2>>],
) -> Vec<(StallQuad, usize, usize)> {
    stalls
        .into_iter()
        .filter(|(stall, face_idx, _spine_idx)| {
            if *face_idx >= faces.len() {
                return false;
            }
            entrance_coverage(&stall.corners, &faces[*face_idx]) >= ENTRANCE_MIN_COVERAGE
        })
        .collect()
}

/// Fraction of samples along the entrance segment (corners[3]→corners[2])
/// that are *not* strictly inside the face interior — i.e., either on a
/// face boundary edge (within `ENTRANCE_EPS`) or past it.
fn entrance_coverage(corners: &[Vec2; 4], face_shape: &[Vec<Vec2>]) -> f64 {
    let a = corners[3];
    let b = corners[2];
    if (b - a).length() < 1e-9 || face_shape.is_empty() {
        return 0.0;
    }
    let outer = &face_shape[0];
    if outer.len() < 3 {
        return 0.0;
    }
    let n = ENTRANCE_SAMPLES;
    let mut good = 0usize;
    for i in 0..n {
        let t = i as f64 / (n - 1) as f64;
        let p = a + (b - a) * t;

        // A sample counts as "on the edge" if it's within ENTRANCE_EPS
        // of any ring's edge. That includes being near a hole boundary,
        // where the stall is adjacent to an island or building.
        let mut on_edge = false;
        for ring in face_shape {
            if ring.len() < 2 {
                continue;
            }
            for j in 0..ring.len() {
                let c = ring[j];
                let d = ring[(j + 1) % ring.len()];
                if point_to_segment_dist(p, c, d) <= ENTRANCE_EPS {
                    on_edge = true;
                    break;
                }
            }
            if on_edge {
                break;
            }
        }
        if on_edge {
            good += 1;
            continue;
        }

        // Otherwise the sample is "outside" the face if it's not inside
        // the outer ring, or it's inside a hole.
        let mut inside = point_in_polygon(&p, outer);
        if inside {
            for hole in face_shape.iter().skip(1) {
                if point_in_polygon(&p, hole) {
                    inside = false;
                    break;
                }
            }
        }
        if !inside {
            good += 1;
        }
    }
    good as f64 / n as f64
}

/// §1.4 post-placement pass: retype stalls whose centroid falls
/// within `radius` of any modifier's polyline. `StallKind::Suppressed`
/// is itself a valid kind — the renderer skips those. Modifiers with
/// the same kind as the stall are no-ops (prevents churning when a
/// fixture is re-run with an empty list).
///
/// A zero-length polyline (single point) is treated as a point
/// modifier: every stall within `radius` of that point gets retyped.
///
/// The function preserves `Island` markings (those are coupled to the
/// already-carved landscape island polygons, so retyping them would
/// create a geometric conflict). Standard, Ada, Compact, and Extension
/// stalls are all retypeable. Suppression applies regardless of kind,
/// so fire lanes clear extension and island stalls alike.
pub(crate) fn apply_stall_modifiers(
    stalls: &mut Vec<StallQuad>,
    modifiers: &[StallModifier],
    radius: f64,
    params: &ParkingParams,
) {
    if modifiers.is_empty() {
        return;
    }

    // Phase 1 — Suppressed (any kind) + Island (retypeable). Per-stall,
    // no count change. Angle gating doesn't apply here: suppressing or
    // islanding angled stalls is fine.
    for stall in stalls.iter_mut() {
        for m in modifiers {
            if !modifier_hits_stall(&m.polyline, &stall.corners, radius) {
                continue;
            }
            match m.kind {
                StallKind::Suppressed => stall.kind = StallKind::Suppressed,
                StallKind::Island => {
                    if is_retypeable(&stall.kind) {
                        stall.kind = StallKind::Island;
                    }
                }
                _ => {}
            }
        }
    }

    // Phase 2 — Ada / Compact. The line spatially defines the
    // modification region. For each row a custom-kind modifier hits,
    // we identify the contiguous run of hit stalls, then *replace*
    // that run with as many custom stalls as fit in the run's span,
    // packed tight from the leftmost hit-stall's left edge. Any
    // remainder is filled with extra Standards if a full standard
    // fits; leftover under-`stall_width` slack stays as a small gap.
    // Stalls outside the hit run keep their original positions.
    //
    // Restricted to perpendicular stalls — angled stalls have a
    // sheared footprint that doesn't accommodate a different
    // rectangular width without re-shearing the geometry. A single
    // lot can mix angles per region, so the gate is per-stall.
    let s = params.stall_width;
    if s <= 1e-9 {
        return;
    }

    // Group perpendicular non-suppressed stalls by row.
    let mut by_row: std::collections::HashMap<RowKey, Vec<usize>> =
        std::collections::HashMap::new();
    for (i, stall) in stalls.iter().enumerate() {
        if stall.kind == StallKind::Suppressed {
            continue;
        }
        if !is_stall_perpendicular(stall) {
            continue;
        }
        if let Some(key) = row_key(stall) {
            by_row.entry(key).or_default().push(i);
        }
    }

    let mut to_remove: std::collections::HashSet<usize> =
        std::collections::HashSet::new();
    let mut to_add: Vec<StallQuad> = Vec::new();

    for (_, mut row) in by_row {
        if row.is_empty() {
            continue;
        }
        let (ux, uy) = back_unit(&stalls[row[0]]);
        // Sort by left-edge (back-left corner) projection along the
        // row axis. Stalls in the row are then in spatial order.
        row.sort_by(|&a, &b| {
            let pa = stalls[a].corners[0].x * ux + stalls[a].corners[0].y * uy;
            let pb = stalls[b].corners[0].x * ux + stalls[b].corners[0].y * uy;
            pa.partial_cmp(&pb).unwrap()
        });

        for m in modifiers {
            let (custom_width, buf_w, shared) = match m.kind {
                StallKind::Ada => (
                    params.ada_stall_width,
                    params.ada_buffer_width.max(0.0),
                    params.ada_buffer_shared,
                ),
                StallKind::Compact => (params.compact_stall_width, 0.0, false),
                _ => continue,
            };
            if custom_width <= 1e-9 {
                continue;
            }
            // Find stalls hit by this modifier (and not already
            // claimed by a prior modifier).
            let hits: Vec<usize> = row
                .iter()
                .enumerate()
                .filter(|(_, &si)| {
                    !to_remove.contains(&si)
                        && modifier_hits_stall(&m.polyline, &stalls[si].corners, radius)
                })
                .map(|(idx, _)| idx)
                .collect();
            if hits.is_empty() {
                continue;
            }
            // Bounds of the placement region: the spatial extent of
            // the hit run (leftmost hit stall's left edge → rightmost
            // hit stall's right edge). The modifier line's projection
            // is *not* used directly — a line that runs past either
            // end of the row would otherwise spill stalls outside
            // the bay. Customs only fill the gap left by the removed
            // stalls.
            let i_first = hits[0];
            let i_last = *hits.last().unwrap();
            let p_left = stalls[row[i_first]].corners[0].x * ux
                + stalls[row[i_first]].corners[0].y * uy;
            let last_stall = &stalls[row[i_last]];
            let p_right = last_stall.corners[1].x * ux + last_stall.corners[1].y * uy;
            let line_a = p_left;
            let line_span = p_right - p_left;
            if line_span <= 1e-9 {
                continue;
            }

            // Solve for K customs (with buffers) that fit in `line_span`.
            //   non-shared: [w][b][w][b]…[w][b]
            //               → K stalls, K buffers, total = K·(w + b)
            //   shared:     [b][w][w][b][w][w][b]…[b][w][b]?
            //               → pairs of ADAs share end-cap buffers with
            //                 neighbouring pairs; each ADA is adjacent
            //                 to exactly one buffer. With K = 2p + s
            //                 (s ∈ {0,1}), there are ⌈K/2⌉ + 1 buffers.
            //                 total = K·w + (⌈K/2⌉ + 1)·b
            let (k, cluster_span) = if buf_w <= 1e-9 {
                let n = (line_span / custom_width).floor().max(0.0) as usize;
                (n, (n as f64) * custom_width)
            } else if shared {
                let denom = 2.0 * custom_width + buf_w;
                // Even K = 2p: L ≥ 2p·w + (p+1)·b → p ≤ (L − b) / (2w + b).
                let p_even = ((line_span - buf_w) / denom).floor().max(0.0) as usize;
                // Odd K = 2p+1: L ≥ (2p+1)·w + (p+2)·b → p ≤ (L − w − 2b) / (2w + b).
                let p_odd_f = (line_span - custom_width - 2.0 * buf_w) / denom;
                let p_odd = if p_odd_f < 0.0 { None } else { Some(p_odd_f.floor() as usize) };
                let k_even = 2 * p_even;
                let total_even =
                    (k_even as f64) * custom_width + ((p_even as f64) + 1.0) * buf_w;
                let mut best = (k_even, if k_even == 0 { 0.0 } else { total_even });
                if let Some(p) = p_odd {
                    let k_odd = 2 * p + 1;
                    let total_odd =
                        (k_odd as f64) * custom_width + ((p as f64) + 2.0) * buf_w;
                    if k_odd > best.0 && total_odd <= line_span + 1e-9 {
                        best = (k_odd, total_odd);
                    }
                }
                best
            } else {
                let pitch = custom_width + buf_w;
                let n = (line_span / pitch).floor().max(0.0) as usize;
                (n, (n as f64) * pitch)
            };
            let remainder = (line_span - cluster_span).max(0.0);
            let extra_standards = if s > 1e-9 {
                (remainder / s).floor() as usize
            } else {
                0
            };

            // Mark every hit stall (and any in-between slots between
            // the first and last hit) for removal so partial standards
            // don't overlap the customs we're about to place.
            for idx in i_first..=i_last {
                to_remove.insert(row[idx]);
            }

            // Place the cluster starting at line_a:
            //   non-shared (or buf=0): [ADA][buf][ADA][buf]…[ADA][buf]
            //   shared:                [buf][ADA][ADA][buf][ADA][ADA][buf]…
            //                          (trailing [ADA][buf] for an odd K)
            // Then any extra standards that fit in the leftover slack.
            let template = stalls[row[i_first]].clone();
            let mut cursor = line_a;
            let custom_kind = m.kind.clone();
            let emit_buffer = |cursor: &mut f64, to_add: &mut Vec<StallQuad>| {
                if buf_w > 1e-9 {
                    to_add.push(make_stall_along_row(
                        &template,
                        *cursor,
                        buf_w,
                        ux,
                        uy,
                        StallKind::Buffer,
                    ));
                    *cursor += buf_w;
                }
            };
            let emit_ada = |cursor: &mut f64, to_add: &mut Vec<StallQuad>| {
                to_add.push(make_stall_along_row(
                    &template,
                    *cursor,
                    custom_width,
                    ux,
                    uy,
                    custom_kind.clone(),
                ));
                *cursor += custom_width;
            };

            if shared && buf_w > 1e-9 {
                let pairs = k / 2;
                let singleton = k % 2;
                emit_buffer(&mut cursor, &mut to_add);
                for _ in 0..pairs {
                    emit_ada(&mut cursor, &mut to_add);
                    emit_ada(&mut cursor, &mut to_add);
                    emit_buffer(&mut cursor, &mut to_add);
                }
                if singleton == 1 {
                    emit_ada(&mut cursor, &mut to_add);
                    emit_buffer(&mut cursor, &mut to_add);
                }
            } else {
                for _ in 0..k {
                    emit_ada(&mut cursor, &mut to_add);
                    emit_buffer(&mut cursor, &mut to_add);
                }
            }
            for _ in 0..extra_standards {
                to_add.push(make_stall_along_row(
                    &template,
                    cursor,
                    s,
                    ux,
                    uy,
                    StallKind::Standard,
                ));
                cursor += s;
            }
        }
    }

    if !to_remove.is_empty() {
        let mut idx = 0usize;
        stalls.retain(|_| {
            let keep = !to_remove.contains(&idx);
            idx += 1;
            keep
        });
    }
    stalls.extend(to_add);
}

/// Build a new stall by translating `template` so its back-left
/// corner sits at `p_left` along the row axis `(ux, uy)`, then
/// resizing its back-edge width to `width`. The depth axis (back→
/// aisle) is preserved from the template, so the new stall sits in
/// the same row at the same depth.
fn make_stall_along_row(
    template: &StallQuad,
    p_left: f64,
    width: f64,
    ux: f64,
    uy: f64,
    kind: StallKind,
) -> StallQuad {
    let mut s = template.clone();
    s.kind = kind;
    let template_left = template.corners[0].x * ux + template.corners[0].y * uy;
    let shift = p_left - template_left;
    if shift.abs() > 1e-12 {
        for c in s.corners.iter_mut() {
            c.x += ux * shift;
            c.y += uy * shift;
        }
    }
    let cur_back_x = s.corners[1].x - s.corners[0].x;
    let cur_back_y = s.corners[1].y - s.corners[0].y;
    let cur_w = (cur_back_x * cur_back_x + cur_back_y * cur_back_y).sqrt();
    if cur_w > 1e-9 && (width - cur_w).abs() > 1e-9 {
        let dw = width - cur_w;
        s.corners[1].x += ux * dw;
        s.corners[1].y += uy * dw;
        s.corners[2].x += ux * dw;
        s.corners[2].y += uy * dw;
    }
    s
}

type RowKey = (i64, i64, i64);

/// Group key: (back-edge angle bucket, back-edge line distance bucket,
/// depth-side bucket). Two stalls in the same physical row share all
/// three. Buckets at 0.1° / 0.1ft to absorb float noise.
fn row_key(s: &StallQuad) -> Option<RowKey> {
    let bx = s.corners[1].x - s.corners[0].x;
    let by = s.corners[1].y - s.corners[0].y;
    let blen = (bx * bx + by * by).sqrt();
    if blen < 1e-9 {
        return None;
    }
    // Back direction angle, mod 180° (a row is a line, not a ray).
    let mut ang = by.atan2(bx).to_degrees();
    if ang < 0.0 {
        ang += 180.0;
    }
    if ang >= 180.0 {
        ang -= 180.0;
    }
    // Perpendicular distance from origin to back-edge line.
    let nx = -by / blen;
    let ny = bx / blen;
    let dist = s.corners[0].x * nx + s.corners[0].y * ny;
    // Which side the stall extends toward (back→aisle direction).
    let dx = s.corners[3].x - s.corners[0].x;
    let dy = s.corners[3].y - s.corners[0].y;
    let depth_ang = if dx * dx + dy * dy > 1e-18 {
        let mut a = dy.atan2(dx).to_degrees();
        if a < 0.0 {
            a += 360.0;
        }
        a
    } else {
        0.0
    };
    Some((
        (ang * 10.0).round() as i64,
        (dist * 10.0).round() as i64,
        (depth_ang * 10.0).round() as i64,
    ))
}

/// True when the stall is a "perpendicular" stall (90°-style), i.e.
/// its back edge is parallel to its aisle edge. For angled parking
/// the back edge runs along `width_dir` (rotated by stall_angle from
/// the spine) while the aisle edge always runs along the spine
/// itself, so the two diverge by the stall angle. For 90° parking
/// they coincide.
///
/// Note: an angled stall quad is still a *rectangle* (back ⊥ left),
/// just rotated relative to the spine, so checking back ⊥ left
/// would always return true — that's why we test back ∥ aisle here.
/// Inferred per-stall from corner geometry: the same lot can mix
/// angles (e.g. perimeter row at 90°, interior at 45°).
fn is_stall_perpendicular(s: &StallQuad) -> bool {
    // Back edge: corners[0] → corners[1]. Aisle edge: corners[3] →
    // corners[2]. (Note: aisle_right - aisle_left, same direction as
    // back_right - back_left for 90° stalls.)
    let bx = s.corners[1].x - s.corners[0].x;
    let by = s.corners[1].y - s.corners[0].y;
    let ax = s.corners[2].x - s.corners[3].x;
    let ay = s.corners[2].y - s.corners[3].y;
    let blen = (bx * bx + by * by).sqrt();
    let alen = (ax * ax + ay * ay).sqrt();
    if blen < 1e-9 || alen < 1e-9 {
        return false;
    }
    // Parallel ⟺ |cross| / (|a| * |b|) ≈ 0. sin(1°) ≈ 0.0175.
    let cross_norm = (bx * ay - by * ax) / (blen * alen);
    cross_norm.abs() < 0.0175
}

/// Unit vector along a stall's back edge ([0]→[1]).
fn back_unit(s: &StallQuad) -> (f64, f64) {
    let dx = s.corners[1].x - s.corners[0].x;
    let dy = s.corners[1].y - s.corners[0].y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-9 {
        (1.0, 0.0)
    } else {
        (dx / len, dy / len)
    }
}

/// Resize a stall quad symmetrically along its back-edge axis.
/// Strict geometric overlap test: a stall is "hit" by a modifier
/// polyline iff
///   * (1-point polyline) the click point lies inside the stall quad
///     OR within `click_radius` of any quad edge (small slop so users
///     don't have to click pixel-perfectly into the interior);
///   * (≥2-point polyline) any segment of the polyline crosses the
///     stall quad — either the segment endpoint sits inside, or the
///     segment intersects one of the quad's four edges.
///
/// Replaces an earlier centroid-distance heuristic that was generous
/// enough to sweep in stalls in adjacent rows for angled bays.
fn modifier_hits_stall(polyline: &[Vec2], corners: &[Vec2; 4], click_radius: f64) -> bool {
    match polyline.len() {
        0 => false,
        1 => {
            let p = polyline[0];
            if point_in_polygon(&p, corners) {
                return true;
            }
            // Edge slop for clicks landing just outside the quad.
            for i in 0..4 {
                let a = corners[i];
                let b = corners[(i + 1) % 4];
                if point_to_segment_dist(p, a, b) <= click_radius {
                    return true;
                }
            }
            false
        }
        _ => {
            for w in polyline.windows(2) {
                let (a, b) = (w[0], w[1]);
                // Endpoint inside quad → hit.
                if point_in_polygon(&a, corners) || point_in_polygon(&b, corners) {
                    return true;
                }
                // Otherwise check segment vs each quad edge.
                for i in 0..4 {
                    let q0 = corners[i];
                    let q1 = corners[(i + 1) % 4];
                    if segments_intersect(a, b, q0, q1) {
                        return true;
                    }
                }
            }
            false
        }
    }
}

fn is_retypeable(kind: &StallKind) -> bool {
    matches!(
        kind,
        StallKind::Standard | StallKind::Ada | StallKind::Compact
    )
}
