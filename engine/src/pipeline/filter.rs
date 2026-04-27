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
use crate::geom::clip::point_in_polygon;
use crate::geom::poly::{point_to_segment_dist, signed_area};
use crate::pipeline::bays::normalize_face_winding;
use crate::types::{StallKind, StallModifier, StallQuad, Vec2};

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
/// create a geometric conflict). Standard, Compact, Ev, and Extension
/// stalls are all retypeable. Suppression applies regardless of kind,
/// so fire lanes clear extension and island stalls alike.
pub(crate) fn apply_stall_modifiers(
    stalls: &mut [StallQuad],
    modifiers: &[StallModifier],
    radius: f64,
) {
    if modifiers.is_empty() {
        return;
    }
    for stall in stalls.iter_mut() {
        let c = stall_centroid(stall);
        for m in modifiers {
            if modifier_hit(&m.polyline, c, radius) {
                match m.kind {
                    StallKind::Suppressed => stall.kind = StallKind::Suppressed,
                    ref other => {
                        if is_retypeable(&stall.kind) {
                            stall.kind = other.clone();
                        }
                    }
                }
            }
        }
    }
}

fn stall_centroid(s: &StallQuad) -> Vec2 {
    (s.corners[0] + s.corners[1] + s.corners[2] + s.corners[3]) * 0.25
}

fn modifier_hit(polyline: &[Vec2], p: Vec2, radius: f64) -> bool {
    match polyline.len() {
        0 => false,
        1 => (p - polyline[0]).length() <= radius,
        _ => {
            for i in 0..polyline.len() - 1 {
                if point_to_segment_dist(p, polyline[i], polyline[i + 1]) <= radius {
                    return true;
                }
            }
            false
        }
    }
}

fn is_retypeable(kind: &StallKind) -> bool {
    matches!(
        kind,
        StallKind::Standard | StallKind::Compact | StallKind::Ev
    )
}
