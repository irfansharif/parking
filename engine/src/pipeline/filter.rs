//! §3.7 Stall filtering.
//!
//! Post-placement passes that drop stalls which don't belong:
//!
//!   - `clip_stalls_to_faces`  — drop stalls whose shrunk corners
//!     protrude past their own face boundary, or whose depth-ray
//!     projection straddles a face corner.
//!   - `quad_contained_in_face` — boolean-intersection area test
//!     (stall ∩ face ≥ min_frac × stall_area).
//!   - `quad_contained_in_boundary` — centroid-in-boundary +
//!     no-quad-edge-crosses-boundary-edge.
//!   - `shrink_toward_centroid` — overlap-test helper shared with
//!     extension-stall greedy placement.

use crate::geom::boolean::{self, FillRule};
use crate::geom::clip::{point_in_polygon, segments_intersect};
use crate::geom::inset::signed_area;
use crate::geom::poly::{point_in_face, point_to_segment_dist, ray_hit_face_edge};
use crate::types::{StallKind, StallModifier, StallQuad, Vec2};

/// Remove stalls where any shrunk corner falls outside the stall's own
/// face. This catches stalls that protrude past the face boundary into
/// corridors or adjacent faces.
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
            let shape = &faces[*face_idx];
            // Shrink 40% toward centroid for tolerance. Angled stalls have
            // corners that extend past the spine into the corridor, so a
            // proportional shrink handles all angles.
            let cx = stall.corners.iter().map(|c| c.x).sum::<f64>() / 4.0;
            let cy = stall.corners.iter().map(|c| c.y).sum::<f64>() / 4.0;
            let centroid = Vec2::new(cx, cy);
            let shrunk: Vec<Vec2> = stall
                .corners
                .iter()
                .map(|c| *c + (centroid - *c) * 0.4)
                .collect();
            // Every shrunk corner must be inside the stall's own face.
            if !shrunk.iter().all(|corner| point_in_face(*corner, shape)) {
                return false;
            }
            // Corner ray check: rays from p2 (dir p1→p2) and p3 (dir p0→p3)
            // must hit the same face boundary edge. Reject stalls that
            // straddle a face corner (each depth side projects to a
            // different edge).
            if !shape.is_empty() {
                let origin_a = stall.corners[2] + (centroid - stall.corners[2]) * 0.2;
                let origin_b = stall.corners[3] + (centroid - stall.corners[3]) * 0.2;
                let dir_a = (stall.corners[2] - stall.corners[1]).normalize();
                let dir_b = (stall.corners[3] - stall.corners[0]).normalize();
                let hit_a = ray_hit_face_edge(origin_a, dir_a, shape);
                let hit_b = ray_hit_face_edge(origin_b, dir_b, shape);
                match (hit_a, hit_b) {
                    (Some((ci_a, ei_a)), Some((ci_b, ei_b))) => {
                        if ci_a != ci_b || ei_a != ei_b {
                            return false;
                        }
                    }
                    _ => {
                        return false;
                    }
                }
            }
            true
        })
        .collect()
}

/// Test whether a stall quad is mostly contained in a face by computing
/// the boolean intersection `stall ∩ face`. If the intersection area is
/// less than `min_frac` of the stall area, the stall bleeds outside.
/// Angled stall corners that poke slightly into the corridor are
/// tolerated since they represent a small fraction of total area.
pub(crate) fn quad_contained_in_face(
    corners: &[Vec2; 4],
    face_shape: &[Vec<Vec2>],
    min_frac: f64,
) -> bool {
    let stall_area = signed_area(corners).abs();
    if stall_area < 1e-6 {
        return false;
    }

    let stall_subj = vec![corners.to_vec()];

    // Build face as multi-contour (outer CCW + holes CW).
    let mut face_paths: Vec<Vec<Vec2>> = Vec::new();
    for (i, contour) in face_shape.iter().enumerate() {
        let p = if i == 0 {
            if signed_area(contour) < 0.0 {
                contour.iter().rev().copied().collect()
            } else {
                contour.clone()
            }
        } else if signed_area(contour) > 0.0 {
            contour.iter().rev().copied().collect()
        } else {
            contour.clone()
        };
        face_paths.push(p);
    }

    let intersection = boolean::intersect(&stall_subj, &face_paths, FillRule::NonZero);

    let mut isect_area = 0.0;
    for shape in &intersection {
        for contour in shape {
            isect_area += signed_area(contour).abs();
        }
    }

    isect_area >= min_frac * stall_area
}

/// Test whether a stall quad is fully contained within the site boundary
/// (inside outer, outside all holes). Same geometric approach: centroid
/// inside + no edge crossings.
pub(crate) fn quad_contained_in_boundary(
    corners: &[Vec2; 4],
    raw_outer: &[Vec2],
    raw_holes: &[Vec<Vec2>],
) -> bool {
    let cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
    let cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
    let centroid = Vec2::new(cx, cy);

    if !point_in_polygon(&centroid, raw_outer) {
        return false;
    }
    for hole in raw_holes {
        if point_in_polygon(&centroid, hole) {
            return false;
        }
    }

    // No quad edge may cross any boundary edge.
    let raw_outer_vec = raw_outer.to_vec();
    let all_contours = std::iter::once(&raw_outer_vec).chain(raw_holes.iter());
    for contour in all_contours {
        let n = contour.len();
        for i in 0..4 {
            let a = corners[i];
            let b = corners[(i + 1) % 4];
            for j in 0..n {
                let c = contour[j];
                let d = contour[(j + 1) % n];
                if segments_intersect(a, b, c, d) {
                    return false;
                }
            }
        }
    }
    true
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
        StallKind::Standard | StallKind::Compact | StallKind::Ev | StallKind::Extension
    )
}

/// Shrink polygon vertices toward the centroid by a fixed distance
/// (overlap-test helper for conflict removal).
pub(crate) fn shrink_toward_centroid(corners: &[Vec2], amount: f64) -> Vec<Vec2> {
    let n = corners.len() as f64;
    let cx = corners.iter().map(|c| c.x).sum::<f64>() / n;
    let cy = corners.iter().map(|c| c.y).sum::<f64>() / n;
    let centroid = Vec2::new(cx, cy);
    corners
        .iter()
        .map(|c| {
            let d = *c - centroid;
            let len = d.length();
            if len < 1e-12 {
                return *c;
            }
            *c - d * (amount / len)
        })
        .collect()
}
