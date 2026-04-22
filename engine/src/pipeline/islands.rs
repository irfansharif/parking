//! Island computation: per-spine stall marking and face residual extraction.
//!
//! `mark_island_stalls` flips every Nth stall along each spine to
//! `StallKind::Island` using absolute-grid math so paired sides agree.
//! `compute_islands` then boolean-subtracts the surviving (non-island)
//! stalls from each face polygon; the residual gaps become landscape
//! island contours.

use crate::geom::boolean::{self, FillRule};
use crate::geom::inset::signed_area;
use crate::types::*;

pub(crate) fn stall_key(s: &StallQuad) -> [u64; 8] {
    [
        s.corners[0].x.to_bits(), s.corners[0].y.to_bits(),
        s.corners[1].x.to_bits(), s.corners[1].y.to_bits(),
        s.corners[2].x.to_bits(), s.corners[2].y.to_bits(),
        s.corners[3].x.to_bits(), s.corners[3].y.to_bits(),
    ]
}

pub(crate) fn stall_center(s: &StallQuad) -> Vec2 {
    (s.corners[0] + s.corners[1] + s.corners[2] + s.corners[3]) * 0.25
}

/// Mark every Nth stall per spine row as StallKind::Island. Uses
/// absolute position-based grid marking so paired sides agree on
/// which positions become islands even when extensions add different
/// stall counts per side.
pub(crate) fn mark_island_stalls(
    stalls_3: &mut [(StallQuad, usize, usize)],
    tagged: &mut [(StallQuad, usize)],
    spines: &[SpineSegment],
    params: &ParkingParams,
) {
    let interval = params.island_stall_interval as usize;
    if interval < 2 { return; }
    let target_rem = (interval / 2) as i64;

    // Group stall indices by spine_idx.
    let mut by_spine: std::collections::BTreeMap<usize, Vec<usize>> =
        std::collections::BTreeMap::new();
    for (i, (_, _, spine_idx)) in stalls_3.iter().enumerate() {
        by_spine.entry(*spine_idx).or_default().push(i);
    }

    let mut island_keys: std::collections::HashSet<[u64; 8]> = std::collections::HashSet::new();

    for (&spine_idx, indices) in &by_spine {
        if spine_idx >= spines.len() { continue; }
        let seg = &spines[spine_idx];
        let proj_dir = seg.oriented_dir();
        let pitch = if seg.is_interior { params.stall_pitch() } else { params.stall_width };

        let mut sorted: Vec<(f64, usize)> = indices.iter().map(|&idx| {
            let center = stall_center(&stalls_3[idx].0);
            (center.dot(proj_dir), idx)
        }).collect();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let count = sorted.len();
        if count < interval { continue; }

        let end_margin = if seg.is_interior { 1.5 * pitch } else { 0.5 * pitch };
        let (seg_start, seg_end) = seg.oriented_endpoints();
        let proj_min = seg_start.dot(proj_dir).min(seg_end.dot(proj_dir));
        let proj_max = seg_start.dot(proj_dir).max(seg_end.dot(proj_dir));
        for &(proj, idx) in &sorted {
            if proj - proj_min < end_margin || proj_max - proj < end_margin { continue; }
            let k = (proj / pitch).floor() as i64;
            if k.rem_euclid(interval as i64) == target_rem {
                island_keys.insert(stall_key(&stalls_3[idx].0));
            }
        }
    }

    for (s, _, _) in stalls_3.iter_mut() {
        if island_keys.contains(&stall_key(s)) {
            s.kind = StallKind::Island;
        }
    }
    for (s, _) in tagged.iter_mut() {
        if island_keys.contains(&stall_key(s)) {
            s.kind = StallKind::Island;
        }
    }
}

/// Subtract non-island stalls from each face polygon; residual contours
/// (with area >= `min_area`) are emitted as landscape islands.
pub(crate) fn compute_islands(
    faces: &[Vec<Vec<Vec2>>],
    all_stalls: &[StallQuad],
    min_area: f64,
) -> Vec<Island> {
    let ensure_ccw = |pts: &[Vec2]| -> Vec<Vec2> {
        if signed_area(pts) >= 0.0 { pts.to_vec() }
        else { pts.iter().rev().copied().collect() }
    };
    let ensure_cw = |pts: &[Vec2]| -> Vec<Vec2> {
        if signed_area(pts) <= 0.0 { pts.to_vec() }
        else { pts.iter().rev().copied().collect() }
    };

    let all_stall_paths: Vec<Vec<Vec2>> = all_stalls.iter()
        .filter(|s| s.kind != StallKind::Island)
        .map(|s| s.corners.to_vec())
        .collect();

    let mut islands = Vec::new();

    for (face_idx, shape) in faces.iter().enumerate() {
        if shape.is_empty() || shape[0].len() < 3 { continue; }

        let mut subj: Vec<Vec<Vec2>> = vec![ensure_ccw(&shape[0])];
        for hole in shape.iter().skip(1) {
            subj.push(ensure_cw(hole));
        }

        if all_stall_paths.is_empty() {
            let contour = ensure_ccw(&shape[0]);
            if signed_area(&contour).abs() >= min_area {
                islands.push(Island { contour, holes: vec![], face_idx });
            }
            continue;
        }

        let raw = boolean::difference(&subj, &all_stall_paths, FillRule::NonZero);

        for sc in raw {
            if sc.is_empty() { continue; }
            let contour = ensure_ccw(&sc[0]);
            let holes: Vec<Vec<Vec2>> = sc[1..].iter().cloned().collect();
            let net = signed_area(&contour).abs()
                - holes.iter().map(|h| signed_area(h).abs()).sum::<f64>();
            if net < min_area { continue; }
            islands.push(Island { contour, holes, face_idx });
        }
    }

    islands
}
