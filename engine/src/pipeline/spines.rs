//! §3.5 Spine post-processing — validity passes on the raw spine
//! segments emitted by `compute_face_spines` before stall placement.
//!
//! Phase 7 scope: the three collinear-spine cleaners
//! (merge-collinear, try-merge, dedup-overlapping). Spine *generation*
//! (the weighted-skeleton wavefront extraction that produces the raw
//! `SpineSegment` vector) stays in `face.rs` until the final orchestrator
//! collapse; this file is just the reusable tail of the pipeline.

use crate::types::{SpineSegment, Vec2};

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
