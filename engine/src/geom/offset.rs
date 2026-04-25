//! Polygon offsetting (Minkowski sum/difference with a disk) via
//! `i_overlay`'s outline API. Confines `i_overlay`-v4's offset import
//! to this module so callers stay in `Vec2`-shaped polygons.

use i_overlay::mesh::outline::offset::OutlineOffset;
use i_overlay::mesh::style::{LineJoin, OutlineStyle};

use crate::types::Vec2;

/// Convert a face shape (outer + optional holes) into the i_overlay
/// shape format: `Vec<Vec<[f64; 2]>>` — first contour is the outer
/// ring, the rest are holes. Drops duplicate-position and colinear
/// interior vertices on the way: i_overlay's outline builder
/// `debug_assert!`s on adjacent colinear segments
/// (`UniqueSegmentsIter guarantee it`), which fires when the input
/// has redundant chord vertices left over from arc discretization.
fn shape_to_paths(shape: &[Vec<Vec2>]) -> Vec<Vec<[f64; 2]>> {
    shape.iter().map(|c| dedupe_loop(c)).collect()
}

fn dedupe_loop(loop_: &[Vec2]) -> Vec<[f64; 2]> {
    if loop_.len() < 3 {
        return loop_.iter().map(|v| [v.x, v.y]).collect();
    }
    // Pass 1: drop adjacent duplicates (cyclically).
    let mut deduped: Vec<Vec2> = Vec::with_capacity(loop_.len());
    for &v in loop_ {
        if let Some(last) = deduped.last() {
            if (v - *last).length() < 1e-9 {
                continue;
            }
        }
        deduped.push(v);
    }
    if deduped.len() >= 2 {
        let first = deduped[0];
        let last = *deduped.last().unwrap();
        if (first - last).length() < 1e-9 {
            deduped.pop();
        }
    }
    if deduped.len() < 3 {
        return deduped.into_iter().map(|v| [v.x, v.y]).collect();
    }
    // Pass 2: drop colinear interior vertices. Iterate until no more
    // vertices are removed (a removal can expose a new colinear
    // triplet at the seam).
    loop {
        let n = deduped.len();
        if n < 3 {
            break;
        }
        let mut next = Vec::with_capacity(n);
        let mut removed = false;
        for i in 0..n {
            let prev = deduped[(i + n - 1) % n];
            let curr = deduped[i];
            let nxt = deduped[(i + 1) % n];
            let d1 = curr - prev;
            let d2 = nxt - curr;
            // Cross product = 0 ⇒ colinear. Use a small absolute
            // epsilon — the dedupe pass already handled near-zero
            // edge lengths, so any remaining cross of ~0 is a true
            // colinear chord vertex (e.g. arc-discretized straight
            // section, or matched offset edges).
            let cross = d1.x * d2.y - d1.y * d2.x;
            if cross.abs() < 1e-9 && d1.dot(d2) > 0.0 {
                removed = true;
                continue;
            }
            next.push(curr);
        }
        deduped = next;
        if !removed {
            break;
        }
    }
    deduped.into_iter().map(|v| [v.x, v.y]).collect()
}

fn paths_to_shapes(raw: Vec<Vec<Vec<[f64; 2]>>>) -> Vec<Vec<Vec<Vec2>>> {
    raw.into_iter()
        .map(|shape| {
            shape
                .into_iter()
                .map(|contour| {
                    contour
                        .into_iter()
                        .map(|p| Vec2::new(p[0], p[1]))
                        .collect()
                })
                .collect()
        })
        .collect()
}

/// Morphological opening: erode the shape by `r` then dilate it back
/// by `r` with round joins. The resulting shape is the area reachable
/// by a disk of radius `r` rolling inside the input — convex corners
/// become `r`-radius arcs, thin protrusions narrower than `2r`
/// disappear, concave corners stay sharp.
///
/// `chord_angle_rad` controls how finely arcs are tessellated by
/// i_overlay (smaller → smoother but more vertices). 0.1 rad
/// (≈ 5.7°) gives ~32 segments per quarter-turn — plenty smooth at
/// typical parking-lot scales.
pub fn morph_open_round(shape: &[Vec<Vec2>], r: f64, chord_angle_rad: f64) -> Vec<Vec<Vec<Vec2>>> {
    if r <= 0.0 || shape.is_empty() || shape[0].len() < 3 {
        return vec![shape.to_vec()];
    }
    let join = LineJoin::Round(chord_angle_rad);
    let paths = shape_to_paths(shape);

    // Erode: negative offset shrinks the polygon inward.
    let eroded_style = OutlineStyle::new(-r).line_join(join.clone());
    let eroded = paths.outline(&eroded_style);
    if eroded.is_empty() {
        return vec![];
    }

    // Dilate: positive offset grows it back outward — round joins
    // produce the rounded corners.
    let dilate_style = OutlineStyle::new(r).line_join(join);
    let opened = eroded.outline(&dilate_style);
    paths_to_shapes(opened)
}
