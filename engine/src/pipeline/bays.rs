//! §3.3 Parking bay extraction.
//!
//! Subtracts merged corridor polygons and building footprints from the
//! site boundary to yield the positive-space regions where stalls are
//! placed. Returns `Vec<Shape>`, where each `Shape` is
//! `Vec<Vec<Vec2>>` with `shape[0]` = outer contour, `shape[1..]` =
//! holes within that face.
//!
//! The two subtractions run separately so that corridor contours
//! (which have consistent relative winding from the boolean union
//! output) never share a clip set with boundary hole contours (which
//! have user-determined winding). Mixing them in a single NonZero
//! operation causes winding interference.

use crate::geom::boolean::{self, FillRule};
use crate::geom::poly::{ensure_ccw, signed_area};
use crate::types::Vec2;

pub(crate) fn extract_faces(
    raw_outer: &[Vec2],
    merged_corridors: &[Vec<Vec<Vec2>>],
    raw_holes: &[Vec<Vec2>],
) -> Vec<Vec<Vec<Vec2>>> {
    // Step 1: boundary MINUS corridors.
    let subj = vec![raw_outer.to_vec()];
    let corridor_paths: Vec<Vec<Vec2>> = merged_corridors
        .iter()
        .flat_map(|shape| shape.iter().cloned())
        .collect();
    let after_corridors = boolean::difference(&subj, &corridor_paths, FillRule::NonZero);

    // Step 2: subtract raw building holes.
    if raw_holes.is_empty() {
        return after_corridors;
    }

    let hole_paths: Vec<Vec<Vec2>> = raw_holes.iter().map(|hole| ensure_ccw(hole.clone())).collect();

    // Feed step 1 output as subject paths for the second subtraction.
    let subj2: Vec<Vec<Vec2>> = after_corridors
        .into_iter()
        .flat_map(|shape| shape.into_iter())
        .collect();
    boolean::difference(&subj2, &hole_paths, FillRule::NonZero)
}

/// Normalize winding for multi-contour face input. Outer contour
/// (index 0) → CCW (positive signed area); hole contours (index 1+)
/// → CW (negative signed area). Skips contours with fewer than 3
/// vertices. Used by spine generation (`offset_aisle_edges_to_spines`)
/// and by the stall-vs-face containment check (`quad_fully_in_face`).
pub(crate) fn normalize_face_winding(shape: &[Vec<Vec2>]) -> Vec<Vec<Vec2>> {
    shape
        .iter()
        .enumerate()
        .filter_map(|(ci, c)| {
            if c.len() < 3 {
                return None;
            }
            let sa = signed_area(c);
            let needs_reverse = (ci == 0 && sa < 0.0) || (ci > 0 && sa > 0.0);
            Some(if needs_reverse {
                c.iter().rev().copied().collect()
            } else {
                c.clone()
            })
        })
        .collect()
}
