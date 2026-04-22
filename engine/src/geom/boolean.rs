//! Thin wrapper over `i_overlay` for polygon boolean ops.
//!
//! Confines every `i_overlay` import to this module so callers work in
//! `Vec2`-shaped polygons and never touch the underlying API. The
//! return type is a list of shapes, where each shape is a `Vec<Vec<Vec2>>`:
//! the first contour is the outer ring, the remainder are holes.

use i_overlay::core::fill_rule::FillRule as IFillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

use crate::types::Vec2;

#[derive(Clone, Copy, Debug)]
pub enum FillRule {
    /// Cumulative winding ≠ 0 → inside. Tolerates mixed windings in
    /// union inputs.
    NonZero,
    /// Odd winding → inside. Useful for clip-to-ring operations where
    /// the clip ring is a single self-consistent loop.
    EvenOdd,
}

impl FillRule {
    fn to_i(self) -> IFillRule {
        match self {
            FillRule::NonZero => IFillRule::NonZero,
            FillRule::EvenOdd => IFillRule::EvenOdd,
        }
    }
}

/// Result shape: list of disjoint shapes, each with one outer contour
/// followed by zero-or-more hole contours.
pub type OverlayResult = Vec<Vec<Vec<Vec2>>>;

/// Union: A ∪ B. NonZero is almost always what you want for unioning
/// differently-wound contours.
pub fn union(subject: &[Vec<Vec2>], clip: &[Vec<Vec2>], fill: FillRule) -> OverlayResult {
    overlay_impl(subject, clip, OverlayRule::Union, fill)
}

/// Difference: A \ B. NonZero is the standard fill rule for this op.
pub fn difference(subject: &[Vec<Vec2>], clip: &[Vec<Vec2>], fill: FillRule) -> OverlayResult {
    overlay_impl(subject, clip, OverlayRule::Difference, fill)
}

/// Intersection: A ∩ B.
pub fn intersect(subject: &[Vec<Vec2>], clip: &[Vec<Vec2>], fill: FillRule) -> OverlayResult {
    overlay_impl(subject, clip, OverlayRule::Intersect, fill)
}

fn overlay_impl(
    subject: &[Vec<Vec2>],
    clip: &[Vec<Vec2>],
    rule: OverlayRule,
    fill: FillRule,
) -> OverlayResult {
    let subj: Vec<Vec<[f64; 2]>> = subject.iter().map(to_path).collect();
    let clip: Vec<Vec<[f64; 2]>> = clip.iter().map(to_path).collect();
    let raw = subj.overlay(&clip, rule, fill.to_i());
    raw.into_iter().map(shape_to_vec2).collect()
}

fn to_path(contour: &Vec<Vec2>) -> Vec<[f64; 2]> {
    contour.iter().map(|v| [v.x, v.y]).collect()
}

fn shape_to_vec2(shape: Vec<Vec<[f64; 2]>>) -> Vec<Vec<Vec2>> {
    shape
        .into_iter()
        .map(|contour| contour.into_iter().map(|p| Vec2::new(p[0], p[1])).collect())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sq(x: f64, y: f64, w: f64, h: f64) -> Vec<Vec2> {
        vec![
            Vec2::new(x, y),
            Vec2::new(x + w, y),
            Vec2::new(x + w, y + h),
            Vec2::new(x, y + h),
        ]
    }

    #[test]
    fn union_overlapping_squares_produces_single_shape() {
        let a = sq(0.0, 0.0, 10.0, 10.0);
        let b = sq(5.0, 5.0, 10.0, 10.0);
        let out = union(&[a], &[b], FillRule::NonZero);
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn difference_hole_in_square() {
        let outer = sq(0.0, 0.0, 10.0, 10.0);
        let hole = sq(2.0, 2.0, 6.0, 6.0);
        let out = difference(&[outer], &[hole], FillRule::NonZero);
        assert_eq!(out.len(), 1);
        // One outer + one hole contour.
        assert_eq!(out[0].len(), 2);
    }

    #[test]
    fn intersect_disjoint_squares_empty() {
        let a = sq(0.0, 0.0, 5.0, 5.0);
        let b = sq(100.0, 100.0, 5.0, 5.0);
        let out = intersect(&[a], &[b], FillRule::NonZero);
        assert!(out.is_empty());
    }
}
