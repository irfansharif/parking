//! Polygon offset / inset operations.
//!
//! The public entry point `inset_polygon` is a thin wrapper around
//! `i_overlay::OutlineOffset` with mitered joins — the crate handles
//! per-edge offsetting, miter intersections, edge-collapse detection,
//! and self-intersection cleanup. `raw_inset_polygon` keeps the
//! hand-rolled "no-cleanup miter polygon" for one specific consumer
//! (`derive_raw_holes`) that wants the self-intersecting result for a
//! follow-up boolean intersect against the source ring.

use i_overlay::mesh::outline::offset::OutlineOffset;
use i_overlay::mesh::style::{LineJoin, OutlineStyle};

use super::poly::signed_area;
use crate::types::Vec2;

/// Minimum interior corner angle (radians) below which a miter join
/// falls back to a bevel. Same threshold as the corridor stroker; in
/// practice every right-angled lot corner is well above this.
const MITER_MIN_SHARP_ANGLE: f64 = 15.0 * std::f64::consts::PI / 180.0;

/// Raw miter polygon: offset each edge by `d` and compute miter
/// intersections at each vertex. Self-intersects on concave inputs;
/// caller is expected to follow up with a boolean op against the
/// source ring to clean up. Used only by `derive_raw_holes`.
pub fn raw_inset_polygon(polygon: &[Vec2], d: f64) -> Vec<Vec2> {
    let n = polygon.len();
    if n < 3 {
        return Vec::new();
    }
    let sign = if signed_area(polygon) >= 0.0 { 1.0 } else { -1.0 };

    let mut edge_dirs: Vec<Vec2> = Vec::with_capacity(n);
    let mut normals: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let e = polygon[j] - polygon[i];
        edge_dirs.push(e);
        normals.push(Vec2::new(-e.y * sign, e.x * sign).normalize());
    }

    let mut result = Vec::with_capacity(n);
    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let a = polygon[prev] + normals[prev] * d;
        let da = edge_dirs[prev];
        let b = polygon[i] + normals[i] * d;
        let db = edge_dirs[i];
        let denom = da.cross(db);
        let miter = if denom.abs() < 1e-12 {
            polygon[i] + normals[i] * d
        } else {
            let t = (b - a).cross(db) / denom;
            a + da * t
        };
        result.push(miter);
    }
    result
}

/// Inset a polygon inward by uniform distance `d` (negative `d`
/// expands outward), with mitered corners. Returns the offset
/// polygon, or an empty vec if it collapses entirely or degenerates.
/// When the offset disconnects the input into multiple components,
/// returns the largest one by absolute area.
pub fn inset_polygon(polygon: &[Vec2], d: f64) -> Vec<Vec2> {
    let n = polygon.len();
    if n < 3 {
        return Vec::new();
    }
    if d == 0.0 {
        return polygon.to_vec();
    }
    let path: Vec<Vec<[f64; 2]>> = vec![polygon.iter().map(|v| [v.x, v.y]).collect()];
    let style = OutlineStyle::new(-d).line_join(LineJoin::Miter(MITER_MIN_SHARP_ANGLE));
    let result = path.outline(&style);

    // Pick the largest outer contour across all returned shapes.
    let best = result
        .into_iter()
        .filter_map(|shape| shape.into_iter().next())
        .max_by(|a, b| {
            path_area_abs(a).partial_cmp(&path_area_abs(b)).unwrap_or(std::cmp::Ordering::Equal)
        });

    match best {
        Some(c) if c.len() >= 3 => c.into_iter().map(|p| Vec2::new(p[0], p[1])).collect(),
        _ => Vec::new(),
    }
}

fn path_area_abs(path: &[[f64; 2]]) -> f64 {
    let n = path.len();
    if n < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..n {
        let p = path[i];
        let q = path[(i + 1) % n];
        a += p[0] * q[1] - q[0] * p[1];
    }
    (a * 0.5).abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signed_area_ccw() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        assert!((signed_area(&poly) - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_inset_square() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];
        let inset = inset_polygon(&poly, 10.0);
        assert!(inset.len() >= 3);
        // Should be roughly an 80x80 square inside the original.
        for v in &inset {
            assert!(v.x >= 9.0 && v.x <= 91.0);
            assert!(v.y >= 9.0 && v.y <= 91.0);
        }
    }

    #[test]
    fn test_inset_collapse() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        // Insetting by more than half the width should collapse.
        let inset = inset_polygon(&poly, 6.0);
        assert!(inset.is_empty());
    }
}
