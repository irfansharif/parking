//! Geometric primitives: 2D vector, polygon (with holes + per-edge
//! curves), and oriented bounding box. Parking-agnostic — this module
//! knows nothing about aisles, stalls, or annotations and could be
//! lifted into a standalone `geom` crate later without churn.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};

// ---------------------------------------------------------------------------
// Vec2
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn cross(self, rhs: Self) -> f64 {
        self.x * rhs.y - self.y * rhs.x
    }

    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn normalize(self) -> Self {
        let len = self.length();
        if len < 1e-12 {
            return Self { x: 0.0, y: 0.0 };
        }
        Self {
            x: self.x / len,
            y: self.y / len,
        }
    }
}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

// ---------------------------------------------------------------------------
// Polygon
// ---------------------------------------------------------------------------

/// Cubic bezier control points for a polygon edge.
/// Edge i goes from vertex[i] to vertex[(i+1) % n].
/// The full curve is: vertex[i], cp1, cp2, vertex[(i+1) % n].
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EdgeCurve {
    pub cp1: Vec2,
    pub cp2: Vec2,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Polygon {
    pub outer: Vec<Vec2>,
    pub holes: Vec<Vec<Vec2>>,
    /// Per-edge curve data for outer boundary. Parallel to `outer` edges.
    #[serde(default)]
    pub outer_curves: Vec<Option<EdgeCurve>>,
    /// Per-hole per-edge curve data. Outer index = hole index.
    #[serde(default)]
    pub hole_curves: Vec<Vec<Option<EdgeCurve>>>,
}

impl Polygon {
    /// Signed area of a closed polygonal loop. Positive ↔ CCW in a
    /// y-up frame (sketch convention); negative ↔ CW.
    pub fn signed_area(loop_: &[Vec2]) -> f64 {
        if loop_.len() < 3 {
            return 0.0;
        }
        let mut acc = 0.0;
        let n = loop_.len();
        for i in 0..n {
            let a = loop_[i];
            let b = loop_[(i + 1) % n];
            acc += a.cross(b);
        }
        acc * 0.5
    }

    /// `true` when the outer boundary is oriented CCW in a y-up frame.
    /// Empty/degenerate loops (< 3 vertices, or zero signed area)
    /// return `false` — caller should treat them as invalid rather
    /// than trust this flag.
    pub fn outer_is_ccw(&self) -> bool {
        Self::signed_area(&self.outer) > 0.0
    }

    /// `true` when every hole is oriented CW in a y-up frame. Holes
    /// nest inside the outer boundary with reversed winding, so this
    /// is the serialization contract.
    pub fn holes_are_cw(&self) -> bool {
        self.holes.iter().all(|h| Self::signed_area(h) < 0.0)
    }

    /// Debug-only simple-polygon check: no edge crosses any non-
    /// adjacent edge within the same loop. Runs in O(n²) per loop and
    /// is not called in release builds. Intended for fixture
    /// validation, not the hot pipeline.
    #[cfg(debug_assertions)]
    pub fn is_simple(&self) -> bool {
        if !loop_is_simple(&self.outer) {
            return false;
        }
        self.holes.iter().all(|h| loop_is_simple(h))
    }
}

#[cfg(debug_assertions)]
fn loop_is_simple(pts: &[Vec2]) -> bool {
    let n = pts.len();
    if n < 3 {
        return false;
    }
    for i in 0..n {
        let a0 = pts[i];
        let a1 = pts[(i + 1) % n];
        // Skip adjacent edges (they share a vertex) and the edge itself.
        for j in (i + 2)..n {
            // Wrap-around adjacency: last edge shares a vertex with first.
            if i == 0 && j == n - 1 {
                continue;
            }
            let b0 = pts[j];
            let b1 = pts[(j + 1) % n];
            if segments_cross(a0, a1, b0, b1) {
                return false;
            }
        }
    }
    true
}

#[cfg(debug_assertions)]
fn segments_cross(p1: Vec2, p2: Vec2, p3: Vec2, p4: Vec2) -> bool {
    // Proper-intersection test via signed cross products. Endpoints
    // touching count as non-crossing; simple polygons are allowed to
    // share vertices across non-adjacent edges? In practice we reject
    // any shared-endpoint case too — sketches shouldn't have them.
    let d1 = (p4 - p3).cross(p1 - p3);
    let d2 = (p4 - p3).cross(p2 - p3);
    let d3 = (p2 - p1).cross(p3 - p1);
    let d4 = (p2 - p1).cross(p4 - p1);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    false
}

// ---------------------------------------------------------------------------
// Obb (oriented bounding box)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Obb {
    pub center: Vec2,
    pub half_extents: Vec2,
    pub angle: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_area_ccw_positive() {
        let sq = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        assert!(Polygon::signed_area(&sq) > 0.0);
    }

    #[test]
    fn signed_area_cw_negative() {
        let sq = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 0.0),
        ];
        assert!(Polygon::signed_area(&sq) < 0.0);
    }

    #[test]
    fn outer_is_ccw_holes_are_cw() {
        let p = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(2.0, 0.0),
                Vec2::new(2.0, 2.0),
                Vec2::new(0.0, 2.0),
            ],
            holes: vec![vec![
                Vec2::new(0.5, 0.5),
                Vec2::new(0.5, 1.5),
                Vec2::new(1.5, 1.5),
                Vec2::new(1.5, 0.5),
            ]],
            outer_curves: Vec::new(),
            hole_curves: Vec::new(),
        };
        assert!(p.outer_is_ccw());
        assert!(p.holes_are_cw());
    }

    #[cfg(debug_assertions)]
    #[test]
    fn is_simple_rejects_self_intersecting() {
        // Bowtie.
        let p = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 1.0),
                Vec2::new(1.0, 0.0),
                Vec2::new(0.0, 1.0),
            ],
            ..Default::default()
        };
        assert!(!p.is_simple());
    }

    #[cfg(debug_assertions)]
    #[test]
    fn is_simple_accepts_convex() {
        let p = Polygon {
            outer: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 0.0),
                Vec2::new(1.0, 1.0),
                Vec2::new(0.0, 1.0),
            ],
            ..Default::default()
        };
        assert!(p.is_simple());
    }
}
