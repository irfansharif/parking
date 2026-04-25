//! Geometric primitives: 2D vector, polygon (with holes + per-edge
//! circular arcs), and oriented bounding box. Parking-agnostic — this
//! module knows nothing about aisles, stalls, or annotations and could
//! be lifted into a standalone `geom` crate later without churn.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};
use ts_rs::TS;

// ---------------------------------------------------------------------------
// Vec2
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, TS)]
#[ts(export)]
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
// VertexId
// ---------------------------------------------------------------------------

/// Stable identifier for a sketch vertex. Allocated at input parse
/// time and threaded through the pipeline so that perimeter
/// annotations can address sketch edges as `(start, end)` pairs
/// instead of by array index or arc length. Mirrors Arcol's
/// `VertexId` (a `Brand<number, "VertexId">` over a 32-bit
/// integer) for an eventual integration.
///
/// Ids are stable within a single user-authored sketch — they
/// survive vertex insertion, deletion, and reordering elsewhere on
/// the loop. Splitting an edge with a new sketch vertex *does*
/// invalidate the original edge identity; resolution promotes the
/// annotation onto the appropriate child edge by length.
///
/// Post-discretization vertices that don't correspond to a sketch
/// corner carry synthetic ids — they're not addressable from
/// annotations but exist so that the parallel `outer_ids` /
/// `hole_ids` arrays stay 1:1 with the vertex lists.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct VertexId(pub u32);

impl VertexId {
    /// Sentinel synthetic id used while discretizing or before ids are
    /// allocated. Real sketch vertices always carry a value below
    /// `SYNTHETIC_BASE`.
    pub const SYNTHETIC_BASE: u32 = 1 << 30;

    pub fn is_synthetic(self) -> bool {
        self.0 >= Self::SYNTHETIC_BASE
    }
}

// ---------------------------------------------------------------------------
// Polygon
// ---------------------------------------------------------------------------

/// Circular arc along a polygon edge, AutoCAD-style bulge
/// parameterization. Edge i goes from `vertex[i]` to `vertex[(i+1) % n]`.
/// `bulge = sagitta / (chord_length / 2)` — `+1` is a semicircle on the
/// CCW side of the edge direction, `-1` a semicircle on the CW side,
/// `0` collapses to a straight line (and should be stored as `None`).
#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct EdgeArc {
    pub bulge: f64,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct Polygon {
    pub outer: Vec<Vec2>,
    pub holes: Vec<Vec<Vec2>>,
    /// Per-edge arc data for outer boundary. Parallel to `outer` edges.
    #[serde(default)]
    pub outer_arcs: Vec<Option<EdgeArc>>,
    /// Per-hole per-edge arc data. Outer index = hole index.
    #[serde(default)]
    pub hole_arcs: Vec<Vec<Option<EdgeArc>>>,
    /// Per-vertex stable ids for the outer ring. Parallel to `outer`.
    /// Auto-allocated by `ensure_ids` if missing on input.
    #[serde(default)]
    pub outer_ids: Vec<VertexId>,
    /// Per-vertex stable ids for each hole. Parallel to `holes`.
    #[serde(default)]
    pub hole_ids: Vec<Vec<VertexId>>,
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

    /// Allocate `VertexId`s for any vertex without one. Idempotent.
    /// Existing ids (from external input — Arcol, JSON, hand-written
    /// fixtures) are preserved. Newly allocated ids start above the
    /// max existing id, so re-running this on a partially-populated
    /// polygon stays consistent.
    pub fn ensure_ids(&mut self) {
        let outer_n = self.outer.len();
        let mut max_existing: u32 = 0;
        let mut have_existing = |ids: &Vec<VertexId>| {
            for v in ids {
                if !v.is_synthetic() && v.0 > max_existing {
                    max_existing = v.0;
                }
            }
        };
        if self.outer_ids.len() == outer_n {
            have_existing(&self.outer_ids);
        }
        for hi in 0..self.holes.len() {
            if let Some(ids) = self.hole_ids.get(hi) {
                if ids.len() == self.holes[hi].len() {
                    have_existing(ids);
                }
            }
        }
        let mut next: u32 = max_existing + 1;
        let mut alloc = || {
            let id = VertexId(next);
            next += 1;
            id
        };

        if self.outer_ids.len() != outer_n {
            self.outer_ids = (0..outer_n).map(|_| alloc()).collect();
        }
        if self.hole_ids.len() != self.holes.len() {
            self.hole_ids = vec![Vec::new(); self.holes.len()];
        }
        for hi in 0..self.holes.len() {
            if self.hole_ids[hi].len() != self.holes[hi].len() {
                self.hole_ids[hi] = (0..self.holes[hi].len()).map(|_| alloc()).collect();
            }
        }
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
            ..Default::default()
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
