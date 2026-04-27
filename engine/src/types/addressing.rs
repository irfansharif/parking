//! Abstract coordinate systems + the annotations that ride on them.
//!
//! Contains:
//!
//! - `AbstractFrame` / `AbstractPoint2` — the per-region transform
//!   between world and the integer aisle lattice.
//! - `RegionId` — stable hash of a region's bounding signature.
//! - `Annotation` + `Target` + `GridStop` + `PerimeterLoop` + `Axis` —
//!   substrate-keyed edits that survive regen.

use serde::{Deserialize, Serialize};
use ts_rs::TS;

use super::geom::{Vec2, VertexId};
use super::graph::AisleDirection;
use super::io::ParkingParams;

// ---------------------------------------------------------------------------
// AbstractFrame
// ---------------------------------------------------------------------------
//
// Per-region transformation from the abstract integer grid into world
// space. The abstract grid is a full integer lattice of driving aisles
// (both axes, every integer line). An abstract point (x, y) maps to:
//
//     world(x, y) = origin_world + x · dx · x_dir + y · dy · y_dir
//
// Where `x_dir`, `y_dir` are orthonormal unit vectors derived from the
// region's effective aisle angle, `dx` is the canvas-space distance
// between adjacent parallel driving aisles, and `dy` is the canvas-space
// distance between adjacent cross driving aisles (= face length along the
// aisle = `stalls_per_face · stall_pitch`).
//
// Frames are derived fresh on every generate and never serialized.

#[derive(Clone, Copy, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct AbstractFrame {
    pub origin_world: Vec2,
    pub x_dir: Vec2,          // unit, perpendicular to parallel aisle
    pub y_dir: Vec2,          // unit, along parallel aisle
    pub dx: f64,              // 2*effective_depth + 2*aisle_width
    pub dy: f64,              // stalls_per_face * stall_pitch
    pub stalls_per_face: u32, // number of stalls along the aisle per face
}

/// Real-valued point in an abstract frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct AbstractPoint2 {
    pub x: f64,
    pub y: f64,
}

impl AbstractFrame {
    /// Compute the root frame for a lot — used for the lot-wide base
    /// orientation (no region override applied).
    pub fn root(params: &ParkingParams) -> Self {
        Self::from_params(params, params.aisle_angle_deg, params.aisle_offset)
    }

    /// Compute a frame for a region with an effective aisle angle and
    /// offset that may differ from the lot-wide base (e.g., via a
    /// `RegionOverride`).
    pub fn region(params: &ParkingParams, aisle_angle_deg: f64, aisle_offset: f64) -> Self {
        Self::from_params(params, aisle_angle_deg, aisle_offset)
    }

    fn from_params(params: &ParkingParams, aisle_angle_deg: f64, aisle_offset: f64) -> Self {
        // `aisle_angle_deg` is the world-space direction the parallel
        // driving aisles run in. `y_dir` is a unit vector along that
        // direction; `x_dir` is perpendicular (rotated +90°).
        let rad = aisle_angle_deg.to_radians();
        let y_dir = Vec2::new(rad.cos(), rad.sin());
        let x_dir = Vec2::new(-rad.sin(), rad.cos());

        let dx = 2.0 * params.effective_depth() + 2.0 * params.aisle_width;
        let stalls_per_face = params.stalls_per_face.max(1);
        let dy = (stalls_per_face as f64) * params.stall_pitch();

        // Shift the canvas-anchored origin along the perpendicular
        // axis by aisle_offset. When the user drags the aisle vector
        // in the UI, aisle_offset changes and the grid slides
        // accordingly — abstract annotations keyed by integer
        // (xi, yi) follow the shift because their world position is
        // computed as `origin_world + xi * dx * x_dir + yi * dy * y_dir`.
        let origin_world = x_dir * aisle_offset;

        Self {
            origin_world,
            x_dir,
            y_dir,
            dx,
            dy,
            stalls_per_face,
        }
    }

    /// Forward transform: abstract → world.
    pub fn forward(&self, p: AbstractPoint2) -> Vec2 {
        self.origin_world + self.x_dir * (p.x * self.dx) + self.y_dir * (p.y * self.dy)
    }

    /// Inverse transform: world → abstract. Relies on `x_dir` and
    /// `y_dir` being orthonormal, so the inverse is just a projection
    /// onto each axis followed by a division by the corresponding
    /// scale.
    pub fn inverse(&self, w: Vec2) -> AbstractPoint2 {
        let rel = w - self.origin_world;
        AbstractPoint2 {
            x: rel.dot(self.x_dir) / self.dx,
            y: rel.dot(self.y_dir) / self.dy,
        }
    }
}

// ---------------------------------------------------------------------------
// RegionId
// ---------------------------------------------------------------------------
//
// Stable identifier for a region derived from the decomposition inputs.
// Today, regions are built from consecutive pairs of separators on a
// shared hole (sorted CCW by hole-vertex index), so the identifying
// tuple is (hole_index, hole_vertex_start, hole_vertex_end). Packing
// those into a u64 gives a stable ID that:
//
// - does NOT shuffle when the sort order of separators changes
//   (the pair is stable, unlike the integer array position)
// - changes only when a separator is added, removed, or moved to a
//   different hole vertex
// - is deterministic across regenerates and serialization roundtrips
//
// 20 bits per field gives up to ~1M hole vertices per hole and ~1M
// holes per lot, comfortably beyond any realistic limit.

/// `RegionId` stays in JavaScript's safe-integer range by construction
/// (see `from_signature` packing), so we expose it to TS as `number`
/// rather than ts-rs's default `bigint` mapping for `u64`. Serde
/// (de)serializes through the inner `u64` via manual impls rather than
/// `#[serde(transparent)]` because ts-rs 12's serde-compat parser
/// doesn't understand that attribute and emits a noisy warning.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, TS)]
#[ts(export, type = "number")]
pub struct RegionId(pub u64);

impl Serialize for RegionId {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.0.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for RegionId {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Ok(RegionId(u64::deserialize(deserializer)?))
    }
}

impl RegionId {
    /// Build a stable ID from the three identifying integers. Ordered:
    /// the pair (vi, vj) is directed CCW around the hole and must not be
    /// swapped. Each field is 16 bits to keep the whole value inside
    /// JavaScript's 53-bit safe integer range after JSON roundtrip.
    /// The top bit is set to 0 to distinguish from the reserved
    /// single-region-fallback ID.
    pub fn from_signature(hole_index: usize, vi: usize, vj: usize) -> Self {
        let h = (hole_index as u64) & 0xFFFF;
        let i = (vi as u64) & 0xFFFF;
        let j = (vj as u64) & 0xFFFF;
        Self((h << 32) | (i << 16) | j)
    }

    /// Reserved ID for the single-region fallback case (no separators or
    /// only one separator per hole, so no enclosed regions exist). Uses
    /// a sentinel value with a high bit set, outside the range of
    /// `from_signature` outputs but still within JavaScript's safe
    /// integer range.
    pub fn single_region_fallback() -> Self {
        Self(1 << 48)
    }
}

// ---------------------------------------------------------------------------
// Annotations (spatial intents that survive graph regeneration)
// ---------------------------------------------------------------------------
//
// An annotation targets a vertex or one-or-more sub-edges by addressing a
// point (or, for grid lines, a span) in one substrate's own coordinate
// system. The three substrates — abstract grid (per region), drive lines,
// and the perimeter — each carry stable coordinates and a canonical
// direction. Constraint: no collinear stretches between any two substrates,
// so every pair crosses the other at most once per traversal and
// CrossesDriveLine / CrossesPerimeter stops resolve unambiguously.
//
// Single dormancy rule: any referenced substrate or stop missing from the
// current graph => annotation dormant this regen.

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[serde(tag = "kind")]
#[ts(export)]
pub enum Annotation {
    DeleteVertex {
        target: Target,
    },
    DeleteEdge {
        target: Target,
    },
    Direction {
        target: Target,
        /// Direction in the *carrier's* canonical frame (Grid: low→high
        /// along free axis; DriveLine: +t; Perimeter: +arc). The
        /// pipeline translates this into the corresponding edge-frame
        /// `AisleDirection` via `apply_*_direction` — which may map
        /// `OneWay` → `OneWayReverse` (or vice versa) if the edge's
        /// stored `(start, end)` order disagrees with carrier canonical.
        traffic: AisleDirection,
    },
}

/// A referenceable region in one substrate's own coord system.
/// - Grid.range = None     → whole grid line (edge annotations only).
/// - Grid.range = Some((s, s)) → a vertex.
/// - Grid.range = Some((s1, s2)), s1 != s2 → a span of sub-edges.
/// - DriveLine / Perimeter: a single parametric point. Vertex annotations
///   resolve to the graph vertex at the point (if one exists); edge
///   annotations resolve to the sub-edge containing the point.
#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[serde(tag = "on")]
#[ts(export)]
pub enum Target {
    Grid {
        region: RegionId,
        axis: Axis,
        coord: i32,
        /// None = whole grid line; Some((s, s)) = vertex; Some((s1, s2)) = span.
        range: Option<(GridStop, GridStop)>,
    },
    DriveLine {
        id: u32,
        /// Parametric t ∈ [0, 1] from drive_line.start → drive_line.end.
        t: f64,
    },
    Perimeter {
        #[serde(rename = "loop")]
        #[ts(rename = "loop")]
        loop_: PerimeterLoop,
        /// Sketch vertex at the start of the addressed edge (loop
        /// traversal order — the edge runs from `start` to `end`
        /// going forward along the loop's stored winding).
        start: VertexId,
        /// Sketch vertex at the end of the addressed edge.
        end: VertexId,
        /// Position along the edge in sketch parameter space:
        ///   straight edges — linear `lerp(start_pos, end_pos, t)`,
        ///   arc edges      — angular fraction along the circular arc.
        /// `t = 0` resolves to the start vertex; `t = 1` to the end.
        /// Vertex annotations use `t = 0` (the edge's start vertex).
        t: f64,
    },
}

/// A stop along a grid line — either an integer lattice coord or a crossing
/// with another substrate. For crossings, resolution picks the first
/// crossing along the carrier's canonical direction from the other stop
/// (well-defined under the no-collinear-stretches constraint).
#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[serde(tag = "at")]
#[ts(export)]
pub enum GridStop {
    Lattice {
        /// Integer coord on the axis the line runs along.
        other: i32,
    },
    CrossesDriveLine {
        id: u32,
    },
    CrossesPerimeter {
        #[serde(rename = "loop")]
        #[ts(rename = "loop")]
        loop_: PerimeterLoop,
    },
}

/// A perimeter loop on the sketch boundary: the unique outer boundary, or
/// a specific hole. In the prototype, holes are identified positionally;
/// the Arcol integration uses a stable member sketch-vertex id instead.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize, TS)]
#[serde(tag = "kind")]
#[ts(export)]
pub enum PerimeterLoop {
    Outer,
    Hole {
        index: usize,
    },
}

/// Which axis the grid line's fixed coordinate is on. `axis=X, coord=c` is
/// the line x=c (fixed x; runs along Y). Canonical direction is +along the
/// "run axis" (low → high) — i.e. +Y for axis=X, +X for axis=Y.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, TS)]
#[ts(export)]
pub enum Axis {
    X,
    Y,
}

#[cfg(test)]
mod frame_tests {
    use super::*;

    fn p(x: f64, y: f64) -> AbstractPoint2 {
        AbstractPoint2 { x, y }
    }

    fn close(a: Vec2, b: Vec2, eps: f64) -> bool {
        (a.x - b.x).abs() < eps && (a.y - b.y).abs() < eps
    }

    fn close_ap(a: AbstractPoint2, b: AbstractPoint2, eps: f64) -> bool {
        (a.x - b.x).abs() < eps && (a.y - b.y).abs() < eps
    }

    #[test]
    fn matches_row_spacing_formula() {
        // AbstractFrame::dx must match today's row_spacing in
        // aisle_graph.rs exactly (2 * effective_depth + 2 * aisle_width),
        // so the abstract grid lines up with the existing generator at
        // 90° perpendicular stalls.
        let params = ParkingParams::default();
        let frame = AbstractFrame::root(&params);
        let expected_dx =
            2.0 * params.effective_depth() + 2.0 * params.aisle_width;
        assert!((frame.dx - expected_dx).abs() < 1e-9);
    }

    #[test]
    fn dy_matches_stall_pitch_times_n() {
        let params = ParkingParams::default();
        let frame = AbstractFrame::root(&params);
        let expected_dy =
            (frame.stalls_per_face as f64) * params.stall_pitch();
        assert!((frame.dy - expected_dy).abs() < 1e-9);
    }

    #[test]
    fn origin_is_at_world_origin() {
        let frame = AbstractFrame::root(&ParkingParams::default());
        assert_eq!(frame.origin_world, Vec2::new(0.0, 0.0));
        assert!(close(frame.forward(p(0.0, 0.0)), Vec2::new(0.0, 0.0), 1e-9));
    }

    #[test]
    fn axes_are_orthonormal() {
        // Sweep a range of angles.
        for angle in [0.0, 30.0, 45.0, 60.0, 90.0, 135.0, 180.0, -45.0] {
            let mut params = ParkingParams::default();
            params.aisle_angle_deg = angle;
            let f = AbstractFrame::root(&params);
            assert!((f.x_dir.length() - 1.0).abs() < 1e-9);
            assert!((f.y_dir.length() - 1.0).abs() < 1e-9);
            assert!(f.x_dir.dot(f.y_dir).abs() < 1e-9);
        }
    }

    #[test]
    fn roundtrip_under_angle_sweep() {
        let test_points = [
            p(0.0, 0.0),
            p(1.0, 0.0),
            p(0.0, 1.0),
            p(3.0, 5.0),
            p(-2.0, -7.0),
            p(0.5, 1.5),
        ];
        for angle in [0.0, 15.0, 45.0, 90.0, 135.0, 180.0, 270.0, -30.0] {
            let mut params = ParkingParams::default();
            params.aisle_angle_deg = angle;
            let f = AbstractFrame::root(&params);
            for &pt in &test_points {
                let world = f.forward(pt);
                let back = f.inverse(world);
                assert!(
                    close_ap(pt, back, 1e-9),
                    "angle {angle}: {pt:?} -> {world:?} -> {back:?}"
                );
            }
        }
    }

    #[test]
    fn roundtrip_under_stretch_sweep() {
        for &stall_depth in &[12.0, 18.0, 24.0] {
            for &aisle_width in &[8.0, 12.0, 16.0] {
                for &stall_width in &[7.0, 9.0, 11.0] {
                    let mut params = ParkingParams::default();
                    params.stall_depth = stall_depth;
                    params.aisle_width = aisle_width;
                    params.stall_width = stall_width;
                    let f = AbstractFrame::root(&params);
                    let pt = p(2.5, -3.25);
                    let world = f.forward(pt);
                    let back = f.inverse(world);
                    assert!(close_ap(pt, back, 1e-9));
                }
            }
        }
    }

    #[test]
    fn roundtrip_under_stall_angle_sweep() {
        for &stall_angle in &[30.0, 45.0, 60.0, 75.0, 90.0] {
            let mut params = ParkingParams::default();
            params.stall_angle_deg = stall_angle;
            let f = AbstractFrame::root(&params);
            // dx should reduce to 2*stall_depth + 2*aisle_width at 90°.
            if (stall_angle - 90.0).abs() < 1e-9 {
                let expected =
                    2.0 * params.stall_depth + 2.0 * params.aisle_width;
                assert!((f.dx - expected).abs() < 1e-9);
            }
            let pt = p(1.0, 1.0);
            let world = f.forward(pt);
            let back = f.inverse(world);
            assert!(close_ap(pt, back, 1e-9));
        }
    }

    #[test]
    fn region_override_rotates_frame() {
        let params = ParkingParams::default(); // aisle_angle_deg = 90
        let base = AbstractFrame::root(&params);
        let rotated = AbstractFrame::region(&params, 45.0, params.aisle_offset);
        // Same scales, different axes.
        assert!((base.dx - rotated.dx).abs() < 1e-9);
        assert!((base.dy - rotated.dy).abs() < 1e-9);
        assert!((base.y_dir.dot(rotated.y_dir) - (45.0_f64).to_radians().cos()).abs() < 1e-9);
    }

    #[test]
    fn aisle_offset_shifts_origin() {
        // Dragging the aisle vector to set aisle_offset should shift
        // the grid origin along the perpendicular direction. An
        // annotation keyed by (xi=0, yi=0) should follow the shift.
        let mut params = ParkingParams::default();
        params.aisle_offset = 25.0;
        let frame = AbstractFrame::root(&params);
        let origin = frame.forward(p(0.0, 0.0));
        // origin_world = x_dir * aisle_offset. At aisle_angle_deg=90,
        // x_dir = (-sin(90), cos(90)) = (-1, 0), so origin = (-25, 0).
        assert!((origin.x - (-25.0)).abs() < 1e-9);
        assert!((origin.y - 0.0).abs() < 1e-9);

        // Roundtrip still holds under the shifted origin.
        let pt = p(3.0, 5.0);
        let back = frame.inverse(frame.forward(pt));
        assert!((back.x - pt.x).abs() < 1e-9);
        assert!((back.y - pt.y).abs() < 1e-9);
    }

    #[test]
    fn forward_distances_match_scales() {
        // Moving 1 unit along abstract x should produce dx canvas-space
        // distance, along abstract y should produce dy.
        let params = ParkingParams::default();
        let f = AbstractFrame::root(&params);
        let dx_world = f.forward(p(1.0, 0.0)) - f.forward(p(0.0, 0.0));
        let dy_world = f.forward(p(0.0, 1.0)) - f.forward(p(0.0, 0.0));
        assert!((dx_world.length() - f.dx).abs() < 1e-9);
        assert!((dy_world.length() - f.dy).abs() < 1e-9);
    }
}
