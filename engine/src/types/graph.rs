//! Drive-aisle graph topology + user-drawn drive lines.
//!
//! Defined in its own submodule (not `pipeline/graph`) so types that
//! cross module boundaries — e.g. `DriveAisleGraph` inside the public
//! `ParkingLayout` — don't pull in pipeline internals.

use serde::{Deserialize, Serialize};
use ts_rs::TS;

use super::geom::Vec2;

// ---------------------------------------------------------------------------
// Drive-aisle graph
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, TS)]
#[ts(export)]
pub enum AisleDirection {
    /// Two-way with stall strips angled the opposite way from the
    /// default — a mirrored slash pattern. Treated as default two-way
    /// for spine geometry; the downstream effect is a per-edge
    /// `flip_angle` on the spines bordering this aisle in stall
    /// placement.
    TwoWayReverse,
    /// One-way; traffic flows along the edge's stored start → end.
    OneWay,
    /// One-way; traffic flows end → start (against stored order).
    /// Avoids mutating `(start, end)` when an annotation is applied —
    /// the canonical edge geometry stays stable, direction is carried
    /// entirely in the tag.
    OneWayReverse,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct AisleEdge {
    pub start: usize,
    pub end: usize,
    pub width: f64,
    #[serde(default)]
    pub interior: bool,
    /// `None` = default two-way (the unannotated state). `Some(_)`
    /// means an annotation has set this aisle's direction.
    #[serde(default)]
    pub direction: Option<AisleDirection>,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct DriveAisleGraph {
    pub vertices: Vec<Vec2>,
    pub edges: Vec<AisleEdge>,
    #[serde(default)]
    pub perim_vertex_count: usize,
}

// ---------------------------------------------------------------------------
// Drive lines (user-drawn cutting lines clipped to boundary)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct DriveLine {
    pub start: Vec2,
    pub end: Vec2,
    /// When set, this drive line is a separator pinned to a hole vertex.
    #[serde(default, rename = "holePin")]
    #[ts(rename = "holePin", optional)]
    pub hole_pin: Option<HolePin>,
    /// Stable identifier assigned by the UI at creation time. Splice
    /// annotations key off this id + a fractional position along the
    /// line, so the annotation survives rotation/stretch of the grid
    /// even though splice vertices don't sit on the abstract grid.
    /// Default 0 is the "no id" sentinel for legacy fixtures and is
    /// never matched by splice annotations.
    #[serde(default)]
    pub id: u32,
    /// When true, this drive line participates in planar-arrangement
    /// face enumeration and partitions the lot into regions. When
    /// false, it's a corridor-only line (aisle graph edge, no
    /// partitioning effect).
    #[serde(default)]
    pub partitions: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct HolePin {
    #[serde(rename = "holeIndex")]
    #[ts(rename = "holeIndex")]
    pub hole_index: usize,
    #[serde(rename = "vertexIndex")]
    #[ts(rename = "vertexIndex")]
    pub vertex_index: usize,
}
