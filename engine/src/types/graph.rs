//! Drive-aisle graph topology + user-drawn drive lines.
//!
//! Defined in its own submodule (not `pipeline/graph`) so types that
//! cross module boundaries — e.g. `DriveAisleGraph` inside the public
//! `ParkingLayout` — don't pull in pipeline internals.

use serde::{Deserialize, Serialize};

use super::geom::Vec2;

// ---------------------------------------------------------------------------
// Drive-aisle graph
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum AisleDirection {
    TwoWay,
    TwoWayOriented,
    OneWay,
}

impl Default for AisleDirection {
    fn default() -> Self {
        AisleDirection::TwoWay
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AisleEdge {
    pub start: usize,
    pub end: usize,
    pub width: f64,
    #[serde(default)]
    pub interior: bool,
    #[serde(default)]
    pub direction: AisleDirection,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DriveAisleGraph {
    pub vertices: Vec<Vec2>,
    pub edges: Vec<AisleEdge>,
    #[serde(default)]
    pub perim_vertex_count: usize,
}

// ---------------------------------------------------------------------------
// Drive lines (user-drawn cutting lines clipped to boundary)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DriveLine {
    pub start: Vec2,
    pub end: Vec2,
    /// When set, this drive line is a separator pinned to a hole vertex.
    #[serde(default, rename = "holePin")]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HolePin {
    #[serde(rename = "holeIndex")]
    pub hole_index: usize,
    #[serde(rename = "vertexIndex")]
    pub vertex_index: usize,
}
