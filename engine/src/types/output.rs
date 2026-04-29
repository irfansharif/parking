//! Pipeline-output types: stalls, faces, spines, islands, and the
//! debug buckets for intermediate stages. Everything downstream of
//! aisle-polygon construction lives in this module.

use serde::{Deserialize, Serialize};
use ts_rs::TS;

use super::geom::Vec2;

// ---------------------------------------------------------------------------
// Stalls
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, TS)]
#[ts(export)]
pub enum StallKind {
    Standard,
    Ada,
    /// Striped no-park access aisle adjacent to one or more ADA
    /// stalls. Auto-inserted by `apply_stall_modifiers` when an ADA
    /// modifier line places a cluster; not user-paintable directly.
    /// Doesn't count toward `total_stalls`.
    Buffer,
    Compact,
    Island,
    /// Stall suppressed by a `StallModifier` post-pass. The renderer
    /// skips these entirely; kept as a distinguished variant (rather
    /// than dropping the stall) so downstream consumers can still see
    /// where the suppression occurred.
    Suppressed,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct StallQuad {
    pub corners: [Vec2; 4],
    pub kind: StallKind,
}

// ---------------------------------------------------------------------------
// Faces (positive-space regions between corridors)
// ---------------------------------------------------------------------------

/// Source of a face edge: either a site boundary wall or an aisle corridor.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EdgeSource {
    Wall,
    Aisle {
        corridor_idx: usize,
        interior: bool,
        travel_dir: Option<Vec2>,
        is_two_way_oriented: bool,
    },
}

/// A single edge of a face contour with its provenance.
#[derive(Clone, Debug)]
pub struct FaceEdge {
    pub start: Vec2,
    pub end: Vec2,
    pub source: EdgeSource,
}

/// A face with per-edge provenance tags. Computed once after `extract_faces()`
/// and consumed by all downstream classification/spine logic.
#[derive(Clone, Debug)]
pub struct TaggedFace {
    pub edges: Vec<FaceEdge>,
    pub hole_edges: Vec<Vec<FaceEdge>>,
    pub is_boundary: bool,
    pub wall_edge_indices: Vec<usize>,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct Face {
    pub contour: Vec<Vec2>,
    #[serde(default)]
    pub holes: Vec<Vec<Vec2>>,
    #[serde(default)]
    pub is_boundary: bool,
    /// Per-edge source labels for debug visualization ("wall", "interior", "perimeter").
    #[serde(default)]
    pub edge_sources: Vec<String>,
    /// Per-hole per-edge source labels.
    #[serde(default)]
    pub hole_edge_sources: Vec<Vec<String>>,
}

// ---------------------------------------------------------------------------
// Islands (landscape gaps between/at-ends-of stall rows)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct Island {
    pub contour: Vec<Vec2>,
    #[serde(default)]
    pub holes: Vec<Vec<Vec2>>,
    pub face_idx: usize,
}

// ---------------------------------------------------------------------------
// Spines
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct SpineSegment {
    pub start: Vec2,
    pub end: Vec2,
    pub outward_normal: Vec2,
    pub face_idx: usize,
    pub is_interior: bool,
    /// Stall-lean flip for this spine — the final XOR of OneWay's
    /// per-side asymmetry and the carrier aisle's per-edge
    /// TwoWayReverse flag. Computed once at spine construction;
    /// placement just reads it.
    pub flip_angle: bool,
    /// Half-pitch grid stagger. Held equal to `flip_angle` so that two
    /// opposing back-to-back spines (with antiparallel oriented_dir)
    /// produce a relative half-pitch offset whenever their leans
    /// disagree, interlocking their back edges; when leans agree the
    /// shifts cancel and back edges meet on the same line.
    pub staggered: bool,
    /// True if this spine borders a direction-annotated aisle (OneWay,
    /// OneWayReverse, or TwoWayReverse). Used to tie-break in stall
    /// conflict removal so annotated rows aren't dropped in favor of
    /// identically-long unannotated neighbors.
    pub is_annotated: bool,
}

impl SpineSegment {
    /// Direction oriented so the left normal aligns with `outward_normal`.
    pub fn oriented_dir(&self) -> Vec2 {
        let d = (self.end - self.start).normalize();
        let left = Vec2::new(-d.y, d.x);
        if left.dot(self.outward_normal) > 0.0 { d } else { d * -1.0 }
    }

    /// Start/end oriented so fill_strip's side=+1 places stalls toward
    /// the outward_normal direction.
    pub fn oriented_endpoints(&self) -> (Vec2, Vec2) {
        let d = (self.end - self.start).normalize();
        let left = Vec2::new(-d.y, d.x);
        if left.dot(self.outward_normal) > 0.0 {
            (self.start, self.end)
        } else {
            (self.end, self.start)
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct SpineLine {
    pub start: Vec2,
    pub end: Vec2,
    pub normal: Vec2,
}

// ---------------------------------------------------------------------------
// Metrics + region debug
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct Metrics {
    pub total_stalls: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct RegionDebug {
    /// One clip polygon per region, each with its aisle angle.
    pub regions: Vec<RegionInfo>,
    /// Separator line segments (hole corner → shortened endpoint).
    pub separators: Vec<(Vec2, Vec2)>,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct RegionInfo {
    pub id: super::addressing::RegionId,
    pub clip_poly: Vec<Vec2>,
    pub aisle_angle: f64,
    pub aisle_offset: f64,
    pub center: Vec2,
}
