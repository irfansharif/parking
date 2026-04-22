//! Pipeline-output types: stalls, faces, spines, islands, and the
//! debug buckets for intermediate stages. Everything downstream of
//! aisle-polygon construction lives in this module.

use serde::{Deserialize, Serialize};

use super::geom::Vec2;

// ---------------------------------------------------------------------------
// Stalls
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum StallKind {
    Standard,
    Compact,
    Ev,
    Extension,
    Island,
    /// Stall suppressed by a `StallModifier` post-pass. The renderer
    /// skips these entirely; kept as a distinguished variant (rather
    /// than dropping the stall) so downstream consumers can still see
    /// where the suppression occurred.
    Suppressed,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
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
    /// Travel direction of the adjacent aisle. `None` for two-way or perimeter edges.
    pub travel_dir: Option<Vec2>,
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpineLine {
    pub start: Vec2,
    pub end: Vec2,
    pub normal: Vec2,
    #[serde(default)]
    pub is_extension: bool,
}

// ---------------------------------------------------------------------------
// Skeleton debug output
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SkeletonDebug {
    pub arcs: Vec<[Vec2; 2]>,
    pub nodes: Vec<Vec2>,
    /// Split event points where the wavefront separated into independent regions.
    pub split_nodes: Vec<Vec2>,
    /// Original polygon vertices from which skeleton arcs originate.
    pub sources: Vec<Vec2>,
}

// ---------------------------------------------------------------------------
// Metrics + region debug
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Metrics {
    pub total_stalls: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RegionDebug {
    /// One clip polygon per region, each with its aisle angle.
    pub regions: Vec<RegionInfo>,
    /// Separator line segments (hole corner → shortened endpoint).
    pub separators: Vec<(Vec2, Vec2)>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RegionInfo {
    pub id: super::addressing::RegionId,
    pub clip_poly: Vec<Vec2>,
    pub aisle_angle_deg: f64,
    pub aisle_offset: f64,
    pub center: Vec2,
}
