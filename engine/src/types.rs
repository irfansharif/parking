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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Polygon {
    pub outer: Vec<Vec2>,
    pub holes: Vec<Vec<Vec2>>,
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

// ---------------------------------------------------------------------------
// Stalls
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum StallKind {
    Standard,
    Compact,
    Ev,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StallQuad {
    pub corners: [Vec2; 4],
    pub kind: StallKind,
}

// ---------------------------------------------------------------------------
// Faces (positive-space regions between corridors)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Face {
    pub contour: Vec<Vec2>,
    #[serde(default)]
    pub holes: Vec<Vec<Vec2>>,
    #[serde(default)]
    pub is_boundary: bool,
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
// Layout output
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Metrics {
    pub total_stalls: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingLayout {
    pub aisle_polygons: Vec<Vec<Vec2>>,
    pub stalls: Vec<StallQuad>,
    pub metrics: Metrics,
    pub resolved_graph: DriveAisleGraph,
    #[serde(default)]
    pub spines: Vec<SpineLine>,
    #[serde(default)]
    pub faces: Vec<Face>,
    #[serde(default)]
    pub miter_fills: Vec<Vec<Vec2>>,
    #[serde(default)]
    pub skeleton_debug: Vec<SkeletonDebug>,
    #[serde(default)]
    pub islands: Vec<Island>,
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
// Params
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParkingParams {
    pub stall_width: f64,
    pub stall_depth: f64,
    pub aisle_width: f64,
    pub stall_angle_deg: f64,
    pub aisle_angle_deg: f64,
    pub aisle_offset: f64,
    pub site_offset: f64,
    #[serde(default, alias = "cross_aisle_spacing")]
    pub cross_aisle_max_run: f64,
}

impl Default for ParkingParams {
    fn default() -> Self {
        Self {
            stall_width: 9.0,
            stall_depth: 18.0,
            aisle_width: 12.0,
            stall_angle_deg: 45.0,
            aisle_angle_deg: 90.0,
            aisle_offset: 0.0,
            site_offset: 0.0,
            cross_aisle_max_run: 15.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Drive-aisle graph (defined here to avoid circular deps)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum AisleDirection {
    TwoWay,
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
// Spine-based face types
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpineLine {
    pub start: Vec2,
    pub end: Vec2,
    pub normal: Vec2,
}

// ---------------------------------------------------------------------------
// Debug toggles (diagnostic flags for isolating pipeline stages)
// ---------------------------------------------------------------------------

fn default_true() -> bool {
    true
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DebugToggles {
    // Corridor merging
    #[serde(default = "default_true")]
    pub miter_fills: bool,
    #[serde(default = "default_true")]
    pub boundary_only_miters: bool,
    #[serde(default = "default_true")]
    pub spike_removal: bool,
    #[serde(default = "default_true")]
    pub hole_filtering: bool,

    // Face extraction
    #[serde(default = "default_true")]
    pub face_extraction: bool,

    // Spine generation
    #[serde(default)]
    pub face_simplification: bool,
    #[serde(default = "default_true")]
    pub edge_classification: bool,
    #[serde(default = "default_true")]
    pub spine_clipping: bool,

    // Spine post-processing
    #[serde(default = "default_true")]
    pub spine_dedup: bool,
    #[serde(default = "default_true")]
    pub spine_merging: bool,
    #[serde(default = "default_true")]
    pub short_spine_filter: bool,

    // Stall placement
    #[serde(default = "default_true")]
    pub stall_face_clipping: bool,

    // Boundary
    #[serde(default = "default_true")]
    pub boundary_clipping: bool,

    // Conflict removal
    #[serde(default = "default_true")]
    pub conflict_removal: bool,

    // Skeleton debug visualization
    #[serde(default)]
    pub skeleton_debug: bool,
}

impl Default for DebugToggles {
    fn default() -> Self {
        Self {
            miter_fills: true,
            boundary_only_miters: true,
            spike_removal: false,
            hole_filtering: true,
            face_extraction: true,
            face_simplification: false,
            edge_classification: true,
            spine_clipping: true,
            spine_dedup: true,
            spine_merging: true,
            short_spine_filter: true,
            stall_face_clipping: true,
            boundary_clipping: true,
            conflict_removal: true,
            skeleton_debug: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Drive lines (user-drawn cutting lines clipped to boundary)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DriveLine {
    pub start: Vec2,
    pub end: Vec2,
}

// ---------------------------------------------------------------------------
// Annotations (spatial intents that survive graph regeneration)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum Annotation {
    /// Mark the nearest aisle edge as one-way in the given travel direction.
    /// When `chain` is true (default), expands to the full collinear chain.
    OneWay {
        midpoint: Vec2,
        travel_dir: Vec2,
        #[serde(default = "default_true")]
        chain: bool,
    },
    /// Remove the nearest vertex and all its incident edges.
    DeleteVertex {
        point: Vec2,
    },
    /// Remove the nearest edge. When `chain` is true, removes the full
    /// collinear chain.
    DeleteEdge {
        midpoint: Vec2,
        edge_dir: Vec2,
        #[serde(default = "default_true")]
        chain: bool,
    },
}

// ---------------------------------------------------------------------------
// Top-level input
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GenerateInput {
    pub boundary: Polygon,
    pub aisle_graph: Option<DriveAisleGraph>,
    #[serde(default)]
    pub drive_lines: Vec<DriveLine>,
    #[serde(default)]
    pub annotations: Vec<Annotation>,
    pub params: ParkingParams,
    #[serde(default)]
    pub debug: DebugToggles,
}
