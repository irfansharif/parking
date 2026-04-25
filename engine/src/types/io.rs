//! Top-level input / output shapes and the tuning surface.
//!
//! Everything exposed to the UI lands in this module:
//!
//!   ParkingParams     — numeric knobs (widths, angles, offsets)
//!   DebugToggles      — per-stage diagnostic flags
//!   RegionOverride    — per-region angle/offset overrides
//!   GenerateInput     — what the UI hands to the engine
//!   ParkingLayout     — what the engine hands back

use serde::{Deserialize, Serialize};
use ts_rs::TS;

use super::addressing::{Annotation, RegionId};
use super::geom::{Polygon, Vec2};
use super::graph::{DriveAisleGraph, DriveLine};
use super::output::{
    Face, Island, Metrics, RegionDebug, SpineLine, StallKind, StallQuad,
};

// ---------------------------------------------------------------------------
// ParkingParams
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct ParkingParams {
    pub stall_width: f64,
    pub stall_depth: f64,
    pub aisle_width: f64,
    pub stall_angle_deg: f64,
    pub aisle_angle_deg: f64,
    pub aisle_offset: f64,
    pub site_offset: f64,
    /// Number of stalls along the aisle direction in one face — i.e.,
    /// between adjacent cross driving aisles. Drives the canvas-space
    /// length of a face along the aisle: dy = stalls_per_face *
    /// stall_pitch. Was `cross_aisle_max_run` (a float in stall-pitch
    /// units); now an integer count.
    #[serde(default)]
    pub stalls_per_face: u32,
    #[serde(default)]
    pub use_regions: bool,
    #[serde(default)]
    pub island_stall_interval: u32,
    /// Minimum stall count for a spine (primary + extensions combined)
    /// to survive the short-segment filter. 1 is an effective no-op —
    /// every spine with any stall meets the threshold.
    #[serde(default = "default_min_stalls_per_spine")]
    pub min_stalls_per_spine: u32,
    /// Chord-deflection tolerance (feet) used when discretizing curved
    /// boundary edges into straight-line polylines. Smaller = smoother
    /// arcs but more perimeter vertices. See `geom::arc::discretize_polygon`.
    #[serde(default = "default_arc_discretize_tolerance")]
    pub arc_discretize_tolerance: f64,
    /// Maximum angular difference (degrees) between two spine segments
    /// for them to be considered colinear and fused. Controls how
    /// aggressively chord-derived spines along a discretized arc collapse
    /// into a single straight segment in `try_merge_spines`.
    #[serde(default = "default_spine_merge_angle_deg")]
    pub spine_merge_angle_deg: f64,
    /// Endpoint-share tolerance (feet) for the spine-merge pass. Two
    /// spines must have a pair of endpoints within this distance to be
    /// candidates for fusion in `merge_collinear_spines`.
    #[serde(default = "default_spine_merge_endpoint_tol")]
    pub spine_merge_endpoint_tol: f64,
    /// Turn radius (feet) for island corner rounding. A disk of this
    /// radius is rolled inside each face polygon (morphological
    /// opening) before residual island extraction, so convex face
    /// corners become arcs rather than 90° wedges. Only takes effect
    /// when `DebugToggles.island_corner_rounding` is on.
    #[serde(default = "default_island_corner_radius")]
    pub island_corner_radius: f64,
}

fn default_min_stalls_per_spine() -> u32 {
    3
}

fn default_arc_discretize_tolerance() -> f64 {
    5.0
}

fn default_spine_merge_angle_deg() -> f64 {
    8.1
}

fn default_spine_merge_endpoint_tol() -> f64 {
    1.0
}

fn default_island_corner_radius() -> f64 {
    6.0
}

impl ParkingParams {
    /// Stall pitch: spacing between stall centers along a spine.
    pub fn stall_pitch(&self) -> f64 {
        let sin_a = self.stall_angle_deg.to_radians().sin();
        if sin_a.abs() > 1e-12 { self.stall_width / sin_a } else { self.stall_width }
    }

    /// Effective perpendicular depth of one stall row, compensating for
    /// angled stalls. At 90° this is just `stall_depth`. Matches the
    /// `effective_depth` used in auto-generation (see
    /// `aisle_graph.rs:compute_inset_d`).
    pub fn effective_depth(&self) -> f64 {
        let rad = self.stall_angle_deg.to_radians();
        self.stall_depth * rad.sin() + rad.cos() * self.stall_width / 2.0
    }
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
            stalls_per_face: 15,
            use_regions: false,
            island_stall_interval: 8,
            min_stalls_per_spine: 3,
            arc_discretize_tolerance: 5.0,
            spine_merge_angle_deg: 8.1,
            spine_merge_endpoint_tol: 1.0,
            island_corner_radius: 6.0,
        }
    }
}

// ---------------------------------------------------------------------------
// DebugToggles (diagnostic flags for isolating pipeline stages)
// ---------------------------------------------------------------------------

fn default_true() -> bool {
    true
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct DebugToggles {
    // Corridor merging
    #[serde(default = "default_true")]
    pub miter_fills: bool,
    #[serde(default = "default_true")]
    pub boundary_only_miters: bool,
    #[serde(default = "default_true")]
    pub spike_removal: bool,

    // Spine generation
    /// Trim each spine endpoint inward from its face edge based on the
    /// corner angle with the neighboring contour edge (see
    /// `offset_aisle_edges_to_spines`). Off ⇒ spines run the full face
    /// edge length.
    #[serde(default = "default_true")]
    pub spine_end_trim: bool,

    // Spine post-processing
    #[serde(default = "default_true")]
    pub spine_merging: bool,

    // Spine extensions (extend spines colinearly to face boundary)
    #[serde(default = "default_true")]
    pub spine_extensions: bool,

    // Stall placement
    #[serde(default = "default_true")]
    pub stall_face_clipping: bool,
    #[serde(default = "default_true")]
    pub entrance_on_face_filter: bool,

    // Conflict removal
    #[serde(default = "default_true")]
    pub conflict_removal: bool,

    // Island stall dilation (close sliver gaps in face-minus-stalls subtraction)
    #[serde(default = "default_true")]
    pub island_stall_dilation: bool,

    // Round island corners by morphological opening (erode-then-dilate
    // with round joins) of the face polygon before residual extraction.
    // Mimics drive-aisle turn-radius geometry so islands aren't sharp
    // 90° wedges. The opened polygon is also threaded through the
    // downstream stall-vs-face checks so stalls never overshoot the
    // rounded contour.
    #[serde(default = "default_true")]
    pub island_corner_rounding: bool,
}

impl Default for DebugToggles {
    fn default() -> Self {
        Self {
            miter_fills: true,
            boundary_only_miters: true,
            spike_removal: true,
            spine_end_trim: true,
            spine_merging: true,
            spine_extensions: true,
            stall_face_clipping: true,
            entrance_on_face_filter: true,
            conflict_removal: true,
            island_stall_dilation: true,
            island_corner_rounding: true,
        }
    }
}

// ---------------------------------------------------------------------------
// Top-level input
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct GenerateInput {
    pub boundary: Polygon,
    #[serde(default)]
    pub drive_lines: Vec<DriveLine>,
    #[serde(default)]
    pub annotations: Vec<Annotation>,
    pub params: ParkingParams,
    #[serde(default)]
    pub debug: DebugToggles,
    #[serde(default, rename = "regionOverrides")]
    #[ts(rename = "regionOverrides")]
    pub region_overrides: Vec<RegionOverride>,
    /// User-drawn post-pass modifiers that retype or suppress overlapping
    /// stalls (DESIGN §1.4). Empty = no-op. Applied after placement by
    /// `pipeline::filter::apply_stall_modifiers`.
    #[serde(default)]
    pub stall_modifiers: Vec<StallModifier>,
}

/// Post-placement modifier applied to stalls whose geometry overlaps
/// the modifier's polyline.
///
/// - `kind = Suppressed` + a polyline → remove stalls from rendering
///   (used for fire lanes, loading zones, entrance corridors).
/// - Any other `kind` → retype overlapping stalls (ADA, EV, Compact,
///   etc.). A zero-length polyline (single point) places a single
///   retyped stall at that location.
#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct StallModifier {
    pub polyline: Vec<Vec2>,
    pub kind: StallKind,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct RegionOverride {
    pub region_id: RegionId,
    #[ts(optional)]
    pub aisle_angle_deg: Option<f64>,
    #[ts(optional)]
    pub aisle_offset: Option<f64>,
}

// ---------------------------------------------------------------------------
// Top-level output
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct ParkingLayout {
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
    pub islands: Vec<Island>,
    /// Raw lot boundary derived by expanding the aisle-edge perimeter outward.
    #[serde(default)]
    pub derived_outer: Vec<Vec2>,
    /// Raw building footprints derived by shrinking aisle-edge rings inward.
    #[serde(default)]
    pub derived_holes: Vec<Vec<Vec2>>,
    /// Debug: region clip polygons and separator segments for visualization.
    #[serde(default)]
    pub region_debug: Option<RegionDebug>,
    /// Annotations that didn't resolve to any graph feature this pass.
    /// Each entry pairs the index into `GenerateInput.annotations` with
    /// a human-readable reason explaining why resolution failed (substrate
    /// missing, vertex id not on current sketch, no graph feature within
    /// tolerance, etc.). The annotation is stored but not applied this
    /// regenerate; the UI can surface these as "dormant."
    #[serde(default)]
    pub dormant_annotations: Vec<DormantAnnotation>,
}

#[derive(Clone, Debug, Serialize, Deserialize, TS)]
#[ts(export)]
pub struct DormantAnnotation {
    /// Index into `GenerateInput.annotations`.
    pub index: usize,
    /// Why the annotation couldn't be applied this regen.
    pub reason: String,
}
