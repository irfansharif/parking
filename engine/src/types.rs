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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum StallKind {
    Standard,
    Compact,
    Ev,
    Extension,
    Island,
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
    /// Raw lot boundary derived by expanding the aisle-edge perimeter outward.
    #[serde(default)]
    pub derived_outer: Vec<Vec2>,
    /// Raw building footprints derived by shrinking aisle-edge rings inward.
    #[serde(default)]
    pub derived_holes: Vec<Vec<Vec2>>,
    /// Debug: region clip polygons and separator segments for visualization.
    #[serde(default)]
    pub region_debug: Option<RegionDebug>,
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
    pub clip_poly: Vec<Vec2>,
    pub aisle_angle_deg: f64,
    pub aisle_offset: f64,
    pub center: Vec2,
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
    #[serde(default)]
    pub use_regions: bool,
    #[serde(default)]
    pub island_stall_interval: u32,
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

#[derive(Clone, Copy, Debug)]
pub struct AbstractFrame {
    pub origin_world: Vec2,
    pub x_dir: Vec2,          // unit, perpendicular to parallel aisle
    pub y_dir: Vec2,          // unit, along parallel aisle
    pub dx: f64,              // 2*effective_depth + 2*aisle_width
    pub dy: f64,              // stalls_per_face * stall_pitch
    pub stalls_per_face: u32, // number of stalls along the aisle per face
}

/// Real-valued point in an abstract frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AbstractPoint2 {
    pub x: f64,
    pub y: f64,
}

impl AbstractFrame {
    /// Compute the root frame for a lot — used for the lot-wide base
    /// orientation (no region override applied).
    pub fn root(params: &ParkingParams) -> Self {
        Self::from_params(params, params.aisle_angle_deg)
    }

    /// Compute a frame for a region with an effective aisle angle that
    /// may differ from the lot-wide base (e.g., via a `RegionOverride`).
    pub fn region(params: &ParkingParams, aisle_angle_deg: f64) -> Self {
        Self::from_params(params, aisle_angle_deg)
    }

    fn from_params(params: &ParkingParams, aisle_angle_deg: f64) -> Self {
        // `aisle_angle_deg` is the world-space direction the parallel
        // driving aisles run in. `y_dir` is a unit vector along that
        // direction; `x_dir` is perpendicular (rotated +90°).
        let rad = aisle_angle_deg.to_radians();
        let y_dir = Vec2::new(rad.cos(), rad.sin());
        let x_dir = Vec2::new(-rad.sin(), rad.cos());

        let dx = 2.0 * params.effective_depth() + 2.0 * params.aisle_width;
        // stalls_per_face comes from the current cross_aisle_max_run
        // parameter until P3 renames it. Rounded to an integer stall
        // count because the abstract grid indexes cross aisles at
        // integer positions.
        let stalls_per_face = params.cross_aisle_max_run.round().max(1.0) as u32;
        let dy = (stalls_per_face as f64) * params.stall_pitch();

        Self {
            origin_world: Vec2::new(0.0, 0.0),
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
        let rotated = AbstractFrame::region(&params, 45.0);
        // Same scales, different axes.
        assert!((base.dx - rotated.dx).abs() < 1e-9);
        assert!((base.dy - rotated.dy).abs() < 1e-9);
        assert!((base.y_dir.dot(rotated.y_dir) - (45.0_f64).to_radians().cos()).abs() < 1e-9);
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
            use_regions: false,
            island_stall_interval: 8,
        }
    }
}

// ---------------------------------------------------------------------------
// Drive-aisle graph (defined here to avoid circular deps)
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
    #[serde(default)]
    pub contour_simplification: bool,
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

    // Spine extensions (extend spines colinearly to face boundary)
    #[serde(default = "default_true")]
    pub spine_extensions: bool,

    // Stall placement
    #[serde(default = "default_true")]
    pub stall_centering: bool,
    #[serde(default = "default_true")]
    pub stall_face_clipping: bool,

    // Boundary
    #[serde(default = "default_true")]
    pub boundary_clipping: bool,

    // Conflict removal
    #[serde(default = "default_true")]
    pub conflict_removal: bool,

    // Short segment filter (remove spines with too few stalls after merge)
    #[serde(default = "default_true")]
    pub short_segment_filter: bool,

    // Edge provenance (tag face edges with source corridor/wall)
    #[serde(default = "default_true")]
    pub edge_provenance: bool,

    // Skeleton debug visualization
    #[serde(default)]
    pub skeleton_debug: bool,
}

impl Default for DebugToggles {
    fn default() -> Self {
        Self {
            miter_fills: true,
            boundary_only_miters: true,
            spike_removal: true,
            contour_simplification: true,
            hole_filtering: true,
            face_extraction: true,
            face_simplification: false,
            edge_classification: true,
            spine_clipping: true,
            spine_dedup: true,
            spine_merging: true,
            short_spine_filter: true,
            spine_extensions: true,
            stall_centering: true,
            stall_face_clipping: true,
            boundary_clipping: true,
            conflict_removal: true,
            short_segment_filter: false,
            edge_provenance: true,
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
    /// When set, this drive line is a separator pinned to a hole vertex.
    #[serde(default, rename = "holePin")]
    pub hole_pin: Option<HolePin>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HolePin {
    #[serde(rename = "holeIndex")]
    pub hole_index: usize,
    #[serde(rename = "vertexIndex")]
    pub vertex_index: usize,
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
    /// Mark the nearest aisle edge as two-way with oriented lanes. The
    /// `travel_dir` determines which side gets which lane direction, which
    /// in turn affects stall angles on adjacent faces. Full aisle width is
    /// preserved (unlike OneWay).
    TwoWayOriented {
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
    #[serde(default, rename = "regionOverrides")]
    pub region_overrides: Vec<RegionOverride>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RegionOverride {
    pub region_index: usize,
    pub aisle_angle_deg: Option<f64>,
    pub aisle_offset: Option<f64>,
}
