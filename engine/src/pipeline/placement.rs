//! §3.6 Stall placement along spines.
//!
//! The `fill_strip` primitive (one-side-of-an-edge stall row) plus
//! spine-level orchestration that iterates each spine and emits stalls.

use crate::types::*;

/// Fill one side of a drive-aisle edge with a strip of angled stalls.
///
/// `side` is +1.0 or -1.0, selecting which side of the edge to place stalls.
/// `edge_width` is the half-width of the aisle edge (distance from centerline
/// to the near side of the stall row).
/// `angle_override` overrides `params.stall_angle` when set.
/// `flip_angle` negates the cosine component, reversing the stall lean
/// direction. Used for one-way aisles where both sides lean the same way.
/// `grid_offset` shifts the global grid by a fraction of stall_pitch (0.0–1.0)
/// to stagger stalls from opposing spines in same-direction one-way faces.
///
/// Corner layout (spine horizontal, stalls growing away from aisle):
///
///   corners: [0]=back_left, [1]=back_right, [2]=aisle_right, [3]=aisle_left
///
/// The renderer draws [3]→[0]→[1]→[2] (left divider, back edge, right
/// divider) and leaves [2]→[3] open (the aisle/entrance edge).
pub fn fill_strip(
    edge_start: Vec2,
    edge_end: Vec2,
    side: f64,
    edge_width: f64,
    params: &ParkingParams,
    angle_override: Option<f64>,
    flip_angle: bool,
    grid_offset: f64,
) -> Vec<StallQuad> {
    let angle_deg = angle_override.unwrap_or(params.stall_angle);
    let angle_rad = angle_deg.to_radians();
    let sin_a = angle_rad.sin();
    let cos_a = if flip_angle { -angle_rad.cos() } else { angle_rad.cos() };

    if sin_a.abs() < 1e-12 {
        return Vec::new();
    }

    let edge_vec = edge_end - edge_start;
    let edge_len = edge_vec.length();
    if edge_len < 1e-12 {
        return Vec::new();
    }

    let edge_dir = edge_vec.normalize();
    // Normal points away from the aisle on the chosen side.
    let normal = Vec2::new(-edge_dir.y, edge_dir.x) * side;

    let stall_pitch = params.stall_width / sin_a;

    // Depth direction: the direction a car drives into the stall.
    // At 90° this is the normal; at smaller angles it leans toward edge_dir.
    let depth_dir = normal * sin_a + edge_dir * cos_a;
    // Width direction: perpendicular to depth_dir within the plane.
    let width_dir = edge_dir * sin_a - normal * cos_a;

    // For boundary spines (angle overridden to 90°), the stall depth
    // matches the effective_depth (spine-to-corridor gap) so 90° stalls
    // fill exactly from spine to corridor edge.
    let stall_depth = if angle_override.is_some() {
        let theta = params.stall_angle.to_radians();
        params.stall_depth * theta.sin() + theta.cos() * params.stall_width / 2.0
    } else {
        params.stall_depth
    };

    // Snap stall midpoints to a grid along the spine direction.
    // The grid_offset staggers by a fraction of stall_pitch for one-way
    // spine interleaving.
    let proj_start = edge_start.dot(edge_dir);
    let offset_proj = proj_start - grid_offset * stall_pitch;
    let k_min = (offset_proj / stall_pitch).ceil() as i64;
    let k_max = ((offset_proj + edge_len) / stall_pitch - 1.0).floor() as i64;

    if k_max < k_min {
        return Vec::new();
    }

    let mut stalls = Vec::with_capacity((k_max - k_min + 1) as usize);

    // Divider length: travel along depth_dir from the spine (normal=0)
    // to effective_depth (stall_depth * sin_a + cos_a * w/2). This
    // accounts for the corridor spacing that includes the width_dir
    // normal offset.
    let effective_depth = stall_depth * sin_a + cos_a.abs() * params.stall_width / 2.0;
    let divider_len = effective_depth / sin_a;

    for k in k_min..=k_max {
        let t = (k as f64 + 0.5) * stall_pitch - offset_proj;
        let mid = edge_start + edge_dir * t + normal * edge_width;

        // Back corners: on the spine, along width_dir (at the stall angle).
        // This preserves the braided/interlocking pattern where opposing
        // stalls' back edges zigzag against each other.
        let back_left  = mid - width_dir * (params.stall_width / 2.0);
        let back_right = mid + width_dir * (params.stall_width / 2.0);

        // Aisle (entrance) corners: at divider_len depth (reaching the
        // face edge), flat along edge_dir so adjacent stalls tile cleanly.
        let aisle_center = mid + depth_dir * divider_len;
        let aisle_left  = aisle_center - edge_dir * (stall_pitch / 2.0);
        let aisle_right = aisle_center + edge_dir * (stall_pitch / 2.0);

        // [0]=back_left, [1]=back_right, [2]=aisle_right, [3]=aisle_left
        stalls.push(StallQuad {
            corners: [back_left, back_right, aisle_right, aisle_left],
            kind: StallKind::Standard,
        });
    }

    stalls
}

/// Compute fill_strip parameters for a spine segment. All direction
/// math is precomputed at spine construction; this function just maps
/// flag bits to the fill_strip args.
pub(crate) fn spine_fill_params(seg: &SpineSegment) -> (Vec2, Vec2, Option<f64>, bool, f64) {
    let (start, end) = seg.oriented_endpoints();
    let angle_override = if seg.is_interior { None } else { Some(90.0) };
    let grid_offset = if seg.staggered { 0.5 } else { 0.0 };
    (start, end, angle_override, seg.flip_angle, grid_offset)
}

/// Place stalls on a spine.
pub(crate) fn fill_spine(seg: &SpineSegment, params: &ParkingParams) -> Vec<StallQuad> {
    let (start, end, angle_override, flip_angle, grid_offset) = spine_fill_params(seg);
    fill_strip(start, end, 1.0, 0.0, params, angle_override, flip_angle, grid_offset)
}

/// Place stalls on spine segments, returning stalls tagged with
/// (face_idx, spine_idx). Spines are processed longest-first so that
/// downstream conflict resolution favors the longer strip.
pub(crate) fn place_stalls_on_spines(
    spines: &[SpineSegment],
    params: &ParkingParams,
) -> Vec<(StallQuad, usize, usize)> {
    let mut order: Vec<(usize, f64)> = spines.iter().enumerate()
        .map(|(i, s)| (i, (s.end - s.start).length()))
        .collect();
    order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    let mut stalls = Vec::new();
    for &(spine_idx, _) in &order {
        let seg = &spines[spine_idx];
        for quad in fill_spine(seg, params) {
            stalls.push((quad, seg.face_idx, spine_idx));
        }
    }
    stalls
}

