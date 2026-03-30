use crate::types::*;

/// Fill one side of a drive-aisle edge with a strip of angled stalls.
///
/// `side` is +1.0 or -1.0, selecting which side of the edge to place stalls.
/// `edge_width` is the half-width of the aisle edge (distance from centerline
/// to the near side of the stall row).
/// `angle_override` overrides `params.stall_angle_deg` when set.
pub fn fill_strip(
    edge_start: Vec2,
    edge_end: Vec2,
    side: f64,
    edge_width: f64,
    params: &ParkingParams,
    angle_override: Option<f64>,
) -> Vec<StallQuad> {
    let angle_deg = angle_override.unwrap_or(params.stall_angle_deg);
    let angle_rad = angle_deg.to_radians();
    let sin_a = angle_rad.sin();
    let cos_a = angle_rad.cos();

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
    let stall_count = (edge_len / stall_pitch).floor() as usize;
    if stall_count == 0 {
        return Vec::new();
    }

    let padding = (edge_len - stall_count as f64 * stall_pitch) / 2.0;

    // Depth direction: the direction a car drives into the stall.
    // At 90° this is the normal; at smaller angles it leans toward edge_dir.
    let depth_dir = normal * sin_a + edge_dir * cos_a;
    // Width direction: perpendicular to depth_dir within the plane.
    let width_dir = edge_dir * sin_a - normal * cos_a;

    let mut stalls = Vec::with_capacity(stall_count);

    // For boundary spines (angle overridden to 90°), the stall depth
    // matches the effective_depth (spine-to-corridor gap) so 90° stalls
    // fill exactly from spine to corridor edge.
    let stall_depth = if angle_override.is_some() {
        let theta = params.stall_angle_deg.to_radians();
        params.stall_depth * theta.sin() + theta.cos() * params.stall_width / 2.0
    } else {
        params.stall_depth
    };

    for i in 0..stall_count {
        // Midpoint of the stall's back (deep) side sits on the spine.
        let mid = edge_start
            + edge_dir * (padding + (i as f64 + 0.5) * stall_pitch)
            + normal * edge_width;
        let p0 = mid - width_dir * (params.stall_width / 2.0);
        let p1 = mid + width_dir * (params.stall_width / 2.0);
        let p2 = p1 + depth_dir * stall_depth;
        let p3 = p0 + depth_dir * stall_depth;

        stalls.push(StallQuad {
            corners: [p0, p1, p2, p3],
            kind: StallKind::Standard,
        });
    }

    stalls
}
