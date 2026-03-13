use crate::types::*;

/// Fill one side of a drive-aisle edge with a strip of angled stalls.
///
/// `side` is +1.0 or -1.0, selecting which side of the edge to place stalls.
/// `edge_width` is the half-width of the aisle edge (distance from centerline
/// to the near side of the stall row).
pub fn fill_strip(
    edge_start: Vec2,
    edge_end: Vec2,
    side: f64,
    edge_width: f64,
    params: &ParkingParams,
) -> Vec<StallQuad> {
    let angle_rad = params.stall_angle_deg.to_radians();
    let sin_a = angle_rad.sin();

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

    // Stripe direction: at 90° (perpendicular stalls) the stripe points along
    // the normal; at smaller angles it leans toward the edge direction.
    let stripe = normal * angle_rad.sin() + edge_dir * angle_rad.cos();

    let mut stalls = Vec::with_capacity(stall_count);

    for i in 0..stall_count {
        let base = edge_start + edge_dir * (padding + i as f64 * stall_pitch) + normal * edge_width;
        let p0 = base;
        let p1 = base + edge_dir * stall_pitch;
        let p2 = p1 + stripe * params.stall_depth;
        let p3 = p0 + stripe * params.stall_depth;

        stalls.push(StallQuad {
            corners: [p0, p1, p2, p3],
            kind: StallKind::Standard,
        });
    }

    stalls
}
