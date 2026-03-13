use crate::types::*;

pub fn generate_islands(
    stalls: &[StallQuad],
    params: &ParkingParams,
) -> Vec<Island> {
    let mut islands = Vec::new();

    if params.max_run == 0 || stalls.is_empty() {
        return islands;
    }

    // Group stalls into rows. Detect row breaks when consecutive stalls
    // have aisle-edge midpoints that are far apart (> 2x stall pitch).
    let stall_pitch = if params.stall_angle_deg.to_radians().sin().abs() > 1e-6 {
        params.stall_width / params.stall_angle_deg.to_radians().sin()
    } else {
        params.stall_width
    };
    let break_threshold = stall_pitch * 3.0;

    let mut row_start = 0;
    for i in 0..stalls.len() {
        let is_row_break = if i + 1 < stalls.len() {
            let mid_a = midpoint(&stalls[i].corners[0], &stalls[i].corners[1]);
            let mid_b = midpoint(&stalls[i + 1].corners[0], &stalls[i + 1].corners[1]);
            (mid_b - mid_a).length() > break_threshold
        } else {
            true // last stall ends the row
        };

        if is_row_break {
            // Process row from row_start to i (inclusive)
            let row_len = i - row_start + 1;
            if row_len > params.max_run {
                // Insert islands at every max_run interval
                let mut pos = row_start + params.max_run;
                while pos <= i {
                    // Island goes between stalls[pos-1] and stalls[pos]
                    let prev = &stalls[pos - 1];
                    let curr = &stalls[pos];
                    let island = make_max_run_island(prev, curr, params);
                    islands.push(island);
                    pos += params.max_run;
                }
            }
            row_start = i + 1;
        }
    }

    islands
}

fn midpoint(a: &Vec2, b: &Vec2) -> Vec2 {
    Vec2::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
}

fn make_max_run_island(prev: &StallQuad, curr: &StallQuad, _params: &ParkingParams) -> Island {
    // The island sits between prev's right edge (corners[1], corners[2])
    // and curr's left edge (corners[0], corners[3]).
    // Use the gap between them, but ensure the island is island_width wide.

    // Island corners: from prev.corners[1] to curr.corners[0] (aisle side)
    // and from prev.corners[2] to curr.corners[3] (back side)
    let p0 = prev.corners[1];  // aisle-side, right of previous stall
    let p1 = curr.corners[0];  // aisle-side, left of current stall
    let p2 = curr.corners[3];  // back-side, left of current stall
    let p3 = prev.corners[2];  // back-side, right of previous stall

    Island {
        polygon: vec![p0, p1, p2, p3],
        kind: IslandKind::MaxRun,
    }
}
