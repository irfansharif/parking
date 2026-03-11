use crate::aisle_graph::{auto_generate, merge_with_auto};
use crate::clip::clip_stalls_to_boundary;
use crate::face::generate_from_spines;
use crate::island::generate_islands;
use crate::types::*;

fn stall_center(stall: &StallQuad) -> Vec2 {
    let mut cx = 0.0;
    let mut cy = 0.0;
    for c in &stall.corners {
        cx += c.x;
        cy += c.y;
    }
    Vec2::new(cx / 4.0, cy / 4.0)
}

pub fn generate(input: GenerateInput) -> ParkingLayout {
    let graph = match input.aisle_graph {
        Some(manual) => merge_with_auto(manual, &input.boundary, &input.params),
        None => auto_generate(&input.boundary, &input.params),
    };

    let (stalls, spine_islands, aisle_polygons, spines, faces, miter_fills, skeleton_debug) =
        generate_from_spines(&graph, &input.boundary, &input.params, &input.debug);

    let mut stalls = if input.debug.boundary_clipping {
        clip_stalls_to_boundary(stalls, &input.boundary)
    } else {
        stalls
    };

    // ADA stall assignment: mark the N stalls closest to a focus point
    // (default: centroid of the first hole, or bottom-left of boundary).
    if input.params.ada_count > 0 && !stalls.is_empty() {
        let focus = if let Some(hole) = input.boundary.holes.first() {
            let cx = hole.iter().map(|v| v.x).sum::<f64>() / hole.len() as f64;
            let cy = hole.iter().map(|v| v.y).sum::<f64>() / hole.len() as f64;
            Vec2::new(cx, cy)
        } else {
            input.boundary.outer[0]
        };

        let mut indices: Vec<usize> = (0..stalls.len()).collect();
        indices.sort_by(|&a, &b| {
            let ca = stall_center(&stalls[a]);
            let cb = stall_center(&stalls[b]);
            let da = (ca - focus).length();
            let db = (cb - focus).length();
            da.partial_cmp(&db).unwrap()
        });

        for &idx in indices.iter().take(input.params.ada_count) {
            stalls[idx].kind = StallKind::Ada;
        }
    }

    let mut islands = generate_islands(&stalls, &input.params);
    islands.extend(spine_islands);

    let ada_stalls = stalls
        .iter()
        .filter(|s| matches!(s.kind, StallKind::Ada))
        .count();
    let total = stalls.len();

    ParkingLayout {
        aisle_polygons,
        stalls,
        islands,
        metrics: Metrics {
            total_stalls: total,
            ada_stalls,
        },
        resolved_graph: graph,
        spines,
        faces,
        miter_fills,
        skeleton_debug,
    }
}
