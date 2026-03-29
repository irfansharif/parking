use crate::aisle_graph::{auto_generate, merge_with_auto};
use crate::clip::{clip_stalls_to_boundary, remove_conflicting_stalls};
use crate::face::generate_from_spines;
use crate::types::*;

pub fn generate(input: GenerateInput) -> ParkingLayout {
    let graph = match input.aisle_graph {
        Some(manual) => merge_with_auto(manual, &input.boundary, &input.params),
        None => auto_generate(&input.boundary, &input.params),
    };

    let (stalls, aisle_polygons, spines, faces, miter_fills, skeleton_debug, islands) =
        generate_from_spines(&graph, &input.boundary, &input.params, &input.debug);

    let stalls = if input.debug.boundary_clipping {
        clip_stalls_to_boundary(stalls, &input.boundary)
    } else {
        stalls
    };
    let stalls = remove_conflicting_stalls(stalls);

    let total = stalls.len();

    ParkingLayout {
        aisle_polygons,
        stalls,
        metrics: Metrics {
            total_stalls: total,
        },
        resolved_graph: graph,
        spines,
        faces,
        miter_fills,
        skeleton_debug,
        islands,
    }
}
