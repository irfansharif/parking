pub mod types;
pub mod inset;
pub mod aisle_graph;
pub mod segment;
pub mod island;
pub mod clip;
pub mod skeleton;
pub mod face;
pub mod generate;

use wasm_bindgen::prelude::*;
use crate::generate::generate;
use crate::types::{GenerateInput, ParkingParams};

#[wasm_bindgen]
pub fn generate_js(input_json: &str) -> String {
    let input: GenerateInput = serde_json::from_str(input_json).unwrap();
    let layout = generate(input);
    serde_json::to_string(&layout).unwrap()
}

/// Serialize a GenerateInput JSON into the datadriven test fixture format
/// used by tests/testdata/*.txt. Call from the browser console:
///   copy(debug_input_js(window.__parkingInput))
/// Then paste into a new .txt file under tests/testdata/.
#[wasm_bindgen]
pub fn debug_input_js(input_json: &str) -> String {
    let input: GenerateInput = serde_json::from_str(input_json).unwrap();
    format_fixture(&input)
}

fn fmt_coord(v: f64) -> String {
    if (v - v.round()).abs() < 1e-6 {
        format!("{}", v.round() as i64)
    } else {
        format!("{:.2}", v)
    }
}

fn format_fixture(input: &GenerateInput) -> String {
    let mut out = String::new();

    // Boundary outer polygon.
    out.push_str("polygon outer\n");
    for v in &input.boundary.outer {
        out.push_str(&format!("{},{}\n", fmt_coord(v.x), fmt_coord(v.y)));
    }
    out.push_str("----\n");
    out.push_str(&format!("polygon outer: {} vertices\n", input.boundary.outer.len()));

    // Boundary holes.
    for hole in &input.boundary.holes {
        out.push_str("\npolygon hole\n");
        for v in hole {
            out.push_str(&format!("{},{}\n", fmt_coord(v.x), fmt_coord(v.y)));
        }
        out.push_str("----\n");
        out.push_str(&format!("hole: {} vertices\n", hole.len()));
    }

    // Non-default params.
    let defaults = ParkingParams::default();
    let p = &input.params;
    let mut param_lines = Vec::new();
    if (p.stall_width - defaults.stall_width).abs() > 1e-6 {
        param_lines.push(format!("set stall_width={}", fmt_coord(p.stall_width)));
    }
    if (p.stall_depth - defaults.stall_depth).abs() > 1e-6 {
        param_lines.push(format!("set stall_depth={}", fmt_coord(p.stall_depth)));
    }
    if (p.aisle_width - defaults.aisle_width).abs() > 1e-6 {
        param_lines.push(format!("set aisle_width={}", fmt_coord(p.aisle_width)));
    }
    if (p.stall_angle_deg - defaults.stall_angle_deg).abs() > 1e-6 {
        param_lines.push(format!("set stall_angle_deg={}", fmt_coord(p.stall_angle_deg)));
    }
    if (p.aisle_angle_deg - defaults.aisle_angle_deg).abs() > 1e-6 {
        param_lines.push(format!("set aisle_angle_deg={}", fmt_coord(p.aisle_angle_deg)));
    }
    if (p.aisle_offset - defaults.aisle_offset).abs() > 1e-6 {
        param_lines.push(format!("set aisle_offset={}", fmt_coord(p.aisle_offset)));
    }
    if p.max_run != defaults.max_run {
        param_lines.push(format!("set max_run={}", p.max_run));
    }
    if (p.island_width - defaults.island_width).abs() > 1e-6 {
        param_lines.push(format!("set island_width={}", fmt_coord(p.island_width)));
    }
    if p.ada_count != defaults.ada_count {
        param_lines.push(format!("set ada_count={}", p.ada_count));
    }
    if (p.site_offset - defaults.site_offset).abs() > 1e-6 {
        param_lines.push(format!("set site_offset={}", fmt_coord(p.site_offset)));
    }
    for line in &param_lines {
        out.push_str(&format!("\n{}\n----\n{}\n", line, line));
    }

    // Aisle graph vertices and edges.
    if let Some(ref graph) = input.aisle_graph {
        out.push('\n');
        for (i, v) in graph.vertices.iter().enumerate() {
            out.push_str(&format!(
                "vertex add {},{}\n----\n{}\n\n",
                fmt_coord(v.x), fmt_coord(v.y), i
            ));
        }
        for e in &graph.edges {
            out.push_str(&format!(
                "edge add {},{}\n----\nedge {}->{}\n\n",
                e.start, e.end, e.start, e.end
            ));
        }
    }

    // Generate.
    out.push_str("\ngenerate\n----\n");

    // Screenshot.
    out.push_str("\nscreenshot\n----\n");

    out
}
