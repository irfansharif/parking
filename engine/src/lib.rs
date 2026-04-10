pub mod types;
pub mod inset;
pub mod aisle_graph;
pub mod bezier;
pub mod segment;
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
    use crate::types::Annotation;

    let mut out = String::new();

    // Boundary outer polygon.
    out.push_str("polygon outer\n");
    for (i, v) in input.boundary.outer.iter().enumerate() {
        out.push_str(&format!("{},{}\n", fmt_coord(v.x), fmt_coord(v.y)));
        if let Some(Some(c)) = input.boundary.outer_curves.get(i) {
            out.push_str(&format!("curve {},{} {},{}\n",
                fmt_coord(c.cp1.x), fmt_coord(c.cp1.y),
                fmt_coord(c.cp2.x), fmt_coord(c.cp2.y)));
        }
    }
    out.push_str("----\n");
    out.push_str(&format!("polygon outer: {} vertices\n", input.boundary.outer.len()));

    // Boundary holes.
    for (hi, hole) in input.boundary.holes.iter().enumerate() {
        out.push_str("\npolygon hole\n");
        let hole_curves = input.boundary.hole_curves.get(hi);
        for (i, v) in hole.iter().enumerate() {
            out.push_str(&format!("{},{}\n", fmt_coord(v.x), fmt_coord(v.y)));
            if let Some(Some(c)) = hole_curves.and_then(|hc| hc.get(i)) {
                out.push_str(&format!("curve {},{} {},{}\n",
                    fmt_coord(c.cp1.x), fmt_coord(c.cp1.y),
                    fmt_coord(c.cp2.x), fmt_coord(c.cp2.y)));
            }
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
    if (p.site_offset - defaults.site_offset).abs() > 1e-6 {
        param_lines.push(format!("set site_offset={}", fmt_coord(p.site_offset)));
    }
    if p.stalls_per_face != defaults.stalls_per_face {
        param_lines.push(format!("set stalls_per_face={}", p.stalls_per_face));
    }
    if p.use_regions != defaults.use_regions {
        param_lines.push(format!("set use_regions={}", p.use_regions));
    }
    if p.island_stall_interval != defaults.island_stall_interval {
        param_lines.push(format!("set island_stall_interval={}", p.island_stall_interval));
    }
    for line in &param_lines {
        out.push_str(&format!("\n{}\n----\n{}\n", line, line));
    }

    // Drive lines.
    for dl in &input.drive_lines {
        let pin_suffix = match &dl.hole_pin {
            Some(p) => format!(" hole-pin={},{}", p.hole_index, p.vertex_index),
            None => String::new(),
        };
        out.push_str(&format!(
            "\ndrive-line{}\n{},{}\n{},{}\n----\ndrive-line: {},{} -> {},{}{}\n",
            pin_suffix,
            fmt_coord(dl.start.x), fmt_coord(dl.start.y),
            fmt_coord(dl.end.x), fmt_coord(dl.end.y),
            fmt_coord(dl.start.x), fmt_coord(dl.start.y),
            fmt_coord(dl.end.x), fmt_coord(dl.end.y),
            pin_suffix,
        ));
    }

    // Annotations.
    for ann in &input.annotations {
        match ann {
            Annotation::OneWay { midpoint, travel_dir, chain } => {
                out.push_str(&format!(
                    "\nannotation one-way{}\n{},{}\n{},{}\n----\nannotation one-way at {},{}\n",
                    if *chain { "" } else { " no-chain" },
                    fmt_coord(midpoint.x), fmt_coord(midpoint.y),
                    fmt_coord(travel_dir.x), fmt_coord(travel_dir.y),
                    fmt_coord(midpoint.x), fmt_coord(midpoint.y),
                ));
            }
            Annotation::TwoWayOriented { midpoint, travel_dir, chain } => {
                out.push_str(&format!(
                    "\nannotation two-way-oriented{}\n{},{}\n{},{}\n----\nannotation two-way-oriented at {},{}\n",
                    if *chain { "" } else { " no-chain" },
                    fmt_coord(midpoint.x), fmt_coord(midpoint.y),
                    fmt_coord(travel_dir.x), fmt_coord(travel_dir.y),
                    fmt_coord(midpoint.x), fmt_coord(midpoint.y),
                ));
            }
            Annotation::DeleteVertex { point } => {
                out.push_str(&format!(
                    "\nannotation delete-vertex\n{},{}\n----\nannotation delete-vertex at {},{}\n",
                    fmt_coord(point.x), fmt_coord(point.y),
                    fmt_coord(point.x), fmt_coord(point.y),
                ));
            }
            Annotation::DeleteEdge { midpoint, edge_dir, chain } => {
                out.push_str(&format!(
                    "\nannotation delete-edge{}\n{},{}\n{},{}\n----\nannotation delete-edge at {},{}\n",
                    if *chain { "" } else { " no-chain" },
                    fmt_coord(midpoint.x), fmt_coord(midpoint.y),
                    fmt_coord(edge_dir.x), fmt_coord(edge_dir.y),
                    fmt_coord(midpoint.x), fmt_coord(midpoint.y),
                ));
            }
            Annotation::AbstractDeleteVertex { region, xi, yi } => {
                out.push_str(&format!(
                    "\nannotation abstract-delete-vertex region=0x{:016x} x={} y={}\n----\nannotation abstract-delete-vertex region=0x{:016x} x={} y={}\n",
                    region.0, xi, yi, region.0, xi, yi,
                ));
            }
            Annotation::AbstractDeleteEdge { region, xa, ya, xb, yb } => {
                out.push_str(&format!(
                    "\nannotation abstract-delete-edge region=0x{:016x} from={},{} to={},{}\n----\nannotation abstract-delete-edge region=0x{:016x} from={},{} to={},{}\n",
                    region.0, xa, ya, xb, yb, region.0, xa, ya, xb, yb,
                ));
            }
        }
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

    // Region overrides. Keyed by RegionId (u64 hex) rather than the
    // old unstable integer index.
    for ov in &input.region_overrides {
        let mut parts = vec![format!("region=0x{:016x}", ov.region_id.0)];
        if let Some(a) = ov.aisle_angle_deg {
            parts.push(format!("angle={}", fmt_coord(a)));
        }
        if let Some(o) = ov.aisle_offset {
            parts.push(format!("offset={}", fmt_coord(o)));
        }
        let spec = parts.join(" ");
        out.push_str(&format!(
            "\nregion-override {}\n----\n{}\n",
            spec, spec,
        ));
    }

    // Generate.
    out.push_str("\ngenerate\n----\n");

    // Screenshot.
    out.push_str("\nscreenshot\n----\n");

    out
}
