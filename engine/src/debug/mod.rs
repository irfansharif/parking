//! Human-readable pretty-printers for engine inputs.
//!
//! Shared by the wasm `debug_input_js` entry point and the native
//! test harness — one source of truth for the `.txt` fixture DSL.
//! Keeps formatting logic off of `lib.rs`.

use crate::types::*;

fn fmt_coord(v: f64) -> String {
    if (v - v.round()).abs() < 1e-6 {
        format!("{}", v.round() as i64)
    } else {
        format!("{:.2}", v)
    }
}

fn fmt_bulge(v: f64) -> String {
    // Bulges are small (|b| ≤ 1), so three decimals is a better default
    // than `fmt_coord`'s two. Integers (e.g. 0, 1) still round-trip
    // cleanly.
    if (v - v.round()).abs() < 1e-6 {
        format!("{}", v.round() as i64)
    } else {
        format!("{:.3}", v)
    }
}

/// Dump a `GenerateInput` as a fixture text body matching the
/// `engine/tests/testdata/*.txt` grammar consumed by `runner.rs` /
/// `harness/directives.rs`. Expected blocks are intentionally left
/// empty — running `UPDATE_SNAPSHOTS=1 cargo test --test runner`
/// against the fixture file fills them in. The first run also emits
/// the `<fixture-stem>.svg` golden via the auto-snapshot pass.
pub fn format_fixture(input: &GenerateInput) -> String {
    let mut out = String::new();

    // Boundary outer polygon: `(x,y)` body lines, optional inline
    // `arc=<bulge>` suffix.
    out.push_str("polygon outer\n");
    emit_vertex_block(&mut out, &input.boundary.outer, &input.boundary.outer_arcs);
    out.push_str("----\n");

    // Boundary holes.
    for (hi, hole) in input.boundary.holes.iter().enumerate() {
        out.push_str("\npolygon hole\n");
        let empty: Vec<Option<EdgeArc>> = Vec::new();
        let hole_arcs = input.boundary.hole_arcs.get(hi).unwrap_or(&empty);
        emit_vertex_block(&mut out, hole, hole_arcs);
        out.push_str("----\n");
    }

    // Non-default params, dash form, silent (the harness's `set`
    // directive produces no echo, so the expected block stays empty).
    let defaults = ParkingParams::default();
    let p = &input.params;
    let mut param_lines: Vec<String> = Vec::new();
    if (p.stall_width - defaults.stall_width).abs() > 1e-6 {
        param_lines.push(format!("set stall-width={}", fmt_coord(p.stall_width)));
    }
    if (p.stall_depth - defaults.stall_depth).abs() > 1e-6 {
        param_lines.push(format!("set stall-depth={}", fmt_coord(p.stall_depth)));
    }
    if (p.aisle_width - defaults.aisle_width).abs() > 1e-6 {
        param_lines.push(format!("set aisle-width={}", fmt_coord(p.aisle_width)));
    }
    if (p.stall_angle - defaults.stall_angle).abs() > 1e-6 {
        param_lines.push(format!("set stall-angle={}", fmt_coord(p.stall_angle)));
    }
    if (p.aisle_angle - defaults.aisle_angle).abs() > 1e-6 {
        param_lines.push(format!("set aisle-angle={}", fmt_coord(p.aisle_angle)));
    }
    if (p.aisle_offset - defaults.aisle_offset).abs() > 1e-6 {
        param_lines.push(format!("set aisle-offset={}", fmt_coord(p.aisle_offset)));
    }
    if (p.site_offset - defaults.site_offset).abs() > 1e-6 {
        param_lines.push(format!("set site-offset={}", fmt_coord(p.site_offset)));
    }
    if p.stalls_per_face != defaults.stalls_per_face {
        param_lines.push(format!("set stalls-per-face={}", p.stalls_per_face));
    }
    if p.use_regions != defaults.use_regions {
        param_lines.push(format!("set use-regions={}", p.use_regions));
    }
    if p.island_stall_interval != defaults.island_stall_interval {
        param_lines.push(format!("set island-stall-interval={}", p.island_stall_interval));
    }
    if p.min_stalls_per_spine != defaults.min_stalls_per_spine {
        param_lines.push(format!("set min-stalls-per-spine={}", p.min_stalls_per_spine));
    }
    if (p.ada_stall_width - defaults.ada_stall_width).abs() > 1e-6 {
        param_lines.push(format!("set ada-stall-width={}", fmt_coord(p.ada_stall_width)));
    }
    if (p.compact_stall_width - defaults.compact_stall_width).abs() > 1e-6 {
        param_lines.push(format!(
            "set compact-stall-width={}",
            fmt_coord(p.compact_stall_width),
        ));
    }
    if (p.ada_buffer_width - defaults.ada_buffer_width).abs() > 1e-6 {
        param_lines.push(format!("set ada-buffer-width={}", fmt_coord(p.ada_buffer_width)));
    }
    if p.ada_buffer_shared != defaults.ada_buffer_shared {
        param_lines.push(format!("set ada-buffer-shared={}", p.ada_buffer_shared));
    }
    for line in &param_lines {
        out.push_str(&format!("\n{}\n----\n", line));
    }

    // Structural drive lines.
    for dl in &input.drive_lines {
        let suffix = if dl.partitions { " partitions" } else { "" };
        out.push_str(&format!(
            "\ndrive-line{}\n({},{})\n({},{})\n----\n",
            suffix,
            fmt_coord(dl.start.x),
            fmt_coord(dl.start.y),
            fmt_coord(dl.end.x),
            fmt_coord(dl.end.y),
        ));
    }

    // Stall-modifier lines — same `drive-line` directive with a
    // `stall=<kind>` flag. The harness routes these to the
    // `stall_modifiers` field instead of `drive_lines`. Buffer-kind
    // modifiers aren't user-paintable (auto-inserted by the engine),
    // so the dump skips them.
    for sm in &input.stall_modifiers {
        if sm.polyline.len() < 2 {
            continue;
        }
        let kind_str = match sm.kind {
            StallKind::Standard => "standard",
            StallKind::Ada => "ada",
            StallKind::Compact => "compact",
            StallKind::Island => "island",
            StallKind::Suppressed => "suppressed",
            StallKind::Buffer => continue,
        };
        let a = sm.polyline.first().unwrap();
        let b = sm.polyline.last().unwrap();
        out.push_str(&format!(
            "\ndrive-line stall={}\n({},{})\n({},{})\n----\n",
            kind_str,
            fmt_coord(a.x),
            fmt_coord(a.y),
            fmt_coord(b.x),
            fmt_coord(b.y),
        ));
    }

    // Annotations — `<kind>` on the directive line, KV args on a body
    // line. The body may carry `:` / `,` / `→` (e.g. grid range stops
    // and perimeter `edge=A→B`), which datadriven's directive parser
    // rejects, hence the body-line split.
    for ann in &input.annotations {
        let (kind_word, args) = match ann {
            Annotation::DeleteVertex { target } => ("delete-vertex", format_target(target)),
            Annotation::DeleteEdge { target } => ("delete-edge", format_target(target)),
            Annotation::Direction { target, traffic } => (
                "direction",
                format!("{} traffic={}", format_target(target), format_traffic(*traffic)),
            ),
        };
        out.push_str(&format!("\nannotation {}\n{}\n----\n", kind_word, args));
    }

    // Region overrides — hex form. The harness also accepts an
    // `(x,y)` anchor; we don't have post-generate region info here
    // so hex is the stable choice.
    for ov in &input.region_overrides {
        let mut spec = format!("region=0x{:016x}", ov.region_id.0);
        if let Some(a) = ov.aisle_angle {
            spec.push_str(&format!(" angle={}", fmt_coord(a)));
        }
        if let Some(o) = ov.aisle_offset {
            spec.push_str(&format!(" offset={}", fmt_coord(o)));
        }
        out.push_str(&format!("\nregion-override {}\n----\n", spec));
    }

    // Final `generate`. `runner.rs` auto-snapshots after the last
    // directive, so no explicit `screenshot` is needed.
    out.push_str("\ngenerate\n----\n");

    out
}

/// Emit a vertex block in the current `(x,y) [arc=<bulge>]` form
/// consumed by `parse_vertex_block` in the harness.
fn emit_vertex_block(out: &mut String, verts: &[Vec2], arcs: &[Option<EdgeArc>]) {
    for (i, v) in verts.iter().enumerate() {
        match arcs.get(i).and_then(|a| a.as_ref()) {
            Some(a) => out.push_str(&format!(
                "({},{}) arc={}\n",
                fmt_coord(v.x),
                fmt_coord(v.y),
                fmt_bulge(a.bulge),
            )),
            None => out.push_str(&format!("({},{})\n", fmt_coord(v.x), fmt_coord(v.y))),
        }
    }
}

/// Stable text form of an annotation for fixture round-tripping.
/// Format:
///   delete-vertex   on=<substrate-spec>
///   delete-edge     on=<substrate-spec>
///   direction       on=<substrate-spec> traffic=<traffic>
/// where substrate-spec is:
///   grid region=0x... axis=<x|y> coord=<n> [range=whole | at=<stop> | range=<s1>,<s2>]
///   drive-line id=<n> t=<t>
///   perimeter loop=<loop> arc=<arc>
/// and <stop> is lattice:<n> | crosses-drive-line:<id> | crosses-perimeter:<loop>
/// and <loop> is outer | hole:<n>
pub fn format_annotation_line(ann: &crate::types::Annotation) -> String {
    use crate::types::Annotation;
    match ann {
        Annotation::DeleteVertex { target } => format!("delete-vertex {}", format_target(target)),
        Annotation::DeleteEdge { target } => format!("delete-edge {}", format_target(target)),
        Annotation::Direction { target, traffic } => format!(
            "direction {} traffic={}",
            format_target(target),
            format_traffic(*traffic),
        ),
    }
}

fn format_target(t: &crate::types::Target) -> String {
    use crate::types::Target;
    match t {
        Target::Grid { region, axis, coord, range } => {
            let axis_s = match axis {
                crate::types::Axis::X => "x",
                crate::types::Axis::Y => "y",
            };
            let scope = match range {
                None => "range=whole".to_string(),
                Some((s1, s2)) if stops_eq(s1, s2) => format!("at={}", format_stop(s1)),
                Some((s1, s2)) => format!("range={},{}", format_stop(s1), format_stop(s2)),
            };
            format!(
                "on=grid region=0x{:016x} axis={} coord={} {}",
                region.0, axis_s, coord, scope,
            )
        }
        Target::DriveLine { id, t } => {
            format!("on=drive-line id={} t={}", id, fmt_coord(*t))
        }
        Target::Perimeter { loop_, start, end, t } => {
            format!(
                "on=perimeter loop={} edge={}→{} t={}",
                format_loop(loop_),
                start.0,
                end.0,
                fmt_coord(*t),
            )
        }
    }
}

fn format_stop(s: &crate::types::GridStop) -> String {
    use crate::types::GridStop;
    match s {
        GridStop::Lattice { other } => format!("lattice:{}", other),
        GridStop::CrossesDriveLine { id } => format!("crosses-drive-line:{}", id),
        GridStop::CrossesPerimeter { loop_ } => {
            format!("crosses-perimeter:{}", format_loop(loop_))
        }
    }
}

fn format_loop(l: &crate::types::PerimeterLoop) -> String {
    use crate::types::PerimeterLoop;
    match l {
        PerimeterLoop::Outer => "outer".to_string(),
        PerimeterLoop::Hole { index } => format!("hole:{}", index),
    }
}

fn format_traffic(t: crate::types::AisleDirection) -> &'static str {
    use crate::types::AisleDirection;
    match t {
        AisleDirection::OneWay => "one-way",
        AisleDirection::OneWayReverse => "one-way-reverse",
        AisleDirection::TwoWayReverse => "two-way-reverse",
    }
}

fn stops_eq(a: &crate::types::GridStop, b: &crate::types::GridStop) -> bool {
    use crate::types::GridStop;
    match (a, b) {
        (GridStop::Lattice { other: x }, GridStop::Lattice { other: y }) => x == y,
        (GridStop::CrossesDriveLine { id: x }, GridStop::CrossesDriveLine { id: y }) => x == y,
        (GridStop::CrossesPerimeter { loop_: x }, GridStop::CrossesPerimeter { loop_: y }) => {
            x == y
        }
        _ => false,
    }
}
