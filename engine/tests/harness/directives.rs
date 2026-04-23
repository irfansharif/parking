//! Directive dispatch — maps the `.txt` DSL onto `GenerateInput`
//! mutations and engine invocations.
//!
//! Phase 0 directives:
//!   polygon outer / polygon hole   — append boundary loops
//!   set <key>=<value>              — mutate ParkingParams
//!   generate                       — run pipeline, echo total_stalls
//!   snapshot [name]                — emit SVG, diff against golden

use parking_lot_engine::pipeline::generate::generate;
use parking_lot_engine::types::{
    AisleDirection, AisleEdge, Annotation, Axis, DebugToggles, DriveAisleGraph, DriveLine,
    EdgeArc, GenerateInput, GridStop, HolePin, ParkingLayout, ParkingParams, PerimeterLoop,
    Polygon, RegionId, Target, TrafficDirection, Vec2,
};
use std::path::{Path, PathBuf};

use crate::harness::svg;

pub struct FixtureCtx {
    pub input: GenerateInput,
    pub last_layout: Option<ParkingLayout>,
    /// Root directory where `snapshots/<name>.svg` goldens live.
    pub snapshot_root: PathBuf,
    /// Default snapshot name when a `snapshot` directive omits one;
    /// derived from the fixture file stem.
    pub default_snapshot_name: String,
    /// When set, `snapshot` writes the golden instead of comparing.
    pub update_snapshots: bool,
    /// Monotonic drive-line id allocator for `drive-line` directives
    /// that don't specify `id=N` explicitly. Starts at 1 so the
    /// default 0 stays reserved as the "no id" sentinel.
    next_drive_line_id: u32,
    /// Currently-selected aisle-graph edge (seed + chain + mode) —
    /// mirrors the UI's `app.state.selectedEdge`. Set by `edge-select`,
    /// consumed by `edge-delete` / `edge-cycle`.
    selected_edge: Option<SelectedEdge>,
}

/// Engine-harness analogue of the UI's `EdgeRef`. Kept in the
/// directive module since it's pure test state; doesn't need ts-rs
/// codegen.
struct SelectedEdge {
    seed_index: usize,
    chain: Vec<usize>,
    /// `true` for chain mode (single-click), `false` for segment mode
    /// (double-click narrow).
    is_chain_mode: bool,
}

impl FixtureCtx {
    pub fn new(snapshot_root: PathBuf, default_snapshot_name: String, update: bool) -> Self {
        Self {
            input: GenerateInput {
                boundary: Polygon::default(),
                aisle_graph: None,
                drive_lines: Vec::new(),
                annotations: Vec::new(),
                params: ParkingParams::default(),
                debug: DebugToggles::default(),
                region_overrides: Vec::new(),
                stall_modifiers: Vec::new(),
            },
            last_layout: None,
            snapshot_root,
            default_snapshot_name,
            update_snapshots: update,
            next_drive_line_id: 1,
            selected_edge: None,
        }
    }
}

/// One directive outcome. Text assertions compare `output` against the
/// fixture's expected block. Snapshot directives return empty output
/// and report success/failure through `snapshot_error`.
#[derive(Debug, Default)]
pub struct Outcome {
    pub output: String,
    /// Present when a snapshot mismatched the golden.
    pub snapshot_error: Option<String>,
}

pub fn execute(ctx: &mut FixtureCtx, command: &str, body: &str) -> Result<Outcome, String> {
    let mut parts = command.splitn(2, char::is_whitespace);
    let verb = parts.next().unwrap_or("");
    let rest = parts.next().unwrap_or("").trim();

    match verb {
        "polygon" => polygon(ctx, rest, body),
        "polygon-info" => polygon_info(ctx),
        "set" => set_param(ctx, rest),
        "generate" => generate_cmd(ctx),
        "graph" => graph_cmd(ctx, rest),
        "faces" => faces_cmd(ctx, rest),
        "annotation-json" => annotation_json(ctx, body),
        "annotation" => annotation_cmd(ctx, rest),
        "annotations" => annotations_cmd(ctx),
        "stall-modifier-json" => stall_modifier_json(ctx, body),
        "drive-line" => drive_line_cmd(ctx, rest, body),
        "vertex" => vertex_cmd(ctx, rest),
        "edge" => edge_cmd(ctx, rest),
        "edge-select" => edge_select_cmd(ctx, rest),
        "edge-delete" => edge_delete_cmd(ctx),
        "edge-cycle" => edge_cycle_cmd(ctx),
        "dormant" => dormant_cmd(ctx),
        "snapshot" => snapshot(ctx, rest),
        _ => Err(format!("unknown directive: {}", verb)),
    }
}

fn polygon_info(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    let b = &ctx.input.boundary;
    let mut lines = Vec::new();
    lines.push(loop_info("outer", &b.outer));
    for (i, h) in b.holes.iter().enumerate() {
        lines.push(loop_info(&format!("hole {}", i), h));
    }
    Ok(Outcome {
        output: lines.join("\n"),
        ..Default::default()
    })
}

fn loop_info(label: &str, pts: &[Vec2]) -> String {
    if pts.is_empty() {
        return format!("{}: empty", label);
    }
    let signed = Polygon::signed_area(pts);
    let winding = if signed > 0.0 {
        "ccw"
    } else if signed < 0.0 {
        "cw"
    } else {
        "degenerate"
    };
    let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
    let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts {
        min_x = min_x.min(p.x);
        min_y = min_y.min(p.y);
        max_x = max_x.max(p.x);
        max_y = max_y.max(p.y);
    }
    format!(
        "{}: {} vertices, {}, bbox ({},{})-({},{}), area {}",
        label,
        pts.len(),
        winding,
        num(min_x),
        num(min_y),
        num(max_x),
        num(max_y),
        num(signed.abs()),
    )
}

fn num(v: f64) -> String {
    if v == v.round() {
        format!("{}", v.round() as i64)
    } else {
        format!("{:.3}", v)
    }
}

fn polygon(ctx: &mut FixtureCtx, kind: &str, body: &str) -> Result<Outcome, String> {
    let (verts, arcs) = parse_vertex_block(body)?;
    match kind {
        "outer" => {
            ctx.input.boundary.outer = verts;
            ctx.input.boundary.outer_arcs = arcs;
            Ok(Outcome {
                output: format!(
                    "polygon outer: {} vertices",
                    ctx.input.boundary.outer.len()
                ),
                ..Default::default()
            })
        }
        "hole" => {
            let n = verts.len();
            ctx.input.boundary.holes.push(verts);
            ctx.input.boundary.hole_arcs.push(arcs);
            Ok(Outcome {
                output: format!("hole: {} vertices", n),
                ..Default::default()
            })
        }
        other => Err(format!("polygon <outer|hole> expected, got {:?}", other)),
    }
}

fn parse_vertex_block(body: &str) -> Result<(Vec<Vec2>, Vec<Option<EdgeArc>>), String> {
    let mut verts = Vec::new();
    let mut arcs: Vec<Option<EdgeArc>> = Vec::new();
    for line in body.lines() {
        let t = line.trim();
        if t.is_empty() {
            continue;
        }
        if let Some(rest) = t.strip_prefix("arc ") {
            // Attach to the previous vertex's outgoing edge.
            if verts.is_empty() {
                return Err("arc line before any vertex".to_string());
            }
            let a = parse_arc(rest)?;
            let idx = verts.len() - 1;
            while arcs.len() <= idx {
                arcs.push(None);
            }
            arcs[idx] = Some(a);
            continue;
        }
        verts.push(parse_xy(t)?);
    }
    // Pad arcs to parallel edge count so downstream code doesn't have
    // to guard against length mismatch.
    while arcs.len() < verts.len() {
        arcs.push(None);
    }
    Ok((verts, arcs))
}

fn parse_xy(s: &str) -> Result<Vec2, String> {
    let (x, y) = s
        .split_once(',')
        .ok_or_else(|| format!("expected x,y got {:?}", s))?;
    Ok(Vec2::new(
        x.trim()
            .parse::<f64>()
            .map_err(|e| format!("bad x {:?}: {}", x, e))?,
        y.trim()
            .parse::<f64>()
            .map_err(|e| format!("bad y {:?}: {}", y, e))?,
    ))
}

fn parse_arc(rest: &str) -> Result<EdgeArc, String> {
    let bulge = rest
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("arc expects a bulge number, got {:?}: {}", rest, e))?;
    Ok(EdgeArc { bulge })
}

fn set_param(ctx: &mut FixtureCtx, rest: &str) -> Result<Outcome, String> {
    let (raw_key, raw_val) = rest
        .split_once('=')
        .ok_or_else(|| "set requires key=value".to_string())?;
    let key = normalize_param_key(raw_key);
    apply_param(&mut ctx.input.params, &key, raw_val)?;
    Ok(Outcome {
        output: format!("set {}={}", key, raw_val),
        ..Default::default()
    })
}

/// Mirrors `ui/src/commands.ts::KEY_ALIASES` + dash→underscore so the
/// engine-local fixtures stay compatible with the UI's DSL.
fn normalize_param_key(key: &str) -> String {
    let k = key.trim().replace('-', "_");
    match k.as_str() {
        "stall_angle" | "angle" => "stall_angle_deg".to_string(),
        "width" => "stall_width".to_string(),
        "depth" => "stall_depth".to_string(),
        "cross_aisle_max_run" | "cross_aisle_spacing" => "stalls_per_face".to_string(),
        _ => k,
    }
}

fn apply_param(params: &mut ParkingParams, key: &str, raw_val: &str) -> Result<(), String> {
    let v = raw_val.trim();
    match key {
        "stall_width" => params.stall_width = parse_f64(v)?,
        "stall_depth" => params.stall_depth = parse_f64(v)?,
        "aisle_width" => params.aisle_width = parse_f64(v)?,
        "stall_angle_deg" => params.stall_angle_deg = parse_f64(v)?,
        "aisle_angle_deg" => params.aisle_angle_deg = parse_f64(v)?,
        "aisle_offset" => params.aisle_offset = parse_f64(v)?,
        "site_offset" => params.site_offset = parse_f64(v)?,
        "stalls_per_face" => params.stalls_per_face = parse_u32(v)?,
        "use_regions" => params.use_regions = parse_bool(v)?,
        "island_stall_interval" => params.island_stall_interval = parse_u32(v)?,
        other => return Err(format!("unknown param: {}", other)),
    }
    Ok(())
}

fn parse_f64(s: &str) -> Result<f64, String> {
    s.parse::<f64>().map_err(|e| format!("invalid number {:?}: {}", s, e))
}

fn parse_u32(s: &str) -> Result<u32, String> {
    s.parse::<u32>().map_err(|e| format!("invalid uint {:?}: {}", s, e))
}

fn parse_bool(s: &str) -> Result<bool, String> {
    match s {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(format!("expected true|false, got {:?}", s)),
    }
}

fn faces_cmd(ctx: &mut FixtureCtx, mode: &str) -> Result<Outcome, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "faces requires a prior `generate`".to_string())?;
    let faces = &layout.faces;
    let mode = if mode.is_empty() { "summary" } else { mode };
    let mut out = String::new();
    match mode {
        "summary" => {
            let boundary_count = faces.iter().filter(|f| f.is_boundary).count();
            out.push_str(&format!(
                "faces: {}, boundary: {}, interior: {}",
                faces.len(),
                boundary_count,
                faces.len() - boundary_count,
            ));
        }
        "sources" => {
            // Count edge sources across all faces, bucketed by label.
            let mut wall = 0usize;
            let mut interior = 0usize;
            let mut perimeter = 0usize;
            for f in faces {
                for s in &f.edge_sources {
                    match s.as_str() {
                        "wall" => wall += 1,
                        "interior" => interior += 1,
                        "perimeter" => perimeter += 1,
                        _ => {}
                    }
                }
            }
            out.push_str(&format!(
                "edge sources: wall={}, interior={}, perimeter={}",
                wall, interior, perimeter
            ));
        }
        other => {
            return Err(format!(
                "faces: unknown mode {:?} (summary|sources)",
                other
            ))
        }
    }
    Ok(Outcome {
        output: out,
        ..Default::default()
    })
}

fn annotation_json(ctx: &mut FixtureCtx, body: &str) -> Result<Outcome, String> {
    // Body is a JSON array of Annotation. Each entry matches the serde
    // shape of `parking_lot_engine::types::Annotation`, e.g.:
    //   [{"kind":"Direction",
    //     "target":{"on":"Perimeter","loop":{"kind":"Outer"},"arc":0.5},
    //     "traffic":"OneWay"}]
    let trimmed = body.trim();
    if trimmed.is_empty() {
        return Err("annotation-json body is empty".to_string());
    }
    let added: Vec<parking_lot_engine::types::Annotation> =
        serde_json::from_str(trimmed).map_err(|e| format!("annotation-json parse: {}", e))?;
    let count = added.len();
    ctx.input.annotations.extend(added);
    Ok(Outcome {
        output: format!("annotation-json: +{}", count),
        ..Default::default()
    })
}

fn stall_modifier_json(ctx: &mut FixtureCtx, body: &str) -> Result<Outcome, String> {
    // Body is a JSON array of StallModifier:
    //   [{"polyline":[{"x":50,"y":60},{"x":150,"y":60}],"kind":"Suppressed"}]
    // Single-point modifiers (suppress at a location) use a one-element
    // polyline.
    let trimmed = body.trim();
    if trimmed.is_empty() {
        return Err("stall-modifier-json body is empty".to_string());
    }
    let added: Vec<parking_lot_engine::types::StallModifier> =
        serde_json::from_str(trimmed).map_err(|e| format!("stall-modifier-json parse: {}", e))?;
    let count = added.len();
    ctx.input.stall_modifiers.extend(added);
    Ok(Outcome {
        output: format!("stall-modifier-json: +{}", count),
        ..Default::default()
    })
}

fn annotations_cmd(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    // Dump the active annotations in the stable one-line DSL — one
    // line per annotation, in insertion order. Matches the format
    // emitted by `debug::format_annotation_line` so fixtures that
    // assert on annotation shape round-trip cleanly.
    let lines: Vec<String> = ctx
        .input
        .annotations
        .iter()
        .map(parking_lot_engine::debug::format_annotation_line)
        .collect();
    Ok(Outcome {
        output: lines.join("\n"),
        ..Default::default()
    })
}

fn dormant_cmd(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "dormant requires a prior `generate`".to_string())?;
    let dormant = &layout.dormant_annotations;
    if dormant.is_empty() {
        return Ok(Outcome {
            output: "dormant_annotations: 0".to_string(),
            ..Default::default()
        });
    }
    let ids: Vec<String> = dormant.iter().map(|i| i.to_string()).collect();
    Ok(Outcome {
        output: format!("dormant_annotations: {} ({})", dormant.len(), ids.join(",")),
        ..Default::default()
    })
}

fn graph_cmd(ctx: &mut FixtureCtx, mode: &str) -> Result<Outcome, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "graph requires a prior `generate`".to_string())?;
    let g = &layout.resolved_graph;
    let mut out = String::new();
    let mode = if mode.is_empty() { "summary" } else { mode };
    match mode {
        "summary" => {
            out.push_str(&format!(
                "vertices: {}, edges: {}, perim_vertex_count: {}\n",
                g.vertices.len(),
                g.edges.len(),
                g.perim_vertex_count
            ));
            let (interior, perimeter) = g.edges.iter().partition::<Vec<_>, _>(|e| e.interior);
            out.push_str(&format!(
                "  perimeter: {}, interior: {}",
                perimeter.len(),
                interior.len()
            ));
        }
        "vertices" => {
            out.push_str(&format!("vertices: {}\n", g.vertices.len()));
            for (i, v) in g.vertices.iter().enumerate() {
                out.push_str(&format!("  {}: ({}, {})\n", i, num(v.x), num(v.y)));
            }
            out.pop();
        }
        "edges" => {
            out.push_str(&format!("edges: {}\n", g.edges.len()));
            for (i, e) in g.edges.iter().enumerate() {
                let kind = if e.interior { "interior" } else { "perimeter" };
                let dir = match e.direction {
                    AisleDirection::TwoWay => "two-way",
                    AisleDirection::TwoWayOriented => "two-way-oriented",
                    AisleDirection::OneWay => "one-way",
                };
                out.push_str(&format!(
                    "  {}: {}->{} {} {} w={}\n",
                    i, e.start, e.end, kind, dir, num(e.width)
                ));
            }
            out.pop();
        }
        other => {
            return Err(format!(
                "graph: unknown mode {:?} (summary|vertices|edges)",
                other
            ))
        }
    }
    Ok(Outcome {
        output: out,
        ..Default::default()
    })
}

fn generate_cmd(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    let layout = generate(ctx.input.clone());
    let total = layout.metrics.total_stalls;
    let mut lines = vec![format!("total_stalls: {}", total)];
    if !layout.dormant_annotations.is_empty() {
        lines.push(format!(
            "dormant_annotations: {:?}",
            layout.dormant_annotations
        ));
    }
    ctx.last_layout = Some(layout);
    Ok(Outcome {
        output: lines.join("\n"),
        ..Default::default()
    })
}

fn snapshot(ctx: &mut FixtureCtx, rest: &str) -> Result<Outcome, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "snapshot requires a prior `generate`".to_string())?;

    let name = if rest.is_empty() {
        ctx.default_snapshot_name.clone()
    } else {
        rest.trim().to_string()
    };
    let file_name = if name.ends_with(".svg") {
        name.clone()
    } else {
        format!("{}.svg", name)
    };
    let path: PathBuf = ctx.snapshot_root.join(&file_name);

    let svg_text = svg::render(
        &ctx.input.boundary,
        layout,
        &svg::SvgOptions::default(),
    );

    if ctx.update_snapshots {
        std::fs::create_dir_all(&ctx.snapshot_root).map_err(|e| e.to_string())?;
        std::fs::write(&path, &svg_text).map_err(|e| e.to_string())?;
        return Ok(Outcome::default());
    }

    match std::fs::read_to_string(&path) {
        Ok(golden) if golden == svg_text => Ok(Outcome::default()),
        Ok(golden) => Ok(Outcome {
            output: String::new(),
            snapshot_error: Some(snapshot_diff(
                &path_display(&path),
                &golden,
                &svg_text,
            )),
        }),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Outcome {
            output: String::new(),
            snapshot_error: Some(format!(
                "snapshot golden missing: {}\n(rerun with UPDATE_SNAPSHOTS=1 to create it)",
                path_display(&path)
            )),
        }),
        Err(e) => Err(format!("reading golden {}: {}", path_display(&path), e)),
    }
}

fn path_display(p: &Path) -> String {
    p.display().to_string()
}

fn snapshot_diff(label: &str, expected: &str, actual: &str) -> String {
    let mut out = format!("snapshot {} differs\n", label);
    out.push_str("--- expected\n");
    for l in expected.lines() {
        out.push_str("  ");
        out.push_str(l);
        out.push('\n');
    }
    out.push_str("+++ actual\n");
    for l in actual.lines() {
        out.push_str("  ");
        out.push_str(l);
        out.push('\n');
    }
    out.push_str("(rerun with UPDATE_SNAPSHOTS=1 to accept)\n");
    out
}

// ---------------------------------------------------------------------------
// drive-line / vertex / edge / annotation — mirror the UI's one-line DSL.
// ---------------------------------------------------------------------------

/// Accept optional `id=N`, `partitions`, and `hole-pin=h,v` tokens in
/// any order. Body is exactly two `x,y` lines (start and end).
fn drive_line_cmd(ctx: &mut FixtureCtx, args: &str, body: &str) -> Result<Outcome, String> {
    let (points, _) = parse_vertex_block(body)?;
    if points.len() != 2 {
        return Err(format!(
            "drive-line body requires exactly two x,y lines (got {})",
            points.len()
        ));
    }
    let start = points[0];
    let end = points[1];

    let mut explicit_id: Option<u32> = None;
    let mut partitions_flag = false;
    let mut hole_pin: Option<(usize, usize)> = None;
    for tok in args.split_whitespace() {
        if let Some(v) = tok.strip_prefix("id=") {
            explicit_id = Some(v.parse::<u32>().map_err(|e| format!("id: {}", e))?);
        } else if tok == "partitions" {
            partitions_flag = true;
        } else if let Some(v) = tok.strip_prefix("hole-pin=") {
            let (h, i) = v
                .split_once(',')
                .ok_or_else(|| "hole-pin requires h,i".to_string())?;
            hole_pin = Some((
                h.parse::<usize>().map_err(|e| format!("hole-pin h: {}", e))?,
                i.parse::<usize>().map_err(|e| format!("hole-pin i: {}", e))?,
            ));
        } else if !tok.is_empty() {
            return Err(format!("drive-line: unknown token {:?}", tok));
        }
    }

    let id = match explicit_id {
        Some(n) => {
            if n >= ctx.next_drive_line_id {
                ctx.next_drive_line_id = n + 1;
            }
            n
        }
        None => {
            let n = ctx.next_drive_line_id;
            ctx.next_drive_line_id += 1;
            n
        }
    };

    let partitions = partitions_flag || hole_pin.is_some();
    ctx.input.drive_lines.push(DriveLine {
        start,
        end,
        hole_pin: hole_pin.map(|(h, i)| HolePin {
            hole_index: h,
            vertex_index: i,
        }),
        id,
        partitions,
    });

    // Echo the request back in the same shape as debug::format_fixture
    // so round-tripping stays stable.
    let id_suffix = if explicit_id.is_some() {
        format!(" id={}", id)
    } else {
        String::new()
    };
    let pin_suffix = match (hole_pin, partitions_flag) {
        (Some((h, i)), _) => format!(" hole-pin={},{}", h, i),
        (None, true) => " partitions".to_string(),
        (None, false) => String::new(),
    };
    Ok(Outcome {
        output: format!(
            "drive-line: {},{} -> {},{}{}{}",
            fmt_xy(start.x),
            fmt_xy(start.y),
            fmt_xy(end.x),
            fmt_xy(end.y),
            id_suffix,
            pin_suffix,
        ),
        ..Default::default()
    })
}

fn fmt_xy(v: f64) -> String {
    if (v - v.round()).abs() < 1e-6 {
        format!("{}", v.round() as i64)
    } else {
        format!("{:.2}", v)
    }
}

fn vertex_cmd(ctx: &mut FixtureCtx, args: &str) -> Result<Outcome, String> {
    let mut parts = args.splitn(2, char::is_whitespace);
    let action = parts.next().unwrap_or("").trim();
    let tail = parts.next().unwrap_or("").trim();
    match action {
        "add" => {
            let p = parse_xy(tail)?;
            let graph = ctx.input.aisle_graph.get_or_insert_with(|| DriveAisleGraph {
                vertices: Vec::new(),
                edges: Vec::new(),
                perim_vertex_count: 0,
            });
            let idx = graph.vertices.len();
            graph.vertices.push(p);
            Ok(Outcome {
                output: format!("{}", idx),
                ..Default::default()
            })
        }
        other => Err(format!("vertex: unknown action {:?}", other)),
    }
}

fn edge_cmd(ctx: &mut FixtureCtx, args: &str) -> Result<Outcome, String> {
    let mut parts = args.splitn(2, char::is_whitespace);
    let action = parts.next().unwrap_or("").trim();
    let tail = parts.next().unwrap_or("").trim();
    match action {
        "add" => {
            let (s, e) = tail
                .split_once(',')
                .ok_or_else(|| "edge add requires i,j".to_string())?;
            let start = s
                .trim()
                .parse::<usize>()
                .map_err(|e| format!("edge start: {}", e))?;
            let end = e
                .trim()
                .parse::<usize>()
                .map_err(|e| format!("edge end: {}", e))?;
            let width = ctx.input.params.aisle_width / 2.0;
            let graph = ctx.input.aisle_graph.get_or_insert_with(|| DriveAisleGraph {
                vertices: Vec::new(),
                edges: Vec::new(),
                perim_vertex_count: 0,
            });
            graph.edges.push(AisleEdge {
                start,
                end,
                width,
                interior: false,
                direction: AisleDirection::TwoWay,
            });
            Ok(Outcome {
                output: format!("edge {}->{}", start, end),
                ..Default::default()
            })
        }
        other => Err(format!("edge: unknown action {:?}", other)),
    }
}

/// Parse the UI's one-line annotation DSL:
///   annotation <kind> on=<substrate> [axis=...] [coord=...]
///              [range=whole | at=<stop> | range=<s1>,<s2>] [traffic=...]
/// where <kind> ∈ {delete-vertex, delete-edge, direction}.
/// Echoes the parsed annotation back through the canonical
/// `debug::format_annotation_line` so fixtures round-trip verbatim.
fn annotation_cmd(ctx: &mut FixtureCtx, args: &str) -> Result<Outcome, String> {
    let mut parts = args.split_whitespace();
    let kind = parts
        .next()
        .ok_or_else(|| "annotation requires a kind".to_string())?;
    let mut kv: std::collections::HashMap<&str, &str> = std::collections::HashMap::new();
    for p in parts {
        if let Some(eq) = p.find('=') {
            kv.insert(&p[..eq], &p[eq + 1..]);
        }
    }
    let target = parse_annotation_target(&kv)?;
    let ann = match kind {
        "delete-vertex" => Annotation::DeleteVertex { target },
        "delete-edge" => Annotation::DeleteEdge { target },
        "direction" => {
            let traffic = kv
                .get("traffic")
                .ok_or_else(|| "direction requires traffic=<...>".to_string())?;
            Annotation::Direction {
                target,
                traffic: parse_traffic(traffic)?,
            }
        }
        other => return Err(format!("unknown annotation kind {:?}", other)),
    };
    let line = parking_lot_engine::debug::format_annotation_line(&ann);
    ctx.input.annotations.push(ann);
    Ok(Outcome {
        output: format!("annotation {}", line),
        ..Default::default()
    })
}

fn parse_annotation_target(
    kv: &std::collections::HashMap<&str, &str>,
) -> Result<Target, String> {
    let on = kv
        .get("on")
        .ok_or_else(|| "annotation: missing on=<substrate>".to_string())?;
    match *on {
        "grid" => {
            let region = kv
                .get("region")
                .ok_or_else(|| "grid: missing region=<id>".to_string())?;
            let region = parse_region_id(region)?;
            let axis = match kv.get("axis").copied() {
                Some("x") => Axis::X,
                Some("y") => Axis::Y,
                _ => return Err("grid: axis must be x or y".to_string()),
            };
            let coord = kv
                .get("coord")
                .ok_or_else(|| "grid: missing coord=<n>".to_string())?
                .parse::<i32>()
                .map_err(|e| format!("grid coord: {}", e))?;
            let range = if kv.get("range").copied() == Some("whole") {
                None
            } else if let Some(at) = kv.get("at") {
                let s = parse_stop(at)?;
                Some((s.clone(), s))
            } else if let Some(r) = kv.get("range") {
                let (a, b) = r
                    .split_once(',')
                    .ok_or_else(|| "grid range=<s1>,<s2> needs two stops".to_string())?;
                Some((parse_stop(a)?, parse_stop(b)?))
            } else {
                return Err("grid: need range=whole|<s1>,<s2> or at=<stop>".to_string());
            };
            Ok(Target::Grid {
                region,
                axis,
                coord,
                range,
            })
        }
        "drive-line" => {
            let id = kv
                .get("id")
                .ok_or_else(|| "drive-line: missing id".to_string())?
                .parse::<u32>()
                .map_err(|e| format!("drive-line id: {}", e))?;
            let t = kv
                .get("t")
                .ok_or_else(|| "drive-line: missing t".to_string())?
                .parse::<f64>()
                .map_err(|e| format!("drive-line t: {}", e))?;
            Ok(Target::DriveLine { id, t })
        }
        "perimeter" => {
            let loop_ = parse_loop(
                kv.get("loop")
                    .ok_or_else(|| "perimeter: missing loop".to_string())?,
            )?;
            let arc = kv
                .get("arc")
                .ok_or_else(|| "perimeter: missing arc".to_string())?
                .parse::<f64>()
                .map_err(|e| format!("perimeter arc: {}", e))?;
            Ok(Target::Perimeter { loop_, arc })
        }
        other => Err(format!("unknown substrate 'on={}'", other)),
    }
}

fn parse_region_id(s: &str) -> Result<RegionId, String> {
    let n = if let Some(hex) = s.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).map_err(|e| format!("region id hex: {}", e))?
    } else {
        s.parse::<u64>()
            .map_err(|e| format!("region id dec: {}", e))?
    };
    Ok(RegionId(n))
}

fn parse_stop(s: &str) -> Result<GridStop, String> {
    let (head, tail) = s
        .split_once(':')
        .ok_or_else(|| format!("stop {:?} missing ':'", s))?;
    match head {
        "lattice" => Ok(GridStop::Lattice {
            other: tail
                .parse::<i32>()
                .map_err(|e| format!("lattice: {}", e))?,
        }),
        "crosses-drive-line" => Ok(GridStop::CrossesDriveLine {
            id: tail
                .parse::<u32>()
                .map_err(|e| format!("crosses-drive-line: {}", e))?,
        }),
        "crosses-perimeter" => Ok(GridStop::CrossesPerimeter {
            loop_: parse_loop(tail)?,
        }),
        other => Err(format!("unknown stop head {:?}", other)),
    }
}

fn parse_loop(s: &str) -> Result<PerimeterLoop, String> {
    if s == "outer" {
        return Ok(PerimeterLoop::Outer);
    }
    if let Some(i) = s.strip_prefix("hole:") {
        return Ok(PerimeterLoop::Hole {
            index: i.parse::<usize>().map_err(|e| format!("hole: {}", e))?,
        });
    }
    Err(format!("unknown loop {:?}", s))
}

fn parse_traffic(s: &str) -> Result<TrafficDirection, String> {
    match s {
        "one-way" => Ok(TrafficDirection::OneWay),
        "one-way-reverse" => Ok(TrafficDirection::OneWayReverse),
        "two-way-oriented" => Ok(TrafficDirection::TwoWayOriented),
        "two-way-oriented-reverse" => Ok(TrafficDirection::TwoWayOrientedReverse),
        other => Err(format!("unknown traffic {:?}", other)),
    }
}

// ---------------------------------------------------------------------------
// edge-select / edge-delete / edge-cycle — harness mirrors of the UI's
// click-to-annotation flow. `edge-select` records a seed edge + its
// collinear chain in `ctx.selected_edge`; the commit directives resolve
// that selection to a Grid or DriveLine target and push the
// corresponding annotation.
// ---------------------------------------------------------------------------

fn edge_select_cmd(ctx: &mut FixtureCtx, args: &str) -> Result<Outcome, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "edge-select requires a prior `generate`".to_string())?;
    let graph = &layout.resolved_graph;

    let mut seed: Option<usize> = None;
    let mut near: Option<Vec2> = None;
    let mut mode = "chain";
    for tok in args.split_whitespace() {
        if let Some(v) = tok.strip_prefix("seed=") {
            seed = Some(v.parse::<usize>().map_err(|e| format!("seed: {}", e))?);
        } else if let Some(v) = tok.strip_prefix("near=") {
            near = Some(parse_xy(v)?);
        } else if let Some(v) = tok.strip_prefix("mode=") {
            if v != "chain" && v != "segment" {
                return Err(format!("edge-select: mode must be chain|segment, got {:?}", v));
            }
            mode = if v == "chain" { "chain" } else { "segment" };
        }
    }
    let seed_index = match (seed, near) {
        (Some(s), _) => s,
        (None, Some(p)) => {
            let seen = &mut std::collections::HashSet::<(usize, usize)>::new();
            let mut best: Option<(usize, f64)> = None;
            for (i, edge) in graph.edges.iter().enumerate() {
                let key = (edge.start.min(edge.end), edge.start.max(edge.end));
                if !seen.insert(key) {
                    continue;
                }
                let a = graph.vertices[edge.start];
                let b = graph.vertices[edge.end];
                let d = parking_lot_engine::geom::poly::point_to_segment_dist(p, a, b);
                if best.map(|(_, bd)| d < bd).unwrap_or(true) {
                    best = Some((i, d));
                }
            }
            best.map(|(i, _)| i)
                .ok_or_else(|| "edge-select near=X,Y found no edges".to_string())?
        }
        _ => return Err("edge-select requires seed=<i> or near=<x>,<y>".to_string()),
    };

    let chain = if mode == "chain" {
        parking_lot_engine::resolve::find_collinear_chain(graph, seed_index)
    } else {
        vec![seed_index]
    };
    let chain_len = chain.len();
    ctx.selected_edge = Some(SelectedEdge {
        seed_index,
        chain,
        is_chain_mode: mode == "chain",
    });
    Ok(Outcome {
        output: format!(
            "edge-select seed={} mode={} chain_len={}",
            seed_index, mode, chain_len
        ),
        ..Default::default()
    })
}

fn edge_delete_cmd(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    let sel = ctx
        .selected_edge
        .take()
        .ok_or_else(|| "edge-delete requires a prior `edge-select`".to_string())?;
    let target = resolve_selection_target(ctx, &sel)?;
    ctx.input.annotations.push(parking_lot_engine::types::Annotation::DeleteEdge { target });
    ctx.input.aisle_graph = None;
    Ok(Outcome {
        output: "edge-delete".to_string(),
        ..Default::default()
    })
}

fn edge_cycle_cmd(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    use parking_lot_engine::types::{Annotation, TrafficDirection};
    let sel = ctx
        .selected_edge
        .take()
        .ok_or_else(|| "edge-cycle requires a prior `edge-select`".to_string())?;
    let target = resolve_selection_target(ctx, &sel)?;
    let existing = ctx.input.annotations.iter().position(|ann| {
        matches!(ann, Annotation::Direction { .. })
            && parking_lot_engine::resolve::targets_equal(annotation_target(ann), &target)
    });
    if let Some(idx) = existing {
        // Non-tombstone cycle: TwoWayOriented → TwoWayOrientedReverse
        // → OneWay → OneWayReverse → TwoWayOriented.
        if let Annotation::Direction { traffic, .. } = &mut ctx.input.annotations[idx] {
            *traffic = match traffic {
                TrafficDirection::TwoWayOriented => TrafficDirection::TwoWayOrientedReverse,
                TrafficDirection::TwoWayOrientedReverse => TrafficDirection::OneWay,
                TrafficDirection::OneWay => TrafficDirection::OneWayReverse,
                TrafficDirection::OneWayReverse => TrafficDirection::TwoWayOriented,
            };
        }
    } else {
        ctx.input.annotations.push(Annotation::Direction {
            target,
            traffic: TrafficDirection::TwoWayOriented,
        });
    }
    ctx.input.aisle_graph = None;
    Ok(Outcome {
        output: "edge-cycle".to_string(),
        ..Default::default()
    })
}

fn annotation_target(ann: &parking_lot_engine::types::Annotation) -> &parking_lot_engine::types::Target {
    use parking_lot_engine::types::Annotation;
    match ann {
        Annotation::DeleteVertex { target } => target,
        Annotation::DeleteEdge { target } => target,
        Annotation::Direction { target, .. } => target,
    }
}

/// Resolve a `SelectedEdge` to a `Target` — grid-first, drive-line
/// fallback. Mirrors `app.deleteSelectedEdge` / `cycleEdgeDirection` on
/// the UI side.
fn resolve_selection_target(
    ctx: &FixtureCtx,
    sel: &SelectedEdge,
) -> Result<parking_lot_engine::types::Target, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "selection requires a prior `generate`".to_string())?;
    let graph = &layout.resolved_graph;
    if let Some(rd) = layout.region_debug.as_ref() {
        if let Some(target) = parking_lot_engine::resolve::resolve_grid_target(
            sel.seed_index,
            &sel.chain,
            sel.is_chain_mode,
            graph,
            &ctx.input.params,
            rd,
        ) {
            return Ok(target);
        }
    }
    // Splice fallback: chain must live on a single drive line.
    let seed = graph
        .edges
        .get(sel.seed_index)
        .ok_or_else(|| format!("seed index {} out of bounds", sel.seed_index))?;
    let s = graph.vertices[seed.start];
    let e = graph.vertices[seed.end];
    let sa = parking_lot_engine::resolve::world_to_splice_vertex(s, &ctx.input.drive_lines, 0.5);
    let sb = parking_lot_engine::resolve::world_to_splice_vertex(e, &ctx.input.drive_lines, 0.5);
    match (sa, sb) {
        (Some(a), Some(b)) if a.drive_line_id == b.drive_line_id => {
            let (mut ta, mut tb) = (a.t, b.t);
            if sel.is_chain_mode && sel.chain.len() > 1 {
                let mut ts = Vec::new();
                for ei in &sel.chain {
                    if let Some(edge) = graph.edges.get(*ei) {
                        if let Some(p) = parking_lot_engine::resolve::world_to_splice_vertex(
                            graph.vertices[edge.start],
                            &ctx.input.drive_lines,
                            0.5,
                        ) {
                            if p.drive_line_id == a.drive_line_id {
                                ts.push(p.t);
                            }
                        }
                        if let Some(p) = parking_lot_engine::resolve::world_to_splice_vertex(
                            graph.vertices[edge.end],
                            &ctx.input.drive_lines,
                            0.5,
                        ) {
                            if p.drive_line_id == a.drive_line_id {
                                ts.push(p.t);
                            }
                        }
                    }
                }
                if ts.len() >= 2 {
                    ta = ts.iter().cloned().fold(f64::INFINITY, f64::min);
                    tb = ts.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                }
            }
            Ok(parking_lot_engine::types::Target::DriveLine {
                id: a.drive_line_id,
                t: (ta + tb) * 0.5,
            })
        }
        _ => Err("selection: no stable anchor (no grid or drive-line target)".to_string()),
    }
}
