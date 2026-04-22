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
    AisleDirection, DebugToggles, EdgeCurve, GenerateInput, ParkingLayout, ParkingParams, Polygon,
    Vec2,
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
        "annotations" => annotations_cmd(ctx),
        "stall-modifier-json" => stall_modifier_json(ctx, body),
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
    let (verts, curves) = parse_vertex_block(body)?;
    match kind {
        "outer" => {
            ctx.input.boundary.outer = verts;
            ctx.input.boundary.outer_curves = curves;
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
            ctx.input.boundary.hole_curves.push(curves);
            Ok(Outcome {
                output: format!("hole: {} vertices", n),
                ..Default::default()
            })
        }
        other => Err(format!("polygon <outer|hole> expected, got {:?}", other)),
    }
}

fn parse_vertex_block(body: &str) -> Result<(Vec<Vec2>, Vec<Option<EdgeCurve>>), String> {
    let mut verts = Vec::new();
    let mut curves: Vec<Option<EdgeCurve>> = Vec::new();
    for line in body.lines() {
        let t = line.trim();
        if t.is_empty() {
            continue;
        }
        if let Some(rest) = t.strip_prefix("curve ") {
            // Attach to the previous vertex's outgoing edge.
            if verts.is_empty() {
                return Err("curve line before any vertex".to_string());
            }
            let c = parse_curve(rest)?;
            let idx = verts.len() - 1;
            while curves.len() <= idx {
                curves.push(None);
            }
            curves[idx] = Some(c);
            continue;
        }
        verts.push(parse_xy(t)?);
    }
    // Pad curves to parallel edge count so downstream code doesn't have
    // to guard against length mismatch.
    while curves.len() < verts.len() {
        curves.push(None);
    }
    Ok((verts, curves))
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

fn parse_curve(rest: &str) -> Result<EdgeCurve, String> {
    let (cp1, cp2) = rest
        .split_once(' ')
        .ok_or_else(|| format!("curve expects 'cp1x,cp1y cp2x,cp2y', got {:?}", rest))?;
    Ok(EdgeCurve {
        cp1: parse_xy(cp1)?,
        cp2: parse_xy(cp2)?,
    })
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
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "annotations requires a prior `generate`".to_string())?;
    let total = ctx.input.annotations.len();
    let dormant = &layout.dormant_annotations;
    Ok(Outcome {
        output: format!("annotations: {}, dormant: {:?}", total, dormant),
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
