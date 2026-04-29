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
    AisleDirection, Annotation, Axis, DebugToggles, DriveLine,
    EdgeArc, GenerateInput, GridStop, ParkingLayout, ParkingParams, PerimeterLoop,
    Polygon, RegionId, RegionOverride, StallKind, StallModifier, Target, Vec2, VertexId,
};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Args parsed off the directive line by `datadriven`. Keys are arg
/// names; an empty `Vec` means the arg appeared as a bare flag
/// (e.g. `polygon outer`), a single-element `Vec` is a `k=v` pair,
/// multiple elements is `k=(v1,v2)`.
pub type DirectiveArgs = HashMap<String, Vec<String>>;

/// Return the (sorted-first) bare flag — an arg with no value. Used for
/// directives that carry a single positional kind on the directive line
/// (`polygon outer`, `graph summary`, `snapshot foo`).
fn pop_positional(args: &DirectiveArgs) -> Option<&str> {
    let mut keys: Vec<&String> = args
        .iter()
        .filter(|(_, v)| v.is_empty())
        .map(|(k, _)| k)
        .collect();
    keys.sort();
    keys.first().map(|s| s.as_str())
}

/// Return the single value bound to `key`, or `None` if missing or
/// multi-valued.
fn single<'a>(args: &'a DirectiveArgs, key: &str) -> Option<&'a str> {
    let v = args.get(key)?;
    if v.len() == 1 {
        Some(v[0].as_str())
    } else {
        None
    }
}

/// Read a `region=` arg, accepting both single-value forms (`0x…`,
/// decimal) and the paren-wrapped two-value `(x,y)` anchor — the latter
/// is parsed by datadriven into a 2-element values list, which we glue
/// back into one string so `resolve_region` can treat it uniformly.
fn region_arg(args: &DirectiveArgs) -> Option<String> {
    let v = args.get("region")?;
    match v.as_slice() {
        [s] => Some(s.clone()),
        [x, y] => Some(format!("({},{})", x, y)),
        _ => None,
    }
}

use crate::harness::svg;

pub struct FixtureCtx {
    pub input: GenerateInput,
    pub last_layout: Option<ParkingLayout>,
    /// Root directory where `<name>.svg` goldens live — currently
    /// `engine/tests/testdata/`, the same dir as the `.txt` fixtures.
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
}

/// Engine-harness analogue of the UI's `EdgeRef`. Lives transiently
/// inside `edge-delete` / `edge-cycle` — those directives select and
/// commit in one shot, so no cross-directive state is needed.
struct EdgeSelection {
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

pub fn execute(
    ctx: &mut FixtureCtx,
    directive: &str,
    args: &DirectiveArgs,
    body: &str,
) -> Result<Outcome, String> {
    match directive {
        "polygon" => polygon(ctx, args, body),
        "set" => set_param(ctx, args),
        "generate" => generate_cmd(ctx),
        // `annotation` and `edge-{delete,cycle}` carry args (`,`, `:`,
        // `→`) that datadriven's directive parser rejects, so fixtures
        // place those args on a body line; the dispatchers parse the
        // body directly.
        "annotation" => annotation_cmd(ctx, args, body),
        "annotations" => annotations_cmd(ctx),
        "region-override" => region_override_cmd(ctx, args),
        "drive-line" => drive_line_cmd(ctx, args, body),
        _ => Err(format!("unknown directive: {}", directive)),
    }
}

fn loop_info(label: &str, pts: &[Vec2]) -> String {
    if pts.is_empty() {
        return format!("{} empty", label);
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
        "{} {} vertices, {}, bbox ({},{})-({},{}), area {}",
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

fn polygon(ctx: &mut FixtureCtx, args: &DirectiveArgs, body: &str) -> Result<Outcome, String> {
    let kind = pop_positional(args)
        .ok_or_else(|| "polygon needs <outer|hole>".to_string())?;
    let (verts, arcs) = parse_vertex_block(body)?;
    let output = match kind {
        "outer" => {
            ctx.input.boundary.outer = verts;
            ctx.input.boundary.outer_arcs = arcs;
            loop_info("polygon outer", &ctx.input.boundary.outer)
        }
        "hole" => {
            ctx.input.boundary.holes.push(verts);
            ctx.input.boundary.hole_arcs.push(arcs);
            loop_info(
                "polygon hole",
                ctx.input.boundary.holes.last().unwrap(),
            )
        }
        other => return Err(format!("polygon <outer|hole> expected, got {:?}", other)),
    };
    Ok(Outcome { output, ..Default::default() })
}

/// Body-line vertex format: `(x,y)` with an optional outgoing-edge
/// arc as ` arc=<bulge>` on the same line. Arcs apply to the segment
/// from this vertex to the next.
fn parse_vertex_block(body: &str) -> Result<(Vec<Vec2>, Vec<Option<EdgeArc>>), String> {
    let mut verts = Vec::new();
    let mut arcs: Vec<Option<EdgeArc>> = Vec::new();
    for line in body.lines() {
        let t = line.trim();
        if t.is_empty() {
            continue;
        }
        let (pt, arc) = parse_vertex_line(t)?;
        verts.push(pt);
        arcs.push(arc);
    }
    Ok((verts, arcs))
}

fn parse_vertex_line(s: &str) -> Result<(Vec2, Option<EdgeArc>), String> {
    let inner = s
        .strip_prefix('(')
        .ok_or_else(|| format!("expected `(x,y)`, got {:?}", s))?;
    let (xy, rest) = inner
        .split_once(')')
        .ok_or_else(|| format!("missing `)` in {:?}", s))?;
    let pt = parse_xy(xy)?;
    let rest = rest.trim();
    if rest.is_empty() {
        return Ok((pt, None));
    }
    let bulge_str = rest
        .strip_prefix("arc=")
        .ok_or_else(|| format!("expected ` arc=<bulge>` after `(x,y)`, got {:?}", rest))?;
    let bulge = bulge_str
        .parse::<f64>()
        .map_err(|e| format!("bad arc bulge {:?}: {}", bulge_str, e))?;
    Ok((pt, Some(EdgeArc { bulge })))
}

/// Strict `(x,y)` form for anchor-style coords (`near=`, `region=`).
/// Bare `x,y` is rejected — paren-wrapping is the canonical shape and
/// mirrors the polygon/drive-line body syntax. (Bare `x,y` survives
/// only as the polygon body's per-vertex format; that's parsed by
/// `parse_xy` directly.)
fn parse_anchor(s: &str) -> Result<Vec2, String> {
    let inner = s
        .strip_prefix('(')
        .and_then(|t| t.strip_suffix(')'))
        .ok_or_else(|| format!("anchor must be `(x,y)`, got {:?}", s))?;
    parse_xy(inner)
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

fn set_param(ctx: &mut FixtureCtx, args: &DirectiveArgs) -> Result<Outcome, String> {
    let (raw_key, raw_val) = sole_kv(args, "set")?;
    let key = normalize_param_key(raw_key);
    apply_param(&mut ctx.input.params, &key, raw_val)?;
    // No echo — the directive line already shows what was set; an
    // expected block that just mirrors it adds noise.
    Ok(Outcome::default())
}

/// Extract the (only) `key=value` pair from `args`. Used by directives
/// that take exactly one such pair (currently just `set`).
fn sole_kv<'a>(args: &'a DirectiveArgs, verb: &str) -> Result<(&'a str, &'a str), String> {
    if args.len() != 1 {
        return Err(format!("{} requires exactly one key=value", verb));
    }
    let (k, v) = args.iter().next().unwrap();
    if v.len() != 1 {
        return Err(format!("{} requires key=value", verb));
    }
    Ok((k.as_str(), v[0].as_str()))
}

/// Mirrors `ui/src/commands.ts::KEY_ALIASES` + dash→underscore so the
/// engine-local fixtures stay compatible with the UI's DSL.
fn normalize_param_key(key: &str) -> String {
    let k = key.trim().replace('-', "_");
    match k.as_str() {
        "angle" => "stall_angle".to_string(),
        "width" => "stall_width".to_string(),
        "depth" => "stall_depth".to_string(),
        _ => k,
    }
}

fn apply_param(params: &mut ParkingParams, key: &str, raw_val: &str) -> Result<(), String> {
    let v = raw_val.trim();
    match key {
        "stall_width" => params.stall_width = parse_f64(v)?,
        "stall_depth" => params.stall_depth = parse_f64(v)?,
        "aisle_width" => params.aisle_width = parse_f64(v)?,
        "stall_angle" => params.stall_angle = parse_f64(v)?,
        "aisle_angle" => params.aisle_angle = parse_f64(v)?,
        "aisle_offset" => params.aisle_offset = parse_f64(v)?,
        "site_offset" => params.site_offset = parse_f64(v)?,
        "stalls_per_face" => params.stalls_per_face = parse_u32(v)?,
        "use_regions" => params.use_regions = parse_bool(v)?,
        "island_stall_interval" => params.island_stall_interval = parse_u32(v)?,
        "min_stalls_per_spine" => params.min_stalls_per_spine = parse_u32(v)?,
        "ada_stall_width" => params.ada_stall_width = parse_f64(v)?,
        "compact_stall_width" => params.compact_stall_width = parse_f64(v)?,
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

fn generate_cmd(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    let layout = generate(ctx.input.clone());
    let g = &layout.resolved_graph;
    let boundary_faces = layout.faces.iter().filter(|f| f.is_boundary).count();
    let interior_faces = layout.faces.len() - boundary_faces;
    let interior_edges = g.edges.iter().filter(|e| e.interior).count();
    let perimeter_edges = g.edges.len() - interior_edges;
    let regions = layout
        .region_debug
        .as_ref()
        .map(|rd| rd.regions.len())
        .unwrap_or(0);
    let output = format!(
        "total_stalls: {}\n\
         faces: {} (boundary: {}, interior: {})\n\
         graph: {} vertices, {} edges (perimeter: {}, interior: {})\n\
         regions: {}",
        layout.metrics.total_stalls,
        layout.faces.len(),
        boundary_faces,
        interior_faces,
        g.vertices.len(),
        g.edges.len(),
        perimeter_edges,
        interior_edges,
        regions,
    );
    ctx.last_layout = Some(layout);
    Ok(Outcome { output, ..Default::default() })
}

/// Render an SVG snapshot for the fixture's final layout. Called by
/// the runner once per fixture after all directives have executed; no
/// longer surfaced as a directive in fixture files.
pub fn snapshot(ctx: &mut FixtureCtx) -> Result<Outcome, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| "snapshot requires a prior `generate`".to_string())?;

    let path: PathBuf = ctx
        .snapshot_root
        .join(format!("{}.svg", ctx.default_snapshot_name));

    let opts = svg::SvgOptions::default();
    let svg_text = svg::render(
        &ctx.input.boundary,
        layout,
        &ctx.input.annotations,
        &ctx.input.drive_lines,
        &opts,
        ctx.input.params.arc_discretize_tolerance,
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

/// `drive-line [partitions] [stall=<kind>]` with a 2-point `(x,y)`
/// body. Without `stall=`, the line is a structural drive-line; with
/// it, the body is interpreted as a stall-modifier polyline of the
/// given `<kind>` (suppressed | standard | ada | compact | island).
/// Annotations target a specific drive-line by `near=(x,y)`, so the
/// directive auto-allocates IDs internally — no `id=` user arg.
fn drive_line_cmd(
    ctx: &mut FixtureCtx,
    args: &DirectiveArgs,
    body: &str,
) -> Result<Outcome, String> {
    let (points, _) = parse_vertex_block(body)?;
    if points.len() != 2 {
        return Err(format!(
            "drive-line body requires exactly two x,y lines (got {})",
            points.len()
        ));
    }
    let start = points[0];
    let end = points[1];

    let mut partitions = false;
    let mut stall: Option<&str> = None;
    for (k, vals) in args {
        match (k.as_str(), vals.as_slice()) {
            ("partitions", []) => partitions = true,
            ("stall", [v]) => stall = Some(v.as_str()),
            _ => return Err(format!("drive-line: unknown arg {:?}", k)),
        }
    }

    if let Some(kind_str) = stall {
        let kind = parse_stall_kind(kind_str)?;
        ctx.input.stall_modifiers.push(StallModifier {
            polyline: vec![start, end],
            kind,
        });
        return Ok(Outcome {
            output: format!(
                "drive-line ({},{}) -> ({},{}) stall={}",
                fmt_xy(start.x),
                fmt_xy(start.y),
                fmt_xy(end.x),
                fmt_xy(end.y),
                kind_str,
            ),
            ..Default::default()
        });
    }

    let id = ctx.next_drive_line_id;
    ctx.next_drive_line_id += 1;
    ctx.input.drive_lines.push(DriveLine {
        start,
        end,
        hole_pin: None,
        id,
        partitions,
    });

    let partitions_suffix = if partitions { " partitions" } else { "" };
    Ok(Outcome {
        output: format!(
            "drive-line ({},{}) -> ({},{}){}",
            fmt_xy(start.x),
            fmt_xy(start.y),
            fmt_xy(end.x),
            fmt_xy(end.y),
            partitions_suffix,
        ),
        ..Default::default()
    })
}

fn parse_stall_kind(s: &str) -> Result<StallKind, String> {
    match s {
        "standard" => Ok(StallKind::Standard),
        "ada" => Ok(StallKind::Ada),
        "compact" => Ok(StallKind::Compact),
        "island" => Ok(StallKind::Island),
        "suppressed" => Ok(StallKind::Suppressed),
        other => Err(format!("unknown stall kind {:?}", other)),
    }
}

fn fmt_xy(v: f64) -> String {
    if (v - v.round()).abs() < 1e-6 {
        format!("{}", v.round() as i64)
    } else {
        format!("{:.2}", v)
    }
}

/// Parse the UI's one-line annotation DSL. Fixtures always put
/// `<kind>` on the directive line and the args on a single body line —
/// uniform shape, since the args may carry `:` / `,` / `→` that
/// datadriven's directive parser rejects.
///
///   annotation <kind>
///   on=<substrate> [axis=...] [coord=...]
///   [range=whole | at=<stop> | range=<s1>,<s2>] [traffic=...]
///   ----
///
/// where <kind> ∈ {delete-vertex, delete-edge, direction}. Echoes the
/// parsed annotation back through `debug::format_annotation_line` so
/// fixtures round-trip verbatim.
fn annotation_cmd(
    ctx: &mut FixtureCtx,
    args: &DirectiveArgs,
    body: &str,
) -> Result<Outcome, String> {
    let kind = pop_positional(args)
        .ok_or_else(|| "annotation requires a kind".to_string())?;
    let mut kv: HashMap<&str, &str> = HashMap::new();
    for tok in body.split_whitespace() {
        if let Some(eq) = tok.find('=') {
            kv.insert(&tok[..eq], &tok[eq + 1..]);
        }
    }
    // Two ways to specify a target:
    //   * `near=(x,y) mode=…` (no `on=`) — geometric selection of the
    //     nearest aisle edge, optionally expanded to its collinear
    //     chain. Formerly the `edge-delete` / `edge-cycle` directives.
    //   * `on=<substrate> …` — substrate-specific addressing
    //     (`on=grid region=… axis=…`, `on=drive-line near=(x,y) t=…`,
    //     `on=perimeter loop=… edge=…`).
    let target = if kv.contains_key("near") && !kv.contains_key("on") {
        let sel = select_edge_from_kv(&kv, ctx, "annotation")?;
        resolve_selection_target(ctx, &sel)?
    } else {
        parse_annotation_target(&kv, ctx)?
    };
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

fn region_override_cmd(ctx: &mut FixtureCtx, args: &DirectiveArgs) -> Result<Outcome, String> {
    let region_str = region_arg(args)
        .ok_or_else(|| "region-override: missing region=<id|(x,y)>".to_string())?;
    let region_id = resolve_region(&region_str, ctx)?;
    let aisle_angle = single(args, "angle")
        .map(|s| s.parse::<f64>().map_err(|e| format!("region-override angle: {}", e)))
        .transpose()?;
    let aisle_offset = single(args, "offset")
        .map(|s| s.parse::<f64>().map_err(|e| format!("region-override offset: {}", e)))
        .transpose()?;
    if aisle_angle.is_none() && aisle_offset.is_none() {
        return Err("region-override: needs at least one of angle=<n> or offset=<n>".to_string());
    }
    ctx.input.region_overrides.push(RegionOverride {
        region_id,
        aisle_angle,
        aisle_offset,
    });
    let mut out = format!("region-override region=0x{:016x}", region_id.0);
    if let Some(a) = aisle_angle {
        out.push_str(&format!(" angle={}", a));
    }
    if let Some(o) = aisle_offset {
        out.push_str(&format!(" offset={}", o));
    }
    Ok(Outcome { output: out, ..Default::default() })
}

fn parse_annotation_target(
    kv: &std::collections::HashMap<&str, &str>,
    ctx: &FixtureCtx,
) -> Result<Target, String> {
    let on = kv
        .get("on")
        .ok_or_else(|| "annotation: missing on=<substrate>".to_string())?;
    match *on {
        "grid" => {
            let region = kv
                .get("region")
                .ok_or_else(|| "grid: missing region=<id>".to_string())?;
            let region = resolve_region(region, ctx)?;
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
            let id = if let Some(near) = kv.get("near") {
                resolve_drive_line_near(near, ctx)?
            } else if let Some(s) = kv.get("id") {
                s.parse::<u32>()
                    .map_err(|e| format!("drive-line id: {}", e))?
            } else {
                return Err("drive-line: missing near=(x,y) or id=N".to_string());
            };
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
            // Accept either `start=A end=B` or `edge=A→B` (the engine's
            // debug formatter emits the latter, so fixtures often
            // round-trip that form).
            let (start_str, end_str) = if let Some(edge) = kv.get("edge") {
                edge.split_once('→')
                    .ok_or_else(|| "perimeter edge=<A>→<B> needs an arrow".to_string())?
            } else {
                let s = kv.get("start").copied()
                    .ok_or_else(|| "perimeter: missing start".to_string())?;
                let e = kv.get("end").copied()
                    .ok_or_else(|| "perimeter: missing end".to_string())?;
                (s, e)
            };
            let start = VertexId(
                start_str.parse::<u32>()
                    .map_err(|e| format!("perimeter start: {}", e))?,
            );
            let end = VertexId(
                end_str.parse::<u32>()
                    .map_err(|e| format!("perimeter end: {}", e))?,
            );
            let t = kv
                .get("t")
                .ok_or_else(|| "perimeter: missing t".to_string())?
                .parse::<f64>()
                .map_err(|e| format!("perimeter t: {}", e))?;
            Ok(Target::Perimeter { loop_, start, end, t })
        }
        other => Err(format!("unknown substrate 'on={}'", other)),
    }
}

/// Resolve a `region=` argument. Accepts:
///   - `(x,y)` — anchor point inside the target region. Looked up
///     against `ctx.last_layout.region_debug.regions[*].clip_poly` via
///     point-in-polygon. Stable across small layout tweaks and
///     spellable from the snapshot.
///   - `0x…` / decimal — raw `RegionId`. Engine's canonical form;
///     copy-pasted from a previous run's echo. Brittle, but kept as a
///     low-friction escape hatch.
/// Resolve `near=(x,y)` (or bare `near=x,y`) to the id of the drive-
/// line whose segment passes nearest the anchor point. Same pattern
/// as `resolve_region`, keyed off `ctx.input.drive_lines`.
fn resolve_drive_line_near(s: &str, ctx: &FixtureCtx) -> Result<u32, String> {
    let p = parse_anchor(s)?;
    let mut best: Option<(u32, f64)> = None;
    for dl in &ctx.input.drive_lines {
        let d = parking_lot_engine::geom::poly::point_to_segment_dist(p, dl.start, dl.end);
        if best.map(|(_, bd)| d < bd).unwrap_or(true) {
            best = Some((dl.id, d));
        }
    }
    best.map(|(id, _)| id)
        .ok_or_else(|| format!("drive-line near=({}, {}) found no drive-line", p.x, p.y))
}

fn resolve_region(s: &str, ctx: &FixtureCtx) -> Result<RegionId, String> {
    if let Some(inner) = s.strip_prefix('(').and_then(|t| t.strip_suffix(')')) {
        let p = parse_xy(inner)?;
        let layout = ctx
            .last_layout
            .as_ref()
            .ok_or_else(|| "region=(x,y) requires a prior `generate`".to_string())?;
        let rd = layout
            .region_debug
            .as_ref()
            .ok_or_else(|| "region=(x,y): layout has no region_debug".to_string())?;
        for region in &rd.regions {
            if parking_lot_engine::geom::clip::point_in_polygon(&p, &region.clip_poly) {
                return Ok(region.id);
            }
        }
        let centroids: Vec<String> = rd
            .regions
            .iter()
            .map(|r| {
                let n = r.clip_poly.len() as f64;
                let cx = r.clip_poly.iter().map(|q| q.x).sum::<f64>() / n;
                let cy = r.clip_poly.iter().map(|q| q.y).sum::<f64>() / n;
                format!("  0x{:016x} centroid=({:.0},{:.0})", r.id.0, cx, cy)
            })
            .collect();
        return Err(format!(
            "region=(x,y): no region contains ({}, {})\navailable regions:\n{}",
            p.x,
            p.y,
            centroids.join("\n"),
        ));
    }
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

fn parse_traffic(s: &str) -> Result<AisleDirection, String> {
    match s {
        "one-way" => Ok(AisleDirection::OneWay),
        "one-way-reverse" => Ok(AisleDirection::OneWayReverse),
        "two-way-reverse" => Ok(AisleDirection::TwoWayReverse),
        other => Err(format!(
            "unknown traffic {:?} (annotation can only carry one-way, \
             one-way-reverse, two-way-reverse)",
            other
        )),
    }
}

// ---------------------------------------------------------------------------
// Geometric edge selection — used by `annotation <kind> near=(x,y) mode=…`
// to pick an aisle-graph edge by world coords (and optionally expand
// to its collinear run) instead of via the substrate-specific
// `on=grid region=… axis=… coord=… range=…` form.
// ---------------------------------------------------------------------------

/// Pick the graph edge nearest `near=(x,y)` and (in chain mode) expand
/// it to its collinear run.
fn select_edge_from_kv(
    kv: &HashMap<&str, &str>,
    ctx: &FixtureCtx,
    verb: &str,
) -> Result<EdgeSelection, String> {
    let layout = ctx
        .last_layout
        .as_ref()
        .ok_or_else(|| format!("{} requires a prior `generate`", verb))?;
    let graph = &layout.resolved_graph;

    let p = kv
        .get("near")
        .map(|v| parse_anchor(v))
        .transpose()?
        .ok_or_else(|| format!("{} requires near=(x,y)", verb))?;
    let is_chain_mode = match kv.get("mode").copied().unwrap_or("chain") {
        "chain" => true,
        "segment" => false,
        other => {
            return Err(format!(
                "{}: mode must be chain|segment, got {:?}",
                verb, other
            ));
        }
    };
    let seed_index = {
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
            .ok_or_else(|| format!("{} near=X,Y found no edges", verb))?
    };
    let chain = if is_chain_mode {
        parking_lot_engine::resolve::find_collinear_chain(graph, seed_index)
    } else {
        vec![seed_index]
    };
    Ok(EdgeSelection { seed_index, chain, is_chain_mode })
}

/// Resolve an `EdgeSelection` to a `Target` — grid-first, drive-line
/// fallback. Mirrors `app.deleteSelectedEdge` / `cycleEdgeDirection` on
/// the UI side.
fn resolve_selection_target(
    ctx: &FixtureCtx,
    sel: &EdgeSelection,
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
