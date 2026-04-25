//! Deterministic SVG emission for ParkingLayout snapshots.
//!
//! Phase 0 scope: boundary outer, holes, aisle polygons, stalls (colored
//! by kind). Every coordinate is `{:.3}` and geometry is emitted in a
//! fixed order so goldens are byte-stable across runs.

use parking_lot_engine::geom::arc::discretize_polygon;
use parking_lot_engine::types::{
    AisleDirection, Annotation, DriveAisleGraph, DriveLine, Island, ParkingLayout, Polygon,
    StallKind, StallQuad, Target, TrafficDirection, Vec2,
};

pub struct SvgOptions {
    /// Outer margin around the boundary bbox, as a fraction of the max
    /// bbox dimension. 0.05 means +5% on every side.
    pub margin: f64,
    /// Pixels per user-unit (ft). Controls the SVG's declared `width`/
    /// `height` so stalls render at a fixed physical size regardless of
    /// which viewer is showing the file.
    pub px_per_unit: f64,
}

impl Default for SvgOptions {
    fn default() -> Self {
        Self {
            margin: 0.05,
            px_per_unit: 2.0,
        }
    }
}

pub fn render(
    boundary: &Polygon,
    layout: &ParkingLayout,
    annotations: &[Annotation],
    drive_lines: &[DriveLine],
    opts: &SvgOptions,
    arc_tolerance: f64,
) -> String {
    // Render the boundary the engine actually sees — the sketch's
    // arcs are discretized before the pipeline runs (see
    // `pipeline/generate.rs`), so showing the raw arc polygon would
    // hide the curvature and desync from the generated aisles.
    let discretized = discretize_polygon(boundary, arc_tolerance);

    let (min, max) = content_bbox(&discretized, layout);
    let w = (max.x - min.x).max(1.0);
    let h = (max.y - min.y).max(1.0);
    let pad = w.max(h) * opts.margin;
    let vb_min = Vec2::new(min.x - pad, min.y - pad);
    let vb_w = w + 2.0 * pad;
    let vb_h = h + 2.0 * pad;

    let mut out = String::new();
    // `width`/`height` pin the canonical render size so a 9×18 stall
    // is always the same number of pixels, no matter which viewer
    // opens the file. `preserveAspectRatio="xMidYMid meet"` is the
    // SVG default, but stating it explicitly guards against viewers
    // that would otherwise stretch the viewBox non-uniformly to fill
    // their pane (which makes top/bottom stalls read shorter than
    // the left/right ones).
    out.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" height=\"{}\" viewBox=\"{} {} {} {}\" preserveAspectRatio=\"xMidYMid meet\">\n",
        f(vb_w * opts.px_per_unit),
        f(vb_h * opts.px_per_unit),
        f(vb_min.x),
        f(vb_min.y),
        f(vb_w),
        f(vb_h),
    ));
    // Keep strokes at a constant pixel width regardless of how the
    // viewer scales the viewBox — otherwise small preview panes drop
    // stall boundaries below one pixel and the row reads as a solid
    // band. `vector-effect` doesn't inherit through `<g>` in the
    // spec, so apply it via CSS.
    out.push_str("<style>polygon,polyline,line,path{vector-effect:non-scaling-stroke}</style>\n");
    // Arrow markers for direction-annotated aisles. Two color variants
    // matching the interior/perimeter aisle palette. `markerUnits` is
    // strokeWidth by default, so the arrow scales with line stroke.
    // `orient="auto-start-reverse"` lets `marker-start` point outward
    // from the line so back-to-back arrows on a TwoWayOriented edge
    // form a clear double-headed indicator instead of two arrows
    // pointing the same way.
    out.push_str(
        "<defs>\
<marker id=\"arrow-i\" viewBox=\"0 0 10 10\" refX=\"10\" refY=\"5\" markerWidth=\"6\" markerHeight=\"6\" orient=\"auto-start-reverse\"><path d=\"M0 0L10 5L0 10z\" fill=\"#3e68a8\"/></marker>\
<marker id=\"arrow-p\" viewBox=\"0 0 10 10\" refX=\"10\" refY=\"5\" markerWidth=\"6\" markerHeight=\"6\" orient=\"auto-start-reverse\"><path d=\"M0 0L10 5L0 10z\" fill=\"#a84a1e\"/></marker>\
</defs>\n",
    );
    // SVG y-axis flips down, but parking coords treat y-up. We mirror
    // the whole picture so a CCW outer boundary stays visually CCW.
    out.push_str(&format!(
        "<g transform=\"translate(0,{}) scale(1,-1)\">\n",
        f(2.0 * vb_min.y + vb_h)
    ));

    // Layer order: island fills → stall fills → paint lines → graph.
    // The outer lot boundary / holes aren't drawn as a separate layer
    // — the perimeter graph edges already trace them in red-dash, and
    // an underlying boundary stroke would show through the dash gaps
    // and tint the red line.
    render_islands(&mut out, &layout.islands);
    render_stalls(&mut out, &layout.stalls);
    render_paint(&mut out, &layout.stalls, &layout.islands);
    render_graph(&mut out, &layout.resolved_graph);
    render_annotations(&mut out, annotations, drive_lines);

    out.push_str("</g>\n</svg>\n");
    out
}

fn render_islands(out: &mut String, islands: &[Island]) {
    if islands.is_empty() {
        return;
    }
    // Landscape green fill only — the outline is drawn in the paint
    // layer so it sits on top of stall fills at the edges. Holes are
    // carved with fill-rule="evenodd" to match the UI's even-odd fill.
    out.push_str("<g id=\"islands\" fill=\"#9fbf8a\" fill-rule=\"evenodd\" stroke=\"none\">\n");
    for isl in islands {
        if isl.contour.is_empty() {
            continue;
        }
        out.push_str("<path d=\"");
        append_subpath(out, &isl.contour);
        for hole in &isl.holes {
            if hole.is_empty() {
                continue;
            }
            out.push(' ');
            append_subpath(out, hole);
        }
        out.push_str("\"/>\n");
    }
    out.push_str("</g>\n");
}

fn append_subpath(out: &mut String, pts: &[Vec2]) {
    for (i, p) in pts.iter().enumerate() {
        if i == 0 {
            out.push_str(&format!("M{} {}", f(p.x), f(p.y)));
        } else {
            out.push_str(&format!(" L{} {}", f(p.x), f(p.y)));
        }
    }
    out.push_str(" Z");
}

fn render_stalls(out: &mut String, stalls: &[StallQuad]) {
    if stalls.is_empty() {
        return;
    }
    // Fill only — the painted sides are emitted in the paint layer so
    // each stall reads as "3 painted edges + open entrance" rather than
    // a closed rectangle. Standard and Island stalls are left unfilled
    // since the paint lines (and hatch, for islands) carry all the
    // needed information on their own.
    out.push_str("<g id=\"stalls\" stroke=\"none\">\n");
    for s in stalls {
        // Suppressed stalls are intentionally not rendered — they're a
        // marker the renderer should skip.
        if s.kind == StallKind::Suppressed {
            continue;
        }
        let Some(fill) = stall_color(&s.kind) else {
            continue;
        };
        out.push_str(&polygon_el(&s.corners, Some(fill)));
    }
    out.push_str("</g>\n");
}

fn render_paint(out: &mut String, stalls: &[StallQuad], islands: &[Island]) {
    let any_stall = stalls.iter().any(|s| s.kind != StallKind::Suppressed);
    if !any_stall && islands.is_empty() {
        return;
    }
    // Paint-stripe look: dark charcoal lines on top of fills, matching
    // the UI's white-on-asphalt conventions translated for a white
    // background. Stroke width is a bit heavier than the fill outlines
    // so the paint reads as the dominant line work.
    out.push_str(
        "<g id=\"paint\" fill=\"none\" stroke=\"#2a2a2a\" stroke-width=\"0.4\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
    );

    // Stall paint: 3 sides, front open. Regular stalls leave [2]→[3]
    // open (aisle/entrance); island stalls leave [0]→[1] open (back,
    // connecting to the island).
    for s in stalls {
        if s.kind == StallKind::Suppressed {
            continue;
        }
        let c = &s.corners;
        let path = if s.kind == StallKind::Island {
            [c[1], c[2], c[3], c[0]]
        } else {
            [c[3], c[0], c[1], c[2]]
        };
        out.push_str("<polyline points=\"");
        for (i, p) in path.iter().enumerate() {
            if i > 0 {
                out.push(' ');
            }
            out.push_str(&format!("{},{}", f(p.x), f(p.y)));
        }
        out.push_str("\"/>\n");
    }

    // Island-stall hatching: 45° stripes clipped to each stall quad.
    // Computed as line–quad intersections so the output is a flat list
    // of segments — no SVG <clipPath> plumbing needed to stay
    // byte-stable.
    for s in stalls {
        if s.kind != StallKind::Island {
            continue;
        }
        emit_hatch(out, &s.corners);
    }

    // Island outlines — edge + hole boundaries painted on top, so the
    // green landscape has the same "painted curb" read as stall edges.
    for isl in islands {
        if isl.contour.is_empty() {
            continue;
        }
        out.push_str("<path d=\"");
        append_subpath(out, &isl.contour);
        for hole in &isl.holes {
            if hole.is_empty() {
                continue;
            }
            out.push(' ');
            append_subpath(out, hole);
        }
        out.push_str("\"/>\n");
    }

    out.push_str("</g>\n");
}

/// Emit 45° hatch segments clipped to a convex quad, matching the UI's
/// island-stall hatch (back-edge direction rotated by 45°, spacing=3).
fn emit_hatch(out: &mut String, corners: &[Vec2; 4]) {
    // Back edge: [0] → [1]. Hatch direction = back edge rotated by 45°.
    let ux = corners[1].x - corners[0].x;
    let uy = corners[1].y - corners[0].y;
    let ulen = (ux * ux + uy * uy).sqrt();
    if ulen < 1e-9 {
        return;
    }
    let unx = ux / ulen;
    let uny = uy / ulen;
    let a = std::f64::consts::FRAC_PI_4;
    let hx = unx * a.cos() - uny * a.sin();
    let hy = unx * a.sin() + uny * a.cos();
    // Perpendicular sweep axis.
    let nx = -hy;
    let ny = hx;

    // Project corners onto sweep axis to bracket the hatch range.
    let projs: [f64; 4] = [
        corners[0].x * nx + corners[0].y * ny,
        corners[1].x * nx + corners[1].y * ny,
        corners[2].x * nx + corners[2].y * ny,
        corners[3].x * nx + corners[3].y * ny,
    ];
    let min_p = projs.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_p = projs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let spacing = 3.0_f64;
    // Start at the first multiple of `spacing` inside [min_p, max_p] so
    // the pattern is translation-invariant: two island stalls that share
    // a back edge get hatches that align across the seam rather than
    // jittering based on each quad's own min corner.
    let start = (min_p / spacing).ceil() * spacing;

    let mut p = start;
    while p <= max_p + 1e-9 {
        if let Some((q0, q1)) = clip_line_to_quad(corners, hx, hy, nx, ny, p) {
            out.push_str(&format!(
                "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>\n",
                f(q0.x),
                f(q0.y),
                f(q1.x),
                f(q1.y),
            ));
        }
        p += spacing;
    }
}

/// Intersect the infinite line `{x : x·n = p}` (direction `h`) with a
/// convex quad, returning the entry/exit points.
fn clip_line_to_quad(
    corners: &[Vec2; 4],
    hx: f64,
    hy: f64,
    nx: f64,
    ny: f64,
    p: f64,
) -> Option<(Vec2, Vec2)> {
    // Signed distance of each vertex from the line.
    let d: [f64; 4] = [
        corners[0].x * nx + corners[0].y * ny - p,
        corners[1].x * nx + corners[1].y * ny - p,
        corners[2].x * nx + corners[2].y * ny - p,
        corners[3].x * nx + corners[3].y * ny - p,
    ];
    let mut hits: Vec<Vec2> = Vec::with_capacity(2);
    for i in 0..4 {
        let j = (i + 1) % 4;
        if (d[i] >= 0.0 && d[j] < 0.0) || (d[i] < 0.0 && d[j] >= 0.0) {
            let t = d[i] / (d[i] - d[j]);
            hits.push(Vec2::new(
                corners[i].x + t * (corners[j].x - corners[i].x),
                corners[i].y + t * (corners[j].y - corners[i].y),
            ));
            if hits.len() == 2 {
                break;
            }
        }
    }
    if hits.len() < 2 {
        return None;
    }
    // Order the hits along +h so the segment direction is deterministic
    // (flipping the sign of the edge-crossing traversal would otherwise
    // swap endpoints between adjacent stalls).
    let t0 = hits[0].x * hx + hits[0].y * hy;
    let t1 = hits[1].x * hx + hits[1].y * hy;
    if t0 <= t1 {
        Some((hits[0], hits[1]))
    } else {
        Some((hits[1], hits[0]))
    }
}

fn render_graph(out: &mut String, graph: &DriveAisleGraph) {
    if graph.edges.is_empty() && graph.vertices.is_empty() {
        return;
    }
    out.push_str("<g id=\"graph\">\n");
    // Edges. Perimeter = brown-orange, interior = blue, matching the
    // DESIGN.md figure conventions. Undirected edges are frequently
    // stored twice (A→B and B→A) in `resolved_graph.edges`, which
    // would double-draw the same dashed segment with opposing dash
    // phases — producing a solid-looking line instead of dashes, or
    // the appearance of a second overlapping color. Dedupe by
    // unordered vertex pair and draw each segment once; if the same
    // pair carries both perimeter and interior edges, perimeter wins
    // so the lot outline reads cleanly on top.
    // Track each unique edge by (min, max) vertex pair. For directional
    // edges keep the actual `start → end` order (which the annotation
    // pipeline aligns with travel direction) so we can orient
    // arrowheads correctly.
    let mut seen: std::collections::HashMap<(usize, usize), (bool, AisleDirection, usize, usize)> =
        std::collections::HashMap::new();
    let mut ordered_keys: Vec<(usize, usize)> = Vec::new();
    for edge in &graph.edges {
        if edge.start >= graph.vertices.len() || edge.end >= graph.vertices.len() {
            continue;
        }
        let key = if edge.start <= edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        match seen.get_mut(&key) {
            Some(entry) => {
                // Promote to perimeter (interior=false) if any copy is
                // perimeter. If a directional copy shows up after a
                // TwoWay copy, adopt its direction + travel order so
                // the arrowhead points the right way.
                if entry.0 && !edge.interior {
                    entry.0 = false;
                }
                if matches!(entry.1, AisleDirection::TwoWay)
                    && !matches!(edge.direction, AisleDirection::TwoWay)
                {
                    entry.1 = edge.direction.clone();
                    entry.2 = edge.start;
                    entry.3 = edge.end;
                }
            }
            None => {
                seen.insert(
                    key,
                    (edge.interior, edge.direction.clone(), edge.start, edge.end),
                );
                ordered_keys.push(key);
            }
        }
    }
    for key in &ordered_keys {
        let (interior, direction, sidx, didx) = &seen[key];
        let a = graph.vertices[*sidx];
        let b = graph.vertices[*didx];
        let color = if *interior { "#3e68a8" } else { "#a84a1e" };
        let marker_id = if *interior { "arrow-i" } else { "arrow-p" };
        // Road-style centerline: every aisle is dashed so the graph
        // reads as a drivable path. Dash numbers are in world units
        // (feet) — an 8-ft dash + 5-ft gap reads as road paint at the
        // typical parking-lot scale. Directional variants add density
        // on top of the base cadence.
        let dash = match *direction {
            AisleDirection::TwoWay => " stroke-dasharray=\"8 5\"",
            AisleDirection::OneWay => " stroke-dasharray=\"4 2\"",
            AisleDirection::TwoWayOriented | AisleDirection::TwoWayOrientedReverse => {
                " stroke-dasharray=\"8 2\""
            }
        };
        // Arrowheads encode direction:
        //   OneWay           → single arrow at the travel-end of each segment.
        //   TwoWayOriented   → arrows at both ends, pointing outward, so the
        //                       segment reads as a double-headed arrow.
        //   TwoWay           → no markers.
        let markers = match *direction {
            AisleDirection::TwoWay => String::new(),
            AisleDirection::OneWay => format!(" marker-end=\"url(#{marker_id})\""),
            AisleDirection::TwoWayOriented | AisleDirection::TwoWayOrientedReverse => format!(
                " marker-start=\"url(#{marker_id})\" marker-end=\"url(#{marker_id})\""
            ),
        };
        out.push_str(&format!(
            "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"{}\" stroke-width=\"0.6\"{}{}/>\n",
            f(a.x),
            f(a.y),
            f(b.x),
            f(b.y),
            color,
            dash,
            markers,
        ));
    }
    // Vertices.
    for (i, v) in graph.vertices.iter().enumerate() {
        let is_perim = i < graph.perim_vertex_count;
        let fill = if is_perim { "#a84a1e" } else { "#3e68a8" };
        out.push_str(&format!(
            "<circle cx=\"{}\" cy=\"{}\" r=\"0.8\" fill=\"{}\"/>\n",
            f(v.x),
            f(v.y),
            fill
        ));
    }
    out.push_str("</g>\n");
}

/// Overlay annotation badges on top of the graph layer so the user can
/// see *where* a Direction annotation was applied. Each badge is a
/// filled chevron pointing in the annotated traffic direction:
///   - OneWay / OneWayReverse           → single chevron.
///   - TwoWayOriented / *Reverse        → double chevron (back-to-back).
///
/// Only DriveLine targets are resolved here — Perimeter and Grid
/// targets need carrier-substrate context that the harness doesn't
/// have direct access to. Unresolved annotations are silently skipped.
fn render_annotations(out: &mut String, annotations: &[Annotation], drive_lines: &[DriveLine]) {
    let mut emitted = false;
    let open = |out: &mut String, emitted: &mut bool| {
        if !*emitted {
            out.push_str("<g id=\"annotations\" fill=\"#d97706\" stroke=\"#7c3a06\" stroke-width=\"0.3\">\n");
            *emitted = true;
        }
    };
    for ann in annotations {
        let Annotation::Direction { target, traffic } = ann else { continue };
        let Target::DriveLine { id, t } = target else { continue };
        let Some(line) = drive_lines.iter().find(|d| d.id == *id) else { continue };
        let pos = Vec2::new(
            line.start.x + (line.end.x - line.start.x) * t,
            line.start.y + (line.end.y - line.start.y) * t,
        );
        // Direction unit vector along the drive line, flipped for the
        // *Reverse variants so the chevron always points along traffic flow.
        let dx = line.end.x - line.start.x;
        let dy = line.end.y - line.start.y;
        let len = (dx * dx + dy * dy).sqrt().max(1e-9);
        let (mut ux, mut uy) = (dx / len, dy / len);
        if matches!(
            traffic,
            TrafficDirection::OneWayReverse | TrafficDirection::TwoWayOrientedReverse
        ) {
            ux = -ux;
            uy = -uy;
        }
        // Perpendicular (left-perp) for chevron base width.
        let (px, py) = (-uy, ux);
        let two_way = matches!(
            traffic,
            TrafficDirection::TwoWayOriented | TrafficDirection::TwoWayOrientedReverse
        );
        // Chevron geometry: tip ahead of `pos` along travel, wings behind.
        let tip_len = 5.0_f64;
        let half_w = 3.0_f64;
        let tail = if two_way { tip_len * 0.4 } else { 0.0 };
        let chevron = |center_along: f64| -> String {
            let cx = pos.x + ux * center_along;
            let cy = pos.y + uy * center_along;
            let tip = (cx + ux * tip_len, cy + uy * tip_len);
            let lwing = (cx - ux * tail + px * half_w, cy - uy * tail + py * half_w);
            let rwing = (cx - ux * tail - px * half_w, cy - uy * tail - py * half_w);
            format!(
                "<polygon points=\"{},{} {},{} {},{}\"/>\n",
                f(tip.0),
                f(tip.1),
                f(lwing.0),
                f(lwing.1),
                f(rwing.0),
                f(rwing.1),
            )
        };
        open(out, &mut emitted);
        if two_way {
            // Two stacked chevrons offset along travel axis.
            out.push_str(&chevron(-tip_len));
            out.push_str(&chevron(tip_len * 0.2));
        } else {
            out.push_str(&chevron(0.0));
        }
    }
    if emitted {
        out.push_str("</g>\n");
    }
}

fn stall_color(kind: &StallKind) -> Option<&'static str> {
    // Only semantic "non-default" kinds get a fill. Standard stalls are
    // left transparent so the paint lines carry the stall shape on their
    // own, and Island stalls are left transparent so the hatch + island
    // green underneath do the same job without a second green layered
    // on top.
    match kind {
        StallKind::Standard | StallKind::Island => None,
        StallKind::Compact => Some("#d8d096"),
        StallKind::Ev => Some("#7ec6d9"),
        // Unreachable — render_stalls skips Suppressed before dispatching
        // here — but the match must be exhaustive.
        StallKind::Suppressed => None,
    }
}

fn polygon_el<P: AsRef<[Vec2]>>(pts: P, fill: Option<&str>) -> String {
    let pts = pts.as_ref();
    let mut s = String::from("<polygon points=\"");
    for (i, p) in pts.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        s.push_str(&format!("{},{}", f(p.x), f(p.y)));
    }
    s.push('"');
    if let Some(c) = fill {
        s.push_str(&format!(" fill=\"{}\"", c));
    }
    s.push_str("/>\n");
    s
}

/// Union of every renderable point — boundary, aisle polygons, stall
/// quads. Perimeter stalls sit outside the user-drawn boundary, so a
/// boundary-only bbox clips them on smaller lots (DESIGN note: the
/// boundary the user draws is the drive-aisle perimeter, and stalls
/// extend beyond it).
fn content_bbox(boundary: &Polygon, layout: &ParkingLayout) -> (Vec2, Vec2) {
    let mut min = Vec2::new(f64::INFINITY, f64::INFINITY);
    let mut max = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut extend = |p: &Vec2| {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
    };
    for p in &boundary.outer {
        extend(p);
    }
    for s in &layout.stalls {
        if s.kind == StallKind::Suppressed {
            continue;
        }
        for p in &s.corners {
            extend(p);
        }
    }
    for isl in &layout.islands {
        for p in &isl.contour {
            extend(p);
        }
    }
    if min.x.is_infinite() {
        // Empty content; fall back to a unit box at origin.
        return (Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    }
    (min, max)
}

/// Format a float with 3 decimals, trimming a trailing ".000" so
/// integer-valued coords don't carry noisy zeros in the golden. Any
/// value that rounds to zero collapses to "0" (no signed zero, no
/// empty-string trailing from a too-eager strip).
fn f(v: f64) -> String {
    let s = format!("{:.3}", v);
    match s.as_str() {
        "0.000" | "-0.000" => "0".to_string(),
        _ => s.strip_suffix(".000").map(|s| s.to_string()).unwrap_or(s),
    }
}

#[cfg(test)]
mod tests {
    use super::f;

    #[test]
    fn coord_formatter_handles_edges() {
        assert_eq!(f(0.0), "0");
        assert_eq!(f(-0.0), "0");
        assert_eq!(f(1e-10), "0");
        assert_eq!(f(-1e-10), "0");
        assert_eq!(f(1.0), "1");
        assert_eq!(f(-123.0), "-123");
        assert_eq!(f(1.5), "1.500");
        assert_eq!(f(-0.5), "-0.500");
    }
}
