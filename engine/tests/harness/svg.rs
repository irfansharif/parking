//! Deterministic SVG emission for ParkingLayout snapshots.
//!
//! Phase 0 scope: boundary outer, holes, aisle polygons, stalls (colored
//! by kind). Every coordinate is `{:.3}` and geometry is emitted in a
//! fixed order so goldens are byte-stable across runs.

use parking_lot_engine::geom::arc::discretize_polygon;
use parking_lot_engine::types::{
    AisleDirection, DriveAisleGraph, ParkingLayout, Polygon, StallKind, StallQuad, Vec2,
};

pub struct SvgOptions {
    /// Outer margin around the boundary bbox, as a fraction of the max
    /// bbox dimension. 0.05 means +5% on every side.
    pub margin: f64,
}

impl Default for SvgOptions {
    fn default() -> Self {
        Self { margin: 0.05 }
    }
}

pub fn render(boundary: &Polygon, layout: &ParkingLayout, opts: &SvgOptions) -> String {
    // Render the boundary the engine actually sees — the sketch's
    // arcs are discretized before the pipeline runs (see
    // `pipeline/generate.rs`), so showing the raw arc polygon would
    // hide the curvature and desync from the generated aisles.
    let discretized = discretize_polygon(boundary);

    let (min, max) = bbox(&discretized);
    let w = (max.x - min.x).max(1.0);
    let h = (max.y - min.y).max(1.0);
    let pad = w.max(h) * opts.margin;
    let vb_min = Vec2::new(min.x - pad, min.y - pad);
    let vb_w = w + 2.0 * pad;
    let vb_h = h + 2.0 * pad;

    let mut out = String::new();
    out.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{} {} {} {}\">\n",
        f(vb_min.x),
        f(vb_min.y),
        f(vb_w),
        f(vb_h),
    ));
    // SVG y-axis flips down, but parking coords treat y-up. We mirror
    // the whole picture so a CCW outer boundary stays visually CCW.
    out.push_str(&format!(
        "<g transform=\"translate(0,{}) scale(1,-1)\">\n",
        f(2.0 * vb_min.y + vb_h)
    ));

    // Layer order: boundary → holes → aisles → stalls → graph. Each
    // layer is its own <g id="..."> so a reviewer can read the SVG
    // top-to-bottom matching pipeline stages.
    render_boundary(&mut out, &discretized);
    render_aisles(&mut out, &layout.aisle_polygons);
    render_stalls(&mut out, &layout.stalls);
    render_graph(&mut out, &layout.resolved_graph);

    out.push_str("</g>\n</svg>\n");
    out
}

fn render_boundary(out: &mut String, boundary: &Polygon) {
    out.push_str("<g id=\"boundary\" fill=\"none\" stroke=\"#222\" stroke-width=\"0.5\">\n");
    out.push_str(&polygon_el(&boundary.outer, None));
    out.push_str("</g>\n");

    if !boundary.holes.is_empty() {
        out.push_str(
            "<g id=\"holes\" fill=\"#e5e5e5\" stroke=\"#666\" stroke-width=\"0.5\">\n",
        );
        for h in &boundary.holes {
            out.push_str(&polygon_el(h, None));
        }
        out.push_str("</g>\n");
    }
}

fn render_aisles(out: &mut String, aisles: &[Vec<Vec2>]) {
    if aisles.is_empty() {
        return;
    }
    out.push_str(
        "<g id=\"aisles\" fill=\"#bcd5f0\" fill-opacity=\"0.6\" stroke=\"#4a7bb7\" stroke-width=\"0.4\">\n",
    );
    for poly in aisles {
        out.push_str(&polygon_el(poly, None));
    }
    out.push_str("</g>\n");
}

fn render_stalls(out: &mut String, stalls: &[StallQuad]) {
    if stalls.is_empty() {
        return;
    }
    out.push_str("<g id=\"stalls\" stroke=\"#1f5f1a\" stroke-width=\"0.25\">\n");
    for s in stalls {
        // Suppressed stalls are intentionally not rendered — they're a
        // marker the renderer should skip.
        if s.kind == StallKind::Suppressed {
            continue;
        }
        let fill = stall_color(&s.kind);
        out.push_str(&polygon_el(&s.corners, Some(fill)));
    }
    out.push_str("</g>\n");
}

fn render_graph(out: &mut String, graph: &DriveAisleGraph) {
    if graph.edges.is_empty() && graph.vertices.is_empty() {
        return;
    }
    out.push_str("<g id=\"graph\">\n");
    // Edges. Perimeter = brown-orange, interior = blue, matching the
    // DESIGN.md figure conventions.
    for edge in &graph.edges {
        if edge.start >= graph.vertices.len() || edge.end >= graph.vertices.len() {
            continue;
        }
        let a = graph.vertices[edge.start];
        let b = graph.vertices[edge.end];
        let color = if edge.interior { "#3e68a8" } else { "#a84a1e" };
        let dash = match edge.direction {
            AisleDirection::OneWay => " stroke-dasharray=\"2 1\"",
            AisleDirection::TwoWayOriented => " stroke-dasharray=\"4 1\"",
            AisleDirection::TwoWay => "",
        };
        out.push_str(&format!(
            "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"{}\" stroke-width=\"0.6\"{}/>\n",
            f(a.x),
            f(a.y),
            f(b.x),
            f(b.y),
            color,
            dash,
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

fn stall_color(kind: &StallKind) -> &'static str {
    match kind {
        StallKind::Standard => "#8fc98a",
        StallKind::Compact => "#b2d8a8",
        StallKind::Ev => "#7ec6d9",
        StallKind::Extension => "#d4b86a",
        StallKind::Island => "#a1b885",
        // Unreachable — render_stalls skips Suppressed before dispatching
        // here — but the match must be exhaustive.
        StallKind::Suppressed => "#eeeeee",
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

fn bbox(boundary: &Polygon) -> (Vec2, Vec2) {
    let mut min = Vec2::new(f64::INFINITY, f64::INFINITY);
    let mut max = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &boundary.outer {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
    }
    if min.x.is_infinite() {
        // Empty boundary; fall back to a unit box at origin.
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
