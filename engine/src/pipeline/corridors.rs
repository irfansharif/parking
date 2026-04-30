//! Aisle polygon construction (DESIGN.md §3.2).
//!
//! The merged driveable surface comes from stroking the centerline
//! graph through `i_overlay::StrokeOffset`. Perimeter and hole loops
//! are passed as closed polylines so their corners miter internally;
//! interior aisles are passed as 2-point open polylines with butt
//! caps (their perpendicular butts overlap at every interior 4-way
//! crossing, so the centre is fully covered without explicit
//! wedges). The stroker handles extrusion, joins, caps, and despike
//! in one shot, so this module no longer carries hand-rolled
//! rectangle building, junction wedge computation, or post-union
//! spike cleanup.
//!
//! `corridor_polygon` / `corridor_polygons` remain — they emit per-edge
//! rectangles consumed by face-edge tagging (`tag_face_edges`) to
//! classify each face contour edge against the corridor that carved it.

use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, LineJoin, StrokeStyle};

use crate::geom::boolean::{self, FillRule};
use crate::types::*;

/// Build the corridor rectangle for a single aisle edge.
///
/// Direction-annotated edges flip their stored start/end to encode
/// traffic direction. Independent of that orientation, the corridor
/// rectangle's vertex order must stay consistent so downstream
/// face-edge tagging sees a stable winding for every edge.
pub(crate) fn corridor_polygon(vertices: &[Vec2], edge: &AisleEdge) -> Vec<Vec2> {
    let s = vertices[edge.start];
    let e = vertices[edge.end];
    // Canonicalize endpoint order by world position so reversed
    // direction annotations don't flip the polygon's winding.
    let (a, b) = if (s.x, s.y) <= (e.x, e.y) { (s, e) } else { (e, s) };
    let dir = (b - a).normalize();
    let normal = Vec2::new(-dir.y, dir.x);
    let w = edge.width;
    vec![
        a + normal * w,
        b + normal * w,
        b - normal * w,
        a - normal * w,
    ]
}

/// Build one corridor rectangle per aisle edge, paired with the
/// edge's `interior` flag and a `travel_dir` when the edge is OneWay
/// (None for two-way and TwoWayReverse). Consumed by `tag_face_edges`
/// to classify each face edge against its source corridor.
pub(crate) fn corridor_polygons(
    graph: &DriveAisleGraph,
) -> Vec<(Vec<Vec2>, bool, Option<Vec2>)> {
    let mut corridors = Vec::new();
    for edge in &graph.edges {
        if edge.start == edge.end { continue; }
        let s = graph.vertices[edge.start];
        let e = graph.vertices[edge.end];
        // Only OneWay aisles affect stall lean — they need per-side
        // flip_angle to produce a herringbone chevron in travel
        // direction. TwoWay and TwoWayReverse keep the default
        // mirror-symmetric stall pattern; *Reverse only mirrors the
        // lean across the aisle axis (handled separately).
        let travel_dir = match edge.direction {
            Some(AisleDirection::OneWay) => Some((e - s).normalize()),
            Some(AisleDirection::OneWayReverse) => Some((s - e).normalize()),
            Some(AisleDirection::TwoWayReverse) | None => None,
        };
        corridors.push((corridor_polygon(&graph.vertices, edge), edge.interior, travel_dir));
    }
    corridors
}

/// Minimum interior corner angle (radians) below which a miter join
/// falls back to a bevel. Aisles meeting at very acute angles would
/// otherwise produce miter spikes far longer than the aisle is wide;
/// 15° is roughly where the miter length exceeds ~4× the half-width
/// (the cap the previous hand-rolled implementation used).
const MITER_MIN_SHARP_ANGLE: f64 = 15.0 * std::f64::consts::PI / 180.0;

/// Stroke the centerline graph by `aisle_width` (mitered joins along
/// closed perimeter/hole loops, butt caps on interior segment ends)
/// and return the merged driveable surface. Each output shape is
/// `Vec<Vec<Vec2>>` with `[0]` = outer contour and `[1..]` = hole
/// contours (for corridor loops enclosing a face).
///
/// `aisle_width` is the half-width of an undirected aisle (the same
/// `edge.width` carried on each `AisleEdge`); the stroker doubles it
/// internally — `StrokeStyle::new(_)` takes full stroke width, and a
/// two-way aisle is two lanes wide.
pub(crate) fn merge_corridor_surface(
    graph: &DriveAisleGraph,
    aisle_width: f64,
) -> Vec<Vec<Vec<Vec2>>> {
    let stroke_width = aisle_width * 2.0;
    // Round caps on segment endpoints. Each interior aisle is its own
    // 2-point polyline; at every junction multiple aisles meet and their
    // round semicircles compose into a full disk that covers oblique
    // angles. With butt caps, an oblique 4-way junction (e.g., a diagonal
    // drive line crossing a perpendicular cross-aisle) leaves triangular
    // gaps on the obtuse-angle sides, which survive into the face
    // polygon as "nubs" and let stalls encroach into the corridor.
    // `Round(angle)` controls the arc segment size; pi/16 ≈ 11° is fine
    // for visualisation and keeps the polyline count modest.
    let cap = || LineCap::Round(std::f64::consts::PI / 16.0);
    let make_style = || {
        StrokeStyle::new(stroke_width)
            .line_join(LineJoin::Miter(MITER_MIN_SHARP_ANGLE))
            .start_cap(cap())
            .end_cap(cap())
    };

    let mut union_paths: Vec<Vec<Vec2>> = Vec::new();

    // Perimeter and hole sub-graph — walk maximal paths through it.
    // A simple cycle (no deletions) yields one closed polyline per
    // loop; an annotation that deletes a perimeter vertex breaks
    // that cycle into one or more open polylines that we stroke
    // with butt caps so the "doorway" stays open downstream.
    let perim_paths = extract_perim_paths(graph);
    let (closed, open): (Vec<_>, Vec<_>) = perim_paths.into_iter().partition(|p| p.closed);
    if !closed.is_empty() {
        let coords: Vec<Vec<[f64; 2]>> = closed
            .iter()
            .map(|p| p.verts.iter().map(|v| [v.x, v.y]).collect())
            .collect();
        push_stroke(&mut union_paths, &coords, make_style(), true);
    }
    if !open.is_empty() {
        let coords: Vec<Vec<[f64; 2]>> = open
            .iter()
            .map(|p| p.verts.iter().map(|v| [v.x, v.y]).collect())
            .collect();
        push_stroke(&mut union_paths, &coords, make_style(), false);
    }

    // Interior aisles — each edge as its own 2-point polyline. Butt
    // caps at every endpoint; perpendicular butts at interior 4-way
    // crossings overlap, so the junction is fully covered without
    // explicit wedge geometry.
    let interior_segments: Vec<Vec<[f64; 2]>> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| e.interior && e.start != e.end)
        .map(|(_, e)| {
            let s = graph.vertices[e.start];
            let t = graph.vertices[e.end];
            vec![[s.x, s.y], [t.x, t.y]]
        })
        .collect();
    if !interior_segments.is_empty() {
        push_stroke(&mut union_paths, &interior_segments, make_style(), false);
    }

    if union_paths.is_empty() {
        return vec![];
    }

    // Stroke produces multiple disjoint shapes when the graph is in
    // multiple components (e.g., outer + holes); their unions are
    // already de-duped, but boolean-unioning here merges adjacent
    // shapes that happen to touch (interior aisle running into a
    // perimeter loop) into a single contour with consistent winding.
    boolean::union(&union_paths, &[], FillRule::NonZero)
}

struct PerimPath {
    verts: Vec<Vec2>,
    closed: bool,
}

/// Decompose the perimeter sub-graph (`!edge.interior`) into maximal
/// paths. A simple loop yields one closed polyline. A loop broken by
/// a deleted perimeter vertex (the surrounding edges go away with it)
/// yields one or more open polylines whose endpoints sit at the
/// would-be deletion points; the caller butt-caps those so the
/// driveway "doorway" is preserved.
fn extract_perim_paths(graph: &DriveAisleGraph) -> Vec<PerimPath> {
    let n = graph.vertices.len();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (ei, e) in graph.edges.iter().enumerate() {
        if e.interior || e.start == e.end {
            continue;
        }
        adj[e.start].push(ei);
        adj[e.end].push(ei);
    }

    let mut visited = vec![false; graph.edges.len()];
    let mut out = Vec::new();
    for start_ei in 0..graph.edges.len() {
        let e0 = &graph.edges[start_ei];
        if e0.interior || e0.start == e0.end || visited[start_ei] {
            continue;
        }
        visited[start_ei] = true;

        // Forward: walk from e0.end onward.
        let mut forward_verts: Vec<Vec2> = Vec::new();
        let (mut cur_v, mut cur_e) = (e0.end, start_ei);
        let mut closed = false;
        loop {
            let next_e = adj[cur_v]
                .iter()
                .copied()
                .find(|&ei| ei != cur_e && !visited[ei]);
            let Some(ne) = next_e else { break };
            visited[ne] = true;
            let edge = &graph.edges[ne];
            let next_v = if edge.start == cur_v { edge.end } else { edge.start };
            if next_v == e0.start {
                // Loop closed back through e0's start. Stop without
                // re-emitting the start vertex (the closed path
                // wraps).
                closed = true;
                break;
            }
            forward_verts.push(graph.vertices[next_v]);
            cur_v = next_v;
            cur_e = ne;
        }

        // Backward: walk from e0.start outward (only relevant when the
        // loop wasn't closed forward — otherwise we'd retrace).
        let mut backward_verts: Vec<Vec2> = Vec::new();
        if !closed {
            let (mut cur_v, mut cur_e) = (e0.start, start_ei);
            loop {
                let next_e = adj[cur_v]
                    .iter()
                    .copied()
                    .find(|&ei| ei != cur_e && !visited[ei]);
                let Some(ne) = next_e else { break };
                visited[ne] = true;
                let edge = &graph.edges[ne];
                let next_v = if edge.start == cur_v { edge.end } else { edge.start };
                backward_verts.push(graph.vertices[next_v]);
                cur_v = next_v;
                cur_e = ne;
            }
        }

        // Assemble: backward (reversed) ++ e0.start ++ e0.end ++ forward.
        let mut verts = Vec::with_capacity(2 + forward_verts.len() + backward_verts.len());
        for v in backward_verts.iter().rev() {
            verts.push(*v);
        }
        verts.push(graph.vertices[e0.start]);
        verts.push(graph.vertices[e0.end]);
        verts.extend(forward_verts);

        if verts.len() >= 2 {
            out.push(PerimPath { verts, closed });
        }
    }
    out
}

fn push_stroke(
    out: &mut Vec<Vec<Vec2>>,
    paths: &[Vec<[f64; 2]>],
    style: StrokeStyle<[f64; 2], f64>,
    is_closed: bool,
) {
    let shapes = paths.stroke(style, is_closed);
    for shape in shapes {
        for contour in shape {
            out.push(contour.into_iter().map(|p| Vec2::new(p[0], p[1])).collect());
        }
    }
}
