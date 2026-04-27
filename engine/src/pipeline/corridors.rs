//! Aisle polygon construction (DESIGN.md §3.2).
//!
//! Each undirected aisle edge is extruded into a rectangle. Miter-fill
//! wedges close the gaps at junctions. Boolean union of all rectangles
//! plus miter fills produces the merged driveable surface, with cleanup
//! passes for spikes, simplification, and small junction holes.

use crate::geom::boolean::{self, FillRule};
use crate::geom::inset::signed_area;
use crate::types::*;

/// Build the corridor rectangle for a single aisle edge.
///
/// Direction-annotated edges flip their stored start/end to encode
/// traffic direction. Independent of that orientation, the corridor
/// rectangle's vertex order must stay consistent so downstream boolean
/// union and face extraction see a stable winding for every edge.
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

/// Per-edge corridor rectangles. Returns (polygon, interior,
/// travel_dir); travel_dir is Some for one-way edges only.
pub(crate) fn deduplicate_corridors(
    graph: &DriveAisleGraph,
) -> Vec<(Vec<Vec2>, bool, Option<Vec2>)> {
    let mut corridors = Vec::new();
    for edge in &graph.edges {
        if edge.start == edge.end { continue; }
        // Only OneWay aisles affect stall lean — they need per-side
        // flip_angle to produce a herringbone chevron in travel
        // direction. TwoWay and TwoWayReverse keep the default
        // mirror-symmetric stall pattern; *Reverse only mirrors the
        // lean across the aisle axis (handled separately).
        let s = graph.vertices[edge.start];
        let e = graph.vertices[edge.end];
        let travel_dir = match edge.direction {
            Some(AisleDirection::OneWay) => Some((e - s).normalize()),
            Some(AisleDirection::OneWayReverse) => Some((s - e).normalize()),
            Some(AisleDirection::TwoWayReverse) | None => None,
        };
        corridors.push((corridor_polygon(&graph.vertices, edge), edge.interior, travel_dir));
    }
    corridors
}

/// Miter-fill wedges at graph vertices where edges meet. For each
/// consecutive pair of outgoing edges (sorted by angle), produce a
/// triangular/quadrilateral fill that bridges their offset rects.
/// Outer/convex gaps are capped to prevent acute miters from
/// shooting spikes across the lot.
pub(crate) fn generate_miter_fills(graph: &DriveAisleGraph, debug: &DebugToggles) -> Vec<Vec<Vec2>> {
    if !debug.miter_fills {
        return vec![];
    }
    let mut fills = Vec::new();

    let nv = graph.vertices.len();
    let mut adj: Vec<Vec<(Vec2, f64, bool)>> = vec![vec![]; nv];

    for edge in &graph.edges {
        let s = graph.vertices[edge.start];
        let e = graph.vertices[edge.end];
        if (e - s).length() < 1e-9 { continue; }
        let dir = (e - s).normalize();
        let w = edge.width;

        adj[edge.start].push((dir, w, edge.interior));
        adj[edge.end].push((Vec2::new(-dir.x, -dir.y), w, edge.interior));
    }

    for vi in 0..nv {
        if adj[vi].len() < 2 {
            continue;
        }

        if debug.boundary_only_miters && adj[vi].iter().all(|(_, _, int)| *int) {
            continue;
        }

        let v = graph.vertices[vi];

        let mut edges_sorted: Vec<(f64, Vec2, f64)> = adj[vi]
            .iter()
            .map(|(d, w, _)| (d.y.atan2(d.x), *d, *w))
            .collect();
        edges_sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let ne = edges_sorted.len();
        for i in 0..ne {
            let j = (i + 1) % ne;
            let (a1, d1, w1) = edges_sorted[i];
            let (a2, d2, w2) = edges_sorted[j];

            let n1 = Vec2::new(-d1.y, d1.x);
            let n2 = Vec2::new(-d2.y, d2.x);

            let p1 = v + n1 * w1;
            let p2 = v - n2 * w2;

            let denom = d1.cross(d2);
            if denom.abs() < 1e-12 {
                continue;
            }

            let t = (p2 - p1).cross(d2) / denom;
            let miter = p1 + d1 * t;

            let gap = if j > i {
                a2 - a1
            } else {
                (a2 + std::f64::consts::TAU) - a1
            };

            if gap < std::f64::consts::PI {
                let max_w = w1.max(w2);
                if (miter - v).length() > max_w * 4.0 {
                    continue;
                }
            }

            let wedge = vec![v, p1, miter, p2];
            if signed_area(&wedge).abs() < 1.0 {
                continue;
            }
            fills.push(wedge);
        }
    }

    fills
}

/// Boolean-union all corridor rectangles + miter fills into merged
/// shapes. Each shape is `Vec<Vec<Vec2>>` where [0] = outer contour
/// and [1..] = hole contours (for corridor loops enclosing faces).
pub(crate) fn merge_corridor_shapes(
    corridors: &[Vec<Vec2>],
    graph: &DriveAisleGraph,
    debug: &DebugToggles,
) -> Vec<Vec<Vec<Vec2>>> {
    if corridors.is_empty() {
        return vec![];
    }

    let miter_fills = generate_miter_fills(graph, debug);

    let subj: Vec<Vec<Vec2>> = corridors
        .iter()
        .chain(miter_fills.iter())
        .map(|c| {
            let mut pts = c.clone();
            if signed_area(&pts) < 0.0 {
                pts.reverse();
            }
            pts
        })
        .collect();

    if subj.is_empty() {
        return vec![];
    }

    let result = boolean::union(&subj, &[], FillRule::NonZero);

    result
        .into_iter()
        .filter(|shape| !shape.is_empty())
        .map(|shape| {
            shape
                .into_iter()
                .map(|pts| {
                    if debug.spike_removal {
                        remove_contour_spikes(pts, 2.0)
                    } else {
                        pts
                    }
                })
                .collect()
        })
        .collect()
}

/// Remove anti-parallel "spike" vertices left by boolean-union of
/// thin wedges with corridor rects.
fn remove_contour_spikes(mut contour: Vec<Vec2>, tolerance: f64) -> Vec<Vec2> {
    let mut changed = true;
    while changed {
        changed = false;
        let n = contour.len();
        if n < 4 {
            break;
        }
        for i in 0..n {
            let prev = if i == 0 { n - 1 } else { i - 1 };
            let next = (i + 1) % n;
            let a = contour[prev];
            let b = contour[i];
            let c = contour[next];
            let ab = b - a;
            let bc = c - b;
            let ab_len = ab.length();
            let bc_len = bc.length();
            if ab_len < f64::EPSILON || bc_len < f64::EPSILON {
                contour.remove(i);
                changed = true;
                break;
            }
            let ab_n = ab * (1.0 / ab_len);
            let bc_n = bc * (1.0 / bc_len);
            if ab_n.dot(bc_n) < -0.95 && (c - a).length() < tolerance {
                let remove_second = next;
                if remove_second > i {
                    contour.remove(remove_second);
                    contour.remove(i);
                } else {
                    contour.remove(i);
                    contour.remove(remove_second);
                }
                changed = true;
                break;
            }
        }
    }
    contour
}
