//! Aisle polygon construction (DESIGN.md §3.2).
//!
//! Each undirected aisle edge is extruded into a rectangle. Miter-fill
//! wedges close the gaps at junctions. Boolean union of all rectangles
//! plus miter fills produces the merged driveable surface, with cleanup
//! passes for spikes, simplification, and small junction holes.

use crate::geom::boolean::{self, FillRule};
use crate::geom::inset::signed_area;
use crate::types::*;
use geo::{Coord, LineString, Simplify};

/// Build the corridor rectangle for a single aisle edge.
pub(crate) fn corridor_polygon(vertices: &[Vec2], edge: &AisleEdge) -> Vec<Vec2> {
    let start = vertices[edge.start];
    let end = vertices[edge.end];
    let dir = (end - start).normalize();
    let normal = Vec2::new(-dir.y, dir.x);
    let w = edge.width;
    vec![
        start + normal * w,
        end + normal * w,
        end - normal * w,
        start - normal * w,
    ]
}

/// Deduplicated corridor rectangles (one per undirected edge).
/// Returns (polygon, interior, travel_dir); travel_dir is Some for
/// one-way and two-way-oriented edges.
pub(crate) fn deduplicate_corridors(
    graph: &DriveAisleGraph,
) -> Vec<(Vec<Vec2>, bool, Option<Vec2>)> {
    let mut seen = std::collections::HashSet::new();
    let mut corridors = Vec::new();
    for edge in &graph.edges {
        if edge.start == edge.end { continue; }
        let key = if edge.start < edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        if !seen.insert(key) {
            continue;
        }
        let travel_dir = match edge.direction {
            AisleDirection::OneWay | AisleDirection::TwoWayOriented => {
                Some((graph.vertices[edge.end] - graph.vertices[edge.start]).normalize())
            }
            AisleDirection::TwoWay => None,
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
    let mut seen_edges: std::collections::HashSet<(usize, usize)> =
        std::collections::HashSet::new();

    let nv = graph.vertices.len();
    let mut adj: Vec<Vec<(Vec2, f64, bool)>> = vec![vec![]; nv];

    for edge in &graph.edges {
        let key = if edge.start < edge.end {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        };
        if !seen_edges.insert(key) {
            continue;
        }
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

    let max_width = graph.edges.iter().map(|e| e.width).fold(0.0f64, f64::max);
    let min_hole_area = max_width * max_width;

    result
        .into_iter()
        .filter(|shape| !shape.is_empty())
        .map(|shape| {
            shape
                .into_iter()
                .enumerate()
                .filter_map(|(i, pts)| {
                    let pts = if debug.spike_removal {
                        remove_contour_spikes(pts, 2.0)
                    } else {
                        pts
                    };
                    let pts = if debug.contour_simplification {
                        rdp_simplify_contour(pts, 2.0)
                    } else {
                        pts
                    };
                    if i > 0 && debug.hole_filtering && signed_area(&pts).abs() < min_hole_area {
                        return None;
                    }
                    Some(pts)
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
            if ab_len < tolerance || bc_len < tolerance {
                contour.remove(i);
                changed = true;
                break;
            }
            let ab_n = ab * (1.0 / ab_len);
            let bc_n = bc * (1.0 / bc_len);
            if ab_n.dot(bc_n) < -0.95 {
                if (c - a).length() < tolerance {
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
    }
    contour
}

/// Ramer-Douglas-Peucker simplification (via the `geo` crate).
fn rdp_simplify_contour(contour: Vec<Vec2>, epsilon: f64) -> Vec<Vec2> {
    if contour.len() < 4 {
        return contour;
    }
    let coords: Vec<Coord<f64>> = contour.iter().map(|v| Coord { x: v.x, y: v.y }).collect();
    let ring = LineString::new(coords);
    let poly = geo::Polygon::new(ring, vec![]);
    let simplified = poly.simplify(&epsilon);
    let ext = simplified.exterior();
    let n = ext.0.len().saturating_sub(1);
    ext.0[..n].iter().map(|c| Vec2::new(c.x, c.y)).collect()
}
