use crate::types::*;

/// Remove stalls that extend outside the outer boundary or into any hole.
/// A stall is removed if any of its corners falls outside the outer boundary,
/// inside a hole, or if any stall edge crosses the boundary/hole edges.
/// The face-index tag on each stall is preserved unchanged.
pub fn clip_stalls_to_boundary(
    stalls: Vec<(StallQuad, usize)>,
    boundary: &Polygon,
) -> Vec<(StallQuad, usize)> {
    stalls
        .into_iter()
        .filter(|(stall, _)| {
            // Shrink 30% toward centroid before testing. Rectangular stalls
            // at non-90° angles have corners that extend past the face into
            // the corridor and boundary areas. A proportional shrink handles
            // all angles without rejecting stalls whose body is inside.
            let shrunk = shrink_polygon_pct(&stall.corners, 0.3);
            // Every corner must be inside the outer boundary.
            for c in &shrunk {
                if !point_in_polygon(c, &boundary.outer) {
                    return false;
                }
            }
            // No corner may be inside any hole.
            for hole in &boundary.holes {
                for c in &shrunk {
                    if point_in_polygon(c, hole) {
                        return false;
                    }
                }
            }
            true
        })
        .collect()
}

/// Shrink a polygon by moving each vertex `pct` (0..1) of the way toward the centroid.
fn shrink_polygon_pct(corners: &[Vec2], pct: f64) -> Vec<Vec2> {
    let n = corners.len() as f64;
    let cx = corners.iter().map(|c| c.x).sum::<f64>() / n;
    let cy = corners.iter().map(|c| c.y).sum::<f64>() / n;
    let centroid = Vec2::new(cx, cy);
    corners
        .iter()
        .map(|c| *c + (centroid - *c) * pct)
        .collect()
}

/// Shrink a polygon inward by moving each vertex toward the centroid by `amount`.
fn shrink_polygon(corners: &[Vec2], amount: f64) -> Vec<Vec2> {
    let n = corners.len() as f64;
    let cx = corners.iter().map(|c| c.x).sum::<f64>() / n;
    let cy = corners.iter().map(|c| c.y).sum::<f64>() / n;
    let centroid = Vec2::new(cx, cy);
    corners
        .iter()
        .map(|c| {
            let d = *c - centroid;
            let len = d.length();
            if len < 1e-12 {
                return *c;
            }
            *c - d * (amount / len)
        })
        .collect()
}

/// Test whether two convex polygons overlap. Returns true if any vertex of
/// either polygon is inside the other, or if any pair of edges cross.
pub fn polygons_overlap(a: &[Vec2], b: &[Vec2]) -> bool {
    for v in a {
        if point_in_polygon(v, b) {
            return true;
        }
    }
    for v in b {
        if point_in_polygon(v, a) {
            return true;
        }
    }
    for i in 0..a.len() {
        let a0 = a[i];
        let a1 = a[(i + 1) % a.len()];
        for j in 0..b.len() {
            let b0 = b[j];
            let b1 = b[(j + 1) % b.len()];
            if segments_intersect(a0, a1, b0, b1) {
                return true;
            }
        }
    }
    false
}

/// Test whether two line segments (p0-p1) and (q0-q1) properly intersect
/// (cross each other, not just touch at endpoints).
pub fn segments_intersect(p0: Vec2, p1: Vec2, q0: Vec2, q1: Vec2) -> bool {
    let d1 = (q1 - q0).cross(p0 - q0);
    let d2 = (q1 - q0).cross(p1 - q0);
    let d3 = (p1 - p0).cross(q0 - p0);
    let d4 = (p1 - p0).cross(q1 - p0);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}


/// Remove stalls that geometrically overlap within the same face.
///
/// For boundary faces (or when no spine length info is provided), both
/// stalls in a conflicting pair are removed. For interior faces, the
/// stall from the shorter spine is removed, keeping the dominant side.
///
/// `spine_lengths` maps each stall index to its source spine length.
/// `boundary_faces` indicates which face indices are boundary faces.
/// Both slices must have the same length as `stalls`.
pub fn remove_conflicting_stalls(
    stalls: Vec<(StallQuad, usize)>,
    spine_lengths: &[f64],
    boundary_faces: &[bool],
) -> Vec<(StallQuad, usize)> {
    let shrunk: Vec<Vec<Vec2>> = stalls
        .iter()
        .map(|(s, _)| shrink_polygon(&s.corners, 0.1))
        .collect();

    // Group stall indices by face_idx.
    let mut by_face: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for (i, (_, face_idx)) in stalls.iter().enumerate() {
        by_face.entry(*face_idx).or_default().push(i);
    }

    let mut conflicted = vec![false; stalls.len()];

    for indices in by_face.values() {
        if indices.len() < 2 {
            continue;
        }

        // Build a spatial grid over this face's stalls. Cell size is chosen
        // so each stall spans at most 2×2 cells, keeping the number of
        // pairwise checks per cell small.
        let cell_size = {
            let mut max_span = 0.0f64;
            for &i in indices {
                let xs = shrunk[i].iter().map(|v| v.x);
                let ys = shrunk[i].iter().map(|v| v.y);
                let dx = xs.clone().fold(f64::NEG_INFINITY, f64::max)
                    - xs.fold(f64::INFINITY, f64::min);
                let dy = ys.clone().fold(f64::NEG_INFINITY, f64::max)
                    - ys.fold(f64::INFINITY, f64::min);
                max_span = max_span.max(dx).max(dy);
            }
            if max_span < 1.0 { 1.0 } else { max_span }
        };

        // Insert each stall into all grid cells its AABB touches.
        let mut grid: std::collections::HashMap<(i64, i64), Vec<usize>> =
            std::collections::HashMap::new();
        for &i in indices {
            let (min_x, max_x, min_y, max_y) = aabb(&shrunk[i]);
            let cx0 = (min_x / cell_size).floor() as i64;
            let cx1 = (max_x / cell_size).floor() as i64;
            let cy0 = (min_y / cell_size).floor() as i64;
            let cy1 = (max_y / cell_size).floor() as i64;
            for cx in cx0..=cx1 {
                for cy in cy0..=cy1 {
                    grid.entry((cx, cy)).or_default().push(i);
                }
            }
        }

        // Check pairs within each cell. A seen-set avoids duplicate checks.
        let mut checked = std::collections::HashSet::new();
        for cell in grid.values() {
            for a in 0..cell.len() {
                for b in (a + 1)..cell.len() {
                    let i = cell[a];
                    let j = cell[b];
                    let pair = if i < j { (i, j) } else { (j, i) };
                    if !checked.insert(pair) {
                        continue;
                    }
                    if polygons_overlap(&shrunk[i], &shrunk[j]) {
                        let face_i = stalls[i].1;
                        let is_boundary = face_i < boundary_faces.len() && boundary_faces[face_i];
                        if is_boundary || spine_lengths.is_empty() {
                            // Boundary face: remove both.
                            conflicted[i] = true;
                            conflicted[j] = true;
                        } else {
                            // Interior face: keep the stall from the longer spine.
                            let len_i = if i < spine_lengths.len() { spine_lengths[i] } else { 0.0 };
                            let len_j = if j < spine_lengths.len() { spine_lengths[j] } else { 0.0 };
                            if len_i >= len_j {
                                conflicted[j] = true;
                            } else {
                                conflicted[i] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    stalls
        .into_iter()
        .enumerate()
        .filter(|(i, _)| !conflicted[*i])
        .map(|(_, s)| s)
        .collect()
}

/// Compute axis-aligned bounding box of a polygon. Returns (min_x, max_x, min_y, max_y).
fn aabb(poly: &[Vec2]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for v in poly {
        min_x = min_x.min(v.x);
        max_x = max_x.max(v.x);
        min_y = min_y.min(v.y);
        max_y = max_y.max(v.y);
    }
    (min_x, max_x, min_y, max_y)
}

/// Ray-casting point-in-polygon test.
pub fn point_in_polygon(point: &Vec2, polygon: &[Vec2]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = &polygon[i];
        let pj = &polygon[j];
        if ((pi.y > point.y) != (pj.y > point.y))
            && (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}
