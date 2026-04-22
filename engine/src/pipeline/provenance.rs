//! Recover face-edge provenance by matching each output edge back to the
//! input segment it descends from (DESIGN.md §3.4, folded into construction).
//!
//! The boolean union that produces merged aisle polygons inserts a vertex at
//! every intersection, so each edge in the output is a subsegment of exactly
//! one input segment. If we tag input segments at construction — corridor
//! sides as `Aisle`, boundary/hole sides as `Wall` — we can recover each
//! face-edge's source by a collinear-containment lookup, instead of the
//! point-sampling + binary-search dance in `face::tag_face_edges`.
//!
//! Tolerances are generous (0.5 units) because i_overlay snaps and the
//! boundary-minus-aisles subtraction can shift coordinates slightly.

use crate::types::*;

/// An input segment with its source. Feed these into the index for every
/// segment we hand to the boolean-op layer (corridor sides, miter sides,
/// boundary/hole sides).
#[derive(Clone, Debug)]
pub struct TaggedSegment {
    pub a: Vec2,
    pub b: Vec2,
    pub source: EdgeSource,
}

pub struct EdgeTagIndex {
    segments: Vec<TaggedSegment>,
    eps: f64,
}

impl EdgeTagIndex {
    pub fn new(segments: Vec<TaggedSegment>, eps: f64) -> Self {
        let segments = segments
            .into_iter()
            .filter(|s| (s.b - s.a).length() > 1e-9)
            .collect();
        Self { segments, eps }
    }

    /// Return the source of the input segment best covering the face-edge
    /// `(p, q)` collinearly.
    ///
    /// The query must be covered by same-source segments along one line
    /// for **most** of its length (`MIN_COVER_FRACTION`). This keeps
    /// face edges that straddle a corridor side and its collinear miter
    /// extension from being misclassified as Wall while rejecting loose
    /// overlap against a faraway segment. Segments parallel but offset
    /// beyond `eps` are ignored.
    pub fn tag(&self, p: Vec2, q: Vec2) -> Option<EdgeSource> {
        const MIN_COVER_FRACTION: f64 = 0.75;
        let eps = self.eps;
        let qd = q - p;
        let qlen = qd.length();
        if qlen < 1e-12 {
            return None;
        }
        let qdir = Vec2::new(qd.x / qlen, qd.y / qlen);

        // Per-source projection intervals onto the query line, for
        // segments collinear with the query within `eps`.
        let mut intervals: Vec<(EdgeSource, f64, f64)> = Vec::new();
        for s in &self.segments {
            let sd = s.b - s.a;
            let slen = sd.length();
            if slen < 1e-9 {
                continue;
            }
            let sdir = Vec2::new(sd.x / slen, sd.y / slen);
            let snormal = Vec2::new(-sdir.y, sdir.x);
            let perp_p = (p - s.a).dot(snormal).abs();
            let perp_q = (q - s.a).dot(snormal).abs();
            if perp_p > eps || perp_q > eps {
                continue;
            }
            let t_a = (s.a - p).dot(qdir);
            let t_b = (s.b - p).dot(qdir);
            let (lo, hi) = if t_a < t_b { (t_a, t_b) } else { (t_b, t_a) };
            let ov_lo = lo.max(0.0);
            let ov_hi = hi.min(qlen);
            if ov_hi - ov_lo < eps {
                continue;
            }
            intervals.push((s.source.clone(), ov_lo, ov_hi));
        }

        // Group intervals by source, merge overlaps, score by coverage of
        // the query extent. Sources share identity via their corridor_idx
        // (or the unit Wall variant).
        let source_key = |src: &EdgeSource| -> i64 {
            match src {
                EdgeSource::Wall => -1,
                EdgeSource::Aisle { corridor_idx, .. } => *corridor_idx as i64,
            }
        };
        let mut keys: Vec<i64> = intervals.iter().map(|(s, _, _)| source_key(s)).collect();
        keys.sort();
        keys.dedup();

        let mut best: Option<(f64, EdgeSource)> = None;
        for key in keys {
            let mut per_src: Vec<(f64, f64)> = intervals
                .iter()
                .filter(|(s, _, _)| source_key(s) == key)
                .map(|(_, a, b)| (*a, *b))
                .collect();
            per_src.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            let mut coverage = 0.0;
            let mut cur: Option<(f64, f64)> = None;
            for (a, b) in per_src {
                match cur {
                    None => cur = Some((a, b)),
                    Some((ca, cb)) => {
                        if a <= cb + eps {
                            cur = Some((ca, cb.max(b)));
                        } else {
                            coverage += cb - ca;
                            cur = Some((a, b));
                        }
                    }
                }
            }
            if let Some((ca, cb)) = cur {
                coverage += cb - ca;
            }
            if coverage < qlen * MIN_COVER_FRACTION {
                continue;
            }
            let source = intervals
                .iter()
                .find(|(s, _, _)| source_key(s) == key)
                .map(|(s, _, _)| s.clone())
                .unwrap();
            if best.as_ref().map_or(true, |(c, _)| coverage > *c) {
                best = Some((coverage, source));
            }
        }
        best.map(|(_, s)| s)
    }

    /// Build an index over all input segments fed to the boolean union
    /// that produces the merged aisle polygons: corridor rectangle sides
    /// (4 per corridor edge) and miter wedge sides at junctions. Each
    /// segment carries the metadata of the corridor it descends from, so
    /// face edges that ride miter extensions still tag as Aisle.
    pub fn for_aisle_layout(
        graph: &DriveAisleGraph,
        per_edge_aisles: &[(Vec<Vec2>, bool, Option<Vec2>)],
        two_way_oriented_dirs: &[Option<Vec2>],
        debug: &DebugToggles,
        eps: f64,
    ) -> Self {
        let source_for = |idx: usize| -> EdgeSource {
            let (_, interior, travel_dir) = &per_edge_aisles[idx];
            let is_two_way_oriented =
                two_way_oriented_dirs.get(idx).and_then(|v| *v).is_some();
            EdgeSource::Aisle {
                corridor_idx: idx,
                interior: *interior,
                travel_dir: *travel_dir,
                is_two_way_oriented,
            }
        };

        let mut segs: Vec<TaggedSegment> = Vec::new();

        // Corridor rectangle sides.
        for (idx, (poly, _, _)) in per_edge_aisles.iter().enumerate() {
            let src = source_for(idx);
            let n = poly.len();
            for i in 0..n {
                let j = (i + 1) % n;
                segs.push(TaggedSegment { a: poly[i], b: poly[j], source: src.clone() });
            }
        }

        // Miter wedge sides — mirrors `aisle_polygon::generate_miter_fills`
        // but threads the originating corridor index through each wedge
        // edge. Wedge corners are [v, p1, miter, p2]: sides v→p1 and
        // p1→miter inherit corridor i; miter→p2 and p2→v inherit corridor j.
        if debug.miter_fills {
            let nv = graph.vertices.len();
            let mut adj: Vec<Vec<(Vec2, f64, bool, usize)>> = vec![vec![]; nv];
            let mut seen = std::collections::HashSet::new();
            let mut idx = 0usize;
            for edge in &graph.edges {
                if edge.start == edge.end { continue; }
                let key = if edge.start < edge.end {
                    (edge.start, edge.end)
                } else {
                    (edge.end, edge.start)
                };
                if !seen.insert(key) { continue; }
                let s = graph.vertices[edge.start];
                let e = graph.vertices[edge.end];
                if (e - s).length() < 1e-9 { idx += 1; continue; }
                let dir = (e - s).normalize();
                let w = edge.width;
                adj[edge.start].push((dir, w, edge.interior, idx));
                adj[edge.end].push((Vec2::new(-dir.x, -dir.y), w, edge.interior, idx));
                idx += 1;
            }

            for vi in 0..nv {
                if adj[vi].len() < 2 { continue; }
                if debug.boundary_only_miters
                    && adj[vi].iter().all(|(_, _, int, _)| *int)
                {
                    continue;
                }
                let v = graph.vertices[vi];
                let mut sorted: Vec<(f64, Vec2, f64, usize)> = adj[vi]
                    .iter()
                    .map(|(d, w, _, ci)| (d.y.atan2(d.x), *d, *w, *ci))
                    .collect();
                sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
                let ne = sorted.len();
                for i in 0..ne {
                    let j = (i + 1) % ne;
                    let (a1, d1, w1, ci1) = sorted[i];
                    let (a2, d2, w2, ci2) = sorted[j];
                    let n1 = Vec2::new(-d1.y, d1.x);
                    let n2 = Vec2::new(-d2.y, d2.x);
                    let p1 = v + n1 * w1;
                    let p2 = v - n2 * w2;
                    let denom = d1.cross(d2);
                    if denom.abs() < 1e-12 { continue; }
                    let t = (p2 - p1).cross(d2) / denom;
                    let miter = p1 + d1 * t;

                    let gap = if j > i {
                        a2 - a1
                    } else {
                        (a2 + std::f64::consts::TAU) - a1
                    };
                    if gap < std::f64::consts::PI {
                        let max_w = w1.max(w2);
                        if (miter - v).length() > max_w * 4.0 { continue; }
                    }

                    let s1 = source_for(ci1);
                    let s2 = source_for(ci2);
                    segs.push(TaggedSegment { a: v, b: p1, source: s1.clone() });
                    segs.push(TaggedSegment { a: p1, b: miter, source: s1 });
                    segs.push(TaggedSegment { a: miter, b: p2, source: s2.clone() });
                    segs.push(TaggedSegment { a: p2, b: v, source: s2 });
                }
            }
        }

        Self::new(segs, eps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vec2 { Vec2::new(x, y) }

    fn aisle(idx: usize) -> EdgeSource {
        EdgeSource::Aisle {
            corridor_idx: idx,
            interior: true,
            travel_dir: None,
            is_two_way_oriented: false,
        }
    }

    fn is_wall(s: &EdgeSource) -> bool {
        matches!(s, EdgeSource::Wall)
    }

    fn aisle_idx(s: &EdgeSource) -> Option<usize> {
        match s {
            EdgeSource::Aisle { corridor_idx, .. } => Some(*corridor_idx),
            _ => None,
        }
    }

    #[test]
    fn tags_pure_wall_segment() {
        let idx = EdgeTagIndex::new(
            vec![TaggedSegment { a: v(0.0, 0.0), b: v(100.0, 0.0), source: EdgeSource::Wall }],
            0.5,
        );
        let tag = idx.tag(v(10.0, 0.0), v(40.0, 0.0)).expect("should tag");
        assert!(is_wall(&tag));
    }

    #[test]
    fn tags_pure_aisle_segment_with_corridor_idx() {
        let idx = EdgeTagIndex::new(
            vec![TaggedSegment { a: v(0.0, 10.0), b: v(100.0, 10.0), source: aisle(7) }],
            0.5,
        );
        let tag = idx.tag(v(20.0, 10.0), v(80.0, 10.0)).expect("should tag");
        assert_eq!(aisle_idx(&tag), Some(7));
    }

    #[test]
    fn splits_at_wall_aisle_junction() {
        // A face-edge that used to straddle a wall→aisle transition now
        // arrives pre-split: i_overlay inserts a vertex at the intersection
        // (x=50). Each sub-edge tags distinctly.
        let segs = vec![
            TaggedSegment { a: v(0.0, 0.0), b: v(50.0, 0.0), source: EdgeSource::Wall },
            TaggedSegment { a: v(50.0, 0.0), b: v(100.0, 0.0), source: aisle(3) },
        ];
        let idx = EdgeTagIndex::new(segs, 0.5);
        assert!(is_wall(&idx.tag(v(10.0, 0.0), v(40.0, 0.0)).unwrap()));
        assert_eq!(aisle_idx(&idx.tag(v(60.0, 0.0), v(90.0, 0.0)).unwrap()), Some(3));
    }

    #[test]
    fn rejects_non_collinear_query() {
        let idx = EdgeTagIndex::new(
            vec![TaggedSegment { a: v(0.0, 0.0), b: v(100.0, 0.0), source: EdgeSource::Wall }],
            0.5,
        );
        // Parallel but offset by 5 units — outside eps.
        assert!(idx.tag(v(10.0, 5.0), v(40.0, 5.0)).is_none());
    }

    #[test]
    fn rejects_query_outside_segment_extent() {
        let idx = EdgeTagIndex::new(
            vec![TaggedSegment { a: v(0.0, 0.0), b: v(100.0, 0.0), source: EdgeSource::Wall }],
            0.5,
        );
        // Collinear but past the segment end.
        assert!(idx.tag(v(110.0, 0.0), v(140.0, 0.0)).is_none());
    }

    #[test]
    fn tolerates_snap_within_eps() {
        // i_overlay may snap coordinates up to ~0.5 units. A query that's
        // slightly off the ideal line should still tag.
        let idx = EdgeTagIndex::new(
            vec![TaggedSegment { a: v(0.0, 0.0), b: v(100.0, 0.0), source: aisle(1) }],
            0.5,
        );
        let tag = idx.tag(v(10.0, 0.3), v(40.0, -0.2)).expect("should tag within eps");
        assert_eq!(aisle_idx(&tag), Some(1));
    }

    #[test]
    fn picks_closest_when_multiple_candidates_overlap() {
        // Two collinear sources sharing an overlap range — picks the one
        // with tighter perpendicular fit. This matters when a miter wedge
        // shares a line with a corridor side: corridor side is the true
        // source.
        let segs = vec![
            TaggedSegment { a: v(0.0, 0.4), b: v(100.0, 0.4), source: aisle(9) },
            TaggedSegment { a: v(0.0, 0.0), b: v(100.0, 0.0), source: aisle(2) },
        ];
        let idx = EdgeTagIndex::new(segs, 0.5);
        let tag = idx.tag(v(20.0, 0.0), v(80.0, 0.0)).expect("should tag");
        assert_eq!(aisle_idx(&tag), Some(2));
    }

    #[test]
    fn tags_query_spanning_collinear_same_source_segments() {
        // A corridor side and its miter-wedge extensions form collinear
        // touching segments on the same line. The boolean union absorbs
        // the seam, so face edges can span the joint.
        let segs = vec![
            TaggedSegment { a: v(-12.0, 0.0), b: v(0.0, 0.0), source: aisle(4) }, // miter ext
            TaggedSegment { a: v(0.0, 0.0), b: v(200.0, 0.0), source: aisle(4) }, // corridor side
            TaggedSegment { a: v(200.0, 0.0), b: v(212.0, 0.0), source: aisle(4) }, // miter ext
        ];
        let idx = EdgeTagIndex::new(segs, 0.5);
        // Face edge spanning the full run.
        let tag = idx.tag(v(-12.0, 0.0), v(212.0, 0.0)).expect("should tag");
        assert_eq!(aisle_idx(&tag), Some(4));
        // Face edge wholly inside one miter extension.
        let tag = idx.tag(v(-10.0, 0.0), v(-2.0, 0.0)).expect("should tag");
        assert_eq!(aisle_idx(&tag), Some(4));
    }

    #[test]
    fn tags_reversed_query_orientation() {
        // Face contour winding may traverse an input segment in reverse.
        let idx = EdgeTagIndex::new(
            vec![TaggedSegment { a: v(0.0, 0.0), b: v(100.0, 0.0), source: aisle(5) }],
            0.5,
        );
        let tag = idx.tag(v(80.0, 0.0), v(20.0, 0.0)).expect("should tag");
        assert_eq!(aisle_idx(&tag), Some(5));
    }
}
