//! Planar-arrangement face enumeration for regions.
//!
//! Treats the outer boundary, hole boundaries, and partitioning drive
//! lines as a single planar graph; builds a DCEL; walks faces; returns
//! each bounded interior face as a region with provenance.
//!
//! A partitioning line that dangles (one endpoint strictly interior to
//! a face) is inert — its half-edges form a degenerate bridge in the
//! enclosing face's boundary cycle but don't split it.
//!
//! Region IDs are derived from the canonicalized cyclic sequence of
//! edge kinds bounding each face. The sequence is deduped for
//! consecutive identical kinds (an outer edge split by a partition
//! still contributes one `Outer(i)` entry on each side) and then
//! rotated so the lex-smallest element leads. This yields IDs stable
//! under unrelated geometry edits.

use crate::types::{RegionId, Vec2};

const EPS: f64 = 1e-6;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EdgeKind {
    Outer(u32),
    Hole(u32, u32),
    Partition(u32),
}

#[derive(Clone, Debug)]
pub struct PartitionedRegion {
    pub id: RegionId,
    pub clip_poly: Vec<Vec2>,
    pub boundary_kinds: Vec<EdgeKind>,
}

pub fn enumerate_regions(
    outer: &[Vec2],
    holes: &[&[Vec2]],
    partitioning_lines: &[(u32, Vec2, Vec2)],
) -> Vec<PartitionedRegion> {
    // 1. Collect source segments with provenance. Orient outer CCW and
    //    each hole CW so that the "forward" half-edge of every boundary
    //    segment has the lot interior on its left.
    let outer_ccw = ensure_orientation(outer, true);
    let segments = collect_segments(&outer_ccw, holes, partitioning_lines);

    // 2. Intersect all pairs, split segments at every crossing and
    //    endpoint, produce sub-segments.
    let subs = split_segments(&segments);

    // 3. Dedup vertices, build half-edges, link next via angular sort.
    let mut arr = Arrangement::new();
    for ss in &subs {
        arr.add_sub(ss);
    }
    arr.link_faces();

    // 4. Walk faces, keep interior-bounded ones, extract region data.
    arr.extract_regions()
}

// ---------------------------------------------------------------------------
// Segment collection
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct Segment {
    p0: Vec2,
    p1: Vec2,
    kind: EdgeKind,
    /// When true, the forward half-edge (p0 → p1) has the lot interior
    /// on its left; the reverse has exterior/hole-interior. When false
    /// (partition lines), both sides are lot interior.
    boundary: bool,
}

fn collect_segments(
    outer_ccw: &[Vec2],
    holes: &[&[Vec2]],
    partitions: &[(u32, Vec2, Vec2)],
) -> Vec<Segment> {
    let mut out = Vec::new();
    let n = outer_ccw.len();
    for i in 0..n {
        out.push(Segment {
            p0: outer_ccw[i],
            p1: outer_ccw[(i + 1) % n],
            kind: EdgeKind::Outer(i as u32),
            boundary: true,
        });
    }
    for (h, hole) in holes.iter().enumerate() {
        if hole.len() < 3 {
            continue;
        }
        let hole_cw = ensure_orientation(hole, false);
        let m = hole_cw.len();
        for i in 0..m {
            out.push(Segment {
                p0: hole_cw[i],
                p1: hole_cw[(i + 1) % m],
                kind: EdgeKind::Hole(h as u32, i as u32),
                boundary: true,
            });
        }
    }
    for &(id, p0, p1) in partitions {
        if (p1 - p0).length() < EPS {
            continue;
        }
        out.push(Segment {
            p0,
            p1,
            kind: EdgeKind::Partition(id),
            boundary: false,
        });
    }
    out
}

fn ensure_orientation(poly: &[Vec2], ccw: bool) -> Vec<Vec2> {
    let mut v = poly.to_vec();
    let area = signed_area(&v);
    if (ccw && area < 0.0) || (!ccw && area > 0.0) {
        v.reverse();
    }
    v
}

fn signed_area(poly: &[Vec2]) -> f64 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..n {
        let p = poly[i];
        let q = poly[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    a * 0.5
}

// ---------------------------------------------------------------------------
// Segment splitting via pairwise intersection
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct SubSegment {
    p0: Vec2,
    p1: Vec2,
    kind: EdgeKind,
    boundary: bool,
}

fn split_segments(segs: &[Segment]) -> Vec<SubSegment> {
    // For each segment, collect parametric splits in (0, 1) from
    // intersections with other segments.
    let n = segs.len();
    let mut splits: Vec<Vec<f64>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in (i + 1)..n {
            if let Some((ti, tj)) = segment_intersection(&segs[i], &segs[j]) {
                if ti > EPS && ti < 1.0 - EPS {
                    splits[i].push(ti);
                }
                if tj > EPS && tj < 1.0 - EPS {
                    splits[j].push(tj);
                }
            }
        }
    }
    let mut out = Vec::new();
    for (i, seg) in segs.iter().enumerate() {
        let mut ts = splits[i].clone();
        ts.push(0.0);
        ts.push(1.0);
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ts.dedup_by(|a, b| (*a - *b).abs() < EPS);
        for w in ts.windows(2) {
            let (a, b) = (w[0], w[1]);
            if b - a < EPS {
                continue;
            }
            let p0 = seg.p0 + (seg.p1 - seg.p0) * a;
            let p1 = seg.p0 + (seg.p1 - seg.p0) * b;
            out.push(SubSegment {
                p0,
                p1,
                kind: seg.kind,
                boundary: seg.boundary,
            });
        }
    }
    out
}

/// Returns (ti, tj) such that seg_i(ti) == seg_j(tj) for a proper crossing
/// or a T-junction (endpoint of one on interior of other). Collinear
/// overlaps are not reported — they would produce ambiguous faces.
fn segment_intersection(a: &Segment, b: &Segment) -> Option<(f64, f64)> {
    let r = a.p1 - a.p0;
    let s = b.p1 - b.p0;
    let denom = r.x * s.y - r.y * s.x;
    let qp = b.p0 - a.p0;
    if denom.abs() < 1e-12 {
        return None; // parallel / collinear
    }
    let t = (qp.x * s.y - qp.y * s.x) / denom;
    let u = (qp.x * r.y - qp.y * r.x) / denom;
    if t < -EPS || t > 1.0 + EPS || u < -EPS || u > 1.0 + EPS {
        return None;
    }
    Some((t.clamp(0.0, 1.0), u.clamp(0.0, 1.0)))
}

// ---------------------------------------------------------------------------
// Arrangement / DCEL
// ---------------------------------------------------------------------------

struct Arrangement {
    vertices: Vec<Vec2>,
    /// Half-edges; each pair `(2k, 2k+1)` are twins.
    origin: Vec<usize>,
    kind: Vec<EdgeKind>,
    /// Whether this half-edge's LEFT face is lot interior.
    interior_left: Vec<bool>,
    next: Vec<usize>,
}

impl Arrangement {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            origin: Vec::new(),
            kind: Vec::new(),
            interior_left: Vec::new(),
            next: Vec::new(),
        }
    }

    fn add_vertex(&mut self, p: Vec2) -> usize {
        for (i, v) in self.vertices.iter().enumerate() {
            if (*v - p).length() < EPS * 10.0 {
                return i;
            }
        }
        self.vertices.push(p);
        self.vertices.len() - 1
    }

    fn add_sub(&mut self, ss: &SubSegment) {
        let vi = self.add_vertex(ss.p0);
        let vj = self.add_vertex(ss.p1);
        if vi == vj {
            return;
        }
        // Forward half-edge (vi → vj)
        self.origin.push(vi);
        self.kind.push(ss.kind);
        // For boundary segments, interior is on the left of forward.
        // For partitions (non-boundary), both sides are interior.
        self.interior_left.push(if ss.boundary { true } else { true });
        self.next.push(usize::MAX);
        // Reverse half-edge (vj → vi)
        self.origin.push(vj);
        self.kind.push(ss.kind);
        self.interior_left.push(if ss.boundary { false } else { true });
        self.next.push(usize::MAX);
    }

    fn twin(&self, h: usize) -> usize {
        h ^ 1
    }

    fn destination(&self, h: usize) -> usize {
        self.origin[self.twin(h)]
    }

    fn direction(&self, h: usize) -> Vec2 {
        let o = self.vertices[self.origin[h]];
        let d = self.vertices[self.destination(h)];
        d - o
    }

    fn link_faces(&mut self) {
        // For each vertex, sort outgoing half-edges by angle CCW; link
        // next via o_i.twin.next = o_{(i+1) mod k}.
        let n = self.vertices.len();
        let mut out_by_vertex: Vec<Vec<usize>> = vec![Vec::new(); n];
        for h in 0..self.origin.len() {
            out_by_vertex[self.origin[h]].push(h);
        }
        for v in 0..n {
            let outs = &mut out_by_vertex[v];
            outs.sort_by(|a, b| {
                let da = self.direction(*a);
                let db = self.direction(*b);
                let aa = da.y.atan2(da.x);
                let bb = db.y.atan2(db.x);
                aa.partial_cmp(&bb).unwrap()
            });
            // For each outgoing o_i at v, its arriving twin's .next is
            // the CCW-predecessor outgoing: this keeps the face walk on
            // the LEFT of every half-edge.
            let k = outs.len();
            for i in 0..k {
                let o_i = outs[i];
                let o_prev = outs[(i + k - 1) % k];
                let t = self.twin(o_i);
                self.next[t] = o_prev;
            }
        }
    }

    fn extract_regions(&self) -> Vec<PartitionedRegion> {
        let mut visited = vec![false; self.origin.len()];
        let mut regions = Vec::new();
        for start in 0..self.origin.len() {
            if visited[start] {
                continue;
            }
            // Only consider half-edges whose LEFT face is lot interior.
            if !self.interior_left[start] {
                visited[start] = true;
                continue;
            }
            // Walk the cycle.
            let mut cycle = Vec::new();
            let mut h = start;
            loop {
                if visited[h] {
                    break;
                }
                visited[h] = true;
                cycle.push(h);
                h = self.next[h];
                if h == usize::MAX {
                    cycle.clear();
                    break;
                }
                if h == start {
                    break;
                }
            }
            if cycle.is_empty() {
                continue;
            }
            // Build polygon.
            let poly: Vec<Vec2> = cycle
                .iter()
                .map(|&he| self.vertices[self.origin[he]])
                .collect();
            // Reject unbounded / exterior cycles by requiring positive
            // signed area. A cycle walking the exterior has negative
            // area (it goes CW around the infinite face). Hole interior
            // cycles are filtered out by the `interior_left` check
            // above; this catches any remaining degenerate cycles.
            if signed_area(&poly) <= EPS {
                continue;
            }
            // Extract edge-kind sequence, dedup consecutive identical.
            let mut kinds: Vec<EdgeKind> = cycle.iter().map(|&he| self.kind[he]).collect();
            kinds.dedup();
            // Handle wrap-around dedup.
            if kinds.len() > 1 && kinds.first() == kinds.last() {
                kinds.pop();
            }
            let id = region_id_from_kinds(&kinds);
            regions.push(PartitionedRegion {
                id,
                clip_poly: poly,
                boundary_kinds: kinds,
            });
        }
        regions
    }
}

fn region_id_from_kinds(kinds: &[EdgeKind]) -> RegionId {
    if kinds.is_empty() {
        return RegionId::single_region_fallback();
    }
    // Canonicalize: rotate so the lex-smallest position leads.
    let n = kinds.len();
    let mut best = 0usize;
    for i in 1..n {
        for j in 0..n {
            let a = &kinds[(best + j) % n];
            let b = &kinds[(i + j) % n];
            if a == b {
                continue;
            }
            if b < a {
                best = i;
            }
            break;
        }
    }
    // Hash FNV-1a over the canonical sequence.
    let mut h: u64 = 0xcbf29ce484222325;
    for j in 0..n {
        let k = &kinds[(best + j) % n];
        let code: u64 = match *k {
            EdgeKind::Outer(i) => (1 << 32) | i as u64,
            EdgeKind::Hole(h_, e) => (2u64 << 40) | ((h_ as u64) << 20) | (e as u64),
            EdgeKind::Partition(p) => (3u64 << 32) | p as u64,
        };
        for b in code.to_le_bytes() {
            h ^= b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
    }
    // Keep inside JS safe-integer range (53 bits), avoid the fallback
    // sentinel bit (bit 48).
    RegionId(h & 0x0000_FFFF_FFFF_FFFF)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(w: f64, h: f64) -> Vec<Vec2> {
        vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(w, 0.0),
            Vec2::new(w, h),
            Vec2::new(0.0, h),
        ]
    }

    #[test]
    fn outer_only_single_region() {
        let outer = rect(10.0, 10.0);
        let regions = enumerate_regions(&outer, &[], &[]);
        assert_eq!(regions.len(), 1);
        assert!((signed_area(&regions[0].clip_poly) - 100.0).abs() < 1e-6);
    }

    #[test]
    fn outer_with_hole_single_region() {
        let outer = rect(10.0, 10.0);
        let hole = vec![
            Vec2::new(3.0, 3.0),
            Vec2::new(7.0, 3.0),
            Vec2::new(7.0, 7.0),
            Vec2::new(3.0, 7.0),
        ];
        let regions = enumerate_regions(&outer, &[&hole], &[]);
        // Outer + hole → one annular face.
        assert_eq!(regions.len(), 1);
    }

    #[test]
    fn chord_splits_into_two_regions() {
        let outer = rect(10.0, 10.0);
        // Horizontal chord from (0, 5) to (10, 5).
        let parts = vec![(42u32, Vec2::new(0.0, 5.0), Vec2::new(10.0, 5.0))];
        let regions = enumerate_regions(&outer, &[], &parts);
        assert_eq!(regions.len(), 2);
        // Both regions should have area ≈ 50.
        for r in &regions {
            assert!((signed_area(&r.clip_poly) - 50.0).abs() < 1e-3);
        }
        // Distinct stable IDs.
        assert_ne!(regions[0].id, regions[1].id);
    }

    #[test]
    fn dangling_partition_does_not_split() {
        let outer = rect(10.0, 10.0);
        // Partition from (0,5) to (5,5) — endpoint (5,5) is interior.
        let parts = vec![(7u32, Vec2::new(0.0, 5.0), Vec2::new(5.0, 5.0))];
        let regions = enumerate_regions(&outer, &[], &parts);
        assert_eq!(regions.len(), 1, "dangling partition must not split");
    }

    #[test]
    fn two_chords_three_regions() {
        let outer = rect(10.0, 10.0);
        let parts = vec![
            (1u32, Vec2::new(3.0, 0.0), Vec2::new(3.0, 10.0)),
            (2u32, Vec2::new(7.0, 0.0), Vec2::new(7.0, 10.0)),
        ];
        let regions = enumerate_regions(&outer, &[], &parts);
        assert_eq!(regions.len(), 3);
    }

    #[test]
    fn region_ids_stable_across_runs() {
        let outer = rect(10.0, 10.0);
        let parts = vec![(42u32, Vec2::new(0.0, 5.0), Vec2::new(10.0, 5.0))];
        let r1 = enumerate_regions(&outer, &[], &parts);
        let r2 = enumerate_regions(&outer, &[], &parts);
        let mut ids1: Vec<_> = r1.iter().map(|r| r.id).collect();
        let mut ids2: Vec<_> = r2.iter().map(|r| r.id).collect();
        ids1.sort_by_key(|r| r.0);
        ids2.sort_by_key(|r| r.0);
        assert_eq!(ids1, ids2);
    }
}
