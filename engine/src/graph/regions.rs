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
//! edge kinds bounding each face. Boundary edges are keyed by the
//! `VertexId` of the segment's start vertex — stable across vertex
//! insertions, deletions, and reorderings elsewhere on the loop —
//! rather than by positional segment index, which renumbered every
//! later edge whenever any vertex was added or removed. The sequence
//! is deduped for consecutive identical kinds (an outer edge split
//! by a partition still contributes one `Outer(id)` entry on each
//! side) and then rotated so the lex-smallest element leads.

use crate::geom::poly::signed_area;
use crate::types::{RegionId, Vec2, VertexId};

const EPS: f64 = 1e-6;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EdgeKind {
    /// Outer-loop edge keyed by the `VertexId` of its start vertex
    /// (in CCW orientation). Stable across edits elsewhere on the
    /// loop. Synthetic chord vertices (post-arc-discretization) carry
    /// synthetic ids; whether those persist across regenerations is
    /// the caller's responsibility.
    Outer(VertexId),
    /// Hole-loop edge keyed by `(hole_index, start_vertex_id)` in CW
    /// orientation. The hole index is positional (matches
    /// `boundary.holes[i]`), consistent with `PerimeterLoop::Hole`.
    Hole(u32, VertexId),
    /// Partition (drive-line) edge keyed by the drive line's stable id.
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
    outer_ids: &[VertexId],
    holes: &[(&[Vec2], &[VertexId])],
    partitioning_lines: &[(u32, Vec2, Vec2)],
) -> Vec<PartitionedRegion> {
    // 1. Collect source segments with provenance. Orient outer CCW and
    //    each hole CW so that the "forward" half-edge of every boundary
    //    segment has the lot interior on its left. Parallel ids are
    //    reversed alongside if the orientation flips.
    let (outer_ccw, outer_ids_ccw) = orient_with_ids(outer, outer_ids, true);
    let segments = collect_segments(&outer_ccw, &outer_ids_ccw, holes, partitioning_lines);

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
    outer_ids_ccw: &[VertexId],
    holes: &[(&[Vec2], &[VertexId])],
    partitions: &[(u32, Vec2, Vec2)],
) -> Vec<Segment> {
    let mut out = Vec::new();
    let n = outer_ccw.len();
    // Map each segment's start vertex to its *upstream sketch corner*
    // (most recent non-synthetic id at or before this position around
    // the loop). Multiple chord segments along one arc all share the
    // same anchor — region-id dedup then collapses them into a single
    // EdgeKind entry, making the region id invariant to chord count
    // (which fluctuates with bulge magnitude vs discretize tolerance).
    let outer_anchors = ancestors_for_loop(outer_ids_ccw, n);
    for i in 0..n {
        out.push(Segment {
            p0: outer_ccw[i],
            p1: outer_ccw[(i + 1) % n],
            kind: EdgeKind::Outer(outer_anchors[i]),
            boundary: true,
        });
    }
    for (h, (hole, hole_ids)) in holes.iter().enumerate() {
        if hole.len() < 3 {
            continue;
        }
        let (hole_cw, hole_ids_cw) = orient_with_ids(hole, hole_ids, false);
        let m = hole_cw.len();
        let hole_anchors = ancestors_for_loop(&hole_ids_cw, m);
        for i in 0..m {
            out.push(Segment {
                p0: hole_cw[i],
                p1: hole_cw[(i + 1) % m],
                kind: EdgeKind::Hole(h as u32, hole_anchors[i]),
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

/// For each position `i` in a loop of `length` vertices, return the
/// most-recent non-synthetic `VertexId` at or before `i` walking
/// forward through the loop with wrap-around. Lets boundary edges
/// produced by arc discretization (interior chord samples carrying
/// synthetic ids) inherit their parent sketch corner as a stable
/// anchor.
///
/// If `ids.len() != length` the input is treated as missing — the
/// returned vec is filled with synthetic positional ids so collect
/// still produces unique-but-unstable EdgeKinds (matching pre-fix
/// behavior).
///
/// If every id in the loop is synthetic, the function returns the ids
/// as-is — the loop has no sketch anchor to attach to.
fn ancestors_for_loop(ids: &[VertexId], length: usize) -> Vec<VertexId> {
    if ids.len() != length {
        return (0..length as u32)
            .map(|i| VertexId(VertexId::SYNTHETIC_BASE | (i & !VertexId::SYNTHETIC_BASE)))
            .collect();
    }
    let Some(start_idx) = ids.iter().position(|id| !id.is_synthetic()) else {
        return ids.to_vec();
    };
    let mut out = vec![VertexId(0); length];
    let mut current = ids[start_idx];
    for k in 0..length {
        let i = (start_idx + k) % length;
        if !ids[i].is_synthetic() {
            current = ids[i];
        }
        out[i] = current;
    }
    out
}

/// `ensure_orientation` paired with a parallel `VertexId` array. Reverses
/// the ids alongside the polygon so `ids[i]` remains the start vertex of
/// segment `i` after orientation. If `ids.len()` doesn't match `poly.len()`
/// the ids array is returned empty (caller falls back to synthetic ids).
fn orient_with_ids(
    poly: &[Vec2],
    ids: &[VertexId],
    ccw: bool,
) -> (Vec<Vec2>, Vec<VertexId>) {
    let mut v = poly.to_vec();
    let area = signed_area(&v);
    let must_reverse = (ccw && area < 0.0) || (!ccw && area > 0.0);
    let mut ids_out: Vec<VertexId> =
        if ids.len() == poly.len() { ids.to_vec() } else { Vec::new() };
    if must_reverse {
        v.reverse();
        if !ids_out.is_empty() {
            // After reversing both arrays, segment `i` in the reversed
            // polygon goes from v[i] to v[i+1] — which is the *reverse*
            // of the original edge ending at v[i]. The start vertex of
            // segment i in the reversed poly is the original vertex
            // n-1-i, whose id sits at ids_reversed[i] after reverse.
            ids_out.reverse();
        }
    }
    (v, ids_out)
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
        // Forward half-edge (vi → vj). Boundary forward has the lot
        // interior on its left; partition forward has interior on both
        // sides — either way, the LEFT face is interior.
        self.origin.push(vi);
        self.kind.push(ss.kind);
        self.interior_left.push(true);
        self.next.push(usize::MAX);
        // Reverse half-edge (vj → vi). For boundary edges this puts
        // exterior/hole on the left (filtered out in face extraction).
        // For partitions both sides are interior.
        self.origin.push(vj);
        self.kind.push(ss.kind);
        self.interior_left.push(!ss.boundary);
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
            // Build polygon. When a partition line passes through a
            // hole, the face cycle here weaves hole edges into the
            // region's boundary. Strip those out: emit a vertex only
            // when the half-edge is NOT a hole edge, plus the entry
            // vertex of the first hole edge in each hole-run (so the
            // polygon bridges across the hole instead of wrapping
            // around it). The renderer subtracts whole holes
            // separately, so the bridge is invisible.
            let m = cycle.len();
            let mut poly: Vec<Vec2> = Vec::with_capacity(m);
            for i in 0..m {
                let he = cycle[i];
                let prev_he = cycle[(i + m - 1) % m];
                let is_hole = matches!(self.kind[he], EdgeKind::Hole(_, _));
                let prev_hole = matches!(self.kind[prev_he], EdgeKind::Hole(_, _));
                if !is_hole || !prev_hole {
                    poly.push(self.vertices[self.origin[he]]);
                }
            }
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
    // Hash FNV-1a over the canonical sequence. Each kind packs a tag
    // byte plus its payload (32-bit VertexId or 16-bit hole index +
    // 32-bit VertexId or 32-bit partition id), all comfortably within
    // a u64 — VertexId is u32 by definition, so even the dual-payload
    // Hole variant fits.
    let mut h: u64 = 0xcbf29ce484222325;
    for j in 0..n {
        let k = &kinds[(best + j) % n];
        let code: u64 = match *k {
            EdgeKind::Outer(v) => (1u64 << 56) | v.0 as u64,
            EdgeKind::Hole(h_, v) => {
                (2u64 << 56) | ((h_ as u64 & 0xFFFF) << 32) | v.0 as u64
            }
            EdgeKind::Partition(p) => (3u64 << 56) | p as u64,
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

    fn ids(start: u32, n: usize) -> Vec<VertexId> {
        (0..n as u32).map(|i| VertexId(start + i)).collect()
    }

    #[test]
    fn outer_only_single_region() {
        let outer = rect(10.0, 10.0);
        let oid = ids(1, outer.len());
        let regions = enumerate_regions(&outer, &oid, &[], &[]);
        assert_eq!(regions.len(), 1);
        assert!((signed_area(&regions[0].clip_poly) - 100.0).abs() < 1e-6);
    }

    #[test]
    fn outer_with_hole_single_region() {
        let outer = rect(10.0, 10.0);
        let oid = ids(1, outer.len());
        let hole = vec![
            Vec2::new(3.0, 3.0),
            Vec2::new(7.0, 3.0),
            Vec2::new(7.0, 7.0),
            Vec2::new(3.0, 7.0),
        ];
        let hid = ids(100, hole.len());
        let regions = enumerate_regions(
            &outer,
            &oid,
            &[(hole.as_slice(), hid.as_slice())],
            &[],
        );
        // Outer + hole → one annular face.
        assert_eq!(regions.len(), 1);
    }

    #[test]
    fn chord_splits_into_two_regions() {
        let outer = rect(10.0, 10.0);
        let oid = ids(1, outer.len());
        // Horizontal chord from (0, 5) to (10, 5).
        let parts = vec![(42u32, Vec2::new(0.0, 5.0), Vec2::new(10.0, 5.0))];
        let regions = enumerate_regions(&outer, &oid, &[], &parts);
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
        let oid = ids(1, outer.len());
        // Partition from (0,5) to (5,5) — endpoint (5,5) is interior.
        let parts = vec![(7u32, Vec2::new(0.0, 5.0), Vec2::new(5.0, 5.0))];
        let regions = enumerate_regions(&outer, &oid, &[], &parts);
        assert_eq!(regions.len(), 1, "dangling partition must not split");
    }

    #[test]
    fn two_chords_three_regions() {
        let outer = rect(10.0, 10.0);
        let oid = ids(1, outer.len());
        let parts = vec![
            (1u32, Vec2::new(3.0, 0.0), Vec2::new(3.0, 10.0)),
            (2u32, Vec2::new(7.0, 0.0), Vec2::new(7.0, 10.0)),
        ];
        let regions = enumerate_regions(&outer, &oid, &[], &parts);
        assert_eq!(regions.len(), 3);
    }

    #[test]
    fn region_ids_stable_across_runs() {
        let outer = rect(10.0, 10.0);
        let oid = ids(1, outer.len());
        let parts = vec![(42u32, Vec2::new(0.0, 5.0), Vec2::new(10.0, 5.0))];
        let r1 = enumerate_regions(&outer, &oid, &[], &parts);
        let r2 = enumerate_regions(&outer, &oid, &[], &parts);
        let mut ids1: Vec<_> = r1.iter().map(|r| r.id).collect();
        let mut ids2: Vec<_> = r2.iter().map(|r| r.id).collect();
        ids1.sort_by_key(|r| r.0);
        ids2.sort_by_key(|r| r.0);
        assert_eq!(ids1, ids2);
    }

    #[test]
    fn region_ids_stable_when_unrelated_vertex_inserted() {
        // Region IDs must not change when a vertex is inserted on a part
        // of the outer loop that doesn't bound the region. Original loop
        // is a 4-vertex rect split horizontally; inserting a vertex on
        // the bottom edge (between v0 and v1) should leave the upper
        // region's ID unchanged, since none of its bounding edges touch
        // the new vertex.
        let outer = rect(10.0, 10.0);
        let oid = ids(1, outer.len()); // [V1, V2, V3, V4] for [P0..P3]
        let parts = vec![(42u32, Vec2::new(0.0, 5.0), Vec2::new(10.0, 5.0))];
        let r1 = enumerate_regions(&outer, &oid, &[], &parts);

        // Insert (5,0) between v0 (0,0) and v1 (10,0) with a *new* id.
        // The original v1..v3 keep their ids — exactly the stability
        // property positional indexing failed to provide.
        let outer2 = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(5.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        let oid2 = vec![
            VertexId(1),
            VertexId(99), // new vertex
            VertexId(2),
            VertexId(3),
            VertexId(4),
        ];
        let r2 = enumerate_regions(&outer2, &oid2, &[], &parts);

        // Upper region (above the chord) should have the same id in
        // both runs — its boundary touches v3 (top-right corner),
        // v4 (top-left), and the partition; none of those changed.
        let pick = |rs: &[PartitionedRegion]| -> RegionId {
            rs.iter()
                .find(|r| r.clip_poly.iter().all(|p| p.y >= 5.0 - EPS))
                .expect("upper region")
                .id
        };
        assert_eq!(pick(&r1), pick(&r2));
    }

    #[test]
    fn region_ids_stable_across_chord_count_changes() {
        // Simulate what `discretize_polygon` produces when an arc edge
        // gets discretized into different numbers of chord segments.
        // The polygon is [V1, V2, ...synth chords..., V3, V4] — a
        // rectangle whose top edge is replaced by chord samples
        // standing in for an arc from V2 to V3. The number of chord
        // samples differs between the two runs; region IDs must match
        // because all chord segments share the same upstream sketch
        // corner (V2).
        let parts = vec![(42u32, Vec2::new(0.0, 5.0), Vec2::new(10.0, 5.0))];

        // Loop 1 — three chord interiors along the top edge.
        let mut outer1 = vec![
            Vec2::new(0.0, 0.0),  // V1
            Vec2::new(10.0, 0.0), // V2 (arc start)
        ];
        let mut ids1 = vec![VertexId(1), VertexId(2)];
        for i in 0..3 {
            outer1.push(Vec2::new(10.0 - 1.0, 10.0 + 0.1 * i as f64));
            ids1.push(VertexId(VertexId::SYNTHETIC_BASE + i as u32));
        }
        outer1.push(Vec2::new(0.0, 10.0)); // V3 (arc end)
        outer1.push(Vec2::new(0.0, 5.0)); // bend so chord crosses
        ids1.push(VertexId(3));
        ids1.push(VertexId(4));

        // Loop 2 — five chord interiors. Different chord count.
        let mut outer2 = vec![Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0)];
        let mut ids2 = vec![VertexId(1), VertexId(2)];
        for i in 0..5 {
            outer2.push(Vec2::new(10.0 - 1.5, 10.0 + 0.1 * i as f64));
            ids2.push(VertexId(VertexId::SYNTHETIC_BASE + i as u32));
        }
        outer2.push(Vec2::new(0.0, 10.0));
        outer2.push(Vec2::new(0.0, 5.0));
        ids2.push(VertexId(3));
        ids2.push(VertexId(4));

        let r1 = enumerate_regions(&outer1, &ids1, &[], &parts);
        let r2 = enumerate_regions(&outer2, &ids2, &[], &parts);

        // Lower region is bounded by V1, V2, and the partition — no
        // chord segments touch it. Its id was already stable; assert
        // the chord-touching upper region is now also stable.
        let upper = |rs: &[PartitionedRegion]| -> RegionId {
            rs.iter()
                .max_by(|a, b| {
                    let ay: f64 =
                        a.clip_poly.iter().map(|p| p.y).sum::<f64>() / a.clip_poly.len() as f64;
                    let by: f64 =
                        b.clip_poly.iter().map(|p| p.y).sum::<f64>() / b.clip_poly.len() as f64;
                    ay.partial_cmp(&by).unwrap()
                })
                .expect("a region")
                .id
        };
        assert_eq!(
            upper(&r1),
            upper(&r2),
            "region id flipped when chord count changed",
        );
    }
}
