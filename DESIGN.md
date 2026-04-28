# Procedural Parking Sites

— @Irfan Sharif

This page proposes an architecture for parking-v2, a procedural parking
generation system with additional controls we've found wanting in parking-v1.
It's a fresh system, so we won't refer to parking-v1 much. It will allow
defining a parking site boundary with optional building footprints ("holes")
within it, and produce a complete layout — driving aisles, stalls, islands —
through a layered pipeline of geometric operations. It will allow for some
manual operations that "persist" to continually steer the procedural generation
itself.

## Motivation

We want to support a few additional things over parking-v1, see [Parking
V2](https://www.notion.so/Parking-V2-21550072194f80cc9bdcf073380944a4?pvs=21) for background.

1. **Drive aisle editing** — We want control over aisle layouts beyond what
   auto-generation produces: adding cut-through drive aisles, deleting
   edges/vertices, change traffic flow (one-way, two-way-oriented).
2. **Angled and one-way parking** — Like 45°/60° stalls, for instance.
3. **Intersecting geometry** — stalls should respond to columns, cores, and
   other geometry that intersects the parking sketch (important for structured
   garages where columns punch through).
4. **Region segmentation** — the ability to segment a site into regions with
   independent aisle angles/grid offsets, and potentially other properties
   (stall sizes, angles, etc.)
5. **Islands and max run** — V1 produced unbroken seas of stalls with no island
   stalls. We want some max-run spacing, end-of-row islands, and corner islands.
6. **Arc/curve support** — parking along curved site boundaries. And also
   potentially allowing inner (i.e. not hugging the site boundary/holes) drive
   aisles to be curved.
7. **Site offset** — distance from sketch edges to first stalls.

We built a prototype
([**http://irfansharif.io/parking](http://irfansharif.io/parking) —**
[**http://github.com/irfansharif/parking**](http://github.com/irfansharif/parking))
that tries to flesh out the core generation pipeline: boundaries with arc
support (via Bézier edges), semi-automatic aisle graphs and with edits
(add/delete edges and vertices, and direction control), region-based
segmentation, angled and/or one-way stalls, islands with max run, corner
islands, cut through drive lines, and site offsets. There are few other features
we want to support, and we'll incorporate into the production integration:

1. **Ramps** — sloped drive aisles and outside-edge ramps with angle-based
   length calculation from floor-to-floor height.
2. **Custom stall types** — ADA, EV, compact stalls with user-specified counts
   and placement, potentially split across different buildings.

![Parking-v1, with various controls. Missing the things listed above.](attachment:e83dbebb-5bec-4937-8c06-aa225c0248fb:image.png)

Parking-v1, with various controls. Missing the things listed above.

![Prototype for parking-v2, showing some of the additional features above. ](attachment:f88edb32-c51b-4e77-a848-4ee41e26fd20:image.png)

Prototype for parking-v2, showing some of the additional features above.

## 1. Core Primitives

### 1.1 Polygon with Holes

The boundary input is a **polygon with holes**: a CCW outer boundary
representing the parking site (the lot **deed line** — the outermost
sketch the user draws), with zero or more CW inner polygons representing
building footprints or other obstacles. Holes are stored as **aisle-edge
rings** — they sit directly where the perimeter aisle around each
building hugs the wall. The outer is inset inward by one parking-row
depth + one aisle width (+ optional `site-offset`) at the top of the
pipeline to recover the aisle-edge perim ring; holes need no inset.

Edges of both the outer boundary and the holes can optionally carry a
circular arc (AutoCAD-style bulge parameterization), allowing curved
site boundaries. Curved edges are discretized into dense polylines at
the top of the pipeline, so all downstream geometry operates on
straight segments only.

Each sketch vertex carries a stable `VertexId` (a 32-bit integer
allocated at parse time and threaded through the pipeline). Annotations
that target perimeter sub-edges address them as `(start_vid, end_vid)`
pairs rather than by array index or arc length, so the address survives
vertex insertion, deletion, and reordering elsewhere on the loop. Synthetic
ids (above `1<<30`) are auto-allocated for chord vertices created during
arc discretization; they're not addressable from annotations.

### 1.2 Drive-Aisle Graph

The structural primitive is a planar graph of **vertices** and **edges**.
Vertices are 2D points. Each edge carries some metadata:

- **Interior flag** — distinguishes interior aisles (generated within the site)
  from perimeter aisles (tracing the inset of the outer boundary and the hole
  loops).
- **Direction** — Edge endpoints (start, end) are stable and never reordered to
  encode direction. `None` is the default unannotated two-way state; `Some` is
  one of `OneWay`, `OneWayReverse`, or `TwoWayReverse`. `OneWay` means traffic
  flows along the edge's stored start→end; `OneWayReverse` flips that without
  mutating the endpoints (the canonical edge geometry stays stable, direction
  rides entirely in the tag). `TwoWayReverse` keeps both lanes but mirrors the
  stall-lean across the aisle axis. Direction informs stall placement: which
  sides of an aisle get stalls, which way angled stalls lean, and whether
  opposing rows interleave (one-way) vs braid (two-way).

Each vertex is tagged **perimeter** (the first `perim_vertex_count` indices,
pinned to the aisle-edge inset of the outer boundary and to hole loops, brown
dots below) or **interior** (blue dots in the figure below). Interior aisle
endpoints spliced onto the perimeter are *not* counted as perimeter — the user
needs to drag the perimeter vertices themselves to reshape the site.

![Underlying edges and vertices, with the red edges indicating the perimeter aisles. We also see the edge provenance in play, showing each side of the individual parking bays coloured depending on the corresponding aisle (blue for interior edges therefore containing angled stalls, yellow otherwise, and pink of representing some wall where there’s no aisle entry).](attachment:77214313-a5a1-4f70-a6b1-7d8ecf494600:image.png)

Underlying edges and vertices, with the red edges indicating the perimeter
aisles. We also see the edge provenance in play, showing each side of the
individual parking bays coloured depending on the corresponding aisle (blue for
interior edges therefore containing angled stalls, yellow otherwise, and pink of
representing some wall where there’s no aisle entry).

### 1.3 Drive Lines and Regions

**Drive lines** are user-drawn line segments that cut through the site, creating
additional driving aisles (think cross-cuts). They're clipped to the aisle-edge
perimeter (the same ring the perim aisles live on, not the raw deed line) minus
expanded hole interiors, and clamped to the nearest auto-graph edge crossings
at each end so a short segment between two interior aisles only extends to
those aisles. Surviving sub-intervals are spliced into the aisle graph as
interior edges and vertices, snapping endpoints to nearby vertices or splitting
existing edges to create junctions. Each drive line carries a stable `id` and a
`partitions` flag — when set, the line participates in region decomposition;
when unset, it's a corridor that lives in the aisle graph but doesn't divide
space. Splice vertices remember the originating `(drive_line_id, t)` so
annotations can address them parametrically.

**Regions** are the bounded faces of the planar arrangement of `{outer
boundary ∪ hole boundaries ∪ partitioning drive lines}`. Holes aren't special:
they're arrangement edges like anything else. A dangling partition (one that
doesn't close a cycle) is inert by construction — no new face. Decomposition
runs against the **raw sketch** (not the inset) — region IDs are tied to
sketch corners, so they don't shift when params like `stall-depth`,
`aisle-width`, or `site-offset` change the inset distance. Each region's
clip polygon is then intersected with the aisle-edge perim before driving
the auto-generated aisles.

Region IDs are derived from the **canonicalized cyclic sequence of edge kinds**
bounding each face — `Outer(VertexId)`, `Hole(hole_index, VertexId)`, or
`Partition(drive_line_id)` — deduped for consecutive identical kinds and
rotated to lex-smallest first. This survives vertex renumbering, partition
re-ordering, and edits elsewhere on the loop, so per-region overrides keep
resolving across regenerations. The single-region case (no partitions, or
partitions that don't close any sub-region) gets a reserved fallback ID.

Each region gets its own aisle angle and offset, overridable per-region through
a "control vector" anchored at the region's pole of inaccessibility (computed
via the `polylabel` crate) so the vector sits inside even for concave or
ring-shaped regions.

### 1.4 Stall Modifiers

Stall modifiers are user-drawn geometry — a polyline plus a target
`StallKind` — applied as a post-placement pass against placed stalls.
Suppression is not a separate concept from retype: it folds into
`StallKind` as a distinguished `Suppressed` variant, and the renderer
skips stalls of that kind. So:

- `kind = Suppressed` — removes any stall within `radius` of the modifier
  (useful for clearing entrances, fire lanes, or any corridor that
  should remain unobstructed). Suppression applies regardless of the
  stall's prior kind.
- Any other kind (`Compact`, `Ev`, etc.) — retypes overlapping stalls.
  `Standard`, `Compact`, and `Ev` are retypeable; `Island` is preserved
  (it's coupled to already-carved landscape geometry, so retyping would
  create a conflict).

The modifier geometry is a polyline; a single point is the zero-length
case (e.g., place one ADA stall here). The match radius scales with stall
footprint (`stall_depth * 0.5`) so users don't have to draw pixel-perfect
lines to catch a row of stalls. Modifiers stay in world-space and don't
follow grid transforms — suppressing a fire lane shouldn't move when
the user nudges aisle-angle.

![Drive lines, with partitioning ones dividing the space into independent regions (colored). Each region has a “control vector” to adjust angle and offset.](attachment:79be9336-9a37-4bc3-977c-f52c3d218995:image.png)

### 1.5 Annotations

Annotations are spatial intents that survive graph regeneration. Each
targets a vertex or sub-edge(s) by addressing a point (or, on grid lines,
a span) in one substrate's own coordinate system. The three substrates
each carry a canonical direction:

- **Abstract grid** (per region) — integer lattice with both axes, plus
  optional `Lattice`/`CrossesDriveLine`/`CrossesPerimeter` stops along
  the line's free axis. Grid lines run low→high along their free axis.
  The lattice is materialized as an `AbstractFrame`: an orthonormal
  transform with `dx = 2 · effective_depth + 2 · aisle_width` and
  `dy = stalls_per_face · stall_pitch`, derived per-region from the
  region's effective `aisle_angle_deg` / `aisle_offset`. Annotations
  keyed by integer `(xi, yi)` translate to world coords via
  `frame.forward(p)` — so dragging the aisle vector
  (changing `aisle_offset`) slides every grid annotation in lockstep
  with the lattice it sits on.
- **Drive lines** — parametric `t ∈ [0, 1]` from start to end. Splice
  vertices placed by the drive-line splicer remember their `t`, so
  annotations resolve by t-proximity rather than world distance.
- **Perimeter** — addressed by the **sketch edge** carrying the point:
  `(loop, start_vid, end_vid, t)` where `loop` is `Outer` or
  `Hole(index)`, the vertex pair names a sketch edge in the loop's
  canonical winding (CCW outer, CW holes), and `t ∈ [0, 1]` is the
  position along that edge's chord (or arc, when the sketch edge has
  bulge data). The pipeline evaluates `t` to a world point, projects
  onto the graph perim ring (the inset for `Outer`, the hole sketch
  itself for `Hole`), and matches against graph vertices and sub-edges
  by arc length. Outer projection allows `inset_d + arc_slack` of
  perpendicular offset; hole projection just `arc_slack`.

Three annotation kinds: **delete-vertex** (cascades to incident
sub-edges; degree-2 collinear interior vertices left over from a
deletion are merged into single edges so columns don't carry vestigial
crossings), **delete-edge**, and **direction** (`OneWay` /
`OneWayReverse` / `TwoWayReverse`, relative to the carrier's canonical
direction — the resolver flips the stored target into the corresponding
edge-frame variant when start/end disagrees with canonical). User-drawn
polygon corners are edited via the sketch, not via annotation.

The grid lines seen below are a transformation of a unit grid in an
infinite space, rotated/stretched/translated onto the 2D canvas;
annotations stored in that unit grid's coordinates follow
transformations for free.

**Constraint.** No collinear stretches between any two substrates. Every
substrate pair crosses the other at most once per traversal, so
`CrossesDriveLine` / `CrossesPerimeter` stops resolve unambiguously to
the first crossing along the carrier's canonical direction.

**Dormancy (single rule).** Any referenced substrate or stop not
present in the current graph → the annotation is dormant this regen.
Each dormant annotation is reported back to the UI as a
`{ index, reason }` pair (e.g., "region {…} not in current
decomposition", "no graph vertex on perimeter loop within tol of
arc 0.42"), so the UI can surface them without losing the user's intent.
Types in §6.1.

![Some annotations shown, like driving directions, deletion of a vertex, and also on specific edges (in this example just individual segments, not collinear stretches).](attachment:b27c618f-bbc9-4f25-b78e-68b744c61d46:image.png)

Some annotations shown, like driving directions, deletion of a vertex, and also
on specific edges (in this example just individual segments, not collinear
stretches).

## 2. Pipeline Overview

The generation pipeline is deterministic and stateless — given the same input,
it produces the same output. Each layer's output is the next layer's input. The
layers are pure functions — no shared mutable state, no callbacks, no side
effects.

```
Input (boundary, holes, curves, drive lines, annotations, params)
  │
  ├─ 1. Boundary discretization (curves → polylines)
  │     - All downstream geometry operates on straight
  │       segments only.
  │
  ├─ 2. Aisle graph construction (auto-generate, merge
  │     manual edits, splice drive lines, apply annotations)
  │     - Produces the topological structure: vertices,
  │       edges with direction and interior/perimeter
  │       metadata.
  │
  ├─ 3. Aisle polygon construction (centerline graph → merged
  │     driveable surface)
  │     - Stroke the centerline graph by `aisle-width` via
  │       i_overlay's polyline stroker: closed perim/hole
  │       loops with mitered joins, interior segments
  │       butt-capped (their perpendicular butts overlap at
  │       4-way crossings, filling junctions implicitly).
  │       Boolean-union the strokes into merged shapes.
  │
  ├─ 4. Parking bay extraction (boundary − aisles − holes)
  │     - Produces the "negative space" polygons where
  │       stalls will be placed.
  │
  ├─ 5. Edge provenance tagging (TaggedFace per bay)
  │     - Classify each face edge as Wall or
  │       Aisle{corridor_idx, interior, travel_dir,
  │       is_two_way_oriented}. Edges that straddle an
  │       aisle/wall transition are split via binary search.
  │       Drives boundary-face classification, stall-angle
  │       policy, one-way braiding, and spine direction.
  │
  ├─ 6. Spine generation (aisle-facing edge → offset
  │     centerline)
  │     - For each aisle-facing edge of each bay, offset
  │       inward by `stall-reach` and trim each end inward
  │       by a corner-angle-dependent amount (so the spine
  │       sits on the integer abstract-grid line). Post-
  │       process: clip to the face, dedup overlaps, merge
  │       collinear chains across face seams, extend
  │       interior spines colinearly to the face boundary
  │       (folded back into the primary's extent), drop
  │       spines that won't fit `min-stalls-per-spine`.
  │
  ├─ 7. Stall placement (grid-locked fill along spines)
  │     - Fill each spine with stalls at `stall-pitch`
  │       intervals. `flip_angle` (lean) and `staggered`
  │       (half-pitch shift) bits are precomputed at spine
  │       construction from per-aisle direction; placement
  │       just reads them. Opposing spines grid-lock through
  │       a shared absolute-grid origin.
  │
  ├─ 8. Stall filtering
  │     - Conflict removal within each face (annotated
  │       spines get a small length bonus to outrank
  │       identically-long unannotated neighbors), face
  │       clipping (boolean containment with overhang
  │       tolerance), entrance-on-face filter, stall-modifier
  │       retype/suppress, short-segment removal.
  │
  └─ 9. Island computation (face − stalls → landscape gaps)
        - Mark every Nth stall as an island gap.
          Boolean-subtract non-island stalls from face
          polygons to produce landscape contours. When
          `island-corner-rounding` is on, morphologically
          open each face by `island-corner-radius` first
          (round joins) so islands at convex corners
          inherit a turn-radius-shaped arc instead of a
          90° wedge.
  │
Output (ParkingLayout)

```

## 3. Pipeline Details

### 3.1 Aisle Graph Construction

Given a boundary polygon with holes, we produce a regular grid of driving
aisles in two phases.

**Phase A — auto-generate.**

1. Inset the outer deed line inward by `inset_d + site-offset` to
   recover the **aisle-edge perim** ring (the centerline that the perim
   aisle rectangles sit on), where `inset_d = effective_depth +
   aisle-width` is the mandatory perimeter parking-band thickness, and
   `effective_depth = stall-depth * sin(stall-angle) +
   cos(stall-angle) * stall-width / 2`. Holes need no inset — the
   sketched hole IS the aisle-edge ring around the building.
2. Snap hole vertices onto nearby aisle-edge perim edges (within
   `aisle-width`). When a building sits close to the boundary, this
   collapses near-duplicate parallel edges that would otherwise produce
   unusable sliver faces.
3. Decompose the lot into regions (bounded faces of the planar
   arrangement of `{outer ∪ holes ∪ partitioning drive lines}` on the
   raw sketch). For each region, intersect its raw clip polygon with
   the aisle-edge perim to get the aisle-usable area, then:
    - Pick the dominant aisle angle as the longest edge of the region
      polygon (regardless of whether it came from outer / hole /
      partition). User-overridable per region via `RegionOverride`.
    - Walk grid lines along the region's `AbstractFrame` integer
      lattice — every integer `xi` in abstract space contributes one
      driving aisle, anchored to `aisle_offset` so dragging the aisle
      vector slides the entire grid (and any annotations keyed by
      integer `(xi, yi)` follow the shift).
    - For each grid line, intersect with the region's clip face minus
      hole interiors to get enter/exit intervals.
    - Record where grid lines cross perim/hole edges (split points).
4. Generate perpendicular cross-aisles at every integer `yi` in the
   abstract frame — i.e., spaced by `stalls-per-face * stall-pitch`
   along the aisle direction. `stalls-per-face` is what the user
   tunes (the number of stalls in each face along the aisle direction);
   the spacing falls out. Split main-aisle and cross-aisle pairs at
   their mutual intersections so the graph is fully connected.
5. Build perim edges from the inset/hole loops, splitting at grid
   intersections.
6. Build interior edges for each grid and cross-aisle segment.

**Phase B — apply edits.**

1. (**Later**) Merge any user-edited manual graph: auto edges that overlap
   manual aisles are filtered out.
2. Splice in drive lines: clipped to the aisle-edge perim minus
   expanded holes, clamped to the nearest auto-graph edge crossings at
   each end, and merged into the graph (snapping endpoints to nearby
   vertices or splitting existing edges to create junctions). Each
   splice vertex remembers `(drive_line_id, t)` for annotation lookup.
3. Apply annotations — direction (first pass) then deletion (second
   pass). Direction tags the edge without flipping `(start, end)`;
   deletion drops edges and rebuilds adjacency, then iteratively merges
   any degree-2 collinear interior vertex (vestige of a deleted
   crossing) into a single edge.

### 3.2 Aisle Polygon Construction

The merged driveable surface comes from **stroking the centerline graph**
through `i_overlay::StrokeOffset`:

- **Perim/hole loops** — the perimeter sub-graph is decomposed into
  maximal paths. A loop with no annotation-driven deletions yields one
  closed polyline per ring; an annotation that deletes a perimeter
  vertex breaks that cycle into one or more open polylines whose
  endpoints sit at the deletion point. Closed polylines are stroked
  with **mitered joins** so corners turn cleanly; open polylines are
  stroked with **butt caps** so the "doorway" stays open downstream.
- **Interior aisles** — each interior edge is stroked as its own
  2-point open polyline with butt caps. The perpendicular butts of two
  edges meeting at an interior 4-way crossing overlap at the centre,
  so the junction is fully covered without explicit wedge geometry.

Miter joins below 15° fall back to bevels — without that, two aisles
meeting at a sharp angle would shoot a miter spike many aisle-widths
long. The result is boolean-unioned (`FillRule::NonZero`) into one or
more merged shapes; each shape is `[outer, hole₁, hole₂, …]`.

This replaces the previous hand-rolled "rectangle + junction wedge +
spike-removal + RDP-simplify + small-hole filter" passes — the stroker
absorbs all of them in one shot, and the matching `boundary_only_miters`
debug toggle is gone (it falls out of the strategy: closed loops miter,
open polylines butt-cap).

(**Later**) Curved drive aisles can be supported by discretizing into a
dense polyline before stroking.

`corridor_polygons` (per-edge rectangle, paired with each edge's
`interior` flag and OneWay travel direction) is still emitted as a
side-channel feed for face-edge tagging in §3.4 — it lets the tagger
attribute each face edge to a specific source aisle even after the
merged surface erases per-edge identity.

### 3.3 Parking Bay Extraction

Parking bays are the positive-space regions between drive aisles where stalls
will be placed. They're computed by subtracting the merged aisles and the
building footprints (holes) from the site boundary. The end state is a set of
polygons-with-holes, each representing a parking bay.

### 3.4 Edge Provenance

After extracting parking bays, each edge of each bay is tagged with its
source into a `TaggedFace` (one per bay) carrying `{edges, hole_edges,
is_boundary, wall_edge_indices}`. Every `FaceEdge` carries `{start, end,
source}`:

- `EdgeSource::Wall` — the face edge lies on the site deed line or a
  building footprint.
- `EdgeSource::Aisle { corridor_idx, interior, travel_dir,
  is_two_way_oriented }` — the face edge lies on a drive aisle.
  `corridor_idx` points back to the per-edge corridor rectangle (so
  downstream code can recover the carrier edge),
  `interior`/`perimeter` distinguishes inner aisles from the perim
  band, `travel_dir` is `Some(unit_vec)` for OneWay aisles and `None`
  otherwise, and `is_two_way_oriented` flags TwoWayReverse aisles.

The tagger samples each face edge's endpoints against the merged aisle
shapes; if both are off, it's a wall; if both are on, it's an aisle (and
the per-edge corridor with the best segment-overlap score wins
`corridor_idx`); if one is on and the other off, the transition point
is found via binary search and the edge splits into a wall sub-edge
plus an aisle sub-edge in the correct winding order.

Per-edge provenance drives every downstream decision:

- **Boundary classification** — a face with any wall edge (or with only
  perimeter aisle edges, no interior aisles) is flagged as a
  **boundary face**. Boundary faces get 90° stalls regardless of the
  global `stall-angle` (see §3.6).
- **Stall direction handling** — for OneWay aisles, `travel_dir`
  drives per-side asymmetric flip + half-pitch stagger so opposing
  rows herringbone in travel direction. For TwoWayReverse,
  `is_two_way_oriented` flips the lean on the spines bordering that
  carrier aisle only; opposing strips in the same bay stay on the
  default lean unless their own carrier aisle is also annotated.
- **Spine emission** — only aisle-facing edges produce spines. Wall
  edges contribute only to corner-trim math at the spine endpoints
  (see §3.5).

![Actual parking bays extracted after having subtracted the merged aisles. It also shows edge provenance by the side colors, representing walls (pink), interior (blue) or perimeter (yellow) sides. Also, there’s a hatched pattern around the”boundary” faces — those that have no interior sides.](attachment:5ee668ab-293a-4f81-9a38-c938b65551a4:image.png)

Actual parking bays extracted after having subtracted the merged aisles. It also
shows edge provenance by the side colors, representing walls (pink), interior
(blue) or perimeter (yellow) sides. Also, there’s a hatched pattern around
the”boundary” faces — those that have no interior sides.

### 3.5 Spine Generation

Spines are the centerlines along which stalls are placed. They're emitted
**per aisle-facing face edge** by parallel offset — there's no event
queue, no wavefront, no straight skeleton. For each `FaceEdge` tagged
as aisle-facing, the spine runs parallel to the edge at `effective_depth`
inward (left-perp, given normalized winding) so its outward normal points
toward the aisle it serves.

Each spine endpoint is **trimmed inward** by an amount that depends on
the corner angle with its neighbor in the same contour, so the spine
ends sit on the integer abstract-grid line that the back-to-back row
on the next bay over will share:

- **aisle ↔ aisle corner** — `trim = ed / tan(θ/2)`, no floor. At a
  90° cross-aisle corner that's exactly `ed`, matching a grid bay's
  natural back-to-back row extent. Tangent-continuous neighbors
  (chord chains on a discretized arc, interior angle → 180°) trim
  toward zero so the chain stays within endpoint-merge tolerance.
- **aisle ↔ wall corner** — `trim = ed / tan(θ)`, clamped to zero at
  obtuse interior angles, with a floor of `ed/2` so perpendicular
  corners still leave room for an end island.

Per-side trims are capped at `edge_len / 2` so the spine can't invert at
very acute corners. The `spine-end-trim` debug toggle disables this
entirely (spines run the full edge length). Each spine is tagged with
its `face_idx`, `is_interior`, and three precomputed bits used by
placement (§3.6):

- `flip_angle` — XOR of OneWay's per-side asymmetry and the carrier
  aisle's per-edge TwoWayReverse flag; controls stall-lean direction.
- `staggered` — held equal to `flip_angle`. Half-pitch grid offset
  applied whenever the lean is flipped from default. Two opposing
  back-to-back spines have antiparallel oriented_dir, so when their
  flips disagree (e.g., one side TwoWayReverse, the other default;
  or OneWay's per-side asymmetry) the relative half-pitch shift
  interlocks their back edges; when their flips agree the shifts
  cancel and back edges meet on the same line.
- `is_annotated` — true if the spine borders any direction-annotated
  aisle. Used as a tie-break in conflict removal so annotated rows
  aren't dropped in favor of identically-long unannotated neighbors.

Spine post-processing:

- **Spine merging** — collinear spines with the same `flip_angle`,
  `staggered`, `is_annotated`, and matching outward normals are
  fused if their endpoints share within `spine-merge-endpoint-tol`
  and their direction vectors are within `spine-merge-angle-deg`.
  The merged spine projects all four endpoints onto the shared line
  and takes the full extent. This straightens chord-derived spine
  chains on discretized arcs into single straight segments.
- **Spine extensions** — interior spines are extended colinearly to
  the face boundary in both directions via dual-clipping (both the
  spine and the stall-reach offset must stay inside the face). The
  outer end is pulled inward by 1.5 × pitch to prevent sliver
  islands; the inner end overlaps the primary by one pitch so
  stalls flow continuously across the seam. Extensions then fold
  back into their source primary's extent (`extend_primary_with_extensions`)
  so the rest of the pipeline sees one longer `SpineSegment` per row.
- **Short-spine filter** — runs after stall placement (see §3.7);
  any spine with fewer than `min-stalls-per-spine` stalls drops.

(**Future**) For curved face edges, we could instead consider a manual inset
at `stall-reach`.

### 3.6 Stall Placement

Stalls are placed on spines:

1. **Stall angle policy** — interior faces use the configured global stall
   angle. Boundary faces (those flagged in §3.4) override to 90°: the
   perimeter band between the outer driving loop and the site edge reads and
   packs better at 90°. This is a design choice, not a geometric constraint.
2. **Stall pitch** = `stall-width / sin(angle)` — the spacing between stall
   centers along the spine.
3. **Lean and stagger** are read straight off the spine's precomputed
   bits (`flip_angle`, `staggered`), with the single invariant
   `staggered = flip_angle`. The three direction regimes set
   `flip_angle` differently, but the stagger rule is the same:
    - **Default (TwoWay, no annotation)** — `flip_angle = false`,
      `staggered = false`. Opposing back-to-back strips share the
      grid origin and back edges meet on the same line; their
      antiparallel `width_dir`s give back-to-back herringbone for
      free.
    - **TwoWayReverse** — `flip_angle = true` for spines bordering
      the annotated aisle, `staggered = true`. Mirrors the default
      lean across the aisle axis on that strip only; the half-pitch
      shift keeps back edges interlocking with the un-annotated
      partner. If both bordering aisles are annotated, both strips
      shift by ½ pitch (relative offset 0) and the flipped-mirror
      back edges meet on the same line.
    - **OneWay / OneWayReverse** — per-side asymmetric `flip_angle`
      (from `oriented_dir · travel_dir`); `staggered` follows.
      Opposing spines have antiparallel `oriented_dir`, so exactly
      one side gets staggered. Both sides end up leaning with
      traffic flow, and the half-pitch shift interlocks them.

   The stagger isn't a separate decision — it's a direct consequence
   of `flip_angle`. Two opposing spines have antiparallel
   `oriented_dir`, so the relative grid offset between them is
   `½ · (flip_A XOR flip_B)`: zero when leans agree (back edges
   coincide), ½ pitch when they disagree (back edges interlock).
4. **Stall quad corners** (corner indexing
   `[back_left, back_right, aisle_right, aisle_left]`):
    - Back corners (at the spine): `mid ± width-dir * (stall-width / 2)`,
      preserving the braided/interlocking pattern where opposing back
      edges zigzag against each other.
    - Aisle corners (at the drive aisle): `depth-dir` projection at
      `effective-depth / sin(stall-angle)`, spread by `± edge-dir *
      (pitch / 2)`. The renderer draws three sides
      (`[3]→[0]→[1]→[2]`) and leaves `[2]→[3]` open as the entrance.
5. **Grid alignment** — every stall midpoint snaps to a shared
   absolute grid along the spine direction. A `grid_offset ∈ [0, 1)`
   fraction of a pitch shifts the snap origin (used for the
   half-pitch stagger driven by `staggered = flip_angle`). Spines on
   opposite sides of a face grid-lock to the same shift through this
   shared origin.
6. **Placement order** — spines are processed longest-first, so when
   §3.7 conflict removal compares overlapping candidates the dominant
   row wins by length without further bookkeeping.

![Stall placement within each parking bay with respect to underlying spines, and following driving aisle directions (sometimes stall strips are suppressed accordingly). Stalls are also actually centered on the spine somewhat, and we try to place islands in opposing spines in way such that they’re aligned.](attachment:b5a72bb3-2a4c-4369-a8db-097625acc1a8:image.png)

Stall placement within each parking bay with respect to underlying spines, and
following driving aisle directions (sometimes stall strips are suppressed
accordingly). Stalls are also actually centered on the spine somewhat, and we
try to place islands in opposing spines in way such that they’re aligned.

### 3.7 Stall Filtering

After placement, stalls go through several filters in this order — the
order matters:

- **Entrance-on-face filter** — drops stalls whose aisle-facing
  (entrance) edge falls strictly inside the face interior. The entrance
  must sit on the face boundary or past it; an entrance interior to
  the face means the stall opens into a parking bay rather than a
  drive aisle, so it's unreachable. Sampled at 11 points along the
  segment with a 0.9 minimum coverage threshold and a 0.5 ft
  edge-tolerance.
- **Conflict removal** — runs **before** face-overhang clipping. When
  multiple candidate spines exist for the same narrow face (e.g.,
  opposing offset spines in a sub-`2 · effective-depth` bay), their
  stalls overlap spatially and need to fight it out on spine length.
  Running face clipping first would asymmetrically drop the candidate
  whose stalls hang slightly off a slanted face edge, silently handing
  the contest to the shorter spine. Tie-break: spines on
  direction-annotated aisles (`is_annotated`) get a 0.5-unit length
  bonus so they outrank identically-long unannotated neighbors. On
  boundary faces, both conflicting stalls are removed (more
  symmetrical corners); on interior faces, only the shorter wins.
- **Face clipping** — stalls not fully contained within their face are
  removed via boolean difference (`stall − face`); leftover area must
  be ≤ 2% of the stall area (`STALL_OVERHANG_TOL`). Since faces are
  already clipped to the site boundary minus holes, this subsumes
  boundary clipping.
- **Stall modifiers** — `apply_stall_modifiers` retypes overlapping
  stalls (see §1.4). `Suppressed` stays as a `StallKind::Suppressed`
  variant so downstream consumers can still see the suppression
  location; the renderer skips them.
- **Short-segment filter** — drops every stall belonging to a spine
  whose surviving stall count is below `min-stalls-per-spine`. Default
  3; setting to 1 effectively disables the filter.

### 3.8 Island Computation

Islands are landscape gaps between and at the ends of stall rows. They're
produced by:

1. **Mark island stalls** — every Nth stall (by `island-stall-interval`) is
   marked as `StallKind::Island`, using absolute position-based grid marking
   along each spine (`floor(proj / pitch) % interval == interval / 2`). End
   margins prevent marking stalls at row endpoints (1.5 × pitch on interior
   spines, 0.5 × pitch on boundary spines). Same-row stalls on opposite sides
   of an aisle agree on which positions become islands because the marking
   is anchored to absolute spine projection.
2. **Optional face rounding** — when `island-corner-rounding` is on,
   each face polygon is morphologically opened by `island-corner-radius`
   (erode-then-dilate with round joins) before residual extraction.
   Convex face corners pick up a turn-radius-shaped arc instead of a
   90° wedge, mimicking drive-aisle cornering geometry. The rounded
   shape is also threaded through the §3.7 stall-vs-face checks so
   stalls don't visually overshoot the rounded contour. If opening
   severs a face into multiple pieces (thin neck < 2r), the largest
   piece per face is kept so per-face indexing stays 1:1.
3. **Boolean subtraction** — for each face, subtract all non-island
   stalls (optionally outset by `STALL_DILATION = 0.5` ft to close
   sliver gaps from braided back edges) from the face polygon. The
   residual polygons (where island stalls were left as gaps) become
   island contours. Islands with net area below 10 ft² are filtered
   out. When a face has zero stalls placed, the whole face becomes
   one island (carrying its holes through so the area maths match
   the actual face area).


## 4. User Interaction Model

The user draws inputs (boundary, holes, drive lines) and then manipulates the
generated result. All interactive elements — vertices, edge midpoints, control
points, annotation markers, region vectors — are unified as draggable/clickable
anchors with a single hit-test mechanism. Annotations (direction, delete,
suppress) are placed on whatever is currently selected; they persist as spatial
anchors that survive graph regeneration, re-matching to nearby graph elements by
proximity after each rebuild. For example, pressing F in the prototype on a
selected edge cycles through direction modes. Each region has a draggable vector
that controls both the aisle angle (rotate the endpoints) and perpendicular
offset (drag the body).

Every edit triggers full regeneration: the input state is serialized, sent to
the engine, and the output is re-rendered. The pipeline is fast enough for
interactive use. Some parameters:

 | Parameter                   | Range   | Effect                                                          |
 |-----------------------------|---------|-----------------------------------------------------------------|
 | `stall-width`               | 7–40 ft | Individual stall width                                          |
 | `stall-depth`               | 8–30 ft | Stall depth perpendicular to aisle                              |
 | `aisle-width`               | 8–24 ft | One driving lane width (aisle is two lanes)                     |
 | `stall-angle`               | 45–90°  | Stall angle relative to aisle                                   |
 | `aisle-angle`               | any°    | Dominant aisle grid direction (lot-wide, region-overridable)    |
 | `aisle-offset`              | any     | Perpendicular grid offset (lot-wide, region-overridable)        |
 | `site-offset`               | 0–50 ft | Extra setback from the deed line, on top of the parking band    |
 | `stalls-per-face`           | 3–50    | Stalls along the aisle per face (drives cross-aisle spacing)    |
 | `island-stall-interval`     | 0–20    | Stalls between landscape islands                                |
 | `island-corner-radius`      | 0–20 ft | Rounding radius for island corners (when rounding on)           |
 | `min-stalls-per-spine`      | 1–10    | Drop spines with fewer surviving stalls (1 = off)               |
 | `arc-discretize-tolerance`  | 0.5–10  | Chord-deflection budget for arc → polyline                      |
 | `spine-merge-angle-deg`     | 1–30    | Max angular drift for collinear spine fusion                    |
 | `spine-merge-endpoint-tol`  | 0–5 ft  | Endpoint-share tolerance for spine fusion                       |

## 5. Additional Integration Points

### 5.1 Intersecting Geometry

In structured garages, columns and cores punch through parking floors. The
system's polygon-with-holes model already handles this — columns become
additional holes in the boundary input. We can project intersecting 3D geometry
onto the parking plane and inject them as holes before generation. The
pipeline's face extraction and stall clipping will handle the rest. The
`Suppress` stall modifier (§1.4) covers the case where a particular column or
core warrants extra clearance beyond its hole footprint.

### 5.2 Custom Stall Types and Placement

The prototype's `StallKind` enum (Standard, Compact, Ev, Island, Suppressed)
provides the hook. In production we want something akin to:

- User-specified counts per type (not auto-calculated — letting users do the
  math themselves and input counts directly).
- Drag-to-place for ADA and specialty stalls. The `StallType` modifier (§1.4)
  is the mechanism: drop a point or short polyline at a position; the
  post-placement pass retypes the overlapping stall(s). We can start with a
  best-effort default guess but let users manipulate to their needs.
    - Stall groups are simply individual stalls underneath, so can be split
      across buildings (e.g., ADA stalls distributed near multiple entrances,
      not blocked together).

### 5.3 Ramps

Two types:

- **Sloped drive aisles** are aisle edges with additional metadata, like the
  grade. The aisle polygon layer extrudes them normally; the stall layer could
  automatically skip faces adjacent to ramp edges (we’d likely power it through
  the `Suppress` stall modifier from §1.4). We could tie the ramp angle
  parameter and floor-to-floor height to auto-calculate the ramp length. We’d
  start off with just showing a 2D sketch of the ramp on the parking floor plan,
  extending to actual 3D ramp geometry later.
- **Outside-edge ramps** are dedicated ramp geometry attached to the site
  boundary — think a helix or switchback on the exterior of a structured garage.
  These aren't aisle edges in the graph; they're boundary features defined by a
  start/end point along the perimeter, a width, and a grade.  The ramp polygon
  is subtracted from the boundary (like a hole) so stalls and aisles don't
  encroach, and the ramp surface itself is emitted as standalone geometry with
  slope metadata for 3D extrusion.

### 5.4 Bake to Finalize

"Baking" converts generated geometry into serializable sketch primitives. The
output already contains all necessary geometry (stall quads, island contours,
aisle polygons). Baking means writing these back as (editable?) geometries into
the project file and detaching them from the generation pipeline. We want
stability across project opens even if the generation code itself changes. But
allow for someway to re-enter the procedural mode and trigger re-generation off
of input elements again. That would re-run the procedure using the latest
generation algorithm, but preserving the previous state in the undo stack if
necessary.

### 5.5 Metrics and Labels

The prototype outputs `total-stalls`. Production should add per-type counts,
parking ratio (stalls per unit area), and utilization percentage. Stall run
labels (e.g., "12 stalls" per row) can be derived from the spine data — each
spine's stall count is known after placement, and the spine's midpoint gives the
label position.

## 6. Arcol Integration

### 6.1 File format

Parking v2 is a function assigned to a sketch — attached per-face on
`FaceProperties`, alongside the existing `parking` field.
Feature-flagged until launch.

```ts
type FaceProperties = {
  color: Color;  isVoid?: boolean;
  typology?: TypologyId;  areaType?: AreaTypeId;
  parking?:   ParkingConfig;
  parkingV2?: ParkingV2;       // new
};

type ParkingV2 = {
  inputs: ParkingV2Inputs;     // editable
  baked?: ParkingV2Baked;      // immutable pipeline output
};
```

Two buckets. `inputs` is the only thing the UI writes; `baked` is a
complete replacement each regen. Everything the tools need between
regens — including the resolved aisle graph that backs edit-mode
handles — is part of `baked` (see below). Transient pipeline state
like the grid transform and region signatures are pure functions of
`inputs` + current topology, recomputed inside the generator each
regen.

**Inputs:**

```ts
type ParkingV2Inputs = {
  config: ParkingV2Config;                    // global params + region overrides
  annotations: ParkingV2Annotation[];         // grid-space, survive regen
};

type ParkingV2Config = {
  // global parameters (DESIGN §4): stallWidth, stallDepth, aisleWidth,
  // stallAngle, aisleAngle, aisleOffset, siteOffset, stallsPerFace,
  // islandStallInterval, islandCornerRadius, minStallsPerSpine,
  // arcDiscretizeTolerance, spine-merge tolerances, pillar / ADA
  // controls, etc.
  // ...

  // Per-region overrides of the global aisle angle / offset, keyed by
  // stable region id (§6.2).
  regionOverrides: ParkingV2RegionOverride[];
};

type ParkingV2RegionOverride = { regionId: RegionId; aisleAngle?: number; aisleOffset?: number };
type RegionId = number;     // FNV-1a hash of the canonicalized cyclic
                            // edge-kind sequence; masked to 48 bits so it
                            // round-trips through JSON safely. A reserved
                            // sentinel above the mask range tags the
                            // single-region fallback.

type ParkingV2Annotation =
  | { kind: "deleteVertex"; target: Target }
  | { kind: "deleteEdge";   target: Target }
  | { kind: "direction";    target: Target; traffic: TrafficDirection };

type TrafficDirection = "oneWay" | "oneWayReverse" | "twoWayReverse";

// A referenceable region in one substrate's own coord system (§1.5).
// Grid:       range=null → whole grid line; [s, s] → vertex; [s1, s2] (s1 ≠ s2) → span.
// DriveLine:  parametric t ∈ [0, 1] from drive_line.start → drive_line.end.
// Perimeter:  addresses a sketch edge by its (start, end) vertex-id pair plus
//             an in-edge parameter t ∈ [0, 1]. The pipeline evaluates t against
//             the edge (chord or arc, depending on stored bulge data), then
//             projects onto the graph perim ring for resolution.
type Target =
  | { on: "grid"; region: RegionId; axis: "x" | "y"; coord: number;
      range: [GridStop, GridStop] | null }
  | { on: "driveLine"; id: number; t: number }
  | { on: "perimeter"; loop: PerimeterLoop;
      start: VertexId; end: VertexId; t: number };

type GridStop =
  | { at: "lattice"; other: number }
  | { at: "crossesDriveLine"; id: number }
  | { at: "crossesPerimeter"; loop: PerimeterLoop };

type PerimeterLoop = { kind: "Outer" } | { kind: "Hole"; index: number };
type VertexId = number;
```

Annotations that fail to resolve this regen are returned in
`baked.dormantAnnotations: { index, reason }[]` so the UI can surface
them without losing the user's intent.

All the user-drawn lines — boundary, holes, drive lines, **and stall
modifiers** — live as edges in the sketch's own `PlanarNetwork`, so
they share its editing infrastructure (endpoint drag, snap,
split-on-cross, vertex-merge). A new additive per-edge discriminator
distinguishes their role to the pipeline:

```ts
userKind: "boundary" | "hole" | "driveLine" | "stallModifier";
partitions?: boolean;              // drive-line-only; §1.3
stallKind?: ParkingV2StallKind;    // stall-modifier-only; §1.4
```

"Suppress" is not a separate concept from retype — it folds into
`stallKind` as a distinguished `Suppressed` value; the pipeline
retypes overlapping stalls to that kind and the renderer skips them.
No separate suppress-vs-retype discriminator at the schema level.

**Baked** (the intermediate representation):

```ts
type ParkingV2Baked = {
  generatorVersion: number;
  inputHash: string;                           // over (sketch PlanarNetwork, inputs.*)

  // Render geometry — 2D polylines + arcs, no back-refs.
  stalls: Array<{ loop: PlanarPath; kind: ParkingV2StallKind }>;
  aisles: PlanarPath[];
  islands: PlanarPath[];
  spineLabels: Array<{ midpoint: Vec2; stallCount: number }>;
  metrics: { totalStalls: number; stallsByKind: Record<ParkingV2StallKind, number> };

  // The resolved aisle graph. Vertices carry their (u,v) grid coord;
  // edges carry interior/perimeter and direction. Read by the
  // edit-mode manipulation tool to render addressable handles; not
  // rendered directly. (This is why it's in baked rather than
  // recomputed — it backs user-visible handles, so regenerating it
  // on entry to edit mode would flash.)
  aisleNetwork: PlanarNetwork;

  // Annotations that didn't resolve this regen — surface in the UI
  // (e.g., dimmed handle) without dropping the user's intent.
  dormantAnnotations: Array<{ index: number; reason: string }>;
};
```

Persisting `baked` does two things. It locks doc appearance to the
last bake — a doc saved at generator version N opened at N+1 keeps
its look until an input changes. And because Liveblocks swaps
`inputs` and `baked` together during undo/redo, history ops display
correctly without waiting on a regen round-trip; no post-undo flash
of stale or empty geometry. The pattern is generator-agnostic — the
`baked<T>` field + `Execution.ts` skip-on-hash-match path should be
parameterised so other procedural sketch functions can reuse it.

Invariants:

1. The renderer reads `baked` only; `inputs` are inputs, never drawn.
2. Regen runs on a worker. The render tick compares
   `inputHash(current inputs)` against `baked.inputHash`; on mismatch,
   enqueue regen. Until it returns, render current `baked`. This makes
   invalidation indifferent to the write path — user edits, config
   changes, and undo/redo all route through the same check.
3. When regen completes, replace `baked` atomically. Skip the write
   if `inputHash` matches and `generatorVersion === current`. `baked`
   is never edited in place.

### 6.2 PlanarNetwork reuse — where things live

| Primitive                                     | Lives in                                          |
|-----------------------------------------------|---------------------------------------------------|
| Boundary, holes, drive lines, stall modifiers | Sketch's `PlanarNetwork`, tagged via `userKind`   |
| Aisle graph (output)                          | `baked.aisleNetwork` (separate `PlanarNetwork`)   |
| Annotations                                   | `inputs.annotations`, grid-space keyed            |
| Region overrides                              | `inputs.config.regionOverrides`, keyed by stable id |
| Stalls, islands, spine labels                 | `baked` geometry arrays                           |

User-drawn lines all share `PlanarNetwork` so they share its editing
infrastructure. `userKind` tells the pipeline the role of each line:
drive lines participate in the aisle graph (and optionally region
decomposition, via `partitions`); stall modifiers are a post-pass
overlay keyed off `stallKind`. Stalls / islands / spines aren't
editable topology — they're pipeline outputs, held in `baked` as
geometry.

Region ids are a canonical signature of the region's bounding edge
sequence — `Outer(VertexId)`, `Hole(hole_index, VertexId)`, or
`Partition(drive_line_id)`, deduped for consecutive identical kinds and
rotated to lex-smallest first. Decomposition runs on the **raw sketch**
(not the inset), so ids stay invariant when params like `stall-depth`,
`aisle-width`, or `site-offset` change the inset distance. The
single-region case (no partitions or no enclosed sub-region) gets a
reserved fallback id so pre-existing `RegionOverride`s on a one-region
lot keep resolving.

### 6.3 When controls appear

- **Object mode (sketch selected)**: sidebar shows `ParkingV2Config`
  as scrubbable inputs, three action buttons (*Add drive line*, *Add
  zone divider*, *Add stall modifier*), and `baked.metrics`.
- **Edit mode**: a layered `ParkingV2ManipulationTool` renders
  handles on `baked.aisleNetwork` edges/vertices, drive-line
  endpoints, and region control vectors. `F` cycles direction on a
  selected edge; `Delete` removes an edge/vertex. Each writes an
  annotation; regen runs; annotations re-match by grid-space ref
  against the fresh `aisleNetwork`.

Mirrors the existing `StructuralGridManipulationTool` layering.

### 6.4 Intersecting building geometry

Structured garages have columns, cores, and building footprints
punching through the lot. The parking sketch owns only the user-drawn
lot boundary; at regen time the pipeline fetches current projections
of adjacent building geometry and injects them as holes for the run.
This requires the geometry engine to support computations whose
inputs span multiple elements that aren't parented to each other. The
pipeline itself is unchanged — it already takes `{boundary, holes}`.
