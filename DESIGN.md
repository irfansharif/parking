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
representing the parking site, with zero or more CW inner polygons representing
building footprints or other obstacles. Edges of both the outer boundary and the
holes can optionally carry a circular arc, allowing curved site boundaries.
Curved edges are discretized into dense polylines at the top of the pipeline, so
all downstream geometry operates on straight segments only.

### 1.2 Drive-Aisle Graph

The structural primitive is a planar graph of **vertices** and **edges**.
Vertices are 2D points. Each edge carries some metadata:

- **Interior flag** — distinguishes interior aisles (generated within the site)
  from perimeter aisles (tracing the boundary/hole loops).
- **Direction** — Edge endpoints (start, end) are stable and never reordered to
  encode direction. One of `TwoWay(Forward|Reverse)`, or
  `OneWay(Forward|Reverse)`, where `Forward`/`Reverse` is relative to the edge's
  start→end. Direction is carried through edge provenance onto adjacent parking
  bays, where it informs stall placement: which sides of an aisle gets stalls,
  and which way angled stalls lean.

Each vertex is tagged **perimeter** (pinned to the inset boundary and hole
loops, brown dots below) or **interior** (blue dots in the figure below). Only
perimeter vertices can be moved around manually in the prototype, effectively
reshaping the site/holes.

![Underlying edges and vertices, with the red edges indicating the perimeter aisles. We also see the edge provenance in play, showing each side of the individual parking bays coloured depending on the corresponding aisle (blue for interior edges therefore containing angled stalls, yellow otherwise, and pink of representing some wall where there’s no aisle entry).](attachment:77214313-a5a1-4f70-a6b1-7d8ecf494600:image.png)

Underlying edges and vertices, with the red edges indicating the perimeter
aisles. We also see the edge provenance in play, showing each side of the
individual parking bays coloured depending on the corresponding aisle (blue for
interior edges therefore containing angled stalls, yellow otherwise, and pink of
representing some wall where there’s no aisle entry).

### 1.3 Drive Lines and Regions

**Drive lines** are user-drawn line segments that cut through the site, creating
additional driving aisles (think cross-cuts). They're clipped to the inset
boundary and spliced into the aisle graph using interior edges and vertices.
Each drive line carries a `partitions` flag — when set, the line participates
in region decomposition; when unset, it's a corridor that lives in the aisle
graph but doesn't divide space.

**Regions** are the bounded faces of the planar arrangement of `{outer
boundary ∪ hole boundaries ∪ partitioning drive lines}`. Holes aren't special:
they're arrangement edges like anything else. A dangling partition (one that
doesn't close a cycle) is inert by construction — no new face. Each region
gets its own aisle angle and offset, overridable through a "control vector".
Region centers are computed via polylabel (pole of inaccessibility) so the
control vector sits inside the region even for concave or ring-shaped ones.

### 1.4 Stall Modifiers

Stall modifiers are user-drawn geometry, applied as a post-pass against placed
stalls and faces. We'll support two kinds:

- **Suppress** — removes any stall whose geometry overlaps the modifier
  (useful for clearing entrances, fire lanes, or any corridor that should
  remain unobstructed).
- **StallType** — retypes overlapping stalls (ADA, EV, compact, etc.).

The modifier geometry is a polyline; a single point is the zero-length case
(e.g., place one ADA stall here). Modifiers stay in world-space and don't
follow grid transforms — suppressing a fire lane shouldn't move when the user
nudges aisle-angle.

![Drive lines, with partitioning ones dividing the space into independent regions (colored). Each region has a “control vector” to adjust angle and offset.](attachment:79be9336-9a37-4bc3-977c-f52c3d218995:image.png)

### 1.5 Annotations

Annotations are spatial intents that survive graph regeneration. They’re tied to
the underlying grid itself (i.e. specific edges and vertices), and they remain
valid when the graph topology changes. Edge-targeted annotations carry a `chain`
flag based on selection scope (single-click = collinear chain, double-click =
single segment). Three core kinds:

- **Direction** — (edge-targeted) sets the `AisleDirection` on matched edge(s).
  The annotation carries a travel direction vector and the desired direction
  kind; application resolves `Forward`/`Reverse` relative to each edge's
  start→end and writes the result to edge metadata. Suppresses stalls on the
  left side of travel for `OneWay`; controls stall angle lean for both `OneWay`
  and `TwoWay`.
- **DeleteVertex** — removes a vertex and all incident edges.
- **DeleteEdge** — (edge-targeted) removes an edge or collinear chain. Combined
  with DeleteVertex, these let the user carve custom face shapes.

The grid lines seen below, those that form the driving aisles, are a
transformation of a unit grid in an infinite space, rotated/stretched/translated
back out to our 2D canvas, with a fixed coordinate being the reference point
across the two. The annotations are stored referring to that fixed unit grid, so
our annotations “follow” the transformations.

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
  ├─ 3. Aisle polygon construction (edges → merged polygons)
  │     - Extrude each edge into a rectangle, fill miter
  │       wedges at junctions, boolean union into merged
  │       driveable surfaces.
  │
  ├─ 4. Parking bay extraction (boundary − aisles − holes)
  │     - Produces the "negative space" polygons where
  │       stalls will be placed.
  │
  ├─ 5. Edge provenance tagging
  │     - Classify each face edge as wall or aisle. Carries
  │       direction, interior/perimeter flag, and suppression
  │       state through to downstream decisions (boundary
  │       classification, stall suppression, spine weighting).
  │
  ├─ 6. Spine generation (aisle edges → placement
  │     centerlines)
  │     - Offset each aisle-facing edge inward by
  │       stall-reach, clip to the face polygon. Post-process:
  │       dedup, merge collinear, extend to face boundary,
  │       filter short spines.
  │
  ├─ 7. Stall placement (grid-locked fill along spines)
  │     - Fill each spine with stalls at stall-pitch
  │       intervals, with angle braiding and grid alignment.
  │       Opposing spines grid-lock to each other.
  │
  ├─ 8. Stall filtering
  │     - Face clipping (boolean containment), conflict
  │       removal within each face, suppression line
  │       filtering, short segment removal.
  │
  └─ 9. Island computation (face − stalls → landscape gaps)
        - Mark every Nth stall as an island gap.
          Boolean-subtract non-island stalls from face
          polygons to produce landscape contours.
  │
Output (ParkingLayout)

```

## 3. Pipeline Details

### 3.1 Aisle Graph Construction

Given a boundary polygon with holes, we produce a regular grid of driving
aisles in two phases.

**Phase A — auto-generate.**

1. Apply site offset (inset the outer boundary).
2. Snap hole vertices onto nearby perimeter edges (within `aisle-width`). When
   a building sits close to the boundary, this collapses near-duplicate
   parallel edges that would otherwise produce unusable sliver faces.
3. Decompose the site into regions (bounded faces of the planar arrangement of
   outer boundary, hole boundaries, and partitioning drive lines). For each
   region:
    - Determine the dominant aisle angle (e.g. from the longest hole edge, or
      a global fallback). User-overridable per region.
    - Compute how wide the region is in the direction perpendicular to the
      aisles, to determine how many rows fit.
    - Space grid lines at intervals of `2 * stall-reach + aisle-width`, where
      `stall-reach = stall-depth * sin(stall-angle) + cos(stall-angle) *
      stall-width / 2` is the perpendicular depth a stall occupies from the
      aisle edge.
    - For each grid line, intersect with the region face to get enter/exit
      intervals.
    - Record where grid lines cross perimeter/hole edges (split points).
4. Generate perpendicular cross-aisles at user-configured stall intervals. If
   both main and cross aisles exist, split edges at their intersections.
5. Build perimeter edges from boundary/hole loops, splitting at grid
   intersections.
6. Build interior edges for each grid and cross-aisle segment.

**Phase B — apply edits.**

1. (**Later**) Merge any user-edited manual graph: auto edges that overlap
   manual aisles are filtered out.
2. Splice in non-partitioning drive lines: clipped to the inset boundary minus
   holes, clamped to the nearest auto-graph edge crossings at each end, and
   merged into the graph (snapping endpoints to nearby vertices or splitting
   existing edges to create junctions).
3. Apply annotations — direction and deletion — to produce the resolved
   graph.

### 3.2 Aisle Polygon Construction

Each undirected aisle edge becomes an **aisle rectangle** — the edge offset by
`aisle-width / 2` on each side. At vertices where two or more edges meet,
**miter fill wedges** are computed: for each pair of consecutive edges (sorted
by outgoing angle), intersect their offset lines and create a triangular or
quadrilateral fill. Acute-angle miters are capped to prevent spikes. (**Later**)
We can add support for curved drive aisles by discretizing it into a dense
polylines.

![Red polygons show the miter fills generated off of perimeter edges.](attachment:108b4a69-fd1e-458e-9182-09c48d74c76d:image.png)

Red polygons show the miter fills generated off of perimeter edges.

![Red polygons show the miter fills generated off of perimeter edges. They particularly help around points where two edges meet at near perpendicular angles, so we get proper cornering.](attachment:bc8f5b5d-8a6c-4f42-9992-b1aedf56a520:image.png)

Red polygons show the miter fills generated off of perimeter edges. They
particularly help around points where two edges meet at near perpendicular
angles, so we get proper cornering.

All aisle rectangles and miter fills are boolean-unioned into merged aisle
polygons. Post-union cleanup:

- **Spike removal** — clean up vertices where the path doubles back on itself
  (anti-parallel edges, dot product less than −0.95).
- **Contour simplification** — Ramer-Douglas-Peucker to remove vertices
  contributing less than a tolerance of perpendicular distance.
- **Hole filtering** — removes small holes (area less than max-aisle-width²)
  that are junction artifacts.

### 3.3 Parking Bay Extraction

Parking bays are the positive-space regions between drive aisles where stalls
will be placed. They're computed by subtracting the merged aisles and the
building footprints (holes) from the site boundary. The end state is a set of
polygons-with-holes, each representing a parking bay.

### 3.4 Edge Provenance

After extracting parking payes, each edge of each bay is tagged with its source:

- **Wall** — the face edge lies on the site boundary or a building footprint.
- **Aisle** — the face edge lies on a drive aisle. Tagged with the specific
  aisle index, whether it's interior or perimeter, the travel direction (for
  one-way/oriented aisles), and whether it's two-way-oriented.

Edges that straddle the boundary between aisle and wall are split via binary
search at the transition point. The resulting tagged edge carries per-edge
provenance that informs all downstream decisions:

- **Boundary classification** — a face with any wall edge (or only perimeter
  aisle edges) is flagged as a **boundary face**. This flag drives a stall-angle
  policy applied during stall placement (see §3.6).
- **Stall direction handling** — on one-way aisles, stalls on each side are
  angled with the travel direction in mind. On two-way-oriented aisles the
  orientation determines which side gets which angle lean.
- **Spine weighting** — wall edges get skeleton weight 0 (fixed), aisle edges
  get weight 1 (shrink normally).

![Actual parking bays extracted after having subtracted the merged aisles. It also shows edge provenance by the side colors, representing walls (pink), interior (blue) or perimeter (yellow) sides. Also, there’s a hatched pattern around the”boundary” faces — those that have no interior sides.](attachment:5ee668ab-293a-4f81-9a38-c938b65551a4:image.png)

Actual parking bays extracted after having subtracted the merged aisles. It also
shows edge provenance by the side colors, representing walls (pink), interior
(blue) or perimeter (yellow) sides. Also, there’s a hatched pattern around
the”boundary” faces — those that have no interior sides.

### 3.5 Spine Generation

Spines are the centerlines along which stalls are placed. They're extracted from
the **weighted straight skeleton** of each face — aisle-facing edges have weight
1.0 (shrink normally) while wall edges have weight 0.0 (stay fixed).

The straight skeleton is an event-driven medial axis. Each polygon edge shrinks
inward at a rate determined by its weight. Two kinds of events: **edge events**
(two adjacent wavefront vertices converge, collapsing an edge) and **split
events** (a reflex vertex hits a non-adjacent edge, dividing the polygon). The
skeleton records these events as a timeline.

The wavefront is extracted at `stall-reach` — one stall depth in from the aisle.
Each segment of the wavefront that corresponds to an aisle-facing edge becomes a
**spine**. The spine's outward normal points toward the aisle it serves.

![Underlying weighted straight skeletons with edge collapse events (yellow dots), split events (pink squares), spines, spine extensions (described below), and also for asymmetric parking bays like those on the right, we generate one-sided spines (described below).](attachment:81194aec-d337-47ec-ba25-2ee73abfb180:image.png)

Underlying weighted straight skeletons with edge collapse events (yellow dots),
split events (pink squares), spines, spine extensions (described below), and
also for asymmetric parking bays like those on the right, we generate one-sided
spines (described below).

We do some additional spine post-processing, in two groups.

- **Validity passes** — drop or repair spines that won't yield usable stalls:
    - **Clipping** — spines are clipped to the face interior, with a dual-clip that
      also validates the stall-reach offset has depth clearance.
    - **Deduplication** — overlapping collinear spines are trimmed to their
      non-overlapping tails.
    - **Merging** — collinear touching spines with matching normals are joined.
    - **Short spine filter** — spines that won't fit at least 3 stalls (length
      shorter than `stall-reach` or insufficient for `3 × pitch`) are removed.

- **Shaping passes** — handle awkward face geometry and reach into corners:
    - **Narrow face handling** — if the face is too narrow for a wavefront at the
      target depth (e.g., a face between two close aisles), the system suppresses
      one aisle-facing edge at a time, re-runs the skeleton, and collects one-sided
      spines.
    - **Spine extensions** — Interior spines are extended colinearly to the face
      boundary in both directions. Extensions use dual-clipping (both spine and
      stall-reach positions must be inside the face) and a margin of 1.5x stall
      pitch to prevent sliver islands. Extensions are placed greedily in
      longest-first order, skipping any that conflict with already-placed stalls.

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
3. **Depth direction** (angle braiding): `depth_dir = normal * sin(stall_angle)
   ± edge_dir * cos(stall_angle)`. At 90° this is purely perpendicular to the
   spine. At 45° the stall leans along the spine direction. The sign of the
   cosine term is determined by travel direction: it flips so that stalls on
   opposite sides of a two-way aisle lean in opposite directions (herringbone),
   while stalls on both sides of a one-way aisle lean the same way (matching
   traffic flow).
4. **Stall quad corners**:
    - Back corners (at the spine): `mid ± width-dir * (stall-width / 2)`
    - Aisle corners (at the drive aisle): `depth-dir` projection at `stall-reach
      / sin(stall-angle)`, spread by `± edge-dir * (pitch / 2)`.
    - Quads are emitted closed, tagged with which side is the aisle entrance.
5. **Grid alignment**: Stalls snap to an absolute grid along the spine
   direction. A centering shift distributes remaining space symmetrically.
   Opposing spines (back-to-back across a face) grid-lock to each other's
   shift, or negate it (mod pitch) for opposite orientations.

![Stall placement within each parking bay with respect to underlying spines, and following driving aisle directions (sometimes stall strips are suppressed accordingly). Stalls are also actually centered on the spine somewhat, and we try to place islands in opposing spines in way such that they’re aligned.](attachment:b5a72bb3-2a4c-4369-a8db-097625acc1a8:image.png)

Stall placement within each parking bay with respect to underlying spines, and
following driving aisle directions (sometimes stall strips are suppressed
accordingly). Stalls are also actually centered on the spine somewhat, and we
try to place islands in opposing spines in way such that they’re aligned.

### 3.7 Stall Filtering

After placement, stalls go through several filters:

- **Face clipping** — stalls not fully contained within their face are removed
  (boolean intersection of stall quad with face polygon; remove if intersection
  area less than stall area). Since faces are already clipped to the site boundary minus
  holes, this subsumes boundary clipping.
- **Conflict removal** — within each face, overlapping stalls are detected. On
  interior faces the stall from the shorter spine is removed. On boundary faces
  both conflicting stalls are removed (makes for more symmetrical corners).
- **Suppression filter** — removes stalls whose geometry overlaps any
  `Suppress` stall modifier (see §1.4).

### 3.8 Island Computation

Islands are landscape gaps between and at the ends of stall rows. They're
produced by:

1. **Mark island stalls** — every Nth stall (by `island-stall-interval`) is
   marked as `StallKind::Island`, using absolute position-based grid marking
   (`floor(proj / pitch) % interval == interval / 2`). End margins prevent
   marking stalls at row endpoints.
2. **Boolean subtraction** — for each face, subtract all non-island stalls from
   the face polygon. The residual polygons (where island stalls were left as
   gaps) become island contours. Islands with net area less than 10 are filtered out.

    ![Actual island geometry,. In production we’ll want to add some rounding.](attachment:089bb8f8-0fe7-4829-88ea-73bebe44cfc3:image.png)

    Actual island geometry,. In production we’ll want to add some rounding.


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

 | Parameter               | Range   | Effect                             |
 |-------------------------|---------|------------------------------------|
 | `stall-width`           | 7–40 ft | Individual stall width             |
 | `stall-depth`           | 8–30 ft | Stall depth perpendicular to aisle |
 | `aisle-width`           | 8–24 ft | Full driving aisle width           |
 | `stall-angle`           | 45–90°  | Stall angle relative to aisle      |
 | `aisle-angle`           | any°    | Dominant aisle grid direction      |
 | `aisle-offset`          | any     | Perpendicular grid offset          |
 | `site-offset`           | 0–50 ft | Inset from raw boundary            |
 | `cross-aisle-max-run`   | 3–50    | Stalls between cross-aisles        |
 | `island-stall-interval` | 0–20    | Stalls between landscape islands   |

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

The prototype's `StallKind` enum (Standard, Compact, Ev, Extension, Island)
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
