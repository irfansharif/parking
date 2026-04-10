# Procedural Parking Site

This page proposes an architecture for parking-v2, a procedural parking site generation system with some additional controls we've found wanting. It's a fresh system so we'll not refer to parking-v1 much here. At a high level it'll allow for definining a parking site boundary with optional building footprints within it, and produce a complete layout -- driving aisles, stalls, islands -- through a layered pipeline of geometric operations. It'll allow for a few manual operations, operations that "persist", to continually steer the procedural generation itself.

---

## Motivation

We want to support a few additional things:
- **Drive aisle editing** — We want control over aisle layouts beyond what auto-generation produces: adding cut-through drive aisles, deleting edges/vertices, change traffic direction (one-way, two-way-oriented).
- **Angled and one-way parking** — Like 45°/60° stalls with one-way aisles, for instance.
- **Intersecting geometry** — stalls should respond to columns, cores, and other geometry that intersects the parking sketch (important for structured garages where columns punch through).
- **Region segmentation** — the ability to segment a site into regions with different aisle angles, and potentially other properties (stall sizes, angles, etc.)
- **Islands and max run** — V1 produced unbroken seas of stalls with no landscape islands. We want some max-run spacing, end-of-row islands, and corner islands.
- **Arc/curve support** — parking along curved site boundaries. And also potentially allowing inner (i.e. not hugging the site boundary) drive aisles to be curved.
- **Site offset** — distance from sketch edges to first stalls.

We built a prototype (http://irfansharif.io/parking, and http://github.com/irfansharif/parking) that tries to flesh out the core generation pipeline: boundaries with curves (arc support via Bézier edges), semi-automatic aisle graphs and with edits (add/delete edges and vertices, and direction control), region-based segmentation, angled/one-way stalls, islands with max run, corner islands, cut through drive lines, and site offsets. There are few other features we want to support, and we'll incorporate into the production integration:

- **Ramps** — sloped drive aisles and outside-edge ramps with angle-based length calculation from floor-to-floor height.
- **Custom stall types** — ADA, EV, compact stalls with user-specified counts and placement, potentially split across different buildings.

---

## 1. Core Primitives

### 1.1 Polygon with Holes

The fundamental input is a **polygon with holes**: a CCW outer boundary representing the parking site, with zero or more CW inner polygons representing building footprints or other obstacles. Each edge of both the outer boundary and the holes can optionally carry a **cubic Bézier curve** (two control points), allowing curved site boundaries.

Curved edges are discretized into dense polylines at the top of the pipeline, so all downstream geometry operates on straight segments only.

### 1.2 Drive-Aisle Graph

The primary structural primitive is a planar graph of **vertices** and **edges**. Vertices are 2D points. Each edge carries some metadata:

- **Interior flag** — distinguishes interior aisles (generated within the site) from perimeter aisles (tracing the boundary/hole loops).
- **Direction** — pure metadata; edge endpoints (start, end) are stable and never reordered to encode direction. One of `TwoWay` (no preferred direction), `TwoWayOriented(Forward|Reverse)`, or `OneWay(Forward|Reverse)`, where `Forward`/`Reverse` is relative to the edge's start→end. Direction is carried through edge provenance onto faces, where it informs stall placement: which side of an aisle gets stalls, and which way angled stalls lean.

The graph's vertices are partitioned: there are are **boundary vertices** (pinned to the inset boundary and hole loops), and the rest are interior vertices.

### 1.3 Drive Lines and Separation

**Drive lines** are user-drawn line segments that cut through the site, creating additional driving aisles (entrances, cross-cuts). They're clipped to the inset boundary and spliced into the aisle graph as interior edges.

A drive line can be **pinned** to a hole vertex at one end (making it a **separator line**) and to a boundary edge at the other. Separator lines partition the space around a building into independent **regions**, each with its own aisle angle and offset. The boundary edge can be moved around.

### 1.4 Annotations

Annotations are spatial intents that survive graph regeneration. They're matched to graph elements by proximity (point-to-segment distance), not by index, so they remain valid when the graph topology changes. Edge-targeted annotations carry a `chain` flag based on selection scope (single-click = collinear chain, double-click = single segment). Four kinds:

- **Direction** — (edge-targeted) sets the `AisleDirection` on matched edge(s). The annotation carries a travel direction vector and the desired direction kind; application resolves `Forward`/`Reverse` relative to each edge's start→end and writes the result to edge metadata. Suppresses stalls on the left side of travel for `OneWay`; controls stall angle lean for `TwoWayOriented`.
- **DeleteVertex** — removes a vertex and all incident edges.
- **DeleteEdge** — (edge-targeted) removes an edge or collinear chain.  Combined with DeleteVertex, these let the user carve custom face shapes.
- **SuppressStalls** — a freeform line drawn across the site; any stalls whose geometry overlaps the line are removed. Useful for clearing entrances, fire lanes, or any corridor that should remain unobstructed.

### 1.5 Regions

When a building has two or more separator lines, the annular space around it is decomposed into **regions**. Each region is a clipping polygon with its own aisle angle and perpendicular offset. The user can override these per-region. Region centers are computed via polylabel (pole of inaccessibility) to try and ensure the UI anchor sits inside the region even for concave or ring-shaped ones. Without separators, the entire site is a single region with the global aisle angle/offset.

---

## 2. Pipeline Overview

The generation pipeline is deterministic and stateless — given the same input, it produces the same output. Each layer's output is the next layer's input.  The layers are pure functions — no shared mutable state, no callbacks, no side effects.

```
Input (boundary, holes, curves, drive lines, annotations, params)
  │
  ├─ 1. Boundary discretization (curves → polylines)
  │     - All downstream geometry operates on straight segments only.
  │
  ├─ 2. Aisle graph construction (auto-generate, merge manual edits, splice drive lines, apply annotations)
  │     - Produces the topological structure: vertices, edges with direction and interior/perimeter metadata.
  │
  ├─ 3. Aisle polygon construction (edges → merged polygons)
  │     - Extrude each edge into a rectangle, fill miter wedges at junctions, boolean union into merged driveable surfaces.
  │
  ├─ 4. Face extraction (boundary − aisles − holes)
  │     - Produces the "negative space" polygons where stalls will be placed.
  │
  ├─ 5. Edge provenance tagging
  │     - Classify each face edge as wall or aisle. Carries direction, interior/perimeter flag, and suppression state through to downstream decisions (boundary classification, stall suppression, spine weighting).
  │
  ├─ 6. Spine generation (aisle edges → placement centerlines)
  │     - Offset each aisle-facing edge inward by stall-reach, clip to the face polygon. Post-process: dedup, merge collinear, extend to face boundary, filter short spines.
  │
  ├─ 7. Stall placement (grid-locked fill along spines)
  │     - Fill each spine with stalls at stall-pitch intervals, with angle braiding and grid alignment. Opposing spines grid-lock to each other.
  │
  ├─ 8. Stall filtering
  │     - Face clipping (boolean containment), conflict removal within each face, suppression line filtering, short segment removal.
  │
  └─ 9. Island computation (face − stalls → landscape gaps)
        - Mark every Nth stall as an island gap. Boolean-subtract non-island stalls from face polygons to produce landscape contours.
  │
Output (ParkingLayout)
```

---

## 3. Phase Details

### 3.1 Aisle Graph Construction

Given a boundary polygon with holes, we produce a regular grid of driving aisles.

1. Apply site offset (inset the outer boundary).
2. (Optional) Snap hole vertices onto nearby perimeter edges (within `aisle-width`). When a building sits close to the boundary, this collapses near-duplicate parallel edges that would otherwise produce unusable sliver faces.
3. For each region:
   - Determine the dominant aisle angle for the region (e.g. from the longest hole edge, or a global fallback). User-overridable per region.
   - Compute how wide the site is in the direction perpendicular to the aisles, to determine how many rows fit.
   - Space grid lines at intervals of `2 * stall-reach + aisle-width`, where `stall-reach = stall-depth * sin(stall-angle) + cos(stall-angle) * stall-width / 2` is the perpendicular depth a stall occupies from the aisle edge.
   - For each grid line, intersect with the outer perimeter to get enter/exit intervals. Subtract hole interiors. Clip to the region polygon if applicable.
   - Record where grid lines cross perimeter/hole edges (split points).
4. Generate perpendicular cross-aisles at user-configured stall intervals. If both main and cross aisles exist, split edges at their intersections.
6. Build perimeter edges from boundary/hole loops, splitting at grid intersections.
7. Build interior edges for each grid and cross-aisle segment.

The result is a `DriveAisleGraph` with all vertices, edges, and the perimeter vertex count. (Later) If the user has edited aisle positions (manual graph), the auto-generated graph is merged with it: auto edges that overlap manual aisles are filtered out.

Drive lines are then spliced in: clipped to the inset boundary minus holes, clamped to the nearest auto-graph edge crossings at each end, and merged into the graph (snapping endpoints to nearby vertices or splitting existing edges to create junctions). Finally, annotations are applied — direction and deletion — to produce the resolved graph.

### 3.2 Aisle Polygon Construction

Each undirected aisle edge becomes an **aisle rectangle** — the edge offset by `aisle-width / 2` on each side. At vertices where two or more edges meet, **miter fill wedges** are computed: for each pair of consecutive edges (sorted by outgoing angle), intersect their offset lines and create a triangular or quadrilateral fill. Acute-angle miters are capped to prevent spikes. (Later) We can add support for curved drive aisles by discretizing it into a dense polylines.

All aisle rectangles and miter fills are boolean-unioned into merged aisle polygons. Post-union cleanup:

- (Optional) **Spike removal** — clean up vertices where the path doubles back on itself (anti-parallel edges, dot product < −0.95).
- (Optional) **Contour simplification** — Ramer-Douglas-Peucker to remove vertices contributing less than a tolerance of perpendicular distance.
- (Optional) **Hole filtering** — removes small holes (area < max-aisle-width²) that are junction artifacts.

### 3.3 Face Extraction

Faces are the positive-space regions between drive aisles where stalls will be placed. They're computed via two-step boolean subtraction:

1. `boundary − merged-aisles` (removes drive aisles from the site).
2. `result − raw-holes` (removes building footprints).

The end state is a set of polygons-with-holes, each representing a parking bay.

### 3.4 Edge Provenance

After face extraction, each edge of each face is tagged with its **source**:

- **Wall** — the face edge lies on the site boundary or a building footprint.
- **Aisle** — the face edge lies on a drive aisle. Tagged with the specific aisle index, whether it's interior or perimeter, the travel direction (for one-way/oriented aisles), and whether it's two-way-oriented.

Edges that straddle the boundary between aisle and wall are split via binary search at the transition point. The resulting `TaggedFace` carries per-edge provenance that informs all downstream decisions:

- **Boundary classification** — a face with any wall edge (or only perimeter aisle edges) is a boundary face and gets 90° stalls regardless of the global stall angle. Interior faces use the configured angle.
- **Stall suppression** — on one-way aisles, stalls are suppressed on the left side of the travel direction. On two-way-oriented aisles, the orientation determines which side gets which angle lean. Edges with `SuppressStalls` lines suppress any overlapping stalls.
- **Spine weighting** — wall edges get skeleton weight 0 (fixed), aisle edges get weight 1 (shrink normally).

### 3.5 Spine Generation

Spines are the centerlines along which stalls are placed. They're extracted from the **weighted straight skeleton** of each face.

The straight skeleton is an event-driven medial axis. Each polygon edge shrinks inward at a rate determined by its weight. Two kinds of events: **edge events** (two adjacent wavefront vertices converge, collapsing an edge) and **split events** (a reflex vertex hits a non-adjacent edge, dividing the polygon). The skeleton records these events as a timeline.

For parking:
- Aisle-facing edges have weight 1.0 (shrink normally).
- Boundary wall edges have weight 0.0 (stay fixed).

The wavefront is extracted at `stall_reach` (with a small epsilon for numerical robustness) — one stall depth in from the aisle. Each segment of the wavefront that corresponds to an aisle-facing edge becomes a **spine**. The spine's outward normal points toward the aisle it serves.

**Narrow face handling:** If the face is too narrow for a wavefront at the target depth (e.g., a face between two close aisles), the system suppresses one aisle-facing edge at a time, re-runs the skeleton, and collects one-sided spines.

**Spine post-processing:**
- **Clipping** — spines are clipped to the face interior, with a dual-clip that also validates the stall-reach offset has depth clearance.
- **Deduplication** — overlapping collinear spines are trimmed to their non-overlapping tails.
- **Merging** — collinear touching spines with matching normals are joined.
- **Short spine filter** — spines shorter than `stall-reach` are removed.

**Spine extensions:** Interior spines are extended colinearly to the face boundary in both directions. Extensions use dual-clipping (both spine and stall-reach positions must be inside the face) and a margin of 1.5x stall pitch to prevent sliver islands. Extensions are placed greedily in longest-first order, skipping any that conflict with already-placed stalls.

(Optional) For curved face edges, we could instead consider a manual inset at `stall-reach`.

### 3.6 Stall Placement

Stalls are placed on spines:

1. **Stall pitch** = `stall-width / sin(angle)` — the spacing between stall centers along the spine.

2. **Depth direction** (angle braiding): `depth_dir = normal * sin(stall_angle) ± edge_dir * cos(stall_angle)`. At 90° this is purely perpendicular to the spine. At 45° the stall leans along the spine direction. The sign of the cosine term is determined by travel direction: it flips so that stalls on opposite sides of a two-way aisle lean in opposite directions (herringbone), while stalls on both sides of a one-way aisle lean the same way (matching traffic flow).

3. **Stall quad corners**:
   - Back corners (at the spine): `mid ± width-dir * (stall-width / 2)`
   - Aisle corners (at the drive aisle): `depth-dir` projection at `stall-reach / sin(stall-angle)`, spread by `± edge-dir * (pitch / 2)`.
   - Convention: corners [0]–[1] are the back wall, [2]–[3] are the aisle entrance. The renderer draws [3]→[0]→[1]→[2], leaving [2]→[3] open.

4. (Optional) **Grid alignment**: Stalls snap to an absolute grid along the spine direction. A centering shift distributes remaining space symmetrically.  Opposing spines (back-to-back across a face) grid-lock to each other's shift, or negate it (mod pitch) for opposite orientations.

### 3.7 Stall Filtering

After placement, stalls go through several filters:

- **Face clipping** — stalls not fully contained within their face are removed (boolean intersection of stall quad with face polygon; remove if intersection area < stall area). Since faces are already clipped to the site boundary minus holes, this subsumes boundary clipping.
- **Conflict removal** — within each face, overlapping stalls are detected.  On interior faces the stall from the shorter spine is removed. On boundary faces both conflicting stalls are removed (makes for more symmetrical corners).
- **Short segment filter** — removes all stalls on spines with fewer than 3 total stalls.
- **Suppression filter** — removes stalls whose geometry overlaps any `SuppressStalls` line.

### 3.8 Island Computation

Islands are landscape gaps between and at the ends of stall rows. They're produced by:

1. **Mark island stalls** — every Nth stall (by `island-stall-interval`) is marked as `StallKind::Island`, using absolute position-based grid marking (`floor(proj / pitch) % interval == interval / 2`). End margins prevent marking stalls at row endpoints.

2. **Boolean subtraction** — for each face, subtract all non-island stalls from the face polygon. The residual polygons (where island stalls were left as gaps) become island contours. Islands with net area < 10 are filtered out.

---

## 4. Key Geometric Algorithms

### 4.1 Weighted Straight Skeleton

The skeleton uses per-edge weights to handle asymmetric shrinkage. An edge at offset distance `d` moves to:

```
point + normal * d * weight
```

Weight 0 (walls) doesn't move; weight 1 (aisles) shrinks normally. This produces medial axes that respect the constraint that stalls only grow from aisle-facing edges.

Events are computed by projecting vertex velocities and solving for collision times. Edge events (adjacent vertices converge) eliminate edges. Split events (reflex vertex hits non-adjacent edge) divide the polygon. The wavefront at any distance `d` is reconstructed by intersecting offset lines of surviving edges.

### 4.2 Boolean Operations

The system uses `i_overlay` with NonZero and EvenOdd fill rules for:
- Self-union of aisle shapes (aisle polygon merging).
- Subtraction of aisles and holes from the boundary (face extraction).
- Subtraction of stalls from faces (island computation).

Consistent winding (CCW outer, CW holes) is enforced before each operation.

### 4.3 Polygon Inset/Offset

Raw inset computes miter vertices by intersecting adjacent edge offset lines.  For non-convex polygons this can self-intersect, so derived holes use boolean intersection with the original ring to resolve. An iterative edge-collapse-aware variant (`inset-per-edge-core`) detects edges that flip direction during inset and removes them.

---

## 5. User Interaction Model

The user draws inputs (boundary, holes, drive lines) and then manipulates the generated result. All interactive elements — vertices, edge midpoints, control points, annotation markers, region vectors — are unified as draggable/clickable anchors with a single hit-test mechanism. Annotations (direction, delete, suppress) are placed on whatever is currently selected; they persist as spatial anchors that survive graph regeneration, re-matching to nearby graph elements by proximity after each rebuild. For example, pressing F on a selected edge cycles through direction modes. Each region has a draggable vector that controls both the aisle angle (rotate the endpoints) and perpendicular offset (drag the body).

Every edit triggers full regeneration: the input state is serialized, sent to the engine, and the output is re-rendered. The pipeline is fast enough for interactive use.

### 5.1 Parameters

 | Parameter               | Range   | Effect                                                  |
 |-------------------------|---------|---------------------------------------------------------|
 | `stall-width`           | 7–40 ft | Individual stall width                                  |
 | `stall-depth`           | 8–30 ft | Stall depth perpendicular to aisle                      |
 | `aisle-width`           | 8–24 ft | Full driving aisle width (same for one-way and two-way) |
 | `stall-angle`           | 45–90°  | Stall angle relative to aisle                           |
 | `aisle-angle`           | any°    | Dominant aisle grid direction                           |
 | `aisle-offset`          | any     | Perpendicular grid offset                               |
 | `site-offset`           | 0–50 ft | Inset from raw boundary                                 |
 | `cross-aisle-max-run`   | 3–50    | Stalls between cross-aisles                             |
 | `island-stall-interval` | 0–20    | Stalls between landscape islands                        |

---

## 6. Production Integration Considerations

### 6.1 Intersecting Geometry

In structured garages, columns and cores punch through parking floors. The system's polygon-with-holes model already handles this — columns become additional holes in the boundary input. We can project intersecting 3D geometry onto the parking plane and inject them as holes before generation. The pipeline's face extraction and stall clipping will handle the rest.

### 6.2 Custom Stall Types and Placement

The prototype's `StallKind` enum (Standard, Compact, Ev, Extension, Island) provides the hook. In production we want something akin to:
- User-specified counts per type (not auto-calculated — letting users do the math themselves and input counts directly).
- Drag-to-place for ADA and specialty stalls rather than control-point-based placement. The annotation system's proximity-matching pattern is the right model: place a "stall type override" annotation at a position, match it to the nearest stall(s) at generation time. We can start off with a best-effort default guess but lets users manipulate to their needs.
  - Stall groups are simply individual stalls underneath, so can be split across buildings (e.g., ADA stalls distributed near multiple entrances, not blocked together). 

### 6.3 Ramps

Two types. **Sloped drive aisles** are aisle edges with additional metadata (grade, suppress stalls on adjacent faces). The aisle polygon layer extrudes them normally; the stall layer skips faces adjacent to ramp edges. Both need a ramp angle parameter and auto-calculated length from floor-to-floor height.

**Outside-edge ramps** are dedicated ramp geometry attached to the site boundary — think a helix or switchback on the exterior of a structured garage. These aren't aisle edges in the graph; they're boundary features defined by a start/end point along the perimeter, a width, and a grade.  The ramp polygon is subtracted from the boundary (like a hole) so stalls and aisles don't encroach, and the ramp surface itself is emitted as standalone geometry with slope metadata for 3D extrusion.

### 6.4 Bake to Finalize

"Baking" converts generated geometry into serializable sketch primitives. The output already contains all necessary geometry (stall quads, island contours, aisle polygons). Baking means writing these back as uneditable geometries into the project file and detaching them from the generation pipeline. We want stability across project opens even if the generation code itself changes. But allow for someway to re-enter the edit mode and trigger re-generation off of input elements again.

### 6.5 Metrics and Labels

The prototype outputs `total-stalls`. Production should add per-type counts, parking ratio (stalls per unit area), and utilization percentage. Stall run labels (e.g., "12 stalls" per row) can be derived from the spine data — each spine's stall count is known after placement, and the spine's midpoint gives the label position.