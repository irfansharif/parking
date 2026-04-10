# Parking Generation Design

## Purpose

This document distills the parking lot generator prototype into a design for a production-quality procedural parking system that can later live inside a larger BIM application such as Arcol.

The goal is not to preserve the prototype's UI or file layout. The goal is to preserve its core ideas:

- Model parking generation as a stateless geometry pipeline.
- Persist user intent as a small set of authoring primitives.
- Recompute the full layout from those primitives on every edit.
- Carry provenance through the pipeline so downstream stages can make smarter decisions.
- Expose enough debug geometry to make the system understandable and debuggable in a host app.

## Design Principles

1. **Intent in, geometry out**

The host app should store user-authored intent: boundary, holes, separators, drive lines, aisle graph edits, direction overrides, and parameters. Everything else should be derived.

2. **Single regeneration pipeline**

The system should fully regenerate from canonical input on every meaningful edit. This keeps the implementation simpler and more correct than trying to update derived geometry incrementally.

3. **One geometric language**

The entire system should reduce to a few reusable primitives: vertices, edges, polygons-with-holes, and derived faces/spines/stalls. Even special features like separators and drive lines should reuse the same primitive when possible.

4. **Provenance matters**

Face edges must remember where they came from: boundary, hole, interior aisle, perimeter aisle, and sometimes travel direction. Stall placement and later classification depend on that metadata.

5. **Curves are authoring-time, polylines are runtime**

Users may author curved boundary and hole edges as cubic Beziers. Early in the pipeline, those curves should be discretized into polylines so the rest of the geometry engine can operate on a consistent representation.

6. **The host app owns interaction**

Arcol should own selection, snapping, handles, overlays, editing history, persistence, and BIM integration. The parking engine should own geometry generation.

## High-Level Architecture

The system naturally separates into three layers:

1. **Authoring layer**
   The host app stores editable parking intent and presents tools for manipulating it.

2. **Generation engine**
   A stateless geometry engine consumes authoring input and produces a complete parking layout plus debug/provenance outputs.

3. **Presentation layer**
   The host app renders the result, surfaces diagnostics, and lets users convert visual feedback back into intent edits.

The prototype already points toward this architecture: a thin TypeScript UI around a Rust/WASM engine with a JSON input/output boundary. That is a good shape to preserve.

## Canonical Input Model

The engine should consume a single `GenerateInput`-style object. Conceptually, it contains the following:

### 1. Site boundary

The parking lot starts as a polygon with holes:

- `outer`: the outer lot boundary
- `holes`: inner voids, usually buildings or reserved areas
- optional per-edge Bezier curve data on both outer and hole loops

This is the top-level feasible area from the user's perspective.

### 2. Drive aisle graph

The main circulation primitive is a graph:

- `vertices`
- `edges`

Each edge represents a directed aisle segment with metadata such as:

- width
- whether it is interior vs perimeter-derived
- directionality: `OneWay`, `TwoWay`, or `TwoWayOriented`

An important design choice from the prototype is worth preserving: treat bidirectional circulation as the same primitive as one-way circulation, not as a completely different type of object. In practice, that means all aisle behavior is still edge-based, and a two-way aisle may still be represented as a paired or semantically bidirectional edge construct rather than a separate geometric family.

### 3. Drive lines and separators

The prototype has an important unification: separator lines and drive lines are really the same primitive.

They are user-authored line segments that cut through the lot. Depending on how they are pinned and interpreted, they serve two roles:

- **separator lines**
  Usually start from a hole edge or hole vertex and partition the lot into independent regions.

- **drive lines**
  Represent through-lot drives or entrances that carve out circulation corridors and suppress stalls where they pass.

These should remain one shared primitive with different downstream interpretations.

### 4. Region overrides

Each independent region may override:

- aisle angle
- aisle offset

This lets the system produce different aisle grids in different parts of the lot while still participating in one global solve.

### 5. Anchors and annotations

The host app needs an overlay editing model for non-destructive user manipulations. The prototype expresses this as persistent annotations:

- delete vertex
- delete edge
- one-way assignment
- two-way-oriented assignment
- chain-based propagation along collinear edges

These are not final geometry. They are intent overlays that get replayed during graph resolution.

This is the right abstraction for a BIM host app: store user overrides as durable semantic edits, not as ad hoc mutations to downstream geometry.

### 6. Parking parameters

The core parameters include:

- stall width
- stall depth
- aisle width
- stall angle
- aisle angle
- aisle offset
- site offset
- cross-aisle spacing / max run
- island interval and related spacing controls

These parameters define the lattice from which parking geometry is derived.

## Canonical Output Model

The engine should return both final geometry and intermediate diagnostic geometry.

### Final geometry

- merged aisle polygons
- resolved drive aisle graph
- faces / parking bays
- stall quads with kind metadata
- island polygons
- metrics

### Diagnostic geometry

- miter fills
- spine lines
- skeleton debug geometry
- derived raw outer boundary and raw hole polygons
- region debug geometry

### Provenance metadata

Each face should retain per-edge provenance such as:

- boundary wall
- hole wall
- interior aisle edge
- perimeter aisle edge
- adjacent travel direction where relevant

This provenance is essential. It is how downstream logic distinguishes a boundary face from an interior bay and determines what kinds of stall placement are valid.

## Core Geometric Model

The prototype suggests the following conceptual ladder:

1. **Site polygon**
   The user-authored outer boundary and holes.

2. **Partition lines**
   Separators and drive lines, some pinned to holes or boundary edges.

3. **Drive aisle graph**
   A graph of vertices and directed edges defining circulation structure.

4. **Corridors**
   Thickened aisle-edge geometry, including join treatment.

5. **Faces**
   Positive-space polygons left over after subtracting corridors from the site.

6. **Spines**
   Internal guide lines inside a face that determine parking strip placement.

7. **Stalls**
   Final stall quads, optionally including extension stalls and separator stalls.

8. **Islands**
   Residual geometry after subtracting stalls from the face, plus explicit parking-island logic.

This is the right mental model for the whole system.

## End-to-End Pipeline

### Stage 0: Normalize authored input

Before any parking logic runs:

- validate rings and graph references
- resolve pinned line endpoints
- discretize Bezier boundary/hole edges into polylines
- normalize winding and remove obviously degenerate geometry

Everything downstream should operate on straight segments.

### Stage 1: Resolve the drive aisle graph

The system starts from either:

- a fully manual graph
- an automatically generated graph
- or a hybrid of both

The graph resolution stage should:

- auto-generate base aisle rows when needed
- merge manual edges with auto-generated structure
- append drive-line-derived edges
- split edges at crossings
- deduplicate overlapping or equivalent edges
- apply direction and deletion annotations

This stage produces the canonical `resolved_graph`.

### Stage 2: Derive raw lot and raw hole geometry

The prototype distinguishes between authored aisle-edge loops and the raw lot/building outlines used for clipping.

This is important enough to preserve as an explicit stage:

- expand the perimeter aisle-edge loop outward to recover the raw outer lot boundary
- shrink hole-adjacent aisle-edge loops inward to recover raw building footprints
- apply site offset
- resolve miter and concavity artifacts during inset/outset operations

This derived geometry is later used for face extraction, boundary clipping, and diagnostic rendering.

### Stage 3: Partition into regions

Separator lines divide the lot into independent regions, usually around holes.

Each region carries:

- a clip polygon
- an aisle angle
- an aisle offset

This stage lets different subareas receive different parking orientation while still participating in the same larger lot.

Conceptually, region decomposition should happen before grid generation, because the grid is region-specific.

### Stage 4: Generate the region aisle lattice

Within each region, generate a regular aisle lattice from the controlling parameters:

- stall angle
- stall depth
- aisle angle
- aisle offset
- aisle width
- cross-aisle spacing / max run

This stage lays out the regular network of edges and vertices that defines circulation. The result is still graph-first, not stall-first.

The key idea is:

- the user defines the site and partitions
- the engine fills each region with a regular graph aligned to the chosen parking logic

### Stage 5: Build corridor geometry from graph edges

Each aisle edge becomes corridor geometry by thickening the edge according to aisle width.

At this point the engine must handle the ugly but essential join logic:

- corridor merging
- miter fills
- spike removal
- contour simplification
- hole filtering

This stage converts a graph into a clean circulation polygon set.

### Stage 6: Extract faces

Subtract the merged corridor geometry from the lot polygon-with-holes:

`faces = site - aisles`

The remaining connected polygons are the positive-space faces that can host parking bays or islands.

This is the most important topological transition in the pipeline:

- before this point, the engine is mostly reasoning about circulation
- after this point, it is reasoning about parking bays

Face extraction should also compute edge provenance while the relationship between faces and corridors is still fresh.

### Stage 7: Tag and classify face edges

For each face edge, record where it came from:

- wall / site boundary
- hole boundary
- perimeter aisle edge
- interior aisle edge
- travel direction and orientation hints where needed

This provenance drives several downstream rules:

- identify boundary faces
- avoid unsupported stall angles on boundary-constrained faces
- determine which sides are aisle-facing vs wall-facing
- support one-way and angled stall behavior
- distinguish parking-worthy faces from residual slivers

Without this stage, later heuristics become brittle.

### Stage 8: Generate spines inside faces

The prototype uses straight skeletons and related inset logic to turn each face into one or more spine segments.

This stage should:

- simplify the face when useful
- classify shrinking vs fixed edges
- compute a weighted straight skeleton or equivalent wavefront
- extract candidate spines at the effective parking depth
- clip spines to valid subregions

The straight skeleton is valuable because it adapts to irregular faces instead of assuming a rectangular bay.

### Stage 9: Post-process spines

Raw spines are not yet fit for stall placement. They need substantial cleanup:

- spine clipping
- spine post-processing
- spine dedup
- spine merging
- short spine filter
- spine extensions

Spine extensions are especially important: they allow the system to opportunistically fit more stalls by extending strips to face boundaries when geometry permits.

This is where the engine turns topological candidates into practical parking strips.

### Stage 10: Place stalls

Stall placement happens along spines, not directly against arbitrary face boundaries.

Core logic includes:

- compute stall pitch from stall width and stall angle
- center the run where appropriate
- account for aisle directionality on angled stalls
- place regular stall quads
- place extension stalls
- place separator stalls where needed

This stage should remain strongly parameterized and deterministic.

### Stage 11: Clip and clean stalls

Raw stall placement must be validated and cleaned:

- stall-to-face clipping
- boundary clipping
- conflict removal
- short segment filter

The prototype also uses surrounding face geometry and provenance to decide when particular placements are invalid, especially near boundaries and constrained faces.

This is the stage where "generated candidates" become "accepted stalls."

### Stage 12: Generate islands

After stalls are accepted, subtract them from the surrounding face geometry to produce islands and residual landscape geometry.

This yields:

- endcaps
- corner islands
- interior residual islands
- other non-stall positive-space leftovers

In other words:

`islands = face - stalls`

This is a strong design idea from the prototype. Instead of hardcoding every island type first, the system can derive much of the island geometry from subtraction after stall placement, while still allowing explicit island rules where necessary.

### Stage 13: Assemble output

The final output should include:

- final stalls
- islands
- merged aisle polygons
- resolved graph
- faces with provenance
- spines
- metrics
- debug geometry

The host app can then choose which layers to render or persist.

## Why Face Provenance Is a First-Class Concern

One of the most important lessons from the prototype is that face provenance should not be treated as a debug-only detail.

It is needed to:

- identify whether a face is a boundary face
- know which face edges are adjacent to a particular aisle edge
- inherit travel direction into adjacent angled stall placement
- restrict or alter placement modes on boundary-constrained geometry
- distinguish parking edges from wall edges
- produce better downstream editing and diagnostics in the host app

In a BIM context, provenance also opens the door to more semantic behavior later:

- tagging stalls by access condition
- differentiating parking adjacent to buildings vs circulation
- computing code-related metrics from edge context

## Recommended Module Structure

The prototype's exact file names are not the point, but the decomposition is good. A production engine should likely separate concerns roughly as follows:

### Input and schema

- shared geometry types
- serialization contract
- parameters
- annotations

### Authoring normalization

- curve discretization
- ring validation
- graph validation
- pin resolution

### Graph resolution

- auto-generation
- manual/auto merge
- annotation application
- drive-line incorporation

### Regioning

- separator interpretation
- region decomposition
- region override application

### Corridor geometry

- corridor thickening
- miter fill generation
- corridor merge and simplification

### Face extraction and provenance

- boolean subtraction
- face extraction
- edge provenance tagging
- face classification

### Spine generation

- face simplification
- edge weighting
- straight skeleton / wavefront solve
- spine extraction and clipping

### Stall placement

- strip fill
- centering
- angle-aware placement
- direction-aware placement
- extension placement
- separator stall rules

### Cleanup and island generation

- clipping
- conflict removal
- short-run filtering
- island derivation

### Diagnostics

- debug toggles
- intermediate geometry outputs
- deterministic fixture export

## Recommended Host-App Boundary for Arcol

Arcol should own:

- editable curves, polygons, and pins
- handle and anchor UX
- snapping
- layer visibility
- command history / undo-redo
- object persistence
- BIM integration and downstream metadata

The parking engine should own:

- graph resolution
- topology
- booleans
- skeletons
- spine generation
- stall placement
- island derivation

The clean integration contract is:

`authoring state -> generate() -> derived parking layout`

The host app should avoid mutating derived geometry directly. If a user wants to "delete an aisle edge" or "flip a one-way direction," that should be stored as an annotation or graph edit and then replayed through regeneration.

## Persistence Model

The host app should persist the smallest stable set of editable inputs:

- boundary outer loop and holes
- curve controls
- drive lines / separators
- manual aisle graph edits
- region overrides
- annotations / anchors
- parking parameters

It should not persist:

- spines
- merged corridors
- faces
- stalls
- islands

Those are all derived outputs and should be regenerated.

## Determinism and Testing

The prototype's testing approach is worth preserving because procedural geometry systems are hard to reason about without layered verification.

The system should support at least three test surfaces:

1. **metrics assertions**
   Stable high-level checks such as stall counts.

2. **geometry state assertions**
   Deterministic JSON or equivalent structured output for exact comparisons.

3. **visual regression**
   Rendered image comparison for layout-level regressions.

The engine should also preserve a way to export a live interactive state into a deterministic fixture format for regression tests.

## Debuggability

The prototype already exposes debug toggles for individual stages. That is the right design.

Useful toggles include:

- corridor merging
- miter fills
- spike removal
- contour simplification
- hole filtering
- face extraction
- edge provenance
- face simplification
- edge classification
- spine clipping
- spine dedup
- spine merging
- short spine filter
- spine extensions
- stall centering
- stall-to-face clipping
- boundary clipping
- conflict removal
- short segment filter
- skeleton debug

In a host app, these should be available in an internal diagnostic mode, not necessarily in the normal product UI.

## Important Implementation Notes to Preserve

### 1. Full regeneration is the right default

For this class of problem, correctness and conceptual simplicity beat partial incremental updates. Parking layout is heavily topological; small edits can have wide downstream effects.

### 2. Drive lines should act both topologically and subtractively

They can:

- contribute graph structure
- cut through the lot
- remove stalls from their corridor footprint
- define region separators when pinned to holes

That dual role is a core strength of the current model.

### 3. Boundary faces are different

Faces adjacent to the lot boundary or building holes often need special treatment. They may not support the same angled placement logic as interior faces, and the system should use provenance to detect that rather than relying on weak geometric heuristics.

### 4. Curves belong at the boundary of the system

Curved edges are important for authoring, but they should be normalized away before core topology and parking logic run.

### 5. Islands are partly emergent

Many island shapes should fall out of subtraction after stall placement, not only from explicit island templates. This makes the system more robust to irregular geometry.

## Suggested Future Extensions

These are natural next steps once the core structure is stable in a BIM host:

- ADA and accessible-route-aware stall assignment
- entrance-aware placement rules
- richer stall kinds and code-driven schedules
- pedestrian circulation overlays
- column / obstruction handling beyond simple holes
- better bay decomposition for crossing aisle graphs
- semantics for loading, fire, EV, and reserved zones
- 3D instancing and BIM object generation from final 2D layout

## Summary

The prototype's central idea is strong and should be preserved:

Start from a user-authored site polygon, holes, partition lines, and a drive aisle graph. Use those primitives to generate corridor geometry, subtract that from the site to obtain positive-space faces, carry edge provenance into those faces, derive spines inside them, place stalls from those spines, clean the result, and finally derive islands from what remains.

That gives Arcol the right split of responsibilities:

- Arcol owns intent, editing, persistence, and presentation.
- The parking engine owns procedural geometry and regeneration.

If we preserve that boundary and keep provenance first-class, the prototype can evolve into a robust parking subsystem rather than staying a one-off demo.
