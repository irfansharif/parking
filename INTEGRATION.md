# Parking v2: Arcol Integration Plan

## Context

Arcol today has a per-face parking feature: `FaceProperties.parking?: ParkingConfig`
(`packages/engine/src/schema/engineFileFormat.ts:136-142, 207-273`). A Rust/WASM
generator (`packages/engine/arcol-rust/src/parking.rs`, `generate_parking_js`)
consumes one face loop + that config and emits sketch geometry which is baked into
the face's mesh at meshing time (`packages/engine/src/engine/Execution.ts`). Two
layouts, no drive lines, no annotations, no regions. Output is never persisted —
regenerated on every load.

We want to port the parking-v2 prototype (`/Users/irfansharif/Desktop/parking-lot-gen`,
see `DESIGN.md`) into Arcol: polygon-with-holes input, semi-automatic drive-aisle
graph, user-drawn drive lines (partitioning or corridor), regions with independent
angle/offset, persistent annotations (direction + delete), stall modifiers
(suppress/retype). The prototype is mature; the hard part is grafting it onto
Arcol's data model and interaction infrastructure, not the algorithm.

Two things should be nailed down in this doc before coding:

1. **The persisted data model** — what shape do parking inputs and outputs take in
   the file format? Pre-baked vs baked. Per-face vs per-sketch. On `SketchElement`,
   on `FaceProperties`, as children, or as its own sketch variant.
2. **The interaction model** — which tools, entered when, layered how.

## Arcol primitives we'll build on

Grounding everything that follows. These are at HEAD of `dev`.

- **`ArcolDoc`** (`packages/types/src/types/ArcolDoc.ts:15-31`) — keyed atom
  collections per group: `shapes2D`, `shapes3D`, `points`, `curves`, `planes`,
  `levels`, `parameters`, `comments`, `sheetSets`. No dedicated parking
  collection.

- **`Sketch`** (`packages/types/src/types/sketch.ts`) is a `Shape2D` with a
  discriminated `sketchType`:
  ```ts
  sketchType:
    | { type: "flat-area"; network: PlanarNetwork; terrainFalloffSlope; terrainSlopingDistance }
    | { type: "default"; network: PlanarNetwork }
    | { type: "structural_grid"; network: PlanarNetwork; labelText? }
    | { type: "interior"; network: PlanarNetwork }
  ```
  Every variant carries a `PlanarNetwork`. The variant discriminator is how
  different "kinds of sketches" behave differently in the editor.

- **`PlanarNetwork`** is Arcol's shared topology primitive — a graph of
  vertices + curves + derived faces — used for floor plans, structural grids,
  terrain outlines, and flat areas. `RsPlanarNetwork` is the Rust-side mirror.
  Stored in every sketch.

- **`FaceProperties`** (`packages/engine/src/schema/engineFileFormat.ts:136`)
  is keyed on a cycle index of the sketch's `PlanarNetwork` and holds
  `{color, isVoid?, typology?, areaType?, parking?}`. Parking is per-face today.

- **`EditorTool`** (`apps/client/src/editor/canvas/tools/editorTool.ts:78`) —
  abstract class. Each tool owns a small state atom, implements
  `handleEvent(e): undefined | "prevent-propagation"`, and registers itself in
  `activeToolsAtom`. Lifecycle: constructor, `handleEvent` many times, `exit`.

- **`LayeredTool`** (`apps/client/src/editor/canvas/tools/layeredTool.ts`) —
  composes several `EditorTool`s into one. Dispatches events top-down; first
  to return `"prevent-propagation"` wins. The real "selection tool" is
  `new LayeredTool(editor, [ImageReferenceOverlayTool, StructuralGridManipulationTool, SelectionTool])`
  (`apps/client/src/editor/canvas/editorInteractions.ts:117-130`). This is
  the direct precedent for our parking-manipulation layering.

- **`StructuralGridCreationTool`** and **`StructuralGridManipulationTool`**
  (under `apps/client/src/editor/canvas/tools/penTool/`) are the closest
  existing pattern to what we want for drive-lines:
  - Creation tool: two-point draw; constructor asserts floor-plan viewport;
    writes to the sketch's `PlanarNetwork` via `planarNetwork.addVertex` +
    `planarNetwork.addNewEdge`; flushes via `elementStore.changeManager.makeChanges`.
  - Manipulation tool: subscribes to `useAppStore` selection; when a
    structural-grid sketch is singly-selected, initializes a context with
    vertex-handle sprites; layered under the selection tool so the user can
    drag endpoints without switching tools.

- Key takeaway: **a structural grid persists as a whole Sketch atom whose
  `sketchType.type === "structural_grid"` and whose `network: PlanarNetwork`
  is its data.** That's the right shape to imitate for parking.

## Data model proposal

### Where parking lives (and what "separate" means)

**The v2 parking system is a fully separate feature from the existing
parking pipeline.** Nothing is shared: not the per-face config, not the
Rust generator, not the sidebar component, not the tools. The v1 parking
(`FaceProperties.parking: ParkingConfig` + `generate_parking_js`) is
untouched. v2 lives alongside it, gated by a feature flag, as a second
function assignable to a sketch. A sketch is either v1-parking or
v2-parking, never both; the UI surfaces whichever the feature flag allows
(with v2 visible only when the flag is on). We keep both running until we
explicitly decide to retire v1 in a later project — not in this plan.

Like v1, v2 parking is a function assignable to a sketch: the sketch's
outer boundary is the parking-lot boundary, inner shapes are holes,
boundary edges can be circular arcs (matching Arcol's existing
curve-support convention — arcs only, no Béziers; the prototype already
uses arcs natively, so there is nothing to port in that regard).

New field on `FaceProperties`:

```ts
type FaceProperties = {
  color: Color;
  isVoid?: boolean;
  typology?: TypologyId;
  areaType?: AreaTypeId;
  parking?:   ParkingConfig;  // v1, unchanged
  parkingV2?: ParkingV2;      // v2, fully separate
};
```

`parking` and `parkingV2` are mutually exclusive by convention enforced
at the UI (a sketch's function picker is single-select), but the schema
doesn't have to reject both being set — treat v1 as authoritative if
both appear, and flag the inconsistency once in the console.

No migration either way: v1 docs keep their v1 parking; new v2 sketches
start from scratch.

Why not a new `sketchType` variant? Same reasons as before: the
function-on-a-sketch UX is the precedent the existing parking already
uses, and a parallel variant would ripple into the tool/selection layer
without user-visible benefit. The key requirement here is *separation
from v1*, not elevation to a sketch variant.

### Pre-baked form (inputs — what the user draws/sets)

The v2 payload separates three buckets by type so the distinction is
visible on the type, not just convention:

1. **`inputs`** — user-editable state. These plus the sketch's own
   `PlanarNetwork` are the exact regen input set; anything the user can
   change lives here.
2. **`derived`** — pipeline state carried between regens that is **not**
   user-editable and **not** rendered directly: the aisle graph used for
   annotation re-matching and the region signatures used for stable
   region ids. Throwing this away between regens would break annotation
   and region-override persistence, so we keep it; but it's an
   implementation detail of the pipeline, not an output.
3. **`baked`** — the final, immutable, render-only 2D geometry
   (polylines + arcs). Everything the user sees on the canvas. No graph,
   no ids, no back-references. Output of the pipeline, consumed only by
   renderers.

```ts
type ParkingV2 = {
  inputs: ParkingV2Inputs;       // editable
  derived?: ParkingV2Derived;    // pipeline-internal, persisted
  baked?: ParkingV2Baked;        // immutable render artifact
};

type ParkingV2Inputs = {
  config: ParkingV2Config;
  annotations: ParkingV2Annotation[];    // grid-space anchored; survive regen
  modifiers:   ParkingV2Modifier[];      // world-space stall modifiers
  regionOverrides: ParkingV2RegionOverride[]; // per-region angle/offset
};
```

Nothing in `ParkingV2Inputs` is reused from v1's `ParkingConfig`.

The parking sketch's boundary + holes come from the sketch's own
`PlanarNetwork` (existing) — outer loop is the lot boundary, inner loops
are holes, boundary edges may be circular arcs. No new fields required
for those.

**Drive lines** live as edges inside the sketch's `PlanarNetwork`, not
as a separate collection. We extend the network's per-edge metadata with
a `userKind` discriminator: `"boundary" | "hole" | "driveLine"`, plus
`partitions: boolean` for drive-line edges. This is additive for every
other sketch variant (they only produce `"boundary"`/`"hole"` edges) and
means drive lines inherit all of the `PlanarNetwork` infrastructure —
endpoint-drag, snapping, split-on-cross, vertex-merge — without
reinvention. v1 parking doesn't read these fields, so there's no
interaction.

```ts
type ParkingV2Config = {
  // Global parameters from DESIGN §4.
  stallWidth: number;           stallDepth: number;
  aisleWidth: number;           stallAngle: number;    // radians
  aisleAngle: number;           aisleOffset: number;
  siteOffset: number;           crossAisleMaxRun: number;
  islandStallInterval: number;
  // ADA and pillar controls — redeclared here (not imported from v1).
  adaStallCount: number;  adaStallWidth: number;  adaAccessAisleWidth: number;
  pillarInterval: number; pillarWidth: number;    pillarDepth: number;
};

type ParkingV2Annotation =
  | { kind: "direction"; target: GridEdgeRef; chain: boolean;
      direction: "TwoWayForward" | "TwoWayReverse" | "OneWayForward" | "OneWayReverse" }
  | { kind: "deleteEdge";   target: GridEdgeRef;   chain: boolean }
  | { kind: "deleteVertex"; target: GridVertexRef };

// Grid-space refs (see DESIGN §1.5). The abstract grid is a unit grid
// transformed into world space by `gridTransform` (stored on `baked`).
// Annotations persist in grid-space so regen can re-match them to the new
// aisle graph. GridEdgeRef is "edge between unit-grid vertex (u1,v1) and
// (u2,v2)"; GridVertexRef is (u,v).
type GridEdgeRef   = { u1: number; v1: number; u2: number; v2: number };
type GridVertexRef = { u: number; v: number };

type ParkingV2Modifier =
  | { kind: "suppress"; polyline: Vec2[] }
  | { kind: "retype"; polyline: Vec2[]; stallType: ParkingV2StallTypeId };

type ParkingV2RegionOverride = {
  regionId: string;
  aisleAngle?: number;
  aisleOffset?: number;
};
```

### Derived (pipeline-internal state)

Persisted so annotations and region overrides survive regens, and so
the manipulation tool has selectable edges/vertices to attach
annotations to. Never user-edited — the user edits annotations, and
those map through `derived` to the right graph element each regen.

```ts
type ParkingV2Derived = {
  // Abstract grid → world transform. Used to convert between grid-space
  // annotation refs and world-space edges each regen.
  gridTransform: { origin: Vec2; u: Vec2; v: Vec2 };

  // The derived aisle graph. Vertices carry their grid-space (u,v) coord
  // for annotation re-matching; edges carry perimeter/interior + direction.
  aisleNetwork: PlanarNetwork;

  // Stable region ids via a canonical signature of bounding partition
  // edges; see "Region identity" below.
  regionSignatures: Array<{ regionId: string; signature: string }>;
};
```

### Baked (immutable render artifact)

This is what the user sees and can't edit — the "paint lines" and
outlines — represented exclusively as 2D polylines + arcs in the sketch
plane. No graph, no ids, no back-references. Same kind of artifact
today's parking pipeline produces at the end of generation, except here
it is persisted in the doc instead of being thrown away after render. On
load the renderer consumes `baked` directly; regen only runs when
`inputs` change (by hash) or when the user explicitly asks, so doc
appearance is stable across algorithm changes.

```ts
type ParkingV2Baked = {
  generatorVersion: number;     // algorithm version that produced this
  inputHash: string;            // hash over (sketch network, inputs.*)

  // All geometry below is 2D polylines + arcs (PlanarPath = closed loop
  // of lines and circular arcs, same primitive the rest of Arcol uses).
  // The only metadata per entry is what the renderer needs to draw it.
  stalls: Array<{ loop: PlanarPath; kind: ParkingV2StallKind }>;
  aisles: PlanarPath[];                      // merged driveable surfaces
  islands: PlanarPath[];                     // landscape contours
  spineLabels: Array<{ midpoint: Vec2; stallCount: number }>; // "12 stalls" placement

  metrics: { totalStalls: number; stallsByKind: Record<ParkingV2StallKind, number> };
};
```

Hit-testing for per-stall interactions (click to delete, click to
retype) runs against `baked.stalls[].loop` directly. A click resolves to
a world-space point; the resulting `ParkingV2Modifier` is written into
`inputs.modifiers` at that point, and the next regen applies it via
polygon overlap in the pipeline. No back-reference from baked into the
aisle graph is required.

Invariants:

1. **Normal rendering reads `baked` only.** The scene graph, labels,
   sidebar counts, export-to-CAD consume `baked` exclusively — so doc
   appearance never depends on whether `derived` is present. The
   manipulation tool in sketch edit mode is the only reader of
   `derived`, and it uses it solely to render selection handles and
   hit-target geometry on aisle edges/vertices. Those handles disappear
   on exit from edit mode.
2. **Regen is input-driven.** `inputHash` covers the sketch's
   `PlanarNetwork` + `inputs.*`. If it matches the stored
   `baked.inputHash` and `baked.generatorVersion <= current`, skip
   regen. Otherwise run the generator, rewrite `derived` and `baked`
   atomically.
3. **`baked` is never edited in place.** Every write to `baked` comes
   out of the generator as a complete replacement.

### Region identity (stable ids)

Regions are derived faces; their count/shape can change when the user edits
inputs. We want `regionOverrides` to survive minor edits. Proposal: region id
= a canonical signature of the region's bounding partition edges (outer
boundary segment indices + partitioning drive-line ids it touches), not the
face's geometry. A drive line's id is stable (it's a user atom). An outer
boundary segment's id is stable as long as the segment exists. Signature
collides only when two regions are bounded by identical partition sets,
which by construction can't happen in a planar arrangement. Falls back to
"nearest polylabel center" when signature changes.

### PlanarNetwork vs. new structures

- **In PlanarNetwork (reused):** boundary, holes, drive lines — all as
  edges/vertices in the parking sketch's `network`, discriminated by
  per-edge `userKind`. The aisle graph (output) lives in `baked.aisleNetwork`.
- **Outside PlanarNetwork:** annotations, modifiers, region overrides — these
  aren't topology, they're metadata keyed to topology.
- **Why not collapse more into PlanarNetwork?** Stalls, islands, spines are
  not planar-graph entities (stalls are quads that overlap aisle polygons;
  islands have holes; spines are centerlines). They're outputs, not
  editable topology.

## Interaction model

### When controls appear

- **Sketch selected (object mode):** sidebar shows `ParkingConfigV2`
  parameters, "Add drive line" / "Add zone divider" / "Add stall modifier"
  buttons, and aggregate metrics (stall count, per-type counts). No
  in-canvas drive-line handles yet.
- **Sketch edit mode:** the layered `ParkingV2ManipulationTool`
  activates beneath the selection tool. It reads `derived.aisleNetwork`
  and renders selection handles on every aisle edge and vertex — that
  is how the graph produced by the pipeline becomes addressable by the
  user. Single-click selects a collinear chain of edges (per DESIGN
  §1.5); double-click selects a single segment. Drive-line endpoint
  handles (from the sketch's own `PlanarNetwork`) also render here.
  Region control vectors render at each region's polylabel center.
  Keybindings while in edit mode:
  - `F` on a selected aisle edge → add (or cycle) a
    `ParkingV2Annotation` of kind `"direction"` targeting that edge's
    grid-space ref (from the vertex `(u,v)` tags). The
    `chain` flag is set from the selection mode.
  - `Delete` on a selected aisle edge → add a `"deleteEdge"`
    annotation.
  - `Delete` on a selected aisle vertex → add a `"deleteVertex"`
    annotation.
  Each keystroke writes the annotation into `inputs.annotations`; the
  input hash changes; regen re-runs; the generator re-applies
  annotations by grid-space lookup against the fresh `derived`. The
  annotation thus survives any pipeline change that preserves the
  grid's (u,v) coord for that vertex/edge.

This mirrors the way `StructuralGridManipulationTool` activates only
when a structural-grid sketch is singly selected.

### Tools

| Tool | Base | Purpose |
|------|------|---------|
| `ParkingV2DriveLineCreationTool` | `EditorTool` | Entered from the sidebar. Two buttons — "Add drive line" (partitions=false) and "Add zone divider" (partitions=true) — enter the same tool with its flag pre-set (no keybinding). Two-point draw modeled on `StructuralGridCreationTool`. Clips to boundary, snaps to existing vertices, splits crossed edges, writes edges tagged `userKind: "driveLine"` into the sketch's `PlanarNetwork`. |
| `ParkingV2ManipulationTool` | `EditorTool` | Layered under selection when a parking sketch is edited. Observes `useAppStore` selection. Handles drive-line endpoint drag, midpoint drag, region control-vector drag, annotation keybindings (`F` to cycle direction, `Delete` for delete-edge/vertex), stall-modifier drop handling. |
| `ParkingV2StallModifierTool` | `EditorTool` | Entered from sidebar. Draw polyline (or single click for zero-length retype marker). Persists a `ParkingModifier`. |

Parking creation itself doesn't need a dedicated tool: the user creates a
sketch the normal way and assigns it the "parking" function, identical to
today's flow.

Layered composition when a parking sketch is being edited:
```ts
new LayeredTool(editor, [
  ParkingV2ManipulationTool,
  SelectionTool,
])
```

Keybindings: only **F** (direction cycle, matching the prototype) and
**Delete** (for selected edge/vertex annotation). No drive-line key —
entry is exclusively via the two sidebar buttons.

## Engine integration

- Port the prototype Rust crate into `packages/engine/arcol-rust/` as a
  new module `parking_v2`, fully separate from the existing
  `parking.rs`. Nothing shared between the two — new types, new entry
  points, new helpers. The existing `parking.rs` is untouched by this
  work.
- Export a single binding `generate_parking_v2_js(inputJson): outputJson`.
  The input JSON shape is the sketch's `{network, parking}` (where
  `parking` is the `ParkingV2` struct from above) serialized; output is
  `ParkingV2Baked`.
- In `Execution.ts`, for each sketch whose outer face has `parkingV2`
  assigned: compute `inputHash` over `(sketch PlanarNetwork,
  parkingV2.inputs)`; if it matches `parkingV2.baked.inputHash` and
  `baked.generatorVersion <= current`, render from `baked` directly.
  Otherwise call the generator, which returns a fresh `derived` +
  `baked` pair written back atomically.
- `PropertyValue`/`PlanarPath` does not need a polygon-with-holes variant in
  v1 of this integration — all polygons with holes live inside
  `ParkingV2Baked` as plain `{ outer: Vec2[]; holes: Vec2[][] }`. We render
  them with a new parking-specific scene object, not via `PlanarPath`.
  (Matches structural grids, which have their own renderer.)
- Worker execution: the engine already runs in a worker
  (`packages/engine/src/workers/EngineWorker.ts`), so the generator runs
  off the main thread for free.

## Infrastructure we may want to land ahead of product work

Each of these is self-contained and could be landed by someone else while the
parking work is in flight:

1. **`PlanarNetwork` per-edge `userKind` discriminator + per-edge `partitions`
   flag.** Additive schema change. No-op for existing sketch variants.
2. **Baked-output pattern on a sketch variant.** A `baked?: {...}` field
   plus the `Execution.ts` pattern that checks `inputHash` before
   regenerating. Parking is the first consumer but the pattern is
   generator-agnostic — keep the types parameterized so future generators
   can reuse the skip-regen-on-hash-match machinery.
3. **Layered tool activation by `sketchType` variant.** Generalize the
   structural-grid-specific check in the layered selection tool
   (`editorInteractions.ts:117-130`) into a registry keyed by
   `sketchType.type`. Adding a new variant then only requires registering
   `(variant, manipulationTool)` pairs.
4. **Grid-space annotation anchoring utility.** A small library for
   `gridTransform` + `GridEdgeRef` / `GridVertexRef` conversions and nearest-
   neighbor re-matching. Parking needs it; any future generator with
   persistent annotations would too.

Points 2–4 are the ones most worth landing ahead of the product work.

## Piecewise delivery

Each step ends with a working product, not a half-built one. v1 parking
is untouched throughout. v2 is gated behind a feature flag until M6.

- **M0 — Foundations.** Land items 1, 3, 4 from the "infra ahead"
  section. Add `FaceProperties.parkingV2` alongside `parking` with an
  additive schema bump (no value migration — v1 docs are unaffected).
  Feature-flagged off.
- **M1 — Minimal port.** Port the prototype engine into
  `arcol-rust/src/parking_v2/`. Generator consumes boundary-only +
  params-only input; no drive lines, annotations, regions, or modifiers.
  Produces `ParkingV2Baked` with stalls, aisles, islands. Sidebar shows the
  v2 parameter set. Parity test vs. prototype goldens (`basic`, a real
  fixture).
- **M2 — Drive lines.** `ParkingV2DriveLineCreationTool` + the drive-line parts of
  `ParkingV2ManipulationTool`. Engine consumes drive lines (partitioning =
  region decomposition; non-partitioning = corridor). Two sidebar buttons
  (drive line / zone divider) enter the creation tool with the right
  flag.
- **M3 — Annotations.** `F` cycles direction; `Delete` removes edges/
  vertices. Grid-space persistence working end-to-end through regen.
- **M4 — Regions.** Control-vector rendering and drag-to-rotate-and-
  offset; `regionOverrides` round-tripping.
- **M5 — Modifiers + stall UX.** Suppress + retype modifier tool; per-
  stall click to delete (places a Suppress) or retype (places a
  StallType). Per-type stall counts in sidebar.
- **M6 — Flip the flag on.** v1 and v2 both continue to ship;
  `parkingV2` becomes a first-class function in the sketch function
  picker. v1 is untouched. Retiring v1 is a separate project.

Explicitly out-of-scope here (future plans): ramps (DESIGN §5.3), custom
stall-type editor UI beyond the modifier, rounded island contours, bake-
to-finalize/export (DESIGN §5.4), metrics beyond stall counts.

## Critical files

- `packages/engine/src/schema/engineFileFormat.ts` — add
  `FaceProperties.parkingV2?: ParkingV2` alongside (not replacing)
  `parking?: ParkingConfig`. Define `ParkingV2`, `ParkingV2Inputs`,
  `ParkingV2Config`, `ParkingV2Annotation`, `ParkingV2Modifier`,
  `ParkingV2RegionOverride`, `ParkingV2Derived`, `ParkingV2Baked`.
- `packages/migrations/schemas/ArcolDoc.N.ts` +
  `packages/migrations/src/migrations/NtoM.ts` — additive schema bump
  for the new `parkingV2` field. v1 data untouched.
- `packages/types/src/types/planarNetwork*` — add per-edge `userKind` +
  `partitions` flag.
- `packages/engine/arcol-rust/src/parking_v2/` — port of prototype
  `engine/`.
- `packages/engine/src/engine/Execution.ts` — parking regen path with
  input-hash skip, writing back `baked`.
- `apps/client/src/editor/canvas/tools/parkingV2/` — new directory:
  `driveLineCreationTool.ts`, `parkingV2ManipulationTool.ts`,
  `parkingV2StallModifierTool.ts`. New classes, patterned on
  `penTool/structuralGrid*.ts`. Nothing shared with v1 parking code.
- `apps/client/src/editor/canvas/editorInteractions.ts` — layered-tool
  registration: add `ParkingV2ManipulationTool` to the layered selection
  stack when a v2-parking sketch is edited. v1-parking sketches follow
  their existing code path.
- New `apps/client/src/components/metrics/FaceParkingV2Section.tsx` —
  sidebar for the v2 function (parameters, add-drive-line /
  add-zone-divider / add-modifier buttons, per-type counts).
  `FaceParkingSection.tsx` (v1) is untouched.

## Open questions (should resolve before M1)

1. **Region id stability.** Propose the "partition signature + polylabel
   fallback" scheme. Acceptable for undo/redo correctness, or do we need
   something stronger?
2. **Input hash scope.** The hash includes `config`, `annotations`,
   `modifiers`, `regionOverrides`, and the `network` topology+geometry —
   excludes `baked` itself. Any other Liveblocks-ephemeral state that
   should be excluded to avoid spurious regens?
3. **Per-edge metadata on `PlanarNetwork`.** The `userKind` +
   `partitions` addition is a small additive change, but it's a shared
   primitive across several sketch types. Worth coordinating with anyone
   touching `PlanarNetwork` in parallel.

## Verification

- Snapshot tests in `packages/engine/src/test/parking.test.ts` — port the
  prototype's golden fixtures alongside the existing Arcol goldens. Keep
  both v1 and v2 paths green through M5.
- Manual: open an existing document with parking at each milestone;
  confirm (flag off) existing layouts render unchanged, and (flag on)
  equivalent-or-better layouts.
- Interaction: end-to-end smoke — create a parking sketch, draw a
  partitioning drive line, flip direction on an aisle (F), delete a
  vertex, drop an ADA modifier; each triggers a regen producing the
  expected output.
- Performance: sustain ≤16 ms per frame while dragging a region control
  vector on a ~200-stall fixture. Worker regen is fire-and-forget; if a
  drag delta lands before the last regen completes, drop the older regen
  request.
- Cross-version stability: load a doc saved at generatorVersion N; confirm
  no rerender happens in a build at version N+1 (baked form is
  authoritative) until the user opts in.
