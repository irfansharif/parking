# Parking v2: Arcol Integration Plan

> Companion to `DESIGN.md §6`, which is the **canonical schema and
> behavioural spec** for parking-v2 inside Arcol. This doc is the
> *delivery plan*: how to graft the prototype onto Arcol's data model,
> tooling, and milestones. Where the two disagree, DESIGN wins.

## Context

The prototype (`/Users/irfansharif/Desktop/parking-lot-gen`) is
mature: substrate-keyed annotations across grid / drive line /
perimeter, region decomposition with stable ids, drive lines with
optional `partitions` flag, dormant-annotation reporting,
arc-supporting boundaries, stall modifiers (suppress + retype), and
all nine pipeline stages from `pipeline/mod.rs`. ts-rs exports every
input/output type to TypeScript, so the engine boundary is already a
shared cross-language surface.

Arcol today has a per-face parking feature: `FaceProperties.parking?:
ParkingConfig` (`packages/engine/src/schema/engineFileFormat.ts`). A
Rust/WASM generator (`packages/engine/arcol-rust/src/parking.rs`,
`generate_parking_js`) consumes one face loop + that config and emits
sketch geometry baked into the face's mesh at meshing time
(`packages/engine/src/engine/Execution.ts`). Two layouts, no drive
lines, no annotations, no regions. Output is never persisted —
regenerated on every load.

The plan is to port parking-v2 into Arcol as a **second, fully
separate** function assignable to a sketch, gated behind a feature
flag, alongside (not replacing) v1. v2 lives in its own module; v1 is
untouched. Retiring v1 is a future project.

The prototype is mature; the hard part is grafting it onto Arcol's
data model and interaction infrastructure, not the algorithm.

## Arcol primitives we'll build on

These are at HEAD of `dev`.

- **`ArcolDoc`** (`packages/types/src/types/ArcolDoc.ts`) — keyed
  atom collections per group: `shapes2D`, `shapes3D`, `points`,
  `curves`, `planes`, `levels`, `parameters`, `comments`,
  `sheetSets`. No dedicated parking collection.

- **`Sketch`** (`packages/types/src/types/sketch.ts`) is a `Shape2D`
  with a discriminated `sketchType`:
  ```ts
  sketchType:
    | { type: "flat-area"; network: PlanarNetwork; ... }
    | { type: "default"; network: PlanarNetwork }
    | { type: "structural_grid"; network: PlanarNetwork; labelText? }
    | { type: "interior"; network: PlanarNetwork }
  ```
  Every variant carries a `PlanarNetwork`. The variant discriminator
  is how different "kinds of sketches" behave differently in the
  editor.

- **`PlanarNetwork`** is Arcol's shared topology primitive — a graph
  of vertices + curves + derived faces — used for floor plans,
  structural grids, terrain outlines, and flat areas. `RsPlanarNetwork`
  is the Rust-side mirror.

- **`FaceProperties`** is keyed on a cycle index of the sketch's
  `PlanarNetwork` and holds `{color, isVoid?, typology?, areaType?,
  parking?}`. Parking is per-face today.

- **`EditorTool`** (`apps/client/src/editor/canvas/tools/editorTool.ts`)
  — abstract class. Each tool owns a small state atom, implements
  `handleEvent(e): undefined | "prevent-propagation"`, and registers
  itself in `activeToolsAtom`.

- **`LayeredTool`** composes several `EditorTool`s into one;
  dispatches events top-down. The selection tool is
  `new LayeredTool(editor, [ImageReferenceOverlayTool,
  StructuralGridManipulationTool, SelectionTool])`
  (`apps/client/src/editor/canvas/editorInteractions.ts`). Direct
  precedent for our parking-manipulation layering.

- **`StructuralGridCreationTool`** and
  **`StructuralGridManipulationTool`** are the closest existing
  pattern to what we want for drive-lines: two-point draw, writes via
  `planarNetwork.addVertex` + `planarNetwork.addNewEdge`, flushes via
  `elementStore.changeManager.makeChanges`. Manipulation tool reads
  selection from `useAppStore`, layered under selection so endpoints
  drag without switching tools.

- Key takeaway: **a structural grid persists as a whole `Sketch` atom
  whose `sketchType.type === "structural_grid"` and whose
  `network: PlanarNetwork` is its data.** That's the right shape to
  imitate for parking.

## Data model — overview

`DESIGN.md §6.1` defines the file-format schema. The short version,
since this plan keeps referring to it:

```ts
type FaceProperties = {
  color: Color;  isVoid?: boolean;
  typology?: TypologyId;  areaType?: AreaTypeId;
  parking?:   ParkingConfig;  // v1, untouched
  parkingV2?: ParkingV2;      // v2, new
};

type ParkingV2 = {
  inputs: ParkingV2Inputs;     // editable; the only thing the UI writes
  baked?: ParkingV2Baked;      // immutable pipeline output, replaced atomically
};
```

Two buckets. There is intentionally **no `derived` bucket**: pipeline
state that a previous draft of this doc wanted to persist
(`gridTransform`, `regionSignatures`) is now recomputed inside the
generator each regen as a pure function of `inputs` + the sketch's
current `PlanarNetwork`. The one piece of historically-derived state
that *is* persisted is the resolved aisle graph, and it lives in
`baked.aisleNetwork` because the edit-mode manipulation tool renders
selection handles on it — pulling it out of `baked` would force a
regen on every entry to edit mode and flash blank handles.

A sketch is either v1-parking or v2-parking, never both; the UI's
function picker is single-select. Schema treats v1 as authoritative
if both appear and logs the inconsistency once.

## Mapping prototype types onto Arcol

The prototype's engine types (in `engine/src/types/`) are auto-exported
to TypeScript via ts-rs (`make bindings` writes to
`ui/src/bindings/`). When the engine moves into
`packages/engine/arcol-rust/src/parking_v2/`, the same `ts-rs`
pattern carries the types into Arcol's TS world — no hand-written
mirrors.

Mapping the prototype's runtime types to Arcol persistence shapes:

| Prototype (engine + UI)              | Arcol persistence shape                                  |
|--------------------------------------|----------------------------------------------------------|
| `Polygon { outer, holes, outer_arcs, hole_arcs, outer_ids, hole_ids }` | Sketch's own `PlanarNetwork` (boundary edges + circular-arc bulges + stable per-vertex ids) |
| `DriveLine { id, start, end, partitions, hole_pin? }` | `PlanarNetwork` edges tagged `userKind: "driveLine"`, with a per-edge `partitions: boolean` |
| `StallModifier { polyline, kind }`   | `PlanarNetwork` edges tagged `userKind: "stallModifier"`, with a per-edge `stallKind: StallKind` (`Suppressed` is a distinguished kind) |
| `GenerateInput.annotations: Vec<Annotation>` | `inputs.annotations: ParkingV2Annotation[]` (substrate-keyed; survives regen) |
| `RegionOverride { region_id, aisle_angle_deg?, aisle_offset? }` | `inputs.config.regionOverrides`, keyed by stable `RegionId` |
| `ParkingParams` (numeric knobs)      | `inputs.config.{stallWidth,...,islandStallInterval,...}` |
| `ParkingLayout.{stalls, faces, spines, islands, derived_outer, derived_holes, region_debug, metrics}` | `baked.{stalls, aisles, islands, spineLabels, metrics}` (2D polylines + arcs only; no back-references). The prototype's `ParkingLayout` doesn't currently surface the merged drive-aisle surface — it's computed inside `pipeline::corridors::merge_corridor_surface` and consumed by `extract_faces` / `tag_face_edges`. The integration adds an `aisles: PlanarPath[]` to `baked` derived from that same merged shape. |
| `ParkingLayout.resolved_graph: DriveAisleGraph` | `baked.aisleNetwork: PlanarNetwork` |
| `ParkingLayout.dormant_annotations: Vec<DormantAnnotation>` | `baked.dormantAnnotations: { index, reason }[]`; surfaced in the sidebar as "N dormant annotation(s)" with reasons (the prototype already does this via `showToast`) |

`Polygon.outer` is the **lot deed line** by current convention (commit
`b818614`); the aisle-edge perimeter is computed by inset
(`compute_inset_d` → `aisle_edge_perim`). Arcol doesn't need to
persist the inset ring.

Per-vertex stable ids on the sketch (`outer_ids`, `hole_ids`) map to
Arcol's existing `VertexId`. The prototype already assumes the same
shape — `engine/src/types/geom.rs:VertexId(u32)` mirrors Arcol's
`Brand<number, "VertexId">`. Perimeter annotations key off
`(start_vid, end_vid, t)` pairs of these ids and survive vertex
insertion / deletion / reordering elsewhere on the loop. Edge splits
invalidate the original pair; resolution is dormant for the affected
annotation that regen.

### Annotations — the substrate-local model

`DESIGN §1.5` and `engine/src/types/addressing.rs` define this.
Reproduced here at the level of detail this plan needs:

```ts
type ParkingV2Annotation =
  | { kind: "deleteVertex"; target: Target }
  | { kind: "deleteEdge";   target: Target }
  | { kind: "direction";    target: Target; traffic: AisleDirection };

type AisleDirection = "TwoWayReverse" | "OneWay" | "OneWayReverse";
//   The unannotated default IS two-way (default lean). Only the
//   *Reverse* direction variants and the one-way variants need an
//   annotation; "TwoWay" is not a value an annotation can carry.

type Target =
  | { on: "Grid";       region: RegionId; axis: "X" | "Y"; coord: number;
                        range: [GridStop, GridStop] | null }
  | { on: "DriveLine";  id: number; t: number }
  | { on: "Perimeter";  loop: PerimeterLoop;
                        start: VertexId; end: VertexId; t: number };
```

The three substrates each carry stable coordinates and a canonical
direction:

- **Grid** — per-region integer lattice, materialized at regen as an
  `AbstractFrame` with `dx = 2·effective_depth + 2·aisle_width` and
  `dy = stalls_per_face · stall_pitch`. Annotations key by `(region,
  axis, coord, range)`; integer `(xi, yi)` cells follow rotation,
  stretch, and `aisle_offset` shifts of the lattice for free.
- **DriveLine** — parametric `t ∈ [0, 1]` from start to end. Splice
  vertices remember their `(drive_line_id, t)` so resolution is by
  t-proximity, not world distance.
- **Perimeter** — sketch-edge anchored: `(loop, start_vid, end_vid, t)`
  where the vertex pair names a sketch edge in the loop's canonical
  winding (CCW outer, CW holes) and `t ∈ [0, 1]` is the fraction along
  the edge (chord, or arc when the sketch edge has bulge data). The
  engine evaluates `t` to a world point, projects onto the graph perim
  ring (the inset for `Outer`; the hole sketch directly for `Hole`),
  and matches against graph vertices / sub-edges by arc length. Outer
  projection allows `inset_d + arc_slack` of perpendicular offset to
  cover the gap between the deed line and the inset ring.

Resolution is substrate-local — single dormancy rule: any referenced
substrate or stop missing from the current graph makes the annotation
dormant this regen. The engine reports each dormant annotation as a
`{ index, reason }` entry (e.g. "perimeter edge {start}→{end} not
addressable (vertex deleted or edge split)"); the UI surfaces them.

A previous version of this plan proposed grid-space-only annotations
(`GridEdgeRef`, `GridVertexRef`) and a persisted `gridTransform`. That
shape doesn't survive a delete that lands on the perimeter or on a
splice vertex (neither sits on a lattice point), and the prototype
has since grown all three. The single substrate-keyed model is the
one that ships.

### Region identity (stable ids)

Regions are bounded faces of the planar arrangement of `{outer
boundary ∪ hole boundaries ∪ partitioning drive lines}`. We need
region ids that survive minor edits so per-region overrides round-trip
through edits.

Current scheme (`engine/src/graph/regions.rs:region_id_from_kinds`):
each region's bounding cycle is collapsed to a sequence of `EdgeKind`
entries — `Outer(VertexId)`, `Hole(hole_idx, VertexId)`, and
`Partition(drive_line_id)` — keyed by *stable identifiers*, then
canonicalized (rotate to lex-smallest leading element), then hashed
via FNV-1a. The hash is masked to JavaScript's 53-bit safe-integer
range (so it round-trips through JSON without `bigint`); a reserved
sentinel above that range tags the single-region fallback.

Stability properties:

- A vertex inserted on an edge that doesn't bound the region leaves
  its id untouched.
- A drive line's id is stable across regens (it's a user atom).
- Boundary edges cut by arc discretization carry synthetic vertex
  ids; resolution promotes them to their "ancestor" sketch corner so
  chord count doesn't shift the region id.
- A partition added or removed changes only the regions it bounds,
  not unrelated regions.

When the topology shifts enough to break the hash, the override falls
back to "nearest polylabel center." That's good enough for undo/redo
correctness in interactive use.

## Where things live

| Primitive                                              | Lives in                                          |
|--------------------------------------------------------|---------------------------------------------------|
| Boundary, holes, drive lines, stall modifiers          | Sketch's own `PlanarNetwork`, tagged by `userKind` |
| Aisle graph (output of regen)                          | `baked.aisleNetwork` (separate `PlanarNetwork`)   |
| Annotations                                            | `inputs.annotations`, substrate-keyed             |
| Region overrides                                       | `inputs.config.regionOverrides`, keyed by `RegionId` |
| Stalls, islands, spine labels, aisle merged surface    | `baked` geometry arrays                           |

User-drawn lines all share `PlanarNetwork` so they share its editing
infrastructure (endpoint drag, snap, split-on-cross, vertex-merge).
A new additive per-edge discriminator distinguishes their pipeline
role: `userKind: "boundary" | "hole" | "driveLine" | "stallModifier"`,
plus a `partitions: boolean` for drive-line edges and a
`stallKind: StallKind` for stall-modifier edges. All three are
no-ops for the existing sketch variants, which only emit `"boundary"`
/ `"hole"` edges.

Stalls / islands / spines aren't editable topology — they're pipeline
outputs, held in `baked` as flat 2D polylines + arcs with no
back-references.

## Interaction model

### Object mode (sketch selected)

Sidebar shows `ParkingV2Config` parameters as scrubbable inputs,
three action buttons (*Add drive line*, *Add zone divider*, *Add
stall modifier*), and `baked.metrics`. No in-canvas drive-line
handles yet.

### Edit mode

A layered `ParkingV2ManipulationTool` activates beneath the selection
tool. It reads `baked.aisleNetwork` and renders selection handles on
every aisle edge and vertex — that is how the graph produced by the
pipeline becomes addressable by the user. Single-click selects a
collinear chain of edges (`resolve::find_collinear_chain`);
double-click selects a single segment. Drive-line endpoint handles
(from the sketch's own `PlanarNetwork`) also render here. Region
control vectors render at each region's polylabel center
(`region.center` from `region_debug`). Keybindings while in edit
mode mirror the prototype:

- `F` on a selected aisle edge → resolve target (Grid → DriveLine
  fallback), upsert a `direction` annotation, cycle through
  `TwoWayReverse → OneWay → OneWayReverse → tombstone`. The cycle
  ends with a tombstone (the unannotated state IS two-way), so a
  cycled-past annotation is filtered out before regen.
- `Delete` on a selected aisle edge → resolve target (Grid → DriveLine
  fallback), append a `deleteEdge` annotation.
- `Delete` on a selected aisle vertex → choose the most explicit
  anchor (Perimeter > Grid > DriveLine), append a `deleteVertex`
  annotation.

Each keystroke writes the annotation into `inputs.annotations`; the
input hash changes; regen re-runs; the generator re-applies
annotations by substrate-local lookup against the fresh aisle graph.
Annotations that don't resolve are returned alongside `baked` as
dormant entries with reasons — surfaced in the UI as a non-blocking
toast / sidebar entry, identical to the prototype.

This mirrors the way `StructuralGridManipulationTool` activates only
when a structural-grid sketch is singly selected.

### Tools

| Tool                              | Base         | Purpose |
|-----------------------------------|--------------|---------|
| `ParkingV2DriveLineCreationTool`  | `EditorTool` | Entered from the sidebar. Two buttons — *Add drive line* (`partitions=false`) and *Add zone divider* (`partitions=true`) — enter the same tool with its flag pre-set (no keybinding). Two-point draw modeled on `StructuralGridCreationTool`. Clips to boundary, snaps to existing vertices, splits crossed edges, writes edges tagged `userKind: "driveLine"` with the appropriate `partitions` flag. |
| `ParkingV2ManipulationTool`       | `EditorTool` | Layered under selection when a v2-parking sketch is edited. Observes `useAppStore` selection. Handles drive-line endpoint drag, region control-vector drag, annotation keybindings (`F` to cycle direction, `Delete` for delete-edge / delete-vertex), and click-on-stall flows for modifier drop. |
| `ParkingV2StallModifierTool`      | `EditorTool` | Entered from sidebar. Two-point or single-click polyline draw. Writes a `userKind: "stallModifier"` edge with the chosen `stallKind` (`Suppressed` for fire lanes / clearances, anything else for retype). |

Parking creation itself doesn't need a dedicated tool: the user
creates a sketch the normal way and assigns it the parking-v2
function, identical to today's flow.

Layered composition when a v2-parking sketch is being edited:

```ts
new LayeredTool(editor, [
  ParkingV2ManipulationTool,
  SelectionTool,
])
```

Keybindings: only **F** (direction cycle) and **Delete** (selected
edge or vertex annotation). No drive-line key — entry is exclusively
via the sidebar buttons.

## Engine integration

- Port the prototype engine into `packages/engine/arcol-rust/src/parking_v2/`,
  fully separate from the existing `parking.rs`. Preserve the current
  module layout — `types/`, `geom/`, `graph/`, `pipeline/`,
  `annotations.rs`, `resolve.rs`, `wasm.rs`. Nothing shared between
  v1 and v2.
- The pipeline now leans heavily on `i_overlay` (`StrokeOffset`,
  `OutlineOffset`, boolean ops) and `polylabel` for medial-axis label
  placement. Both are pure Rust crates with no native dependencies,
  so they wasm-compile cleanly. The previous hand-rolled corridor
  rectangle / miter-wedge / spike-removal / RDP-simplify passes are
  gone — `merge_corridor_surface` strokes the centerline graph in
  one shot. Spine generation similarly replaced the weighted straight
  skeleton with a per-aisle-edge offset (`offset_aisle_edges_to_spines`),
  which is much simpler to validate against fixtures than the
  event-driven version it replaced.
- Use ts-rs as the binding source (the prototype already does — see
  `Makefile` `bindings` target). The TS files end up in
  `ui/src/bindings/` in the prototype; in Arcol they'd land in
  `packages/engine/src/types/parkingV2/` (or wherever Arcol prefers
  generated types). No hand-written TS mirrors.
- Wasm surface is **not** a single entry point. The prototype exposes
  one `generate_js(inputJson) → outputJson` plus a dozen narrow
  helpers used per-frame for hit-testing and address resolution
  (`engine/src/wasm.rs`). Arcol needs both. The narrow helpers —
  `target_world_pos_js`, `world_to_perimeter_pos_js`,
  `world_to_abstract_vertex_js`, `world_to_splice_vertex_js`,
  `chain_to_abstract_lattice_edge_js`, `resolve_grid_target_js`,
  `find_collinear_chain_js`, `hit_test_edge_js`, frame math — back
  the manipulation tool's per-event resolution work without round-
  tripping the full pipeline.
- In `Execution.ts`, for each sketch whose outer face has `parkingV2`
  assigned: compute `inputHash` over `(sketch PlanarNetwork,
  parkingV2.inputs)`; if it matches `parkingV2.baked.inputHash` and
  `baked.generatorVersion <= current`, render from `baked` directly.
  Otherwise enqueue regen on the engine worker; render the previous
  `baked` until the result returns; then atomically replace
  `baked` (and the dormant-annotation list).
- Worker execution: the engine already runs in a worker
  (`packages/engine/src/workers/EngineWorker.ts`), so the generator
  runs off the main thread for free.
- `PropertyValue` / `PlanarPath` does not need a polygon-with-holes
  variant in v1 of this integration — all geometry inside
  `ParkingV2Baked` is plain `{ outer: Vec2[]; holes: Vec2[][] }` (or
  `{ start, end, arcs[] }` polylines). We render with a parking-
  specific scene object, not via `PlanarPath`. (Matches structural
  grids, which have their own renderer.)

## Infrastructure to land ahead of product work

Each is self-contained and could be landed by someone else while the
parking work is in flight:

1. **`PlanarNetwork` per-edge `userKind` discriminator + per-edge
   `partitions` and `stallKind` fields.** Additive schema change.
   No-op for existing sketch variants.
2. **Stable per-vertex `VertexId` exposure on `PlanarNetwork`.** Arcol
   already has `VertexId` as a brand on number; the prototype matches.
   Confirm the contract: ids survive insertion / deletion / reordering
   elsewhere on a loop, and edge-splits invalidate the parent pair
   (annotations using it go dormant). The prototype's
   `Polygon::ensure_ids` is the reference implementation.
3. **Baked-output pattern on a sketch variant.** A `baked?: {...}`
   field plus the `Execution.ts` pattern that checks `inputHash`
   before regenerating, swaps `baked` atomically when regen returns,
   and bundles dormant-annotation reporting with the swap. Parking is
   the first consumer but the pattern is generator-agnostic — keep
   the types parameterized so future generators can reuse the
   skip-regen-on-hash-match machinery.
4. **Layered tool activation by `sketchType` variant.** Generalize
   the structural-grid-specific check in
   `editorInteractions.ts` into a registry keyed by
   `sketchType.type`. Adding a new variant then only requires
   registering `(variant, manipulationTool)` pairs.

Items 1, 3, and 4 are the ones most worth landing ahead of the
product work. 2 may already be done, depending on Arcol's current
`PlanarNetwork` contract.

## Piecewise delivery

Each step ends with a working product, not a half-built one. v1
parking is untouched throughout. v2 is gated behind a feature flag
until M6.

- **M0 — Foundations.** Land items 1, 3, 4 from the "infra ahead"
  section. Add `FaceProperties.parkingV2` alongside `parking` with an
  additive schema bump (no value migration — v1 docs are unaffected).
  Feature-flagged off.
- **M1 — Minimal port.** Port the prototype engine into
  `arcol-rust/src/parking_v2/`. Generator consumes boundary-only +
  params-only input; no drive lines, annotations, regions, or
  modifiers. Produces `ParkingV2Baked` with stalls, aisles, islands.
  Sidebar shows the v2 parameter set (the contents of
  `ParkingParams` in `engine/src/types/io.rs`). Parity test against
  prototype goldens — start with `engine/tests/testdata/basic.txt`
  and the simpler real-fixture cases. Refresh harness via
  `UPDATE_SNAPSHOTS=1 cargo test --test runner`.
- **M2 — Drive lines.** `ParkingV2DriveLineCreationTool` + the
  drive-line parts of `ParkingV2ManipulationTool`. Engine consumes
  drive lines (partitioning = region decomposition; non-partitioning
  = aisle-only). Two sidebar buttons (drive line / zone divider)
  enter the creation tool with the right flag.
- **M3 — Annotations.** `F` cycles direction; `Delete` removes
  edges / vertices. Substrate-local persistence working end-to-end
  through regen, with dormant-annotation reporting wired to the
  sidebar. All three substrates (Grid / DriveLine / Perimeter)
  reachable from the UI's resolution flow.
- **M4 — Regions.** Per-region control vectors rendered at polylabel
  centers; drag rotates aisle angle / drag body slides aisle offset.
  `regionOverrides` round-tripping under `RegionId`.
- **M5 — Modifiers + stall UX.** `ParkingV2StallModifierTool`. Per-
  stall click to delete (places a `Suppressed` modifier) or retype
  (places a typed modifier). Per-type stall counts (`metrics`).
- **M6 — Flip the flag on.** v1 and v2 both ship; `parkingV2`
  becomes a first-class function in the sketch function picker. v1
  is untouched. Retiring v1 is a separate project.

Explicitly out-of-scope here (future plans): ramps (DESIGN §5.3),
custom stall-type editor UI beyond the modifier, bake-to-finalize /
export (DESIGN §5.4), metrics beyond stall counts. Rounded island
contours land for free at M1 — the prototype already does the
morphological-open + round-join pass via `island_corner_rounding`
(DESIGN §3.8), and the `island-corner-radius` knob ships alongside
the rest of `ParkingParams`.

## Critical files

Engine port:

- `packages/engine/arcol-rust/src/parking_v2/` — module layout
  mirrors `engine/src/`: `types/{geom,graph,addressing,io,output,mod}.rs`,
  `geom/{arc,boolean,clip,inset,offset,poly,mod}.rs`,
  `graph/{aisle,regions,mod}.rs`,
  `pipeline/{bays,corridors,filter,generate,islands,placement,
  spines,tagging,mod}.rs`, plus top-level `annotations.rs`,
  `resolve.rs`, `wasm.rs`.
- `packages/engine/src/types/parkingV2/` — ts-rs-generated bindings
  (or wherever Arcol prefers them). Sole TS-side type surface.

Schema + execution:

- `packages/engine/src/schema/engineFileFormat.ts` — add
  `FaceProperties.parkingV2?: ParkingV2`. Define `ParkingV2`,
  `ParkingV2Inputs`, `ParkingV2Config`, `ParkingV2Annotation`,
  `ParkingV2Baked`. Per DESIGN §6.1.
- `packages/migrations/schemas/ArcolDoc.N.ts` +
  `packages/migrations/src/migrations/NtoM.ts` — additive schema
  bump for the new `parkingV2` field. v1 data untouched.
- `packages/types/src/types/planarNetwork*` — add per-edge
  `userKind`, `partitions`, `stallKind`. Confirm or expose stable
  per-vertex `VertexId`s on the network.
- `packages/engine/src/engine/Execution.ts` — parking-v2 regen path
  with `inputHash` skip, atomic `baked` replacement, dormant-
  annotation surfacing.

Tools + sidebar:

- `apps/client/src/editor/canvas/tools/parkingV2/` — new directory:
  `driveLineCreationTool.ts`, `parkingV2ManipulationTool.ts`,
  `parkingV2StallModifierTool.ts`. New classes, patterned on
  `penTool/structuralGrid*.ts`. Nothing shared with v1 parking code.
- `apps/client/src/editor/canvas/editorInteractions.ts` — register
  `ParkingV2ManipulationTool` in the layered selection stack when a
  v2-parking sketch is edited. v1-parking sketches follow their
  existing code path.
- `apps/client/src/components/metrics/FaceParkingV2Section.tsx` —
  sidebar for the v2 function (parameters, action buttons, per-type
  counts, dormant-annotation list). `FaceParkingSection.tsx` (v1) is
  untouched.

## Open questions (resolve before M1)

1. **Region id stability.** The current scheme — FNV-1a hash over the
   canonicalized cyclic sequence of `EdgeKind`s, with a polylabel-
   center fallback when the hash breaks — is good enough in
   interactive use. Acceptable for undo/redo correctness in Arcol's
   model? Or do we need a stronger spatial-fingerprint fallback?
2. **Input hash scope.** The hash covers `config`, `annotations`,
   `regionOverrides`, the sketch's `PlanarNetwork` topology + arc
   bulges + stable ids, and the `userKind` / `partitions` /
   `stallKind` per-edge metadata. Excludes Liveblocks-ephemeral
   selection state and `baked` itself. Anything else worth excluding
   to avoid spurious regens?
3. **Per-edge metadata on `PlanarNetwork`.** The `userKind` +
   `partitions` + `stallKind` addition is small and additive, but
   it's a shared primitive across several sketch types. Worth
   coordinating with anyone touching `PlanarNetwork` in parallel.

## Verification

- **Snapshot tests** in `packages/engine/src/test/parkingV2.test.ts`
  — port the prototype's golden fixtures from
  `engine/tests/testdata/`. Refresh with the same
  `UPDATE_SNAPSHOTS=1 cargo test --test runner` flow.
- **Manual** — open an existing document with parking at each
  milestone; confirm (flag off) existing layouts render unchanged,
  and (flag on) equivalent-or-better layouts.
- **Interaction** — end-to-end smoke: create a parking sketch, draw
  a partitioning drive line, flip direction on an aisle (`F`), delete
  a vertex on the perimeter, drop a `Suppressed` modifier; each
  triggers a regen producing the expected output, and any failed
  resolutions show up as dormant entries with reasons.
- **Performance** — sustain ≤16 ms per frame while dragging a region
  control vector on a ~200-stall fixture. Worker regen is fire-and-
  forget; if a drag delta lands before the last regen completes, drop
  the older request. The narrow wasm helpers
  (`target_world_pos_js` etc.) are the hot path during drag and
  shouldn't round-trip the full pipeline.
- **Cross-version stability** — load a doc saved at
  `generatorVersion N`; confirm no rerender happens in a build at
  version N+1 (`baked` is authoritative) until the user opts in.
