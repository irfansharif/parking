 Plan to implement

 Parking Layout Generator Prototype

 Context

 We're building a standalone procedural parking layout generator that will later
 integrate into a larger Rust/TypeScript codebase. The system takes a boundary
 polygon (with holes for buildings), an editable drive aisle vertex network, and
 configuration parameters — and produces a complete parking layout (stalls,
 islands, drive aisles) in real-time.

 The prototype has three layers:
 1. Rust engine (compiled to WASM) — all geometry computation
 2. TypeScript/Canvas UI — visualization + interactive vertex editing
 3. Datadriven test harness — scriptable command interface for automated testing

 ---
 Project Structure

 parking-lot-gen/
 ├── engine/                         # Rust crate → WASM
 │   ├── Cargo.toml
 │   └── src/
 │       ├── lib.rs                  # WASM entry point (generate_js)
 │       ├── types.rs                # Vec2, Polygon, Obb, shared geometry
 │       ├── aisle_graph.rs          # DriveAisleGraph, AisleEdge, auto-generation
 │       ├── bay.rs                  # Bay decomposition (boundary + graph → regions)
 │       ├── segment.rs              # Fill a rail edge with stalls (ParkingSegment)
 │       ├── island.rs               # Island generation (max-run, end-cap, corner)
 │       ├── generate.rs             # Top-level pipeline: generate() → ParkingLayout
 │       └── clip.rs                 # Polygon clipping helpers (wraps i_overlay)
 │
 ├── ui/                             # TypeScript + Vite
 │   ├── package.json
 │   ├── tsconfig.json
 │   ├── vite.config.ts
 │   ├── index.html
 │   └── src/
 │       ├── main.ts                 # Bootstrap: init WASM, create app, start render loop
 │       ├── app.ts                  # App state: boundary, graph, params, layout result
 │       ├── renderer.ts             # Canvas 2D drawing (stalls, aisles, islands, grid)
 │       ├── interaction.ts          # Mouse handling: vertex drag, pan, zoom
 │       ├── params-panel.ts         # HTML parameter controls (sliders, inputs)
 │       ├── commands.ts             # Command executor (shared by UI + test harness)
 │       └── types.ts                # TS types mirroring Rust output structs
 │
 ├── tests/                          # Datadriven test files
 │   ├── runner.ts                   # Playwright-based test runner
 │   ├── parser.ts                   # Datadriven file parser
 │   ├── testdata/                   # .txt test files
 │   │   ├── basic-rectangle.txt
 │   │   ├── hole-building.txt
 │   │   ├── angled-stalls.txt
 │   │   └── vertex-drag.txt
 │   └── golden/                     # Expected outputs (screenshots + JSON)
 │
 ├── playwright.config.ts
 └── Makefile                        # build-engine, dev, test

 ---
 Milestone 1: Vertical Slice — Stalls on Screen

 Goal: Hardcoded rectangle boundary → Rust generates stalls → Canvas renders them.

 engine/src/types.rs

 // Core geometry types
 pub struct Vec2 { pub x: f64, pub y: f64 }
 pub struct Polygon { pub outer: Vec<Vec2>, pub holes: Vec<Vec<Vec2>> }
 pub struct Obb { pub center: Vec2, pub half_extents: Vec2, pub angle: f64 }

 // Stall quad: 4 corners in world coordinates
 pub struct StallQuad {
     pub corners: [Vec2; 4],  // CCW: aisle-edge p0, p1, back p2, p3
     pub kind: StallKind,     // Standard, Ada, Compact, Ev
 }
 pub enum StallKind { Standard, Ada, Compact, Ev }

 // Island shape
 pub struct Island {
     pub polygon: Vec<Vec2>,  // closed polygon
     pub kind: IslandKind,
 }
 pub enum IslandKind { MaxRun, EndCap, Corner, AislePadding }

 // The full output
 pub struct ParkingLayout {
     pub aisle_polygons: Vec<Vec<Vec2>>,
     pub stalls: Vec<StallQuad>,
     pub islands: Vec<Island>,
     pub metrics: Metrics,
 }
 pub struct Metrics {
     pub total_stalls: usize,
     pub ada_stalls: usize,
 }

 engine/src/aisle_graph.rs

 pub struct DriveAisleGraph {
     pub vertices: Vec<Vec2>,
     pub edges: Vec<AisleEdge>,
 }
 pub struct AisleEdge {
     pub start: usize,       // vertex index
     pub end: usize,         // vertex index
     pub width: f64,         // full aisle width (for one-way edge, half the visual aisle)
 }

 Auto-generation for milestone 1: given a rectangular boundary, produce parallel
 one-way aisle edges (alternating direction) spaced by 2 * stall_depth + aisle_width. Each pair of opposing edges
 forms a two-way aisle.

 engine/src/generate.rs

 Top-level pipeline (simplified for M1 — no bay decomposition yet):

 generate(boundary, params) → ParkingLayout:
   1. Auto-generate aisle graph if none provided
   2. For each aisle edge:
      a. Compute corridor rectangle (edge centerline ± width/2)
      b. For left/right side: build parking strip, fill with stalls
   3. Collect all stalls, corridors → ParkingLayout

 engine/src/segment.rs

 The core stall placement along a single edge side:

 fill_strip(edge_start, edge_end, flow_dir, side_normal, params) → Vec<StallQuad>:
   - Compute strip length along edge
   - stall_pitch = stall_width / sin(stall_angle)
   - stall_count = floor(strip_length / stall_pitch)
   - padding = (strip_length - stall_count * stall_pitch) / 2
   - For i in 0..stall_count:
       build parallelogram stall at position padding + i * stall_pitch
       stall leans forward in flow direction (edge start→end)

 engine/src/lib.rs

 WASM entry point:

 #[wasm_bindgen]
 pub fn generate_js(input_json: &str) -> String {
     let input: GenerateInput = serde_json::from_str(input_json).unwrap();
     let layout = generate(input);
     serde_json::to_string(&layout).unwrap()
 }

 ui/ — Minimal Canvas Renderer

 - main.ts: Load WASM, create hardcoded rectangular boundary, call generate_js,
 parse result, draw stalls as filled parallelograms on canvas.
 - renderer.ts: For each stall quad, draw filled polygon (light gray fill, dark
 gray stroke). For aisle polygons, draw filled (darker gray). Grid lines in
 background.

 Verification

 - make build-engine compiles Rust to WASM
 - make dev starts Vite dev server
 - Browser shows a rectangular lot with parallel rows of angled stalls

 ---
 Milestone 2: Interactive Vertex Network

 Goal: Display vertex network, drag vertices, regenerate on mouse move.

 ui/src/app.ts — App State

 interface AppState {
   boundary: { outer: Vec2[], holes: Vec2[][] };
   aisleGraph: { vertices: Vec2[], edges: AisleEdge[] } | null; // null = auto
   params: ParkingParams;
   layout: ParkingLayout | null;   // last generated result
   // Interaction state
   selectedVertex: number | null;
   hoveredVertex: number | null;
   isDragging: boolean;
   camera: { offsetX: number, offsetY: number, zoom: number };
 }

 ui/src/interaction.ts

 Mouse event handling:
 - mousedown: Hit-test all vertices (boundary + aisle graph). If within radius,
 select and begin drag.
 - mousemove: If dragging, update vertex position, call generate(), re-render.
 - mouseup: End drag.
 - wheel: Zoom. middle-drag: Pan.

 Hit testing: for each vertex, check distance to mouse position in world coords
 (accounting for camera transform). Threshold: 8px screen-space.

 ui/src/renderer.ts — Enhanced

 Drawing layers (bottom to top):
 1. Background grid (light lines every 10ft, darker every 50ft)
 2. Aisle corridor polygons (medium gray fill)
 3. Stall quads (light fill + dark stroke per stall)
 4. Islands (green fill)
 5. Vertex network overlay: edges as dashed lines, vertices as circles
 6. Selected/hovered vertex highlight

 Verification

 - Blue dots appear at aisle graph vertices
 - Dragging a vertex moves connected aisle edges
 - Stalls regenerate smoothly during drag
 - Stall count changes only at pitch thresholds (no flickering)

 ---
 Milestone 3: Holes + Boundary Editing

 Goal: Support inner polygons (buildings), editable boundary vertices.

 Engine Changes

 - generate() clips all strip geometry against the feasible region
 (outer boundary minus holes) using i_overlay crate.
 - Strips that overlap hole polygons get carved out.
 - Stalls that fall outside the clipped strip are discarded.

 UI Changes

 - Boundary vertices (outer + holes) are draggable, rendered as orange dots.
 - Hole polygons drawn with crosshatch fill to indicate buildings.
 - Double-click on boundary edge to insert a new vertex.
 - Right-click vertex to delete it (if >3 vertices remain on that ring).

 Verification

 - Place a rectangular hole in the center of the lot.
 - Stalls wrap around the hole, maintaining setback.
 - Dragging hole vertices reshapes the carved-out region in real-time.

 ---
 Milestone 4: Parameter Panel + Full Feature Set

 Goal: UI controls for all generation parameters. Islands.

 ui/src/params-panel.ts

 HTML panel (positioned right side of canvas) with controls:
 - Stall angle: slider 45-90 degrees
 - Stall width: input (default 9ft)
 - Stall depth: input (default 18ft)
 - Aisle width: input (default 24ft)
 - Max run (island interval): input 0-20 (0 = no islands)
 - Island width: input (default 4ft)
 - ADA stall count: input
 - Site offset: input (boundary inset)

 Each change calls generate() and re-renders.

 Engine: Island Generation

 In island.rs:
 - Max-run islands: Every N stalls along a segment, emit a rectangle of
 island_width x stall_depth perpendicular to the aisle.
 - End-cap islands: At span ends that don't touch a boundary, emit a rounded
 rectangle (semicircle cap + rectangle body).
 - Corner islands: At boundary vertices where two stall rows meet, compute the
 dead zone polygon and fill it.

 Engine: ADA Stall Assignment

 After all stalls are placed, select stalls nearest to a configurable focus point
 (default: building entrance) and mark them as ADA. ADA stalls are wider
 (stall_width + buffer_width).

 Verification

 - Adjusting stall angle slider smoothly re-orients all stalls
 - Islands appear at configured intervals
 - Rounded end-caps visible at row terminations
 - ADA stalls highlighted in blue near the building

 ---
 Milestone 5: Datadriven Test Harness

 Goal: Scriptable command interface + Playwright-based test runner.

 Command Language Specification

 # Comments start with #

 # Define outer boundary
 polygon outer
 0,0
 200,0
 200,150
 0,150

 # Define a hole (building)
 polygon hole
 80,50
 120,50
 120,100
 80,100

 # Set parameters
 set stall-angle=60
 set stall-width=9
 set stall-depth=18
 set aisle-width=24
 set max-run=8
 set island-width=4

 # Add a manual vertex to the aisle graph
 vertex add 50,75

 # Add another vertex
 vertex add 150,75

 # Connect them with an aisle edge
 edge add 0,1

 # Move a vertex
 vertex move 0 60,80

 # Generate the layout (auto-generates aisle graph if no manual edges)
 generate
 ----
 total_stalls: 142
 ada_stalls: 4

 # Capture serialized layout state
 state
 ----
 {"stalls":[...],"islands":[...],"aisle_polygons":[...]}

 # Capture screenshot (compared against golden file)
 screenshot
 ----
 basic-rectangle-001.png

 File Format Rules

 Following cockroachdb/datadriven conventions:
 - Each test case: command line, optional body, ----, expected output
 - Blank lines between test cases are ignored
 - # lines are comments
 - Commands with no expected output omit the ---- separator
 - Multi-line expected output: use ----/---- double separator

 ui/src/commands.ts — Command Executor

 The app exposes window.app with methods that map 1:1 to script commands:

 interface CommandAPI {
   execute(command: string, body?: string): string;
   getState(): object;
   screenshotDataURL(): string;
 }

 Command dispatch:
 - polygon outer + body → parse points, set boundary.outer
 - polygon hole + body → parse points, push to boundary.holes
 - set key=value → update params
 - vertex add x,y → add vertex to aisle graph, return vertex index
 - edge add v1,v2 → add edge between vertices
 - vertex move id x,y → update vertex position
 - generate → call WASM generate(), return metrics summary
 - state → return JSON of current layout
 - screenshot → return reference to captured PNG

 The same command executor is used by both the UI (for testing) and the
 Playwright harness (via page.evaluate).

 tests/parser.ts — Datadriven File Parser

 interface TestCase {
   command: string;          // e.g., "polygon outer", "set stall-angle=60"
   args: Record<string, string>;
   body: string;             // lines between command and ----
   expected: string;         // lines after ----
   file: string;
   line: number;
 }

 function parseTestFile(contents: string): TestCase[];

 tests/runner.ts — Playwright Test Runner

 // For each .txt file in testdata/:
 //   1. Launch browser, navigate to app
 //   2. Parse test file into cases
 //   3. For each case:
 //      a. Send command via page.evaluate(() => window.app.execute(cmd, body))
 //      b. Capture actual output
 //      c. Compare to expected:
 //         - For "screenshot": pixel-diff against golden PNG (threshold-based)
 //         - For "state" / "generate": exact string match
 //   4. Report pass/fail, optionally rewrite golden files with --update flag

 Screenshot comparison uses golden PNGs: on screenshot, the runner captures
 the canvas via Playwright's page.screenshot(), saves/compares against a golden
 .png file in tests/golden/. Comparison uses pixelmatch with a configurable
 threshold (default: 0.1, maxDiffPixels: 50). Running with --update overwrites
 the golden files.

 Verification

 - make test runs all .txt files in testdata/
 - make test-update rewrites expected outputs
 - A basic-rectangle.txt test creates a lot, generates, and validates stall count
 - A vertex-drag.txt test moves vertices and validates layout changes

 ---
 Milestone 6: Polish + Edge Cases

 - Bay decomposition: proper planar subdivision when aisle edges cross
 - Boundary-hugging aisles (outer loop parking)
 - Stall clipping against irregular boundary shapes
 - Overlap resolution between adjacent strips (deterministic by edge ID)
 - Hysteresis on stall count changes (dead zone ~0.5ft)
 - Entrance markers on boundary edges (no stalls placed)

 ---
 Build & Dev Workflow

 # Makefile

 build-engine:
       cd engine && wasm-pack build --target web --release --out-dir ../ui/src/wasm

 dev: build-engine
       cd ui && npx vite

 test: build-engine
       cd ui && npx vite build
       npx playwright test

 test-update: build-engine
       cd ui && npx vite build
       npx playwright test --update-snapshots

 Dependencies

 Rust (engine/Cargo.toml):
 - wasm-bindgen = "0.2"
 - serde = { version = "1", features = ["derive"] }
 - serde_json = "1"
 - i_overlay = "1" (polygon boolean ops — union, intersection, difference)

 TypeScript (ui/package.json):
 - vite (dev server + bundler)
 - typescript

 Testing:
 - @playwright/test
 - pixelmatch (if doing manual screenshot comparison)

 ---
 Units

 All dimensions in feet. Default values:
 - Stall width: 9ft
 - Stall depth: 18ft
 - Aisle width: 24ft (full two-way), 12ft (one-way edge width)
 - Island width: 4ft
 - Turn radius: 3ft
 - Site offset: 0ft

 The Canvas renderer shows a scale bar and grid lines at 10ft / 50ft intervals.

 ---
 Key Design Decisions

 1. JSON bridge over wasm-bindgen structs. Simpler, more portable. The JSON
 overhead is negligible at our scale (<1000 stalls). One generate_js(json) → json function is the entire WASM
 surface.
 2. Every edge is one-way. Two-way aisle = two opposing edges. This is the
 single primitive — no special-casing for bidirectional aisles. The
 auto-generator creates opposing pairs.
 3. Full regeneration, not incremental. On every edit (vertex drag, param
 change), the entire pipeline re-runs. At <5ms for a typical lot, this is
 cheaper and more correct than incremental updates.
 4. Command executor shared between UI and tests. The same commands.ts
 code parses and executes commands whether they come from a test script or
 from the UI's internal wiring. This guarantees test scripts exercise the
 exact same code paths as interactive use.
 5. Dual output format for tests. generate returns metrics text (stall
 counts — stable, easy to assert). state returns full geometry JSON
 (deterministic, diffable). screenshot captures pixels (visual regression,
 threshold-based). Tests can mix all three.
 6. No frameworks in the UI. Raw Canvas 2D + vanilla TypeScript. The UI is a
 thin shell around the Rust engine. This keeps it portable and avoids
 framework lock-in for later integration.
