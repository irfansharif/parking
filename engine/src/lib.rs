//! Procedural parking-lot engine (DESIGN.md).
//!
//! Module layout mirrors the nine-stage pipeline:
//!
//!   types       — input/output schema + pure-data types
//!   geom        — polygon primitives, boolean ops, inset, bezier
//!   graph       — drive-aisle graph auto-gen + region decomposition
//!   annotations — substrate-keyed edits that survive regen
//!   pipeline    — the nine-stage pipeline itself (corridors, bays,
//!                 provenance, spines, placement, islands) and the
//!                 `generate` orchestrator
//!   skeleton    — weighted straight skeleton
//!   face        — legacy face-computation module, partially migrated
//!                 into `pipeline/*`; slated for dissolution.
//!   debug       — human-readable dumps for the wasm debug route and
//!                 the native test harness
//!   wasm        — `#[wasm_bindgen]` entry points

pub mod annotations;
pub mod debug;
pub mod face;
pub mod geom;
pub mod graph;
pub mod pipeline;
pub mod skeleton;
pub mod types;
pub mod wasm;
