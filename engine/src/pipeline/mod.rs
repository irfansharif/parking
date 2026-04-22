//! The nine-stage parking generation pipeline (DESIGN.md §2, §3).
//!
//! Each stage lives in its own submodule. Today this module only hosts
//! `corridors` (§3.2 aisle polygon construction). Subsequent phases
//! add `bays` (§3.3), `provenance` (§3.4), `spines` (§3.5),
//! `placement` (§3.6), `filter` (§3.7), and `islands` (§3.8); the
//! final phase collapses `generate.rs` into a thin orchestrator living
//! here.

pub mod bays;
pub mod corridors;
pub mod generate;
pub mod islands;
pub mod placement;
pub mod provenance;
pub mod spines;
