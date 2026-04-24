//! The nine-stage parking generation pipeline (DESIGN.md §2, §3).
//!
//! Each stage lives in its own submodule:
//!
//!   corridors — §3.2 aisle polygon construction
//!   bays      — §3.3 positive-space face extraction
//!   tagging   — §3.4 edge provenance (wall vs corridor source)
//!   spines    — §3.5 spine generation via per-aisle-edge offset
//!   placement — §3.6 stall placement along spines
//!   filter    — §3.7 post-placement stall filtering (incl. §1.4 modifiers)
//!   islands   — §3.8 residual gap extraction
//!   generate  — top-level orchestrator tying the stages together

pub mod bays;
pub mod corridors;
pub mod filter;
pub mod generate;
pub mod islands;
pub mod placement;
pub mod spines;
pub mod tagging;

#[cfg(test)]
mod integration_tests;
