//! Drive-aisle graph: topology, auto-generation, and planar-arrangement
//! region decomposition.
//!
//!   aisle   — Phase A auto-generation (`auto_generate`), `Region`
//!             struct, and all the cross-cutting helpers
//!             (inset-distance derivation, hole snapping, interval
//!             math).
//!   regions — Planar-arrangement face enumeration (`enumerate_regions`)
//!             used by `aisle::decompose_regions` to slice the lot into
//!             independently-configurable regions.
//!
//! All public surface is re-exported flat so callers can say
//! `crate::graph::auto_generate(...)` rather than the more explicit
//! `crate::graph::aisle::auto_generate(...)`.

pub mod aisle;
pub mod regions;

pub use aisle::*;
pub use regions::*;
