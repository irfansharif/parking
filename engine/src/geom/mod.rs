//! Parking-agnostic 2D geometry utilities.
//!
//! All operations here take and return `types::geom` primitives and
//! know nothing about aisles, stalls, or annotations. The module is
//! designed to be extractable as a standalone `geom` crate without
//! churn; no `i_overlay` types escape `boolean.rs`.

pub mod bezier;
pub mod boolean;
pub mod clip;
pub mod inset;
pub mod poly;
