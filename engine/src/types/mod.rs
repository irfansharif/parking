//! Public type surface for the engine, grouped by purpose.
//!
//!   geom       — Vec2, Polygon, EdgeCurve, Obb
//!   graph      — drive-aisle graph (DriveAisleGraph, AisleEdge, ...)
//!                and user-drawn drive lines
//!   addressing — abstract grid (AbstractFrame, RegionId) and
//!                annotation substrates (Annotation, Target, ...)
//!   output     — pipeline-output types (StallQuad, Face, Island,
//!                spines, debug buckets)
//!   io         — top-level input/output shapes (GenerateInput,
//!                ParkingLayout) and tuning surface (ParkingParams,
//!                DebugToggles, RegionOverride)
//!
//! Sub-modules are kept `pub` so callers can import either
//! `types::Vec2` (re-export) or `types::geom::Vec2` (canonical path).

pub mod addressing;
pub mod geom;
pub mod graph;
pub mod io;
pub mod output;

pub use addressing::*;
pub use geom::*;
pub use graph::*;
pub use io::*;
pub use output::*;
