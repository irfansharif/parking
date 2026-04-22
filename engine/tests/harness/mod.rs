//! Data-driven test harness for the parking-lot engine.
//!
//! Fixtures live in `engine/tests/testdata/*.txt` and use the same DSL
//! shape as the UI's Playwright suite (see `tests/parser.ts`):
//!
//!   command [args]
//!   <body lines>
//!   ----
//!   <expected output lines>
//!
//! Text directives assert on engine output; the `snapshot` directive
//! emits a deterministic SVG and diffs against `engine/tests/snapshots/`.
//!
//! Run with `cargo test -p parking-lot-engine` or, to refresh goldens,
//! `UPDATE_SNAPSHOTS=1 cargo test -p parking-lot-engine`.

pub mod directives;
pub mod parser;
pub mod rewriter;
pub mod svg;
