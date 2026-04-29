//! Integration test entry for the data-driven fixtures.
//!
//! One `#[test]` per fixture file under `engine/tests/testdata/`. Each
//! fixture is driven through the `datadriven` crate, which owns parsing
//! the `cmd / body / ---- / expected` DSL and rewriting expected blocks
//! when `UPDATE_SNAPSHOTS=1` (mapped to datadriven's `REWRITE`).
//!
//! Per-directive dispatch lives in `harness::directives`; this file is
//! just the orchestration shell.

mod harness;

use harness::directives::{execute, snapshot, FixtureCtx};
use std::path::{Path, PathBuf};
use std::sync::Once;

fn testdata_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("testdata")
}

fn update_snapshots() -> bool {
    matches!(
        std::env::var("UPDATE_SNAPSHOTS").as_deref(),
        Ok("1") | Ok("true")
    )
}

/// Bridge our `UPDATE_SNAPSHOTS` knob to datadriven's `REWRITE`. Run
/// exactly once across all parallel test threads — `set_var` racing
/// concurrent reads is UB, so guard with `Once`.
static REWRITE_INIT: Once = Once::new();
fn init_rewrite_env() {
    REWRITE_INIT.call_once(|| {
        if update_snapshots() {
            std::env::set_var("REWRITE", "1");
        }
    });
}

fn run_fixture(path: &Path) {
    init_rewrite_env();
    let update = update_snapshots();
    let default_name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("snapshot")
        .to_string();
    let mut ctx = FixtureCtx::new(testdata_dir(), default_name, update);

    datadriven::walk(path.to_str().expect("utf-8 path"), |tf| {
        tf.run(|case| -> Result<String, String> {
            let body = case.input.trim_end_matches('\n');
            let outcome = execute(&mut ctx, &case.directive, &case.args, body)?;
            if let Some(err) = outcome.snapshot_error {
                return Err(err);
            }
            // datadriven compares raw strings; parsed expected blocks
            // always end in `\n`, so mirror that for non-empty actuals.
            Ok(if outcome.output.is_empty() {
                String::new()
            } else {
                format!("{}\n", outcome.output)
            })
        });
    });

    // Auto-snapshot the final layout. Skipped if the fixture never ran
    // `generate` (e.g. parse-only fixtures); under UPDATE_SNAPSHOTS=1
    // the .svg golden is (re)written, otherwise it's diffed.
    if ctx.last_layout.is_some() {
        let outcome = snapshot(&mut ctx).expect("snapshot");
        if let Some(err) = outcome.snapshot_error {
            panic!("{}", err);
        }
    }
}

// -- individual tests ------------------------------------------------
//
// One #[test] per fixture. Adding a fixture = adding a stanza here.
// The pair is intentional: `cargo test smoke` names one test, and a
// failure message points at one fixture file.

#[test]
fn smoke_rectangle() {
    run_fixture(&testdata_dir().join("smoke-rectangle.txt"));
}

#[test]
fn curved_boundary() {
    run_fixture(&testdata_dir().join("curved-boundary.txt"));
}

#[test]
fn graph_cross_aisle() {
    run_fixture(&testdata_dir().join("graph-cross-aisle.txt"));
}

#[test]
fn annotation_perimeter_direction() {
    run_fixture(&testdata_dir().join("annotation-perimeter-direction.txt"));
}

#[test]
fn annotation_direction_quartet() {
    run_fixture(&testdata_dir().join("annotation-direction-quartet.txt"));
}

#[test]
fn perimeter_delete_arc_vertex() {
    run_fixture(&testdata_dir().join("perimeter-delete-arc-vertex.txt"));
}

#[test]
fn stall_modifier_suppress() {
    run_fixture(&testdata_dir().join("stall-modifier-suppress.txt"));
}

#[test]
fn stall_modifier_resize() {
    run_fixture(&testdata_dir().join("stall-modifier-resize.txt"));
}

#[test]
fn basic_rectangle() {
    run_fixture(&testdata_dir().join("basic-rectangle.txt"));
}

#[test]
fn angled_stalls() {
    run_fixture(&testdata_dir().join("angled-stalls.txt"));
}

#[test]
fn cross_aisle_spacing() {
    run_fixture(&testdata_dir().join("cross-aisle-spacing.txt"));
}

#[test]
fn corner_islands() {
    run_fixture(&testdata_dir().join("corner-islands.txt"));
}

#[test]
fn hole_building() {
    run_fixture(&testdata_dir().join("hole-building.txt"));
}

#[test]
fn hole_stalls() {
    run_fixture(&testdata_dir().join("hole-stalls.txt"));
}

#[test]
fn rect_rect_hole() {
    run_fixture(&testdata_dir().join("rect-rect-hole.txt"));
}

#[test]
fn drive_line_long() {
    run_fixture(&testdata_dir().join("drive-line-long.txt"));
}

#[test]
fn drive_line_intersection() {
    run_fixture(&testdata_dir().join("drive-line-intersection.txt"));
}

#[test]
fn partitioning_two_regions() {
    run_fixture(&testdata_dir().join("partitioning-two-regions.txt"));
}

#[test]
fn partitioning_dangling() {
    run_fixture(&testdata_dir().join("partitioning-dangling.txt"));
}

#[test]
fn half_hull() {
    run_fixture(&testdata_dir().join("half-hull.txt"));
}

#[test]
fn hole_boundary_face() {
    run_fixture(&testdata_dir().join("hole-boundary-face.txt"));
}

#[test]
fn annotation_multi_region_chain() {
    run_fixture(&testdata_dir().join("annotation-multi-region-chain.txt"));
}

#[test]
fn annotation_chain_vs_segment() {
    run_fixture(&testdata_dir().join("annotation-chain-vs-segment.txt"));
}

#[test]
fn default_app_layout() {
    run_fixture(&testdata_dir().join("default-app-layout.txt"));
}

#[test]
fn offset_carriers_rect_hole() {
    run_fixture(&testdata_dir().join("offset-carriers-rect-hole.txt"));
}

#[test]
fn offset_carriers_basic_rect() {
    run_fixture(&testdata_dir().join("offset-carriers-basic-rect.txt"));
}

#[test]
fn offset_carriers_tilted_rect() {
    run_fixture(&testdata_dir().join("offset-carriers-tilted-rect.txt"));
}

#[test]
fn offset_carriers_arc() {
    run_fixture(&testdata_dir().join("offset-carriers-arc.txt"));
}

#[test]
fn offset_carriers_tight_arc() {
    run_fixture(&testdata_dir().join("offset-carriers-tight-arc.txt"));
}

#[test]
fn offset_carriers_arc_wedges() {
    run_fixture(&testdata_dir().join("offset-carriers-arc-wedges.txt"));
}

#[test]
fn offset_carriers_outward_arc() {
    run_fixture(&testdata_dir().join("offset-carriers-outward-arc.txt"));
}

#[test]
fn offset_carriers_interior_rect_hole() {
    run_fixture(&testdata_dir().join("offset-carriers-interior-rect-hole.txt"));
}

#[test]
fn offset_carriers_interior_arc_wedges() {
    run_fixture(&testdata_dir().join("offset-carriers-interior-arc-wedges.txt"));
}

#[test]
fn offset_carriers_interior_default_app() {
    run_fixture(&testdata_dir().join("offset-carriers-interior-default-app.txt"));
}
