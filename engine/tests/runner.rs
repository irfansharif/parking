//! Integration test entry for the data-driven fixtures.
//!
//! One `#[test]` per fixture file under `engine/tests/testdata/`. Each
//! fixture runs through the harness dispatcher in sequence, sharing a
//! single `FixtureCtx` so later directives see earlier state.

mod harness;

use harness::directives::{execute, FixtureCtx, Outcome};
use harness::parser::{parse, CaseSpan, TestCase};
use harness::rewriter::rewrite;

use std::path::{Path, PathBuf};

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

fn run_fixture(path: &Path) {
    let source = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("reading {}: {}", path.display(), e));
    let (cases, spans) = parse(&source);
    let update = update_snapshots();

    let default_name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("snapshot")
        .to_string();

    let mut ctx = FixtureCtx::new(testdata_dir(), default_name, update);
    let mut actuals: Vec<String> = Vec::with_capacity(cases.len());
    let mut failures: Vec<String> = Vec::new();

    for (case, span) in cases.iter().zip(spans.iter()) {
        match execute(&mut ctx, &case.command, &case.body) {
            Ok(outcome) => {
                if let Some(err) = &outcome.snapshot_error {
                    failures.push(format!("{}:{}: {}", display(path), case.line, err));
                }
                actuals.push(outcome.output.clone());
                check_or_record(&outcome, case, span, path, update, &mut failures);
            }
            Err(err) => {
                failures.push(format!("{}:{}: {}", display(path), case.line, err));
                actuals.push(String::new());
            }
        }
    }

    if update {
        let rewritten = rewrite(&source, &spans, &actuals);
        if rewritten != source {
            std::fs::write(path, rewritten)
                .unwrap_or_else(|e| panic!("rewriting {}: {}", path.display(), e));
        }
    } else if !failures.is_empty() {
        panic!("\n\n{}\n\n", failures.join("\n\n"));
    }
}

fn check_or_record(
    outcome: &Outcome,
    case: &TestCase,
    span: &CaseSpan,
    path: &Path,
    update: bool,
    failures: &mut Vec<String>,
) {
    // Under UPDATE_SNAPSHOTS we accept any text output as-is; the
    // rewriter will splice it back.
    if update {
        return;
    }
    // Cases without a separator have no expected block to compare.
    if span.separator.is_none() {
        return;
    }
    let expected = case.expected.trim();
    let actual = outcome.output.trim();
    if expected == actual {
        return;
    }
    failures.push(format!(
        "{}:{}: {}\n  expected:\n{}\n  actual:\n{}",
        display(path),
        case.line,
        case.command,
        indent(expected, "    "),
        indent(actual, "    "),
    ));
}

fn indent(s: &str, prefix: &str) -> String {
    if s.is_empty() {
        return format!("{}(empty)", prefix);
    }
    s.lines()
        .map(|l| format!("{}{}", prefix, l))
        .collect::<Vec<_>>()
        .join("\n")
}

fn display(p: &Path) -> String {
    p.file_name()
        .and_then(|s| s.to_str())
        .map(|s| s.to_string())
        .unwrap_or_else(|| p.display().to_string())
}

// -- individual tests ------------------------------------------------
//
// One #[test] per fixture. Adding a fixture = adding a stanza here.
// The pair is intentional: `cargo test smoke` names one test, and a
// failure message points at one fixture file. If the fixture set grows
// large we can switch to a `test_cases!` macro, but explicit stanzas
// keep rust-analyzer's "run test" gutter intact.

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
fn stall_modifier_suppress() {
    run_fixture(&testdata_dir().join("stall-modifier-suppress.txt"));
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
fn offset_carriers_tilted_rect_off() {
    run_fixture(&testdata_dir().join("offset-carriers-tilted-rect-off.txt"));
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
fn offset_carriers_tight_arc_off() {
    run_fixture(&testdata_dir().join("offset-carriers-tight-arc-off.txt"));
}

#[test]
fn offset_carriers_arc_wedges() {
    run_fixture(&testdata_dir().join("offset-carriers-arc-wedges.txt"));
}

#[test]
fn offset_carriers_outward_arc() {
    run_fixture(&testdata_dir().join("offset-carriers-outward-arc.txt"));
}
