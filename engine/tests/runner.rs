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

fn snapshot_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("snapshots")
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

    let mut ctx = FixtureCtx::new(snapshot_dir(), default_name, update);
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
