//! Rewrite expected-output blocks back into a fixture file.
//!
//! Only active under `UPDATE_SNAPSHOTS=1 cargo test`. Preserves every
//! line outside of expected blocks (comments, blank lines, body lines,
//! `----` separators) so hand-written structure survives round-trips.

use crate::harness::parser::CaseSpan;

/// Replace each case's expected block with the matching actual output
/// and return the rewritten file contents. `actuals[i]` pairs with
/// `spans[i]`.
pub fn rewrite(source: &str, spans: &[CaseSpan], actuals: &[String]) -> String {
    assert_eq!(spans.len(), actuals.len(), "span/actual length mismatch");
    if spans.is_empty() {
        return source.to_string();
    }

    let lines: Vec<&str> = source.split('\n').collect();
    let mut out = String::new();
    let mut cursor = 0;

    for (span, actual) in spans.iter().zip(actuals.iter()) {
        // Emit everything up to (but not including) this case's
        // expected block. That keeps the command + body + separator
        // exactly as written.
        let copy_end = span.expected_start;
        for i in cursor..copy_end {
            out.push_str(lines[i]);
            out.push('\n');
        }
        // Emit the rewritten expected block. If the case had no
        // separator (expected block was empty), leave it alone.
        if span.separator.is_some() {
            for line in actual.lines() {
                out.push_str(line);
                out.push('\n');
            }
        }
        cursor = span.expected_end;
    }

    // Trailing content (blank lines, final newline).
    for i in cursor..lines.len() {
        out.push_str(lines[i]);
        if i + 1 < lines.len() {
            out.push('\n');
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::harness::parser::parse;

    #[test]
    fn replaces_expected_block_preserving_surroundings() {
        let src = "\
# header
polygon outer
0,0
1,0
----
polygon outer: 2 vertices

generate
----
total_stalls: 0
";
        let (_, spans) = parse(src);
        let actuals = vec![
            "polygon outer: 2 vertices".to_string(),
            "total_stalls: 7".to_string(),
        ];
        let rewritten = rewrite(src, &spans, &actuals);
        assert!(rewritten.contains("total_stalls: 7"));
        assert!(rewritten.starts_with("# header"));
        assert!(rewritten.contains("polygon outer: 2 vertices"));
    }
}
