//! Parse data-driven test fixtures. Same `.txt` DSL as
//! `tests/testdata/*.txt` (see `tests/parser.ts`):
//!
//!   command [args]
//!   <body lines, if any>
//!   ----
//!   <expected output lines, until blank line or EOF>
//!
//! Lines starting with `#` are comments. Blank lines separate sections.

#[derive(Debug, Clone)]
pub struct TestCase {
    pub command: String,
    pub body: String,
    pub expected: String,
    /// 1-indexed line number of the command line in the source file.
    pub line: usize,
}

/// Span of lines in the original file that make up one test case.
/// Used by the rewriter so we can splice new expected blocks back in
/// without reformatting the rest of the file.
#[derive(Debug, Clone)]
pub struct CaseSpan {
    /// 0-indexed line of the `----` separator, or None if absent.
    pub separator: Option<usize>,
    /// 0-indexed [start, end) line range of the expected block. Empty
    /// range when the case has no expected block.
    pub expected_start: usize,
    pub expected_end: usize,
}

pub fn parse(contents: &str) -> (Vec<TestCase>, Vec<CaseSpan>) {
    let lines: Vec<&str> = contents.split('\n').collect();
    let mut cases = Vec::new();
    let mut spans = Vec::new();
    let mut i = 0;

    while i < lines.len() {
        let trimmed = lines[i].trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            i += 1;
            continue;
        }

        let command = trimmed.to_string();
        let command_line = i + 1;
        i += 1;

        // Collect body lines until `----` or blank.
        let body_start = i;
        let mut separator = None;
        while i < lines.len() {
            let t = lines[i].trim();
            if t == "----" {
                separator = Some(i);
                break;
            }
            // A bodyless command followed by a blank line and then
            // another command is valid; stop collecting body.
            if t.is_empty() && i == body_start {
                break;
            }
            i += 1;
        }
        let body_end = i;
        let body = lines[body_start..body_end].join("\n");

        let (expected, expected_start, expected_end) = if separator.is_some() {
            i += 1; // skip ----
            let es = i;
            while i < lines.len() && !lines[i].trim().is_empty() {
                i += 1;
            }
            let ee = i;
            (lines[es..ee].join("\n"), es, ee)
        } else {
            (String::new(), body_end, body_end)
        };

        cases.push(TestCase {
            command,
            body: body.trim_end_matches('\n').to_string(),
            expected: expected.trim_end_matches('\n').to_string(),
            line: command_line,
        });
        spans.push(CaseSpan {
            separator,
            expected_start,
            expected_end,
        });
    }

    (cases, spans)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_simple_case() {
        let src = "\
# comment
polygon outer
0,0
1,0
----
polygon outer: 2 vertices

generate
----
total_stalls: 3
";
        let (cases, _) = parse(src);
        assert_eq!(cases.len(), 2);
        assert_eq!(cases[0].command, "polygon outer");
        assert_eq!(cases[0].body, "0,0\n1,0");
        assert_eq!(cases[0].expected, "polygon outer: 2 vertices");
        assert_eq!(cases[1].command, "generate");
        assert_eq!(cases[1].expected, "total_stalls: 3");
    }

    #[test]
    fn case_without_expected() {
        let src = "snapshot\n----\n";
        let (cases, _) = parse(src);
        assert_eq!(cases.len(), 1);
        assert_eq!(cases[0].command, "snapshot");
        assert_eq!(cases[0].expected, "");
    }

    #[test]
    fn case_with_no_separator() {
        // Command followed only by a blank line — a bodyless directive.
        let src = "generate\n\nscreenshot\n----\n";
        let (cases, _) = parse(src);
        assert_eq!(cases.len(), 2);
        assert_eq!(cases[0].command, "generate");
        assert_eq!(cases[0].expected, "");
        assert_eq!(cases[1].command, "screenshot");
    }
}
