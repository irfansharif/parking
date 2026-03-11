import { TestCase } from "./parser";

/**
 * Rewrite a test file's expected output sections with actual results.
 * Preserves the file structure: command, body, ----, then expected output.
 */
export function rewriteExpected(
  contents: string,
  cases: TestCase[],
  actuals: Map<TestCase, string>,
): string {
  const lines = contents.replace(/\r\n/g, "\n").split("\n");
  const out: string[] = [];
  let i = 0;

  let caseIdx = 0;

  while (i < lines.length) {
    // Skip blank lines and comments between cases.
    if (lines[i].trim() === "" || lines[i].trim().startsWith("#")) {
      out.push(lines[i]);
      i++;
      continue;
    }

    const tc = caseIdx < cases.length ? cases[caseIdx] : undefined;
    caseIdx++;

    // Emit the command line.
    out.push(lines[i]);
    i++;

    // Collect and emit body lines up to ----.
    const bodyLines: string[] = [];
    while (i < lines.length && lines[i].trim() !== "----") {
      if (lines[i].trim() === "" && !bodyLines.length) {
        break;
      }
      bodyLines.push(lines[i]);
      i++;
    }
    for (const bl of bodyLines) {
      out.push(bl);
    }

    if (i < lines.length && lines[i].trim() === "----") {
      out.push(lines[i]); // the ---- separator
      i++;

      // Skip old expected output lines.
      while (i < lines.length && lines[i].trim() !== "") {
        i++;
      }

      // Write new expected output (skip for screenshot commands).
      if (tc && !tc.command.startsWith("screenshot")) {
        const actual = (actuals.get(tc) || "").trim();
        if (actual) {
          out.push(actual);
        }
      }
    }
  }

  return out.join("\n");
}
