export interface TestCase {
  command: string;
  body: string;
  expected: string;
  file: string;
  line: number;
}

export function parseTestFile(contents: string, filename: string): TestCase[] {
  const cases: TestCase[] = [];
  const lines = contents.replace(/\r\n/g, "\n").split("\n");
  let i = 0;

  while (i < lines.length) {
    // Skip blank lines and comments.
    if (lines[i].trim() === "" || lines[i].trim().startsWith("#")) {
      i++;
      continue;
    }

    const commandLine = i + 1; // 1-indexed line number
    const command = lines[i].trim();
    i++;

    // Collect body lines (between command and ----)
    let body = "";
    const bodyLines: string[] = [];
    while (i < lines.length && lines[i].trim() !== "----") {
      // If we hit a blank line or another command-like line with no ----,
      // this command has no body and no expected output.
      if (lines[i].trim() === "" && !bodyLines.length) {
        break;
      }
      bodyLines.push(lines[i]);
      i++;
    }

    if (i < lines.length && lines[i].trim() === "----") {
      body = bodyLines.join("\n");
      i++; // skip the ----

      // Collect expected output lines until blank line or end of file
      const expectedLines: string[] = [];
      while (i < lines.length && lines[i].trim() !== "") {
        expectedLines.push(lines[i]);
        i++;
      }

      cases.push({
        command,
        body: body.trim(),
        expected: expectedLines.join("\n").trim(),
        file: filename,
        line: commandLine,
      });
    } else {
      // Command with body but no expected output separator.
      // Treat body lines as the actual body, no expected output.
      body = bodyLines.join("\n");
      cases.push({
        command,
        body: body.trim(),
        expected: "",
        file: filename,
        line: commandLine,
      });
    }
  }

  return cases;
}
