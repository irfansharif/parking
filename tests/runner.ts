import { test, expect } from "@playwright/test";
import * as fs from "fs";
import * as path from "path";
import { parseTestFile, TestCase } from "./parser";
import { rewriteExpected } from "./rewriter";

const TESTDATA_DIR = path.join(__dirname, "testdata");
const UPDATE = !!process.env.UPDATE_SNAPSHOTS;

// Discover all .txt test files
const testFiles = fs.readdirSync(TESTDATA_DIR).filter((f) => f.endsWith(".txt"));

for (const file of testFiles) {
  test(`datadriven: ${file}`, async ({ page }) => {
    // Navigate to the app
    await page.goto("http://localhost:4173/");
    await page.waitForFunction(() => (window as any).app !== undefined, null, {
      timeout: 10000,
    });

    const filePath = path.join(TESTDATA_DIR, file);
    const contents = fs.readFileSync(filePath, "utf-8");
    const cases = parseTestFile(contents, file);
    const actuals: Map<TestCase, string> = new Map();

    for (const tc of cases) {
      const actual = await executeCase(page, tc);
      actuals.set(tc, actual);

      if (tc.command.startsWith("screenshot")) {
        const parts = tc.command.trim().split(/\s+/);
        const goldenName = parts[1] || tc.expected || `${path.basename(file, ".txt")}.png`;
        await expect(page).toHaveScreenshot(goldenName, {
          maxDiffPixelRatio: 0.01,
        });
      } else if (!UPDATE && tc.expected) {
        // Text comparison
        expect(actual.trim()).toBe(tc.expected.trim());
      }
    }

    if (UPDATE) {
      const updated = rewriteExpected(contents, cases, actuals);
      fs.writeFileSync(filePath, updated);
    }
  });
}

async function executeCase(
  page: any,
  tc: TestCase,
): Promise<string> {
  const result = await page.evaluate(
    ({ command, body }: { command: string; body: string }) => {
      return (window as any).app.execute(command, body || undefined);
    },
    { command: tc.command, body: tc.body },
  );
  return result as string;
}
