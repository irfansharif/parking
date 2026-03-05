import { test, expect } from "@playwright/test";
import * as fs from "fs";
import * as path from "path";
import { parseTestFile, TestCase } from "./parser";

const TESTDATA_DIR = path.join(__dirname, "testdata");

// Discover all .txt test files
const testFiles = fs.readdirSync(TESTDATA_DIR).filter((f) => f.endsWith(".txt"));

for (const file of testFiles) {
  test(`datadriven: ${file}`, async ({ page }) => {
    // Navigate to the app
    await page.goto("http://localhost:4173/");
    await page.waitForFunction(() => (window as any).app !== undefined, null, {
      timeout: 10000,
    });

    const contents = fs.readFileSync(path.join(TESTDATA_DIR, file), "utf-8");
    const cases = parseTestFile(contents, file);

    for (const tc of cases) {
      const actual = await executeCase(page, tc);

      if (tc.command.startsWith("screenshot")) {
        const goldenName = tc.expected || `${path.basename(file, ".txt")}.png`;
        await expect(page).toHaveScreenshot(goldenName, {
          maxDiffPixelRatio: 0.01,
        });
      } else if (tc.expected) {
        // Text comparison
        expect(actual.trim()).toBe(tc.expected.trim());
      }
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
