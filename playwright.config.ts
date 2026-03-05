import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests",
  testMatch: "runner.ts",
  snapshotPathTemplate: "{testDir}/golden/{arg}{ext}",
  timeout: 30000,
  use: {
    baseURL: "http://localhost:4173",
    headless: true,
  },
  webServer: {
    command: "cd ui && npx vite preview --port 4173",
    port: 4173,
    reuseExistingServer: true,
  },
});
