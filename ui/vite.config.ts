import { defineConfig } from "vite";

export default defineConfig({
  base: "/parking/",
  root: ".",
  build: {
    outDir: "dist",
    target: "es2020",
  },
  server: {
    port: 3000,
    hmr: true,
  },
});
