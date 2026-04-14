import init, { generate_js, debug_input_js } from "./wasm/parking_lot_engine";
import { App } from "./app";
import { Renderer } from "./renderer";
import { setupInteraction, updateModeHint } from "./interaction";
import { setupParamsPanel } from "./params-panel";
import { createCommandAPI } from "./commands";
import { EditMode } from "./app";

async function main() {
  await init();

  const canvas = document.getElementById("canvas") as HTMLCanvasElement;
  const paramsPanel = document.getElementById("params-panel") as HTMLElement;

  const renderer = new Renderer(canvas);
  renderer.resize();

  const app = new App(generate_js, () => renderer.render(app.state));

  setupInteraction(canvas, app, renderer);
  setupParamsPanel(paramsPanel, app, () => renderer.render(app.state));
  setupToolbar(app, renderer);
  updateModeHint(app);

  window.addEventListener("resize", () => {
    renderer.resize();
    renderer.render(app.state);
  });

  // Fit all lot boundaries in the canvas with some padding.
  {
    const container = canvas.parentElement!;
    const cw = container.clientWidth;
    const ch = container.clientHeight;
    const xs: number[] = [];
    const ys: number[] = [];
    for (const lot of app.state.lots) {
      for (const v of lot.boundary.outer) { xs.push(v.x); ys.push(v.y); }
      xs.push(lot.aisleVector.start.x, lot.aisleVector.end.x);
      ys.push(lot.aisleVector.start.y, lot.aisleVector.end.y);
    }
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const bw = maxX - minX;
    const bh = maxY - minY;
    const padding = 100;
    const zoom = Math.min((cw - padding * 2) / bw, (ch - padding * 2) / bh);
    app.state.camera.zoom = zoom;
    app.state.camera.offsetX = (cw - bw * zoom) / 2 - minX * zoom;
    app.state.camera.offsetY = (ch - bh * zoom) / 2 - minY * zoom;
  }

  // Initial generate
  app.generate();

  // Expose command API for testing
  const commandAPI = createCommandAPI(app);
  (window as any).app = commandAPI;
  (window as any).__app = app;

  // Expose fixture dumper for debugging. Usage:
  //   copy(window.dumpFixture())
  // Then paste into tests/testdata/<name>.txt
  // For multi-lot, dumps each lot with new-lot/select-lot commands.
  (window as any).dumpFixture = () => {
    if (app.state.lots.length <= 1) {
      return debug_input_js((window as any).__parkingInput);
    }
    const parts: string[] = [];
    for (let i = 0; i < app.state.lots.length; i++) {
      const lot = app.state.lots[i];
      const input = {
        boundary: lot.boundary,
        aisle_graph: lot.aisleGraph,
        drive_lines: lot.driveLines,
        annotations: lot.annotations.filter((a: any) => a._active !== false),
        params: app.state.params,
        debug: app.state.debug,
        regionOverrides: Object.entries(lot.regionOverrides).map(([k, v]: [string, any]) => ({
          region_id: Number(k),
          aisle_angle_deg: v.angle,
          aisle_offset: v.offset,
        })),
      };
      if (i > 0) {
        parts.push(`new-lot\n----\nnew-lot: ${lot.id}\n`);
      }
      parts.push(debug_input_js(JSON.stringify(input)));
    }
    return parts.join("\n");
  };
}

function setupToolbar(app: App, renderer: Renderer): void {
  const toolbar = document.getElementById("toolbar")!;

  const modes: { mode: EditMode; label: string; key: string }[] = [
    { mode: "select", label: "Select (V)", key: "v" },
    { mode: "add-boundary", label: "Add Boundary (B)", key: "b" },
    { mode: "add-hole", label: "Add Hole (H)", key: "h" },
    { mode: "add-drive-line", label: "Drive Line (L)", key: "l" },
  ];

  const buttons: HTMLButtonElement[] = [];

  for (const { mode, label } of modes) {
    const btn = document.createElement("button");
    btn.textContent = label;
    btn.dataset.mode = mode;
    if (mode === app.state.editMode) btn.classList.add("active");
    btn.addEventListener("click", () => setMode(mode));
    toolbar.appendChild(btn);
    buttons.push(btn);
  }

  function setMode(mode: EditMode) {
    if (app.state.editMode === "add-boundary" && mode !== "add-boundary") {
      app.commitPendingBoundary();
    }
    if (app.state.editMode === "add-hole" && mode !== "add-hole") {
      app.commitPendingHole();
    }
    if (app.state.editMode === "add-drive-line" && mode !== "add-drive-line") {
      app.state.pendingDriveLine = null;
      app.state.pendingDriveLinePreview = null;
    }
    app.state.editMode = mode;
    app.state.selectedVertex = null;
    for (const b of buttons) {
      b.classList.toggle("active", b.dataset.mode === mode);
    }
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    canvas.style.cursor = mode === "select" ? "default" : "crosshair";
    updateModeHint(app);
    renderer.render(app.state);
  }

  // Keyboard shortcuts for mode switching
  document.addEventListener("keydown", (e) => {
    // Don't hijack if typing in an input
    if (
      e.target instanceof HTMLInputElement ||
      e.target instanceof HTMLTextAreaElement
    )
      return;
    for (const { mode, key } of modes) {
      if (e.key === key) {
        setMode(mode);
        e.preventDefault();
        break;
      }
    }
  });
}

main().catch(console.error);
