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

  // Initial generate
  app.generate();

  // Expose command API for testing
  const commandAPI = createCommandAPI(app);
  (window as any).app = commandAPI;

  // Expose fixture dumper for debugging. Usage:
  //   copy(window.dumpFixture())
  // Then paste into tests/testdata/<name>.txt
  (window as any).dumpFixture = () => debug_input_js((window as any).__parkingInput);
}

function setupToolbar(app: App, renderer: Renderer): void {
  const toolbar = document.getElementById("toolbar")!;

  const modes: { mode: EditMode; label: string; key: string }[] = [
    { mode: "select", label: "Select (V)", key: "v" },
    { mode: "add-aisle-vertex", label: "Add Aisle Vertex (A)", key: "a" },
    { mode: "add-aisle-edge", label: "Add Aisle Edge (E)", key: "e" },
    { mode: "add-hole", label: "Add Hole (H)", key: "h" },
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

  // Separator + clear aisles button
  const sep = document.createElement("div");
  sep.className = "separator";
  toolbar.appendChild(sep);

  const clearBtn = document.createElement("button");
  clearBtn.textContent = "Clear Aisles";
  clearBtn.addEventListener("click", () => {
    app.state.aisleGraph = null;
    app.state.selectedVertex = null;
    app.generate();
  });
  toolbar.appendChild(clearBtn);

  function setMode(mode: EditMode) {
    // Commit pending hole if switching away from add-hole mode
    if (app.state.editMode === "add-hole" && mode !== "add-hole") {
      app.commitPendingHole();
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
