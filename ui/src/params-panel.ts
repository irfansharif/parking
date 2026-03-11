import { App, LayerVisibility } from "./app";
import { ParkingParams } from "./types";

interface ParamDef {
  key: keyof ParkingParams;
  label: string;
  min: number;
  max: number;
  step: number;
  unit: string;
  type: "range" | "number";
}

const PARAM_DEFS: ParamDef[] = [
  {
    key: "stall_angle_deg",
    label: "Stall Angle",
    min: 45,
    max: 90,
    step: 1,
    unit: "\u00b0",
    type: "range",
  },
  {
    key: "aisle_angle_deg",
    label: "Aisle Angle",
    min: 0,
    max: 180,
    step: 1,
    unit: "\u00b0",
    type: "range",
  },
  {
    key: "stall_width",
    label: "Stall Width",
    min: 7,
    max: 12,
    step: 0.5,
    unit: "ft",
    type: "number",
  },
  {
    key: "stall_depth",
    label: "Stall Depth",
    min: 15,
    max: 22,
    step: 0.5,
    unit: "ft",
    type: "number",
  },
  {
    key: "aisle_width",
    label: "Aisle Width",
    min: 12,
    max: 30,
    step: 1,
    unit: "ft",
    type: "number",
  },
  {
    key: "max_run",
    label: "Max Run (island interval)",
    min: 0,
    max: 20,
    step: 1,
    unit: "stalls",
    type: "number",
  },
  {
    key: "island_width",
    label: "Island Width",
    min: 2,
    max: 8,
    step: 0.5,
    unit: "ft",
    type: "number",
  },
  {
    key: "ada_count",
    label: "ADA Stalls",
    min: 0,
    max: 20,
    step: 1,
    unit: "",
    type: "number",
  },
  {
    key: "site_offset",
    label: "Site Offset",
    min: 0,
    max: 20,
    step: 1,
    unit: "ft",
    type: "number",
  },
];

export function setupParamsPanel(container: HTMLElement, app: App, onUpdate: () => void): void {
  const title = document.createElement("h2");
  title.textContent = "Parameters";
  container.appendChild(title);

  for (const def of PARAM_DEFS) {
    const group = document.createElement("div");
    group.className = "param-group";

    const label = document.createElement("label");
    const valueSpan = document.createElement("span");
    valueSpan.className = "param-value";
    valueSpan.textContent = `${app.state.params[def.key]}${def.unit}`;
    label.textContent = def.label;
    label.appendChild(valueSpan);
    group.appendChild(label);

    if (def.type === "range") {
      const input = document.createElement("input");
      input.type = "range";
      input.min = String(def.min);
      input.max = String(def.max);
      input.step = String(def.step);
      input.value = String(app.state.params[def.key]);
      input.dataset.paramKey = def.key;
      input.addEventListener("input", () => {
        const val = parseFloat(input.value);
        valueSpan.textContent = `${val}${def.unit}`;
        app.setParam(def.key, val);
      });
      group.appendChild(input);
    } else {
      const input = document.createElement("input");
      input.type = "number";
      input.min = String(def.min);
      input.max = String(def.max);
      input.step = String(def.step);
      input.value = String(app.state.params[def.key]);
      input.dataset.paramKey = def.key;
      input.addEventListener("change", () => {
        const val = parseFloat(input.value);
        if (!isNaN(val)) {
          valueSpan.textContent = `${val}${def.unit}`;
          app.setParam(def.key, val);
        }
      });
      group.appendChild(input);
    }

    container.appendChild(group);
  }

  // Layers section
  const layersTitle = document.createElement("h2");
  layersTitle.textContent = "Layers";
  container.appendChild(layersTitle);

  const LAYER_DEFS: { key: keyof LayerVisibility; label: string }[] = [
    { key: "stalls", label: "Stalls" },
    { key: "aisles", label: "Drive Aisles" },
    { key: "vertices", label: "Vertices" },
    { key: "islands", label: "Islands" },
    { key: "spines", label: "Spines" },
    { key: "faces", label: "Faces" },
    { key: "miterFills", label: "Miter Fills" },
  ];

  for (const def of LAYER_DEFS) {
    const group = document.createElement("div");
    group.className = "param-group layer-toggle";

    const label = document.createElement("label");
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = app.state.layers[def.key];
    checkbox.addEventListener("change", () => {
      app.state.layers[def.key] = checkbox.checked;
      onUpdate();
    });
    label.appendChild(checkbox);
    label.appendChild(document.createTextNode(` ${def.label}`));
    group.appendChild(label);
    container.appendChild(group);
  }

  // Metrics section
  const metricsDiv = document.createElement("div");
  metricsDiv.id = "metrics";
  const metricsTitle = document.createElement("h2");
  metricsTitle.textContent = "Metrics";
  metricsDiv.appendChild(metricsTitle);
  container.appendChild(metricsDiv);

  // Update metrics and sync param inputs after each generate
  const origGenerate = app.generate.bind(app);
  app.generate = function () {
    origGenerate();
    updateMetrics(metricsDiv, app);
    syncParamInputs(container, app);
  };
}

function syncParamInputs(container: HTMLElement, app: App): void {
  const inputs = container.querySelectorAll<HTMLInputElement>("input[data-param-key]");
  for (const input of inputs) {
    const key = input.dataset.paramKey as keyof ParkingParams;
    if (!key) continue;
    const current = String(app.state.params[key]);
    if (input.value !== current) {
      input.value = current;
      // Also update the value label.
      const def = PARAM_DEFS.find((d) => d.key === key);
      if (def) {
        const label = input.parentElement?.querySelector(".param-value");
        if (label) label.textContent = `${app.state.params[key]}${def.unit}`;
      }
    }
  }
}

function updateMetrics(container: HTMLElement, app: App): void {
  container.querySelectorAll(".metric-row").forEach((el) => el.remove());

  const layout = app.state.layout;
  if (!layout) return;

  const metrics = [
    { label: "Total Stalls", value: layout.metrics.total_stalls },
    { label: "ADA Stalls", value: layout.metrics.ada_stalls },
    { label: "Islands", value: layout.islands.length },
  ];

  for (const m of metrics) {
    const row = document.createElement("div");
    row.className = "metric-row";
    row.innerHTML = `<span>${m.label}</span><span class="value">${m.value}</span>`;
    container.appendChild(row);
  }
}
