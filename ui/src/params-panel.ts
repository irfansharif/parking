import { App, LayerVisibility } from "./app";
import { ParkingParams, DebugToggles } from "./types";

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
    key: "stall_width",
    label: "Stall Width",
    min: 7,
    max: 40,
    step: 0.5,
    unit: "ft",
    type: "range",
  },
  {
    key: "stall_depth",
    label: "Stall Depth",
    min: 8,
    max: 30,
    step: 0.5,
    unit: "ft",
    type: "range",
  },
  {
    key: "aisle_width",
    label: "Lane Width",
    min: 8,
    max: 24,
    step: 0.5,
    unit: "ft",
    type: "range",
  },
  {
    key: "site_offset",
    label: "Site Offset",
    min: 0,
    max: 50,
    step: 0.5,
    unit: "ft",
    type: "range",
  },
  {
    key: "stalls_per_face",
    label: "Stalls Per Face",
    min: 3,
    max: 50,
    step: 1,
    unit: "stalls",
    type: "range",
  },
  {
    key: "island_stall_interval",
    label: "Island Interval",
    min: 0,
    max: 20,
    step: 1,
    unit: "stalls",
    type: "range",
  },
  {
    key: "min_stalls_per_spine",
    label: "Min Stalls / Spine",
    min: 1,
    max: 10,
    step: 1,
    unit: "stalls",
    type: "range",
  },
  {
    key: "arc_discretize_tolerance",
    label: "Arc Tolerance",
    min: 0.1,
    max: 20,
    step: 0.1,
    unit: "ft",
    type: "range",
  },
  {
    key: "island_corner_radius",
    label: "Island Corner Radius",
    min: 0,
    max: 20,
    step: 0.5,
    unit: "ft",
    type: "range",
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

  // Region decomposition toggle
  const regGroup = document.createElement("div");
  regGroup.className = "param-group layer-toggle";
  const regLabel = document.createElement("label");
  const regCheckbox = document.createElement("input");
  regCheckbox.type = "checkbox";
  regCheckbox.checked = !!app.state.params.use_regions;
  regCheckbox.addEventListener("change", () => {
    (app.state.params as any).use_regions = regCheckbox.checked;
    app.generate();
  });
  regLabel.appendChild(regCheckbox);
  regLabel.appendChild(document.createTextNode(" Region Decomposition"));
  regGroup.appendChild(regLabel);
  container.appendChild(regGroup);

  // Layers section
  const layersTitle = document.createElement("h2");
  layersTitle.textContent = "Layers";
  container.appendChild(layersTitle);

  const LAYER_DEFS: { key: keyof LayerVisibility; label: string }[] = [
    { key: "stalls", label: "Stalls" },
    { key: "vertices", label: "Vertices" },
    { key: "driveLines", label: "Drive Lines" },
    { key: "spines", label: "Spines" },
    { key: "faces", label: "Faces" },
    { key: "miterFills", label: "Miter Fills" },
    { key: "islands", label: "Islands" },
    { key: "regions", label: "Regions" },
    { key: "paintLines", label: "Paint Lines" },
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

  // Debug toggles section (hierarchical)
  const debugTitle = document.createElement("h2");
  debugTitle.textContent = "Debug Toggles";
  container.appendChild(debugTitle);

  interface DebugGroupDef {
    label: string;
    toggles: { key: keyof DebugToggles; label: string; parent?: keyof DebugToggles }[];
  }

  const DEBUG_GROUPS: DebugGroupDef[] = [
    {
      label: "Corridor Merging",
      toggles: [
        { key: "miter_fills", label: "Miter Fills" },

        { key: "spike_removal", label: "Spike Removal" },
      ],
    },
    {
      label: "Spine Generation",
      toggles: [
        { key: "spine_end_trim", label: "Spine End Trim" },
      ],
    },
    {
      label: "Spine Post-processing",
      toggles: [
        { key: "spine_merging", label: "Spine Merging" },
        { key: "spine_extensions", label: "Spine Extensions" },
      ],
    },
    {
      label: "Stall Placement",
      toggles: [
        { key: "stall_face_clipping", label: "Stall-to-Face Clipping" },
        { key: "entrance_on_face_filter", label: "Entrance-on-Face Filter" },
        { key: "island_stall_dilation", label: "Island Stall Dilation" },
        { key: "island_corner_rounding", label: "Island Corner Rounding" },
      ],
    },
    {
      label: "Conflict Removal",
      toggles: [
        { key: "conflict_removal", label: "Conflict Removal" },
      ],
    },
  ];

  const checkboxMap = new Map<keyof DebugToggles, HTMLInputElement>();

  for (const group of DEBUG_GROUPS) {
    const groupLabel = document.createElement("div");
    groupLabel.className = "param-group";
    groupLabel.style.fontWeight = "bold";
    groupLabel.style.fontSize = "0.85em";
    groupLabel.style.opacity = "0.6";
    groupLabel.style.marginTop = "8px";
    groupLabel.textContent = group.label;
    container.appendChild(groupLabel);

    for (const def of group.toggles) {
      const row = document.createElement("div");
      row.className = "param-group layer-toggle";
      if (def.parent) {
        row.style.paddingLeft = "16px";
      }

      const label = document.createElement("label");
      const checkbox = document.createElement("input");
      checkbox.type = "checkbox";
      checkbox.checked = app.state.debug[def.key];
      checkboxMap.set(def.key, checkbox);

      checkbox.addEventListener("change", () => {
        app.state.debug[def.key] = checkbox.checked;
        // If disabling a parent, disable its children too.
        if (!checkbox.checked) {
          for (const t of group.toggles) {
            if (t.parent === def.key) {
              app.state.debug[t.key] = false;
              const childCb = checkboxMap.get(t.key);
              if (childCb) childCb.checked = false;
            }
          }
        }
        app.generate();
      });

      // Disable child checkbox when parent is off.
      if (def.parent) {
        const parentCb = checkboxMap.get(def.parent);
        if (parentCb && !parentCb.checked) {
          checkbox.disabled = true;
        }
        // Watch parent changes to enable/disable.
        parentCb?.addEventListener("change", () => {
          checkbox.disabled = !parentCb.checked;
          if (!parentCb.checked) {
            checkbox.checked = false;
            app.state.debug[def.key] = false;
          }
        });
      }

      label.appendChild(checkbox);
      label.appendChild(document.createTextNode(` ${def.label}`));
      row.appendChild(label);
      container.appendChild(row);
    }
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

  const totalStalls = app.state.lot.layout?.metrics.total_stalls ?? 0;
  if (totalStalls === 0) return;

  const row = document.createElement("div");
  row.className = "metric-row";
  row.innerHTML = `<span>Total Stalls</span><span class="value">${totalStalls}</span>`;
  container.appendChild(row);
}
