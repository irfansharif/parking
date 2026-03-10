import { App } from "./app";
import { Vec2 } from "./types";

export interface CommandAPI {
  execute(command: string, body?: string): string;
  getState(): object;
}

const KEY_ALIASES: Record<string, string> = {
  stall_angle: "stall_angle_deg",
  angle: "stall_angle_deg",
  width: "stall_width",
  depth: "stall_depth",
};

export function createCommandAPI(app: App): CommandAPI {
  return {
    execute(command: string, body?: string): string {
      const parts = command.trim().split(/\s+/);
      const cmd = parts[0];

      switch (cmd) {
        case "polygon": {
          const subtype = parts[1]; // "outer" or "hole"
          if (!body) return "error: polygon requires body with vertices";
          const points = parsePoints(body);
          if (subtype === "outer") {
            app.state.boundary.outer = points;
          } else if (subtype === "hole") {
            app.state.boundary.holes.push(points);
          }
          return `polygon ${subtype}: ${points.length} vertices`;
        }

        case "set": {
          const rest = parts.slice(1).join(" ");
          const eqIdx = rest.indexOf("=");
          if (eqIdx === -1) return "error: set requires key=value";
          const rawKey = rest.slice(0, eqIdx).replace(/-/g, "_");
          const key = KEY_ALIASES[rawKey] ?? rawKey;
          const val = parseFloat(rest.slice(eqIdx + 1));
          if (isNaN(val)) return `error: invalid value for ${key}`;
          app.setParam(key as any, val);
          return `set ${key}=${val}`;
        }

        case "vertex": {
          const action = parts[1];
          if (action === "add") {
            const coords = parts[2];
            const [x, y] = coords.split(",").map(Number);
            if (!app.state.aisleGraph) {
              app.state.aisleGraph = { vertices: [], edges: [] };
            }
            const idx = app.state.aisleGraph.vertices.length;
            app.state.aisleGraph.vertices.push({ x, y });
            return String(idx);
          } else if (action === "move") {
            const idx = parseInt(parts[2]);
            const coords = parts[3];
            const [x, y] = coords.split(",").map(Number);
            if (
              app.state.aisleGraph &&
              idx < app.state.aisleGraph.vertices.length
            ) {
              app.state.aisleGraph.vertices[idx] = { x, y };
            }
            return `moved vertex ${idx} to ${x},${y}`;
          }
          return "error: unknown vertex action";
        }

        case "edge": {
          const action = parts[1];
          if (action === "add") {
            const coords = parts[2];
            const [v1, v2] = coords.split(",").map(Number);
            if (!app.state.aisleGraph) {
              app.state.aisleGraph = { vertices: [], edges: [] };
            }
            app.state.aisleGraph.edges.push({
              start: v1,
              end: v2,
              width: app.state.params.aisle_width / 2,
            });
            return `edge ${v1}->${v2}`;
          }
          return "error: unknown edge action";
        }

        case "generate": {
          app.generate();
          const m = app.state.layout?.metrics;
          if (!m) return "error: generate failed";
          return `total_stalls: ${m.total_stalls}\nada_stalls: ${m.ada_stalls}`;
        }

        case "state": {
          return JSON.stringify(app.state.layout);
        }

        case "screenshot": {
          return "screenshot captured";
        }

        case "drag": {
          // Simulate dragging an aisle vertex.
          // Usage: drag <vertex-index> <x,y>
          // Simulates: mousedown on vertex, mousemove to (x,y), mouseup.
          const vertexIdx = parseInt(parts[1]);
          const coords = parts[2];
          const [x, y] = coords.split(",").map(Number);
          const ref = { type: "aisle" as const, index: vertexIdx };

          // Simulate the drag: moveVertex handles anchor capture and angle mirroring.
          app.moveVertex(ref, { x, y });

          // Simulate mouseup: clear manual graph and regenerate at new angle.
          app.state.aisleGraph = null;
          app.state.dragAnchor = null;
          app.generate();

          const angle = app.state.params.aisle_angle_deg;
          return `dragged vertex ${vertexIdx} to ${x},${y}\naisle_angle_deg: ${angle}`;
        }

        case "graph": {
          // Dump the effective aisle graph vertex positions.
          const graph = app.getEffectiveAisleGraph();
          if (!graph) return "error: no graph";
          const lines = graph.vertices.map((v, i) =>
            `${i}: ${Math.round(v.x)},${Math.round(v.y)}`
          );
          return lines.join("\n");
        }

        case "hole": {
          if (!body) return "error: hole requires body with vertices";
          const points = parsePoints(body);
          app.state.boundary.holes.push(points);
          return `hole: ${points.length} vertices`;
        }

        case "layers": {
          // Toggle layer visibility. Usage: layers stalls=on aisles=off spines=on
          const rest = parts.slice(1).join(" ");
          for (const pair of rest.split(/\s+/)) {
            const [key, val] = pair.split("=");
            if (key in app.state.layers) {
              (app.state.layers as any)[key] = val === "on" || val === "true";
            }
          }
          app.generate(); // re-render with new layer visibility
          return "layers updated";
        }

        default:
          return `error: unknown command '${cmd}'`;
      }
    },

    getState(): object {
      return app.state.layout ?? {};
    },
  };
}

function parsePoints(body: string): Vec2[] {
  return body
    .split("\n")
    .map((line) => line.trim())
    .filter((line) => line.length > 0 && !line.startsWith("#"))
    .map((line) => {
      const [x, y] = line.split(",").map(Number);
      return { x, y };
    });
}
