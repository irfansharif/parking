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
          return `total_stalls: ${m.total_stalls}`;
        }

        case "state": {
          return JSON.stringify(app.state.layout);
        }

        case "screenshot": {
          return "screenshot captured";
        }

        case "graph": {
          // Dump the effective aisle graph vertex positions.
          const graph = app.getEffectiveAisleGraph();
          if (!graph) return "error: no graph";
          const precision = parts[1] === "precise" ? 1 : 0;
          const fmt = (n: number) => precision ? n.toFixed(1) : String(Math.round(n));
          const lines = graph.vertices.map((v, i) =>
            `${i}: ${fmt(v.x)},${fmt(v.y)}`
          );
          return lines.join("\n");
        }

        case "hole": {
          if (!body) return "error: hole requires body with vertices";
          const points = parsePoints(body);
          app.state.boundary.holes.push(points);
          return `hole: ${points.length} vertices`;
        }

        case "drive-line": {
          if (!body) return "error: drive-line requires body with start and end points";
          const points = parsePoints(body);
          if (points.length < 2) return "error: drive-line requires 2 points";
          app.addDriveLine(points[0], points[1]);
          return `drive-line: ${points[0].x},${points[0].y} -> ${points[1].x},${points[1].y}`;
        }

        case "annotation": {
          const subtype = parts[1]; // "one-way", "delete-vertex", "delete-edge"
          if (!body) return "error: annotation requires body";
          const points = parsePoints(body);
          const noChain = parts.includes("no-chain");
          if (subtype === "one-way") {
            if (points.length < 2) return "error: one-way requires midpoint and travel_dir";
            app.state.annotations.push({
              kind: "OneWay",
              midpoint: points[0],
              travel_dir: points[1],
              chain: !noChain,
            });
            return `annotation one-way at ${points[0].x},${points[0].y}`;
          } else if (subtype === "delete-vertex") {
            if (points.length < 1) return "error: delete-vertex requires point";
            app.state.annotations.push({
              kind: "DeleteVertex",
              point: points[0],
            });
            return `annotation delete-vertex at ${points[0].x},${points[0].y}`;
          } else if (subtype === "delete-edge") {
            if (points.length < 2) return "error: delete-edge requires midpoint and edge_dir";
            app.state.annotations.push({
              kind: "DeleteEdge",
              midpoint: points[0],
              edge_dir: points[1],
              chain: !noChain,
            });
            return `annotation delete-edge at ${points[0].x},${points[0].y}`;
          }
          return `error: unknown annotation type '${subtype}'`;
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
