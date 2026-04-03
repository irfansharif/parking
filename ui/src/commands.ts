import { App } from "./app";
import { Vec2, computeBoundaryPin } from "./types";

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
        case "clear": {
          const lot = app.activeLot();
          lot.boundary = { outer: [], holes: [] };
          lot.driveLines = [];
          lot.annotations = [];
          lot.aisleGraph = null;
          return "cleared";
        }

        case "new-lot": {
          // Create a new empty lot and switch to it.
          const newLot = app.addLot({ outer: [], holes: [] });
          return `new-lot: ${newLot.id}`;
        }

        case "select-lot": {
          // Switch active lot by index (0-based).
          const idx = parseInt(parts[1]);
          if (idx < 0 || idx >= app.state.lots.length) {
            return `error: lot index ${idx} out of range (${app.state.lots.length} lots)`;
          }
          app.state.activeLotId = app.state.lots[idx].id;
          return `select-lot: ${idx}`;
        }

        case "polygon": {
          const subtype = parts[1]; // "outer" or "hole"
          if (!body) return "error: polygon requires body with vertices";
          const points = parsePoints(body);
          const lot = app.activeLot();
          if (subtype === "outer") {
            lot.boundary.outer = points;
            lot.boundary.holes = [];
            lot.driveLines = [];
            lot.annotations = [];
            lot.aisleGraph = null;
          } else if (subtype === "hole") {
            lot.boundary.holes.push(points);
          }
          return `polygon ${subtype}: ${points.length} vertices`;
        }

        case "set": {
          const rest = parts.slice(1).join(" ");
          const eqIdx = rest.indexOf("=");
          if (eqIdx === -1) return "error: set requires key=value";
          const rawKey = rest.slice(0, eqIdx).replace(/-/g, "_");
          const key = KEY_ALIASES[rawKey] ?? rawKey;
          const rawVal = rest.slice(eqIdx + 1);
          // Handle boolean params (use_regions).
          if (rawVal === "true" || rawVal === "false") {
            (app.state.params as any)[key] = rawVal === "true";
            for (const lot of app.state.lots) lot.aisleGraph = null;
            app.generate();
            return `set ${key}=${rawVal}`;
          }
          const val = parseFloat(rawVal);
          if (isNaN(val)) return `error: invalid value for ${key}`;
          app.setParam(key as any, val);
          return `set ${key}=${val}`;
        }

        case "vertex": {
          const action = parts[1];
          const lot = app.activeLot();
          if (action === "add") {
            const coords = parts[2];
            const [x, y] = coords.split(",").map(Number);
            if (!lot.aisleGraph) {
              lot.aisleGraph = { vertices: [], edges: [] };
            }
            const idx = lot.aisleGraph.vertices.length;
            lot.aisleGraph.vertices.push({ x, y });
            return String(idx);
          } else if (action === "move") {
            const idx = parseInt(parts[2]);
            const coords = parts[3];
            const [x, y] = coords.split(",").map(Number);
            if (lot.aisleGraph && idx < lot.aisleGraph.vertices.length) {
              lot.aisleGraph.vertices[idx] = { x, y };
            }
            return `moved vertex ${idx} to ${x},${y}`;
          }
          return "error: unknown vertex action";
        }

        case "edge": {
          const action = parts[1];
          const lot = app.activeLot();
          if (action === "add") {
            const coords = parts[2];
            const [v1, v2] = coords.split(",").map(Number);
            if (!lot.aisleGraph) {
              lot.aisleGraph = { vertices: [], edges: [] };
            }
            lot.aisleGraph.edges.push({
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
          if (app.state.lots.length === 1) {
            const m = app.activeLot().layout?.metrics;
            if (!m) return "error: generate failed";
            return `total_stalls: ${m.total_stalls}`;
          }
          // Multi-lot: report per-lot and total.
          let total = 0;
          const perLot: string[] = [];
          for (let i = 0; i < app.state.lots.length; i++) {
            const m = app.state.lots[i].layout?.metrics;
            const n = m?.total_stalls ?? 0;
            perLot.push(`lot_${i}: ${n}`);
            total += n;
          }
          return `total_stalls: ${total}\n${perLot.join("\n")}`;
        }

        case "state": {
          return JSON.stringify(app.activeLot().layout);
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
          app.activeLot().boundary.holes.push(points);
          return `hole: ${points.length} vertices`;
        }

        case "drive-line": {
          if (!body) return "error: drive-line requires body with start and end points";
          const points = parsePoints(body);
          if (points.length < 2) return "error: drive-line requires 2 points";
          // Parse optional hole-pin=holeIndex,vertexIndex
          const pinMatch = command.match(/hole-pin=(\d+),(\d+)/);
          const lot = app.activeLot();
          if (pinMatch) {
            const holeIndex = parseInt(pinMatch[1]);
            const vertexIndex = parseInt(pinMatch[2]);
            const bpin = computeBoundaryPin(points[1], lot.boundary.outer);
            lot.driveLines.push({
              start: points[0],
              end: bpin.pos,
              holePin: { holeIndex, vertexIndex },
              boundaryPin: { edgeIndex: bpin.edgeIndex, t: bpin.t },
            });
          } else {
            lot.driveLines.push({ start: points[0], end: points[1] });
          }
          app.generate();
          const pinSuffix = pinMatch ? ` hole-pin=${pinMatch[1]},${pinMatch[2]}` : "";
          return `drive-line: ${points[0].x},${points[0].y} -> ${points[1].x},${points[1].y}${pinSuffix}`;
        }

        case "annotation": {
          const subtype = parts[1]; // "one-way", "delete-vertex", "delete-edge"
          if (!body) return "error: annotation requires body";
          const points = parsePoints(body);
          const noChain = parts.includes("no-chain");
          const lot = app.activeLot();
          if (subtype === "one-way") {
            if (points.length < 2) return "error: one-way requires midpoint and travel_dir";
            lot.annotations.push({
              kind: "OneWay",
              midpoint: points[0],
              travel_dir: points[1],
              chain: !noChain,
            });
            return `annotation one-way at ${points[0].x},${points[0].y}`;
          } else if (subtype === "two-way-oriented") {
            if (points.length < 2) return "error: two-way-oriented requires midpoint and travel_dir";
            lot.annotations.push({
              kind: "TwoWayOriented",
              midpoint: points[0],
              travel_dir: points[1],
              chain: !noChain,
            });
            return `annotation two-way-oriented at ${points[0].x},${points[0].y}`;
          } else if (subtype === "delete-vertex") {
            if (points.length < 1) return "error: delete-vertex requires point";
            lot.annotations.push({
              kind: "DeleteVertex",
              point: points[0],
            });
            return `annotation delete-vertex at ${points[0].x},${points[0].y}`;
          } else if (subtype === "delete-edge") {
            if (points.length < 2) return "error: delete-edge requires midpoint and edge_dir";
            lot.annotations.push({
              kind: "DeleteEdge",
              midpoint: points[0],
              edge_dir: points[1],
              chain: !noChain,
            });
            return `annotation delete-edge at ${points[0].x},${points[0].y}`;
          }
          return `error: unknown annotation type '${subtype}'`;
        }

        case "region-override": {
          // Usage: region-override region=0 angle=90 offset=10
          const lot = app.activeLot();
          let regionIdx = -1;
          let angle: number | undefined;
          let offset: number | undefined;
          for (const p of parts.slice(1)) {
            const [k, v] = p.split("=");
            if (k === "region") regionIdx = parseInt(v);
            else if (k === "angle") angle = parseFloat(v);
            else if (k === "offset") offset = parseFloat(v);
          }
          if (regionIdx < 0) return "error: region-override requires region=N";
          if (!lot.regionOverrides[regionIdx]) {
            lot.regionOverrides[regionIdx] = {};
          }
          if (angle !== undefined) lot.regionOverrides[regionIdx].angle = angle;
          if (offset !== undefined) lot.regionOverrides[regionIdx].offset = offset;
          lot.aisleGraph = null;
          app.generate();
          const spec = parts.slice(1).join(" ");
          return spec;
        }

        case "curve": {
          // Set a cubic bezier curve on a boundary edge.
          // Usage: curve outer 1 cp1x,cp1y cp2x,cp2y
          //        curve hole 0 2 cp1x,cp1y cp2x,cp2y
          const target = parts[1]; // "outer" or "hole"
          const lot = app.activeLot();
          if (target === "outer") {
            const edgeIdx = parseInt(parts[2]);
            const [cp1x, cp1y] = parts[3].split(",").map(Number);
            const [cp2x, cp2y] = parts[4].split(",").map(Number);
            if (!lot.boundary.outer_curves) {
              lot.boundary.outer_curves = new Array(lot.boundary.outer.length).fill(null);
            }
            lot.boundary.outer_curves[edgeIdx] = {
              cp1: { x: cp1x, y: cp1y },
              cp2: { x: cp2x, y: cp2y },
            };
            lot.aisleGraph = null;
            return `curve outer edge ${edgeIdx}`;
          } else if (target === "hole") {
            const holeIdx = parseInt(parts[2]);
            const edgeIdx = parseInt(parts[3]);
            const [cp1x, cp1y] = parts[4].split(",").map(Number);
            const [cp2x, cp2y] = parts[5].split(",").map(Number);
            if (!lot.boundary.hole_curves) lot.boundary.hole_curves = [];
            if (!lot.boundary.hole_curves[holeIdx]) {
              lot.boundary.hole_curves[holeIdx] = new Array(lot.boundary.holes[holeIdx].length).fill(null);
            }
            lot.boundary.hole_curves[holeIdx][edgeIdx] = {
              cp1: { x: cp1x, y: cp1y },
              cp2: { x: cp2x, y: cp2y },
            };
            lot.aisleGraph = null;
            return `curve hole ${holeIdx} edge ${edgeIdx}`;
          }
          return "error: curve target must be 'outer' or 'hole'";
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
      return app.activeLot().layout ?? {};
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
