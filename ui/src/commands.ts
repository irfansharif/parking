import { App, EdgeRef } from "./app";
import { Annotation, Vec2, computeBoundaryPin } from "./types";
import { findCollinearChain } from "./interaction";

export interface CommandAPI {
  execute(command: string, body?: string): string;
  getState(): object;
}

const KEY_ALIASES: Record<string, string> = {
  stall_angle: "stall_angle_deg",
  angle: "stall_angle_deg",
  width: "stall_width",
  depth: "stall_depth",
  // Legacy aliases for the now-renamed stalls_per_face parameter.
  cross_aisle_max_run: "stalls_per_face",
  cross_aisle_spacing: "stalls_per_face",
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

        case "debug": {
          // Toggle a field on app.state.debug. Usage: debug <key>=<bool>
          const rest = parts.slice(1).join(" ");
          const eqIdx = rest.indexOf("=");
          if (eqIdx === -1) return "error: debug requires key=value";
          const key = rest.slice(0, eqIdx).replace(/-/g, "_");
          const rawVal = rest.slice(eqIdx + 1);
          if (rawVal !== "true" && rawVal !== "false") {
            return `error: debug ${key} requires true|false`;
          }
          (app.state.debug as any)[key] = rawVal === "true";
          for (const lot of app.state.lots) lot.aisleGraph = null;
          app.generate();
          return `debug ${key}=${rawVal}`;
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

        case "dormant": {
          // Report the dormant annotation indices from the most recent
          // generate. Each entry is an index into lot.annotations.
          const layout = app.activeLot().layout;
          if (!layout) return "error: no layout";
          const dormant = (layout as any).dormant_annotations ?? [];
          if (dormant.length === 0) {
            return "dormant_annotations: 0";
          }
          return `dormant_annotations: ${dormant.length} (${dormant.join(",")})`;
        }

        case "screenshot": {
          return "screenshot captured";
        }

        case "graph": {
          // Dump the effective aisle graph. Default is vertex positions;
          // `graph edges` dumps deduplicated edges as `ei: start->end`.
          const graph = app.getEffectiveAisleGraph();
          if (!graph) return "error: no graph";
          if (parts[1] === "edges") {
            const seen = new Set<string>();
            const lines: string[] = [];
            for (let ei = 0; ei < graph.edges.length; ei++) {
              const e = graph.edges[ei];
              const key = Math.min(e.start, e.end) + "," + Math.max(e.start, e.end);
              if (seen.has(key)) continue;
              seen.add(key);
              lines.push(`${ei}: ${e.start}->${e.end}`);
            }
            return lines.join("\n");
          }
          const precision = parts[1] === "precise" ? 1 : 0;
          const fmt = (n: number) => precision ? n.toFixed(1) : String(Math.round(n));
          const lines = graph.vertices.map((v, i) =>
            `${i}: ${fmt(v.x)},${fmt(v.y)}`
          );
          return lines.join("\n");
        }

        case "edge-select": {
          // Test-only: populate `selectedEdge` so a subsequent
          // edge-delete / edge-cycle mirrors what the UI does after a
          // real click. Mode chooses between chain (entire collinear
          // run, from a single click) and segment (just the seed edge,
          // from a double-click).
          //
          // Usage:
          //   edge-select seed=<i> mode=chain|segment
          //   edge-select near=<x>,<y> mode=chain|segment
          const graph = app.getEffectiveAisleGraph();
          if (!graph) return "error: no graph";
          let seedIdx: number | undefined;
          let mode: "chain" | "segment" = "chain";
          let nearX: number | undefined;
          let nearY: number | undefined;
          for (const p of parts.slice(1)) {
            const [k, v] = p.split("=");
            if (k === "seed") {
              seedIdx = Number(v);
            } else if (k === "mode") {
              if (v !== "chain" && v !== "segment") {
                return "error: edge-select mode must be chain or segment";
              }
              mode = v;
            } else if (k === "near") {
              const [xs, ys] = v.split(",");
              nearX = Number(xs);
              nearY = Number(ys);
            }
          }
          if (seedIdx === undefined && (nearX === undefined || nearY === undefined)) {
            return "error: edge-select requires seed=<i> or near=<x>,<y>";
          }
          if (seedIdx === undefined) {
            // Pick the deduplicated edge whose segment is closest to
            // the query point — same logic as hitTestEdge but without
            // the screen-space radius filter.
            const seen = new Set<string>();
            let best: { idx: number; dist: number } | null = null;
            for (let i = 0; i < graph.edges.length; i++) {
              const e = graph.edges[i];
              const key = Math.min(e.start, e.end) + "," + Math.max(e.start, e.end);
              if (seen.has(key)) continue;
              seen.add(key);
              const a = graph.vertices[e.start];
              const b = graph.vertices[e.end];
              const dist = pointToSegmentDist({ x: nearX!, y: nearY! }, a, b);
              if (!best || dist < best.dist) best = { idx: i, dist };
            }
            if (!best) return "error: no edges";
            seedIdx = best.idx;
          }
          const chain = mode === "chain"
            ? findCollinearChain(graph, seedIdx)
            : [seedIdx];
          const ref: EdgeRef = { index: seedIdx, chain, mode };
          app.state.selectedEdge = ref;
          app.state.selectedVertex = null;
          return `edge-select seed=${seedIdx} mode=${mode} chain_len=${chain.length}`;
        }

        case "edge-delete": {
          // Test-only: invoke the UI delete path on the currently
          // selected edge (see edge-select).
          if (!app.state.selectedEdge) return "error: no selectedEdge";
          app.deleteSelectedEdge();
          return "edge-delete";
        }

        case "edge-cycle": {
          // Test-only: invoke the UI direction-cycle path on the
          // currently selected edge (see edge-select).
          const sel = app.state.selectedEdge;
          if (!sel) return "error: no selectedEdge";
          app.cycleEdgeDirection(sel);
          return "edge-cycle";
        }

        case "annotations": {
          // Dump active annotations in a compact, stable form. Anchor
          // is printed when set (chain-mode annotations carry a
          // separate click-anchor for marker placement).
          const lot = app.activeLot();
          const lines = lot.annotations
            .filter((a: any) => a._active !== false)
            .map((a: any) => formatAnnotation(a));
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
          // Parse optional id=N, hole-pin=holeIndex,vertexIndex, and
          // `partitions` flag. When id= is present, reuse it verbatim
          // (round-trips through dumpFixture preserve drive-line ids
          // so annotations keyed off them stay addressable); bump the
          // allocator past it so fresh mints don't collide.
          const idMatch = command.match(/\bid=(\d+)/);
          const pinMatch = command.match(/hole-pin=(\d+),(\d+)/);
          const partitions = /\bpartitions\b/.test(command);
          const lot = app.activeLot();
          const id = idMatch ? parseInt(idMatch[1]) : app.newDriveLineId();
          if (idMatch) app.bumpDriveLineId(id);
          if (pinMatch) {
            const holeIndex = parseInt(pinMatch[1]);
            const vertexIndex = parseInt(pinMatch[2]);
            const bpin = computeBoundaryPin(points[1], lot.boundary.outer);
            lot.driveLines.push({
              id,
              start: points[0],
              end: bpin.pos,
              holePin: { holeIndex, vertexIndex },
              boundaryPin: { edgeIndex: bpin.edgeIndex, t: bpin.t },
              partitions: true,
            });
          } else {
            lot.driveLines.push({
              id,
              start: points[0],
              end: points[1],
              partitions,
            });
          }
          app.generate();
          const idPrefix = idMatch ? ` id=${idMatch[1]}` : "";
          const pinSuffix = pinMatch
            ? ` hole-pin=${pinMatch[1]},${pinMatch[2]}`
            : partitions ? ` partitions` : "";
          return `drive-line: ${points[0].x},${points[0].y} -> ${points[1].x},${points[1].y}${idPrefix}${pinSuffix}`;
        }

        case "annotation": {
          const subtype = parts[1]; // "one-way", "delete-vertex", "delete-edge"
          const isAbstract =
            subtype === "abstract-delete-vertex" ||
            subtype === "abstract-delete-edge" ||
            subtype === "abstract-one-way" ||
            subtype === "abstract-two-way-oriented";
          // These subtypes carry their payload on the command line,
          // not in the body, so they don't need a body. The abstract
          // variants above plus `delete-vertex-at-index` (which wraps
          // an interactive UI entry point).
          const bodyless = isAbstract || subtype === "delete-vertex-at-index";
          if (!bodyless && !body) return "error: annotation requires body";
          const points = body ? parsePoints(body) : [];
          const noChain = parts.includes("no-chain");
          const lot = app.activeLot();
          if (subtype === "delete-vertex-at-index") {
            // Usage: annotation delete-vertex-at-index index=<i>
            // Routes through deleteAisleVertexByAnnotation so it
            // exercises the UI auto-upgrade path — under the
            // abstract-stamp flag this produces an
            // AbstractDeleteVertex, otherwise a world-space
            // DeleteVertex.
            let idx: number | undefined;
            for (const p of parts.slice(2)) {
              const [k, v] = p.split("=");
              if (k === "index") idx = Number(v);
            }
            if (idx === undefined || Number.isNaN(idx)) {
              return "error: delete-vertex-at-index requires index=<i>";
            }
            app.deleteAisleVertexByAnnotation(idx);
            return `annotation delete-vertex-at-index index=${idx}`;
          } else if (
            subtype === "abstract-delete-vertex" ||
            subtype === "abstract-delete-edge" ||
            subtype === "abstract-one-way" ||
            subtype === "abstract-two-way-oriented"
          ) {
            // Usage:
            //   annotation abstract-delete-vertex    region=<id> x=<xi> y=<yi>
            //   annotation abstract-delete-edge      region=<id> from=<xa>,<ya> to=<xb>,<yb>
            //   annotation abstract-one-way          region=<id> from=<xa>,<ya> to=<xb>,<yb>
            //   annotation abstract-two-way-oriented region=<id> from=<xa>,<ya> to=<xb>,<yb>
            //
            // <id> is a RegionId, accepted as decimal u64 or "0x..." hex.
            let regionId: number | undefined;
            let xi: number | undefined;
            let yi: number | undefined;
            let xa: number | undefined;
            let ya: number | undefined;
            let xb: number | undefined;
            let yb: number | undefined;
            for (const p of parts.slice(2)) {
              const [k, v] = p.split("=");
              if (k === "region") {
                regionId = v.startsWith("0x") ? parseInt(v.slice(2), 16) : Number(v);
              } else if (k === "x") {
                xi = Number(v);
              } else if (k === "y") {
                yi = Number(v);
              } else if (k === "from") {
                const [a, b] = v.split(",");
                xa = Number(a);
                ya = Number(b);
              } else if (k === "to") {
                const [a, b] = v.split(",");
                xb = Number(a);
                yb = Number(b);
              }
            }
            if (regionId === undefined || Number.isNaN(regionId)) {
              return `error: ${subtype} requires region=<id>`;
            }
            if (subtype === "abstract-delete-vertex") {
              if (xi === undefined || yi === undefined) {
                return "error: abstract-delete-vertex requires x=<xi> y=<yi>";
              }
              lot.annotations.push({
                kind: "AbstractDeleteVertex",
                region: regionId,
                xi,
                yi,
              } as any);
              return `annotation abstract-delete-vertex region=0x${regionId.toString(16).padStart(16, "0")} x=${xi} y=${yi}`;
            } else {
              if (
                xa === undefined ||
                ya === undefined ||
                xb === undefined ||
                yb === undefined
              ) {
                return `error: ${subtype} requires from=<xa>,<ya> to=<xb>,<yb>`;
              }
              const kind = (
                subtype === "abstract-delete-edge"
                  ? "AbstractDeleteEdge"
                  : subtype === "abstract-one-way"
                    ? "AbstractOneWay"
                    : "AbstractTwoWayOriented"
              ) as Annotation["kind"];
              lot.annotations.push({
                kind,
                region: regionId,
                xa,
                ya,
                xb,
                yb,
              } as any);
              return `annotation ${subtype} region=0x${regionId.toString(16).padStart(16, "0")} from=${xa},${ya} to=${xb},${yb}`;
            }
          }
          return `error: unknown annotation type '${subtype}'`;
        }

        case "region-override": {
          // Usage: region-override region=<id> angle=90 offset=10
          //
          // <id> is a RegionId (stable hash of the bounding separator
          // pair) — accepted as a decimal u64 or "0x..." hex. Look it up
          // in the last layout's region_debug.regions[].id.
          const lot = app.activeLot();
          let regionId: number | undefined;
          let angle: number | undefined;
          let offset: number | undefined;
          for (const p of parts.slice(1)) {
            const [k, v] = p.split("=");
            if (k === "region") {
              regionId = v.startsWith("0x") ? parseInt(v.slice(2), 16) : Number(v);
            } else if (k === "angle") {
              angle = parseFloat(v);
            } else if (k === "offset") {
              offset = parseFloat(v);
            }
          }
          if (regionId === undefined || Number.isNaN(regionId)) {
            return "error: region-override requires region=<id>";
          }
          const regionKey = String(regionId);
          if (!lot.regionOverrides[regionKey]) {
            lot.regionOverrides[regionKey] = {};
          }
          if (angle !== undefined) lot.regionOverrides[regionKey].angle = angle;
          if (offset !== undefined) lot.regionOverrides[regionKey].offset = offset;
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

function pointToSegmentDist(p: Vec2, a: Vec2, b: Vec2): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return Math.sqrt((p.x - a.x) ** 2 + (p.y - a.y) ** 2);
  let t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const projX = a.x + t * dx;
  const projY = a.y + t * dy;
  return Math.sqrt((p.x - projX) ** 2 + (p.y - projY) ** 2);
}

function formatAnnotation(a: any): string {
  const anchor = a.anchor
    ? ` anchor=${a.anchor.xa},${a.anchor.ya}->${a.anchor.xb},${a.anchor.yb}`
    : "";
  switch (a.kind) {
    case "AbstractDeleteVertex":
      return `abstract-delete-vertex region=0x${a.region.toString(16).padStart(16, "0")} x=${a.xi} y=${a.yi}`;
    case "AbstractDeleteEdge":
      return `abstract-delete-edge region=0x${a.region.toString(16).padStart(16, "0")} from=${a.xa},${a.ya} to=${a.xb},${a.yb}${anchor}`;
    case "AbstractOneWay":
      return `abstract-one-way region=0x${a.region.toString(16).padStart(16, "0")} from=${a.xa},${a.ya} to=${a.xb},${a.yb}${anchor}`;
    case "AbstractTwoWayOriented":
      return `abstract-two-way-oriented region=0x${a.region.toString(16).padStart(16, "0")} from=${a.xa},${a.ya} to=${a.xb},${a.yb}${anchor}`;
    case "SpliceDeleteVertex":
      return `splice-delete-vertex line=${a.drive_line_id} t=${a.t}`;
    case "SpliceDeleteEdge":
      return `splice-delete-edge line=${a.drive_line_id} from=${a.ta} to=${a.tb}`;
    case "SpliceOneWay":
      return `splice-one-way line=${a.drive_line_id} from=${a.ta} to=${a.tb}`;
    case "SpliceTwoWayOriented":
      return `splice-two-way-oriented line=${a.drive_line_id} from=${a.ta} to=${a.tb}`;
    default:
      return `unknown ${JSON.stringify(a)}`;
  }
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
