import { App, EdgeRef } from "./app";
import {
  Annotation,
  GridStop,
  PerimeterLoop,
  Target,
  AisleDirection,
  Vec2,
  computeBoundaryPin,
} from "./types";
import { findCollinearChain } from "./interaction";
import { point_to_segment_dist_js } from "./wasm/parking_lot_engine";

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
          const lot = app.state.lot;
          lot.boundary = {
            outer: [],
            holes: [],
            outer_arcs: [],
            hole_arcs: [],
            outer_ids: [],
            hole_ids: [],
          };
          lot.driveLines = [];
          lot.annotations = [];
          return "cleared";
        }

        case "polygon": {
          const subtype = parts[1]; // "outer" or "hole"
          if (!body) return "error: polygon requires body with vertices";
          const points = parsePoints(body);
          const lot = app.state.lot;
          if (subtype === "outer") {
            lot.boundary.outer = points;
            lot.boundary.holes = [];
            lot.driveLines = [];
            lot.annotations = [];
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
          app.generate();
          return `debug ${key}=${rawVal}`;
        }

        case "generate": {
          app.generate();
          const m = app.state.lot.layout?.metrics;
          if (!m) return "error: generate failed";
          return `total_stalls: ${m.total_stalls}`;
        }

        case "state": {
          return JSON.stringify(app.state.lot.layout);
        }

        case "dormant": {
          // Report the dormant annotations from the most recent generate.
          // Each entry pairs an index into lot.annotations with the
          // engine's reason for skipping it.
          const layout = app.state.lot.layout;
          if (!layout) return "error: no layout";
          const dormant = layout.dormant_annotations ?? [];
          if (dormant.length === 0) {
            return "dormant_annotations: 0";
          }
          const lines = dormant.map((d) => `  ${d.index}: ${d.reason}`);
          return `dormant_annotations: ${dormant.length}\n${lines.join("\n")}`;
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
          const lot = app.state.lot;
          const lines = lot.annotations
            .filter((a: any) => a._active !== false)
            .map((a: any) => formatAnnotation(a));
          return lines.join("\n");
        }

        case "hole": {
          if (!body) return "error: hole requires body with vertices";
          const points = parsePoints(body);
          app.state.lot.boundary.holes.push(points);
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
          const lot = app.state.lot;
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
          // Two subtype families:
          //   * `delete-vertex-at-index index=<i>` — test-only UI
          //     passthrough that exercises deleteAisleVertexByAnnotation.
          //   * `<delete-vertex|delete-edge|direction> on=<substrate> ...`
          //     — all other annotations in the new substrate-local model.
          const subtype = parts[1];
          const lot = app.state.lot;
          if (subtype === "delete-vertex-at-index") {
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
          }
          if (
            subtype !== "delete-vertex" &&
            subtype !== "delete-edge" &&
            subtype !== "direction"
          ) {
            return `error: unknown annotation type '${subtype}'`;
          }
          const parsed = parseAnnotationArgs(parts.slice(2));
          if (typeof parsed === "string") {
            return `error: ${subtype}: ${parsed}`;
          }
          const { target, traffic } = parsed;
          let ann: Annotation;
          if (subtype === "direction") {
            if (!traffic) {
              return "error: direction requires traffic=<...>";
            }
            ann = { kind: "Direction", target, traffic };
          } else if (subtype === "delete-vertex") {
            ann = { kind: "DeleteVertex", target };
          } else {
            ann = { kind: "DeleteEdge", target };
          }
          lot.annotations.push(ann);
          return `annotation ${formatAnnotation(ann)}`;
        }

        case "region-override": {
          // Usage: region-override region=<id> angle=90 offset=10
          //
          // <id> is a RegionId (stable hash of the bounding separator
          // pair) — accepted as a decimal u64 or "0x..." hex. Look it up
          // in the last layout's region_debug.regions[].id.
          const lot = app.state.lot;
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
          app.generate();
          const spec = parts.slice(1).join(" ");
          return spec;
        }

        case "arc": {
          // Set a circular arc (bulge) on a boundary edge.
          // Usage: arc outer <edgeIdx> <bulge>
          //        arc hole <holeIdx> <edgeIdx> <bulge>
          const target = parts[1]; // "outer" or "hole"
          const lot = app.state.lot;
          if (target === "outer") {
            const edgeIdx = parseInt(parts[2]);
            const bulge = parseFloat(parts[3]);
            if (!lot.boundary.outer_arcs) {
              lot.boundary.outer_arcs = new Array(lot.boundary.outer.length).fill(null);
            }
            lot.boundary.outer_arcs[edgeIdx] = { bulge };
            return `arc outer edge ${edgeIdx}`;
          } else if (target === "hole") {
            const holeIdx = parseInt(parts[2]);
            const edgeIdx = parseInt(parts[3]);
            const bulge = parseFloat(parts[4]);
            if (!lot.boundary.hole_arcs) lot.boundary.hole_arcs = [];
            if (!lot.boundary.hole_arcs[holeIdx]) {
              lot.boundary.hole_arcs[holeIdx] = new Array(lot.boundary.holes[holeIdx].length).fill(null);
            }
            lot.boundary.hole_arcs[holeIdx][edgeIdx] = { bulge };
            return `arc hole ${holeIdx} edge ${edgeIdx}`;
          }
          return "error: arc target must be 'outer' or 'hole'";
        }

        case "layers": {
          // Toggle layer visibility. Usage: layers stalls=on spines=on
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
      return app.state.lot.layout ?? {};
    },
  };
}

function pointToSegmentDist(p: Vec2, a: Vec2, b: Vec2): number {
  return point_to_segment_dist_js(p.x, p.y, a.x, a.y, b.x, b.y);
}

function formatAnnotation(a: Annotation): string {
  const targetStr = formatTarget(a.target);
  switch (a.kind) {
    case "DeleteVertex":
      return `delete-vertex ${targetStr}`;
    case "DeleteEdge":
      return `delete-edge ${targetStr}`;
    case "Direction":
      return `direction ${targetStr} traffic=${formatTraffic(a.traffic)}`;
  }
}

function formatTarget(t: Target): string {
  if (t.on === "Grid") {
    const axisStr = t.axis === "X" ? "x" : "y";
    const regionHex = "0x" + t.region.toString(16).padStart(16, "0");
    let scope: string;
    if (t.range == null) {
      scope = "range=whole";
    } else if (gridStopsEqual(t.range[0], t.range[1])) {
      scope = `at=${formatStop(t.range[0])}`;
    } else {
      scope = `range=${formatStop(t.range[0])},${formatStop(t.range[1])}`;
    }
    return `on=grid region=${regionHex} axis=${axisStr} coord=${t.coord} ${scope}`;
  }
  if (t.on === "DriveLine") {
    return `on=drive-line id=${t.id} t=${fmtNum(t.t)}`;
  }
  return `on=perimeter loop=${formatLoop(t.loop)} edge=${t.start}→${t.end} t=${fmtNum(t.t)}`;
}

function formatStop(s: GridStop): string {
  if (s.at === "Lattice") return `lattice:${s.other}`;
  if (s.at === "CrossesDriveLine") return `crosses-drive-line:${s.id}`;
  return `crosses-perimeter:${formatLoop(s.loop)}`;
}

function formatLoop(l: PerimeterLoop): string {
  return l.kind === "Outer" ? "outer" : `hole:${l.index}`;
}

function formatTraffic(t: AisleDirection): string {
  switch (t) {
    case "OneWay":
      return "one-way";
    case "OneWayReverse":
      return "one-way-reverse";
    case "TwoWayReverse":
      return "two-way-reverse";
  }
}

function gridStopsEqual(a: GridStop, b: GridStop): boolean {
  if (a.at === "Lattice" && b.at === "Lattice") return a.other === b.other;
  if (a.at === "CrossesDriveLine" && b.at === "CrossesDriveLine") return a.id === b.id;
  if (a.at === "CrossesPerimeter" && b.at === "CrossesPerimeter") {
    if (a.loop.kind !== b.loop.kind) return false;
    if (a.loop.kind === "Outer") return true;
    return a.loop.kind === "Hole" && b.loop.kind === "Hole" && a.loop.index === b.loop.index;
  }
  return false;
}

function fmtNum(n: number): string {
  // Match lib.rs fmt_coord: integer if close to round, else 2-decimal.
  if (Math.abs(n - Math.round(n)) < 1e-6) return `${Math.round(n)}`;
  return n.toFixed(2);
}

/**
 * Parse the `key=value` arg list following an `annotation <kind> ...`
 * command into a `Target` plus optional `AisleDirection`. Returns an
 * error message on malformed input.
 */
function parseAnnotationArgs(
  args: string[],
): { target: Target; traffic: AisleDirection | null } | string {
  const kv: Record<string, string> = {};
  for (const p of args) {
    const eq = p.indexOf("=");
    if (eq < 0) continue;
    kv[p.slice(0, eq)] = p.slice(eq + 1);
  }
  const on = kv["on"];
  if (!on) return "missing on=<substrate>";
  let target: Target;
  if (on === "grid") {
    const region = parseRegionId(kv["region"]);
    if (region == null) return "grid requires region=<id>";
    const axisStr = kv["axis"];
    if (axisStr !== "x" && axisStr !== "y") return "grid requires axis=x|y";
    const axis = axisStr === "x" ? "X" : "Y";
    const coord = Number(kv["coord"]);
    if (Number.isNaN(coord)) return "grid requires coord=<n>";
    let range: [GridStop, GridStop] | null;
    if (kv["range"] === "whole") {
      range = null;
    } else if (kv["at"] !== undefined) {
      const s = parseStop(kv["at"]);
      if (!s) return `grid: bad stop '${kv["at"]}'`;
      range = [s, s];
    } else if (kv["range"] !== undefined) {
      const parts = kv["range"].split(",");
      if (parts.length !== 2) return "grid range=<s1>,<s2> requires two stops";
      const s1 = parseStop(parts[0]);
      const s2 = parseStop(parts[1]);
      if (!s1 || !s2) return `grid: bad range '${kv["range"]}'`;
      range = [s1, s2];
    } else {
      return "grid requires range=whole|<s1>,<s2> or at=<stop>";
    }
    target = { on: "Grid", region, axis, coord, range };
  } else if (on === "drive-line") {
    const id = Number(kv["id"]);
    const t = Number(kv["t"]);
    if (Number.isNaN(id) || Number.isNaN(t)) {
      return "drive-line requires id=<n> t=<t>";
    }
    target = { on: "DriveLine", id, t };
  } else if (on === "perimeter") {
    const loop = parseLoop(kv["loop"]);
    const start = Number(kv["start"]);
    const end = Number(kv["end"]);
    const tNum = Number(kv["t"]);
    if (!loop || Number.isNaN(start) || Number.isNaN(end) || Number.isNaN(tNum)) {
      return "perimeter requires loop=<loop> start=<vid> end=<vid> t=<t>";
    }
    target = { on: "Perimeter", loop, start, end, t: tNum };
  } else {
    return `unknown substrate 'on=${on}'`;
  }
  const trafficStr = kv["traffic"];
  const traffic: AisleDirection | null = trafficStr ? parseTraffic(trafficStr) : null;
  if (trafficStr && !traffic) return `bad traffic '${trafficStr}'`;
  return { target, traffic };
}

function parseRegionId(v: string | undefined): number | null {
  if (!v) return null;
  const n = v.startsWith("0x") ? parseInt(v.slice(2), 16) : Number(v);
  return Number.isNaN(n) ? null : n;
}

function parseStop(s: string): GridStop | null {
  const colon = s.indexOf(":");
  if (colon < 0) return null;
  const head = s.slice(0, colon);
  const tail = s.slice(colon + 1);
  if (head === "lattice") {
    const other = Number(tail);
    return Number.isNaN(other) ? null : { at: "Lattice", other };
  }
  if (head === "crosses-drive-line") {
    const id = Number(tail);
    return Number.isNaN(id) ? null : { at: "CrossesDriveLine", id };
  }
  if (head === "crosses-perimeter") {
    const loop = parseLoop(tail);
    return loop ? { at: "CrossesPerimeter", loop } : null;
  }
  return null;
}

function parseLoop(v: string | undefined): PerimeterLoop | null {
  if (!v) return null;
  if (v === "outer") return { kind: "Outer" };
  if (v.startsWith("hole:")) {
    const index = Number(v.slice(5));
    return Number.isNaN(index) ? null : { kind: "Hole", index };
  }
  return null;
}

function parseTraffic(v: string): AisleDirection | null {
  switch (v) {
    case "one-way":
      return "OneWay";
    case "one-way-reverse":
      return "OneWayReverse";
    case "two-way-reverse":
      return "TwoWayReverse";
    default:
      return null;
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
