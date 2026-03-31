import {
  Vec2,
  Polygon,
  DriveLine,
  DriveAisleGraph,
  ParkingParams,
  ParkingLayout,
  GenerateInput,
  DebugToggles,
  Annotation,
} from "./types";
import { SnapGuide, SnapState, emptySnapState } from "./snap";
import { findCollinearChain } from "./interaction";

export interface Camera {
  offsetX: number;
  offsetY: number;
  zoom: number;
}

export interface VertexRef {
  type: "boundary-outer" | "boundary-hole" | "aisle" | "drive-line" | "annotation" | "aisle-vector";
  index: number;
  holeIndex?: number;
  endpoint?: "start" | "end";
}

export interface EdgeRef {
  index: number; // seed edge index
  chain: number[]; // all deduplicated edge indices in the collinear chain
  mode: "chain" | "segment"; // chain = full line, segment = single edge
}

export type EditMode =
  | "select"
  | "add-hole"
  | "add-drive-line";

export interface LayerVisibility {
  stalls: boolean;
  aisles: boolean;
  vertices: boolean;
  driveLines: boolean;
  spines: boolean;
  faces: boolean;
  faceColors: boolean;
  miterFills: boolean;
  skeletonDebug: boolean;
  islands: boolean;
}

export interface AppState {
  boundary: Polygon;
  aisleGraph: DriveAisleGraph | null;
  params: ParkingParams;
  debug: DebugToggles;
  layout: ParkingLayout | null;
  selectedVertex: VertexRef | null;
  hoveredVertex: VertexRef | null;
  selectedEdge: EdgeRef | null;
  isDragging: boolean;
  camera: Camera;
  editMode: EditMode;
  // For add-hole mode: vertices being placed
  pendingHole: Vec2[];
  // Spatial annotations (survive graph regeneration)
  annotations: Annotation[];
  // Aisle vector control — defines dominant aisle direction/offset
  aisleVector: { start: Vec2; end: Vec2 };
  // For add-drive-line mode
  driveLines: DriveLine[];
  pendingDriveLine: Vec2 | null;
  pendingDriveLinePreview: Vec2 | null;
  layers: LayerVisibility;
  snapGuides: SnapGuide[];
  snapState: SnapState;
}

export type GenerateFn = (input: string) => string;

export class App {
  state: AppState;
  private generateFn: GenerateFn;
  private onUpdate: () => void;

  constructor(generateFn: GenerateFn, onUpdate: () => void) {
    this.generateFn = generateFn;
    this.onUpdate = onUpdate;
    this.state = {
      boundary: {
        outer: [
          { x: -48.47, y: 0 },
          { x: 750, y: 0 },
          { x: 782.80, y: 654.85 },
          { x: 0, y: 654.85 },
        ],
        holes: [
          [
            { x: 275, y: 150 },
            { x: 475, y: 150 },
            { x: 375, y: 350 },
          ],
        ],
      },
      aisleGraph: null,
      params: {
        stall_width: 9,
        stall_depth: 18,
        aisle_width: 12,
        stall_angle_deg: 45,
        aisle_angle_deg: 90,
        aisle_offset: 0,
        site_offset: 0,
        cross_aisle_spacing: 179,
      },
      debug: {
        miter_fills: true,
        spike_removal: false,
        hole_filtering: false,
        face_extraction: true,
        face_simplification: true,
        edge_classification: true,
        spine_clipping: true,
        spine_dedup: true,
        spine_merging: true,
        short_spine_filter: false,
        stall_face_clipping: true,
        boundary_clipping: false,
        conflict_removal: true,
        skeleton_debug: false,
      },
      layout: null,
      selectedVertex: null,
      hoveredVertex: null,
      selectedEdge: null,
      isDragging: false,
      camera: { offsetX: 30, offsetY: 60, zoom: 1.3 },
      editMode: "select",
      pendingHole: [],
      annotations: [],
      aisleVector: aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
      driveLines: [
        { start: { x: 375, y: -13.35 }, end: { x: 139.44, y: 696.35 } },
      ],
      pendingDriveLine: null,
      pendingDriveLinePreview: null,
      snapGuides: [],
      snapState: emptySnapState(),
      layers: {
        stalls: true,
        aisles: false,
        vertices: true,
        driveLines: true,
        spines: false,
        faces: true,
        faceColors: false,
        miterFills: false,
        skeletonDebug: false,
        islands: true,
      },
    };
  }

  generate(): void {
    const input: GenerateInput = {
      boundary: this.state.boundary,
      aisle_graph: this.state.aisleGraph,
      drive_lines: this.state.driveLines,
      annotations: this.state.annotations.filter((a) => a._active !== false),
      params: this.state.params,
      debug: this.state.debug,
    };
    const inputJson = JSON.stringify(input);
    // Stash on window so you can grab it from the console:
    //   copy(window.__parkingInput)
    // Then save to engine/testdata/<name>.json and run:
    //   cargo test -- datadriven
    (window as any).__parkingInput = inputJson;
    try {
      const resultJson = this.generateFn(inputJson);
      this.state.layout = JSON.parse(resultJson);
    } catch (e) {
      console.error("Generate failed:", e);
    }
    this.onUpdate();
  }

  setParam(key: keyof ParkingParams, value: number): void {
    (this.state.params as any)[key] = value;
    this.state.aisleGraph = null;
    if (key === "aisle_angle_deg" || key === "aisle_offset") {
      this.syncAisleVector();
    }
    this.generate();
  }

  private syncAisleVector(): void {
    const vec = this.state.aisleVector;
    const center = {
      x: (vec.start.x + vec.end.x) / 2,
      y: (vec.start.y + vec.end.y) / 2,
    };
    this.state.aisleVector = aisleVectorFromAngle(
      this.state.params.aisle_angle_deg,
      this.state.params.aisle_offset,
      center,
    );
  }

  // Returns the effective aisle graph: manual if set, otherwise resolved from
  // the last generate() call.
  getEffectiveAisleGraph(): DriveAisleGraph | null {
    return this.state.aisleGraph ?? this.state.layout?.resolved_graph ?? null;
  }

  // Promote the auto-generated (resolved) graph into the editable
  // aisleGraph so it can be dragged/modified. Called on first interaction
  // with a resolved vertex.
  materializeAisleGraph(): void {
    if (this.state.aisleGraph) return; // already manual
    const resolved = this.state.layout?.resolved_graph;
    if (!resolved) return;
    // Deep copy so edits don't mutate the layout snapshot.
    this.state.aisleGraph = {
      vertices: resolved.vertices.map((v) => ({ ...v })),
      edges: resolved.edges.map((e) => ({ ...e })),
    };
  }

  getAllVertices(): { ref: VertexRef; pos: Vec2 }[] {
    const result: { ref: VertexRef; pos: Vec2 }[] = [];
    // Aisle vector endpoints — highest hit-test priority.
    result.push({
      ref: { type: "aisle-vector", index: 0, endpoint: "start" },
      pos: this.state.aisleVector.start,
    });
    result.push({
      ref: { type: "aisle-vector", index: 0, endpoint: "end" },
      pos: this.state.aisleVector.end,
    });
    // Annotation anchors next.
    this.state.annotations.forEach((ann, i) => {
      const pos = ann.kind === "DeleteVertex" ? ann.point : ann.midpoint;
      result.push({ ref: { type: "annotation", index: i }, pos });
    });
    // Drive-line vertices next so they win hit-tests over overlapping
    // resolved-graph aisle vertices at the same position.
    this.state.driveLines.forEach((dl, i) => {
      result.push({
        ref: { type: "drive-line", index: i, endpoint: "start" },
        pos: dl.start,
      });
      result.push({
        ref: { type: "drive-line", index: i, endpoint: "end" },
        pos: dl.end,
      });
    });
    this.state.boundary.outer.forEach((v, i) => {
      result.push({ ref: { type: "boundary-outer", index: i }, pos: v });
    });
    this.state.boundary.holes.forEach((hole, hi) => {
      hole.forEach((v, vi) => {
        result.push({
          ref: { type: "boundary-hole", index: vi, holeIndex: hi },
          pos: v,
        });
      });
    });
    const graph = this.getEffectiveAisleGraph();
    if (graph) {
      graph.vertices.forEach((v, i) => {
        result.push({ ref: { type: "aisle", index: i }, pos: v });
      });
    }
    return result;
  }

  moveVertex(ref: VertexRef, pos: Vec2): void {
    if (ref.type === "boundary-outer") {
      this.state.boundary.outer[ref.index] = pos;
      this.state.aisleGraph = null;
    } else if (ref.type === "boundary-hole" && ref.holeIndex !== undefined) {
      this.state.boundary.holes[ref.holeIndex][ref.index] = pos;
      this.state.aisleGraph = null;
    } else if (ref.type === "annotation") {
      const ann = this.state.annotations[ref.index];
      if (!ann) return;
      const isDelete = ann.kind === "DeleteVertex" || ann.kind === "DeleteEdge";
      const graph = this.getEffectiveAisleGraph();

      if (isDelete && graph) {
        // Find nearest vertex and nearest edge to decide which kind to be.
        let bestVtxDist = Infinity;
        for (const v of graph.vertices) {
          const d = Math.sqrt((v.x - pos.x) ** 2 + (v.y - pos.y) ** 2);
          if (d < bestVtxDist) bestVtxDist = d;
        }
        let bestEdgeDist = Infinity;
        let bestEdgeDir: Vec2 | null = null;
        const seen = new Set<string>();
        for (const edge of graph.edges) {
          const key = Math.min(edge.start, edge.end) + "," + Math.max(edge.start, edge.end);
          if (seen.has(key)) continue;
          seen.add(key);
          const s = graph.vertices[edge.start];
          const e = graph.vertices[edge.end];
          const d = pointToSegmentDist(pos, s, e);
          if (d < bestEdgeDist) {
            bestEdgeDist = d;
            const dx = e.x - s.x, dy = e.y - s.y;
            const len = Math.sqrt(dx * dx + dy * dy);
            if (len > 1e-9) bestEdgeDir = { x: dx / len, y: dy / len };
          }
        }
        // Convert to whichever is closer. Vertex needs to be distinctly
        // closer to win (within 5ft), otherwise default to edge.
        if (bestVtxDist < 5 && bestVtxDist < bestEdgeDist) {
          this.state.annotations[ref.index] = {
            kind: "DeleteVertex",
            point: pos,
            _active: ann._active,
          };
        } else if (bestEdgeDir) {
          this.state.annotations[ref.index] = {
            kind: "DeleteEdge",
            midpoint: pos,
            edge_dir: bestEdgeDir,
            chain: true,
            _active: ann._active,
          };
        }
      } else if (ann.kind === "OneWay") {
        ann.midpoint = pos;
        if (graph) {
          let bestDist = Infinity;
          let bestDir: Vec2 | null = null;
          const seen = new Set<string>();
          for (const edge of graph.edges) {
            const key = Math.min(edge.start, edge.end) + "," + Math.max(edge.start, edge.end);
            if (seen.has(key)) continue;
            seen.add(key);
            const s = graph.vertices[edge.start];
            const e = graph.vertices[edge.end];
            const d = pointToSegmentDist(pos, s, e);
            if (d < bestDist) {
              bestDist = d;
              const dx = e.x - s.x, dy = e.y - s.y;
              const len = Math.sqrt(dx * dx + dy * dy);
              if (len > 1e-9) bestDir = { x: dx / len, y: dy / len };
            }
          }
          if (bestDir && bestDist < 10) {
            ann.travel_dir = bestDir;
            ann._origDir = bestDir;
          }
        }
      }
    } else if (ref.type === "drive-line" && ref.endpoint) {
      this.state.driveLines[ref.index][ref.endpoint] = pos;
      this.state.aisleGraph = null;
    } else if (ref.type === "aisle-vector") {
      const vec = this.state.aisleVector;
      if (ref.endpoint === "start") {
        vec.start = pos;
      } else {
        vec.end = pos;
      }
      const anchor = ref.endpoint === "start" ? vec.end : vec.start;
      const dx = vec.end.x - vec.start.x;
      const dy = vec.end.y - vec.start.y;
      const len = Math.sqrt(dx * dx + dy * dy);
      if (len > 1) {
        let angleDeg = Math.atan2(dy, dx) * (180 / Math.PI);
        angleDeg = ((angleDeg % 180) + 180) % 180;
        this.state.params.aisle_angle_deg = Math.round(angleDeg);
      }
      const angleRad = this.state.params.aisle_angle_deg * (Math.PI / 180);
      const perpX = -Math.sin(angleRad);
      const perpY = Math.cos(angleRad);
      this.state.params.aisle_offset = anchor.x * perpX + anchor.y * perpY;
      this.state.aisleGraph = null;
    }
    this.generate();
  }

  addHole(vertices: Vec2[]): void {
    this.state.boundary.holes.push(vertices);
    this.state.aisleGraph = null;
    this.generate();
  }

  insertBoundaryVertex(index: number, pos: Vec2): void {
    this.state.boundary.outer.splice(index + 1, 0, pos);
    this.state.aisleGraph = null;
    this.generate();
  }

  deleteBoundaryVertex(index: number): boolean {
    if (this.state.boundary.outer.length <= 3) return false;
    this.state.boundary.outer.splice(index, 1);
    this.state.aisleGraph = null;
    this.generate();
    return true;
  }

  deleteAisleVertex(index: number): void {
    this.materializeAisleGraph();
    if (!this.state.aisleGraph) return;
    const g = this.state.aisleGraph;
    g.vertices.splice(index, 1);
    // Remove edges referencing this vertex and remap indices.
    g.edges = g.edges
      .filter((e) => e.start !== index && e.end !== index)
      .map((e) => ({
        ...e,
        start: e.start > index ? e.start - 1 : e.start,
        end: e.end > index ? e.end - 1 : e.end,
      }));
    if (g.vertices.length === 0) {
      this.state.aisleGraph = null;
    }
    this.state.selectedVertex = null;
    this.generate();
  }

  deleteHoleVertex(holeIndex: number, vertexIndex: number): void {
    const hole = this.state.boundary.holes[holeIndex];
    if (hole.length <= 3) {
      // Remove the entire hole.
      this.state.boundary.holes.splice(holeIndex, 1);
    } else {
      hole.splice(vertexIndex, 1);
    }
    this.state.selectedVertex = null;
    this.state.aisleGraph = null;
    this.generate();
  }

  insertHoleVertex(holeIndex: number, afterIndex: number, pos: Vec2): void {
    this.state.boundary.holes[holeIndex].splice(afterIndex + 1, 0, pos);
    this.state.aisleGraph = null;
    this.generate();
  }

  commitPendingHole(): void {
    if (this.state.pendingHole.length >= 3) {
      this.state.boundary.holes.push([...this.state.pendingHole]);
      this.state.aisleGraph = null;
      this.generate();
    }
    this.state.pendingHole = [];
  }

  addDriveLine(start: Vec2, end: Vec2): void {
    this.state.driveLines.push({ start, end });
    this.generate();
  }

  deleteAnnotation(index: number): void {
    this.state.annotations.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  cycleAnnotationDirection(index: number): void {
    const ann = this.state.annotations[index];
    if (!ann || ann.kind !== "OneWay") return;
    const orig = ann._origDir ?? ann.travel_dir;

    if (ann._active === false) {
      ann._active = true;
      ann.travel_dir = { x: orig.x, y: orig.y };
    } else {
      const dot = ann.travel_dir.x * orig.x + ann.travel_dir.y * orig.y;
      if (dot > 0) {
        ann.travel_dir = { x: -orig.x, y: -orig.y };
      } else {
        ann._active = false;
      }
    }
    this.generate();
    // Sync: also select the matching edge so it highlights.
    this.syncEdgeSelectionFromAnnotation(ann);
  }

  /// Find the resolved graph edge nearest to an annotation and select it.
  private syncEdgeSelectionFromAnnotation(ann: Annotation): void {
    if (ann.kind === "DeleteVertex") return;
    const pt = ann.midpoint;
    const graph = this.state.layout?.resolved_graph;
    if (!graph) return;
    const seen = new Set<string>();
    let bestIdx = -1;
    let bestDist = Infinity;
    for (let i = 0; i < graph.edges.length; i++) {
      const edge = graph.edges[i];
      const key = Math.min(edge.start, edge.end) + "," + Math.max(edge.start, edge.end);
      if (seen.has(key)) continue;
      seen.add(key);
      const s = graph.vertices[edge.start];
      const e = graph.vertices[edge.end];
      const dist = pointToSegmentDist(pt, s, e);
      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = i;
      }
    }
    if (bestIdx >= 0 && bestDist < 5) {
      this.state.selectedEdge = { index: bestIdx, chain: findCollinearChain(graph, bestIdx), mode: "chain" };
    }
  }

  /// Remove all tombstone (inactive) annotations. Called when selection
  /// changes away from an annotation.
  cleanupTombstones(): void {
    const before = this.state.annotations.length;
    this.state.annotations = this.state.annotations.filter(
      (a) => a._active !== false
    );
    if (this.state.annotations.length !== before) {
      this.generate();
    }
  }

  deleteAisleVertexByAnnotation(vertexIndex: number): void {
    const graph = this.getEffectiveAisleGraph();
    if (!graph) return;
    const v = graph.vertices[vertexIndex];
    if (!v) return;
    this.state.annotations.push({
      kind: "DeleteVertex",
      point: { x: v.x, y: v.y },
    });
    this.state.selectedVertex = null;
    this.state.aisleGraph = null;
    this.generate();
  }

  deleteSelectedEdge(): void {
    const sel = this.state.selectedEdge;
    if (!sel) return;
    const graph = this.getEffectiveAisleGraph();
    if (!graph) return;
    const edge = graph.edges[sel.index];
    if (!edge) return;
    const s = graph.vertices[edge.start];
    const e = graph.vertices[edge.end];
    const mid = { x: (s.x + e.x) / 2, y: (s.y + e.y) / 2 };
    const dx = e.x - s.x, dy = e.y - s.y;
    const len = Math.sqrt(dx * dx + dy * dy);
    if (len < 1e-9) return;
    this.state.annotations.push({
      kind: "DeleteEdge",
      midpoint: mid,
      edge_dir: { x: dx / len, y: dy / len },
      chain: sel.mode === "chain",
    });
    this.state.selectedEdge = null;
    this.state.aisleGraph = null;
    this.generate();
  }

  deleteDriveLine(index: number): void {
    this.state.driveLines.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  /// Cycle an aisle edge's direction via annotations.
  /// TwoWay → OneWay(forward) → OneWay(reversed) → TwoWay.
  /// Creates, updates, or removes a OneWay annotation at the edge midpoint.
  cycleEdgeDirection(edgeIndex: number): void {
    const graph = this.getEffectiveAisleGraph();
    if (!graph) return;
    const edge = graph.edges[edgeIndex];
    if (!edge) return;

    const s = graph.vertices[edge.start];
    const e = graph.vertices[edge.end];
    const mid = { x: (s.x + e.x) / 2, y: (s.y + e.y) / 2 };
    const len = Math.sqrt((e.x - s.x) ** 2 + (e.y - s.y) ** 2);
    if (len < 1e-9) return;
    const edgeDir = { x: (e.x - s.x) / len, y: (e.y - s.y) / len };

    // Find existing annotation for this edge (by midpoint proximity).
    const tolerance = 5.0;
    const existing = this.state.annotations.findIndex(
      (a) => a.kind === "OneWay" &&
        Math.sqrt((a.midpoint.x - mid.x) ** 2 + (a.midpoint.y - mid.y) ** 2) < tolerance
    );

    const isChainMode = this.state.selectedEdge?.mode !== "segment";
    let annIdx: number;
    if (existing < 0) {
      // No annotation → create OneWay in start→end direction.
      annIdx = this.state.annotations.length;
      this.state.annotations.push({
        kind: "OneWay",
        midpoint: mid,
        travel_dir: edgeDir,
        chain: isChainMode,
        _origDir: edgeDir,
        _active: true,
      });
    } else {
      annIdx = existing;
      const ann = this.state.annotations[existing];
      if (ann.kind !== "OneWay") return;
      if (ann._active === false) {
        const orig = ann._origDir ?? ann.travel_dir;
        ann._active = true;
        ann.travel_dir = { x: orig.x, y: orig.y };
      } else {
        const orig = ann._origDir ?? ann.travel_dir;
        const dot = ann.travel_dir.x * orig.x + ann.travel_dir.y * orig.y;
        if (dot > 0) {
          ann.travel_dir = { x: -orig.x, y: -orig.y };
        } else {
          ann._active = false;
        }
      }
    }
    // Keep edge and annotation selections in sync.
    this.state.selectedVertex = { type: "annotation", index: annIdx };
    this.generate();
  }
}

function aisleVectorFromAngle(angleDeg: number, _offset: number, center: Vec2): { start: Vec2; end: Vec2 } {
  const angleRad = angleDeg * (Math.PI / 180);
  const dirX = Math.cos(angleRad);
  const dirY = Math.sin(angleRad);
  const halfLen = 60;
  return {
    start: { x: center.x - dirX * halfLen, y: center.y - dirY * halfLen },
    end: { x: center.x + dirX * halfLen, y: center.y + dirY * halfLen },
  };
}

function pointToSegmentDist(p: Vec2, a: Vec2, b: Vec2): number {
  const dx = b.x - a.x, dy = b.y - a.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return Math.sqrt((p.x - a.x) ** 2 + (p.y - a.y) ** 2);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq));
  const px = a.x + t * dx, py = a.y + t * dy;
  return Math.sqrt((p.x - px) ** 2 + (p.y - py) ** 2);
}
