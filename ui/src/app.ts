import {
  Vec2,
  Polygon,
  DriveLine,
  DriveAisleGraph,
  ParkingParams,
  ParkingLayout,
  GenerateInput,
  DebugToggles,
} from "./types";
import { SnapGuide, SnapState, emptySnapState } from "./snap";

export interface Camera {
  offsetX: number;
  offsetY: number;
  zoom: number;
}

export interface VertexRef {
  type: "boundary-outer" | "boundary-hole" | "aisle" | "drive-line";
  index: number;
  holeIndex?: number;
  endpoint?: "start" | "end";
}

export interface EdgeRef {
  index: number; // index into the graph's edges array
}

export type EditMode =
  | "select"
  | "add-aisle-vertex"
  | "add-aisle-edge"
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
  dragAnchor: Vec2 | null;
  camera: Camera;
  editMode: EditMode;
  // For add-hole mode: vertices being placed
  pendingHole: Vec2[];
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
          { x: 0, y: 0 },
          { x: 750, y: 0 },
          { x: 750, y: 500 },
          { x: 0, y: 500 },
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
        aisle_width: 24,
        stall_angle_deg: 45,
        aisle_angle_deg: 90,
        aisle_offset: 0,
        site_offset: 0,
      },
      debug: {
        miter_fills: true,
        spike_removal: false,
        hole_filtering: true,
        face_extraction: true,
        edge_classification: true,
        spine_clipping: true,
        spine_dedup: true,
        spine_merging: true,
        short_spine_filter: false,
        stall_face_clipping: true,
        boundary_clipping: true,
        skeleton_debug: false,
      },
      layout: null,
      selectedVertex: null,
      hoveredVertex: null,
      selectedEdge: null,
      isDragging: false,
      dragAnchor: null,
      camera: { offsetX: 30, offsetY: 60, zoom: 1.3 },
      editMode: "select",
      pendingHole: [],
      driveLines: [
        { start: { x: -50, y: 250 }, end: { x: 800, y: 250 } },
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
        spines: true,
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
    this.generate();
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
    // Drive-line vertices first so they win hit-tests over overlapping
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
    } else if (ref.type === "drive-line" && ref.endpoint) {
      this.state.driveLines[ref.index][ref.endpoint] = pos;
      this.state.aisleGraph = null;
    } else if (ref.type === "aisle") {
      // Capture the anchor (pinned endpoint) on the first drag move.
      if (!this.state.dragAnchor) {
        const graph = this.getEffectiveAisleGraph();
        if (!graph) return;
        let bestLen = 0;
        let bestOther: Vec2 | null = null;
        for (const e of graph.edges) {
          const otherIdx = e.start === ref.index ? e.end :
                           e.end === ref.index ? e.start : -1;
          if (otherIdx < 0) continue;
          const other = graph.vertices[otherIdx];
          const dx = pos.x - other.x;
          const dy = pos.y - other.y;
          const len = Math.sqrt(dx * dx + dy * dy);
          if (len > bestLen) {
            bestLen = len;
            bestOther = other;
          }
        }
        this.state.dragAnchor = bestOther;
      }

      const anchor = this.state.dragAnchor;
      if (anchor) {
        // Compute angle from anchor to drag position.
        const dx = pos.x - anchor.x;
        const dy = pos.y - anchor.y;
        const len = Math.sqrt(dx * dx + dy * dy);
        if (len > 1) {
          let angleDeg = Math.atan2(dy, dx) * (180 / Math.PI);
          // Normalize to [0, 180) — aisle lines at θ and θ+180° are identical.
          angleDeg = ((angleDeg % 180) + 180) % 180;
          this.state.params.aisle_angle_deg = Math.round(angleDeg);
        }

        // Compute perpendicular offset so the aisle grid is anchored to the
        // pinned vertex. The offset is the anchor's projection onto the
        // perpendicular direction for the current angle.
        const angleRad = this.state.params.aisle_angle_deg * (Math.PI / 180);
        const perpX = -Math.sin(angleRad);
        const perpY = Math.cos(angleRad);
        this.state.params.aisle_offset = anchor.x * perpX + anchor.y * perpY;

        // Create a minimal manual graph with just the dragged edge so that
        // other interior aisles regenerate at the new angle via merge_with_auto.
        const hw = this.state.params.aisle_width / 2;
        this.state.aisleGraph = {
          vertices: [anchor, pos],
          edges: [
            { start: 0, end: 1, width: hw },
            { start: 1, end: 0, width: hw },
          ],
        };
      }
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

  addAisleVertex(pos: Vec2): number {
    if (!this.state.aisleGraph) {
      this.state.aisleGraph = { vertices: [], edges: [] };
    }
    const idx = this.state.aisleGraph.vertices.length;
    this.state.aisleGraph.vertices.push(pos);
    this.generate();
    return idx;
  }

  addAisleEdge(startIdx: number, endIdx: number): void {
    if (!this.state.aisleGraph) return;
    this.state.aisleGraph.edges.push({
      start: startIdx,
      end: endIdx,
      width: this.state.params.aisle_width / 2,
    });
    this.generate();
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

  deleteDriveLine(index: number): void {
    this.state.driveLines.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  cycleEdgeDirection(edgeIndex: number): void {
    this.materializeAisleGraph();
    const g = this.state.aisleGraph;
    if (!g) return;

    // Find the edge and its reverse (edges are stored bidirectionally).
    const edge = g.edges[edgeIndex];
    if (!edge) return;
    const reverseIdx = g.edges.findIndex(
      (e, i) => i !== edgeIndex && e.start === edge.end && e.end === edge.start
    );

    const dir = edge.direction ?? "TwoWay";
    if (dir === "TwoWay") {
      // TwoWay → OneWay (forward = start→end direction)
      edge.direction = "OneWay";
      if (reverseIdx >= 0) g.edges[reverseIdx].direction = "OneWay";
    } else {
      // OneWay → flip direction by swapping start/end on the forward edge.
      // Since we have bidirectional pairs, "flipping" means the forward edge
      // was (A→B, OneWay), reverse was (B→A, OneWay). The canonical travel
      // direction is taken from whichever edge is first seen in dedup.
      // Simplest: swap start/end on this edge, swap on reverse too, keeping
      // OneWay. But if we already flipped once, go back to TwoWay.
      if (edge._flipped) {
        edge.direction = "TwoWay";
        delete edge._flipped;
        if (reverseIdx >= 0) {
          g.edges[reverseIdx].direction = "TwoWay";
          delete g.edges[reverseIdx]._flipped;
        }
      } else {
        // Swap start/end to reverse the one-way direction.
        const tmp = edge.start;
        edge.start = edge.end;
        edge.end = tmp;
        edge._flipped = true;
        if (reverseIdx >= 0) {
          const tmp2 = g.edges[reverseIdx].start;
          g.edges[reverseIdx].start = g.edges[reverseIdx].end;
          g.edges[reverseIdx].end = tmp2;
          g.edges[reverseIdx]._flipped = true;
        }
      }
    }
    this.generate();
  }

  /// Cycle drive line direction: TwoWay → OneWay → OneWay(reversed) → TwoWay.
  /// "Reversed" swaps start/end so the arrow flips.
  cycleDriveLineDirection(index: number): void {
    const dl = this.state.driveLines[index];
    if (!dl) return;
    if (!dl.direction || dl.direction === "TwoWay") {
      // TwoWay → OneWay (start→end direction)
      dl.direction = "OneWay";
    } else if (!dl._reversed) {
      // OneWay → swap endpoints (reverse the one-way direction)
      const tmp = dl.start;
      dl.start = dl.end;
      dl.end = tmp;
      dl._reversed = true;
    } else {
      // OneWay(reversed) → TwoWay
      dl.direction = "TwoWay";
      delete dl._reversed;
    }
    this.generate();
  }
}
