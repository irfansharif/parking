import {
  Vec2,
  Polygon,
  DriveAisleGraph,
  ParkingParams,
  ParkingLayout,
  GenerateInput,
} from "./types";

export interface Camera {
  offsetX: number;
  offsetY: number;
  zoom: number;
}

export interface VertexRef {
  type: "boundary-outer" | "boundary-hole" | "aisle";
  index: number;
  holeIndex?: number;
}

export type EditMode =
  | "select"
  | "add-aisle-vertex"
  | "add-aisle-edge"
  | "add-hole";

export interface LayerVisibility {
  stalls: boolean;
  aisles: boolean;
  vertices: boolean;
  islands: boolean;
  spines: boolean;
  faces: boolean;
}

export interface AppState {
  boundary: Polygon;
  aisleGraph: DriveAisleGraph | null;
  params: ParkingParams;
  layout: ParkingLayout | null;
  selectedVertex: VertexRef | null;
  hoveredVertex: VertexRef | null;
  isDragging: boolean;
  dragAnchor: Vec2 | null;
  camera: Camera;
  editMode: EditMode;
  // For add-hole mode: vertices being placed
  pendingHole: Vec2[];
  layers: LayerVisibility;
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
          { x: 300, y: 0 },
          { x: 300, y: 200 },
          { x: 0, y: 200 },
        ],
        holes: [],
      },
      aisleGraph: null,
      params: {
        stall_width: 9,
        stall_depth: 18,
        aisle_width: 24,
        stall_angle_deg: 90,
        aisle_angle_deg: 90,
        aisle_offset: 0,
        max_run: 0,
        island_width: 4,
        ada_count: 0,
        site_offset: 0,
      },
      layout: null,
      selectedVertex: null,
      hoveredVertex: null,
      isDragging: false,
      dragAnchor: null,
      camera: { offsetX: 50, offsetY: 50, zoom: 3.0 },
      editMode: "select",
      pendingHole: [],
      layers: {
        stalls: true,
        aisles: true,
        vertices: true,
        islands: true,
        spines: true,
        faces: true,
      },
    };
  }

  generate(): void {
    const input: GenerateInput = {
      boundary: this.state.boundary,
      aisle_graph: this.state.aisleGraph,
      params: this.state.params,
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
}
