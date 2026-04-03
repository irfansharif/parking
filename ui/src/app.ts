import {
  Vec2,
  Polygon,
  DriveLine,
  DriveAisleGraph,
  ParkingParams,
  ParkingLayout,
  ParkingLot,
  GenerateInput,
  DebugToggles,
  Annotation,
  computeBoundaryPin,
} from "./types";
import { SnapGuide, SnapState, emptySnapState } from "./snap";
import { findCollinearChain } from "./interaction";

export interface Camera {
  offsetX: number;
  offsetY: number;
  zoom: number;
}

export interface VertexRef {
  type: "boundary-outer" | "boundary-hole" | "aisle" | "drive-line" | "annotation" | "aisle-vector" | "region-vector";
  index: number;
  holeIndex?: number;
  endpoint?: "start" | "end" | "body";
  lotId?: string;
}

export interface EdgeRef {
  index: number; // seed edge index
  chain: number[]; // all deduplicated edge indices in the collinear chain
  mode: "chain" | "segment"; // chain = full line, segment = single edge
}

export type EditMode =
  | "select"
  | "add-hole"
  | "add-drive-line"
  | "add-boundary";

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
  extensionStalls: boolean;
  regions: boolean;
  paintLines: boolean;
}

export interface AppState {
  lots: ParkingLot[];
  activeLotId: string;
  params: ParkingParams;
  debug: DebugToggles;
  selectedVertex: VertexRef | null;
  hoveredVertex: VertexRef | null;
  selectedEdge: EdgeRef | null;
  isDragging: boolean;
  camera: Camera;
  editMode: EditMode;
  // For add-hole mode: vertices being placed
  pendingHole: Vec2[];
  // For add-boundary mode: vertices being placed
  pendingBoundary: Vec2[];
  // For add-drive-line mode
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
  private nextLotId = 1;

  constructor(generateFn: GenerateFn, onUpdate: () => void) {
    this.generateFn = generateFn;
    this.onUpdate = onUpdate;

    const defaultLot: ParkingLot = {
      id: "lot-0",
      boundary: {
        outer: [
          { x: 76.30, y: 0 },
          { x: 750, y: 0 },
          { x: 782.80, y: 654.85 },
          { x: 168.78, y: 654.85 },
        ],
        holes: [
          [
            { x: 394.53, y: 251.80 },
            { x: 611.14, y: 251.80 },
            { x: 613.91, y: 512.33 },
            { x: 394.53, y: 512.33 },
          ],
        ],
      },
      aisleGraph: null,
      annotations: [],
      aisleVector: aisleVectorFromAngle(90, 0, { x: -80, y: 250 }),
      driveLines: [
        { start: { x: 824.53, y: 531.97 }, end: { x: 224.60, y: 654.85 } },
        { start: { x: 611.14, y: 251.80 }, end: { x: 762.23, y: 244.23 }, holePin: { holeIndex: 0, vertexIndex: 1 } },
        { start: { x: 394.53, y: 251.80 }, end: { x: 250.39, y: 0 }, holePin: { holeIndex: 0, vertexIndex: 0 } },
      ],
      regionOverrides: {},
      layout: null,
    };

    this.state = {
      lots: [defaultLot],
      activeLotId: "lot-0",
      params: {
        stall_width: 9,
        stall_depth: 18,
        aisle_width: 12,
        stall_angle_deg: 45,
        aisle_angle_deg: 90,
        aisle_offset: 0,
        site_offset: 0,
        cross_aisle_max_run: 15,
        use_regions: true,
        island_stall_interval: 8,
      },
      debug: {
        miter_fills: true,
        boundary_only_miters: true,
        spike_removal: true,
        contour_simplification: true,
        hole_filtering: false,
        face_extraction: true,
        edge_provenance: true,
        face_simplification: false,
        edge_classification: true,
        spine_clipping: true,
        spine_dedup: true,
        spine_merging: true,
        short_spine_filter: false,
        spine_extensions: true,
        stall_centering: true,
        stall_face_clipping: true,
        boundary_clipping: false,
        conflict_removal: true,
        short_segment_filter: true,
        skeleton_debug: false,
      },
      selectedVertex: null,
      hoveredVertex: null,
      selectedEdge: null,
      isDragging: false,
      camera: { offsetX: 30, offsetY: 60, zoom: 1.3 },
      editMode: "select",
      pendingHole: [],
      pendingBoundary: [],
      pendingDriveLine: null,
      pendingDriveLinePreview: null,
      snapGuides: [],
      snapState: emptySnapState(),
      layers: {
        stalls: false,
        aisles: false,
        vertices: false,
        driveLines: false,
        spines: false,
        faces: false,
        faceColors: false,
        miterFills: false,
        skeletonDebug: false,
        islands: true,
        extensionStalls: false,
        regions: false,
        paintLines: true,
      },
    };
  }

  // ── Lot helpers ──────────────────────────────────────────────────────

  activeLot(): ParkingLot {
    return this.state.lots.find((l) => l.id === this.state.activeLotId)!;
  }

  lotById(id: string): ParkingLot | undefined {
    return this.state.lots.find((l) => l.id === id);
  }

  /** Resolve the lot a VertexRef belongs to. Falls back to active lot. */
  lotForRef(ref: VertexRef): ParkingLot {
    if (ref.lotId) {
      return this.lotById(ref.lotId) ?? this.activeLot();
    }
    return this.activeLot();
  }

  private newLotId(): string {
    return `lot-${this.nextLotId++}`;
  }

  addLot(boundary: Polygon): ParkingLot {
    const id = this.newLotId();
    // Compute centroid for aisle vector placement.
    const cx = boundary.outer.reduce((s, v) => s + v.x, 0) / boundary.outer.length;
    const cy = boundary.outer.reduce((s, v) => s + v.y, 0) / boundary.outer.length;
    const lot: ParkingLot = {
      id,
      boundary,
      aisleGraph: null,
      driveLines: [],
      annotations: [],
      aisleVector: aisleVectorFromAngle(90, 0, { x: cx, y: cy }),
      regionOverrides: {},
      layout: null,
    };
    this.state.lots.push(lot);
    this.state.activeLotId = id;
    return lot;
  }

  deleteLot(id: string): void {
    this.state.lots = this.state.lots.filter((l) => l.id !== id);
    if (this.state.activeLotId === id) {
      this.state.activeLotId = this.state.lots[0]?.id ?? "";
    }
  }

  generate(): void {
    for (const lot of this.state.lots) {
      this.generateLot(lot);
    }
    this.onUpdate();
  }

  generateLot(lot: ParkingLot): void {
    const input: GenerateInput = {
      boundary: lot.boundary,
      aisle_graph: lot.aisleGraph,
      drive_lines: lot.driveLines,
      annotations: lot.annotations.filter((a) => a._active !== false),
      params: this.state.params,
      debug: this.state.debug,
      regionOverrides: Object.entries(lot.regionOverrides).map(([k, v]) => ({
        region_index: Number(k),
        aisle_angle_deg: v.angle,
        aisle_offset: v.offset,
      })),
    };
    const inputJson = JSON.stringify(input);
    if (lot.id === this.state.activeLotId) {
      (window as any).__parkingInput = inputJson;
    }
    try {
      const resultJson = this.generateFn(inputJson);
      lot.layout = JSON.parse(resultJson);
    } catch (e) {
      console.error(`Generate failed for lot ${lot.id}:`, e);
    }
  }

  setParam(key: keyof ParkingParams, value: number): void {
    (this.state.params as any)[key] = value;
    for (const lot of this.state.lots) {
      lot.aisleGraph = null;
    }
    if (key === "aisle_angle_deg" || key === "aisle_offset") {
      this.syncAisleVector(this.activeLot());
    }
    this.generate();
  }

  private syncAisleVector(lot: ParkingLot): void {
    const vec = lot.aisleVector;
    const center = {
      x: (vec.start.x + vec.end.x) / 2,
      y: (vec.start.y + vec.end.y) / 2,
    };
    lot.aisleVector = aisleVectorFromAngle(
      this.state.params.aisle_angle_deg,
      this.state.params.aisle_offset,
      center,
    );
  }

  // Returns the effective aisle graph for a lot (default: active lot).
  getEffectiveAisleGraph(lot?: ParkingLot): DriveAisleGraph | null {
    const l = lot ?? this.activeLot();
    return l.aisleGraph ?? l.layout?.resolved_graph ?? null;
  }

  // Promote the auto-generated (resolved) graph into the editable
  // aisleGraph so it can be dragged/modified.
  materializeAisleGraph(lot?: ParkingLot): void {
    const l = lot ?? this.activeLot();
    if (l.aisleGraph) return;
    const resolved = l.layout?.resolved_graph;
    if (!resolved) return;
    l.aisleGraph = {
      vertices: resolved.vertices.map((v) => ({ ...v })),
      edges: resolved.edges.map((e) => ({ ...e })),
    };
  }

  getAllVertices(): { ref: VertexRef; pos: Vec2 }[] {
    const result: { ref: VertexRef; pos: Vec2 }[] = [];

    for (const lot of this.state.lots) {
      const lid = lot.id;

      // Region vector endpoints — highest hit-test priority.
      const rd = lot.layout?.region_debug;
      if (rd) {
        const halfLen = 30;
        for (let i = 0; i < rd.regions.length; i++) {
          const region = rd.regions[i];
          const angleRad = region.aisle_angle_deg * (Math.PI / 180);
          const dirX = Math.cos(angleRad);
          const dirY = Math.sin(angleRad);
          const cx = region.center.x;
          const cy = region.center.y;
          result.push({
            ref: { type: "region-vector", index: i, endpoint: "start", lotId: lid },
            pos: { x: cx - dirX * halfLen, y: cy - dirY * halfLen },
          });
          result.push({
            ref: { type: "region-vector", index: i, endpoint: "end", lotId: lid },
            pos: { x: cx + dirX * halfLen, y: cy + dirY * halfLen },
          });
        }
      }
      // Annotation anchors.
      lot.annotations.forEach((ann, i) => {
        const pos = ann.kind === "DeleteVertex" ? ann.point : ann.midpoint;
        result.push({ ref: { type: "annotation", index: i, lotId: lid }, pos });
      });
      // Drive-line vertices.
      lot.driveLines.forEach((dl, i) => {
        result.push({
          ref: { type: "drive-line", index: i, endpoint: "start", lotId: lid },
          pos: dl.start,
        });
        result.push({
          ref: { type: "drive-line", index: i, endpoint: "end", lotId: lid },
          pos: dl.end,
        });
      });
      lot.boundary.outer.forEach((v, i) => {
        result.push({ ref: { type: "boundary-outer", index: i, lotId: lid }, pos: v });
      });
      lot.boundary.holes.forEach((hole, hi) => {
        hole.forEach((v, vi) => {
          result.push({
            ref: { type: "boundary-hole", index: vi, holeIndex: hi, lotId: lid },
            pos: v,
          });
        });
      });
      const graph = this.getEffectiveAisleGraph(lot);
      if (graph) {
        graph.vertices.forEach((v, i) => {
          result.push({ ref: { type: "aisle", index: i, lotId: lid }, pos: v });
        });
      }
    }
    return result;
  }

  moveVertex(ref: VertexRef, pos: Vec2): void {
    const lot = this.lotForRef(ref);
    if (ref.type === "boundary-outer") {
      lot.boundary.outer[ref.index] = pos;
      for (const dl of lot.driveLines) {
        this.resolveBoundaryPin(dl, lot);
      }
      lot.aisleGraph = null;
    } else if (ref.type === "boundary-hole" && ref.holeIndex !== undefined) {
      lot.boundary.holes[ref.holeIndex][ref.index] = pos;
      for (const dl of lot.driveLines) {
        if (dl.holePin?.holeIndex === ref.holeIndex && dl.holePin?.vertexIndex === ref.index) {
          dl.start = pos;
        }
      }
      lot.aisleGraph = null;
    } else if (ref.type === "annotation") {
      const ann = lot.annotations[ref.index];
      if (!ann) return;
      const isDelete = ann.kind === "DeleteVertex" || ann.kind === "DeleteEdge";
      const graph = this.getEffectiveAisleGraph(lot);

      if (isDelete && graph) {
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
        if (bestVtxDist < 5 && bestVtxDist < bestEdgeDist) {
          lot.annotations[ref.index] = {
            kind: "DeleteVertex",
            point: pos,
            _active: ann._active,
          };
        } else if (bestEdgeDir) {
          lot.annotations[ref.index] = {
            kind: "DeleteEdge",
            midpoint: pos,
            edge_dir: bestEdgeDir,
            chain: true,
            _active: ann._active,
          };
        }
      } else if (ann.kind === "OneWay" || ann.kind === "TwoWayOriented") {
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
      const dl = lot.driveLines[ref.index];
      if (dl.holePin && ref.endpoint === "start") {
        this.moveVertex(
          { type: "boundary-hole", index: dl.holePin.vertexIndex, holeIndex: dl.holePin.holeIndex, lotId: lot.id },
          pos,
        );
        return;
      }
      if (dl.holePin && ref.endpoint === "end") {
        const proj = this.nearestBoundaryProjection(pos, lot);
        dl.end = proj.pos;
        dl.boundaryPin = { edgeIndex: proj.edgeIndex, t: proj.t };
      } else if (ref.endpoint === "start" || ref.endpoint === "end") {
        dl[ref.endpoint] = pos;
      }
      lot.aisleGraph = null;
    } else if (ref.type === "region-vector") {
      const rd = lot.layout?.region_debug;
      const region = rd?.regions[ref.index];
      if (!region) return;
      const cx = region.center.x;
      const cy = region.center.y;

      if (ref.endpoint === "start" || ref.endpoint === "end") {
        const dx = pos.x - cx;
        const dy = pos.y - cy;
        const len = Math.sqrt(dx * dx + dy * dy);
        if (len > 1) {
          let angleDeg = Math.atan2(dy, dx) * (180 / Math.PI);
          angleDeg = ((angleDeg % 180) + 180) % 180;
          if (!lot.regionOverrides[ref.index]) {
            lot.regionOverrides[ref.index] = {};
          }
          lot.regionOverrides[ref.index].angle = Math.round(angleDeg);
        }
      } else if (ref.endpoint === "body") {
        const angleRad = region.aisle_angle_deg * (Math.PI / 180);
        const perpX = -Math.sin(angleRad);
        const perpY = Math.cos(angleRad);
        const offset = pos.x * perpX + pos.y * perpY;
        if (!lot.regionOverrides[ref.index]) {
          lot.regionOverrides[ref.index] = {};
        }
        lot.regionOverrides[ref.index].offset = offset;
      }
      lot.aisleGraph = null;
    } else if (ref.type === "aisle-vector") {
      const vec = lot.aisleVector;
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
      lot.aisleGraph = null;
    }
    this.generate();
  }

  addHole(vertices: Vec2[]): void {
    const lot = this.activeLot();
    lot.boundary.holes.push(vertices);
    lot.aisleGraph = null;
    this.generate();
  }

  insertBoundaryVertex(index: number, pos: Vec2, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.boundary.outer.splice(index + 1, 0, pos);
    lot.aisleGraph = null;
    this.generate();
  }

  deleteBoundaryVertex(index: number, lotId?: string): boolean {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    if (lot.boundary.outer.length <= 3) return false;
    lot.boundary.outer.splice(index, 1);
    lot.aisleGraph = null;
    this.generate();
    return true;
  }

  deleteAisleVertex(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    this.materializeAisleGraph(lot);
    if (!lot.aisleGraph) return;
    const g = lot.aisleGraph;
    g.vertices.splice(index, 1);
    g.edges = g.edges
      .filter((e) => e.start !== index && e.end !== index)
      .map((e) => ({
        ...e,
        start: e.start > index ? e.start - 1 : e.start,
        end: e.end > index ? e.end - 1 : e.end,
      }));
    if (g.vertices.length === 0) {
      lot.aisleGraph = null;
    }
    this.state.selectedVertex = null;
    this.generate();
  }

  deleteHoleVertex(holeIndex: number, vertexIndex: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    const hole = lot.boundary.holes[holeIndex];
    if (hole.length <= 3) {
      lot.boundary.holes.splice(holeIndex, 1);
    } else {
      hole.splice(vertexIndex, 1);
    }
    this.state.selectedVertex = null;
    lot.aisleGraph = null;
    this.generate();
  }

  insertHoleVertex(holeIndex: number, afterIndex: number, pos: Vec2, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.boundary.holes[holeIndex].splice(afterIndex + 1, 0, pos);
    lot.aisleGraph = null;
    this.generate();
  }

  commitPendingHole(): void {
    if (this.state.pendingHole.length >= 3) {
      const lot = this.activeLot();
      lot.boundary.holes.push([...this.state.pendingHole]);
      lot.aisleGraph = null;
      this.generate();
    }
    this.state.pendingHole = [];
  }

  commitPendingBoundary(): void {
    if (this.state.pendingBoundary.length >= 3) {
      this.addLot({ outer: [...this.state.pendingBoundary], holes: [] });
      this.generate();
    }
    this.state.pendingBoundary = [];
  }

  addDriveLine(start: Vec2, end: Vec2): void {
    this.activeLot().driveLines.push({ start, end });
    this.generate();
  }

  deleteAnnotation(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.annotations.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  toggleSeparator(holeIndex: number, vertexIndex: number): void {
    const lot = this.activeLot();
    const idx = lot.driveLines.findIndex(
      (dl) => dl.holePin?.holeIndex === holeIndex && dl.holePin?.vertexIndex === vertexIndex,
    );
    if (idx >= 0) {
      lot.driveLines.splice(idx, 1);
    } else {
      const hole = lot.boundary.holes[holeIndex];
      if (!hole || vertexIndex >= hole.length) return;
      const vertex = hole[vertexIndex];
      const proj = this.nearestBoundaryProjection(vertex, lot);
      lot.driveLines.push({
        start: vertex,
        end: proj.pos,
        holePin: { holeIndex, vertexIndex },
        boundaryPin: { edgeIndex: proj.edgeIndex, t: proj.t },
      });
    }
    lot.regionOverrides = {};
    lot.aisleGraph = null;
    this.generate();
  }

  /** Project a point onto the nearest outer boundary edge. */
  private nearestBoundaryProjection(pt: Vec2, lot?: ParkingLot): { pos: Vec2; edgeIndex: number; t: number } {
    return computeBoundaryPin(pt, (lot ?? this.activeLot()).boundary.outer);
  }

  /** Recompute the end position of a drive line from its boundaryPin. */
  private resolveBoundaryPin(dl: DriveLine, lot: ParkingLot): void {
    if (!dl.holePin) return;
    if (!dl.boundaryPin) {
      // Migrate: compute boundaryPin from current end position.
      const proj = this.nearestBoundaryProjection(dl.end, lot);
      dl.boundaryPin = { edgeIndex: proj.edgeIndex, t: proj.t };
    }
    const outer = lot.boundary.outer;
    const { edgeIndex, t } = dl.boundaryPin;
    const a = outer[edgeIndex % outer.length];
    const b = outer[(edgeIndex + 1) % outer.length];
    dl.end = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
  }

  cycleAnnotationDirection(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    const ann = lot.annotations[index];
    if (!ann || (ann.kind !== "OneWay" && ann.kind !== "TwoWayOriented")) return;
    const orig = ann._origDir ?? ann.travel_dir;

    if (ann.kind === "TwoWayOriented") {
      const dot = ann.travel_dir.x * orig.x + ann.travel_dir.y * orig.y;
      if (dot > 0) {
        ann.travel_dir = { x: -orig.x, y: -orig.y };
      } else {
        (ann as any).kind = "OneWay";
        ann.travel_dir = { x: orig.x, y: orig.y };
      }
    } else {
      if (ann._active === false) {
        (ann as any).kind = "TwoWayOriented";
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
    }
    this.generate();
    this.syncEdgeSelectionFromAnnotation(ann, lot);
  }

  private syncEdgeSelectionFromAnnotation(ann: Annotation, lot?: ParkingLot): void {
    if (ann.kind === "DeleteVertex") return;
    const pt = ann.midpoint;
    const l = lot ?? this.activeLot();
    const graph = l.layout?.resolved_graph;
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

  cleanupTombstones(): void {
    let changed = false;
    for (const lot of this.state.lots) {
      const before = lot.annotations.length;
      lot.annotations = lot.annotations.filter((a) => a._active !== false);
      if (lot.annotations.length !== before) changed = true;
    }
    if (changed) this.generate();
  }

  deleteAisleVertexByAnnotation(vertexIndex: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    const graph = this.getEffectiveAisleGraph(lot);
    if (!graph) return;
    const v = graph.vertices[vertexIndex];
    if (!v) return;
    lot.annotations.push({
      kind: "DeleteVertex",
      point: { x: v.x, y: v.y },
    });
    this.state.selectedVertex = null;
    lot.aisleGraph = null;
    this.generate();
  }

  deleteSelectedEdge(): void {
    const sel = this.state.selectedEdge;
    if (!sel) return;
    const lot = this.activeLot();
    const graph = this.getEffectiveAisleGraph(lot);
    if (!graph) return;
    const edge = graph.edges[sel.index];
    if (!edge) return;
    const s = graph.vertices[edge.start];
    const e = graph.vertices[edge.end];
    const mid = { x: (s.x + e.x) / 2, y: (s.y + e.y) / 2 };
    const dx = e.x - s.x, dy = e.y - s.y;
    const len = Math.sqrt(dx * dx + dy * dy);
    if (len < 1e-9) return;
    lot.annotations.push({
      kind: "DeleteEdge",
      midpoint: mid,
      edge_dir: { x: dx / len, y: dy / len },
      chain: sel.mode === "chain",
    });
    this.state.selectedEdge = null;
    lot.aisleGraph = null;
    this.generate();
  }

  deleteDriveLine(index: number, lotId?: string): void {
    const lot = lotId ? this.lotById(lotId)! : this.activeLot();
    lot.driveLines.splice(index, 1);
    this.state.selectedVertex = null;
    this.generate();
  }

  cycleEdgeDirection(edgeIndex: number): void {
    const lot = this.activeLot();
    const graph = this.getEffectiveAisleGraph(lot);
    if (!graph) return;
    const edge = graph.edges[edgeIndex];
    if (!edge) return;

    const s = graph.vertices[edge.start];
    const e = graph.vertices[edge.end];
    const mid = { x: (s.x + e.x) / 2, y: (s.y + e.y) / 2 };
    const len = Math.sqrt((e.x - s.x) ** 2 + (e.y - s.y) ** 2);
    if (len < 1e-9) return;
    const edgeDir = { x: (e.x - s.x) / len, y: (e.y - s.y) / len };

    const tolerance = 5.0;
    const existing = lot.annotations.findIndex(
      (a) => (a.kind === "OneWay" || a.kind === "TwoWayOriented") &&
        Math.sqrt((a.midpoint.x - mid.x) ** 2 + (a.midpoint.y - mid.y) ** 2) < tolerance
    );

    const isChainMode = this.state.selectedEdge?.mode !== "segment";
    let annIdx: number;
    if (existing < 0) {
      annIdx = lot.annotations.length;
      lot.annotations.push({
        kind: "TwoWayOriented",
        midpoint: mid,
        travel_dir: edgeDir,
        chain: isChainMode,
        _origDir: edgeDir,
        _active: true,
      });
    } else {
      annIdx = existing;
      this.cycleAnnotationDirection(annIdx, lot.id);
      this.state.selectedVertex = { type: "annotation", index: annIdx, lotId: lot.id };
      return;
    }
    this.state.selectedVertex = { type: "annotation", index: annIdx, lotId: lot.id };
    this.generate();
  }
}

function aisleVectorFromAngle(angleDeg: number, _offset: number, center: Vec2): { start: Vec2; end: Vec2 } {
  const angleRad = angleDeg * (Math.PI / 180);
  const dirX = Math.cos(angleRad);
  const dirY = Math.sin(angleRad);
  const halfLen = 30;
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
