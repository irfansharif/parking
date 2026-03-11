import { AppState, Camera, VertexRef } from "./app";
import { Vec2, StallQuad, Island, DriveAisleGraph, SpineLine, Face } from "./types";

export class Renderer {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d")!;
  }

  resize(): void {
    const container = this.canvas.parentElement!;
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = container.clientWidth * dpr;
    this.canvas.height = container.clientHeight * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  render(state: AppState): void {
    const { ctx } = this;
    const w = this.canvas.width / (window.devicePixelRatio || 1);
    const h = this.canvas.height / (window.devicePixelRatio || 1);
    const cam = state.camera;

    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.translate(cam.offsetX, cam.offsetY);
    ctx.scale(cam.zoom, cam.zoom);

    // 1. Grid
    this.drawGrid(cam, w, h);

    // 2. Boundary polygon
    this.drawBoundary(state);

    if (state.layout) {
      // 2b. Positive-space faces (debug overlay)
      if (state.layers.faces && state.layout.faces) {
        for (const face of state.layout.faces) {
          this.drawFace(face);
        }
      }

      // 2c. Miter fills (debug overlay)
      if (state.layers.miterFills && state.layout.miter_fills) {
        for (const fill of state.layout.miter_fills) {
          this.drawPolygon(
            fill,
            "rgba(255, 100, 100, 0.3)",
            "rgba(255, 80, 80, 0.7)",
            0.5,
          );
        }
      }

      // 3. Aisle corridors
      if (state.layers.aisles) {
        for (const poly of state.layout.aisle_polygons) {
          this.drawPolygon(
            poly,
            "rgba(80, 80, 100, 0.6)",
            "rgba(60, 60, 80, 0.8)",
            0.5,
          );
        }
      }

      // 3a. Spines
      if (state.layers.spines && state.layout.spines) {
        for (const spine of state.layout.spines) {
          this.drawSpine(spine);
        }
      }

      // 4. Stalls
      if (state.layers.stalls) {
        for (const stall of state.layout.stalls) {
          this.drawStall(stall);
        }
      }

      // 5. Islands
      if (state.layers.islands) {
        for (const island of state.layout.islands) {
          this.drawIsland(island);
        }
      }
    }

    // 6. Vertex network overlay
    if (state.layers.vertices) {
      this.drawVertexNetwork(state);
    }

    // 7. Pending hole preview
    if (state.pendingHole && state.pendingHole.length > 0) {
      this.drawPendingHole(state.pendingHole);
    }

    // 8. Scale bar
    this.drawScaleBar(cam);

    ctx.restore();
  }

  private drawGrid(cam: Camera, w: number, h: number): void {
    const { ctx } = this;
    const worldLeft = -cam.offsetX / cam.zoom;
    const worldTop = -cam.offsetY / cam.zoom;
    const worldRight = (w - cam.offsetX) / cam.zoom;
    const worldBottom = (h - cam.offsetY) / cam.zoom;

    // 10ft grid
    ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
    ctx.lineWidth = 0.3;
    const step10 = 10;
    const startX = Math.floor(worldLeft / step10) * step10;
    const startY = Math.floor(worldTop / step10) * step10;
    ctx.beginPath();
    for (let x = startX; x <= worldRight; x += step10) {
      ctx.moveTo(x, worldTop);
      ctx.lineTo(x, worldBottom);
    }
    for (let y = startY; y <= worldBottom; y += step10) {
      ctx.moveTo(worldLeft, y);
      ctx.lineTo(worldRight, y);
    }
    ctx.stroke();

    // 50ft grid
    ctx.strokeStyle = "rgba(255, 255, 255, 0.12)";
    ctx.lineWidth = 0.5;
    const step50 = 50;
    const startX50 = Math.floor(worldLeft / step50) * step50;
    const startY50 = Math.floor(worldTop / step50) * step50;
    ctx.beginPath();
    for (let x = startX50; x <= worldRight; x += step50) {
      ctx.moveTo(x, worldTop);
      ctx.lineTo(x, worldBottom);
    }
    for (let y = startY50; y <= worldBottom; y += step50) {
      ctx.moveTo(worldLeft, y);
      ctx.lineTo(worldRight, y);
    }
    ctx.stroke();
  }

  private drawBoundary(state: AppState): void {
    const { ctx } = this;
    const { boundary } = state;

    // Outer boundary fill
    ctx.beginPath();
    this.tracePath(boundary.outer);
    ctx.fillStyle = "rgba(40, 40, 60, 0.8)";
    ctx.fill();

    // Outer boundary stroke
    ctx.beginPath();
    this.tracePath(boundary.outer);
    ctx.strokeStyle = "rgba(233, 69, 96, 0.8)";
    ctx.lineWidth = 1;
    ctx.stroke();

    // Holes (buildings)
    for (const hole of boundary.holes) {
      // Solid fill for hole
      ctx.beginPath();
      this.tracePath(hole);
      ctx.fillStyle = "rgba(30, 30, 50, 0.95)";
      ctx.fill();

      // Crosshatch pattern
      ctx.save();
      ctx.beginPath();
      this.tracePath(hole);
      ctx.clip();
      ctx.strokeStyle = "rgba(233, 69, 96, 0.3)";
      ctx.lineWidth = 0.5;
      const minX = Math.min(...hole.map((v) => v.x));
      const minY = Math.min(...hole.map((v) => v.y));
      const maxX = Math.max(...hole.map((v) => v.x));
      const maxY = Math.max(...hole.map((v) => v.y));
      const spacing = 5;
      ctx.beginPath();
      for (
        let d = minX + minY - spacing;
        d <= maxX + maxY + spacing;
        d += spacing
      ) {
        ctx.moveTo(d - minY, minY);
        ctx.lineTo(d - maxY, maxY);
      }
      ctx.stroke();
      ctx.restore();

      // Hole boundary stroke
      ctx.beginPath();
      this.tracePath(hole);
      ctx.strokeStyle = "rgba(233, 69, 96, 0.6)";
      ctx.lineWidth = 0.8;
      ctx.stroke();
    }
  }

  private drawFace(face: Face): void {
    const { ctx } = this;
    ctx.beginPath();
    this.tracePath(face.contour);
    if (face.holes) {
      for (const hole of face.holes) {
        this.tracePath(hole);
      }
    }
    ctx.fillStyle = "rgba(255, 240, 150, 0.15)";
    ctx.fill("evenodd");
    ctx.strokeStyle = "rgba(255, 165, 0, 0.6)";
    ctx.lineWidth = 0.5;
    ctx.stroke();
  }

  private drawStall(stall: StallQuad): void {
    const { ctx } = this;
    const colors: Record<string, { fill: string; stroke: string }> = {
      Standard: {
        fill: "rgba(200, 200, 220, 0.4)",
        stroke: "rgba(180, 180, 200, 0.7)",
      },
      Ada: {
        fill: "rgba(60, 120, 220, 0.5)",
        stroke: "rgba(80, 140, 240, 0.8)",
      },
      Compact: {
        fill: "rgba(200, 200, 150, 0.4)",
        stroke: "rgba(180, 180, 130, 0.7)",
      },
      Ev: {
        fill: "rgba(100, 200, 100, 0.4)",
        stroke: "rgba(80, 180, 80, 0.7)",
      },
    };
    const c = colors[stall.kind] || colors.Standard;
    ctx.beginPath();
    ctx.moveTo(stall.corners[0].x, stall.corners[0].y);
    for (let i = 1; i < 4; i++) {
      ctx.lineTo(stall.corners[i].x, stall.corners[i].y);
    }
    ctx.closePath();
    ctx.fillStyle = c.fill;
    ctx.fill();
    ctx.strokeStyle = c.stroke;
    ctx.lineWidth = 0.3;
    ctx.stroke();
  }

  private drawIsland(island: Island): void {
    const { ctx } = this;
    ctx.beginPath();
    this.tracePath(island.polygon);
    ctx.fillStyle = "rgba(60, 160, 80, 0.5)";
    ctx.fill();
    ctx.strokeStyle = "rgba(40, 140, 60, 0.8)";
    ctx.lineWidth = 0.5;
    ctx.stroke();
  }

  private drawSpine(spine: SpineLine): void {
    const { ctx } = this;
    // Draw the spine line itself — bright cyan, dashed.
    ctx.beginPath();
    ctx.moveTo(spine.start.x, spine.start.y);
    ctx.lineTo(spine.end.x, spine.end.y);
    ctx.strokeStyle = "rgba(0, 220, 255, 0.8)";
    ctx.lineWidth = 0.8;
    ctx.setLineDash([3, 2]);
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw the outward normal at the midpoint — short tick showing
    // stall growth direction.
    const mx = (spine.start.x + spine.end.x) / 2;
    const my = (spine.start.y + spine.end.y) / 2;
    const tickLen = 4;
    ctx.beginPath();
    ctx.moveTo(mx, my);
    ctx.lineTo(mx + spine.normal.x * tickLen, my + spine.normal.y * tickLen);
    ctx.strokeStyle = "rgba(0, 220, 255, 0.6)";
    ctx.lineWidth = 0.6;
    ctx.stroke();
  }

  private drawVertexNetwork(state: AppState): void {
    const { ctx } = this;

    const graph: DriveAisleGraph | null =
      state.aisleGraph ?? state.layout?.resolved_graph ?? null;
    const isManualGraph = state.aisleGraph != null;

    // Draw aisle graph edges (dashed lines)
    if (graph) {
      ctx.setLineDash([2, 2]);
      ctx.strokeStyle = isManualGraph
        ? "rgba(100, 150, 255, 0.5)"
        : "rgba(100, 150, 255, 0.3)";
      ctx.lineWidth = 0.5;
      for (const edge of graph.edges) {
        const start = graph.vertices[edge.start];
        const end = graph.vertices[edge.end];
        ctx.beginPath();
        ctx.moveTo(start.x, start.y);
        ctx.lineTo(end.x, end.y);
        ctx.stroke();
      }
      ctx.setLineDash([]);
    }

    const aisleVerts = (graph?.vertices ?? []).map((v, i) => ({
      pos: v,
      ref: { type: "aisle" as const, index: i },
      color: isManualGraph
        ? "rgba(100, 150, 255, 0.9)"
        : "rgba(100, 150, 255, 0.5)",
    }));

    // Draw vertices
    const allVerts = [
      ...state.boundary.outer.map((v, i) => ({
        pos: v,
        ref: { type: "boundary-outer" as const, index: i },
        color: "rgba(255, 160, 50, 0.9)",
      })),
      ...state.boundary.holes.flatMap((hole, hi) =>
        hole.map((v, vi) => ({
          pos: v,
          ref: { type: "boundary-hole" as const, index: vi, holeIndex: hi },
          color: "rgba(255, 120, 50, 0.9)",
        })),
      ),
      ...aisleVerts,
    ];

    const radius = 3;
    for (const vert of allVerts) {
      const isSelected = this.vertexRefsEqual(vert.ref, state.selectedVertex);
      const isHovered = this.vertexRefsEqual(vert.ref, state.hoveredVertex);

      ctx.beginPath();
      ctx.arc(
        vert.pos.x,
        vert.pos.y,
        isSelected || isHovered ? radius * 1.5 : radius,
        0,
        Math.PI * 2,
      );
      ctx.fillStyle = isSelected
        ? "#ffffff"
        : isHovered
          ? "#ffff00"
          : vert.color;
      ctx.fill();
      if (isSelected || isHovered) {
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 0.5;
        ctx.stroke();
      }
    }
  }

  private vertexRefsEqual(
    a: VertexRef | null | undefined,
    b: VertexRef | null | undefined,
  ): boolean {
    if (!a || !b) return false;
    if (a.type !== b.type || a.index !== b.index) return false;
    if (a.type === "boundary-hole")
      return a.holeIndex === (b as VertexRef).holeIndex;
    return true;
  }

  private drawPendingHole(points: Vec2[]): void {
    const { ctx } = this;
    ctx.setLineDash([2, 2]);
    ctx.strokeStyle = "rgba(233, 69, 96, 0.7)";
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) {
      ctx.lineTo(points[i].x, points[i].y);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw vertex dots
    for (const p of points) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(233, 69, 96, 0.9)";
      ctx.fill();
    }
  }

  private drawScaleBar(_cam: Camera): void {
    const { ctx } = this;
    const barLenFt = 50;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.5)";
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    ctx.moveTo(10, -10);
    ctx.lineTo(10 + barLenFt, -10);
    ctx.stroke();
    // Ticks
    ctx.beginPath();
    ctx.moveTo(10, -13);
    ctx.lineTo(10, -7);
    ctx.moveTo(10 + barLenFt, -13);
    ctx.lineTo(10 + barLenFt, -7);
    ctx.stroke();
    // Label
    ctx.fillStyle = "rgba(255, 255, 255, 0.5)";
    ctx.font = "4px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(`${barLenFt} ft`, 10 + barLenFt / 2, -15);
  }

  private tracePath(points: Vec2[]): void {
    if (points.length === 0) return;
    this.ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) {
      this.ctx.lineTo(points[i].x, points[i].y);
    }
    this.ctx.closePath();
  }

  private drawPolygon(
    points: Vec2[],
    fill: string,
    stroke: string,
    lineWidth: number,
  ): void {
    const { ctx } = this;
    ctx.beginPath();
    this.tracePath(points);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lineWidth;
    ctx.stroke();
  }

  screenToWorld(screenX: number, screenY: number, cam: Camera): Vec2 {
    return {
      x: (screenX - cam.offsetX) / cam.zoom,
      y: (screenY - cam.offsetY) / cam.zoom,
    };
  }

  worldToScreen(worldX: number, worldY: number, cam: Camera): Vec2 {
    return {
      x: worldX * cam.zoom + cam.offsetX,
      y: worldY * cam.zoom + cam.offsetY,
    };
  }
}
