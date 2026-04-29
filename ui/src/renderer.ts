import { AppState, Camera, VertexRef } from "./app";
import {
  Vec2,
  EdgeArc,
  ParkingLayout,
  ParkingLot,
  StallQuad,
  DriveAisleGraph,
  SpineLine,
  Face,
  AisleDirection,
  annotationWorldPos,
  arcApex,
  bulgeToArc,
  computeRegionFrame,
} from "./types";
import { SnapGuide } from "./snap";

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

    // 1b. Snap guides
    if (state.snapGuides && state.snapGuides.length > 0) {
      this.drawSnapGuides(state.snapGuides, cam, w, h);
    }

    // 2–5. Lot rendering
    const lot = state.lot;
    this.drawBoundary(state, lot);

    const layout = lot.layout;
    if (layout) {

      if (state.layers.faces && layout.faces) {
        for (let i = 0; i < layout.faces.length; i++) {
          this.drawFace(layout.faces[i]);
        }
      }

      if (state.layers.regions && layout.region_debug) {
        const rd = layout.region_debug;
        const colors = Renderer.FACE_COLORS;
        const BIG = 1e7;
        for (let i = 0; i < rd.regions.length; i++) {
          const [r, g, b] = colors[i % colors.length];
          // Fill inside a stacked clip: (region interior) ∩ (not any
          // lot hole). Clipping avoids the evenodd-with-shared-edges
          // problem that arises when the region's clip_poly weaves
          // hole edges into its boundary cycle.
          ctx.save();
          ctx.beginPath();
          this.tracePath(rd.regions[i].clip_poly);
          ctx.clip();
          for (const hole of lot.boundary.holes) {
            if (hole.length < 3) continue;
            ctx.beginPath();
            ctx.moveTo(-BIG, -BIG);
            ctx.lineTo(BIG, -BIG);
            ctx.lineTo(BIG, BIG);
            ctx.lineTo(-BIG, BIG);
            ctx.closePath();
            this.tracePath(hole);
            ctx.clip("evenodd");
          }
          ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.15)`;
          ctx.fillRect(-BIG, -BIG, 2 * BIG, 2 * BIG);
          ctx.restore();
          // Stroke outline without any clip applied.
          ctx.beginPath();
          this.tracePath(rd.regions[i].clip_poly);
          ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.5)`;
          ctx.lineWidth = 1.0;
          ctx.stroke();
          // Mark the region's abstract-frame (0,0) anchor — the
          // point around which the aisle grid rotates, translates,
          // and stretches.
          const frame = computeRegionFrame(
            state.params,
            rd.regions[i].aisle_angle,
            rd.regions[i].aisle_offset,
          );
          const anchor = frame.origin_world;
          ctx.beginPath();
          ctx.arc(anchor.x, anchor.y, 4, 0, Math.PI * 2);
          ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.9)`;
          ctx.fill();
          ctx.strokeStyle = "rgba(255, 255, 255, 0.8)";
          ctx.lineWidth = 1.0;
          ctx.stroke();
        }
        for (const [start, end] of rd.separators) {
          ctx.beginPath();
          ctx.moveTo(start.x, start.y);
          ctx.lineTo(end.x, end.y);
          ctx.strokeStyle = "rgba(255, 255, 255, 0.7)";
          ctx.lineWidth = 2;
          ctx.stroke();
          for (const pt of [start, end]) {
            ctx.beginPath();
            ctx.arc(pt.x, pt.y, 3, 0, Math.PI * 2);
            ctx.fillStyle = "rgba(255, 255, 255, 0.9)";
            ctx.fill();
          }
        }
      }

      if (state.layers.islands && layout.islands) {
        for (const island of layout.islands) {
          const [fillR, fillG, fillB] = [75, 140, 60];
          const { ctx } = this;
          ctx.beginPath();
          this.tracePath(island.contour);
          if (island.holes) {
            for (const hole of island.holes) {
              this.tracePath(hole);
            }
          }
          ctx.fillStyle = `rgba(${fillR}, ${fillG}, ${fillB}, 0.6)`;
          ctx.fill("evenodd");
          ctx.strokeStyle = `rgba(${fillR}, ${fillG}, ${fillB}, 0.9)`;
          ctx.lineWidth = 0.5;
          ctx.stroke();
        }
      }

      if (state.layers.spines && layout.spines) {
        for (const spine of layout.spines) {
          this.drawSpine(spine);
        }
      }

      if (state.layers.stalls) {
        for (const stall of layout.stalls) {
          if (stall.kind === "Suppressed" || stall.kind === "Buffer") continue;
          this.drawStall(stall);
        }
      }

      // 5b. Custom-stall fills (Ada, Island). Drawn between the
      // optional debug `stalls` layer and the paint lines so the white
      // paint reads on top of the colored background — matches how
      // real-world ADA/island markings look.
      if (state.layers.customStalls) {
        this.drawCustomStalls(layout);
      }

      // 5c. Paint lines
      if (state.layers.paintLines) {
        this.drawPaintLines(state, lot);
      }
    }

    // 6. Vertex network overlay (always called; individual sections
    //    check their own layer flags inside).
    this.drawVertexNetwork(state);

    // 7. Pending hole preview
    if (state.pendingHole && state.pendingHole.length > 0) {
      this.drawPendingHole(state.pendingHole);
    }

    // 7b. Pending boundary preview
    if (state.pendingBoundary && state.pendingBoundary.length > 0) {
      this.drawPendingBoundary(state.pendingBoundary);
    }

    // 8. Pending drive line preview
    if (state.pendingDriveLine && state.pendingDriveLinePreview) {
      this.drawPendingDriveLine(state.pendingDriveLine, state.pendingDriveLinePreview);
    }

    // 8b. Pending stall-modifier line preview
    if (state.pendingStallLine && state.pendingStallLinePreview) {
      this.drawPendingStallLine(state.pendingStallLine, state.pendingStallLinePreview);
    }

    // 9. Scale bar
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

  private drawSnapGuides(guides: SnapGuide[], cam: Camera, w: number, h: number): void {
    const { ctx } = this;
    const worldLeft = -cam.offsetX / cam.zoom;
    const worldTop = -cam.offsetY / cam.zoom;
    const worldRight = (w - cam.offsetX) / cam.zoom;
    const worldBottom = (h - cam.offsetY) / cam.zoom;

    ctx.save();
    ctx.strokeStyle = "rgba(100, 200, 255, 0.7)";
    ctx.lineWidth = 1 / cam.zoom;
    ctx.setLineDash([6 / cam.zoom, 4 / cam.zoom]);

    for (const guide of guides) {
      ctx.beginPath();
      if (guide.axis === "x") {
        // Vertical line — cursor X aligns to another vertex's X.
        ctx.moveTo(guide.worldCoord, worldTop);
        ctx.lineTo(guide.worldCoord, worldBottom);
      } else {
        // Horizontal line — cursor Y aligns to another vertex's Y.
        ctx.moveTo(worldLeft, guide.worldCoord);
        ctx.lineTo(worldRight, guide.worldCoord);
      }
      ctx.stroke();
    }

    ctx.setLineDash([]);
    ctx.restore();
  }

  private drawBoundary(state: AppState, lot: ParkingLot): void {
    const { ctx } = this;
    const { boundary } = lot;

    const derivedOuter = lot.layout?.derived_outer ?? boundary.outer;
    ctx.beginPath();
    this.tracePath(derivedOuter);
    ctx.fillStyle = "rgba(40, 40, 60, 0.8)";
    ctx.fill();

    if (state.layers.vertices) {
      ctx.beginPath();
      this.tracePathWithArcs(boundary.outer, boundary.outer_arcs);
      ctx.strokeStyle = "rgba(233, 69, 96, 0.8)";
      ctx.lineWidth = 1;
      ctx.stroke();
    }

    const derivedHoles = lot.layout?.derived_holes ?? [];
    for (const dh of derivedHoles) {
      ctx.beginPath();
      this.tracePath(dh);
      ctx.fillStyle = "rgba(30, 30, 50, 0.95)";
      ctx.fill();
    }
    if (state.layers.vertices) {
      for (let hi = 0; hi < boundary.holes.length; hi++) {
        const hole = boundary.holes[hi];
        const holeArcs = boundary.hole_arcs?.[hi];
        ctx.beginPath();
        this.tracePathWithArcs(hole, holeArcs);
        ctx.strokeStyle = "rgba(233, 69, 96, 0.6)";
        ctx.lineWidth = 0.8;
        ctx.stroke();
      }

      // Draw edge midpoint / apex handles.
      this.drawEdgeHandles(boundary.outer, boundary.outer_arcs);
      for (let hi = 0; hi < boundary.holes.length; hi++) {
        this.drawEdgeHandles(boundary.holes[hi], boundary.hole_arcs?.[hi]);
      }
    }
  }

  private drawEdgeHandles(
    points: Vec2[],
    arcs: (EdgeArc | null)[] | undefined,
  ): void {
    const { ctx } = this;
    const alpha = 0.6;
    const ghostAlpha = 0.4;

    for (let i = 0; i < points.length; i++) {
      const p0 = points[i];
      const p1 = points[(i + 1) % points.length];
      const arc = arcs?.[i];

      if (arc) {
        // Solid handle at the arc apex.
        const apex = arcApex(p0, p1, arc.bulge);
        ctx.beginPath();
        ctx.arc(apex.x, apex.y, 3, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(255, 160, 50, ${alpha})`;
        ctx.fill();
        ctx.strokeStyle = `rgba(255, 255, 255, ${alpha})`;
        ctx.lineWidth = 0.6;
        ctx.stroke();
      } else {
        // Ghost handle at the chord midpoint for straight edges.
        const mid = { x: (p0.x + p1.x) / 2, y: (p0.y + p1.y) / 2 };
        ctx.beginPath();
        ctx.arc(mid.x, mid.y, 2.5, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(255, 160, 50, ${ghostAlpha})`;
        ctx.fill();
        ctx.strokeStyle = `rgba(255, 255, 255, ${ghostAlpha * 0.5})`;
        ctx.lineWidth = 0.5;
        ctx.stroke();
      }
    }
  }

  private static FACE_COLORS = [
    [255, 100, 100], // red
    [100, 200, 255], // blue
    [100, 255, 100], // green
    [255, 200, 50],  // yellow
    [200, 100, 255], // purple
    [255, 150, 50],  // orange
    [50, 255, 200],  // teal
    [255, 100, 200], // pink
    [150, 255, 50],  // lime
    [100, 150, 255], // indigo
    [255, 220, 100], // gold
    [50, 200, 150],  // seafoam
  ];

  private drawFace(face: Face): void {
    const { ctx } = this;
    const [r, g, b] = [200, 200, 220];
    ctx.beginPath();
    this.tracePath(face.contour);
    if (face.holes) {
      for (const hole of face.holes) {
        this.tracePath(hole);
      }
    }
    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.25)`;
    ctx.fill("evenodd");
    ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.8)`;
    ctx.lineWidth = 0.5;
    ctx.stroke();

    if (face.is_boundary) {
      ctx.save();
      ctx.beginPath();
      this.tracePath(face.contour);
      if (face.holes) {
        for (const hole of face.holes) this.tracePath(hole);
      }
      ctx.clip("evenodd");
      const xs = face.contour.map(v => v.x);
      const ys = face.contour.map(v => v.y);
      const minX = Math.min(...xs), maxX = Math.max(...xs);
      const minY = Math.min(...ys), maxY = Math.max(...ys);
      ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.4)`;
      ctx.lineWidth = 0.3;
      ctx.beginPath();
      for (let d = minX + minY; d < maxX + maxY; d += 6) {
        ctx.moveTo(Math.max(minX, d - maxY), Math.min(maxY, d - minX));
        ctx.lineTo(Math.min(maxX, d - minY), Math.max(minY, d - maxX));
      }
      ctx.stroke();
      ctx.restore();
    }

    // Edge provenance coloring: magenta=wall, cyan=interior, yellow=perimeter.
    const drawEdgeSources = (contour: Vec2[], sources: string[]) => {
      for (let ei = 0; ei < sources.length; ei++) {
        const src = sources[ei];
        const a = contour[ei];
        const b = contour[(ei + 1) % contour.length];
        let color: string;
        if (src === "wall") {
          color = "rgba(255, 0, 255, 0.9)";
        } else if (src === "interior") {
          color = "rgba(0, 200, 255, 0.9)";
        } else {
          color = "rgba(255, 200, 50, 0.9)";
        }
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.strokeStyle = color;
        ctx.lineWidth = 0.6;
        ctx.stroke();
      }
    };
    if (face.edge_sources && face.edge_sources.length > 0) {
      drawEdgeSources(face.contour, face.edge_sources);
    }
    if (face.hole_edge_sources && face.holes) {
      for (let hi = 0; hi < face.hole_edge_sources.length; hi++) {
        if (face.holes[hi] && face.hole_edge_sources[hi].length > 0) {
          drawEdgeSources(face.holes[hi], face.hole_edge_sources[hi]);
        }
      }
    }
  }

  private drawStall(stall: StallQuad): void {
    const { ctx } = this;
    const colors: Record<string, { fill: string; stroke: string }> = {
      Standard: {
        fill: "rgba(200, 200, 220, 0.4)",
        stroke: "rgba(180, 180, 200, 0.7)",
      },
      Ada: {
        fill: "rgba(80, 140, 220, 0.4)",
        stroke: "rgba(60, 110, 190, 0.8)",
      },
      Compact: {
        fill: "rgba(156, 175, 136, 0.4)",
        stroke: "rgba(120, 142, 102, 0.8)",
      },
      Island: {
        fill: "rgba(75, 140, 60, 0.5)",
        stroke: "rgba(60, 120, 45, 0.7)",
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
    // Stroke three sides, leaving one edge open (matches paint lines).
    ctx.beginPath();
    if (stall.kind === "Island") {
      ctx.moveTo(stall.corners[1].x, stall.corners[1].y);
      ctx.lineTo(stall.corners[2].x, stall.corners[2].y);
      ctx.lineTo(stall.corners[3].x, stall.corners[3].y);
      ctx.lineTo(stall.corners[0].x, stall.corners[0].y);
    } else {
      ctx.moveTo(stall.corners[3].x, stall.corners[3].y);
      ctx.lineTo(stall.corners[0].x, stall.corners[0].y);
      ctx.lineTo(stall.corners[1].x, stall.corners[1].y);
      ctx.lineTo(stall.corners[2].x, stall.corners[2].y);
    }
    ctx.strokeStyle = c.stroke;
    ctx.lineWidth = 0.3;
    ctx.stroke();
  }

  /** Filled colored backgrounds for non-Standard, non-Suppressed
   *  stalls. Sits underneath the paint-lines layer so painted edges
   *  read on top of the color (matches real-world ADA/island marking). */
  private drawCustomStalls(layout: ParkingLayout): void {
    const { ctx } = this;
    const fillFor: Record<string, string> = {
      Ada: "rgba(60, 110, 200, 0.85)",
      Compact: "rgba(156, 175, 136, 0.85)",
      Island: "rgba(75, 140, 60, 0.55)",
    };
    for (const stall of layout.stalls) {
      const fill = fillFor[stall.kind];
      if (!fill) continue;
      ctx.beginPath();
      ctx.moveTo(stall.corners[0].x, stall.corners[0].y);
      for (let i = 1; i < 4; i++) {
        ctx.lineTo(stall.corners[i].x, stall.corners[i].y);
      }
      ctx.closePath();
      ctx.fillStyle = fill;
      ctx.fill();
    }
  }

  private drawPaintLines(_state: AppState, lot: ParkingLot): void {
    const { ctx } = this;
    ctx.save();
    ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
    ctx.lineWidth = 0.5;
    ctx.lineCap = "round";

    // Stall paint lines.
    // corners: [0]=back_left, [1]=back_right, [2]=aisle_right, [3]=aisle_left
    // Regular stalls: paint three sides — left/back/right — leaving
    //   the aisle entrance ([2]→[3]) open.
    // Buffer stalls: fully closed rectangle — no parking, no usable entrance.
    // Island stalls: just back ([0]→[1]) and aisle ([2]→[3]); the hatch
    //   inside makes it read as a no-park area without boxing in the
    //   neighboring stalls' side paint.
    if (lot.layout) {
      ctx.beginPath();
      for (const stall of lot.layout.stalls) {
        if (stall.kind === "Suppressed") continue;
        const c = stall.corners;
        if (stall.kind === "Buffer") {
          ctx.moveTo(c[0].x, c[0].y);
          ctx.lineTo(c[1].x, c[1].y);
          ctx.lineTo(c[2].x, c[2].y);
          ctx.lineTo(c[3].x, c[3].y);
          ctx.closePath();
        } else if (stall.kind === "Island") {
          ctx.moveTo(c[0].x, c[0].y);
          ctx.lineTo(c[1].x, c[1].y);
          ctx.moveTo(c[2].x, c[2].y);
          ctx.lineTo(c[3].x, c[3].y);
        } else {
          ctx.moveTo(c[3].x, c[3].y);
          ctx.lineTo(c[0].x, c[0].y);
          ctx.lineTo(c[1].x, c[1].y);
          ctx.lineTo(c[2].x, c[2].y);
        }
      }
      ctx.stroke();

      const hatchAngleDeg = 45;
      const hatchRad = hatchAngleDeg * Math.PI / 180;
      for (const stall of lot.layout.stalls) {
        if (stall.kind !== "Island" && stall.kind !== "Buffer") continue;
        const c = stall.corners;
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(c[0].x, c[0].y);
        ctx.lineTo(c[1].x, c[1].y);
        ctx.lineTo(c[2].x, c[2].y);
        ctx.lineTo(c[3].x, c[3].y);
        ctx.closePath();
        ctx.clip();

        // Hatch at 45° relative to the back edge ([0]→[1]).
        const ux = c[1].x - c[0].x, uy = c[1].y - c[0].y;
        const uLen = Math.sqrt(ux * ux + uy * uy);
        const unx = ux / uLen, uny = uy / uLen;
        // Rotate back-edge direction by 45°.
        const hx = unx * Math.cos(hatchRad) - uny * Math.sin(hatchRad);
        const hy = unx * Math.sin(hatchRad) + uny * Math.cos(hatchRad);
        // hx,hy is already unit length (rotation of unit vector).
        // Perpendicular to hatch direction (sweep axis).
        const nx = -hy, ny = hx;

        // Project corners onto sweep axis to find range.
        const projs = [0, 1, 2, 3].map(i => c[i].x * nx + c[i].y * ny);
        const minP = Math.min(...projs), maxP = Math.max(...projs);

        // Extent along hatch direction for line length.
        const hProjs = [0, 1, 2, 3].map(i => c[i].x * hx + c[i].y * hy);
        const halfH = (Math.max(...hProjs) - Math.min(...hProjs)) / 2 + 2;
        const midH = (Math.max(...hProjs) + Math.min(...hProjs)) / 2;

        const spacing = 3;
        ctx.beginPath();
        for (let p = minP; p <= maxP; p += spacing) {
          const bx = p * nx + midH * hx;
          const by = p * ny + midH * hy;
          ctx.moveTo(bx - halfH * hx, by - halfH * hy);
          ctx.lineTo(bx + halfH * hx, by + halfH * hy);
        }
        ctx.stroke();
        ctx.restore();
      }

      if (lot.layout.islands) {
        ctx.beginPath();
        for (const island of lot.layout.islands) {
          this.tracePath(island.contour);
          if (island.holes) {
            for (const hole of island.holes) {
              this.tracePath(hole);
            }
          }
        }
        ctx.stroke();
      }
    }

    ctx.restore();
  }

  private drawSpine(spine: SpineLine): void {
    const { ctx } = this;

    // Color by normal direction so opposing spines are visually distinct.
    // Map to 4 high-contrast colors: right=cyan, up=orange, left=red, down=blue.
    const angle = Math.atan2(spine.normal.y, spine.normal.x);
    const quadrant = Math.round(((angle * 180 / Math.PI) + 360) % 360 / 90) % 4;
    const hues = [180, 40, 0, 220]; // right, up, left, down
    const hue = hues[quadrant];
    const color = `hsla(${hue}, 100%, 70%, 0.95)`;
    const colorFaint = `hsla(${hue}, 100%, 70%, 0.8)`;

    ctx.beginPath();
    ctx.moveTo(spine.start.x, spine.start.y);
    ctx.lineTo(spine.end.x, spine.end.y);
    ctx.strokeStyle = color;
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
    ctx.strokeStyle = colorFaint;
    ctx.lineWidth = 0.6;
    ctx.stroke();
  }

  private drawVertexNetwork(state: AppState): void {
    const { ctx } = this;
    const graph: DriveAisleGraph | null =
      state.lot.layout?.resolved_graph ?? null;

    // Draw aisle graph edges (dashed lines)
    if (graph && state.layers.vertices) {
      const seen = new Set<string>();
      for (let ei = 0; ei < graph.edges.length; ei++) {
        const edge = graph.edges[ei];
        const key = Math.min(edge.start, edge.end) + "," + Math.max(edge.start, edge.end);
        if (seen.has(key)) continue;
        seen.add(key);

        const start = graph.vertices[edge.start];
        const end = graph.vertices[edge.end];
        const isSelected = state.selectedEdge?.chain.includes(ei) ?? false;
        const isSegmentSelected = isSelected && state.selectedEdge?.mode === "segment" && state.selectedEdge?.index === ei;
        const direction: AisleDirection | null = edge.direction ?? null;
        const isOneWay = direction === "OneWay" || direction === "OneWayReverse";
        const isTwoWayReverse = direction === "TwoWayReverse";
        const hasDirection = isOneWay || isTwoWayReverse;

        // Edge line — segment selection is solid+thick, chain selection
        // is dashed+thick. Directional edges (one-way OR two-way-reverse)
        // get the same yellow solid treatment so they stand out from
        // plain bidirectional edges.
        ctx.setLineDash(isSegmentSelected ? [] : hasDirection ? [] : [2, 2]);
        const isPerimeter = edge.interior === false;
        ctx.strokeStyle = isSelected
          ? "rgba(255, 220, 50, 0.9)"
          : isOneWay
            ? "rgba(255, 180, 50, 0.7)"
            : isTwoWayReverse
              ? "rgba(80, 160, 230, 0.8)"
              : isPerimeter
                ? "rgba(255, 100, 100, 0.5)"
                : "rgba(100, 150, 255, 0.3)";
        ctx.lineWidth = isSegmentSelected ? 2.5 : isSelected ? 1.5 : hasDirection ? 1.0 : 0.5;
        ctx.beginPath();
        ctx.moveTo(start.x, start.y);
        ctx.lineTo(end.x, end.y);
        ctx.stroke();

        // Direction markers along the edge.
        //   OneWay / OneWayReverse → single arrow per tick, pointing
        //     in the travel direction (start→end or end→start).
        //   TwoWayReverse          → bidirectional ↔ at each tick:
        //     a short shaft with arrowheads on both ends, signalling
        //     "two-way traffic, reversed stall lean".
        if (hasDirection) {
          const [travelStart, travelEnd] =
            direction === "OneWayReverse" ? [end, start] : [start, end];
          const dx = travelEnd.x - travelStart.x;
          const dy = travelEnd.y - travelStart.y;
          const len = Math.sqrt(dx * dx + dy * dy);
          if (len > 1) {
            const tnx = dx / len;
            const tny = dy / len;
            const arrowSize = 5;
            const color = isSelected
              ? "rgba(255, 220, 50, 0.9)"
              : isTwoWayReverse
                ? "rgba(80, 160, 230, 0.9)"
                : "rgba(255, 180, 50, 0.8)";
            // Helper: filled arrowhead with tip at (tx, ty), pointing
            // along (dx, dy) (unit). Base sits ½·arrowSize behind tip.
            const drawArrow = (tx: number, ty: number, ux: number, uy: number) => {
              const baseX = tx - ux * arrowSize * 0.5;
              const baseY = ty - uy * arrowSize * 0.5;
              ctx.beginPath();
              ctx.moveTo(tx, ty);
              ctx.lineTo(baseX + uy * arrowSize * 0.5, baseY - ux * arrowSize * 0.5);
              ctx.lineTo(baseX - uy * arrowSize * 0.5, baseY + ux * arrowSize * 0.5);
              ctx.closePath();
              ctx.fillStyle = color;
              ctx.fill();
            };
            for (const t of [0.33, 0.66]) {
              const ax = travelStart.x + dx * t;
              const ay = travelStart.y + dy * t;
              if (isOneWay) {
                // Forward arrow only, tip ahead of (ax, ay).
                drawArrow(ax + tnx * arrowSize, ay + tny * arrowSize, tnx, tny);
              } else {
                // Bidirectional: shaft + arrowheads at both ends.
                const shaftHalf = arrowSize * 1.1;
                ctx.strokeStyle = color;
                ctx.lineWidth = 1.0;
                ctx.beginPath();
                ctx.moveTo(ax - tnx * shaftHalf, ay - tny * shaftHalf);
                ctx.lineTo(ax + tnx * shaftHalf, ay + tny * shaftHalf);
                ctx.stroke();
                drawArrow(
                  ax + tnx * (shaftHalf + arrowSize * 0.5),
                  ay + tny * (shaftHalf + arrowSize * 0.5),
                  tnx,
                  tny,
                );
                drawArrow(
                  ax - tnx * (shaftHalf + arrowSize * 0.5),
                  ay - tny * (shaftHalf + arrowSize * 0.5),
                  -tnx,
                  -tny,
                );
              }
            }
          }
        }
      }
      ctx.setLineDash([]);
    }

    // Draw drive line edges (solid green + faint infinite extent)
    const allDriveLines = state.layers.driveLines ? state.lot.driveLines : [];
    for (const dl of allDriveLines) {
      const dir = { x: dl.end.x - dl.start.x, y: dl.end.y - dl.start.y };
      const len = Math.sqrt(dir.x * dir.x + dir.y * dir.y);
      if (len > 1e-9) {
        const nx = dir.x / len;
        const ny = dir.y / len;
        const ext = 10000;
        // Faint infinite extent
        ctx.beginPath();
        ctx.moveTo(dl.start.x - nx * ext, dl.start.y - ny * ext);
        ctx.lineTo(dl.end.x + nx * ext, dl.end.y + ny * ext);
        ctx.strokeStyle = "rgba(50, 200, 100, 0.25)";
        ctx.lineWidth = 0.5;
        ctx.setLineDash([4, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      // Solid control segment
      ctx.beginPath();
      ctx.moveTo(dl.start.x, dl.start.y);
      ctx.lineTo(dl.end.x, dl.end.y);
      ctx.strokeStyle = "rgba(50, 200, 100, 0.8)";
      ctx.lineWidth = 1.0;
      ctx.stroke();
    }

    // Stall-modifier lines — distinct color per kind. Always shown
    // regardless of layer toggle (they're a small post-pass effect,
    // not part of the structural drive-line family).
    const stallLineColor: Record<string, string> = {
      Suppressed: "rgba(220, 60, 60, 0.85)",   // red — "remove these stalls"
      Ada:        "rgba(80, 140, 220, 0.95)", // ADA blue, matches engine fill
      Compact:    "rgba(156, 175, 136, 0.95)", // sage green
      Island:     "rgba(159, 191, 138, 0.95)", // matches island fill
      Standard:   "rgba(120, 120, 120, 0.85)",
    };
    for (const sm of state.lot.stallModifiers) {
      const pl = sm.polyline;
      if (pl.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(pl[0].x, pl[0].y);
      for (let i = 1; i < pl.length; i++) ctx.lineTo(pl[i].x, pl[i].y);
      ctx.strokeStyle = stallLineColor[sm.kind] ?? stallLineColor.Suppressed;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Hide orphan aisle vertices (no incident edges) — vestiges of edge
    // deletions / post-deletion graph simplification — and hide
    // degree-2 near-collinear perimeter vertices, which are arc
    // discretization facets rather than real graph junctions.
    const nv = graph?.vertices.length ?? 0;
    const aisleDegree = new Int32Array(nv);
    const nbrA = new Int32Array(nv).fill(-1);
    const nbrB = new Int32Array(nv).fill(-1);
    if (graph) {
      const seenE = new Set<string>();
      for (const e of graph.edges) {
        const k = Math.min(e.start, e.end) + "," + Math.max(e.start, e.end);
        if (seenE.has(k)) continue;
        seenE.add(k);
        for (const [v, o] of [[e.start, e.end], [e.end, e.start]] as const) {
          const d = aisleDegree[v]++;
          if (d === 0) nbrA[v] = o;
          else if (d === 1) nbrB[v] = o;
        }
      }
    }
    const aisleVerts = (graph?.vertices ?? []).flatMap((v, i) => {
      const deg = aisleDegree[i];
      if (deg === 0) return [];
      if (deg === 2 && graph) {
        const a = graph.vertices[nbrA[i]];
        const b = graph.vertices[nbrB[i]];
        const d1x = v.x - a.x, d1y = v.y - a.y;
        const d2x = b.x - v.x, d2y = b.y - v.y;
        const l1 = Math.hypot(d1x, d1y);
        const l2 = Math.hypot(d2x, d2y);
        if (l1 > 1e-9 && l2 > 1e-9) {
          const dot = (d1x * d2x + d1y * d2y) / (l1 * l2);
          if (dot > 0.95) return [];
        }
      }
      return [{
        pos: v,
        ref: { type: "aisle" as const, index: i },
        color: "rgba(100, 150, 255, 0.5)",
      }];
    });

    const lot = state.lot;

    const driveLineVerts: { pos: Vec2; ref: VertexRef; color: string }[] = [];
    if (state.layers.driveLines) {
      lot.driveLines.forEach((dl, i) => {
        driveLineVerts.push({
          pos: dl.start,
          ref: { type: "drive-line", index: i, endpoint: "start" },
          color: "#00ff88",
        });
        driveLineVerts.push({
          pos: dl.end,
          ref: { type: "drive-line", index: i, endpoint: "end" },
          color: "#00ff88",
        });
      });
    }

    const stallLineVerts: { pos: Vec2; ref: VertexRef; color: string }[] = [];
    lot.stallModifiers.forEach((sm, i) => {
      const pl = sm.polyline;
      if (pl.length === 0) return;
      stallLineVerts.push({
        pos: pl[0],
        ref: { type: "stall-line", index: i, endpoint: "start" },
        color: "#dc3c3c",
      });
      if (pl.length >= 2) {
        stallLineVerts.push({
          pos: pl[pl.length - 1],
          ref: { type: "stall-line", index: i, endpoint: "end" },
          color: "#dc3c3c",
        });
      }
    });

    const annotationVerts: { pos: Vec2; ref: VertexRef; color: string }[] = [];
    lot.annotations.forEach((ann, i) => {
      const pos = annotationWorldPos(ann, lot, state.params);
      if (!pos) return;
      const isDelete =
        ann.kind === "DeleteVertex" || ann.kind === "DeleteEdge";
      const isTwoWay =
        ann.kind === "Direction" && ann.traffic === "TwoWayReverse";
      const activeColor = isDelete
        ? "rgba(255, 80, 80, 0.95)"
        : isTwoWay
          ? "rgba(100, 200, 255, 0.95)"
          : "rgba(255, 180, 50, 0.95)";
      const inactiveColor = isDelete
        ? "rgba(255, 80, 80, 0.3)"
        : isTwoWay
          ? "rgba(100, 200, 255, 0.3)"
          : "rgba(255, 180, 50, 0.3)";
      annotationVerts.push({
        pos,
        ref: { type: "annotation", index: i },
        color: ann._active === false ? inactiveColor : activeColor,
      });
    });

    const boundaryVerts: { pos: Vec2; ref: VertexRef; color: string }[] = [];
    if (state.layers.vertices) {
      lot.boundary.outer.forEach((v, i) => {
        boundaryVerts.push({
          pos: v,
          ref: { type: "boundary-outer", index: i },
          color: "rgba(255, 160, 50, 0.9)",
        });
      });
      lot.boundary.holes.forEach((hole, hi) => {
        hole.forEach((v, vi) => {
          boundaryVerts.push({
            pos: v,
            ref: { type: "boundary-hole", index: vi, holeIndex: hi },
            color: "rgba(255, 120, 50, 0.9)",
          });
        });
      });
    }

    const allVerts = [
      ...(state.layers.vertices ? [...boundaryVerts, ...aisleVerts] : []),
      ...driveLineVerts,
      ...stallLineVerts,
      ...annotationVerts,
    ];

    const radius = 3;
    for (const vert of allVerts) {
      const isSelected = this.vertexRefsEqual(vert.ref, state.selectedVertex);
      const isHovered = this.vertexRefsEqual(vert.ref, state.hoveredVertex);
      const isAnnotation = vert.ref.type === "annotation";
      const isDriveLine = vert.ref.type === "drive-line";
      const baseRadius = (isAnnotation || isDriveLine) ? radius * 1.5 : radius;

      if (isAnnotation) {
        // Draw diamond shape for annotation anchors.
        const r = baseRadius * 1.3;
        ctx.beginPath();
        ctx.moveTo(vert.pos.x, vert.pos.y - r);
        ctx.lineTo(vert.pos.x + r, vert.pos.y);
        ctx.lineTo(vert.pos.x, vert.pos.y + r);
        ctx.lineTo(vert.pos.x - r, vert.pos.y);
        ctx.closePath();
      } else {
        ctx.beginPath();
        ctx.arc(
          vert.pos.x,
          vert.pos.y,
          isSelected || isHovered ? baseRadius * 1.5 : baseRadius,
          0,
          Math.PI * 2,
        );
      }
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

    if (state.layers.regions) {
      this.drawRegionVectors(state);
    }
  }

  private drawRegionVectors(state: AppState): void {
    const { ctx } = this;
    const colors = Renderer.FACE_COLORS;
    const halfLen = 30;

    {
      const rd = state.lot.layout?.region_debug;
      if (!rd || rd.regions.length === 0) return;

      for (let i = 0; i < rd.regions.length; i++) {
        const region = rd.regions[i];
        const [r, g, b] = colors[i % colors.length];
        const angleRad = region.aisle_angle * (Math.PI / 180);
        const dirX = Math.cos(angleRad);
        const dirY = Math.sin(angleRad);
        const cx = region.center.x;
        const cy = region.center.y;
        const sx = cx - dirX * halfLen;
        const sy = cy - dirY * halfLen;
        const ex = cx + dirX * halfLen;
        const ey = cy + dirY * halfLen;

        const ext = 10000;
        ctx.beginPath();
        ctx.moveTo(cx - dirX * ext, cy - dirY * ext);
        ctx.lineTo(cx + dirX * ext, cy + dirY * ext);
        ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.15)`;
        ctx.lineWidth = 0.5;
        ctx.setLineDash([4, 4]);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(ex, ey);
        ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.9)`;
        ctx.lineWidth = 2;
        ctx.stroke();

        const arrowSize = 7;
        ctx.beginPath();
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - dirX * arrowSize + dirY * arrowSize * 0.4, ey - dirY * arrowSize - dirX * arrowSize * 0.4);
        ctx.lineTo(ex - dirX * arrowSize - dirY * arrowSize * 0.4, ey - dirY * arrowSize + dirX * arrowSize * 0.4);
        ctx.closePath();
        ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.9)`;
        ctx.fill();

        const endpoints: { pos: Vec2; endpoint: "start" | "end" }[] = [
          { pos: { x: sx, y: sy }, endpoint: "start" },
          { pos: { x: ex, y: ey }, endpoint: "end" },
        ];
        for (const ep of endpoints) {
          const ref: VertexRef = { type: "region-vector", index: i, endpoint: ep.endpoint };
          const isSelected = this.vertexRefsEqual(ref, state.selectedVertex);
          const isHovered = this.vertexRefsEqual(ref, state.hoveredVertex);
          const radius = isSelected || isHovered ? 6 : 4.5;
          ctx.beginPath();
          ctx.arc(ep.pos.x, ep.pos.y, radius, 0, Math.PI * 2);
          ctx.fillStyle = isSelected ? "#ffffff" : isHovered ? "#ffff00" : `rgba(${r}, ${g}, ${b}, 0.95)`;
          ctx.fill();
          ctx.strokeStyle = "rgba(0, 0, 0, 0.5)";
          ctx.lineWidth = 0.8;
          ctx.stroke();
        }
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
    if (a.type === "drive-line" || a.type === "aisle-vector" || a.type === "region-vector")
      return a.endpoint === (b as VertexRef).endpoint;
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

  private drawPendingBoundary(points: Vec2[]): void {
    const { ctx } = this;
    ctx.setLineDash([3, 3]);
    ctx.strokeStyle = "rgba(100, 200, 255, 0.8)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) {
      ctx.lineTo(points[i].x, points[i].y);
    }
    if (points.length >= 3) {
      ctx.closePath();
      ctx.fillStyle = "rgba(100, 200, 255, 0.1)";
      ctx.fill();
    }
    ctx.stroke();
    ctx.setLineDash([]);

    for (const p of points) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(100, 200, 255, 0.9)";
      ctx.fill();
    }
  }

  private drawPendingDriveLine(start: Vec2, previewEnd: Vec2): void {
    const { ctx } = this;
    ctx.setLineDash([3, 3]);
    ctx.strokeStyle = "rgba(50, 200, 100, 0.7)";
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(previewEnd.x, previewEnd.y);
    ctx.stroke();
    ctx.setLineDash([]);

    // Dot at start point
    ctx.beginPath();
    ctx.arc(start.x, start.y, 3, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(50, 200, 100, 0.9)";
    ctx.fill();
  }

  private drawPendingStallLine(start: Vec2, previewEnd: Vec2): void {
    const { ctx } = this;
    ctx.setLineDash([3, 3]);
    ctx.strokeStyle = "rgba(220, 60, 60, 0.7)";
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(previewEnd.x, previewEnd.y);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.arc(start.x, start.y, 3, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(220, 60, 60, 0.9)";
    ctx.fill();
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

  private tracePathWithArcs(points: Vec2[], arcs?: (EdgeArc | null)[]): void {
    if (points.length === 0) return;
    this.ctx.moveTo(points[0].x, points[0].y);
    for (let i = 0; i < points.length; i++) {
      const next = (i + 1) % points.length;
      const arc = arcs?.[i];
      if (arc) {
        const p0 = points[i];
        const p1 = points[next];
        const def = bulgeToArc(p0, p1, arc.bulge);
        // Canvas angles are measured from +x and the sweep direction is
        // controlled by the `anticlockwise` flag. Our sweep is signed:
        // positive → CCW (angles increasing). Pass `anticlockwise=true`
        // when sweep is positive so Canvas traces the same short path
        // through the apex that our math samples.
        this.ctx.arc(
          def.center.x,
          def.center.y,
          def.radius,
          def.startAngle,
          def.startAngle + def.sweep,
          def.sweep > 0,
        );
      } else {
        this.ctx.lineTo(points[next].x, points[next].y);
      }
    }
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
