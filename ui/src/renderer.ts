import { AppState, Camera, VertexRef } from "./app";
import {
  Vec2,
  EdgeCurve,
  ParkingLot,
  StallQuad,
  DriveAisleGraph,
  SpineLine,
  Face,
  AisleDirection,
  ParkingParams,
  Annotation,
  isAbstractAnnotation,
  computeRegionFrame,
  frameForward,
} from "./types";

/**
 * Compute the current world position of an abstract annotation by
 * looking up its host region in the layout's region_debug and
 * forward-transforming the integer (xi, yi) — or the midpoint of
 * (xa, ya) and (xb, yb) for edge variants — through that region's
 * AbstractFrame. Returns null if the region isn't in the current
 * layout (dormant annotation).
 */
function abstractAnnotationWorldPos(
  ann: Annotation,
  lot: ParkingLot,
  params: ParkingParams,
): Vec2 | null {
  if (!isAbstractAnnotation(ann)) return null;
  const rd = lot.layout?.region_debug;
  if (!rd) return null;
  const region = rd.regions.find((r) => r.id === ann.region);
  if (!region) return null;
  const frame = computeRegionFrame(
    params,
    region.aisle_angle_deg,
    region.aisle_offset,
  );
  if (ann.kind === "AbstractDeleteVertex") {
    return frameForward(frame, { x: ann.xi, y: ann.yi });
  }
  // Edge variants: show the marker at the midpoint of the two
  // endpoints.
  const a = frameForward(frame, { x: ann.xa, y: ann.ya });
  const b = frameForward(frame, { x: ann.xb, y: ann.yb });
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

/**
 * Find an AbstractOneWay / AbstractTwoWayOriented annotation whose
 * resolved world-space midpoint is near `mid`, and return the unit
 * travel direction implied by its (xa, ya) → (xb, yb) ordering. Used
 * by the edge arrow renderer so abstract direction annotations get
 * arrows just like legacy OneWay / TwoWayOriented do.
 */
function findAbstractDirectionAnnotation(
  lot: ParkingLot | null,
  params: ParkingParams,
  mid: Vec2,
): { tx: number; ty: number } | null {
  if (!lot) return null;
  const rd = lot.layout?.region_debug;
  if (!rd) return null;
  const tol = 5.0;
  for (const ann of lot.annotations) {
    if (
      ann.kind !== "AbstractOneWay" &&
      ann.kind !== "AbstractTwoWayOriented"
    ) {
      continue;
    }
    if (ann._active === false) continue;
    const region = rd.regions.find((r) => r.id === ann.region);
    if (!region) continue;
    const frame = computeRegionFrame(
      params,
      region.aisle_angle_deg,
      region.aisle_offset,
    );
    const a = frameForward(frame, { x: ann.xa, y: ann.ya });
    const b = frameForward(frame, { x: ann.xb, y: ann.yb });
    const worldMid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    const d = Math.sqrt((worldMid.x - mid.x) ** 2 + (worldMid.y - mid.y) ** 2);
    if (d > tol) continue;
    const tx = b.x - a.x;
    const ty = b.y - a.y;
    const len = Math.sqrt(tx * tx + ty * ty);
    if (len < 1e-9) continue;
    return { tx: tx / len, ty: ty / len };
  }
  return null;
}
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

    // 2–5. Per-lot rendering
    for (const lot of state.lots) {
      this.drawBoundary(state, lot);

      const layout = lot.layout;
      if (!layout) continue;

      if (state.layers.faces && layout.faces) {
        for (let i = 0; i < layout.faces.length; i++) {
          this.drawFace(layout.faces[i], i, state.layers.faceColors);
        }
      }

      if (state.layers.regions && layout.region_debug) {
        const rd = layout.region_debug;
        const colors = Renderer.FACE_COLORS;
        for (let i = 0; i < rd.regions.length; i++) {
          const [r, g, b] = colors[i % colors.length];
          ctx.beginPath();
          this.tracePath(rd.regions[i].clip_poly);
          for (const hole of lot.boundary.holes) {
            if (hole.length >= 3) {
              this.tracePath(hole);
            }
          }
          ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.15)`;
          ctx.fill("evenodd");
          ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.5)`;
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

      if (state.layers.miterFills && layout.miter_fills) {
        for (const fill of layout.miter_fills) {
          this.drawPolygon(fill, "rgba(255, 100, 100, 0.3)", "rgba(255, 80, 80, 0.7)", 0.5);
        }
      }

      if (state.layers.skeletonDebug && layout.skeleton_debug) {
        const colors = Renderer.FACE_COLORS;
        for (let si = 0; si < layout.skeleton_debug.length; si++) {
          const sk = layout.skeleton_debug[si];
          let [r, g, b] = state.layers.faceColors
            ? colors[si % colors.length]
            : [78, 205, 196];
          for (const arc of sk.arcs) {
            this.ctx.beginPath();
            this.ctx.moveTo(arc[0].x, arc[0].y);
            this.ctx.lineTo(arc[1].x, arc[1].y);
            this.ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.7)`;
            this.ctx.lineWidth = 1;
            this.ctx.stroke();
          }
          for (const node of sk.nodes) {
            this.ctx.beginPath();
            this.ctx.arc(node.x, node.y, 2.5, 0, Math.PI * 2);
            this.ctx.fillStyle = state.layers.faceColors
              ? `rgba(${r}, ${g}, ${b}, 0.6)`
              : "rgba(255, 221, 87, 0.85)";
            this.ctx.fill();
          }
          const splitColor = state.layers.faceColors
            ? `rgba(${r}, ${g}, ${b}, 0.9)`
            : "rgba(255, 100, 200, 0.9)";
          for (const node of sk.split_nodes ?? []) {
            const s = 3.5;
            this.ctx.fillStyle = splitColor;
            this.ctx.fillRect(node.x - s, node.y - s, s * 2, s * 2);
          }
        }
      }

      if (state.layers.aisles) {
        for (const poly of layout.aisle_polygons) {
          this.drawPolygon(poly, "rgba(80, 80, 100, 0.6)", "rgba(60, 60, 80, 0.8)", 0.5);
        }
      }

      if (state.layers.islands && layout.islands) {
        for (const island of layout.islands) {
          let fillR: number, fillG: number, fillB: number;
          if (state.layers.faceColors) {
            const [fr, fg, fb] = Renderer.FACE_COLORS[island.face_idx % Renderer.FACE_COLORS.length];
            [fillR, fillG, fillB] = [Math.round(fr * 0.45), Math.round(fg * 0.45), Math.round(fb * 0.45)];
          } else {
            [fillR, fillG, fillB] = [75, 140, 60];
          }
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
        const highlightExt = state.layers.extensionStalls;
        for (const stall of layout.stalls) {
          this.drawStall(stall, highlightExt);
        }
      }

      // 5b. Paint lines
      if (state.layers.paintLines) {
        this.drawPaintLines(state, lot);
      }

    } // end per-lot loop

    // 6. Vertex network overlay (always called; individual sections
    //    check their own layer flags inside).
    this.drawVertexNetwork(state);

    // 6b. Skeleton source vertices drawn after vertex network.
    if (state.layers.skeletonDebug) {
      for (const lot of state.lots) {
        const skd = lot.layout?.skeleton_debug;
        if (!skd) continue;
        const colors = Renderer.FACE_COLORS;
        for (let si = 0; si < skd.length; si++) {
          const sk = skd[si];
          const [r, g, b] = state.layers.faceColors
            ? colors[si % colors.length]
            : [255, 80, 80];
          const srcColor = `rgba(${r}, ${g}, ${b}, 0.9)`;
          for (const src of sk.sources) {
            this.ctx.beginPath();
            this.ctx.arc(src.x, src.y, 3.5, 0, Math.PI * 2);
            this.ctx.arc(src.x, src.y, 1.5, 0, Math.PI * 2, true);
            this.ctx.fillStyle = srcColor;
            this.ctx.fill();
          }
        }
      }
    }

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
    const isActive = lot.id === state.activeLotId;

    const derivedOuter = lot.layout?.derived_outer ?? boundary.outer;
    ctx.beginPath();
    this.tracePath(derivedOuter);
    ctx.fillStyle = "rgba(40, 40, 60, 0.8)";
    ctx.fill();

    if (state.layers.vertices) {
      ctx.beginPath();
      this.tracePathWithCurves(boundary.outer, boundary.outer_curves);
      ctx.strokeStyle = isActive ? "rgba(233, 69, 96, 0.8)" : "rgba(233, 69, 96, 0.4)";
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
        const holeCurves = boundary.hole_curves?.[hi];
        ctx.beginPath();
        this.tracePathWithCurves(hole, holeCurves);
        ctx.strokeStyle = isActive ? "rgba(233, 69, 96, 0.6)" : "rgba(233, 69, 96, 0.3)";
        ctx.lineWidth = 0.8;
        ctx.stroke();
      }

      // Draw edge midpoint handles and curve control points.
      this.drawEdgeHandles(boundary.outer, boundary.outer_curves, isActive);
      for (let hi = 0; hi < boundary.holes.length; hi++) {
        this.drawEdgeHandles(boundary.holes[hi], boundary.hole_curves?.[hi], isActive);
      }
    }
  }

  private drawEdgeHandles(
    points: Vec2[],
    curves: (EdgeCurve | null)[] | undefined,
    isActive: boolean,
  ): void {
    const { ctx } = this;
    const alpha = isActive ? 0.6 : 0.25;
    const ghostAlpha = isActive ? 0.4 : 0.15;

    for (let i = 0; i < points.length; i++) {
      const p0 = points[i];
      const p1 = points[(i + 1) % points.length];
      const c = curves?.[i];

      if (c) {
        // Handle lines from endpoints to control points.
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(c.cp1.x, c.cp1.y);
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(c.cp2.x, c.cp2.y);
        ctx.strokeStyle = `rgba(255, 160, 50, ${ghostAlpha})`;
        ctx.lineWidth = 0.5;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Ghost control point dots.
        for (const cp of [c.cp1, c.cp2]) {
          ctx.beginPath();
          ctx.arc(cp.x, cp.y, 2, 0, Math.PI * 2);
          ctx.fillStyle = `rgba(255, 160, 50, ${ghostAlpha})`;
          ctx.fill();
          ctx.strokeStyle = `rgba(255, 255, 255, ${ghostAlpha * 0.7})`;
          ctx.lineWidth = 0.5;
          ctx.stroke();
        }

        // Edge midpoint handle (solid, on the curve at t=0.5).
        const mid = {
          x: (p0.x + 3 * c.cp1.x + 3 * c.cp2.x + p1.x) / 8,
          y: (p0.y + 3 * c.cp1.y + 3 * c.cp2.y + p1.y) / 8,
        };
        ctx.beginPath();
        ctx.arc(mid.x, mid.y, 3, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(255, 160, 50, ${alpha})`;
        ctx.fill();
        ctx.strokeStyle = `rgba(255, 255, 255, ${alpha})`;
        ctx.lineWidth = 0.6;
        ctx.stroke();
      } else {
        // Straight edge — ghost midpoint handle.
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

  private drawFace(face: Face, index: number, perFaceColors: boolean): void {
    const { ctx } = this;
    let r: number, g: number, b: number;
    if (perFaceColors) {
      const colors = Renderer.FACE_COLORS;
      [r, g, b] = colors[index % colors.length];
    } else {
      [r, g, b] = [200, 200, 220];
    }
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
        ctx.lineWidth = 2;
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

  private drawStall(stall: StallQuad, highlightExtensions = true): void {
    const { ctx } = this;
    const colors: Record<string, { fill: string; stroke: string }> = {
      Standard: {
        fill: "rgba(200, 200, 220, 0.4)",
        stroke: "rgba(180, 180, 200, 0.7)",
      },
      Compact: {
        fill: "rgba(200, 200, 150, 0.4)",
        stroke: "rgba(180, 180, 130, 0.7)",
      },
      Ev: {
        fill: "rgba(100, 200, 100, 0.4)",
        stroke: "rgba(80, 180, 80, 0.7)",
      },
      Extension: {
        fill: "rgba(255, 160, 80, 0.35)",
        stroke: "rgba(230, 120, 50, 0.65)",
      },
      Island: {
        fill: "rgba(75, 140, 60, 0.5)",
        stroke: "rgba(60, 120, 45, 0.7)",
      },
    };
    const effectiveKind = (stall.kind === "Extension" && !highlightExtensions) ? "Standard" : stall.kind;
    const c = colors[effectiveKind] || colors.Standard;
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

  private drawPaintLines(state: AppState, lot: ParkingLot): void {
    const { ctx } = this;
    ctx.save();
    ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
    ctx.lineWidth = 0.5;
    ctx.lineCap = "round";

    // Stall paint lines: draw three sides, leaving one edge open.
    // corners: [0]=back_left, [1]=back_right, [2]=aisle_right, [3]=aisle_left
    // Regular stalls: [2]→[3] open (aisle/entrance).
    // Island stalls: [0]→[1] open (back, connects to island).
    if (lot.layout) {
      ctx.beginPath();
      for (const stall of lot.layout.stalls) {
        const c = stall.corners;
        if (stall.kind === "Island") {
          ctx.moveTo(c[1].x, c[1].y);
          ctx.lineTo(c[2].x, c[2].y);
          ctx.lineTo(c[3].x, c[3].y);
          ctx.lineTo(c[0].x, c[0].y);
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
        if (stall.kind !== "Island") continue;
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

    // Draw the spine line — dashed for primary, dotted for extensions.
    ctx.beginPath();
    ctx.moveTo(spine.start.x, spine.start.y);
    ctx.lineTo(spine.end.x, spine.end.y);
    ctx.strokeStyle = color;
    ctx.lineWidth = 0.8;
    ctx.setLineDash(spine.is_extension ? [1, 2] : [3, 2]);
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
    const activeLot = state.lots.find((l) => l.id === state.activeLotId);

    const graph: DriveAisleGraph | null =
      activeLot?.aisleGraph ?? activeLot?.layout?.resolved_graph ?? null;
    const isManualGraph = activeLot?.aisleGraph != null;

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
        const isChainSelected = isSelected && !isSegmentSelected;
        // Find the annotation (if any) that applies to this edge.
        const mid = { x: (start.x + end.x) / 2, y: (start.y + end.y) / 2 };
        const matchedAnn = (activeLot?.annotations ?? []).find((a: any) => {
          if (a.kind !== "OneWay" && a.kind !== "TwoWayOriented") return false;
          if (a._active === false) return false;
          const d = Math.sqrt((a.midpoint.x - mid.x) ** 2 + (a.midpoint.y - mid.y) ** 2);
          return d < 5.0;
        });
        const direction = (matchedAnn?.kind ?? edge.direction ?? "TwoWay") as AisleDirection;
        const isOneWay = direction === "OneWay";
        const isTwoWayOriented = direction === "TwoWayOriented";
        const isDirected = isOneWay || isTwoWayOriented;

        // Edge line — segment selection is solid+thick, chain selection is dashed+thick.
        ctx.setLineDash(isSegmentSelected ? [] : isDirected ? [] : [2, 2]);
        const isPerimeter = edge.interior === false;
        ctx.strokeStyle = isSelected
          ? "rgba(255, 220, 50, 0.9)"
          : isDirected
            ? isTwoWayOriented ? "rgba(100, 200, 255, 0.7)" : "rgba(255, 180, 50, 0.7)"
            : isPerimeter
              ? "rgba(255, 100, 100, 0.5)"
              : isManualGraph
                ? "rgba(100, 150, 255, 0.5)"
                : "rgba(100, 150, 255, 0.3)";
        ctx.lineWidth = isSegmentSelected ? 2.5 : isSelected ? 1.5 : isDirected ? 1.0 : 0.5;
        ctx.beginPath();
        ctx.moveTo(start.x, start.y);
        ctx.lineTo(end.x, end.y);
        ctx.stroke();

        // Draw arrows on directed edges. Travel direction comes from
        // the matched legacy annotation's travel_dir, or — for
        // abstract annotations — from the abstract edge endpoints
        // forward-transformed through the region frame. If neither
        // matches (e.g., a chain-extended legacy annotation
        // whose proximity-matched midpoint is on a different edge),
        // skip arrows to preserve existing legacy screenshots.
        if (isDirected) {
          let tnx: number;
          let tny: number;
          const mid = { x: (start.x + end.x) / 2, y: (start.y + end.y) / 2 };
          if (matchedAnn && (matchedAnn.kind === "OneWay" || matchedAnn.kind === "TwoWayOriented")) {
            const td = matchedAnn.travel_dir;
            const tdLen = Math.sqrt(td.x * td.x + td.y * td.y);
            if (tdLen < 1e-9) continue;
            tnx = td.x / tdLen;
            tny = td.y / tdLen;
          } else {
            const abstractMatch = findAbstractDirectionAnnotation(
              activeLot ?? null,
              state.params,
              mid,
            );
            if (!abstractMatch) continue;
            tnx = abstractMatch.tx;
            tny = abstractMatch.ty;
          }
          // Edge vector for positioning along the edge
          const dx = end.x - start.x;
          const dy = end.y - start.y;
          const len = Math.sqrt(dx * dx + dy * dy);
          if (len > 1) {
            const arrowSize = 5;
            const color = isSelected
              ? "rgba(255, 220, 50, 0.9)"
              : isTwoWayOriented ? "rgba(100, 200, 255, 0.8)" : "rgba(255, 180, 50, 0.8)";

            if (isTwoWayOriented) {
              // Two lanes with arrows in opposite directions.
              // Use a CANONICAL edge direction for the perpendicular offset
              // so "side=1" always maps to the same physical side regardless
              // of which variant is active (the resolved graph flips edge
              // start/end between variants).
              const cdx = dx > 0 || (dx === 0 && dy > 0) ? dx : -dx;
              const cdy = dx > 0 || (dx === 0 && dy > 0) ? dy : -dy;
              const enx = cdx / len;
              const eny = cdy / len;
              const perpx = -eny;
              const perpy = enx;
              const offset = 3;
              for (const side of [1, -1] as const) {
                const ox = perpx * offset * side;
                const oy = perpy * offset * side;
                // side=1 (right of edge): travel_dir; side=-1 (left): reverse
                const dirNx = side === 1 ? tnx : -tnx;
                const dirNy = side === 1 ? tny : -tny;
                const sideColor = side === 1 ? "rgba(100, 200, 255, 0.95)" : "rgba(255, 160, 80, 0.95)";
                for (const t of [0.33, 0.66]) {
                  const ax = start.x + dx * t + ox;
                  const ay = start.y + dy * t + oy;
                  ctx.beginPath();
                  ctx.moveTo(ax + dirNx * arrowSize, ay + dirNy * arrowSize);
                  ctx.lineTo(ax - dirNx * arrowSize * 0.5 + dirNy * arrowSize * 0.5, ay - dirNy * arrowSize * 0.5 - dirNx * arrowSize * 0.5);
                  ctx.lineTo(ax - dirNx * arrowSize * 0.5 - dirNy * arrowSize * 0.5, ay - dirNy * arrowSize * 0.5 + dirNx * arrowSize * 0.5);
                  ctx.closePath();
                  ctx.fillStyle = sideColor;
                  ctx.fill();
                }
              }
            } else {
              // Single direction arrows for one-way: use travel_dir.
              for (const t of [0.33, 0.66]) {
                const ax = start.x + dx * t;
                const ay = start.y + dy * t;
                ctx.beginPath();
                ctx.moveTo(ax + tnx * arrowSize, ay + tny * arrowSize);
                ctx.lineTo(ax - tnx * arrowSize * 0.5 + tny * arrowSize * 0.5, ay - tny * arrowSize * 0.5 - tnx * arrowSize * 0.5);
                ctx.lineTo(ax - tnx * arrowSize * 0.5 - tny * arrowSize * 0.5, ay - tny * arrowSize * 0.5 + tnx * arrowSize * 0.5);
                ctx.closePath();
                ctx.fillStyle = color;
                ctx.fill();
              }
            }
          }
        }
      }
      ctx.setLineDash([]);
    }

    // Draw drive line edges (solid green + faint infinite extent)
    const allDriveLines = state.layers.driveLines ? state.lots.flatMap((l) => l.driveLines) : [];
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

    const aisleVerts = (graph?.vertices ?? []).map((v, i) => ({
      pos: v,
      ref: { type: "aisle" as const, index: i },
      color: isManualGraph
        ? "rgba(100, 150, 255, 0.9)"
        : "rgba(100, 150, 255, 0.5)",
    }));

    const driveLineVerts: { pos: Vec2; ref: VertexRef; color: string }[] = [];
    if (state.layers.driveLines) {
      for (const lot of state.lots) {
        lot.driveLines.forEach((dl, i) => {
          driveLineVerts.push({
            pos: dl.start,
            ref: { type: "drive-line", index: i, endpoint: "start", lotId: lot.id },
            color: "#00ff88",
          });
          driveLineVerts.push({
            pos: dl.end,
            ref: { type: "drive-line", index: i, endpoint: "end", lotId: lot.id },
            color: "#00ff88",
          });
        });
      }
    }

    const annotationVerts: { pos: Vec2; ref: VertexRef; color: string }[] = [];
    for (const lot of state.lots) {
      lot.annotations.forEach((ann, i) => {
        let pos: Vec2 | null;
        let isDelete: boolean;
        let isTwoWayOri: boolean;

        if (isAbstractAnnotation(ann)) {
          // Abstract annotations are keyed by integer grid indices.
          // Compute their current world position through the hosting
          // region's AbstractFrame so the marker follows the grid as
          // parameters change.
          pos = abstractAnnotationWorldPos(ann, lot, state.params);
          if (!pos) return;
          isDelete =
            ann.kind === "AbstractDeleteVertex" ||
            ann.kind === "AbstractDeleteEdge";
          isTwoWayOri = ann.kind === "AbstractTwoWayOriented";
        } else {
          pos = ann.kind === "DeleteVertex" ? ann.point : ann.midpoint;
          isDelete = ann.kind === "DeleteVertex" || ann.kind === "DeleteEdge";
          isTwoWayOri = ann.kind === "TwoWayOriented";
        }
        const activeColor = isDelete ? "rgba(255, 80, 80, 0.95)" : isTwoWayOri ? "rgba(100, 200, 255, 0.95)" : "rgba(255, 180, 50, 0.95)";
        const inactiveColor = isDelete ? "rgba(255, 80, 80, 0.3)" : isTwoWayOri ? "rgba(100, 200, 255, 0.3)" : "rgba(255, 180, 50, 0.3)";
        annotationVerts.push({
          pos,
          ref: { type: "annotation", index: i, lotId: lot.id },
          color: ann._active === false ? inactiveColor : activeColor,
        });
      });
    }

    const boundaryVerts: { pos: Vec2; ref: VertexRef; color: string }[] = [];
    if (state.layers.vertices) {
      for (const lot of state.lots) {
        const isActive = lot.id === state.activeLotId;
        lot.boundary.outer.forEach((v, i) => {
          boundaryVerts.push({
            pos: v,
            ref: { type: "boundary-outer", index: i, lotId: lot.id },
            color: isActive ? "rgba(255, 160, 50, 0.9)" : "rgba(255, 160, 50, 0.4)",
          });
        });
        lot.boundary.holes.forEach((hole, hi) => {
          hole.forEach((v, vi) => {
            boundaryVerts.push({
              pos: v,
              ref: { type: "boundary-hole", index: vi, holeIndex: hi, lotId: lot.id },
              color: isActive ? "rgba(255, 120, 50, 0.9)" : "rgba(255, 120, 50, 0.4)",
            });
          });
        });
      }
    }

    const allVerts = [
      ...(state.layers.vertices ? [...boundaryVerts, ...aisleVerts] : []),
      ...driveLineVerts,
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

    for (const lot of state.lots) {
      const rd = lot.layout?.region_debug;
      if (!rd || rd.regions.length === 0) continue;

      for (let i = 0; i < rd.regions.length; i++) {
        const region = rd.regions[i];
        const [r, g, b] = colors[i % colors.length];
        const angleRad = region.aisle_angle_deg * (Math.PI / 180);
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
          const ref: VertexRef = { type: "region-vector", index: i, endpoint: ep.endpoint, lotId: lot.id };
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

  private drawAisleVector(state: AppState): void {
    const { ctx } = this;
    const activeLot = state.lots.find((l) => l.id === state.activeLotId);
    if (!activeLot) return;
    const vec = activeLot.aisleVector;
    const dx = vec.end.x - vec.start.x;
    const dy = vec.end.y - vec.start.y;
    const len = Math.sqrt(dx * dx + dy * dy);

    // Faint infinite extent line
    if (len > 1e-9) {
      const nx = dx / len;
      const ny = dy / len;
      const ext = 10000;
      ctx.beginPath();
      ctx.moveTo(vec.start.x - nx * ext, vec.start.y - ny * ext);
      ctx.lineTo(vec.end.x + nx * ext, vec.end.y + ny * ext);
      ctx.strokeStyle = "rgba(255, 160, 50, 0.2)";
      ctx.lineWidth = 0.5;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Solid control segment
    ctx.beginPath();
    ctx.moveTo(vec.start.x, vec.start.y);
    ctx.lineTo(vec.end.x, vec.end.y);
    ctx.strokeStyle = "rgba(255, 160, 50, 0.9)";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Arrowhead at end
    if (len > 10) {
      const nx = dx / len;
      const ny = dy / len;
      const arrowSize = 7;
      const ax = vec.end.x;
      const ay = vec.end.y;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(ax - nx * arrowSize + ny * arrowSize * 0.4, ay - ny * arrowSize - nx * arrowSize * 0.4);
      ctx.lineTo(ax - nx * arrowSize - ny * arrowSize * 0.4, ay - ny * arrowSize + nx * arrowSize * 0.4);
      ctx.closePath();
      ctx.fillStyle = "rgba(255, 160, 50, 0.9)";
      ctx.fill();
    }

    // Endpoint dots
    const endpoints = [
      { pos: vec.start, ref: { type: "aisle-vector" as const, index: 0, endpoint: "start" as const } },
      { pos: vec.end, ref: { type: "aisle-vector" as const, index: 0, endpoint: "end" as const } },
    ];
    for (const ep of endpoints) {
      const isSelected = this.vertexRefsEqual(ep.ref, state.selectedVertex);
      const isHovered = this.vertexRefsEqual(ep.ref, state.hoveredVertex);
      const r = isSelected || isHovered ? 6 : 4.5;
      ctx.beginPath();
      ctx.arc(ep.pos.x, ep.pos.y, r, 0, Math.PI * 2);
      ctx.fillStyle = isSelected ? "#ffffff" : isHovered ? "#ffff00" : "rgba(255, 160, 50, 0.95)";
      ctx.fill();
      ctx.strokeStyle = "rgba(0, 0, 0, 0.5)";
      ctx.lineWidth = 0.8;
      ctx.stroke();
    }
  }

  private vertexRefsEqual(
    a: VertexRef | null | undefined,
    b: VertexRef | null | undefined,
  ): boolean {
    if (!a || !b) return false;
    if (a.type !== b.type || a.index !== b.index) return false;
    if (a.lotId !== b.lotId) return false;
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

  private tracePathWithCurves(points: Vec2[], curves?: (EdgeCurve | null)[]): void {
    if (points.length === 0) return;
    this.ctx.moveTo(points[0].x, points[0].y);
    for (let i = 0; i < points.length; i++) {
      const next = (i + 1) % points.length;
      const curve = curves?.[i];
      if (curve) {
        this.ctx.bezierCurveTo(
          curve.cp1.x, curve.cp1.y,
          curve.cp2.x, curve.cp2.y,
          points[next].x, points[next].y,
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
