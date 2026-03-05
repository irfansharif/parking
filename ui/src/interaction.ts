import { App, VertexRef } from "./app";
import { Renderer } from "./renderer";
import { Vec2 } from "./types";

export function setupInteraction(
  canvas: HTMLCanvasElement,
  app: App,
  renderer: Renderer,
): void {
  const HIT_RADIUS = 8; // screen pixels
  let isPanning = false;
  let panStart: Vec2 = { x: 0, y: 0 };

  canvas.addEventListener("mousedown", (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // Middle button or Ctrl+left for pan (always available)
    if (e.button === 1 || (e.button === 0 && e.ctrlKey)) {
      isPanning = true;
      panStart = { x: sx, y: sy };
      e.preventDefault();
      return;
    }

    if (e.button !== 0) return;

    const worldPos = renderer.screenToWorld(sx, sy, app.state.camera);
    const mode = app.state.editMode;

    if (mode === "add-aisle-vertex") {
      const idx = app.addAisleVertex(worldPos);
      app.state.selectedVertex = { type: "aisle", index: idx };
      renderer.render(app.state);
      return;
    }

    if (mode === "add-aisle-edge") {
      // Click on an aisle vertex to start/complete an edge.
      const hit = hitTestType(sx, sy, app, renderer, HIT_RADIUS, "aisle");
      if (!hit) return;
      if (!app.state.selectedVertex || app.state.selectedVertex.type !== "aisle") {
        // First vertex selected — highlight it, wait for second click.
        app.state.selectedVertex = hit;
      } else {
        // Second vertex — create edge.
        app.addAisleEdge(app.state.selectedVertex.index, hit.index);
        app.state.selectedVertex = null;
      }
      renderer.render(app.state);
      return;
    }

    if (mode === "add-hole") {
      app.state.pendingHole.push(worldPos);
      renderer.render(app.state);
      return;
    }

    // Default: select mode — hit test and drag.
    const allVerts = app.getAllVertices();
    const perimCount = app.getEffectiveAisleGraph()?.perim_vertex_count ?? 0;
    let closest: { ref: VertexRef; dist: number } | null = null;

    for (const v of allVerts) {
      // Skip perimeter aisle vertices — they're pinned to the boundary inset.
      if (v.ref.type === "aisle" && v.ref.index < perimCount) continue;

      const screenV = renderer.worldToScreen(
        v.pos.x,
        v.pos.y,
        app.state.camera,
      );
      const dx = screenV.x - sx;
      const dy = screenV.y - sy;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < HIT_RADIUS && (!closest || dist < closest.dist)) {
        closest = { ref: v.ref, dist };
      }
    }

    if (closest) {
      app.state.selectedVertex = closest.ref;
      app.state.isDragging = true;
    } else {
      app.state.selectedVertex = null;
    }
    renderer.render(app.state);
  });

  canvas.addEventListener("mousemove", (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    if (isPanning) {
      app.state.camera.offsetX += sx - panStart.x;
      app.state.camera.offsetY += sy - panStart.y;
      panStart = { x: sx, y: sy };
      renderer.render(app.state);
      return;
    }

    if (app.state.isDragging && app.state.selectedVertex) {
      const worldPos = renderer.screenToWorld(sx, sy, app.state.camera);
      app.moveVertex(app.state.selectedVertex, worldPos);
      return;
    }

    // Hover detection (all modes)
    const allVerts = app.getAllVertices();
    let hovered: VertexRef | null = null;
    for (const v of allVerts) {
      const screenV = renderer.worldToScreen(
        v.pos.x,
        v.pos.y,
        app.state.camera,
      );
      const dx = screenV.x - sx;
      const dy = screenV.y - sy;
      if (Math.sqrt(dx * dx + dy * dy) < HIT_RADIUS) {
        hovered = v.ref;
        break;
      }
    }

    if (hovered !== app.state.hoveredVertex) {
      app.state.hoveredVertex = hovered;
      const mode = app.state.editMode;
      if (mode === "add-aisle-vertex" || mode === "add-hole") {
        canvas.style.cursor = "crosshair";
      } else if (mode === "add-aisle-edge") {
        canvas.style.cursor = hovered?.type === "aisle" ? "pointer" : "crosshair";
      } else {
        canvas.style.cursor = hovered ? "pointer" : "default";
      }
      renderer.render(app.state);
    }
  });

  canvas.addEventListener("mouseup", () => {
    if (isPanning) {
      isPanning = false;
      return;
    }
    // If we were dragging an aisle vertex, clear the manual graph and anchor
    // so all aisles regenerate cleanly at the final aisle_angle_deg.
    if (app.state.isDragging && app.state.selectedVertex?.type === "aisle") {
      app.state.aisleGraph = null;
      app.state.dragAnchor = null;
      app.generate();
    }
    app.state.isDragging = false;
  });

  canvas.addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;

      const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
      const oldZoom = app.state.camera.zoom;
      const newZoom = Math.max(0.5, Math.min(20, oldZoom * zoomFactor));

      app.state.camera.offsetX =
        sx - (sx - app.state.camera.offsetX) * (newZoom / oldZoom);
      app.state.camera.offsetY =
        sy - (sy - app.state.camera.offsetY) * (newZoom / oldZoom);
      app.state.camera.zoom = newZoom;

      renderer.render(app.state);
    },
    { passive: false },
  );

  // Double-click to insert vertex on nearest edge
  canvas.addEventListener("dblclick", (e) => {
    if (app.state.editMode !== "select") return;

    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const worldPos = renderer.screenToWorld(sx, sy, app.state.camera);

    // Check boundary outer edges
    const outer = app.state.boundary.outer;
    let bestDist = Infinity;
    let bestIdx = -1;
    let bestTarget: "outer" | "hole" = "outer";
    let bestHoleIdx = -1;

    for (let i = 0; i < outer.length; i++) {
      const a = outer[i];
      const b = outer[(i + 1) % outer.length];
      const dist = pointToSegmentDist(worldPos, a, b);
      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = i;
        bestTarget = "outer";
      }
    }

    // Check hole edges too
    for (let hi = 0; hi < app.state.boundary.holes.length; hi++) {
      const hole = app.state.boundary.holes[hi];
      for (let i = 0; i < hole.length; i++) {
        const a = hole[i];
        const b = hole[(i + 1) % hole.length];
        const dist = pointToSegmentDist(worldPos, a, b);
        if (dist < bestDist) {
          bestDist = dist;
          bestIdx = i;
          bestTarget = "hole";
          bestHoleIdx = hi;
        }
      }
    }

    const threshold = 10;
    if (bestDist < threshold) {
      if (bestTarget === "outer") {
        app.insertBoundaryVertex(bestIdx, worldPos);
      } else {
        app.insertHoleVertex(bestHoleIdx, bestIdx, worldPos);
      }
    }
  });

  // Right-click to delete vertex (boundary outer, hole, or aisle)
  canvas.addEventListener("contextmenu", (e) => {
    e.preventDefault();
    if (app.state.editMode !== "select") return;

    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    const allVerts2 = app.getAllVertices();
    const perimCount2 = app.getEffectiveAisleGraph()?.perim_vertex_count ?? 0;
    for (const v of allVerts2) {
      if (v.ref.type === "aisle" && v.ref.index < perimCount2) continue;
      const screenV = renderer.worldToScreen(
        v.pos.x,
        v.pos.y,
        app.state.camera,
      );
      const dx = screenV.x - sx;
      const dy = screenV.y - sy;
      if (Math.sqrt(dx * dx + dy * dy) < HIT_RADIUS) {
        if (v.ref.type === "boundary-outer") {
          app.deleteBoundaryVertex(v.ref.index);
        } else if (v.ref.type === "boundary-hole" && v.ref.holeIndex !== undefined) {
          app.deleteHoleVertex(v.ref.holeIndex, v.ref.index);
        } else if (v.ref.type === "aisle") {
          app.deleteAisleVertex(v.ref.index);
        }
        renderer.render(app.state);
        break;
      }
    }
  });

  // Keyboard shortcuts
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      if (app.state.editMode === "add-hole" && app.state.pendingHole.length > 0) {
        app.commitPendingHole();
      }
      app.state.editMode = "select";
      app.state.selectedVertex = null;
      app.state.pendingHole = [];
      canvas.style.cursor = "default";
      updateModeHint(app);
      renderer.render(app.state);
    } else if (e.key === "Delete" || e.key === "Backspace") {
      const sel = app.state.selectedVertex;
      if (!sel) return;
      if (sel.type === "boundary-outer") {
        app.deleteBoundaryVertex(sel.index);
      } else if (sel.type === "boundary-hole" && sel.holeIndex !== undefined) {
        app.deleteHoleVertex(sel.holeIndex, sel.index);
      } else if (sel.type === "aisle") {
        app.deleteAisleVertex(sel.index);
      }
      renderer.render(app.state);
    }
  });
}

function hitTestType(
  sx: number,
  sy: number,
  app: App,
  renderer: Renderer,
  hitRadius: number,
  type: string,
): VertexRef | null {
  const allVerts = app.getAllVertices();
  for (const v of allVerts) {
    if (v.ref.type !== type) continue;
    const screenV = renderer.worldToScreen(v.pos.x, v.pos.y, app.state.camera);
    const dx = screenV.x - sx;
    const dy = screenV.y - sy;
    if (Math.sqrt(dx * dx + dy * dy) < hitRadius) {
      return v.ref;
    }
  }
  return null;
}

function pointToSegmentDist(p: Vec2, a: Vec2, b: Vec2): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0)
    return Math.sqrt((p.x - a.x) ** 2 + (p.y - a.y) ** 2);
  let t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const projX = a.x + t * dx;
  const projY = a.y + t * dy;
  return Math.sqrt((p.x - projX) ** 2 + (p.y - projY) ** 2);
}

export function updateModeHint(app: App): void {
  const hint = document.getElementById("mode-hint");
  if (!hint) return;
  const hints: Record<string, string> = {
    select: "Click to select/drag vertices. Double-click edge to insert vertex. Right-click vertex to delete. Delete key removes selected vertex.",
    "add-aisle-vertex": "Click to place aisle vertices. Press Esc to return to select mode.",
    "add-aisle-edge": "Click two aisle vertices to connect them. Press Esc to cancel.",
    "add-hole": "Click to place hole vertices. Press Esc to finish the hole (needs 3+ vertices).",
  };
  hint.textContent = hints[app.state.editMode] ?? "";
}
