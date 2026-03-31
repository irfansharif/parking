import { App, VertexRef, EdgeRef } from "./app";
import { Renderer } from "./renderer";
import { Vec2 } from "./types";
import { computeSnap, emptySnapState } from "./snap";

export function setupInteraction(
  canvas: HTMLCanvasElement,
  app: App,
  renderer: Renderer,
): void {
  const HIT_RADIUS = 8; // screen pixels
  const EDGE_HIT_RADIUS = 6; // screen pixels for edge hit testing
  let isPanning = false;
  let panStart: Vec2 = { x: 0, y: 0 };

  canvas.addEventListener("mousedown", (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // Right-click to pan.
    if (e.button === 2) {
      e.preventDefault();
      isPanning = true;
      panStart = { x: sx, y: sy };
      return;
    }

    if (e.button !== 0) return;

    const worldPos = renderer.screenToWorld(sx, sy, app.state.camera);
    const mode = app.state.editMode;

    if (mode === "add-aisle-vertex") {
      const { pos } = computeSnap(worldPos, app.getAllVertices(), null, app.state.camera.zoom, emptySnapState());
      const idx = app.addAisleVertex(pos);
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
      const { pos } = computeSnap(worldPos, app.getAllVertices(), null, app.state.camera.zoom, emptySnapState());
      app.state.pendingHole.push(pos);
      renderer.render(app.state);
      return;
    }

    if (mode === "add-drive-line") {
      const { pos } = computeSnap(worldPos, app.getAllVertices(), null, app.state.camera.zoom, emptySnapState());
      if (app.state.pendingDriveLine === null) {
        app.state.pendingDriveLine = pos;
      } else {
        app.addDriveLine(app.state.pendingDriveLine, pos);
        app.state.pendingDriveLine = null;
        app.state.pendingDriveLinePreview = null;
      }
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
      app.state.selectedEdge = null;
      app.state.isDragging = true;
    } else {
      app.state.selectedVertex = null;
      // No vertex hit — try edge hit test.
      const edgeHit = hitTestEdge(worldPos, app, EDGE_HIT_RADIUS / app.state.camera.zoom);
      if (edgeHit) {
        app.state.selectedEdge = edgeHit;
      } else {
        app.state.selectedEdge = null;
        // No vertex or edge hit — start canvas pan.
        isPanning = true;
        panStart = { x: sx, y: sy };
      }
    }
    updateModeHint(app);
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
      const rawPos = renderer.screenToWorld(sx, sy, app.state.camera);
      const { pos, guides, nextSnapState } = computeSnap(rawPos, app.getAllVertices(), app.state.selectedVertex, app.state.camera.zoom, app.state.snapState);
      app.state.snapGuides = guides;
      app.state.snapState = nextSnapState;
      app.moveVertex(app.state.selectedVertex, pos);
      return;
    }

    // Pending drive line preview
    if (app.state.editMode === "add-drive-line" && app.state.pendingDriveLine) {
      const rawPos = renderer.screenToWorld(sx, sy, app.state.camera);
      const { pos } = computeSnap(rawPos, app.getAllVertices(), null, app.state.camera.zoom, emptySnapState());
      app.state.pendingDriveLinePreview = pos;
      renderer.render(app.state);
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
      if (mode === "add-aisle-vertex" || mode === "add-hole" || mode === "add-drive-line") {
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
    app.state.snapGuides = [];
    app.state.snapState = emptySnapState();
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

  // Smooth zoom: accumulate wheel deltas and animate towards the target zoom.
  let targetZoom = app.state.camera.zoom;
  let zoomAnchor: Vec2 = { x: 0, y: 0 };
  let zoomRAF: number | null = null;

  function animateZoom() {
    const cam = app.state.camera;
    const diff = targetZoom - cam.zoom;
    if (Math.abs(diff) < 0.001) {
      cam.zoom = targetZoom;
      zoomRAF = null;
    } else {
      const newZoom = cam.zoom + diff * 0.25;
      cam.offsetX = zoomAnchor.x - (zoomAnchor.x - cam.offsetX) * (newZoom / cam.zoom);
      cam.offsetY = zoomAnchor.y - (zoomAnchor.y - cam.offsetY) * (newZoom / cam.zoom);
      cam.zoom = newZoom;
      zoomRAF = requestAnimationFrame(animateZoom);
    }
    renderer.render(app.state);
  }

  canvas.addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      zoomAnchor = { x: e.clientX - rect.left, y: e.clientY - rect.top };

      // Scale proportionally to delta for smooth trackpad support.
      const delta = -e.deltaY * 0.002;
      targetZoom = Math.max(0.5, Math.min(20, targetZoom * (1 + delta)));

      if (!zoomRAF) {
        zoomRAF = requestAnimationFrame(animateZoom);
      }
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

  // Suppress browser context menu on canvas.
  canvas.addEventListener("contextmenu", (e) => {
    e.preventDefault();
  });

  // Keyboard shortcuts
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      if (app.state.editMode === "add-hole" && app.state.pendingHole.length > 0) {
        app.commitPendingHole();
      }
      app.state.editMode = "select";
      app.state.selectedVertex = null;
      app.state.selectedEdge = null;
      app.state.pendingHole = [];
      app.state.pendingDriveLine = null;
      app.state.pendingDriveLinePreview = null;
      canvas.style.cursor = "default";
      updateModeHint(app);
      renderer.render(app.state);
    } else if (e.key === "f" || e.key === "F") {
      // Cycle direction on selected edge or drive line.
      if (app.state.selectedEdge) {
        app.cycleEdgeDirection(app.state.selectedEdge.index);
        renderer.render(app.state);
      } else {
        const sel = app.state.selectedVertex;
        if (sel?.type === "drive-line") {
          app.cycleDriveLineDirection(sel.index);
          renderer.render(app.state);
        }
      }
    } else if (e.key === "Delete" || e.key === "Backspace") {
      const sel = app.state.selectedVertex;
      if (!sel) return;
      if (sel.type === "boundary-outer") {
        app.deleteBoundaryVertex(sel.index);
      } else if (sel.type === "boundary-hole" && sel.holeIndex !== undefined) {
        app.deleteHoleVertex(sel.holeIndex, sel.index);
      } else if (sel.type === "aisle") {
        app.deleteAisleVertex(sel.index);
      } else if (sel.type === "drive-line") {
        app.deleteDriveLine(sel.index);
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

/// Hit-test against aisle graph edges in world space.
function hitTestEdge(worldPos: Vec2, app: App, worldRadius: number): EdgeRef | null {
  const graph = app.getEffectiveAisleGraph();
  if (!graph) return null;

  // Deduplicate bidirectional edges so clicking picks a canonical one.
  const seen = new Set<string>();
  let best: { index: number; dist: number } | null = null;

  for (let i = 0; i < graph.edges.length; i++) {
    const edge = graph.edges[i];
    const key = Math.min(edge.start, edge.end) + "," + Math.max(edge.start, edge.end);
    if (seen.has(key)) continue;
    seen.add(key);

    const a = graph.vertices[edge.start];
    const b = graph.vertices[edge.end];
    const dist = pointToSegmentDist(worldPos, a, b);
    if (dist < worldRadius && (!best || dist < best.dist)) {
      best = { index: i, dist };
    }
  }

  return best ? { index: best.index } : null;
}

export function updateModeHint(app: App): void {
  const hint = document.getElementById("mode-hint");
  if (!hint) return;
  if (app.state.editMode === "select") {
    if (app.state.selectedEdge) {
      hint.textContent = "Edge selected. F to cycle direction (two-way → one-way → reverse → two-way). Esc to deselect.";
    } else if (app.state.selectedVertex) {
      hint.textContent = "Drag to move. Delete to remove. Right-drag to pan.";
    } else {
      hint.textContent = "Click vertices to drag, or click aisle edges to select. Right-drag to pan.";
    }
    return;
  }
  const hints: Record<string, string> = {
    "add-aisle-vertex": "Click to place aisle vertices. Press Esc to return to select mode.",
    "add-aisle-edge": "Click two aisle vertices to connect them. Press Esc to cancel.",
    "add-hole": "Click to place hole vertices. Press Esc to finish the hole (needs 3+ vertices).",
    "add-drive-line": "Click to place start point, click again to place end point. Press Esc to cancel.",
  };
  hint.textContent = hints[app.state.editMode] ?? "";
}
