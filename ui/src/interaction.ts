import { App, VertexRef, EdgeRef } from "./app";
import { Renderer } from "./renderer";
import { Vec2, DriveAisleGraph } from "./types";
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
  let lastEdgeClickTime = 0;
  let lastEdgeClickIdx = -1;

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

    // Clean up tombstone annotations when selection moves away.
    const wasAnnotation = app.state.selectedVertex?.type === "annotation";
    const stayingOnAnnotation = wasAnnotation && closest?.ref.type === "annotation";
    if (wasAnnotation && !stayingOnAnnotation) {
      app.cleanupTombstones();
    }

    if (closest) {
      app.state.selectedVertex = closest.ref;
      app.state.selectedEdge = null;
      // Aisle vertices are fixed — don't start dragging.
      app.state.isDragging = closest.ref.type !== "aisle";
      // Sync: when clicking an annotation, also select its edge.
      if (closest.ref.type === "annotation") {
        const ann = app.state.annotations[closest.ref.index];
        if (ann && ann.kind !== "DeleteVertex") {
          const pt = ann.midpoint;
          const graph = app.getEffectiveAisleGraph();
          if (graph) {
            const seen = new Set<string>();
            let bestIdx = -1;
            let bestDist = Infinity;
            for (let i = 0; i < graph.edges.length; i++) {
              const e = graph.edges[i];
              const key = Math.min(e.start, e.end) + "," + Math.max(e.start, e.end);
              if (seen.has(key)) continue;
              seen.add(key);
              const s = graph.vertices[e.start];
              const end = graph.vertices[e.end];
              const dist = pointToSegmentDist(pt, s, end);
              if (dist < bestDist) { bestDist = dist; bestIdx = i; }
            }
            if (bestIdx >= 0 && bestDist < 5) {
              app.state.selectedEdge = { index: bestIdx, chain: findCollinearChain(graph, bestIdx), mode: "chain" };
            }
          }
        }
      }
    } else {
      app.state.selectedVertex = null;
      // No vertex hit — try edge hit test.
      const edgeHit = hitTestEdge(worldPos, app, EDGE_HIT_RADIUS / app.state.camera.zoom);
      if (edgeHit) {
        // Double-click (within 400ms) on same chain → narrow to segment.
        const now = Date.now();
        if (app.state.selectedEdge?.chain.includes(edgeHit.index)
            && app.state.selectedEdge.mode === "chain"
            && edgeHit.index === lastEdgeClickIdx
            && now - lastEdgeClickTime < 400) {
          edgeHit.mode = "segment";
          edgeHit.chain = [edgeHit.index];
        }
        lastEdgeClickTime = now;
        lastEdgeClickIdx = edgeHit.index;
        app.state.selectedEdge = edgeHit;
        // Sync: also select annotation on this edge if one exists.
        const graph = app.getEffectiveAisleGraph();
        if (graph) {
          const edge = graph.edges[edgeHit.index];
          if (edge) {
            const s = graph.vertices[edge.start];
            const e = graph.vertices[edge.end];
            const mid = { x: (s.x + e.x) / 2, y: (s.y + e.y) / 2 };
            const annIdx = app.state.annotations.findIndex((a) => {
              const ap = a.kind === "DeleteVertex" ? a.point : a.midpoint;
              return Math.sqrt((ap.x - mid.x) ** 2 + (ap.y - mid.y) ** 2) < 5;
            });
            if (annIdx >= 0) {
              app.state.selectedVertex = { type: "annotation", index: annIdx };
            }
          }
        }
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
      if (mode === "add-hole" || mode === "add-drive-line") {
        canvas.style.cursor = "crosshair";
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
      app.cleanupTombstones();
      app.state.selectedVertex = null;
      app.state.selectedEdge = null;
      app.state.pendingHole = [];
      app.state.pendingDriveLine = null;
      app.state.pendingDriveLinePreview = null;
      canvas.style.cursor = "default";
      updateModeHint(app);
      renderer.render(app.state);
    } else if (e.key === "f" || e.key === "F") {
      // Cycle direction on selected aisle graph edge or annotation anchor.
      if (app.state.selectedEdge) {
        app.cycleEdgeDirection(app.state.selectedEdge.index);
        renderer.render(app.state);
      } else if (app.state.selectedVertex?.type === "annotation") {
        app.cycleAnnotationDirection(app.state.selectedVertex.index);
        renderer.render(app.state);
      }
    } else if (e.key === "Delete" || e.key === "Backspace" || e.key === "d" || e.key === "D") {
      // Delete selected edge via annotation.
      if (app.state.selectedEdge) {
        app.deleteSelectedEdge();
        renderer.render(app.state);
        return;
      }
      const sel = app.state.selectedVertex;
      if (!sel) return;
      if (sel.type === "annotation") {
        app.deleteAnnotation(sel.index);
      } else if (sel.type === "boundary-outer") {
        app.deleteBoundaryVertex(sel.index);
      } else if (sel.type === "boundary-hole" && sel.holeIndex !== undefined) {
        app.deleteHoleVertex(sel.holeIndex, sel.index);
      } else if (sel.type === "aisle") {
        // Create a DeleteVertex annotation instead of mutating the graph.
        app.deleteAisleVertexByAnnotation(sel.index);
      } else if (sel.type === "drive-line") {
        app.deleteDriveLine(sel.index);
      }
      renderer.render(app.state);
    }
  });
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

  if (!best) return null;
  return { index: best.index, chain: findCollinearChain(graph, best.index), mode: "chain" };
}

/// Find all deduplicated edge indices in the same collinear chain as the seed.
export function findCollinearChain(graph: DriveAisleGraph, seedIdx: number): number[] {
  const seed = graph.edges[seedIdx];
  const s = graph.vertices[seed.start];
  const e = graph.vertices[seed.end];
  const dx = e.x - s.x, dy = e.y - s.y;
  const len = Math.sqrt(dx * dx + dy * dy);
  if (len < 1e-9) return [seedIdx];
  const seedDir = { x: dx / len, y: dy / len };

  // Build adjacency: vertex → [(edgeIdx, otherVertex, dir)]
  // Deduplicate bidirectional edges.
  const adj = new Map<number, { ei: number; other: number; dx: number; dy: number }[]>();
  const seen = new Set<string>();
  for (let i = 0; i < graph.edges.length; i++) {
    const edge = graph.edges[i];
    const key = Math.min(edge.start, edge.end) + "," + Math.max(edge.start, edge.end);
    if (seen.has(key)) continue;
    seen.add(key);
    const a = graph.vertices[edge.start];
    const b = graph.vertices[edge.end];
    const edx = b.x - a.x, edy = b.y - a.y;
    const elen = Math.sqrt(edx * edx + edy * edy);
    if (elen < 1e-9) continue;
    const ndx = edx / elen, ndy = edy / elen;
    if (!adj.has(edge.start)) adj.set(edge.start, []);
    if (!adj.has(edge.end)) adj.set(edge.end, []);
    adj.get(edge.start)!.push({ ei: i, other: edge.end, dx: ndx, dy: ndy });
    adj.get(edge.end)!.push({ ei: i, other: edge.start, dx: -ndx, dy: -ndy });
  }

  // BFS along collinear edges.
  const chain = [seedIdx];
  const visited = new Set<number>();
  visited.add(seedIdx);
  const frontier = [seed.start, seed.end];
  while (frontier.length > 0) {
    const v = frontier.pop()!;
    for (const n of adj.get(v) ?? []) {
      if (visited.has(n.ei)) continue;
      const dot = Math.abs(n.dx * seedDir.x + n.dy * seedDir.y);
      if (dot < 0.99) continue;
      visited.add(n.ei);
      chain.push(n.ei);
      frontier.push(n.other);
    }
  }
  return chain;
}

export function updateModeHint(app: App): void {
  const hint = document.getElementById("mode-hint");
  if (!hint) return;
  if (app.state.editMode === "select") {
    if (app.state.selectedEdge) {
      hint.textContent = "Edge selected. F to cycle direction (two-way-A → two-way-B → one-way → reverse → two-way). Esc to deselect.";
    } else if (app.state.selectedVertex?.type === "annotation") {
      hint.textContent = "Annotation selected. F to cycle direction. Delete to remove.";
    } else if (app.state.selectedVertex) {
      hint.textContent = "Drag to move. Delete to remove. Right-drag to pan.";
    } else {
      hint.textContent = "Click vertices to drag, or click aisle edges to select. Right-drag to pan.";
    }
    return;
  }
  const hints: Record<string, string> = {
    "add-hole": "Click to place hole vertices. Press Esc to finish the hole (needs 3+ vertices).",
    "add-drive-line": "Click to place start point, click again to place end point. Press Esc to cancel.",
  };
  hint.textContent = hints[app.state.editMode] ?? "";
}
