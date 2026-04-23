import { App, VertexRef, EdgeRef } from "./app";
import { Renderer } from "./renderer";
import { Vec2, EdgeArc, annotationWorldPos, projectToArc } from "./types";
import { computeSnap, emptySnapState } from "./snap";
import {
  find_collinear_chain_js,
  hit_test_edge_js,
  point_to_segment_dist_js,
} from "./wasm/parking_lot_engine";

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

    if (mode === "add-boundary") {
      const { pos } = computeSnap(worldPos, app.getAllVertices(), null, app.state.camera.zoom, emptySnapState());
      app.state.pendingBoundary.push(pos);
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

    // Clean up tombstone annotations when selection moves away.
    const wasAnnotation = app.state.selectedVertex?.type === "annotation";
    const stayingOnAnnotation = wasAnnotation && closest?.ref.type === "annotation";
    if (wasAnnotation && !stayingOnAnnotation) {
      app.cleanupTombstones();
    }

    if (closest) {
      // Switch active lot to the one owning this vertex.
      if (closest.ref.lotId) {
        app.state.activeLotId = closest.ref.lotId;
      }
      app.state.selectedVertex = closest.ref;
      app.state.selectedEdge = null;
      app.state.isDragging = closest.ref.type !== "aisle";
      // Annotations no longer carry world-space midpoints (they're
      // anchored to abstract grid coords or drive-line splice
      // positions); selecting one no longer infers a graph edge.
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
        const activeLot = app.activeLot();
        const graph = app.getEffectiveAisleGraph(activeLot);
        if (graph) {
          const edge = graph.edges[edgeHit.index];
          if (edge) {
            const s = graph.vertices[edge.start];
            const e = graph.vertices[edge.end];
            const mid = { x: (s.x + e.x) / 2, y: (s.y + e.y) / 2 };
            const annIdx = activeLot.annotations.findIndex((a) => {
              const ap = annotationWorldPos(a, activeLot, app.state.params);
              if (!ap) return false;
              return Math.sqrt((ap.x - mid.x) ** 2 + (ap.y - mid.y) ** 2) < 5;
            });
            if (annIdx >= 0) {
              app.state.selectedVertex = { type: "annotation", index: annIdx, lotId: activeLot.id };
            }
          }
        }
      } else {
        // No aisle edge — try region vector body hit test.
        const bodyHit = hitTestRegionVectorBody(worldPos, app, EDGE_HIT_RADIUS / app.state.camera.zoom);
        if (bodyHit) {
          app.state.selectedVertex = bodyHit;
          app.state.selectedEdge = null;
          app.state.isDragging = true;
        } else {
          app.state.selectedEdge = null;
          // No vertex or edge hit — start canvas pan.
          isPanning = true;
          panStart = { x: sx, y: sy };
        }
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

    let bestDist = Infinity;
    let bestIdx = -1;
    let bestTarget: "outer" | "hole" = "outer";
    let bestHoleIdx = -1;
    let bestLotId = "";

    for (const lot of app.state.lots) {
      const outer = lot.boundary.outer;
      for (let i = 0; i < outer.length; i++) {
        const a = outer[i];
        const b = outer[(i + 1) % outer.length];
        const arc = lot.boundary.outer_arcs?.[i];
        const dist = arc
          ? pointToArcDist(worldPos, a, arc, b)
          : pointToSegmentDist(worldPos, a, b);
        if (dist < bestDist) {
          bestDist = dist;
          bestIdx = i;
          bestTarget = "outer";
          bestLotId = lot.id;
        }
      }
      for (let hi = 0; hi < lot.boundary.holes.length; hi++) {
        const hole = lot.boundary.holes[hi];
        for (let i = 0; i < hole.length; i++) {
          const a = hole[i];
          const b = hole[(i + 1) % hole.length];
          const arc = lot.boundary.hole_arcs?.[hi]?.[i];
          const dist = arc
            ? pointToArcDist(worldPos, a, arc, b)
            : pointToSegmentDist(worldPos, a, b);
          if (dist < bestDist) {
            bestDist = dist;
            bestIdx = i;
            bestTarget = "hole";
            bestHoleIdx = hi;
            bestLotId = lot.id;
          }
        }
      }
    }

    const threshold = 10;
    if (bestDist < threshold) {
      app.state.activeLotId = bestLotId;
      if (bestTarget === "outer") {
        app.insertBoundaryVertex(bestIdx, worldPos, bestLotId);
      } else {
        app.insertHoleVertex(bestHoleIdx, bestIdx, worldPos, bestLotId);
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
      if (app.state.editMode === "add-boundary" && app.state.pendingBoundary.length > 0) {
        app.commitPendingBoundary();
      }
      app.state.editMode = "select";
      app.cleanupTombstones();
      app.state.selectedVertex = null;
      app.state.selectedEdge = null;
      app.state.pendingHole = [];
      app.state.pendingBoundary = [];
      app.state.pendingDriveLine = null;
      app.state.pendingDriveLinePreview = null;
      canvas.style.cursor = "default";
      updateModeHint(app);
      renderer.render(app.state);
    } else if (e.key === "f" || e.key === "F") {
      // Toggle region projection on selected hole vertex, or cycle
      // direction on selected aisle graph edge / annotation anchor.
      const sel = app.state.selectedVertex;
      if (app.state.selectedEdge) {
        app.cycleEdgeDirection(app.state.selectedEdge);
        renderer.render(app.state);
      } else if (sel?.type === "drive-line") {
        app.toggleDriveLinePartitions(sel.index, sel.lotId);
        renderer.render(app.state);
      } else if (sel?.type === "boundary-hole" && sel.holeIndex !== undefined) {
        app.toggleSeparator(sel.holeIndex, sel.index);
        renderer.render(app.state);
      } else if (sel?.type === "annotation") {
        app.cycleAnnotationDirection(sel.index);
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
        app.deleteAnnotation(sel.index, sel.lotId);
      } else if (sel.type === "boundary-outer") {
        app.deleteBoundaryVertex(sel.index, sel.lotId);
      } else if (sel.type === "boundary-hole" && sel.holeIndex !== undefined) {
        app.deleteHoleVertex(sel.holeIndex, sel.index, sel.lotId);
      } else if (sel.type === "aisle") {
        app.deleteAisleVertexByAnnotation(sel.index, sel.lotId);
      } else if (sel.type === "drive-line") {
        app.deleteDriveLine(sel.index, sel.lotId);
      }
      renderer.render(app.state);
    }
  });
}

function pointToSegmentDist(p: Vec2, a: Vec2, b: Vec2): number {
  return point_to_segment_dist_js(p.x, p.y, a.x, a.y, b.x, b.y);
}

function pointToArcDist(p: Vec2, a: Vec2, arc: EdgeArc, b: Vec2): number {
  const { pos } = projectToArc(a, b, arc.bulge, p);
  return Math.sqrt((p.x - pos.x) ** 2 + (p.y - pos.y) ** 2);
}

/// Hit-test against aisle graph edges in world space. Returns the seed
/// edge + its collinear chain for chain-mode selection.
function hitTestEdge(worldPos: Vec2, app: App, worldRadius: number): EdgeRef | null {
  const graph = app.getEffectiveAisleGraph();
  if (!graph) return null;
  const seed = hit_test_edge_js(worldPos.x, worldPos.y, graph, worldRadius) as number | null;
  if (seed == null) return null;
  // `find_collinear_chain_js` returns a `Uint32Array`; copy to a plain
  // `number[]` so downstream chain math (splice/slice/concat) works
  // without TypedArray gotchas.
  const chain = Array.from(find_collinear_chain_js(graph, seed));
  return { index: seed, chain, mode: "chain" };
}

/// Find all deduplicated edge indices in the same collinear chain as the seed.
export function findCollinearChain(
  graph: import("./types").DriveAisleGraph,
  seedIdx: number,
): number[] {
  return Array.from(find_collinear_chain_js(graph, seedIdx));
}

/// Hit-test against region vector line segments (body, not endpoints).
function hitTestRegionVectorBody(worldPos: Vec2, app: App, worldRadius: number): VertexRef | null {
  const halfLen = 30;
  let best: { ref: VertexRef; dist: number } | null = null;

  for (const lot of app.state.lots) {
    const rd = lot.layout?.region_debug;
    if (!rd) continue;

    for (let i = 0; i < rd.regions.length; i++) {
      const region = rd.regions[i];
      const angleRad = region.aisle_angle_deg * (Math.PI / 180);
      const dirX = Math.cos(angleRad);
      const dirY = Math.sin(angleRad);
      const cx = region.center.x;
      const cy = region.center.y;
      const sx = cx - dirX * halfLen;
      const sy = cy - dirY * halfLen;
      const ex = cx + dirX * halfLen;
      const ey = cy + dirY * halfLen;

      const dist = pointToSegmentDist(worldPos, { x: sx, y: sy }, { x: ex, y: ey });
      if (dist < worldRadius && (!best || dist < best.dist)) {
        best = { ref: { type: "region-vector", index: i, endpoint: "body", lotId: lot.id }, dist };
      }
    }
  }

  return best?.ref ?? null;
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
    "add-boundary": "Click to place boundary vertices for a new lot. Press Esc to finish (needs 3+ vertices).",
    "add-hole": "Click to place hole vertices. Press Esc to finish the hole (needs 3+ vertices).",
    "add-drive-line": "Click to place start point, click again to place end point. Press Esc to cancel.",
  };
  hint.textContent = hints[app.state.editMode] ?? "";
}
