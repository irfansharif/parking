import { Vec2 } from "./types";
import { VertexRef } from "./app";

export interface SnapGuide {
  axis: "x" | "y";
  worldCoord: number;
}

// Tracks which world coordinates are currently locked as snap targets.
// Null means not currently snapped on that axis.
export interface SnapState {
  xTarget: number | null;
  yTarget: number | null;
}

export interface SnapResult {
  pos: Vec2;
  guides: SnapGuide[];
  nextSnapState: SnapState;
}

export function emptySnapState(): SnapState {
  return { xTarget: null, yTarget: null };
}

// Enter snap within 8px, release only after moving 16px away.
// This hysteresis prevents thrashing when the cursor is near multiple vertices.
const SNAP_SCREEN_PX = 8;
const RELEASE_SCREEN_PX = 16;

function refsEqual(a: VertexRef, b: VertexRef): boolean {
  return a.type === b.type && a.index === b.index && a.holeIndex === b.holeIndex && a.lotId === b.lotId;
}

// Compute snapped position and active guide lines for a cursor at rawPos.
// Snaps X and Y independently. snapState carries hysteresis across calls so
// an active snap is held until the cursor moves RELEASE_SCREEN_PX away.
export function computeSnap(
  rawPos: Vec2,
  allVertices: { ref: VertexRef; pos: Vec2 }[],
  excludeRef: VertexRef | null,
  zoom: number,
  snapState: SnapState,
): SnapResult {
  const snapThreshold = SNAP_SCREEN_PX / zoom;
  const releaseThreshold = RELEASE_SCREEN_PX / zoom;

  let snappedX = rawPos.x;
  let nextXTarget: number | null = null;

  // If already snapped on X, stay snapped until the cursor moves far enough away.
  if (snapState.xTarget !== null) {
    if (Math.abs(rawPos.x - snapState.xTarget) <= releaseThreshold) {
      snappedX = snapState.xTarget;
      nextXTarget = snapState.xTarget;
    }
    // else: fell outside release threshold — look for a new snap below
  }

  // If not (or no longer) snapped on X, find the nearest candidate.
  if (nextXTarget === null) {
    let bestDist = snapThreshold;
    for (const v of allVertices) {
      if (excludeRef && refsEqual(v.ref, excludeRef)) continue;
      const dx = Math.abs(rawPos.x - v.pos.x);
      if (dx < bestDist) {
        bestDist = dx;
        snappedX = v.pos.x;
        nextXTarget = v.pos.x;
      }
    }
  }

  let snappedY = rawPos.y;
  let nextYTarget: number | null = null;

  if (snapState.yTarget !== null) {
    if (Math.abs(rawPos.y - snapState.yTarget) <= releaseThreshold) {
      snappedY = snapState.yTarget;
      nextYTarget = snapState.yTarget;
    }
  }

  if (nextYTarget === null) {
    let bestDist = snapThreshold;
    for (const v of allVertices) {
      if (excludeRef && refsEqual(v.ref, excludeRef)) continue;
      const dy = Math.abs(rawPos.y - v.pos.y);
      if (dy < bestDist) {
        bestDist = dy;
        snappedY = v.pos.y;
        nextYTarget = v.pos.y;
      }
    }
  }

  const guides: SnapGuide[] = [];
  if (nextXTarget !== null) guides.push({ axis: "x", worldCoord: snappedX });
  if (nextYTarget !== null) guides.push({ axis: "y", worldCoord: snappedY });

  return {
    pos: { x: snappedX, y: snappedY },
    guides,
    nextSnapState: { xTarget: nextXTarget, yTarget: nextYTarget },
  };
}
