export interface Vec2 {
  x: number;
  y: number;
}

export interface StallQuad {
  corners: [Vec2, Vec2, Vec2, Vec2];
  kind: "Standard" | "Compact" | "Ev" | "Extension" | "Island";
}

export interface Metrics {
  total_stalls: number;
}

export interface SpineLine {
  start: Vec2;
  end: Vec2;
  normal: Vec2;
  is_extension?: boolean;
}

export interface Face {
  contour: Vec2[];
  holes?: Vec2[][];
  is_boundary?: boolean;
  edge_sources?: string[];
  hole_edge_sources?: string[][];
}

export interface Island {
  contour: Vec2[];
  holes?: Vec2[][];
  face_idx: number;
}

export interface ParkingLayout {
  aisle_polygons: Vec2[][];
  stalls: StallQuad[];
  metrics: Metrics;
  resolved_graph: DriveAisleGraph;
  spines: SpineLine[];
  faces: Face[];
  miter_fills: Vec2[][];
  skeleton_debug: { arcs: [Vec2, Vec2][]; nodes: Vec2[]; split_nodes: Vec2[]; sources: Vec2[] }[];
  islands: Island[];
  derived_outer?: Vec2[];
  derived_holes?: Vec2[][];
  region_debug?: RegionDebug;
}

export interface RegionDebug {
  regions: RegionInfo[];
  separators: [Vec2, Vec2][];
}

export interface RegionInfo {
  clip_poly: Vec2[];
  aisle_angle_deg: number;
  aisle_offset: number;
  center: Vec2;
}

export type AisleDirection = "TwoWay" | "TwoWayOriented" | "OneWay";

export interface AisleEdge {
  start: number;
  end: number;
  width: number;
  interior?: boolean;
  direction?: AisleDirection;
}

export interface DriveAisleGraph {
  vertices: Vec2[];
  edges: AisleEdge[];
  perim_vertex_count?: number;
}

export interface Polygon {
  outer: Vec2[];
  holes: Vec2[][];
}

export interface ParkingParams {
  stall_width: number;
  stall_depth: number;
  aisle_width: number;
  stall_angle_deg: number;
  aisle_angle_deg: number;
  aisle_offset: number;
  site_offset: number;
  cross_aisle_max_run: number;
  use_regions?: boolean;
  island_stall_interval?: number;
}

export interface DebugToggles {
  // Corridor merging
  miter_fills: boolean;
  boundary_only_miters: boolean;
  spike_removal: boolean;
  contour_simplification: boolean;
  hole_filtering: boolean;
  // Face extraction
  face_extraction: boolean;
  // Spine generation
  face_simplification: boolean;
  edge_classification: boolean;
  spine_clipping: boolean;
  // Spine post-processing
  spine_dedup: boolean;
  spine_merging: boolean;
  short_spine_filter: boolean;
  // Spine extensions
  spine_extensions: boolean;
  // Stall placement
  stall_centering: boolean;
  stall_face_clipping: boolean;
  // Boundary
  boundary_clipping: boolean;
  // Conflict removal
  conflict_removal: boolean;
  // Short segment filter
  short_segment_filter: boolean;
  // Edge provenance
  edge_provenance: boolean;
  // Skeleton debug visualization
  skeleton_debug: boolean;
}

export interface DriveLine {
  start: Vec2;
  end: Vec2;
  /** When set, start is pinned to a hole vertex (separator). */
  holePin?: { holeIndex: number; vertexIndex: number };
  /** When set, end is pinned to a boundary edge at parameter t. */
  boundaryPin?: { edgeIndex: number; t: number };
}

/** Project a point onto the nearest outer boundary edge. */
export function computeBoundaryPin(pt: Vec2, outer: Vec2[]): { pos: Vec2; edgeIndex: number; t: number } {
  let bestDist = Infinity;
  let bestProj = pt;
  let bestEdge = 0;
  let bestT = 0;
  for (let i = 0; i < outer.length; i++) {
    const a = outer[i];
    const b = outer[(i + 1) % outer.length];
    const ab = { x: b.x - a.x, y: b.y - a.y };
    const lenSq = ab.x * ab.x + ab.y * ab.y;
    if (lenSq < 1e-12) continue;
    const t = Math.max(0, Math.min(1, ((pt.x - a.x) * ab.x + (pt.y - a.y) * ab.y) / lenSq));
    const proj = { x: a.x + ab.x * t, y: a.y + ab.y * t };
    const dx = pt.x - proj.x, dy = pt.y - proj.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist < bestDist) {
      bestDist = dist;
      bestProj = proj;
      bestEdge = i;
      bestT = t;
    }
  }
  return { pos: bestProj, edgeIndex: bestEdge, t: bestT };
}

export type Annotation = OneWayAnnotation | TwoWayOrientedAnnotation | DeleteVertexAnnotation | DeleteEdgeAnnotation;

export interface OneWayAnnotation {
  kind: "OneWay";
  midpoint: Vec2;
  travel_dir: Vec2;
  chain?: boolean;
  _origDir?: Vec2;
  _active?: boolean; // false = tombstone
}

export interface TwoWayOrientedAnnotation {
  kind: "TwoWayOriented";
  midpoint: Vec2;
  travel_dir: Vec2;
  chain?: boolean;
  _origDir?: Vec2;
  _active?: boolean;
}

export interface DeleteVertexAnnotation {
  kind: "DeleteVertex";
  point: Vec2;
  _active?: boolean;
}

export interface DeleteEdgeAnnotation {
  kind: "DeleteEdge";
  midpoint: Vec2;
  edge_dir: Vec2;
  chain?: boolean;
  _active?: boolean;
}

export interface GenerateInput {
  boundary: Polygon;
  aisle_graph: DriveAisleGraph | null;
  drive_lines: DriveLine[];
  annotations: Annotation[];
  params: ParkingParams;
  debug: DebugToggles;
  regionOverrides?: RegionOverride[];
}

export interface ParkingLot {
  id: string;
  boundary: Polygon;
  aisleGraph: DriveAisleGraph | null;
  driveLines: DriveLine[];
  annotations: Annotation[];
  aisleVector: { start: Vec2; end: Vec2 };
  regionOverrides: { [regionIndex: number]: { angle?: number; offset?: number } };
  layout: ParkingLayout | null;
}

export interface RegionOverride {
  region_index: number;
  aisle_angle_deg?: number;
  aisle_offset?: number;
}
