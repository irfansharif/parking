export interface Vec2 {
  x: number;
  y: number;
}

export interface StallQuad {
  corners: [Vec2, Vec2, Vec2, Vec2];
  kind: "Standard" | "Compact" | "Ev";
}

export interface Metrics {
  total_stalls: number;
}

export interface SpineLine {
  start: Vec2;
  end: Vec2;
  normal: Vec2;
}

export interface Face {
  contour: Vec2[];
  holes?: Vec2[][];
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
}

export interface AisleEdge {
  start: number;
  end: number;
  width: number;
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
}

export interface DebugToggles {
  // Corridor merging
  miter_fills: boolean;
  spike_removal: boolean;
  hole_filtering: boolean;
  // Face extraction
  face_extraction: boolean;
  // Spine generation
  edge_classification: boolean;
  spine_clipping: boolean;
  // Spine post-processing
  spine_dedup: boolean;
  spine_merging: boolean;
  short_spine_filter: boolean;
  // Stall placement
  stall_face_clipping: boolean;
  // Boundary
  boundary_clipping: boolean;
  // Skeleton debug visualization
  skeleton_debug: boolean;
}

export interface GenerateInput {
  boundary: Polygon;
  aisle_graph: DriveAisleGraph | null;
  params: ParkingParams;
  debug: DebugToggles;
}
