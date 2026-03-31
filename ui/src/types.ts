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
  is_boundary?: boolean;
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

export type AisleDirection = "TwoWay" | "OneWay";

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
  cross_aisle_spacing: number;
}

export interface DebugToggles {
  // Corridor merging
  miter_fills: boolean;
  spike_removal: boolean;
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
  // Stall placement
  stall_face_clipping: boolean;
  // Boundary
  boundary_clipping: boolean;
  // Conflict removal
  conflict_removal: boolean;
  // Skeleton debug visualization
  skeleton_debug: boolean;
}

export interface DriveLine {
  start: Vec2;
  end: Vec2;
}

export type Annotation = OneWayAnnotation | DeleteVertexAnnotation | DeleteEdgeAnnotation;

export interface OneWayAnnotation {
  kind: "OneWay";
  midpoint: Vec2;
  travel_dir: Vec2;
  chain?: boolean;
  _origDir?: Vec2;
  _active?: boolean; // false = tombstone
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
}
