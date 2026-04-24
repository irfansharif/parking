// Re-export all ts-rs-generated engine types from one import site so the
// UI doesn't have to list every `./bindings/Foo` path individually. This
// file is hand-written; the `Foo.ts` peers next to it are all generated
// (do not edit those by hand).
//
// Keep this index in sync when new exported types land in the engine —
// the failing import is the signal.

export type { AbstractFrame } from "./AbstractFrame";
export type { AbstractPoint2 } from "./AbstractPoint2";
export type { AbstractVertexResult } from "./AbstractVertexResult";
export type { ChainLatticeEdge } from "./ChainLatticeEdge";
export type { PerimeterPosResult } from "./PerimeterPosResult";
export type { SpliceVertexResult } from "./SpliceVertexResult";
export type { AisleDirection } from "./AisleDirection";
export type { AisleEdge } from "./AisleEdge";
export type { Annotation } from "./Annotation";
export type { Axis } from "./Axis";
export type { DebugToggles } from "./DebugToggles";
export type { DriveAisleGraph } from "./DriveAisleGraph";
export type { DriveLine } from "./DriveLine";
export type { EdgeArc } from "./EdgeArc";
export type { Face } from "./Face";
export type { GenerateInput } from "./GenerateInput";
export type { GridStop } from "./GridStop";
export type { HolePin } from "./HolePin";
export type { Island } from "./Island";
export type { Metrics } from "./Metrics";
export type { ParkingLayout } from "./ParkingLayout";
export type { ParkingParams } from "./ParkingParams";
export type { PerimeterLoop } from "./PerimeterLoop";
export type { Polygon } from "./Polygon";
export type { RegionDebug } from "./RegionDebug";
export type { RegionId } from "./RegionId";
export type { RegionInfo } from "./RegionInfo";
export type { RegionOverride } from "./RegionOverride";
export type { SpineLine } from "./SpineLine";
export type { StallKind } from "./StallKind";
export type { StallModifier } from "./StallModifier";
export type { StallQuad } from "./StallQuad";
export type { Target } from "./Target";
export type { TrafficDirection } from "./TrafficDirection";
export type { Vec2 } from "./Vec2";
