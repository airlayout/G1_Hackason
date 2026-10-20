// 2D版 stages/*.json と同一スキーマ。単位はpx（1m=80px）で、XZ平面にそのままマッピングする。

export interface Wall {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface Point2D {
  x: number;
  y: number;
}

export interface GuardConfig {
  x: number;
  y: number;
  patrolPath: Point2D[];
  moveSpeed: number;
  visionAngle: number;
  visionDistance: number;
}

export interface Stage {
  id: number;
  name: string;
  width: number;
  height: number;
  start: Point2D;
  target: Point2D;
  walls: Wall[];
  guard: GuardConfig;
}

export type GuardReaction = "STOP_NONE" | "STOP" | "LOOK_AT_PLAYER" | "CHASE";

export interface InputState {
  up: boolean;
  down: boolean;
  left: boolean;
  right: boolean;
  lookLeft: boolean;
  lookRight: boolean;
  lookUp: boolean;
  lookDown: boolean;
}
