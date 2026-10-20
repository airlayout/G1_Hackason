// 1m = 80px（2D版と共通スケール）。3D空間ではXZ平面をpx単位のまま使い、
// レンダリング時にPX_TO_MでThree.jsのメートル単位に変換する。
export const PX_TO_M = 1 / 80;

export const PLAYER_RADIUS_PX = 10;
export const PLAYER_SPEED_PX = 80; // px/sec (1.0 m/s)
// STARTマーカー(床の円、半径0.3m=24px)と重なりやすいように、判定は見た目より広めに取る。
export const TARGET_PICKUP_MARGIN_PX = 20;
export const START_RETURN_MARGIN_PX = 20;
export const PLAYER_EYE_HEIGHT_M = 1.5;
export const KEY_LOOK_SPEED_RAD = Math.PI * 0.6; // rad/sec（矢印キー視点操作の回転速度）

// 人間の視野角の近似値(Panero & Zelnik の人間工学データに基づく実用視野)。
// 水平: 両眼視野 約120°、垂直: 見上げ約50°+見下ろし約70°で約120°。
// 画面のアスペクト比に関わらずこの比率を再現するため、カメラのaspectは
// 画面比率ではなくこのFOVペアから固定値として算出し、余白はレターボックスで埋める。
export const PLAYER_HORIZONTAL_FOV_DEG = 120;
export const PLAYER_VERTICAL_FOV_DEG = 120;

// 見上げ/見下ろしの可動範囲(真上・真下の手前でクランプし、視界反転を防ぐ)。
export const PLAYER_MAX_PITCH_RAD = (85 * Math.PI) / 180;

export const GUARD_RADIUS_PX = 10;
export const WAYPOINT_ARRIVAL_DIST_PX = 4;
export const GUARD_TURN_SPEED = Math.PI / 4; // rad/sec
export const GUARD_TURN_ALIGN_THRESHOLD = (2 * Math.PI) / 180;
export const REACTION_DURATION_SEC = 2.5; // STOP/LOOK/CHASEを継続する時間

// Entame/docs/g1-detection-spec.md の初期値。
export const DETECTION_HOLD_TIME_SEC = 0.35; // 警戒→OUTまでの検知継続しきい値(0.2〜0.5s)
export const DEFAULT_WALL_HEIGHT_M = 1.5; // stage JSONにheightが無い壁のデフォルト高さ
