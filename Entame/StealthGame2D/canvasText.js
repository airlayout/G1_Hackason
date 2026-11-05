// Canvas内の文字サイズ。HTML側の最小文字サイズ（style.css の --fs-min）と揃える。

export const CANVAS_FONT_PX = 32;
export const CANVAS_FONT = `${CANVAS_FONT_PX}px monospace`;

const LABEL_GAP_PX = 6;

// 丸い印（半径 radius）の真上にラベル（textBaseline = "middle"）を置くときの Y 座標。
export function labelY(centerY, radius) {
  return centerY - radius - LABEL_GAP_PX - CANVAS_FONT_PX / 2;
}
