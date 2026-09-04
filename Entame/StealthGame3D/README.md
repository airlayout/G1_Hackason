# Stealth G1 3D PoC

`Entame/StealthGamePoC/`（2D Canvas版）を Three.js + TypeScript + Vite で3D化したPoC。
ゲームルール・ステージデータ（`stages/*.json`）は2D版と共有する（`public/stages` は
`../../StealthGamePoC/stages` へのシンボリックリンク）。

## 起動

```bash
npm install
npm run dev
```

ブラウザで表示されたURL（例: http://localhost:5173/）を開く。

## 操作

- 移動: 矢印キー / WASD
- レベル選択画面で G1 の発見後の反応パターン（①反応しない/②停止/③目撃方向を見る/④追跡）を選べる

## 実装の要点

- 視認判定（距離5m + 水平FOV70° + 遮蔽物）は2D版 `guard.js` の `canSeePlayer` をXZ平面にそのまま移植（`src/guard.ts`）
- 2D版と異なり、検知は瞬間アウトではなく時間しきい値あり（`DETECTION_HOLD_TIME_SEC`、`Entame/docs/g1-detection-spec.md` の 0.2〜0.5秒仕様に準拠）
- スケールは2D版と同じ 1m = 80px。`PX_TO_M` でThree.jsのメートル単位に変換（`src/constants.ts`, `src/scene.ts`）
- パーティション（壁）の高さは stage JSON に無いため固定値 1.5m を補完（`DEFAULT_WALL_HEIGHT_M`）

## 対象外（今後の課題）

- 実会場（20m×20m等）とのパラメータ対応付け
- G1実機・G1制御APIとの接続
