# Pepper 監視役 仕様（2D ステルスゲーム用）

`g1-detection-spec.md` の Pepper 版。2D ステルスゲームの監視役（固定・首振り）に反映する数値を整理する。

**各数値の確度**を次の記号で分ける。推測で埋めない。

| 記号 | 意味 |
|---|---|
| ✅ | リポジトリのコード・README に書かれている値（出典つき） |
| 📖 | NAOqi の一般仕様（記憶）。この Pepper では未確認 |
| ❓ | 未測定。ゲームには「仮置き」を入れる |

出典: `origin/Dev/Entame` の `Entame/real/pepper_app/`（README、`pepperapp/robot.py`、`pepperapp/naoqi_robot.py`）、`Entame/real/pepper_vlm_brain/adapters/pepper/`。

---

## 1. 前提

| 項目 | 値 | 確度 |
|---|---|---|
| 機体 | Pepper、NAOqi 2.5.10.907 | ✅ README |
| 接続 | NAOqi API（ポート 9559）。有線 192.168.123.99 / Wi-Fi 192.168.0.85（DHCP で変わる） | ✅ README |
| ゲームでの役割 | **その場から動かない。首だけ振る。見つかったら即ゲームオーバー** | 決定（2026-11-03） |
| 移動・回転 | しない。`pepper_app` は「回る・進む」を送らない作り | ✅ README |

## 2. カメラ

| 項目 | 値 | 確度 |
|---|---|---|
| 使うカメラ | 頭の上カメラ（`TOP_CAMERA = 0`） | ✅ camera.py |
| 水平画角（FOV） | **57.2°** | ✅ robot.py `LookConfig.hfov_rad` |
| 垂直画角 | 44.3° | ✅ robot.py |
| 解像度 | QVGA（320×240）。VGA も可 | ✅ camera.py / README |
| フレームレート | 有線 VGA 29.7 fps / Wi-Fi QVGA 4.2 fps・VGA 1.1 fps | ✅ README |
| カメラの高さ | 約 1.2 m | ✅ README |
| 推論時間 | Pepper のカメラ映像で 134 ms（3.4 fps）。見本画像は 35 ms。原因は未確定 | ✅ README |

G1（水平 70°）より狭い。**映像を使う認識は有線が前提**（Wi-Fi は遅すぎる）。

## 3. 首の動き

2D ゲームでは水平（yaw）だけ使う。pitch は無視する。

| 項目 | 値 | 確度 |
|---|---|---|
| HeadYaw の関節限界 | ±2.0857 rad（±119.5°） | ✅ robot.py `HEAD_YAW_LIMIT` |
| HeadYaw のアプリ上の使用範囲 | **±1.0 rad（±57.3°）** | ✅ robot.py `yaw_limit` |
| HeadPitch の限界 / 使用範囲 | −0.7068〜+0.6371 rad / −0.4〜+0.3 rad | ✅ robot.py |
| 速度の指定方式 | 関節の最大速度に対する**割合**（0.01〜0.30）。`angleInterpolationWithSpeed` | ✅ naoqi_robot.py |
| 既定の割合 | 追従 0.15（`pepper_app`）、VLM brain の `look_at_joints` は 0.10 | ✅ robot.py / motion.py |
| **回転速度（rad/s）** | 割合×最大速度。**最大速度の絶対値がリポジトリに無い。実測していない** | ❓ |
| 首が着くまで | ブロッキング（着くまで戻らない） | ✅ naoqi_robot.py |
| 不感帯 | 0.05 rad（これ未満の差は動かさない） | ✅ robot.py `deadband_rad` |

首を振ったときにカメラが見える範囲は、中心から ±(57.3° + 57.2°/2) ≒ **±85.9°**（合計 約172°）。ただし ±1.0 rad の使用範囲に収めた場合。

### Web 調査（2026-11-04）：速度の上限の見積もり

**関節の最大速度（rad/s）を公表した資料は見つからなかった。** Aldebaran の仕様書は可動範囲（±119.5° = ±2.0857 rad）だけで、速度は載っていない。NAOqi の `getLimits("HeadYaw")` が最大速度（rad/s）を返す仕様なので、**実機で呼べば確定する**（📖 NAOqi ドキュメント）。

代わりにモーターの仕様（Aldebaran の Pepper Motors ページ）から**理論上の上限**を計算した。

| 項目 | 値 |
|---|---|
| HeadYaw のモーター（SE24P2CTCA、ブラシ付き DC） | 無負荷回転数 5 715 rpm |
| 減速比（Type A） | 101.3 |
| 関節側の無負荷速度（5 715 rpm ÷ 101.3） | 約 56.4 rpm = **約 5.9 rad/s（約 338°/s）** |

これは**負荷なしの理論上限**で、実際の最大速度はこれより遅い。関節側のソフト制限があるかも不明（❓）。割合指定で当てはめた目安は次のとおり（上限が 5.9 rad/s だとした場合）。

| 割合 | 目安 | 使っている所 |
|---|---|---|
| 0.10 | 約 0.59 rad/s（約 34°/s） | VLM brain の `look_at_joints` |
| 0.15 | 約 0.89 rad/s（約 51°/s） | `pepper_app` の追従 |
| 0.30 | 約 1.77 rad/s（約 102°/s） | アプリの割合の上限 |

ゲームの仮置き 28.6°/s（0.5 rad/s）は、この目安より遅い側にある（割合に直すと約 0.085）。**根拠が増えただけで、確定ではない。** 実測するまで仮置きのままにする。

### 速度の測り方（未実施）

1. `ALMotion.getLimits("HeadYaw")` を呼び、`maxVelocity`（rad/s）を読む。
2. 実機で `HeadYaw` を −1.0 → +1.0 rad に、割合 0.10 / 0.15 / 0.30 で動かし、着くまでの時間を測る。rad/s = 2.0 ÷ 秒。
3. 測った値を `securityCamera.js` の `CAMERA_SWING_SPEED_DEG` に入れる。

出典: [Pepper - Motors（Aldebaran）](https://doc.nao6.maxtronics.com/family/pepper_technical/motors_pep.html)、[Pepper - Joints（Aldebaran 2.5）](http://doc.aldebaran.com/2-5/family/pepper_technical/joints_pep.html)、[Joint control（Aldebaran 2.5）](http://doc.aldebaran.com/2-5/naoqi/motion/control-joint.html)。

## 4. 自動で向く機能（ALBasicAwareness）

| 項目 | 内容 | 確度 |
|---|---|---|
| 有効なとき | Pepper が自分で首を向ける | ✅ README（これが理由で `pauseAwareness()` している） |
| 向く刺激の種類（人・音・動き・接触） | NAOqi の一般仕様 | 📖 |
| 声の方向の推定（ALSoundLocalization） | 頭のマイクで推定 | 📖 |
| この機体で声の方向に向くか／精度／遅れ | コードも実測も無い | ❓ |

**ゲームと実機を連動させるなら `ALBasicAwareness` は止める。** 止めないと、実機の向きがゲーム内の視野とずれる。
音で気づく仕掛けは別途の拡張（今回は対象外）。

## 5. 人物検知条件

G1 と同じ 4 条件の形式。

```text
人物検出（YOLO11n、人 = 姿勢モデル ∧ 物体検出モデルが重なる所 IoU 0.4 以上）
  ↓
① 視認角度内？（水平 57.2°）
  ↓ YES
② 視認距離内？
  ↓ YES
③ 遮蔽物なし？
  ↓ YES
④ 一定時間以上検出？
  ↓ YES
「見つかった」＝ 即ゲームオーバー
```

| 条件 | 値 | 確度 |
|---|---|---|
| 水平視認角 | 57.2°（±28.6°） | ✅ |
| 検知継続時間 | 0.5 秒（`person_appeared`。人が現れた判定） | ✅ README |
| 見失う判定 | 1.5 秒（`person_left`） | ✅ README |
| 最大視認距離 | **未測定。ゲームには 3 m を仮置き**（根拠: README に「3 m 以上は関節点がぶれやすい」、録画調整は 1・2・3 m の予定） | ❓ |
| 最小距離 | **未測定。G1 の 2.5 m を流用しない**。安全距離は Pepper の腕・首の可動で別に決める | ❓ |
| 遮蔽 | 遮蔽されていれば検知しない | 設計（G1 と同じ） |

## 6. 2D ゲームへの反映値

換算は 1 m = 80 px（README の既定）。**仮置きの項目は実測後に差し替える。**

ゲーム内ではこの監視役を**セキュリティカメラ**と呼ぶ（2026-11-03 実装済み）。ステージ JSON のキーは `securityCamera`（1 ステージに 1 台。無ければ `null` か省略）。角度はすべて**度**。

| JSON の項目 | 意味 | 既定値 | 元 | 確度 |
|---|---|---|---|---|
| `x`, `y` | **初期位置**（px）。固定で動かない | エディタでクリックして置く | 設計 | 決定 |
| `direction` | 首振りの**中心の向き**（度。0 = 右、90 = 下） | 90 | 設計 | 決定 |
| `swingAngle` | **振り幅**（度、全幅。`direction` を中心に ±半分ずつ） | 90（上限 114.6 = HeadYaw ±1.0 rad の全幅） | 上限は ✅ | 上限 ✅ |
| `swingSpeed` | 首振り速度（度/秒） | **28.6**（= 0.5 rad/s、仮置き） | 実測待ち | ❓ |
| `visionAngle` | 視野角（度） | **57.2** | 水平 FOV | ✅ |
| `visionDistance` | 視認距離（px） | **240**（= 3 m、仮置き） | 実測待ち | ❓ |
| 発見時 | 即ゲームオーバー | — | 決定 | 決定 |
| 検知継続時間 | 0.5 秒（現行の `canSeePlayer` は 0 秒で判定。入れるなら別途） | 未実装 | README | ✅ |

`swingSpeed`・`visionAngle`・`visionDistance` は省略すると既定値が入る（`securityCamera.js` の定数。仮置きの値はここだけ直せばよい）。

首の動きは等速の往復（端で折り返す）。カメラが見渡せる範囲は `direction` を中心に ±(`swingAngle` + `visionAngle`) / 2。

### ステージ JSON の例（既存の `guard` は変えない）

```json
"securityCamera": {
  "x": 700,
  "y": 100,
  "direction": 135,
  "swingAngle": 90,
  "swingSpeed": 28.6,
  "visionAngle": 57.2,
  "visionDistance": 240
}
```

座標系は `guard.js` の `facingAngle` と同じ（0 = +x 方向、y 下向き）。`direction: 135` は左下向き。

### ステージエディタ

- モード「セキュリティカメラ配置（クリック）」でクリックした位置に置く（置き直すと位置だけ動く）。
- 入力欄 `Cam Direction (°)`（中心の向き）と `Cam Swing (°)`（振り幅。0〜114.6 に収める）。未配置の間は無効。
- 「カメラを削除」で外す。壁と重なると他の点と同じく警告（`CAMERA`）。
- 薄い紫の扇形が見渡せる範囲、線が中心の向き。▶ テストプレイで実際に首を振る。

## 7. 未確認・実測が必要なこと

1. 首の最大速度（rad/s）。まず `getLimits("HeadYaw")`、次に割合 0.10 / 0.15 / 0.30 での往復時間（理論上限は約 5.9 rad/s。3 節）
2. 最大視認距離（Pepper のカメラで、人を安定して検出できる距離。1・2・3・4・5 m）
3. 最小距離（安全）。腕・首が届く範囲との関係
4. `ALBasicAwareness` を有効にしたときの、音・人への反応（向く方向、遅れ）
5. 有線時の推論時間（134 ms の原因が未確定）
6. 首を振っている最中の映像のブレが検出に与える影響
