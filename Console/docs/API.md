# 開発コンソール API 定義

<!-- api_spec.py から生成。手で編集せず `python3 Console/g1console/api_spec.py` で再生成する -->

正本は OpenAPI: **`docs/openapi.yaml`**（実行中は `GET /openapi.yaml`、JSON なら `GET /api`）。この文書は人と AI が最初に読む要約。ベース URL は `http://127.0.0.1:18790`。

## 使い方の型

- **「〜を確認して」**: まず `GET /api/status`（接続・モード・古さ）。詳細は `GET /api/snapshot` 1 回で足りる。
  `jetson.state` / `g1.state` が ok でなければ、その先の値は信用しない。`age_s` が数秒以上、または `paused=true` なら古い値。
- **「〜を実行して」**: `x-effect: robot_motion` は**ロボットが動く**。実行前に人へ確認し、実行後は `GET /api/status` で**到達を確認**する（受理 ≠ 到達）。
  `robot_write`（音・LED）は受理のみで、効果は目視・聴取で確認する。
- `x-real-robot-verified: false`・`GET /api/features` の `verified=false` は実機で未確認。結果を断定しない。
- 本文は JSON。エラーはすべて `Error`（`{"error": "理由"}`）。HTTP 400=入力不正 / 404=未定義 / 500=保存失敗 / 502=G1 または Jetson に届かない・G1 が拒否。本文はすべて Error。
- 画面のタブ名 = URL ハッシュ = タグ名（`#state` ↔ `/api/state`）。

## エンドポイント

| メソッド | パス | operationId | x-effect | 概要 |
|---|---|---|---|---|
| GET | `/api` | getApi | read | この API 定義（OpenAPI, JSON） |
| GET | `/api/status` | getStatus | read | まず最初に読む。接続（jetson/g1）・現在のモード・確認の古さ・自動確認の停止中か |
| GET | `/api/state` | getState | read | バッテリー・IMU・オドメトリ・メインボード・Jetson・リモコン・非常停止 |
| GET | `/api/joints` | getJoints | read | 29 関節の角度・速度・トルク推定・温度・指令値 |
| GET | `/api/snapshot` | getSnapshot | read | state と joints を 1 回で取得（状況把握はこれ 1 本で足りる） |
| GET | `/api/audio/volume` | getVolume | read | スピーカー音量を G1 から読む（Jetson 経由で実際に問い合わせる） |
| POST | `/api/audio/volume` | postVolume | robot_write | 音量を設定し、読み戻して一致を確認する ⚠実機未検証 |
| GET | `/api/cameras` | getCameras | read | カメラ一覧と、配信が設定済みか（映像は GET /camera/{name}） |
| GET | `/api/dds` | getDds | read | DDS トピックと RPC サービスの台帳（表示済み／未実装／対象外） |
| GET | `/api/buttons` | getButtons | read | 切り替えできるモード。POST /api/mode の id はここから選ぶ |
| GET | `/api/features` | getFeatures | read | 機能ごとの implemented / verified（verified=false は実機で未確認） |
| GET | `/api/settings` | getSettings | read | 接続先の設定（開発用 PC / Jetson / G1 の IP ほか） |
| POST | `/api/settings` | postSettings | settings | 接続先を保存して反映（Jetson を変えると ssh を張り直す） |
| GET | `/api/scenarios` | getScenarios | read | 模擬シナリオの一覧と現在値 （mock のみ） |
| GET | `/camera/{name}` | getCamera | read | カメラ映像（MJPEG のストリーム）。name は GET /api/cameras のキー |
| GET | `/openapi.yaml` | getOpenapiYaml | read | この API 定義（OpenAPI, YAML） |
| POST | `/api/monitor` | postMonitor | settings | 自動確認・自動再接続の停止／再開、または 1 回だけ確認 |
| POST | `/api/mode` | postMode | robot_motion | G1 のモード（FSM）を切り替える。**ロボットが動く** ⚠実機未検証 |
| POST | `/api/audio/led` | postLed | robot_write | 頭部 LED の色を設定 ⚠実機未検証 |
| POST | `/api/audio/tts` | postTts | robot_write | テキストを読み上げる ⚠実機未検証 |
| POST | `/api/scenario` | postScenario | settings | 模擬シナリオを切り替える （mock のみ） |

## リクエストの例

### POST /api/audio/volume

音量を設定し、読み戻して一致を確認する

```json
{"volume": 40}
```
スキーマ: `VolumePatch`。volume_after が指定値と一致していれば反映済み。

### POST /api/settings

接続先を保存して反映（Jetson を変えると ssh を張り直す）

```json
{"jetson_host": "192.168.123.164", "g1_ip": "192.168.123.161"}
```
スキーマ: `SettingsPatch`

### POST /api/monitor

自動確認・自動再接続の停止／再開、または 1 回だけ確認

```json
{"paused": true}
```
スキーマ: `MonitorPatch`。停止中はサーバーが ssh も DDS も叩かない。停止中もモード切替などの明示操作は実行できる。

### POST /api/mode

G1 のモード（FSM）を切り替える。**ロボットが動く**

```json
{"id": 3}
```
スキーマ: `ModePatch`。受理（set_code=0）は到達ではない。実行後は GET /api/status を繰り返し読み、mode.fsm_id が目標になるか確認する（15 秒を目安）。歩行中（500/501）に切り替えると転倒の恐れがある。実行前に必ず人へ確認する。実機では未検証。

### POST /api/audio/led

頭部 LED の色を設定

```json
{"r": 0, "g": 255, "b": 0}
```
スキーマ: `LedPatch`。code=0 は受理のみ。色が変わったかは目視（読み戻せない）。

### POST /api/audio/tts

テキストを読み上げる

```json
{"text": "こんにちは", "speaker_id": 0}
```
スキーマ: `TtsPatch`。code=0 は受理のみ。音が出たかは耳で確認。

### POST /api/scenario

模擬シナリオを切り替える

```json
{"scenario": "g1_off"}
```
スキーマ: `ScenarioPatch`
