# Console 設計書（Jetson 常駐版）

最終更新: 2026-10-18。要件は [REQUIREMENTS.md](REQUIREMENTS.md)、検証は [TEST_SPEC.md](TEST_SPEC.md)。
状態の凡例: **実装済** / **設計のみ**（未実装）/ **未確認**（実機で確かめていない）。

## 1. 全体構成
```
スマホ / 外部 PC（ブラウザだけ）
        │  HTTP（wlan0、トークン認証）
        ▼
Jetson（PC2）
  ┌──────────────────────────────────────────────┐
  │ ハブ  server.py（stdlib ThreadingHTTPServer）  │  ← 常駐するのはこれだけ
  │   ├ auth.py          認証・Host/Origin 検証     │
  │   ├ LocalHelper ── remote_helper.py（子プロセス）── DDS(eth0) ── G1   状態・モード・音声・LED
  │   ├ MotionController ── motion_helper.py（子プロセス）── DDS ── G1    速度指令 【設計のみ】
  │   ├ LocalCameraCtl / LocalLidarCtl ── camera_ctl.sh / lidar_ctl.sh     オンデマンド
  │   ├ TaskRunner  【設計のみ】 ── tasks/*.json の子プロセス               オンデマンド
  │   └ NavAdapter  【設計のみ】 ── localhost HTTP ── pixi Humble（Nav2）   オンデマンド
  └──────────────────────────────────────────────┘
```
- 待ち受けは loopback と wlan0 の IPv4 だけ。eth0（192.168.123.x）には開かない。
- ハブは ROS を持たない（NF-01）。ROS に触る部分は別プロセスに閉じ、localhost で話す。

## 2. 実装済みの部品（段階 1・2）
### 2.1 ローカル実行（R-01）
- `SshHelper` を継承した `LocalHelper` が、`remote_helper.py` を子プロセスとして起動する（`sys.executable -u -c <src>`、cwd は SDK ディレクトリ。`$UNITREE_SDK_DIR` か `~/unitree_sdk2_python`）。JSON 行の stdin/stdout、要求は**直列処理**。
- `LocalRunMixin` が `camera_proc` / `lidar_proc` の `_run` を `bash -c` のローカル実行に差し替える（`LocalCameraCtl`、`LocalLidarCtl`）。ssh は呼ばない。
- `resolve_host` は IP リテラルならそのまま返す。
- SDK が無いときは、落ちずに「オフライン」と表示する。

### 2.2 待ち受け（R-02）
- `--bind <ip>`（複数可）と `--bind-iface <name>`。`resolve_binds` は loopback を先頭にして重複を除く。`--bind-iface` は `ip -4 -o addr show dev NAME` から取り、名前は検証する。
- `0.0.0.0` は拒否。ループバック以外に開くときはトークン必須（`--require-token` でも強制）。
- バインドごとに 1 つの `ThreadingHTTPServer`。2 つ目以降は daemon スレッド。

### 2.3 認証（R-03〜R-05）
- トークン: `~/.g1console/token`（0600）。無ければ `secrets.token_urlsafe` で作る。比較は `hmac.compare_digest`。
- `/login?token=…` で Cookie `g1c_token`（HttpOnly、SameSite=Strict）を発行し、302 で `/`。以後は Cookie か `Authorization: Bearer`。
- `Handler._gate(post)` の順序: Host 検証（403）→ POST なら Origin 検証（403）→ `/login` 処理 → 認可 → 未認証は `/` ならログイン画面つき 401、API なら JSON 401。
- 失敗が 60 秒に 5 回で 30 秒ロック（429）。成功で履歴を消す。
- `make_handler(..., auth=None)` が既定で、認証なし（従来動作）。
- **限界**: 平文 HTTP なのでトークンは LAN で盗聴されうる。運動の安全はトークンだけに頼らない（NF-08）。

### 2.4 常駐（R-06）
- `jetson/g1-console.service`（雛形、未設置）。`ExecStart=python3 server.py --local --bind-iface wlan0`、`Restart=on-failure`。設置・有効化は人が `sudo` で行う。
- カメラ・lidar の配信は常駐させず、自動再起動もしない。

## 3. 段階 2b: 画面の再編（実装済み。見た目は未確認）
- 既存の `web/index.html`（viewport、`max-width:899px` の下部固定タブ、`pointer:coarse`）を土台にし、作り直していない。
- `GROUPS` 配列が主ナビを決める: 操作（モード・音声）／状態（状態・関節）／マップ・タスク（空。非表示）／センサ（カメラ）／開発（DDS・設定）。主ナビはスマホで下部固定、PC で上部。グループに画面が 2 つ以上あるときだけ、画面の切り替え（`#nav-sub`）を出す。
- `#hash` 直リンクは維持する（タブ名 = URL ハッシュ = API 名）。グループの最後に開いた画面を覚える。
- 設定は「開発」に入れた（独立したメニューにはしていない）。
- 段階 3 以降は、画面を該当グループの配列へ足す（運動操作＝操作、タスク・地図＝マップ・タスク）。
- ブラウザ拡張に接続できず、見た目（幅 360px で崩れないか）は目視していない。

## 4. 段階 3: 運動制御（設計のみ・未確認）
### 4.1 なぜ別プロセスか
`remote_helper.py` は要求を 1 行ずつ直列に処理し、`SshHelper.call` も `threading.Lock` で直列化する。`status` が重いと速度指令が待たされるため、**運動専用の最小ヘルパー**（`jetson/motion_helper.py`）を分ける。`LocoClient` の `SetVelocity` と `StopMove` だけを許し、`SetFsmId` は持たない。

### 4.2 指令の流れと時間定数
| 項目 | 値 | 根拠 |
|---|---|---|
| ブラウザ → ハブ | 10 Hz で `POST /api/motion/ctl {seq, vx, vy, wz}` | stdlib のポーリング式 |
| ハブ → ロボット | 20 Hz で `SetVelocity(vx, vy, wz, duration=0.2)` | 送信周期 < `duration`（Nav2 D-27 と同じ規則） |
| 生存判定 | 最後の受信から 0.5 秒超でゼロ速度、`lost` 状態 | NF-05 |
| ハブ停止時 | `duration` 切れでロボットが止まる | NF-06 |

0.5 秒・0.2 秒・速度クランプの値は**初期値**で、実機で確認するまで確定ではない（未確認）。

### 4.3 状態遷移
`disarmed`（起動直後）→ 操作権を取得（`/api/session/claim`）→ `armed` → 指令受信中 `active` → 途絶・解放・緊急停止で `lost`（ゼロ速度）。`lost` から `active` へは**明示的な再開操作**が必要（自動再開しない）。
- 運動は FSM 500/501 のときだけ許可（`Monitor.snapshot()` の `mode.fsm_id`）。
- `motion.disable_reason` が空でなければ指令を拒否する（段階 5 で「Nav2 稼働中」を入れる）。
- 速度はサーバー側でクランプする（定数。ブラウザの値を信用しない）。

### 4.4 ブラウザ
- Pointer Events のジョイスティック（マルチタッチ）。指を離す・`visibilitychange`・`pagehide` でゼロ速度を送り、`navigator.sendBeacon` で最後のゼロを送る。
- 緊急停止ボタンを常時表示。「ソフトの停止は安全規格外。物理リモコンが最後の砦」と画面に書く。

### 4.5 アクション
`WaveHand` / `ShakeHand` / `SetTaskId` を許可リストで実行する。`modes.py` の `PLANNED_ACTIONS` と `FEATURES` を同時に更新する。

## 5. 段階 4: タスク（設計のみ）
- 定義 `Console/tasks/*.json`: 名前・表示名・説明・argv 配列・cwd・排他グループ・タイムアウト・前提条件。**シェル文字列は受け付けない**（NF-07）。
- 実行: 新しいプロセスグループで子プロセスを起動し、ログはリングバッファ（`since` 付きポーリング）。停止は SIGTERM → SIGKILL（グループ）。排他グループのロックで同時起動を防ぐ。`exit_code` を返す。
- API の形は erasers-app（`get_task` / `run_task` / `kill_task` / `task_running`）を参考にする。erasers は `template` がシェル文字列で、`get_xml` / `save_xml` が任意パスを読み書きできる。**どちらも採らない**。

## 6. 段階 5: Nav2・地図・ロケーション（設計のみ・未確認）
- `/map` は保存済みの固定地図。生きているのは現在地（TF map→base_link）、`/plan`、巡回ルート、`/g1/bridge_status`、`/g1/patrol/status`。
- `G1_HuggingFace/UI/Adapter/ros_adapter.py`（rclpy）を流用し、pixi の Humble 環境で小さなアダプタを動かす。ハブとは localhost HTTP。
- 起動は `g1up.sh`。停止手順は未確認で、先に調べる。`G1_ARM` と `enable_navigation` は自動化しない（NF-09）。
- 操作セッションが生きている間だけ、UDP 47311 に 5 Hz でハートビートを送る（`operator_lost` の入力）。ループバック送信元の受理は未確認。
- 地図表示: `static.pgm` と resolution・origin をハブが HTTP で渡し、ブラウザの canvas に描く。MappingData の `static.pgm`（780×431、resolution 0.1、origin (-12.696, -17.928)）は fixtures 候補。どの `stage_*` が現行かは未確認で、1.3 GB を repo に入れない。
- ロケーション: `Console/data/locations.json`。保存先は固定で、パスをクライアントから受け取らない。

  ```json
  {"map_id": "uis", "rooms": [{"name": "...", "locations": [
    {"name": "...", "x": 0.0, "y": 0.0, "yaw": 0.0, "is_door": false}]}]}
  ```
  RViz と見比べて座標系を検証するまで、座標系は未確認。

## 7. 段階 6: トピック通信（設計のみ）
- 受信: 任意のトピック。IDL 型は `unitree_sdk2py` が持つものだけ。型が分からないものは名前と受信有無のみ。
- 送信: 許可リスト（音声・LED など運動に関係しないもの）だけ。NF-10 の拒否リストは常に優先。レート制限あり。運動は専用画面からだけ。

## 8. 設計上の判断（理由つき）
| 判断 | 理由 |
|---|---|
| ハブを ROS 化しない | 目的は外部 PC 依存の削減。ROS に閉じ込めると Jetson 以外の環境で動かせなくなる |
| 運動専用ヘルパーを分ける | 既存ヘルパーは直列で、重い要求が速度指令を遅らせる |
| 生存判定を受信時刻で行う | TCP / ssh は切断を検知できなかった（2026-09-09 実測） |
| 接続断で自律直立（FSM 500/501）を保つ | ダンピング・ゼロトルクは転倒の危険がある |
| wlan0 直接（tailscale 不使用） | 複数人が追加インストール無しで使える。速度の優劣は未測定（E3 で測る） |
| トピック送信は許可リスト | 運動系を誤って叩けない構造にする |

## 9. 更新ルール
実装した節は「設計のみ」を外し、実機で確かめたら「未確認」を外して日付と測定値を書く。
