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
- `--no-auth`（R-05a）: `Auth("", hosts, required=False)`。`authorized()` が常に真、`/login` は `/` へ 302（Cookie なし）。Host・Origin の検査（`_gate`）は残る。`--require-token` とは排他。起動ログに警告を出し、トークンファイルは作らない。
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

## 4. 段階 3: 運動制御（実装済み・実機未確認）
### 4.1 なぜ別プロセスか
`remote_helper.py` は要求を 1 行ずつ直列に処理し、`SshHelper.call` も `threading.Lock` で直列化する。`status` が重いと速度指令が待たされるため、**運動専用の最小ヘルパー**（`jetson/motion_helper.py`）を分ける。`LocoClient` の `SetVelocity` と `StopMove` だけを許し、`SetFsmId` は持たない（テストでも禁止名を検査）。

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
`SetTaskId` を許可リスト（`modes.py` の `ACTIONS`: wave 0 / wave_turn 1 / shake_start 2 / shake_end 3）で実行する。操作者であり FSM 500/501 のときだけ。番号と動作は SDK の資料からの推定で、実機未確認。`PLANNED_ACTIONS` は空。

### 4.6 実装の要点（実装済み）
- **構成**: `g1console/motion.py` の `MotionController`（送信器を差し替え可能。`HelperSender` = 子プロセス `motion_helper.py`、`MockSender`、ssh 時は `UnavailableSender` で 503）。ハブの tick スレッドが 20 Hz（`SEND_PERIOD_S=0.05`）で `duration=0.2` の `SetVelocity` を送る。
- **操作権**: `claim` が `session` を返す。途絶・緊急停止・解放・横取り・無効化のあと操作者は空になり、再度 `claim` が要る（自動再開しない）。
- **ロックと世代**: 判断は `_lock`、送信は別の `_send_lock`。停止のたびに世代 `_gen` を進め、停止前に決めた古い判断は送らない。`seq` が増えない遅延指令は反映せず、生存の根拠にもしない。
- **ゼロ速度**: 停止を 1 回送ったあとは何も送らない（物理リモコンと取り合わない）。送信が 3 回連続で失敗したら操作者を外す。
- **防御の二重化**: ハブ（`LIMITS`: vx −0.2〜0.3、vy ±0.2、wz ±0.4、`DEADBAND` 0.01）とヘルパー（同値で再クランプ、`duration` 上限 0.5 秒）。
- **既知の限界**: ヘルパーは直列なので、緊急停止は送信中の `move` の後ろに並ぶ（最大 `HELPER_TIMEOUT_S`=1.0 秒）。バックストップはロボット側の `duration=0.2` 切れ。
- **教訓**: 古い `seq` を `_lock` を持ったまま `status()`（同じロックを取る）に渡してデッドロックした。ロックの外で呼ぶ。
- **Nav2 排他の差し込み口**: `set_disable_reason(reason)`（段階 5 が呼ぶ）。

## 5. 段階 4: タスク（実装済み・Jetson 未確認）
- 定義 `Console/tasks/*.json`: 名前・表示名・説明・argv 配列・cwd・排他グループ・タイムアウト・前提条件。**シェル文字列は受け付けない**（NF-07）。
- 実行: 新しいプロセスグループで子プロセスを起動し、ログはリングバッファ（`since` 付きポーリング）。停止は SIGTERM → SIGKILL（グループ）。排他グループのロックで同時起動を防ぐ。`exit_code` を返す。
- API の形は erasers-app（`get_task` / `run_task` / `kill_task` / `task_running`）を参考にする。erasers は `template` がシェル文字列で、`get_xml` / `save_xml` が任意パスを読み書きできる。**どちらも採らない**。

### 5.1 実装の要点
- `g1console/tasks.py`: `load_definitions`（起動時に全 JSON を検証。1 つでも不正ならハブが起動しない）と `TaskRunner`。定義のキーは `title / description / argv / cwd / group / timeout_s` のみ（未知のキーは拒否）。`cwd` は Console からの相対で外に出られない。`timeout_s` は 1〜3600。
- 実行: `shell=False`・`stdin=DEVNULL`・`start_new_session=True`。stdout と stderr は 1 本にまとめて行単位で 2000 行のリングバッファへ（1 行は 2000 字で切る）。結果は `ok / failed / stopped / timeout`、`exit_code`（負はシグナル）。
- 停止: プロセスグループへ SIGTERM、猶予 3 秒で SIGKILL。リーダーが終わったあとも孫が残らないよう、終了時にグループへ SIGKILL を送る。
- 排他: 同じタスクの二重起動、または同じ `group` のタスクがあれば 409。グループなしは自分自身とだけ排他。
- API: `GET /api/tasks`、`GET /api/tasks/logs?name=&since=`（`next` を次回の `since` に渡せば欠落・重複なし）、`POST /api/tasks/start|stop`（受け取るのは `name` だけ。余分なキーは無視）。
- ハブの終了: SIGTERM を `SystemExit` に変えて後始末（運動の停止・運動ヘルパー・タスク）を必ず通す。以前は SIGTERM で後始末が走らず、子が孤児になりえた。systemd 配下では cgroup ごと止まるが、手で起動した場合の備え。
- 既知の限界: 子を reap したあとに `killpg` する経路があり、pid の再利用が極めて短時間に起きると無関係なグループへ送りうる（実運用では無視できる確率。必要なら `waitid(WNOWAIT)` へ）。タスクの中身がロボットを動かす場合の安全は、そのタスク側の責任（運動の指令元の排他は段階 5 の `disable_reason` で扱う）。


## 6. 段階 5: Nav2・地図・ロケーション（実装済み・実機未確認）
### 6.1 構成
- ハブは ROS を持たない。同じ機体で動く ROS アダプタ（`G1_HuggingFace/UI/Adapter` 系、pixi の Humble）へ **loopback の HTTP で GET だけ**する（`g1console/nav.py` の `NavProxy`）。URL は `--nav-adapter http://127.0.0.1:<port>` のみ許す（`parse_adapter_url`。外のホスト・パス・認証情報は起動時に拒否）。環境変数のプロキシは使わない。
- 読むのは `/api/state`・`/api/map.json`・`/api/map.png` だけ。応答は外部データとして検証する（サイズ上限、PNG 署名、型・範囲・点数の上限、ホワイトリスト）。不正は 502（`NavError`）。
- **Nav2 へ送る口は無い**。`NavApi.post` が扱うのは presence とロケーションだけで、それ以外は `None`（404）。Goal・巡回操作・`enable_navigation`・`G1_ARM` は人が現地で扱う（NF-09）。Nav2 の起動・停止はこの画面とは別に、タスクとして行う（§7）。
### 6.2 排他（R-17）
- `NavWatcher` が 1 Hz で状態を読み、`connected`（アダプタの bridge_status が生きている）なら `motion.set_disable_reason(REASON_RUNNING)`。変化したときだけ呼ぶ。
- fail-safe: 一度でも稼働を見たあと、アダプタが応答しなくなったら `REASON_UNKNOWN` で**無効のまま**。解除は「アダプタが応答し、かつ `connected` が偽」のときだけ。一度も稼働を見ていなければ排他なし。
- 監視スレッドは例外で死なせない（死ぬと排他が外れたままになる）。
### 6.3 生存信号（オプトイン）
- `--heartbeat-to HOST:PORT`。形式は Nav2 側の `HeartbeatPacket`（`<IBQQQ`、29 バイト: magic 0x47314842・version 1・session_id・seq・monotonic_ns）。5 Hz。
- **ハブが生きているだけでは送らない**。ブラウザが `POST /api/nav/presence` を 0.5 秒ごとに呼んでいる間（最後から 0.6 秒以内）だけ送る。そうしないと、スマホが切れてもハブが信号を出し続け、途絶検知が働かない。マップタブを表示中かつ画面が見えている間だけ呼ぶ。
- 受信側（g1_cmd_router）がループバック送信元を受け付けるかは未確認（J8）。
### 6.4 ロケーション（R-18）
- `Console/data/locations.json`（`--locations-file` で変更可。パスはクライアントから受け取らない）。`{"version":1,"locations":[{name,x,y,yaw_deg,map_id}]}`。名前は `^[\w\-]{1,40}$`、最大 200 件、|x|・|y| ≤ 10000 m、yaw は ±180°。保存は一時ファイル → fsync → `os.replace`。保存に失敗したらメモリも変えない。壊れたファイルは起動を止め、上書きしない。
- タスクの argv に `{loc:NAME.x|y|yaw_deg}` を書ける。構文は定義の読み込み時に検証、展開は起動時（ロックの前）。未登録は 409、地点ストアが無ければ 500。
### 6.5 画面
- 「マップ」タブ（マップ・タスク群）。canvas に地図 PNG・現在地（▲）・目的地・経路・巡回ルート・登録済みの地点を描く。座標変換は `x = origin[0] + px*resolution`、`y = origin[1] + (height - py)*resolution`（地図 PNG の左下が origin という前提。**RViz と見比べるまで未確認**）。クリックで x・y が入り、「現在地を入れる」「登録」「削除」ができる。表示は `textContent` のみ。
### 6.6 未確認（推測で埋めない）
- 実アダプタの `/api/state` の形（`pose`・`plan`・`route` の単位と並び）、`connected` が「Nav2 稼働中」の判定として十分か。
- アダプタを pixi 環境で `--host 127.0.0.1` 固定で動かせるか。
- 地図 PNG の向きと座標変換。MappingData の `static.pgm`（780×431、resolution 0.1、origin (-12.696, -17.928)）は fixtures 候補。どの `stage_*` が現行かは未確認で、1.3 GB を repo に入れない。
- 画面はブラウザで目視していない（拡張が未接続だった）。Nav2 の起動・停止は §7。

## 7. 段階 6: トピック通信（実装済み・Jetson 未確認）
- `g1console/topics.py`: 判定とサービス層（副作用なし）。`topic_backend.py`: Jetson の DDS ヘルパー（`jetson/topic_helper.py`）を子プロセスで持つ。ヘルパーは最初の購読・送信まで起動しない。`--mock` は偽メッセージを返し送信は記録するだけ。`--local` と `--mock` のときだけ有効（ssh 経由では 404）。
- 受信: 呼び出し側が型を指定し、`SUPPORTED_TYPES`（String_・Error_・WirelessController_・BmsState_・IMUState_・SportModeState_）にあるものだけ購読する（最大 8 件）。**型が読めないものは 422 で断る**。当初の案「型が分からないものは名前と受信有無だけ」は、型なしでは DDS の受信を観測できず、できないことをできるように見せることになるため採らなかった。購読は読み取りだけなので、拒否リストのトピック（`rt/lowcmd` 等）も見られる。配列は 16 要素・文字列は 512 文字で丸める。
- 送信: 許可リスト（JSON `{version:1, publish:[{topic, type:String_, max_bytes, min_interval_s, note}]}`）にある String_ だけ。許可リストはサーバー側のファイルだけで、画面・API から増やせない。ファイルが無ければ空（何も送れない）。判定順: トピック名の検証 → 拒否リスト（403）→ 許可リスト（403）→ data の型（400）→ max_bytes（413）→ 最小間隔（429、時計は注入できる）。送信に失敗したら間隔を消費しない。ヘルパー側でも拒否リストと許可リスト・バイト数を再検査する（ハブの判定が破られても DDS に出さない）。
- 運動は専用画面（運動タブ）からだけ。ここから `rt/api/*` 等は出せない。画面は DDS タブの「トピック通信」。DDS 由来の値は `textContent` だけで描く。
- 未確認: 実際の Jetson 上での購読・送信（型・QoS）、許可リストに載せる実用トピック。

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


## 7. 段階 7: Nav2 の起動・停止タスクと、動作未確認の注意ポップアップ（実装済み・実機未確認）
### 7.1 Nav2 の手順の出どころ（訂正）
- 当初の記述「検証済みの停止手順が無い」は誤りだった。警備班の `Navigation/nav2_stable` に、起動（`g1up.sh` / `start_nav.sh`）と撤収（`safety/runbook_nav2_session.md` §10）の手順があり、2026-09-24・25 の実機で使われている。GitHub の `trcp`（`trcp/g1_ws`・`trcp/erasers-app`）にあるのは launch ファイルで、G1 の起動・停止手順は無い。
- 無いのは「コンソール（ハブ）から実行した」実績である。
### 7.2 構成
- 新しいサーバーコードは足さない。既存の `TaskRunner`（argv のみ・shell なし・新プロセスグループ・排他グループ・ログ）に、定義ファイル 4 つを足した: `nav2-up-dry`・`nav2-up`・`nav2-down-dry`・`nav2-down`。全て排他グループ `nav2`（同時に 2 つ走らない）。
- `nav2-up` = `/home/unitree/g1_nav2/g1up.sh`（姿勢検査 → ブリッジ（ゲート閉）→ 内蔵 SLAM 1801 → 記録 → Nav2 → 自己位置合わせ → 検収。ここで機体は動かない）。`-dry` は `--dry-run`。
- `nav2-down` = `jetson/nav2_down.sh`（runbook §10: ①ゲートを閉じる（sudo・人）→ ②`ros2 bag record` を止める → Nav2・橋渡しノードを落とす → ③`send_slam_api.py 1901` → ④`systemctl stop g1-sdk-bridge`（sudo・人））。②③だけを同じ順で行い、①④は画面に表示するだけ。runbook は単一のコマンドではなく手順の列なので、この 1 ファイルだけ新規に書いた。
- `nav2_down.sh` はゲートを開けない・閉じない・設定ファイルを書かない・sudo を呼ばない。ゲートが開いている（起動引数に `--arm`、または設定ファイルが `G1_ARM=--arm`）ときは、止められる部分は止めたうえで終了コード 3 → 画面は「失敗」。
- 起動タスクは「機体が動かない状態」で終わる。ゲートを開ける・走行許可（`enable_navigation`）は人が現地で行う（NF-09）。
### 7.3 注意ポップアップ
- 目的: 動作未確認の機能のボタンを押したとき、結果を断定せず、確認を取ってから実行する。
- 機能（FEATURES）: `verified` が偽（または一覧に無い）なら、モード・アクション・運動の開始・音声・LED・カメラ/lidar の起動停止・トピック送信の押下時に `guard(機能 id, …)` が `<dialog>` を出す。`verified` が真でも、安全確認の問い（モード切替・読み上げ）は `always` で毎回出す（従来の `confirm` の置き換え）。
- タスク: 定義に `verified`（既定 false）と `caution`（文字列の配列。最大 8 件・各 200 字）を持たせる。未確認または注意書きありのタスクは、開始・停止の押下時にポップアップを出し、注意書きがあれば確認チェックを入れるまで「実行する」が押せない。カードに「実機未確認」タグを出す。`run-tests` と `sample-countdown` はロボットにも Jetson にも触れないので `verified: true`。
- 文面は `textContent` だけで描く（注意書きはサーバー由来なので HTML として解釈しない）。`Esc`・「やめる」・背景は実行にならない。既定のフォーカスは「やめる」。`<dialog>` が使えない古いブラウザは `confirm` に落とす。
- 実機で確認できたら、`modes.py` の `verified` やタスク定義の `verified` を真にすれば、その機能のポップアップは消える（安全確認の `always` は残る）。
### 7.4 未確認（推測で埋めない）
- コンソールから `g1up.sh` を実行したとき、ハブの環境で前提（`ROS_DISTRO` 未設定、pixi、地図、DDS 設定、`/home/unitree/g1_nav2/g1up.sh` の配置）が満たされるか。
- ハブが止まったとき、実行中の `g1up.sh`（長時間）の子プロセスがどうなるか（systemd の cgroup では一緒に止まる。J10 と同じ確認）。
- 走行中に `nav2-down` を押したときの停止距離。先に停止係が物理リモコンで止める。
- ポップアップの見た目と操作（スマホ幅を含む）は、ブラウザで目視していない。

## 10. 段階 8: カメラ・lidar の起動・停止を `--local` に対応（実装済み・実機未確認）
要求: 「他の開発プロジェクトに迷惑をかけないよう、既定は停止、画面で起動したときだけポートを使う」。ボタンと既定停止は既にあったが、`--local` では ctl の場所・待ち受け範囲・pid の置き場が足りなかった。
- **既定は停止**: ハブが ctl に `start` を渡すのは `POST /api/{camera,lidar}/start` だけ。GET（状態・ページ）は `status` のみ（T8-4）。ハブ起動・systemd 起動・設定変更では起動しない。自動再起動もしない。
- **呼び出し**（`camera_proc.SshCameraCtl._ctl`）: `[ENV_PORT=…] [追加の環境変数] <dir>/<script>`。ssh 方式は従来どおり（`~/g1_console_*`、既定 bind 0.0.0.0）。`LocalRunMixin` は `<dir>` を `Console/jetson/`、追加の環境変数を `G1_CTL_STATE_DIR=~/.g1console/run` と `*_BIND=127.0.0.1` に差し替える。シェルに入るのは固定語・整数・リポジトリ内パスのみ。
- **待ち受け**: `camera_stream.py`・`lidar_stream.py` に `--bind`（既定 `0.0.0.0` = 従来互換）。ハブがサーバー側で `/camera`・`/lidar` を中継するので、`--local` では 127.0.0.1 で足りる。wlan0・eth0 へ認証なしの配信ポートを開かない。
- **状態の置き場**: ctl は `G1_CTL_STATE_DIR`（既定は従来どおりスクリプトの隣）に pid・ログを置く。`--local` は `~/.g1console/run/`。リポジトリを汚さない（T8-3）。
- **カメラのポート**: 設定の `camera_port` を ctl へ渡す（以前は lidar だけだった）。
- **判断**: ハブ終了時に配信は止めない（配信はハブより長く生きてよい。止めるのは画面の「停止」）。止めたい運用になったら別途決める。
- **未確認**: Jetson 実機の起動・停止、停止中にポートが空いていること、起動中の loopback のみの待ち受け（`ss -ltn`）、映像デバイスの競合（videohub_pc4）は従来どおり H の確認。
