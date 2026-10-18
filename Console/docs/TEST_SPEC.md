# Console テスト仕様書（Jetson 常駐版）

最終更新: 2026-10-18。要件は [REQUIREMENTS.md](REQUIREMENTS.md)、設計は [DESIGN.md](DESIGN.md)。実機の確認項目は [REAL_ROBOT_CHECKLIST.md](REAL_ROBOT_CHECKLIST.md)（A〜J）。

## 1. テストの種類
| 種類 | 実行方法 | 実機 | 状態の凡例 |
|---|---|---|---|
| 自動（単体・HTTP 結合） | `bash Console/tests/run_tests.sh` | 不要 | 済 / 未 |
| Mac 煙テスト（手動） | `server.py --local`・`--require-token` を起動して `curl` | 不要 | 済 / 未 |
| 実機（人が実施） | Jetson で起動、スマホから操作 | 要 | 未（歩行を伴うものは物理リモコンを手元に置く） |

現在の自動テスト: **286 件**（test_console 47 / test_lidar 20 / test_auth 34 / test_local 16 / test_camera_proc 7 / test_web 8 / test_motion 24 / test_tasks 22 / test_nav 34 / test_topics 19 / test_caution 22 / test_ctl_local 16 / test_hub_stop 9 / test_control 8）。通過確認は下の実行記録に日付つきで残す。

実行記録: 2026-10-18 段階 2b 時点で 124 件通過、段階 3 時点で 148 件、段階 4 時点で 170 件、段階 5 時点で 204 件、段階 6 時点で 223 件、段階 7 時点で 244 件、段階 8 時点で 260 件、`--no-auth` 追加後に 266 件、ハブ停止ボタン追加後に 276 件、スティック2つ化後に 277 件、状態カード化・制御状態の追加後に 286 件通過（`run_tests.sh`。JS の構文検査 `node --check` も通過）。段階 3 は `--mock` ハブでも claim → 保持 → 0.8 秒の沈黙で `lost` → 同じ session は 409 → estop を確認。

## 2. 自動テスト（実施済み）
| 要件 | 内容 | テスト |
|---|---|---|
| R-01 | IP リテラルなら ssh を呼ばない。別名は従来どおり ssh に尋ねる | `test_local.ResolveHostTest` |
| R-01 | カメラ・lidar の制御がローカル実行で、ssh を一切呼ばない。未知の操作は拒否 | `test_local.LocalCtlTest` |
| R-01 | `LocalHelper` が子プロセスで往復できる。SDK が無いときは落ちずオフライン表示 | `test_local.LocalHelperTest` |
| R-01 | 表示文言に ssh が残らない | `test_local.DetailTextTest` |
| R-02 | トークンファイルが 0600 で作られ再利用される。空なら再生成 | `test_auth.TokenFileTest` |
| R-02 | `--bind` / `--bind-iface` の解釈。`ip` コマンド失敗・不正名・アドレス無しの扱い。loopback 先頭・重複除去 | `test_auth.BindTest` |
| R-03〜R-05 | 空トークン拒否、Host 判定、Origin 判定、Cookie / Bearer、5 回失敗でロック、失敗の失効、成功でクリア | `test_auth.AuthUnitTest` |
| R-03 | 認証なしで API は 401。`/` はログイン画面。不正トークンは 401。正しいトークンで Cookie が付き通る。誤った Cookie は拒否 | `test_auth.AuthHttpTest` |
| R-04 | 未知の Host は Cookie があっても拒否。許可した追加 Host は通る。別オリジンの POST は拒否、同一オリジンは通る | `test_auth.AuthHttpTest` |
| R-05a | `--no-auth`: トークン無しで API と画面が開く。`/login` は 302 で Cookie なし。未知の Host・別オリジンの POST は拒否のまま。空トークンは `required=False` のときだけ許す。`--no-auth` と `--require-token` の併用は終了コード 2 | `test_auth.NoAuthHttpTest` / `NoAuthCommandLineTest` |
| R-03 | 未認証ではモード変更が拒否され、ヘルパーに触れない | `test_auth.AuthHttpTest.test_mode_change_blocked_without_auth_and_helper_untouched` |
| R-14 | カメラ・lidar の起動停止、状態解析 | `test_camera_proc`、`test_lidar` |
| 既存 | モード表示・切り替え、模擬シナリオ、API スキーマ | `test_console` |
| R-07 | 主ナビ 5 グループの定義、全画面がちょうど 1 グループから到達できる、画面切り替えボタンの存在、`#hash` = タブ名、設定・DDS が「開発」、スマホ向け CSS の土台が残っている（ソース検査。JS 構文は node で確認） | `test_web.NavStructureTest`（6 件） |
| R-08, R-11 | 指令が `duration` 付き速度で送られる／速度がクランプされ NaN・無限大・文字列・bool は拒否 | `test_motion.ControllerTest` T3-1, T3-2 |
| R-09, NF-05 | 受信が 0.5 秒途絶するとゼロ速度で `lost`。自動再開しない。生存は受信時刻で判定 | `ControllerTest` T3-3, T3-3b |
| R-12 | 緊急停止は常に `stop` を送り操作者を外す | `ControllerTest` T3-4 |
| R-10 | 単独操作者・横取り | `ControllerTest` T3-5 |
| R-11 | FSM 500/501 かつ状態が新しいときだけ運動 | `ControllerTest` T3-6 |
| R-17 下地 | `disable_reason` で claim 拒否・操作者を外す | `ControllerTest` T3-7 |
| R-09 | ゼロ目標は停止 1 回だけで、連続送信しない | `ControllerTest` T3-8 |
| R-08 | 順序が逆の ctl は無視し、生存の根拠にしない（デッドロック回帰） | `ControllerTest` T3-9 |
| R-09 | 送信 3 回連続失敗で操作者を外す | `ControllerTest` T3-10 |
| R-12 | 停止後に古い判断を送らない（世代） | `ControllerTest` T3-11 |
| R-13 | アクションは許可リスト・操作者・歩行 FSM が必要 | `ControllerTest` T3-12 |
| R-08〜R-13 | HTTP 経由の claim / ctl / status / estop、400・409、blocked 理由、`fsm_source` | `test_motion.HttpMotionTest` T3-13 ほか |
| NF-06, NF-09 | `motion_helper` は move/stop/task のみ・禁止名なし。偽の子プロセスで往復、無応答は破棄、拒否はエラー | `test_motion.HelperTest` T3-14 ほか |
| R-15 | 定義の検証（シェル文字列・空 argv・未知キー・cwd 逃げ・timeout 範囲・名前）。同梱の定義が読める。壊れた JSON は起動を止める | `test_tasks.DefinitionTest` T4-1 |
| R-15, NF-07 | 定義にない名前・不正な型は 404 / 400。クライアントが argv・cwd・env を渡しても使われない | `RunnerTest` T4-1、`HttpTaskTest` T4-1b |
| R-15, NF-07 | シェルのメタ文字が文字列のまま渡り、シェルを経由しない | `RunnerTest` T4-2 |
| R-15 | 同じ排他グループ・同じタスクの二重起動は 409。解放後は起動できる | `RunnerTest` T4-3 |
| R-15 | 停止で SIGTERM（`stopped`・-15）。無視されたら SIGKILL（-9）。孫プロセスも止まる | `RunnerTest` T4-4, 4b, 4c |
| R-15 | タイムアウトで `timeout`。終了コードと結果（ok / failed） | `RunnerTest` T4-5, 5b |
| R-15 | リングバッファの溢れ（`truncated`）、`since` の続き取得、長い行の切り詰め | `RunnerTest` T4-6, 6b |
| R-15 | 起動失敗は 500 でタスクが固まらない。再実行でログが作り直される。`shutdown` が全部止める | `RunnerTest` T4-7, 7b, 7c |
| R-15 | HTTP: 一覧・起動・409・ログ・停止、不正入力の 400 / 404 | `HttpTaskTest` T4-8, 8b |
| R-15 | ハブへ SIGTERM を送ると実行中のタスクも止まる（ハブを子プロセスで実起動） | `HubShutdownTest` T4-9 |
| R-16 | アダプタ URL は loopback の http だけ。状態・地図情報はホワイトリストと範囲で絞る。不正（型・範囲・点数・巡回状態）は拒否 | `UrlAndSanitizeTest` T5-1, 2, 2b, 2c |
| R-16, NF-09 | アダプタへは GET だけ（POST が届かない）。大きすぎる応答・PNG でない・HTTP エラー・不達は `NavError` | `ProxyTest` T5-3, 3b, 3c |
| R-17 | Nav2 稼働で `disable_reason` を 1 回だけ設定。解除はアダプタが応答し、かつ未接続のときだけ。一度見たあとの沈黙は無効のまま。一度も見ていなければ排他なし。実際の `MotionController` で操作者が外れる | `WatcherTest` T5-4, 5, 5b, 6, 6b |
| R-17 | 人がいない間は生存信号を送らない。パケット形式が受信側（29 バイト）と一致。presence が古くなれば停止。プロセスごとに別の session_id。送信失敗は報告して例外にしない。背景スレッドは touch の間だけ送る | `HeartbeatTest` T5-7, 7b, 8, 8b, 8c, 8d, 8e |
| R-18 | 登録・上書き・削除と永続化。検証（名前・数値・範囲・map_id）。壊れたファイルは起動を止め上書きしない。保存失敗でメモリが変わらない。上限。`resolve` | `LocationsTest` T5-9, 9b, 9c, 9d, 9e, 9f |
| R-18, R-15 | タスクの `{loc:…}` が起動時に展開される。未登録は 409 で起動しない。定義の構文不正は読み込みで拒否。地点ストアが無ければ 500 | `TaskLocationTest` T5-10, 10b, 10c, 10d |
| R-16〜R-18 | HTTP: 状態・地図・排他、presence で生存信号、地点の CRUD、Nav2 へ届く経路が無い、アダプタ未設定の応答 | `HttpNavTest` T5-11, 11b, 12, 12b, 12c |
| R-19, NF-10 | 拒否リストは許可リストより強い（実行時・起動時の両方）。名前の検証。許可外は 403、既定は空。サイズ（バイト数）・型・レート制限・失敗時に間隔を消費しない | `PolicyTest` T6-1, 1b, 1c, 2, 2b, 2c, 2d |
| R-19 | 型が読めない購読は断る（偽の受信を作らない）。購読・受信・解除・上限。拒否リストのトピックも購読（読み取り）はできるが送信は 403。許可リストファイルの検証 | `PolicyTest` T6-3, 4, 5, `test_allowlist_file_validation` |
| R-19 | ヘルパーのソースに運動系クライアントが無い・拒否リストを二重に検査・置換される。偽の子プロセスで起動・往復・停止、無応答のタイムアウトと購読の復元、拒否の扱い、起動しない子 | `HelperTest` T6-6, 6b, 7, 7b, 7c, 7d |
| R-19 | HTTP: 購読・受信・403/413/422/429/400/404 の返り方、未設定なら 404 | `HttpTopicsTest` T6-8 |
| R-20 | タスク定義の `verified`（既定 false）・`caution` の検証と、`/api/tasks` に出ること | `TaskDefinitionTest` T7-1 |
| R-20, NF-09 | 同梱の Nav2 タスク 4 つが 1 つの排他グループにある・全て未確認で注意書きあり・verified は 2 つのロボット非接触タスクだけ・G1_ARM/enable_navigation/sudo/--arm/--enable を含まない・コマンドが警備班の手順と一致 | `ShippedNav2TasksTest` T7-2 |
| R-20, NF-09 | `nav2_down.sh`: 構文、実行行に sudo/systemctl/ゲート書き込みが無い、dry-run は何も止めない、ゲート閉なら runbook の順（記録 → Nav2 → 1901）、ゲート開（ファイル・稼働中ブリッジ）は止めたうえで終了 3 で設定ファイルは不変、未知の引数は 2 | `Nav2DownScriptTest` T7-3（偽の `pkill`/`ps`/`sleep` を PATH に置く） |
| R-20 | ポップアップの部品が揃う・既定は「やめる」・HTML として解釈しない・チェックで実行ボタンが有効になる・未知の機能は未確認扱い | `WebCautionTest` T7-4 |
| R-20 | 実機に作用するボタン（モード・音声・LED・カメラ・lidar・運動・アクション・トピック）と、未確認・注意書きのタスクが、`fetch` の前にポップアップを出す | `WebCautionTest` T7-5 |
| R-20 | ポップアップを出す機能 id が FEATURES に実在し、未確認である | `WebCautionTest` T7-6 |
| R-14 | `--local` は `Console/jetson/` の ctl を呼び（`~/g1_console_*` を使わない）、`*_PORT`・`*_BIND=127.0.0.1`・`G1_CTL_STATE_DIR` を渡す。ssh 方式の文字列は従来どおり。camera も設定のポートを渡す。シェルに入るのは固定語と整数だけ | `CommandLineTest` T8-1 |
| R-14 | 配信スクリプトが `--bind`（既定 0.0.0.0）を持ち、サーバーがそれを使う | `StreamBindTest` T8-2 |
| R-14 | 本物の `camera_ctl.sh`・`lidar_ctl.sh` を偽の python/setsid で実行: `--port`・`--bind` が渡る、既定は従来どおり、pid・ログは指定ディレクトリ（リポジトリ内に作らない）、status/stop は何も起動しない | `ScriptRunTest` T8-3 |
| R-14 | 既定は停止: 何も頼まなければ ctl は呼ばれない。ページ・状態の GET は `status` だけ。start/stop は POST のときだけ | `DefaultStoppedTest` T8-4 |
| R-14, R-20 | カメラ・lidar の起動・停止ボタンは確認ポップアップ（guard）を通る | `DefaultStoppedTest` T8-5 |
| R-16〜R-18 | API 定義（`/api/nav/*`・`/api/locations*`）がスキーマと実応答に一致。画面に `map` タブのボタンとパネルがある | `test_console`（既存の網羅テスト） |

## 3. Mac 煙テスト（実施済み）
| 内容 | 期待 |
|---|---|
| `server.py --local` を起動（loopback のみ、認証なし） | 画面と `/api/state` が返る |
| `--require-token` で起動 | トークン無しは 401、`/login?token=…` で 302 と Cookie、Cookie で 200 |
| `--mock` と `--local` の併用 | エラー終了 |
| `--mock` でタスク: 一覧 → 起動 → ログ → 二重起動 409 → 停止（`stopped`・-15）→ 再起動してハブへ SIGTERM（段階 4、2026-10-18） | 子プロセス 0、ハブのログに例外なし |
| `--mock` + 偽アダプタ + UDP 受信（段階 5、2026-10-18）: 状態・地図 PNG・地点の保存と一覧・不正地点 400・presence 無しは 0 件／presence 中は約 5 Hz（29 バイト・magic 47314842）／止めると 0 件・Nav2 稼働で claim が 409・アダプタ停止後も 409 のまま | アダプタに届いたのは GET のみ（POST 0）、ハブのログに例外なし |
| `--mock` で claim → ctl 保持 → 0.8 秒の沈黙 → `lost` → 同 session は 409 → estop（段階 3、2026-10-18） | `moving` 真→偽、`last_event.kind` が `lost` / `estop`、ハブのログに例外なし |

未実施: `--bind-iface wlan0`（Mac に `ip` コマンドが無いため、Jetson で確認する）。

## 4. 今後の段階のテスト（作るもの。実装と同じコミットで追加し、状態を更新する）
状態はすべて「未」。ID は段階 + 連番。

### 段階 2b（画面再編、R-07）
T2b-1・T2b-2 は §2 の `test_web` に移した。残り:
| ID | 内容 | 期待 | 種類 |
|---|---|---|---|
| T2b-3 | 幅 360px で横スクロールなし、主ナビ 4 項目（マップ・タスクは空のため非表示）が押せる | スクロール幅 ≤ 画面幅 | ブラウザ／スマホ手動（未実施。実機 A3 に反映） |

### 段階 5（Nav2・地図・ロケーション、R-16〜R-18）
自動テストは §2 に移した（T5-1〜T5-12）。残り（人または実機が要る）:
| ID | 内容 | 期待 | 種類 |
|---|---|---|---|
| T5-13 | マップタブの目視: 地図・▲・経路・地点・クリックで x/y が入る・登録と削除（幅 360px も） | 描ける | ブラウザ手動（未実施。拡張が未接続だった） |
| T5-14 | 地図上の座標を RViz と見比べて一致。実アダプタの `/api/state` の形 | 一致するまで座標系は未確認 | 実機 |
| T5-15 | 実受信側（g1_cmd_router）がループバックの生存信号を受け付ける。途絶で FAULT になる | 記録する | 実機 |

### 段階 6（トピック、R-19）
| ID | 内容 | 期待 |
|---|---|---|
自動テストは §2 に移した（T6-1〜T6-8）。T6-3 は設計を変えた（型が読めないものは受信の有無も出せないので、断る）。残り（人または実機が要る）:

| ID | 内容 | 期待 | 状態 |
|---|---|---|---|
| T6-9 | DDS タブの「トピック通信」の目視: 購読・受信表示・解除・送信欄（幅 360px も） | 描ける | ブラウザ手動（未実施） |
| T6-10 | Jetson で実際に `rt/rtc/state` を購読して値が出る。許可リストに載せた String_ トピックへ送信し、受信側で観測できる | 出る | 実機 |

### 段階 7（Nav2 の起動・停止、注意ポップアップ、R-20）
自動テストは §2 に移した（T7-1〜T7-6）。残り（人または実機が要る）:

| ID | 内容 | 期待 | 状態 |
|---|---|---|---|
| T7-7 | ポップアップの目視（幅 360px も）: 「やめる」が既定、チェックを入れるまで実行できない、Esc で閉じる、未確認タグ | 描ける・操作できる | ブラウザ手動（未実施） |
| T7-8 | Jetson で `nav2-up-dry`・`nav2-down-dry` がハブから通る（`ROS_DISTRO` 未設定の環境で） | 通る | 実機（J18・J19） |
| T7-9 | 機体を立たせて `nav2-up` → Nav2 が起動し機体は動かない → `nav2-down` で止まる。ゲートを閉じたまま・開けたままの両方 | 止まる・開けたままは失敗表示 | 実機（J20・J21） |
| T9-1 | `POST /api/hub/stop`: `confirm: true` で 200 の後に停止関数が 1 回だけ呼ばれる。`confirm` 無し・偽・文字列・数値・不正 JSON・配列は 400 で止まらない。GET は 404。`hub_stop` が無ければ 404 | 期待どおり | `test_hub_stop.HubStopTest` |
| T9-2 | タスクが running / stopping のとき、または操作権が取られているとき 409 で止まらない。idle なら止まる | 期待どおり | `test_hub_stop.HubStopTest` |
| T9-3 | 認証あり: トークン無しは 401、Bearer ありは 200、別オリジンの POST は 403 で止まらない | 期待どおり | `test_hub_stop.HubStopTest` |
| T9-4 | 停止ボタンは `guard('hubstop')`（チェック必須）を通ってから POST する。機能 `hubstop` は実機未確認 | 期待どおり | `test_caution` |
| T9-5 | `--mock` ハブで実際に停止要求 → 終了コード 0、ポートが閉じる（手動） | 済（2026-10-18 Mac） | 手動 |
| T9-6 | Jetson で画面から停止 → 終了コード 0 でプロセスが消え、systemd 配下では再起動しない。カメラ・lidar の配信が残るか | 要確認 | 実機（J24） |
| T8-6 | Jetson で画面から lidar・カメラを起動 → `ss -ltn` で `127.0.0.1:8082`/`8081` のみ（0.0.0.0 でない）。停止後は両ポートが空く。ハブ起動直後は両ポートが空 | 期待どおり | 実機（J23） |

## 5. 実機確認の対応表（人が実施。実施したら日付と測定値を残す）
| 要件 | 確認 | チェックリスト |
|---|---|---|
| R-01, R-06 | Jetson で `--local` が Python 3.8 で動く。`--bind-iface wlan0` が IP を取れる。systemd 設置 | J1〜J3 |
| R-02 | スマホから `http://<wlan0 の IP>:18790/?token=…` が開く。eth0 側アドレスでは届かない | J4, J5 |
| R-03〜R-05 | 別端末でトークン無し・誤りが拒否される | J6 |
| R-07 | 幅 360px 実機（スマホ）で崩れない | A3 |
| R-08, R-09 | 離す・タブ非表示・Wi-Fi 切断で止まる。停止までの遅れ、止まるまでの歩行距離。パッドのタッチ操作（マルチタッチ）と見た目 | E1, E3, E4, E5 |
| R-10, R-12 | 2 台のスマホで操作者が 1 人。緊急停止の効き（物理リモコンを手元に） | E6 |
| R-13 | アクション番号 0〜3 の実際の動作 | E7 |
| R-11 | 上限速度、500 と 501 の違い | E2, B3 |
| R-15 | Jetson 上でタスクが動く（sample-countdown、run-tests）。systemd 配下でハブを止めるとタスクも止まる | J9, J10 |
| R-16, R-17 | 実アダプタでマップタブが描ける。RViz と座標が一致。Nav2 稼働で手動操作が無効、停止後の挙動 | J11, J12, J14 |
| R-17 | 生存信号を実受信側が受け付ける。途絶で止まる | J8, J13 |
| R-18 | 地点をタスクが参照して動く。座標系が map と一致 | J12, J15 |
| R-19 | Jetson の DDS で購読・送信が動く。拒否リストのトピックへ送れない | J16, J17 |
| R-14 | カメラ・lidar が画面から起動・停止できる。停止中はポートが空き、起動中も 127.0.0.1 だけ | J23 |
| R-20 | Nav2 の起動・停止をコンソールから実行できる。ゲートが開いたままなら失敗表示。ポップアップが出る | J18〜J21 |

## 6. 更新ルール
- 実装したテストは「§4」から「§2」へ移し、テスト名と件数を書く。件数は `run_tests.sh` の実行結果と一致させる。
- 実機で確認したら §5 に日付・測定値・結果を書く。確認していないものを「済」にしない。
- `tests/run_tests.sh` に新しいテストファイルを足したら、§1 の件数も更新する。
