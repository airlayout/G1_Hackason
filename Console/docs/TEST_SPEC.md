# Console テスト仕様書（Jetson 常駐版）

最終更新: 2026-10-18。要件は [REQUIREMENTS.md](REQUIREMENTS.md)、設計は [DESIGN.md](DESIGN.md)。実機の確認項目は [REAL_ROBOT_CHECKLIST.md](REAL_ROBOT_CHECKLIST.md)（A〜J）。

## 1. テストの種類
| 種類 | 実行方法 | 実機 | 状態の凡例 |
|---|---|---|---|
| 自動（単体・HTTP 結合） | `bash Console/tests/run_tests.sh` | 不要 | 済 / 未 |
| Mac 煙テスト（手動） | `server.py --local`・`--require-token` を起動して `curl` | 不要 | 済 / 未 |
| 実機（人が実施） | Jetson で起動、スマホから操作 | 要 | 未（歩行を伴うものは物理リモコンを手元に置く） |

現在の自動テスト: **170 件**（test_console 47 / test_lidar 20 / test_auth 28 / test_local 16 / test_camera_proc 7 / test_web 6 / test_motion 24 / test_tasks 22）。通過確認は下の実行記録に日付つきで残す。

実行記録: 2026-10-18 段階 2b 時点で 124 件通過、段階 3 時点で 148 件、段階 4 時点で 170 件通過（`run_tests.sh`）。段階 3 は `--mock` ハブでも claim → 保持 → 0.8 秒の沈黙で `lost` → 同じ session は 409 → estop を確認。

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

## 3. Mac 煙テスト（実施済み）
| 内容 | 期待 |
|---|---|
| `server.py --local` を起動（loopback のみ、認証なし） | 画面と `/api/state` が返る |
| `--require-token` で起動 | トークン無しは 401、`/login?token=…` で 302 と Cookie、Cookie で 200 |
| `--mock` と `--local` の併用 | エラー終了 |
| `--mock` でタスク: 一覧 → 起動 → ログ → 二重起動 409 → 停止（`stopped`・-15）→ 再起動してハブへ SIGTERM（段階 4、2026-10-18） | 子プロセス 0、ハブのログに例外なし |
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
| ID | 内容 | 期待 |
|---|---|---|
| T5-1 | `static.pgm` と resolution / origin から、地図のピクセル ↔ map 座標が往復で一致 | 誤差 1 セル未満（fixtures 使用） |
| T5-2 | `locations.json` の検証（名前の重複・数値以外・範囲外・yaw の単位） | 不正は保存されない |
| T5-3 | 保存先は固定。パス指定や `..` を受け付けない | NF-07 |
| T5-4 | Nav2 稼働中は `motion.disable_reason` が設定される | R-17 |
| T5-5 | `G1_ARM` / `enable_navigation` を操作するコードが存在しない | NF-09（コード検査） |
| T5-6 | ハートビートは操作セッションが生きている間だけ 5 Hz | 切れたら送信停止 |
| 実機 | 地図上の座標を RViz と見比べて一致。Nav2 起動・停止。ループバックのハートビート受理 | 一致するまで座標系は未確認 |

### 段階 6（トピック、R-19）
| ID | 内容 | 期待 |
|---|---|---|
| T6-1 | 拒否リスト（`rt/lowcmd`・`rt/api/sport/*`・`rt/api/loco/*`・`rt/arm_sdk`）への送信は、許可リストにあっても拒否 | NF-10 |
| T6-2 | 許可リスト外は拒否。許可リスト内はレート制限を超えると拒否 | |
| T6-3 | 型が分からないトピックは受信の有無だけ表示 | |

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

## 6. 更新ルール
- 実装したテストは「§4」から「§2」へ移し、テスト名と件数を書く。件数は `run_tests.sh` の実行結果と一致させる。
- 実機で確認したら §5 に日付・測定値・結果を書く。確認していないものを「済」にしない。
- `tests/run_tests.sh` に新しいテストファイルを足したら、§1 の件数も更新する。
