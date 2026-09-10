# トラブルシューティング

症状から原因を引く表です。上から順に確認してください。

---

## まず 5 つ

| 症状 | 最初に疑うこと |
|---|---|
| Quest でページが開かない | **テレオペを起動していない**（一番多い） |
| 点群が出ない | **機種違いの設定**（Mid-360 と Mid-360S） |
| **送信は成功するのに、テレオペも再生も 1 mm も動かない** | **腕のモータが無効**（`motor_state[].mode = 0` = ゼロトルク）。リモコンでダンピングへ。**終了するたびに戻る** |
| **テレオペで `r` を押しても腕が動かない** | **`--motion` の有無がロボットの状態と合っていない**（[03-teleop.md](03-teleop.md#指令トピックmotion-を付けるかどうか)） |
| 再生が成功と出るのに腕が動かない | **指令トピックの選択**（`rt/arm_sdk` が効かない状態） |

`python3 tools/mode_check.py --iface <有線iface>` が 3 条件（指令経路・モータ・委譲）をまとめて判定します。

---

## テレオペ・Quest

| 症状 | 原因と対処 |
|---|---|
| Quest でページが開かない | ① テレオペが起動しているか: `ss -tlnp \| grep 8012`。起動前は待ち受けていない<br>② PC の WiFi IP が変わっていないか: `ip -br addr show <wifi iface>`<br>③ `./scripts/quest_check.sh` でクライアント分離を判定 |
| ページは開くが腕が動かない | 「Virtual Reality」ボタンを押して**没入セッションに入っていない**。ページを開くだけでは姿勢が送られない |
| **`r` は効いて `🚀start Tracking🚀` も出るのに腕が動かない** | **指令トピックの不一致**。モーションコントローラ稼働中は `rt/lowcmd` が無視される（コントローラが 1000Hz で上書きするため）。`python3 tools/mode_check.py --iface <iface>` で状態を確認し、`--motion` の有無を合わせる |
| `[ClientStub] send request error` が大量に流れる | `--motion` を付けたのにモーションコントローラが停止している。歩行サービスが応答しないため。`--motion` を外す |
| `Enter debug mode: Failed` と出る | ① 既にデバッグ状態なら**無害な誤報**（解除不要なので何もしていない）<br>② `motion-switcher-retry` パッチが未適用だと、RPC の散発失敗だけでこう表示される。`python3 setup/apply_patches.py --dry-run` で確認 |
| **送信は成功するのに腕が 1 mm も動かない（テレオペも再生も）** | `rt/lowstate` の `motor_state[].mode` が **0**（ゼロトルク）。`python3 tools/mode_check.py --iface <iface> --print-motors` が `disabled`。**リモコンでダンピング**（FSM 1）へ。指令側の `motor_cmd[].mode=1` では有効にならない |
| 一度は動いたのに、次の実行では動かない | 前回の終了でゼロトルクへ戻っている。**テレオペ・再生のたびにダンピングに入れ直す** |
| `--motion` で `weight=1.00` を送っているのに動かない | この機体では `arm_sdk` の委譲に応じない（検証機で 0.000 rad）。`tools/armsdk_probe.py` で確認し、駄目なら `mode_check.py --release` → `--motion` なし |
| `ai` を解除したら脱力した／`mode=0` になった | 立って制御中の状態から解除すると起きる。吊り下げ等で支持したうえで、リモコンでダンピングへ |
| `LocoClient` / `Damp()` が `code=3102` | `ai` 解除中は loco サービスが応答しない。ダンピングはリモコンで入れる |
| 腕の `tau_est` が全関節ほぼゼロ | どの制御器も腕を駆動していない。`mode=1` ならデバッグ状態 + `rt/lowcmd` で駆動できる。`mode=0` なら上の行 |
| **追従のラグが大きい（体感で 0.2 秒以上）** | ほぼ確実に **Quest ↔ PC の無線**。`ping -c 20 <QuestのIP>` で測る。有線のロボット経路が 0.2ms なのに対し、混雑した 2.4GHz では平均 200ms を超えることがある。下の「ネットワーク」を参照 |
| `winit EventLoopError: neither WAYLAND_DISPLAY ... is set` | SSH 経由で Rerun のビューアを開こうとしている。`--headless` を付ける。記録には影響しない |
| `AssertionError` / `BrokenPipeError` が aiohttp から出る | Quest のブラウザが静的ファイルの取得を途中で切っただけ。websocket が繋がっていれば**実害なし** |
| 証明書の警告が出る | 自己署名なので正常。「詳細設定 / Advanced」→「アクセスする / Proceed」 |
| Quest が勝手に別の WiFi に切り替わる | Quest 側で不要なネットワークを「削除／忘れる」 |
| 起動が 1 分ほど返ってこない | 29DoF 機の初回は逆運動学モデルの再構築が走る。**正常**。待つ |
| `Waiting to subscribe dds...` で止まる | ロボットとの有線疎通 NG。`./scripts/preflight.sh` で確認 |
| 片腕が特定の向きでフリーズ | コントローラがヘッドセットのカメラ視界外（特に真横・背後）。顔の前〜やや横に留める |
| 操作中に左右がズレる | `head-reference` パッチが当たっていない。`python3 setup/apply_patches.py --dry-run` で確認 |
| **`s` を押した瞬間に落ちる** | `record-camera` パッチが当たっていない。頭部カメラが無い環境で画像を保存しようとして `TypeError` になる |
| `Head image is None!` が出続ける | カメラが無いだけ。**正常**。関節角の記録は続いている |
| 起動直後に腕が高速で振られた | `safe-startup` パッチが当たっていない。即 `q`（危険なら電源）→ パッチを当てて再起動 |
| `conda: command not found` | 新しいターミナルを開く。または `source ~/miniforge3/etc/profile.d/conda.sh` |
| `ImportError: cannot import name 'Vuer'` | `params_proto` が 3.x になっている。`pip install 'params_proto==2.13.2'` |

---

## ネットワーク

| 症状 | 原因と対処 |
|---|---|
| 有線 IP が勝手に消える | NetworkManager が管理していて `ip addr add` を上書きする。`nmcli` でプロファイルとして設定する（[02-network.md](02-network.md)） |
| 有線を挿すとインターネットが切れる | 有線プロファイルがデフォルトルートを奪っている。`ipv4.never-default yes` と `ipv4.gateway ""` を設定 |
| ロボットに ping が通らない | ロボットの起動完了を待つ。ケーブルを差し直す。`ip -br addr` でリンクが `up` か確認 |
| `preflight.sh` で「未知のホスト」が出る | 想定外の機器がいる。LiDAR の可能性が高いので `tools/lidar_probe.py` で確認 |
| **Quest との通信が遅い / ラグい** | 2.4GHz の混雑が原因のことが多い。`nmcli -f SSID,CHAN dev wifi list` でチャネルごとの AP 数を数える。同一チャネルに 10 個以上あれば飽和している。5GHz（ch 36〜48）に移すと桁が変わる |
| テザリングの 5GHz/6GHz が一覧に出てこない | ① **6GHz は届く距離が短い**（屋内低出力）。スマホを PC の横に置いて再スキャン<br>② 規制ドメインが `00`（world）だと使える帯域が狭い。`cat /sys/module/cfg80211/parameters/ieee80211_regdom` で確認し、`sudo iw reg set JP`<br>③ 一覧に出なくても、SSID を明示したプロファイルを作れば接続できる（6GHz は `key-mgmt sae` と `pmf 3` が必須） |
| SSID にスラッシュが入っていて繋がらない気がする | SSID の文字は原因になりません。同じ SSID が別の帯域で見えているかを先に確認してください |
| **何をやっても Quest の無線が安定しない** | **PC 自身を AP にする**: `./scripts/quest_ap.sh up`。1 ホップ・分離なし・チャネル選択可。実測で平均 RTT 228ms → 66ms（アイドル時。セッション中はさらに下がる）。[02-network.md](02-network.md) |
| テザリングを「5GHz」にしたら一覧から消えた | 実は **6GHz**（Wi-Fi 6E/7 端末はバンドが 2.4/6 の二択）。6GHz 単独 AP は RNR 誘導が無く PC から発見できない。WPA3 必須なのでセキュリティを下げても 5GHz にはならない。2.4GHz に戻すか PC を AP にする |
| Quest を USB で繋ぎたい | 開発者モードを有効化 → `adb reverse tcp:8012 tcp:8012` → `https://localhost:8012/?ws=wss://localhost:8012`。遅延 1ms 台。`adb devices` に出なければ ADB インターフェース（class ff/42/01）が未公開＝開発者モード未有効 |

---

## LiDAR

| 症状 | 原因と対処 |
|---|---|
| **`Init lds lidar success!` の先へ進まない** | 機種違いの設定。`python3 tools/lidar_probe.py --write-config` で作り直す |
| `lidar_probe.py` が LiDAR を見つけない | ① ドライバが 56000 を占有 → 先に止める<br>② LiDAR が他ホストに接続済み → ロボット再起動直後に実行<br>③ 同じサブネットにいない → 有線設定を確認 |
| トピックはあるがレートが 0 | ロボットを再起動したあとドライバが取り残されている。**ドライバを起動し直す**（プロセスは生きているのでパッと見では気づきにくい） |
| 点群が天地逆に見える | `config/g1.env` の `G1_LIDAR_FLIP` を切り替える。判定は `tools/lidar_probe.py --orientation` |
| 点群が二重に見える | ダミー配信（`--fake`）が残っている。`ros2 topic info /livox/lidar` の Publisher count が 2 なら重複 |
| RViz に何も出ない | Fixed Frame が `viz_base` になっているか。`--fake` 以外では表示用 TF が必要 |
| `bag` が空 | 先に `lidar_view.sh` を起動していない |
| ディスクが一気に減る | 点群は約 300MB/分。`du -sh bags/*` で確認して古いものを消す |

---

## 記録の再生

| 症状 | 原因と対処 |
|---|---|
| **「再生成功」と出るのに腕が動かない** | ① 指令トピックが合っていない（`--topic auto` を使う）<br>② `mode_machine` / `motor_cmd[].mode` が未設定（このキットでは対応済み） |
| フレーム数が 0 | 記録に失敗している。`s` を押したか、記録開始で落ちていないか確認 |
| 再生した姿勢が全く違う | `--dof` の判定違い。dry-run の「機体判定」を見て `--dof 23` / `--dof 29` を明示 |
| 手順 2 で腕が大きく動く | 現在姿勢と記録の初期姿勢が離れている。**正常**だが危なければ Enter で中断 |
| 脚が脱力した | `rt/lowcmd` で全関節ロックが効いていない。「全 29 関節を現在角度でロックしました」が出ているか確認 |
| 再生が途中で「状態が途切れました」で止まる | 有線の疎通か、PC の負荷。RViz/PlotJuggler を閉じるか `--state-timeout 1.0` に上げる（中断自体は安全） |
| 「モーションコントローラが起動しました」で止まる | `rt/lowcmd` 中にロボット側の制御が立ち上がった。リモコン操作の有無を確認。競合を避けるための正常な停止 |
| `--topic rt/lowcmd` が拒否される | モーションコントローラが稼働中。リモコンで止めるか、腕だけなら `--topic rt/arm_sdk` を使う |
| リミット逸脱の警告 | 記録がロボットの可動範囲を超えている。再生時にクリップされるので動作自体は安全 |

---

## 環境・導入

| 症状 | 原因と対処 |
|---|---|
| `sudo` が使えない | VSCode の統合ターミナルは `no_new_privs` で sudo が通らない環境がある。ネイティブ端末（Ctrl+Alt+T）で実行。確認: `grep NoNewPrivs /proc/self/status`（0 なら OK） |
| 貼り付けたコマンドが `^[[200~...` で失敗 | 端末の貼り付け化け。**1 行だけ手で打つ**か、スクリプト経由で実行する |
| `Could not locate cyclonedds` | `CYCLONEDDS_HOME` に `~` を書いている。**`$HOME` を使う**（ダブルクォート内で `~` は展開されない） |
| ROS のコマンドで python エラー | conda が有効になっている。ROS 用のシェルでは conda を抜く（`scripts/lib.sh` の `use_ros` が自動で外す） |
| `colcon build` が失敗する | ROS 環境を `source` していない。または conda が PATH に混ざっている |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` の下で ROS の `setup.bash` を読んでいる。`set +u` してから読む |

---

## 切り分けに使えるコマンド

```bash
# 状態を一通り見る
./scripts/preflight.sh
python3 tools/mode_check.py --iface <有線iface>                  # 指令経路・モータ有効・委譲の 3 条件
python3 tools/mode_check.py --iface <有線iface> --print-motors   # enabled / disabled / unknown
python3 tools/cmd_monitor.py --iface <有線iface> --seconds 120   # 指令と実測を同時に見る（別端末で）
python3 tools/armsdk_probe.py --iface <有線iface>                # arm_sdk が効く機体か（3° だけ動く）
./scripts/quest_ap.sh status                                      # 自前 AP の接続端末と URL
python3 tools/lidar_probe.py
./scripts/quest_check.sh

# 無線のラグを測る（Quest の IP は quest_check.sh のログに出る）
ping -c 20 -i 0.3 <QuestのIP>     # 平均 10ms 未満なら健全。100ms を超えるならチャネル飽和
nmcli -f SSID,CHAN,FREQ,SIGNAL dev wifi list | awk 'NR>1{c[$2]++} END{for(x in c) print "ch"x, c[x]"個"}'

# ネットワーク
ip -br addr
ping -c 3 192.168.123.161
ss -tlnp | grep 8012

# ROS
source scripts/lib.sh && load_config && use_ros
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic info /livox/lidar        # Publisher count が 2 なら重複配信

# パッチの適用状況
python3 setup/apply_patches.py --dry-run

# 止め残しの掃除
pkill -f livox_ros_driver2_node
pkill -f rviz2
pkill -f plotjuggler
# ※ テレオペは必ず q で終わらせる。強制終了しない
```

---

## それでも分からないとき

「何が起きていないか」を切り分けてから聞くと早く解決します。

1. **リンクは上がっているか**（`ip -br addr`）
2. **相手に届いているか**（`ping`）
3. **データは来ているか**（`preflight.sh` / `ros2 topic hz`）
4. **こちらは待ち受けているか**（`ss -tlnp`）

とくに **「プロセスは生きているのにデータが流れていない」** 状態は見落としやすいので、レートまで確認する癖をつけてください。
