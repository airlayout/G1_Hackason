# Navigation を止める

G1 が **Nav2 で指定した経路を勝手に歩き始める / 歩き回り続ける**ときの止め方。
Mac / Ubuntu のどちらの操作PCからでも行える。

> **Claude Code を使っている場合は「Navigationを止めて」と言えばよい。**
> `.claude/skills/stop-navigation/` のスキルが `nav_stop.sh` を実行する。

## まず：足を止める（人の手で）

**リモコンでダンピング（L2+B）にする。** 脱力して倒れるので、**必ず支えるか吊ってから**押すこと。
**純正リモコンが唯一の最終防衛線**（`../nav2_stable/safety/runbook_nav2_session.md`）。
以下の手順は、その後で「再び歩き出さない」状態にするためのもの。

## 一発で止める（Mac / Ubuntu 共通）

```bash
cd G1_Hackason
git pull                               # Dev/Navigation02 ブランチ
bash Navigation/real/nav_stop.sh
```

最後に `[stop] 止めた。` と出れば完了。`⚠️` が出たらその行の指示に従う。

| オプション | 意味 |
|---|---|
| （なし） | G1 の PC2 → この操作PC の順に全部止める |
| `--no-robot` | G1 に ssh しない（この操作PCの ROS 側だけ止める） |
| `--no-ros` | この操作PCの ROS 側に触らない（G1 側だけ止める） |

| 環境変数 | 既定 | 用途 |
|---|---|---|
| `G1_SSH` | `~/.ssh/config` に `g1` があれば `g1`、無ければ `unitree@192.168.123.164` | PC2 への ssh 先 |
| `G1_RVIZ_NAME` | `rviz` | 操作PCで ROS 2 を動かしている Docker コンテナ名 |
| `NAV_STOP_IFACE` | `col0` → `192.168.123.x` を持つ NIC → `lo` の順で自動選択 | 操作PCで DDS を載せる NIC |

⚠️ **ssh は鍵認証でパスワード無しに入れること**（`BatchMode=yes` で打つので、パスワードは聞かない）。
入れない PC では下の手順を手で打つ。

### スクリプトがすること

| 順 | 場所 | 対象 | 中身 |
|---|---|---|---|
| 1 | PC2 | **nav2_stable**（現行。`g1up.sh`） | `/g1/estop` に true → `/g1/patrol/stop` → `/g1/enable_navigation` false → Goal 全取消 → launch を停止 |
| 2 | PC2 | 旧 `loco_driver.py` / `cmd_vel_bridge.py` | **SIGINT** で停止（`StopMove()` を走らせる） |
| 3 | PC2 | G1 本体 | SDK から `SetVelocity(0, 0, 0)` を送り、残った速度指令を上書き |
| 4 | PC2 | 発進ゲート（`G1_ARM=--arm`） | 開いていれば閉じる。**`sudo` にパスワードが要るなら閉じられないので、コマンドを表示する** |
| 5 | 操作PC | 旧 `nav_stack.sh` 構成の Nav2 | Goal 取消 → `/cmd_vel` に 0 → 停止（Mac はコンテナ `rviz`、Ubuntu は `/opt/ros/humble`） |

## なぜ止まらないのか / 再起動しても歩き出すのか

1. **巡回モード（`patrol_ctl.sh start`）は巡回路を回り続ける。** Goal を1つ取り消しても、
   巡回ノードが次の点の Goal を送り直す。**止めるには `/g1/patrol/stop` が要る**
2. RViz で出した **Nav2 の Goal は、到達するか取り消すまで生きている**
3. **発進ゲートが開いたまま**（`/etc/default/g1-sdk-bridge` の `G1_ARM=--arm`）だと、
   `g1-sdk-bridge` が起動した瞬間に指令が足まで届く。2026-09-23〜24 の撤収時にも
   開けたまま終わっている（`../nav2_stable/HANDOVER.md` §5）
4. 旧 `loco_driver.py` は `Move(..., continous_move=True)`（duration=864000 秒）を使うので、
   **SIGTERM / SIGKILL で死ぬと最後の速度指令が G1 の中に 10 日間残る**（SIGINT なら `StopMove()` が走る）

## 手で止める場合

### G1 の PC2（Mac / Ubuntu 共通。ここがいちばん大事）

```bash
ssh g1          # ssh の設定が無い PC では ssh unitree@192.168.123.164
# ⚠️ ログイン時に「ros:foxy(1) noetic(2) ?」と聞かれたら **Enter だけ**を押す（ROS を入れない）
```

PC2 のシェルで:

```bash
# ① nav2_stable の ROS 環境に入る（g1up.sh / start_nav.sh と同じ）
cd ~/g1_nav2/pc2_humble && ~/.pixi/bin/pixi run bash -lc '
  source ~/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$HOME/g1_nav2/cyclonedds_eth0.xml
  ros2 topic pub --times 20 -r 10 /g1/estop std_msgs/msg/Bool "{data: true}"
  ros2 service call /g1/patrol/stop std_srvs/srv/Trigger "{}"
  ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: false}"
  ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'

# ② launch を止める（start_nav.sh と同じ対象）
pkill -INT -f 'ros2 launch g1_navigatio[n]'; pkill -INT -f 'g1_cmd_router_nod[e]'
pkill -INT -f 'patrol_nod[e].py'; pkill -INT -f 'envs/default/lib/nav[2]_'

# ③ 旧 loco_driver 構成が動いていれば SIGINT で止める（SIGTERM だと StopMove() が走らない）
pkill -INT -f 'loco_drive[r].py'; pkill -INT -f 'cmd_vel_bridg[e].py'

# ④ 発進ゲートを閉じる（パスワードを聞かれる）
grep '^G1_ARM=' /etc/default/g1-sdk-bridge        # G1_ARM=--arm なら開いている
sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge && sudo systemctl restart g1-sdk-bridge
pgrep -af g1_sdk_bridge_real_server               # 末尾に --arm が無ければ閉じている
```

### 操作PC：Mac

nav2_stable では、Mac は RViz と heartbeat を動かしているだけなので、PC2 を止めれば足りる。
旧 `nav_stack.sh` 構成（Nav2 を Mac のコンテナ `rviz` で動かす）のときだけ、次を行う:

```bash
cd G1_Hackason/Mapping/real
docker exec -u ubuntu rviz bash -c 'source /opt/ros/humble/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 && \
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"col0\"/></Interfaces></General></Domain></CycloneDDS>" && \
  ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'
bash quickstart/nav_stack.sh stop                  # Nav2・OctoMap・TF・bag 再生（RViz2 は残る）
bash quickstart/start_rviz_mac.sh stop             # コンテナごと止めるなら
```

offline（bag 再生）で起動している場合は、`col0` を `lo` に読み替える。

### 操作PC：Ubuntu

nav2_stable では PC2 を止めれば足りる。Ubuntu に直接入れた ROS 2 で Nav2 を動かしているときだけ:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
# NIC 名は `ip -br addr` で 192.168.123.x が付いているものに置き換える
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0"/></Interfaces></General></Domain></CycloneDDS>'
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
ros2 topic pub --times 5 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"
pkill -INT -f "navigation_launc[h]"; pkill -INT -f "nav2_[a-z]"
pkill -INT -f "octomap_serve[r]"; pkill -INT -f "odom_to_t[f]"; pkill -INT -f "ros2 bag pla[y]"
```

Nav2 を Docker（`Mapping/real/compose.yaml`）で動かしている場合は、`docker exec <コンテナ名> bash -c '...'` で包む。

📌 **操作PCの heartbeat（`g1_heartbeat_sender`）を止めても機体は止まる**（D-31。
`operator_lost` で `cmd_router` が止める）。ssh が通らないときの手段になる。ただし通信断の
検知に `operator_timeout_s`（既定 2 秒）かかり、**止めた後も約 2 秒は歩き続ける**。

## 止まったことの確認

PC2 の ROS 環境（上の ① と同じ）で:

```bash
ros2 topic echo /g1/bridge_status --once           # E_STOP / STANDBY / DISCONNECTED のどれかであること
ros2 topic echo /navigate_to_pose/_action/status --once   # status: 2 (EXECUTING) が無いこと
ros2 topic info /cmd_vel --verbose                 # Publisher count: 0 であること
```

## 再開するとき

`nav_stop.sh` は **E_STOP をラッチしている**（自動では解除されない設計）。再開は runbook の手順で:
`g1up.sh` で上げ直す（launch を止めたので）→ ゲートを開ける → `enable_navigation`。
launch を止めずに E_STOP だけ解除したい場合は、`/g1/estop` に false を送ってから
`ros2 service call /g1/clear_estop std_srvs/srv/Trigger "{}"`。

## それでも歩き出す場合

- **別の PC で Nav2 が動いている。** G1 の内蔵スイッチには他の作業者も繋がっている。
  `ros2 topic info /cmd_vel --verbose` の Publisher のノード名から特定し、その PC でも `nav_stop.sh` を実行する
- 純正ナビ（`../nav/mission.py`、`slam_operate` の 1102）を使っていた場合は、PC1 に 1102 のタスクが残る。
  これは `nav_stop.sh` の対象外。1901（SLAM 終了）を送って破棄する（`../nav/protocol.py` の `close_slam_request`）
- 旧 `loco_driver.py` の根本対策は、D-27（`../nav2_stable/Planning.md`）どおり
  `SetVelocity(vx, vy, vyaw, 0.2)` に直すこと（未対応。nav2_stable の `g1-sdk-bridge` は対応済み）
