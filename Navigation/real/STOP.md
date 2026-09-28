# Navigation を止める（Nav2 方式）

G1 が **Nav2 で指定した経路を勝手に歩き始める**ときの止め方。Mac / Ubuntu のどちらの PC からでも行える。

> **Claude Code を使っている場合は「Navigationを止めて」と言えばよい。**
> `.claude/skills/stop-navigation/` のスキルが `nav_stop.sh` を実行する。

## まず：足を止める（人の手で）

**リモコンでダンピング（L2+B）にする。** 脱力して倒れるので、**必ず支えるか吊ってから**押すこと。
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
| （なし） | G1 側 → ROS 側の順に全部止める |
| `--no-robot` | G1 に ssh しない（ROS 側だけ止める） |
| `--no-ros` | ROS 側に触らない（G1 側だけ止める） |

| 環境変数 | 既定 | 用途 |
|---|---|---|
| `G1_SSH` | `~/.ssh/config` に `g1` があれば `g1`、無ければ `unitree@192.168.123.164` | PC2 への ssh 先 |
| `G1_RVIZ_NAME` | `rviz` | ROS 2 を動かしている Docker コンテナ名 |
| `NAV_STOP_IFACE` | `col0` → `192.168.123.x` を持つ NIC → `lo` の順で自動選択 | DDS を載せる NIC |

⚠️ **ssh は鍵認証でパスワード無しに入れること**（スクリプトは `BatchMode=yes` で打つ。パスワードは聞かない）。
入れない PC では下の手順を手で打つ。

## なぜ再起動しても歩き出すのか

1. RViz で出した **Nav2 の Goal は、到達するか取り消すまで生きている**
2. Nav2 は G1 の**外**（Mac のコンテナ / Ubuntu PC）で動いている。**G1 を再起動しても Nav2 は止まらない**
3. G1 が起き直すと odom と LiDAR が戻り、Nav2 は同じ Goal へ経路を引き直して `/cmd_vel` を出す
4. PC2 の `loco_driver.py --arm` がランニングモードの足へ渡し、**前回の経路の続きを歩き出す**
5. さらに `loco_driver.py` は `Move(..., continous_move=True)`（duration=864000 秒）を使うので、
   **ドライバが SIGTERM / SIGKILL で死ぬと最後の速度指令が G1 の中に 10 日間残る**
   （SIGINT なら `finally` の `StopMove()` が走る）

## 手で止める場合

### Mac（`start_rviz_mac.sh` のコンテナ `rviz` で Nav2 を動かしている）

```bash
cd G1_Hackason/Mapping/real

# ① G1 側（下の「共通」）を先に行う

# ② Goal を取り消す
docker exec -u ubuntu rviz bash -c 'source /opt/ros/humble/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 && \
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"col0\"/></Interfaces></General></Domain></CycloneDDS>" && \
  ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'

# ③ Nav2・OctoMap・TF・bag 再生を止める（RViz2 は残る）
bash quickstart/nav_stack.sh stop
# コンテナごと止めるなら
bash quickstart/start_rviz_mac.sh stop
```

offline（bag 再生）で起動している場合は、`col0` を `lo` に読み替える。

### Ubuntu（ROS 2 Humble を直接入れている）

```bash
# ① G1 側（下の「共通」）を先に行う

# ② ROS 2 の環境。NIC 名は `ip -br addr` で 192.168.123.x が付いているものに置き換える
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0"/></Interfaces></General></Domain></CycloneDDS>'

# ③ Goal を取り消し、速度 0 を流す
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
ros2 topic pub --times 5 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"

# ④ Nav2 一式を止める
pkill -INT -f "navigation_launc[h]"; pkill -INT -f "nav2_[a-z]"
pkill -INT -f "octomap_serve[r]"; pkill -INT -f "odom_to_t[f]"; pkill -INT -f "ros2 bag pla[y]"
```

Nav2 を Docker（`Mapping/real/compose.yaml`）で動かしている場合は、④ を
`docker exec <コンテナ名> bash -c '...'` で包む。

### 共通：G1（PC2）側

```bash
# SIGINT で止める（SIGTERM だと StopMove() が走らない）
ssh g1 'pkill -INT -f "loco_drive[r].py"; pkill -INT -f "cmd_vel_bridg[e].py"'

# G1 の中に残った速度指令を 0 で上書きする
ssh g1 'python3 -c "
import sys; sys.path.insert(0, \"/home/unitree/unitree_sdk2_python\")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
ChannelFactoryInitialize(0, \"eth0\"); c = LocoClient(); c.SetTimeout(5.0); c.Init()
print(c.SetVelocity(0.0, 0.0, 0.0, 1.0))"'

ssh g1 'pgrep -af "loco_drive[r]|cmd_vel_bridg[e]"'   # 何も出なければ OK
```

`ssh g1` の設定が無い PC では `ssh unitree@192.168.123.164` に読み替える。

## 止まったことの確認

ROS 2 の環境で（Mac はコンテナ内で）:

```bash
ros2 topic echo /navigate_to_pose/_action/status --once   # status: 2 (EXECUTING) が無いこと
ros2 topic info /cmd_vel --verbose                         # Publisher count: 0 であること
```

## それでも歩き出す場合

- **別の PC で Nav2 が動いている。** G1 の内蔵スイッチには他の作業者も繋がっている。
  `ros2 topic info /cmd_vel --verbose` の Publisher のノード名から特定し、その PC でも `nav_stop.sh` を実行する
- 純正ナビ（`../nav/mission.py`、`slam_operate` の 1102）を使っていた場合は、PC1 に 1102 のタスクが残る。
  これは `nav_stop.sh` の対象外。1901（SLAM 終了）を送って破棄する（`../nav/protocol.py` の `close_slam_request`）
- 根本対策は `loco_driver.py` を D-27（`../nav2_option/Planning.md`）どおり
  `SetVelocity(vx, vy, vyaw, 0.2)` に直すこと（未対応）
