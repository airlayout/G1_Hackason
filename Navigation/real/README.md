# Navigation / real

実機 G1 上での実行コードを置く。

⚠️ **いま 2 つの方式が同じ木の中に同居している。** どちらを触っているのか取り違えやすいので、
まずここを読むこと。**このフォルダにあるのは Nav2 方式のブリッジだけ**で、純正方式の実装は
`../nav/` にある。

## 2 つの方式

| | 純正方式 | Nav2 方式 |
|---|---|---|
| 実装 | **`../nav/`**（`mission.py` / `route.py` / `transport.py`） | **`./`（このフォルダ）** ＋ `../nav2/g1_nav2.yaml` |
| 移動指令 | `slam_operate` の `1102` を連打 | `/cmd_vel` |
| 自己位置 | 純正 `1804`（保存地図の読込＋自己位置設定） | 地図への ICP 一発合わせ |
| 経路計画 | `route.py`（A\* ＋ 8m 分割） | Nav2 の `planner_server` |
| 障害物 | **純正は停止するだけ**。`mission.py` が止まったのを見て迂回路を引き直す | **Nav2 が止まらずに避ける** |
| 一時停止 | `1201` / `1202` | 指令を止めれば `loco_driver.py` が止める |
| 検証済み | sim（`./navctl sim`。169 テスト） | 実機で指令が出るところまで（足は繋いでいない） |

**なぜ 2 つあるか。** G1 純正のナビには障害物回避が無い（`mode` は 1 固定で、遭遇したら
停止するだけ）。速度も指定できない。**回避が要るなら純正の外で作るしかない**ので Nav2 を載せた。
一方、純正の `1804` は保存地図の座標系で自己位置を出すので、**こちらの ICP の手作業を
置き換えられる可能性がある**。どちらに寄せるかはまだ決めていない。

## このフォルダのファイル

| ファイル | 側 | 役割 |
|---|---|---|
| `cmd_vel_bridge.py` | **ROS 側** | `/cmd_vel` を購読して UDP へ中継するだけ |
| `loco_driver.py` | **SDK 側** | UDP を受けて `LocoClient` に渡す。**安全機構は全部こちらにある** |

**なぜ 2 プロセスに分かれているか。** `rclpy` と `unitree_sdk2py` が同居できないため
（PC2 では pixi の 3.11 に rclpy だけ、system の 3.8 に SDK だけが入る）。
localhost の UDP で繋いでいる。**安全機構は「止められる側」＝ SDK 側に置く**という決めなので、
`cmd_vel_bridge.py` を落としても `loco_driver.py` が指令切れを検知して停止する。

### 安全機構（`loco_driver.py`）

- 速度を頭打ちにする（実測: `vx=-0.5,vy=0.9,vyaw=9.0` を投げると `vx=0.000 / vy=0.200 / vyaw=0.500`）
- **0.50 秒 指令が来なければ停止する**

`.env` は `../../Mapping/real/.env` を共有する。

## 動かす

Nav2 方式は Mapping 側の道具立てと組で動く。手順は
`../../Mapping/real/quickstart/` の `nav_stack.sh` / `start_rviz_mac.sh` を見ること。

```bash
# 実機なしで（記録の再生で立ち上げる）
bash ../../Mapping/real/quickstart/start_rviz_mac.sh offline
bash ../../Mapping/real/quickstart/nav_stack.sh

# 実機あり
bash ../../Mapping/real/quickstart/start_rviz_mac.sh
bash ../../Mapping/real/quickstart/nav_stack.sh live
```

⚠️ **SLAM を再起動するたびに位置合わせは無効になる**（原点が変わる）。
`capture_slam_cloud.py` → `align_to_map.py` → `static_transform_publisher` をやり直すこと。

純正方式は `../navctl sim` で sim を回せる。実機に投げる入口は `../nav/mission.py`。
