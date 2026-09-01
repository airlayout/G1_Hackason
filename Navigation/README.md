# Navigation（作成済み地図を使った自律移動）

`Mapping/`が作成した地図（`.pcd`）を読み込んで自己位置を合わせ、目標地点へG1を移動させる機能。
巡回ルートの管理もここに含む。

Unitree純正のSLAM/ナビサービス（`slam_operate`）に乗る方針。自前でNav2を組む方針から
2026-08-28に変更した。理由は`../CLAUDE.md`および下記「純正APIの制約」を参照。

## `Mapping/`との役割分担

境界は**`.pcd`ファイル**。

| フォルダ | 担当API | 成果物 |
|---|---|---|
| `Mapping/` | 1801（建図開始）/ 1802（建図終了・保存） | `.pcd`ファイル |
| `Navigation/`（ここ） | 1804（地図読込＋自己位置設定）/ 1102（移動）/ 1201・1202（一時停止・再開） | 移動指示 |

自己位置推定（1804）と移動（1102）は純正では**同じ`slam_operate`サービス・同じ地図ファイル・
同じ状態トピック`rt/slam_info`**を使うため、「SLAM」と「Navigation」には分割しない。
（旧`SLAM/`フォルダはこのフォルダにリネームした）

## 構成

- `sim/` — `slam_operate`のモックに対する検証。実機なしでルート分割器やUIを開発するために使う
- `real/` — 実機G1に対して1804/1102を投げる実装

## 環境

`G1_HuggingFace/venv/`（操作PC側）・G1本体側のPython 3.12 conda環境（`lerobot`）を
共通で使う想定。ネットワーク接続・疎通確認は`Common/network/`を参照。

**Dockerは使わない。** `slam_operate`はDDSのAPI-IDにJSONを投げるだけなので、
`unitree_sdk2py`から直接叩けてROS 2が要らない。`Mapping/`がDockerを使うのは
LiDAR点群の高レート購読とFAST-LIO2にROS 2が必要だからで、Navigationには該当しない。

`.env`（G1のIP・NIC名・`ROS_DOMAIN_ID`）は`Mapping/real/.env`と共有する。二重に持たない。
設定読み込み・疎通確認・`runs/`への記録は当面`Mapping/real/python/g1_mapping`の
`config` / `doctor` / `session`をimportして使い、共通レイヤの`Common/`への切り出しは
本フォルダが動いてからの課題とする。

### `g1_mapping`への依存範囲

依存してよいのは`config` / `doctor` / `session`の**3モジュールだけ**。
`mapctl`本体・ROS2ワークスペース・Dockerまわりは`Mapping/`班の内側であり、
予告なく変わる前提なのでimportしない。

この3つは**班をまたぐインターフェース＝契約**として扱う。シグネチャの変更が必要に
なったらNavigation班だけで判断せず、Mapping班と合意してから変える。

**注意: この依存は`Mapping/README.md`には書いていない**（2026-08-28時点）。
Mapping班はNavigationから参照されていることを知らないため、**Mapping側の変更で
こちらが壊れる可能性が残っている**。実際に壊れたら、その時点で`Mapping/README.md`にも
依存を明記すること。

## 純正API（`slam_operate` v1.0.0.1）

unitree_sdk2のAPI-ID方式。リクエスト/レスポンスとも JSON。

**出典（2026-09-01に本文を確認）**: G1 SDK 開発ガイド > 软件服务接口 > SLAM导航服务接口
（公式ドキュメント更新日 2026-07-20）
- 中文: https://support.unitree.com/home/zh/G1_developer/slam_navigation_services_interface
- English: https://support.unitree.com/home/en/G1_developer/slam_navigation_services_interface

> 取得方法の注意: このサイトはTencent Cloud EdgeOneのWAF配下で、curlもヘッドレス
> ブラウザもHTTP 567で遮断される。**GUI Chrome（Playwright headful等）でのみ本文が取れる。**
> Wayback MachineにもURLは残っているがSPAシェルのみで本文は無い。

### 前提条件

| 項目 | 内容 |
|---|---|
| **必要ファームウェア** | **1.5.3 以上** |
| **自動起動** | **しない。** APIを呼ぶか **[APP]-[服务状态]** から導航関連サービスを開始する |
| 前提サービス | G1の`unitree_slam`と`lidar_driver`が起動していること（Appで確認） |
| **併用禁止** | **APP上の導航機能と同時に使わないこと** |
| 位置づけ | 公式に「教育科研行業向け。行業応用には非推奨」と明記 |

ネットワーク構成（公式記載）:

| 機器 | IP |
|---|---|
| NX開発ボード | `192.168.123.164`（ssh `unitree` / 初期パスワード `123`） |
| **LiDAR** | **`192.168.123.120`** |
| 運控PC | `192.168.123.161` |

座標系: SLAMが出力する点群・定位情報の**原点はMid360-IMU座標系の原点**。
X軸正方向＝機体正面、Z軸正方向＝鉛直上向き。

### API一覧

| API ID | 機能 | 主なパラメータ |
|---|---|---|
| 1801 | 建図開始 | `slam_type: "indoor"`（固定値） |
| 1802 | 建図終了・保存 | `address: "/home/unitree/test.pcd"` |
| 1804 | 初期位姿（保存地図の読込＋自己位置設定） | `address` + `x,y,z` + 四元数 `q_x,q_y,q_z,q_w` |
| 1102 | 位姿導航 | `targetPose`(x,y,z+四元数) / **`mode: 1`（固定値）** |
| 1201 / 1202 | 一時停止 / 再開 | なし（`{"data":{}}`） |
| 1901 | SLAM終了 | なし（`{"data":{}}`） |

レスポンスは全API共通:

```json
{ "succeed": true, "errorCode": 0, "info": "", "data": {} }
```

`1802`の注意: ディスク圧迫を避けるため、公式は**ファイル名を`test1.pcd`〜`test10.pcd`に
統一して上書き保存すること**を推奨している。

### トピック

| トピック | 型 | 用途 |
|---|---|---|
| `rt/unitree/slam_mapping/{points,odom}` | `PointCloud2_` / `Odometry_` | 建図中 |
| `rt/unitree/slam_relocation/{points,odom}` | `PointCloud2_` / `Odometry_` | **定位中** |
| **`rt/unitree/slam_relocation/global_map`** | `PointCloud2_` | **全体地図。定位開始後に1回だけ送信** |
| `rt/slam_info` | `String_` | 状態ブロードキャスト（JSON） |
| `rt/slam_key_info` | `String_` | 実行結果フィードバック（JSON） |

`global_map`が**Navigationが地図を受け取る正規の経路**。1804の後に1回だけ飛んでくる。

#### `rt/slam_info` は3種類ある（`type`で判別）

| `type` | 中身 | Navigationでの用途 |
|---|---|---|
| `robot_data` | `motorTemp[]` `motorError[]` `batteryPower` `cpuTemp` `cpuUsage` 等 | 健全性監視 |
| `pos_info` | **`currentPose`**(x,y,z+四元数)、`pcdName`、`address` | **自己位置**（建図中は`mapping_info`） |
| `ctrl_info` | `targetNodeName` `is_arrived` `startPose` `targetPose` `stateMachine` `obsInfo` `progress` | **進捗・障害物・状態機械** |

`ctrl_info`の内訳:
- `stateMachine`: `state`(例 `"follow"`) / `isOpenPlan` / `isBack` / `isClimbStairs` /
  `isRotate` / `isPause` / `ctrName`(例 `"pid"`) / `vx` `vy` `vyaw`
- `obsInfo`: `state`（障害物の有無）/ `time`（遭遇時間 秒）
- `progress`: `used_time` / `last_time` / `completion_percentage`
- `startPose` / `targetPose` は **roll/pitch/yaw**（1102の入力は四元数。単位系が違う）

#### `rt/slam_key_info` — `type: "task_result"`

`targetNodeName` と `is_arrived`。**1回の1102タスクの完了通知。到達判定はこれを待つのが正。**

### 純正APIの制約（設計に直結する）

- **1回の指示は10m以内、かつ直線移動。** Nav2のような全体経路計画ではない。
  任意の地点へ行くには、**経路を10m以内の直線区間に分割して1102を連打する
  ラッパを自前で書く**必要がある。これが本フォルダの中心的な実装物。
- **`mode`は`1`固定値。绕障（回避）モードはG1には無い。** 障害物に遭遇したら停止する。
  `obsInfo.state`と`obsInfo.time`を監視して、一定時間ブロックされたらリルートor中断する
  制御を自前で書く必要がある。
- **`speed`パラメータは存在しない。** 速度は指定できない。
- **障害物の高さは50cm以上ないとLiDARが検知しない。**
- 適用範囲: **X軸/Y軸ともに45m未満**・特徴が豊富・**静的**・屋内・平地。
  大範囲地図は計算資源を食い基礎運控サービスに影響するため、範囲を超えないこと。
- **激しい動作は定位ロストの原因になる。**

### 2026-09-01の訂正: 以前の「G1用ドキュメントは存在しない」は誤り

このREADMEには以前、次の記述があったが**誤りだったので削除した**。

> 公式ドキュメントには「対応機種はGo2とGo2_W」と明記されており、
> G1開発者ガイドにSLAM/ナビのページは存在しない

実際には**G1 SDK開発ガイドの「软件服务接口」配下に`SLAM导航服务接口`が存在する**。
参照していたURLが`/home/en/developer/SLAM and Navigation_service`（**Go2の開発ガイド配下**）
であり、`/home/{zh,en}/G1_developer/slam_navigation_services_interface`に到達していなかった。
加えてWAFとSPAで本文が取れず、確認が完了しないまま結論を書いていた。

**教訓: 「ドキュメントが無い」は「見つけられなかった」と区別して書く。**
本文が取得できていない時点の推測を、確定した事実として残さない。

上記の数値はすべてG1公式の値に差し替え済み。参考までに、削除したGo2版の値との差分:

| 項目 | 旧記述（Go2版） | **G1公式** |
|---|---|---|
| 適用範囲 | 25m×25m未満 | **X/Y軸45m未満** |
| 障害物の検知下限 | 高さ20cm | **高さ50cm** |
| `mode` | 1=停障 / 0=绕障 | **1固定（绕障なし）** |
| `speed` | 0.2〜0.8 m/s | **パラメータ自体が無い** |
| ファームウェア | ≥1.1.7 | **≥1.5.3** |
| LiDAR IP | 192.168.123.20 | **192.168.123.120** |

なお`Mapping/FIELD_RUNBOOK.md`が「`.120` `.161` `.164` `.200`の4台が応答した」と
記録しているのは、公式構成の**`.120`＝LiDAR、`.161`＝運控PC、`.164`＝NX開発ボード**だった。

## 進め方

1. `sim/`でモック相手にロジック（ルート分割器・巡回管理）を作る
2. `real/`で実機G1に接続し、同じロジックが実機でも動くか確認

`Mapping/`が既に1801/1802を通しているので、**未検証なのは1804と1102の2つ**。

優先順:
1. **実機のファームウェアが1.5.3以上か確認** — 満たさないと1804/1102は動かない
2. **導航サービスを起動する** — 自動起動しないため。`Mapping/`の`doctor`は
   `slam_info`の応答しか見ていないので、これだけでは起動判定にならない
3. **1804で保存済み地図に自己位置合わせできるか** — ここが通らないとナビが成立しない最大の関門。
   成功すれば`slam_relocation/global_map`が1回飛んでくるはず
4. **1102に任意座標を投げて到達するか**、`slam_key_info`の`is_arrived`が返るか
5. G1での実際の挙動を実測（10m制限、障害物停止からの復帰、`completion_percentage`の精度）

`Mapping/`の実測地図は43.0m×38.6m×3.8mで、公式の適用範囲（X/Y軸45m未満）に
**ギリギリ収まっている**。これ以上広い空間を建図すると適用外になる。

失敗した内容は`FAILURES.md`に記録する。

## 状態

未着手。
