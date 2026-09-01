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

- `nav/` — sim/real 共通のロジック本体（プロトコル・ルート分割器・ミッション状態機械）
- `sim/` — `slam_operate`のモックと運動学シミュレーション。実機なしで開発・検証する
- `real/` — 実機G1に対して1804/1102を投げる実装

詳細は「[Navigationの実装方針](#navigationの実装方針)」を参照。

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

## G1のハードウェア構成とネットワーク

Navigationは`slam_operate`を叩くだけだが、**どのコンピュータが何をやっているか**を
理解していないと、実行場所の選択もトラブル切り分けもできない。

出典: **G1 SDK 开发指南 > 应用开发 > 软件架构说明**（公式ドキュメント更新 2025-02-08）
- 中文: https://support.unitree.com/home/zh/G1_developer/architecture_description
- English: https://support.unitree.com/home/en/G1_developer/architecture_description
- **公式アーキテクチャ図（直リンク）**:
  https://doc-cdn.unitree.com/static/2024/9/18/d59f65351c874b0891ed9766aa813e1a_8000x6106.jpg

![Unitree G1 システムアーキテクチャ図](https://doc-cdn.unitree.com/static/2024/9/18/d59f65351c874b0891ed9766aa813e1a_8000x6106.jpg)

> 図はUnitree公式ドキュメントの著作物。本リポジトリはpublicなので複製を置かず、
> 公式CDNへのリンクで表示している。CDN側でURLが変われば表示されなくなるため、
> その場合は上記の出典ページから辿り直すこと。

公式図の注記（原文）:

> 注1: 功能模块包括**避障路径规划**、语音识别等。
> 注2: DDS兼容ROS2，可直接访问。
> 注3: 用户可在G1机载PC或者外部开发板/PC部署程序，进行二次开发。

**注1が重要。「功能模块」に「避障路径规划（障害物回避経路計画）」が含まれる**と明記されている。
つまり**ナビゲーションはPC1側の機能**であり、こちらが実装するのは「10m以内に分割して
1102を連打するラッパ」だけでよい。

### 呼び名の対応

ドキュメントによって呼び名が違うので対応させておく。

| 軟件架构説明 | SLAM導航服務接口 | IP | 実測した中身（2026-09-01） |
|---|---|---|---|
| **PC1** | **运控PC** | `192.168.123.161` | クローズド。**TCP全閉・SSH不可** |
| **PC2** | **NX开发板** | `192.168.123.164` | Jetson / Ubuntu / ROS2 foxy+noetic / **SSH可** |
| — | LiDAR | `192.168.123.120` | Livox MID-360。UDPのみ |
| **外部开发板/PC** | — | `192.168.123.X` | 本書では **Client PC** と呼ぶ |

### ネットワーク構成図

```text
                        ┌──────────────── G1 本体 ────────────────┐
                        │  ┌────────────────────────────────────┐  │
                        │  │ PC1 = 运控PC   192.168.123.161     │  │
                        │  │  ・运动控制（バランス・歩容・loco） │  │
                        │  │  ・功能模块（避障路径规划＝ナビ）   │  │
                        │  │  ・SLAM / slam_operate             │  │
                        │  │  ・モータ/センサへシリアル直結      │  │
                        │  │  ※クローズド。TCP全閉、SSH不可     │  │
                        │  └──────────────┬─────────────────────┘  │
                        │                 │ DDS                    │
   ┌─────────────────┐  │        ┌────────┴────────┐               │
   │  Client PC      │  │        │    交换机        │               │
   │ 192.168.123.X   │◄─┼───────►│  (内蔵スイッチ)  │               │
   │ (X≠120,161,164) │  │  有線   └────────┬────────┘               │
   │                 │  │  LAN            │ DDS                    │
   │ unitree_sdk2py  │  │        ┌────────┴──────────────────────┐ │
   │ ROS2/CycloneDDS │  │        │ PC2 = NX开发板 192.168.123.164│ │
   └─────────────────┘  │        │  ・ユーザーが自由に使える開発機│ │
     ※外部开发板/PC     │        │  ・Ubuntu / ROS2 foxy+noetic  │ │
                        │        │  ・unitree_sdk2py / lerobot   │ │
                        │        │  ・SSH可                      │ │
                        │        └───────────────────────────────┘ │
                        │                 │                        │
                        │        ┌────────┴─────────────┐          │
                        │        │ LiDAR (Livox MID-360)│          │
                        │        │  192.168.123.120     │          │
                        │        └──────────────────────┘          │
                        └──────────────────────────────────────────┘
```

Client PCはG1内蔵スイッチに有線でぶら下がるだけ。`192.168.123.0/24`の空きアドレスを
1つ取れば、PC1・PC2・LiDARすべてと同じDDSネットワークに入る。

**2026-09-01の実測では、リンク上は`.120` `.161` `.164` ＋Client PCの4台のみだった。**
（`Mapping/FAILURES.md`は「`.200`が使用中」と記録しているが、当時それはClient PC自身だった
可能性が高い）

### SDKを使ってもPC1は必ず経由する

**SDKはクライアント側のライブラリでしかない。** モータはPC1にシリアル直結されているので、
どの方式を使ってもPC1を通る。

| やること | SDKを置く場所 | **実際に処理する場所** |
|---|---|---|
| `slam_operate` 1801〜1901 | Client PC or PC2 | **PC1** |
| `LocoClient.Move()` | Client PC or PC2 | **PC1** |
| `rt/lowcmd`（低レベル制御） | Client PC or PC2 | **PC1経由でモータへ** |

低レベル制御時に`MotionSwitcherClient.ReleaseMode()`で解放するのは
**PC1上で動く高レベルコントローラ「ソフトウェア」**であって、PC1というハードウェアを
バイパスするわけではない。

### Navigationの実行場所をどこにするか

**SDKはClient PCでもPC2でも動く。** これは設計上の選択。

| 置き場所 | 利点 | 欠点 |
|---|---|---|
| **Client PC** | 開発が速い。手元でログが見える | Ethernetが経路に入る |
| **PC2（NX开发板）** | **機体内で完結。ケーブル不要** | デプロイの手間、ログ回収 |

1102はPC1内でバランス制御が閉じているので、Client PC実行でもケーブルが抜けて即転倒は
しない。ただし**巡回中に抜けると次の区間を投げられなくなる**ので、
実運用では**PC2へ載せるのが筋**。

なお**PC2のROS 2 foxyからは`ros2 topic list`でUnitreeのトピックが見えない**
（`rmw_cyclonedds_cpp`指定・`--no-daemon`でも空）。公式も「DDS IDL 兼容ROS2
（**需要选择适配的RMW**）」と条件付きで書いている。`unitree_sdk2py`の`ChannelSubscriber`で
型を明示した直接DDS購読は成功するので、**PC2で実装するならROS 2 CLIに頼らない**こと。

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

## 実測で確定した挙動（2026-09-01・実機Client PCから採取）

**モックはこの節を仕様書として実装する。** 公式ドキュメントに載っていない情報を含む。

### 待機時（1804を投げる前）の状態

```json
{ "type": "ctrl_info", "errorCode": 0, "info": "not init",
  "data": { "stateMachine": { "state": "ready", "ctrName": "not init",
                              "isOpenPlan": false, "isPause": false, "isBack": false,
                              "isRotate": false, "isClimbStairs": false,
                              "vx": 0.004, "vy": 0.005, "vyaw": 0.065 },
            "currentPose": {"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},
            "is_arrived": false, "targetNodeName": 0,
            "obsInfo": {"state": false, "time": 0.0},
            "progress": {"used_time":0.0,"last_time":0.0,"completion_percentage":0.0},
            "total_distance": -1.0 } }
```

- `info` と `ctrName` が **`"not init"`** ＝ 1804 未実行。`state` は `"ready"`
- **`total_distance`（待機時 `-1.0`）は公式ドキュメントに無いフィールド**。おそらく経路総距離
- 静止中でも `vx/vy/vyaw` は 0 にならず、数mm/s〜0.07rad/s の微小値が乗る
  → **速度が0かどうかで停止判定してはいけない**

### 配信レート（実測）

| トピック / type | レート |
|---|---|
| `rt/slam_info` の `ctrl_info` | **約 5 Hz** |
| `rt/slam_info` の `robot_data` | **約 0.5 Hz** |
| `rt/slam_key_info` | **タスク実行時のみ**（待機中は0件） |
| `rt/unitree/slam_mapping/points` | 建図中のみ（待機中は0件） |
| `rt/unitree/slam_relocation/points` | 定位中のみ（待機中は0件） |
| `rt/utlidar/cloud_livox_mid360` | 10 Hz（LiDARドライバ。SLAMとは独立に常時） |

### `robot_data` の実測値

モータ **29本**（G1の29DoF構成）、`motorError` 全0、`sportMode`/`gaitType` は
どちらも `-1`（公式ドキュメント通り「暂无」＝未実装）。

### 1804 のエラー挙動 ← モック実装で重要

`address` を変えて5パターン試した結果、**すべて同一のエラー**が返る。

| 渡した `address` | code | errorCode | info |
|---|---|---|---|
| 実在するPCD（PC2上に配置） | 1 | 507 | `Load pcd failed.` |
| 存在しないパス | 1 | 507 | `Load pcd failed.` |
| 存在しないディレクトリ | 1 | 507 | `Load pcd failed.` |
| 空文字 `""` | 1 | 507 | `Load pcd failed.` |
| 権限のないパス | 1 | 507 | `Load pcd failed.` |

```json
{"succeed":false,"errorCode":507,"info":"Load pcd failed.","data":{}}
```

- **`errorCode 507` は総称エラー。ファイル不在・形式不正・権限エラーを区別できない**
- 応答は **0.01秒**。定位マッチングに入る前に落ちている
- 失敗しても `state` は `"ready"` のまま。**サービスは異常状態にならない**（リトライ可能）
- RPCの戻り値 `code` は 1（レスポンスJSONの `errorCode` とは別物）

### `address` は PC1 のファイルシステムを指す

**PC2（`.164`）にPCDを置いても1804は読めない。** SLAMはPC1で動いているため。

| 調査 | 結果 |
|---|---|
| PC2 の slam バイナリ | 存在しない（全FS検索） |
| PC2 の slam プロセス | 無い（`key_server`/`master_service`/`ota_pipe`/`video_hub` のみ） |
| ネットワーク共有マウント | 無い |
| PC1 のTCPポート | 主要18ポート全て閉 |

**したがって地図をPC1へ転送する手段は無い。** ただし困らない。

```text
1802 で保存  →  PC1 の /home/unitree/xxx.pcd に書かれる
1804 で読込  →  PC1 の同じファイルを読む        ← 転送不要
1102 で移動
```

**地図はロボット内で完結する。** 外部で作った地図を持ち込む発想自体が不要。

> ⚠️ **Mapping班への影響**: `Mapping/real/.env` は `G1_HOST=192.168.123.164`（PC2）から
> PCDをscp回収する設計だが、1802が書くのはPC1。**通信が切れなくても自動回収は失敗する。**
> ただしナビには影響しない（地図はPC1に残るため）。持ち帰り用のコピーは
> `mapctl rebuild`（rosbagからの再構成）でまかなえる。

## Navigationの実装方針

### 何を作るのか（スコープ）

公式アーキテクチャ図の注1により、**障害物回避経路計画はPC1側の機能**。
`mode:1` 固定で绕障モードも無く、`speed` も指定できない。
つまりこちらが作るのは次の3つだけ。

1. **ルート分割器** — 目標地点までの経路を **10m以内の直線区間**に分割する
2. **ミッション実行器** — 1804 → 1102連打 → `is_arrived`待ち の状態機械
3. **巡回管理** — 複数ウェイポイントの順序・繰り返し・中断/再開

経路計画そのものも、速度制御も、バランス制御も**作らない**。

### mock と sim の役割分担

同じ `mission` コードを、差し替え可能な3つの相手に対して流す。

| 段階 | 相手 | 検証できること | 検証できないこと | 動かす場所 |
|---|---|---|---|---|
| **mock** | プロセス内の偽サービス | **プロトコル**: JSON整合、状態遷移、`is_arrived`待ち、507のリトライ、pause/resume | 幾何、物理 | どこでも（numpyのみ） |
| **sim** | mock + 運動学 + `sim_room.pcd` | **幾何と挙動**: 分割が壁を貫かないか、到達判定の閾値、巡回の所要時間 | 実機の定位精度、実際の歩行 | どこでも |
| **real** | 実機PC1 | 全部 | — | Client PC / PC2 |

**mockとsimは別プログラムではない。** 同じ偽サービスに運動学モデルと地図を足したものがsim。
`--kinematics` で切り替える。

### なぜ最初にDDSを使わないか

**Client PC（Mac）には`cyclonedds`も`unitree_sdk2py`も入らない**（pipが無い）。
一方、ロジックのバグの大半はDDSと無関係な場所（分割の境界条件、状態遷移、待ち合わせ）に出る。

そこで**トランスポートを抽象化**し、3段階で差し替える。

```text
mission.py ──> SlamTransport（抽象）
                   ├─ FakeTransport   … プロセス内。依存numpyのみ。Macで動く
                   ├─ DdsMockTransport … PC2上で別ROS_DOMAIN_IDで起動。本物のDDSを通る
                   └─ RealTransport    … 実機PC1。unitree_sdk2py の Client
```

DDSモックをPC2で動かすときは、**必ず`ROS_DOMAIN_ID`を0以外にする**こと。
0のままだと本物の`slam_operate`と同じDDSネットワークに出て衝突する。

### ディレクトリ構成

```text
Navigation/
├── nav/                     sim/real 共通のロジック（ここが本体。標準ライブラリのみ）
│   ├── geometry.py          ✅ 平面姿勢と四元数↔yaw変換
│   ├── protocol.py          ✅ 1804/1102/slam_info/slam_key_info のJSONスキーマと定数
│   ├── occupancy.py         ✅ 地図PCD → 2D占有格子（直線が歩けるかの判定に使う）
│   ├── route.py             ✅ ルート分割器（迂回経路 + 10m以内の直線区間へ）
│   ├── transport.py         ✅ SlamTransport 抽象 + RealTransport（実機部分は未検証）
│   └── mission.py           ✅ ミッション実行の状態機械
├── tests/                   ✅ 実機・PC2・DDS・numpyのいずれも不要
│   ├── run_tests.sh         `scripts/ci/run_all_tests.sh` が自動で見つける
│   ├── test_geometry.py
│   ├── test_protocol.py
│   ├── test_occupancy.py
│   ├── test_route.py
│   └── test_mission.py
├── sim/
│   ├── npz_to_pcd.py        ✅ scans.npz → 地図PCD + 真値軌跡
│   ├── maps/                （.gitignore済み。npzから再生成できる）
│   ├── fake_service.py      ✅ 偽slam_operate（運動・障害物・507注入。標準ライブラリのみ）
│   └── run_sim.py           ✅ シナリオ実行と評価（ASCII地図つき）
└── real/
    └── run_real.py          実機に対して同じmissionを流す
```

**`nav/` は標準ライブラリだけで書く。** `scripts/ci/run_all_tests.sh` は `pip install` を
持たず、`Mapping/` も同じ理由でnumpy非依存になっている。numpyが要る処理（点群の
大量変換など）は `sim/` 側に置く。

### 段取り

| Phase | 内容 | 実機 | 状態 |
|---|---|---|---|
| **0** | 地図の用意（`npz_to_pcd.py`） | 不要 | ✅ 完了 |
| **1** | `nav/geometry.py` + `nav/protocol.py` + `nav/occupancy.py` + `nav/route.py` + 単体テスト76件 | 不要 | ✅ 完了 |
| **2** | `sim/fake_service.py` + `nav/transport.py` + `nav/mission.py`。mockでプロトコル検証 | 不要 | ✅ 完了 |
| **3** | `sim/run_sim.py`。`sim_room.pcd`で幾何検証 | 不要 | ✅ 完了 |
| **4** | `RealTransport` をPC2で実行してワイヤ形式を検証 | PC2のみ | ← 次 |
| **5** | 実機。1801→1802で本物の地図を作り、1804→1102 | 必要 | |

**Phase 1〜3は実機もPC2も要らない。** ここまでで「10m分割が正しいか」「is_arrived待ちが
正しいか」「507でリトライできるか」は全部潰せる。

### Phase 1 で決めたこと（2026-09-02）

| 決めたこと | 値 | 理由 |
|---|---|---|
| **1区間の上限** | `MAX_SEGMENT_M = 8.0`（公式上限は10.0） | 公式の10mは「目標点と**現在位置**の距離」。現在位置は定位の推定値で誤差を持ち、区間終端で行き過ぎると次の区間の始点がずれて距離が伸びる。2mを誤差の吸収に充てた |
| **直線経路の妥当性判定** | **自分で判定する。** 地図から2D占有格子を作り、区間ごとに視線判定する | 1102は**直線でしか歩かず、障害物では止まるだけ**（`mode`は`1`固定で绕障が無い）。壁を貫く指令を出すと必ず詰まる。PC1には任せられない |
| **迂回** | 直線で行けなければA*（8近傍）→ string pullingで頂点を削る | A*の格子経路をそのまま出すと0.1m刻みで数百区間になる |
| **格子の解像度** | `0.10 m` | 足の置き場の議論ではなく「その直線が壁を貫くか」の判定に使う粒度 |
| **障害物とみなす高さ** | `0.30 m ≤ z ≤ 1.80 m` | 床（実測でz≒0に151万点）と天井（実測2.52m）を除く。公式の「50cm未満はLiDARが検知しない」は**走行時**の話で、地図に写る壁は安全側に0.3mから拾う |
| **膨張半径** | `0.40 m` | G1の肩幅は約0.45m＝機体半径約0.25m。定位誤差と歩容の揺れを足した。これにより経路判定を「点が通れるか」で書ける |
| **未観測領域の扱い** | **通行不可**（点が1つも返っていないセル） | ⚠️ 下記参照 |
| **姿勢の扱い** | `nav/geometry.py` の `Pose2D`（x/y/yaw）を正とし、変換をここだけに閉じる | 1102/1804の入力と`pos_info`は四元数、`ctrl_info`はroll/pitch/yaw。混在したまま扱うと必ず取り違える |
| **その場旋回** | 位置が同じでも3度以上の向き直しは1102として投げる | 1102は位置と姿勢の両方を指定するので、旋回だけの指令が成立する |

`is_arrived`のタイムアウトと507のリトライ方針は、判定に`ctrl_info`のストリームが要るので
**Phase 2（`nav/mission.py`）で決める**。Phase 1のスコープ外とした。

#### ⚠️ 未観測を「自由」にすると建物の外を回る

占有格子の外周には、膨張が地図の縁で切れないよう余白を足している。この余白を
「点が無い＝障害物が無い＝自由」と判定すると、**経路が建物の外側を回る**。
`sim_room.pcd` で実際に起きた（10区間中6区間が部屋の外の x=5.49 を通っていた。
部屋の壁は x=±4.96）。

未観測は自由ではなく未知であり、未知は通れない。判定はこうする:

1. 高さに関係なく**点が1つでも返ったセルを「観測済み」**とする（床の点がここで効く）
2. 格子の外周から4近傍でつながる未観測セルを「外側」として通行不可にする
3. 部屋の**内部**に孤立した未観測（LiDARの死角、家具の陰）は自由のまま残す

3を入れないと死角で通路が消える。床が写っていない地図ではこの判定は使えない。

#### Phase 1 の検証結果

`sim_room.pcd`（75,804点・10m×8mの部屋）で確認した:

| 項目 | 結果 |
|---|---|
| 占有格子の生成 | 121×101（12,221セル）を0.04秒 |
| 自由空間の範囲 | x [-4.51, 4.49] / y [-3.43, 3.47] ＝ 部屋の内側に収まる |
| 自由空間の連結性 | 2成分。最大成分が99%、壁際に32セルの孤立ポケット |
| 四隅の巡回（1周） | 10区間・総距離27.96m・最長6.60m |
| 障害物を貫く区間 | 0本 |
| 部屋の外へ出る区間 | 0本（未観測の扱いを直す前は6本） |

孤立ポケットを目標にすると `RouteError`（到達不能）になる。これは正しい挙動で、
実機に投げてから詰まるより早く分かる。

## 進め方

`Mapping/`が既に1801を通しているので、**未検証なのは1802（PC1への保存）と1804と1102**。

実機で確かめる順（Phase 5）:

1. **1801→1802で本物の地図を作る** — PC1に`/home/unitree/test1.pcd`ができる。
   これが無い限り1804は必ず507になる
2. **1804で自己位置合わせできるか** — 成功すれば`slam_relocation/global_map`が1回飛び、
   `ctrl_info`の`ctrName`が`"not init"`から変わるはず
3. **1102に近距離（1〜2m）を投げて到達するか**、`slam_key_info`の`is_arrived`が返るか
4. 実際の挙動を実測（10m制限、障害物停止からの復帰、`completion_percentage`の精度、
   `total_distance`の意味）

なお**ファームウェアが1.5.3以上かは未確認**。PC2のOTA置き場は`1.5.1.1_G1_Edu+`だったが、
これはPC2のステージングであって機体全体の版数とは限らない。
ナビ用フィールドが`ctrl_info`に揃って配信されている以上、機能自体は載っている。

`Mapping/`の実測地図は43.0m×38.6m×3.8mで、公式の適用範囲（X/Y軸45m未満）に
**ギリギリ収まっている**。これ以上広い空間を建図すると適用外になる。

失敗した内容は`FAILURES.md`に記録する。

## 状態

**Phase 3 まで完了**（2026-09-02）。単体テスト100件がgreen。
**実機・PC2・DDS・numpyのいずれも不要。**

| できていること | 内容 |
|---|---|
| 公式API仕様の確定 | 1801/1802/1804/1102/1201/1202/1901 の全JSONスキーマ |
| 実機挙動の採取 | `ctrl_info`の実データ、配信レート、1804のエラー挙動（507） |
| ハードウェア構成の把握 | PC1/PC2/LiDARの役割と、`address`がPC1を指すこと |
| 地図の用意 | `sim/npz_to_pcd.py` で `scans.npz` → 地図PCD + 真値軌跡 |
| プロトコル層 | `nav/protocol.py`。実機の実測JSONをテストの題材にしている |
| 姿勢の変換 | `nav/geometry.py`。四元数とroll/pitch/yawの混在をここに閉じ込めた |
| 通行判定 | `nav/occupancy.py`。地図PCD → 2D占有格子（未観測は通行不可） |
| ルート分割器 | `nav/route.py`。迂回経路の生成 + 8m以内への分割 |
| 通信の抽象化 | `nav/transport.py`。`FakeTransport`（sim）と`RealTransport`（実機・未検証） |
| ミッション実行器 | `nav/mission.py`。1804 → 1102連打 → 到達待ち → 迂回 → 巡回 |
| 偽サービス | `sim/fake_service.py`。運動・障害物・507注入。仮想時間で動く |
| シナリオ実行 | `sim/run_sim.py`。実地図で巡回を回して評価する |

### 動かす

```bash
# テスト（実機不要）
bash Navigation/tests/run_tests.sh        # Navigationだけ
bash scripts/ci/run_all_tests.sh          # リポジトリ全体（CIと同じ）

# シミュレーション。既定は sim/maps/sim_room.pcd の四隅を1周
python3 Navigation/sim/run_sim.py

# 居座る障害物を置いて迂回を見る
python3 Navigation/sim/run_sim.py --obstacle 2.9,0,0.9 --obstacle-wait 8 --detour-limit 6

# 人が横切る（3秒に現れて16秒に消える）。待って再開するはず
python3 Navigation/sim/run_sim.py --obstacle 0.1,-2.5,0.7,3,16

# 3周させる
python3 Navigation/sim/run_sim.py --laps 3

# 運動を積分しないmockモード（プロトコルと状態遷移だけ）
python3 Navigation/sim/run_sim.py --mock
```

`sim/maps/` は `.gitignore` 済み。無ければ先に作る:

```bash
G1_Hackason/.venv/bin/python Navigation/sim/npz_to_pcd.py \
    ../artifacts/scans.npz --output-dir Navigation/sim/maps --name sim_room
```

### sim で確認できたこと（`sim_room.pcd`・10m×8mの部屋）

| シナリオ | 結果 |
|---|---|
| 四隅を1周 | 6区間・総距離24.17m・最長6.07m・所要47.0秒（仮想時間） |
| 人が横切る（13秒だけ塞ぐ） | 12.8秒待って再開。**迂回0回**・完走 |
| 居座る障害物 | 8秒待って迂回1回。1102は6→11回に増えて完走 |
| mockモード | 3.6秒でプロトコルと状態遷移を検証 |
| 壁を貫く区間 | 0本 |
| 部屋の外へ出る区間 | 0本 |

### Phase 2 で決めたこと

| 決めたこと | 値 | 理由 |
|---|---|---|
| **到達判定の一次情報** | `rt/slam_key_info` の `task_result.is_arrived` | 離散イベントなので取りこぼしと重複が起きにくい |
| **`ctrl_info.is_arrived` の扱い** | 補助。**一度Falseを見てから**しか採用しない | 前の区間の到達フラグが残っていると、次の区間を即「到達」と誤判定する |
| **打ち切り時間** | 区間距離 / 0.5m/s × 3.0 + 20秒 | `speed`は指定できないので想定速度から見積もる。3倍も取るのは回頭・加減速・定位のばらつきが読めないため。実機で計測したら詰める |
| **停滞の検知** | `progress.completion_percentage` が45秒動かなければ打ち切り | 速度では判定できない（実測: 静止中も vx/vy/vyaw が0にならない） |
| **障害物で止まったとき** | 15秒待つ。それでも駄目なら迂回 | 人が横切っただけなら数秒で消える。すぐ迂回すると経路が無用に伸びる |
| **障害物で止まった時間** | 打ち切り時間に**数えない** | 人が横切っただけで巡回全体を落とすのは行き過ぎ |
| **迂回のやり方** | 機体の前方に仮の障害物を書き込んで経路を引き直す | `obsInfo` は有無と経過秒だけで、**障害物の位置を教えてくれない**。LiDARの生点群から実位置を取るのはPhase 5以降 |
| **仮の障害物の位置** | 前方1.5m。ただし機体自身から0.3m以上離す | ⚠️ 離さないと円が機体を飲み込み「出発点が壁の中」で経路が引けなくなる（実装中に踏んだ） |
| **迂回の回数上限** | 3回 | 通路そのものが塞がっているときに無限に試さない |
| **507のリトライ** | 3回（0.5秒・1.5秒待ち）で諦める | 応答0.01秒＝ファイルI/Oの前に落ちている。粘っても同じ。1802直後のレースだけ拾えれば十分。無限リトライは原因を隠す |

### 次: Phase 4（PC2でワイヤ形式を検証）

`nav/transport.py` の `RealTransport` は**まだ一度も動かしていない**。
`unitree_sdk2py` の `Client._Call(apiId, json文字列) -> (code, data)` を使い、
api-idは事前に `_RegistApi` で登録する必要がある（未登録は
`RPC_ERR_CLIENT_API_NOT_REG`=3103 で機体まで届かない）。

PC2で確かめること:

1. `RealTransport` が `rt/slam_info` を購読できるか（`ctrl_info` が約5Hzで来るか）
2. `_Call` の戻り値 `code` とレスポンスJSONの `errorCode` の対応（実測: 507のとき code=1）
3. 1201/1202 が受理されるか（地図が無くても通るはず）

**実機で未検証**: 1802（PC1への保存）、1804（自己位置合わせ）、1102（移動）、到達判定。
