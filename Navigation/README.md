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

| API ID | 機能 | 主なパラメータ |
|---|---|---|
| 1801 | 建図開始 | `slam_type: "indoor"`（固定値） |
| 1802 | 建図終了・保存 | `address: "/home/unitree/xxx.pcd"` |
| 1804 | 初期位姿（保存地図の読込＋自己位置設定） | `address` + `x,y,z` + 四元数 `q_x,q_y,q_z,q_w` |
| 1102 | ナビゲーション | `targetPose`(x,y,z+四元数) / `mode` / `speed` |
| 1201 / 1202 | 一時停止 / 再開 | なし |
| 1901 | SLAM終了 | なし |

トピック:
- `rt/unitree/slam_mapping/{points,odom}` — 建図中の点群・オドメトリ
- `rt/unitree/slam_relocation/{points,odom}` — 再測位の点群・オドメトリ
- `rt/slam_info` — 状態ブロードキャスト（JSON文字列）
- `rt/slam_key_info` — 実行結果フィードバック

`rt/slam_info`の主なフィールド: `targetNodeName`（タスク目標点の番号）、
`isOpenPlan`（経路計画の有効/無効）、`is_arrived`（到達したか）、`isClimbStairs`、
`stateMachine`。

### 純正APIの制約（設計に直結する）

- **1回の指示は10m以内、かつ直線移動。** Nav2のような全体経路計画ではない。
  任意の地点へ行くには、**経路を10m以内の直線区間に分割して1102を連打する
  ラッパを自前で書く**必要がある。これが本フォルダの中心的な実装物。
- `mode: 1` = 停障（障害物で停止）、`mode: 0` = 绕障（回避）。
  回避には**幅0.8m以上の通行可能領域が必要**、障害物の幅は0.5m以下、
  **高さ20cm以上ないとLiDARが検知しない**。
- `speed`: Go2は0.2〜0.8 m/s、Go2_Wは0.2〜1.5 m/s。
- 適用シーン: 25m×25m未満・特徴が豊富・**静的**・屋内・平地。

### 重要: 上記の公式ドキュメントはGo2 / Go2_W用で、G1用ではない

公式ドキュメントには「拡張ドックとUnitree公式購入のLiDAR（MID-360/XT16）版の
EDU機器狗のみ対応」「対応機種はGo2とGo2_W」と明記されており、
**G1開発者ガイドにSLAM/ナビのページは存在しない**（LiDARはLivox MID-360という仕様記載のみ）。

一方、2026-08-26の実機ログ（`../Mapping/real/runs/*/manifest.json`）で
**G1が`rt/slam_info`に応答していること**を確認済み。サービス自体はG1にも載っている。

**つまり上記の数値（10m・0.8m・速度域）がG1でも同じ保証はない。実測で確かめること。**
特に速度域は明確に四足の値なので、二足のG1では異なる公算が大きい。

出典: https://support.unitree.com/home/en/developer/SLAM%20and%20Navigation_service
（JS描画のSPAなので通常のHTTP取得では本文が取れない）

## 進め方

1. `sim/`でモック相手にロジック（ルート分割器・巡回管理）を作る
2. `real/`で実機G1に接続し、同じロジックが実機でも動くか確認

`Mapping/`が既に1801/1802を通しているので、**未検証なのは1804と1102の2つ**。

優先順:
1. **1804で保存済み地図に自己位置合わせできるか** — ここが通らないとナビが成立しない最大の関門
2. **1102に任意座標を投げて到達するか**、`is_arrived`が返るか
3. G1での実際の上限を実測（10m制限は本当か、速度範囲、`mode: 0`の回避が二足で機能するか）

失敗した内容は`FAILURES.md`に記録する。

## 状態

未着手。
