# 実機Mapping：開始 → 保存 → 浮遊点除去

このPCの構築済みnative環境を使用する。Docker・有線接続は不要。
G1は純正リモコンで操作する。PCとG1を同じ5GHz Wi-Fiへ接続し、
G1のカメラ配信とSSH鍵認証を利用できる状態にする。

## 1. 計測開始

G1を静止させてから実行し、開始後も10秒程度は静止を保つ。

```bash
cd /home/ubuntu/dev/G1_Sodai/Mapping/real
./mapctl start --backend raw --name room_a
./mapctl view --live
```

表示された `[STARTED]` のセッションIDを控える。純正リモコンでゆっくり巡回する。
FAST-LIO2で点群を位置合わせし、同時刻のLiDAR姿勢からOctoMapの占有・空きを更新する。
後続の光線が通過した場所の偽点は削除する。生LiDAR・IMU・RGB等はPCのrosbagへ記録する。

## 2. 完了・保存

G1を止め、表示端末でCtrl+Cを押してから実行する。保存完了まで無線接続を維持する。
表示を閉じるだけでは計測は停止しない。

```bash
./mapctl stop
./mapctl validate
```

`runs/<セッションID>/` に原本PCD、OctoMap、rosbag、軌跡を保存する。
カメラ内部パラメータ未設定のWARNは、点群保存の失敗とは別である。

## 3. 事後処理（追加の孤立点除去は任意）

基本経路はFAST-LIO2＋OctoMap → 床基準への補正 → 必要に応じた2mカット・着色。
近傍点数による孤立点除去は元の計画にはない補助処理で、今回の見た目では必須ではなかった。
観測が少ない実物の点も落とし得るので、必要な場合だけ使用する。

以下は今回、追加の孤立点除去も行った版を再現するコマンド。
今回分は実行済みなので、そのまま再実行する必要はない。
原本は上書きしない。再処理時は出力先を別名にする（既存の出力があれば停止する）。

```bash
SESSION=20261021_204548_room_a
RUN="$PWD/runs/$SESSION"

# 全記録スキャン＋対応するLiDAR姿勢を、計測時と同じOctoMap実装・設定で再統合する。
./mapctl rebuild "$SESSION" --method octomap \
  --output "$RUN/derived/postprocess/map_rebuilt.pcd"

# 孤立点除去、床・天井推定、床基準への変換、コピーの2mカット・高さ着色。
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 quickstart/postprocess_floor_pcd.py \
  "$RUN/derived/postprocess/map_rebuilt.pcd" \
  "$RUN/derived/floor_postprocessed" \
  --trajectory "$RUN/trajectory/trajectory.tum" \
  --up -0.4580408 0.05918349 -0.88695882 \
  --up-source '20261021_204548_room_a: 初期IMU平均を姿勢・床天井の平面と照合' \
  --radius 0.15 --max-height 2.0
```

- 半径15cm以内に他の点が2点未満しかない孤立点を除去する。
- `--up` は**この試行の地図座標で測定した上方向**。次回は当該rosbagの静止IMUを
  記録姿勢で地図座標へ回転して求め直し、床・天井面と照合する。今回の値や単純な+Zを流用しない。
- 床をZ=0、床法線を+Zに変換し、天井も含む全高の処理済みPCDを保存する。
  そのコピーから床高さ2m以上を削除し、0m＝青、1m＝緑、2m付近＝赤で着色する。
- 平面・軌跡が整合しなければ処理を停止する。支持点数・残差・変換行列は`processing.json`に残る。
- 今回は3,443スキャンを再統合し、孤立点18,017点を除去。全高版1,128,103点、2m未満版643,074点。
  天井高は約2.80m。孤立点除去だけで全ての誤点が消えるわけではなく、SLAMドリフト・ループ閉じ込み・絶対XY校正は行わない。

## 4. 3D表示

```bash
bash quickstart/view_floor_map.sh "$RUN/derived/floor_postprocessed"
```

RVizで床基準・高さ着色済みの2m未満版を表示する。回転・ズームして確認する。
再生成した`.bt`は元の地図座標のままなので、床基準PCDと同じ座標だと扱わない。

### 基本経路：追加の孤立点除去なし

`--no-denoise` で近傍点フィルタを省略できる。今回の比較版は既に作成済みで、
`--reference-processing` に既存の `floor_postprocessed/processing.json` を指定し、
座標変換だけを共通化した。点群はOctoMap再生成版から直接読み、2m未満の653,565点を着色している。

```bash
bash quickstart/view_floor_map.sh "$RUN/derived/octomap_only_floor" \
  /g1_mapping/octomap_only_floor_height_lt2
```

## 今回の保存先

すべて `real/runs/20261021_204548_room_a/` 以下。元の生センサーデータもrosbag内にある。

| 内容 | セッション内の相対パス |
|---|---|
| rosbag（約3.8GB、生LiDAR・IMU・RGB等） | `raw/rosbag2/rosbag2_0.db3` と `raw/rosbag2/metadata.yaml` |
| 計測終了時の原本点群 | `map/map_raw.pcd` |
| 全記録から再生成した点群 | `derived/postprocess/map_rebuilt.pcd` |
| 孤立点除去・床基準化済み、全高版 | `derived/floor_postprocessed/map_floor_aligned.pcd` |
| 全高版の複製（高さ制限前） | `derived/floor_postprocessed/height_lt_2m/map_source.pcd` |
| 床から2m未満・高さ着色済み | `derived/floor_postprocessed/height_lt_2m/map_floor_height_lt_2m.pcd` |
| OctoMapのみ＋床基準化・2m未満・高さ着色（追加孤立点除去なし） | `derived/octomap_only_floor/height_lt_2m/map_floor_height_lt_2m.pcd` |
| 処理条件・床天井・変換行列 | `derived/floor_postprocessed/processing.json` |

rosbagだけをチームへ渡す場合は `raw/rosbag2/` をフォルダごとアップロードする。
`metadata.yaml` と `rosbag2_0.db3` を同じフォルダに保つ（合計約3.8GB、RGB画像も含む）。
点群・処理条件も含めて共有する場合は、セッション `20261021_204548_room_a/` を丸ごと渡す。

2026-10-21の整理では、このPCの旧4試行だけをごみ箱へ移動した。
最新試行は生データ・加工データ・調査資料を含め丸ごと保持し、G1内部の過去データは変更していない。
