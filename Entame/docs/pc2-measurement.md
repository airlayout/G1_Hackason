# PC2 の余力を測る手順（現場用）

2026-11-07。`preflight-design.md` §7（案 D: PC2 でサーバーを動かす）の、❓を潰すための測定。
測るのは次の 4 つ。**G1 には何も指令しない（読み取り専用）。** 測定スクリプトは `Entame/real/pc2_probe/pc2_probe.py`。

| # | 測るもの | 判断に使う基準（仮） |
|---|---|---|
| 1 | PC2 の機器情報と入っているモジュール（Python・CPU・メモリ・温度・numpy/cv2/torch/ultralytics/flask 等） | YOLO と Web サーバーを載せられるか |
| 2 | PC2 から Pepper（9559）に届くか | 5 回とも接続できること |
| 3 | `rt/lowstate` と LiDAR（`rt/utlidar/cloud_livox_mid360`）の受信（何もしない状態） | lowstate が途切れない（100 ms 超の欠落 0）。LiDAR が来る（Hz と点数を記録） |
| 4 | 負荷をかけた状態の受信（CPU の空回り／YOLO の連続推論）と YOLO の fps | 負荷中も lowstate が途切れない。YOLO は 10 fps 以上（0.2 s の検知窓のため。値は仮） |

## 安全（先に読む）

- スクリプトは受信（DataReader）だけ。writer・RPC・`LocoClient`・ファイル書き込みを作らない。
- 負荷試験中に lowstate が 500 ms（仮）途切れたら、スクリプトが負荷を止めて打ち切る。
- **G1 は吊るすか、台に載せて足を浮かせるか、スタンドで固定した状態**で測る。負荷が G1 の挙動に影響しないとは確認されていない。測定中は G1 を歩かせない。
- コントローラーを持つ人を 1 人置く（`safety.md`）。
- 負荷試験（# 4）は、# 1〜3 の結果を見てから行う。

## 準備

1. 運営 PC を PC2 に届く Wi-Fi/LAN につなぐ。SSH の接続先を `PC2_SSH_TARGET` に入れる（実値はリポジトリに書かない。パスワードは SSH の入力画面で入れる）。
2. PC2 にファイルは置かない。スクリプトは SSH の標準入力で流し込む（既存の `G1_PC2_RUNTIME.md` と同じ方式）。
3. PC2 の SDK は `/home/unitree/unitree_sdk2_python`（既存の記録）。違うなら `--sdk-path` で指定する。
4. Python は PC2 で実績のある専用 venv の `python` を使う（`G1_PC2_RUNTIME.md`。無ければ `/usr/bin/python3`）。

以降のコマンドの `PY` は PC2 の Python のパス。`PROBE` は `/Users/koba/aicle/G1_Hackason/Entame/real/pc2_probe/pc2_probe.py`。

## 手順

### 段階 1: 機器情報と Pepper への到達（G1 に触れない）

```bash
ssh -T "$PC2_SSH_TARGET" 'PYTHONDONTWRITEBYTECODE=1 PY -B - --pepper-ip <PepperのIP>' < PROBE
```

見る所: `sysinfo.modules`（ultralytics・torch・onnxruntime・flask が入っているか）、`sysinfo.memory`、`pepper.ok`。
Pepper が届かない場合は、PC2 から Pepper へ経路が無い（G1 の内部ネットワークと Pepper の LAN が別）。**案 D の前提が崩れるので、ここで止めて相談する。**

### 段階 2: 受信（何もしない状態）

```bash
ssh -T "$PC2_SSH_TARGET" 'PYTHONDONTWRITEBYTECODE=1 PY -B - --dds --seconds 15' < PROBE
```

見る所（`idle.topics`）:
- `rt/lowstate`: `rate_hz`、`stale_events`（0 が望ましい）、`gap_max_ms`。
- LiDAR: `samples`（0 なら **受信できていない**。2026-09-12 に失敗した事象の再現。0 は「未確認」であり、故障の証明ではない）、`rate_hz`、`points`、`frame_id`。

### 段階 3: CPU 負荷をかけた状態の受信

```bash
ssh -T "$PC2_SSH_TARGET" 'PYTHONDONTWRITEBYTECODE=1 PY -B - --dds --seconds 15 --load cpu:2' < PROBE
```

`cpu:N` は N 個の空回りプロセス（コア数 −1 が上限）。`compare_idle_vs_load` で、`rate_hz` と `gap_max_ms` が段階 2 から悪化していないかを見る。

### 段階 4: YOLO を載せた状態

PC2 に ultralytics と重みがある場合だけ（段階 1 の `modules` で確認）。重みは PC2 にすでにあるものを使う（新規にダウンロードして置かない）。

```bash
ssh -T "$PC2_SSH_TARGET" 'PYTHONDONTWRITEBYTECODE=1 PY -B - --dds --seconds 20 --load yolo:<PC2上の重みのパス>' < PROBE
```

見る所: `load_result.fps`（10 以上か）、`load_result.ms_p95`、`compare_idle_vs_load` の lowstate・LiDAR の悪化。
注意: 測定は灰色の 1 枚絵で行う。実際の映像（前処理・カメラ取り込み・複数物体）より速く出る可能性がある。**実カメラ込みの値ではない。**

## 結果の見方と次の判断

| 結果 | 判断 |
|---|---|
| 段階 1 で Pepper に届かない | 案 D（PC2 でダッシュボード）は成立しない。案 B/C か、Pepper 側の経路を相談 |
| 段階 2 で LiDAR が 0 件 | 1 m の安全停止を LiDAR にする案が成立しない。別の測り方を相談（`preflight-design.md` §5 の 2） |
| 段階 3・4 で lowstate が途切れる／悪化 | YOLO を PC2 に載せない。別 PC で動かし、PC2 は G1 制御と LiDAR 停止だけにする（細い形） |
| 全部問題なし | 案 D の細い形で進める。YOLO を載せるかは段階 4 の fps で決める |

## 未検証であること

- このスクリプトの DDS 受信部分は、G1・PC2 が無い環境では動かせていない。ローカルで確認したのは、構文（Python 3.8 互換）、引数の検証、CPU 負荷の起動と停止、Pepper 未到達の報告、SSH 標準入力での実行まで（`Entame/real/pc2_probe/tests/`）。PC2 での初回実行で型の import などが失敗する可能性がある。失敗したら結果をそのまま貼る。
- 基準の数値（10 fps、100 ms、500 ms）はすべて仮。
- 温度はスクリプトが `/sys/class/thermal` から読む。PC2 でその項目が出るかは未確認。
- Web サーバーの常時稼働の安定性は、この測定では分からない（長時間の運用試験が別に要る）。
