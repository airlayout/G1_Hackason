# オフライン開発ガイド(ロボットに触れない間)

前提: 事実の根拠は [g1_analysis.md](g1_analysis.md) と [g1_raw/MANIFEST.md](g1_raw/MANIFEST.md)。観測/推測の区別はそちらを参照。

## 1. オフラインでできること / ロボット必須のこと

| 区分 | 内容 |
|---|---|
| **オフラインで可** | Console の UI 改善・`--mock` での動作確認(`server.py --mock`)/ `tests/` の実行 / `live_20260930_234841.jsonl` のリプレイ(DDS リスト・実機ログ再生タブ)/ `lidar_frame0.bin` を使った点群の可視化・変換コードの検討 / `dds_samples.json` を使ったトピック・型の整理 / docs 整備 / カメラ問題の仮説整理と手順書作成 |
| **ロボット必須** | カメラ実機の open/フォーマット確認 / videohub との競合確認 / `SetFsmId` の実機テスト(未実施)/ 音声・腕・移動の実行 / ros2/DDS のレート実測 / pyrealsense2 の動作確認 / 環境変数の取得 |
| **やらない** | 電池 0% 付近での通電・歩行試験 / 原本(g1_snapshot, g1_logs)の改変 / 秘密を含む原本の共有 |

## 2. 環境の事実(要約)

- Jetson Orin NX / L4T R35.3.1 / Ubuntu 20.04 / Python 3.8(システム)+ lerobot conda(3.12)+ pixi(3.11, ROS Humble)。
- LAN: eth0 `192.168.123.164`、Tailscale 経由で `ssh g1-ts`(操作用 PC 側の ssh config)。
- DDS: CycloneDDS 0.10.2、domain 0、eth0、`AllowMulticast=spdp`。
- カメラ: D435i(video0-5)。RGB は video4、**Unitree の videohub_pc4 が専有**。webcam(Sunplus)は前回未列挙。
- pyrealsense2 2.55.1.6486 は**導入済み**(古い記述の「未導入」は誤り)。
- 詳細: [g1_analysis.md](g1_analysis.md)。

## 3. Console とロボットの関係

```
操作用 PC                                     Jetson (g1-ts)
server.py (127.0.0.1:18790, 標準ライブラリのみ)
  └ SshHelper --ssh--> remote_helper.py (py3.8 + unitree_sdk2py) --DDS--> ロボット
  └ /camera/<name> --HTTP proxy--> camera_stream.py (:8081, 手動起動のみ)
index.html: コンソール / DDS リスト / カメラ の 3 タブ
```

- 監視は約 1 秒周期(切断時 3 秒)。許可 FSM ID は `[0,1,3,501,702,706]`。
- `--mock` でロボット無しに UI を検証できる。`SetFsmId` は実機未検証。
- カメラは `--camera-url http://<tailscale ip>:8081` を指定。カメラ名は `std`, `d435i`。

## 4. カメラサーバーの起動・停止ルール(厳守)

1. **手動起動のみ**。`camera_ctl.sh start|stop|status` を人が実行する。
2. **常駐化禁止**: systemd / cron / rc.local / `while` ループでの監視・再起動は作らない。
3. **自動再起動禁止**: 落ちたら原因を見てから人が起動する。
4. 使い終わったら必ず `stop`。放置しない。
5. 補足(観測): `camera_stream.py` の Grabber は内部で「開けなければ 2 秒待って再 open」を無限に繰り返す。開けないカメラを指定すると CPU/ログを消費し続けるので、`status` と手元ログで開けていないと分かったら即 `stop`。コードは今回変更していない。
6. `videohub_pc4` を勝手に kill しない(master_service が再起動する、標準機能が使えなくなる)。

## 5. 未解決課題と次の一手

### A. D435i の RGB が OpenCV で開けない
- 最有力: videohub_pc4 が /dev/video4 を専有(g1_analysis §4.3)。
- [ ] `fuser -v /dev/video4`(または `ps aux | grep videohub`)で保持者を確認
- [ ] `v4l2-ctl -d /dev/video4 --list-formats-ext` と video5 を比較(どちらが映像か)
- [ ] 読み取りのみで `cv2.VideoCapture` の open 可否を video0/2/4/5 で確認(1 フレーム read して即 release)
- [ ] 標準経路 `VideoClient.GetImageSample()`(DDS、JPEG)で取得できるか確認(videohub を止めない)
- [ ] 上が通れば camera_stream.py の入力を DDS 経由にする案を検討(コード変更は別途承認)

### B. by-id の index0 衝突
- video0 と video4 が同じ `...-index0` 名になり video4 が勝つ。`camera_ctl.sh` は video4 を選ぶ。
- [ ] by-path(`1.0` / `1.3`)で指定する案を検討。Depth/IR/RGB のどれを使うか要件を決める

### C. webcam(Sunplus)が未列挙
- [ ] 物理接続・ケーブル・別ポートを確認、`lsusb | grep -i 1bcf`、`dmesg | tail`(読み取り)
- [ ] 出れば `ls -l /dev/v4l/by-id` で index を確認(番号は付け替わる)

### D. pyrealsense2 は導入済みだが動作未確認
- [ ] `python3 -c "import pyrealsense2 as rs; print(rs.__version__)"`
- [ ] `rs.context().query_devices()` の列挙のみ(ストリーム開始はしない → videohub 競合を避ける)

### E. ros2 CLI が大量メモリを使う疑い(未検証)
- 手元ファイルに bad_alloc は無い。ユーザー報告として扱う。
- [ ] 先に `free -m` を取る。`ros2 topic list/hz` は使わない
- [ ] DDS の確認は `Console/tools/discover` か SDK(unitree_sdk2py)で行う
- [ ] `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI` を `env` で記録

### F. バッテリー
- SoC 7% → 0% で取得が途切れた。
- [ ] 再接続前に充電。最初に SoC を読む(電力低下で挙動が不安定になる)

## 6. ロボット再接続後の最初の 10 分(読み取り専用)

順番に実行。書き込み・起動・kill はしない。

1. 電源/充電状態を目視。SoC が低ければ充電を優先して以降は保留。
2. `ping -c 3 192.168.123.164`(LAN 直結時)/ `ssh g1-ts`(Tailscale)
3. `uptime; free -m; df -h /`
4. `ip -br addr`
5. `ls -l /dev/video*; ls -l /dev/v4l/by-id /dev/v4l/by-path`
6. `ps aux | grep -e videohub -e camera_stream | grep -v grep`
7. `lsusb`
8. `v4l2-ctl --list-devices` と `v4l2-ctl -d /dev/video4 --list-formats-ext`(読み取りのみ)
9. `dpkg -l | grep -i -e realsense -e gstreamer`、`env | grep -e ROS -e RMW -e CYCLONE`、`date; timedatectl`
10. 操作用 PC 側で `server.py` を通常起動し `/api/status` を確認(FSM/バッテリー)。

やらないこと: `ros2 topic list`、`SetFsmId`、カメラサーバーの自動起動、`videohub_pc4` の kill。
取得結果は `g1_snapshot/` と別のフォルダ(日付付き)に保存し、原本を上書きしない。
