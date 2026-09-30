# 次回 G1 接続時の確認項目

G1 の電池切れ（2026-10-01）で取れなかったもの・未確認のものをまとめる。
**すべて読み取り専用の確認から始める。** 接続は `ssh g1-ts`（Tailscale, <Tailscale の IP>, user `unitree`）。
背景は `g1_analysis.md`、原本の一覧は `g1_raw/MANIFEST.md`。

## 0. 守ること
- カメラ配信サーバーは**手動で起動・停止するだけ**（`~/g1_console_camera/camera_ctl.sh start|stop|status`）。
  常時起動・自動再起動（systemd / cron / ループ）は禁止。確認が済んだら必ず `stop` して `status` が「停止中」なのを見る。
- G1 で `ros2 topic list` を気軽に叩かない（前回 `bad_alloc` で失敗し、`_ros2_daemon` が約 7GB=メモリ 45% を使った。
  この観測は会話中のもので、保存ファイルには残っていない）。叩くなら前後で `free -m` と `ps` を記録し、
  終わったら `ros2 daemon stop`。
- モーター・指令系には触らない。

## 1. 最初の 10 分（読み取りのみ）
1. `tailscale status | grep g1-jetson` で到達性を確認。
2. `free -m; df -h /; uptime`、`ps aux --sort=-%mem | head` … 前回の残骸（ros2 デーモン等）が無いか。
3. `date; timedatectl` … Jetson の時刻とタイムゾーン（Mac と約 1 時間ずれていた）。
4. バッテリー残量（Console の「バッテリー」）。低いなら長い取得はしない。
5. `~/g1_console_camera/camera_ctl.sh status` … 停止中であること。

## 2. 未取得・不明だった項目（前回の資料で空欄）
| 項目 | 取り方（読み取りのみ） | 目的 |
|---|---|---|
| `env` の ROS/DDS 関連 | `env \| grep -i -e ROS -e RMW -e CYCLONE -e DDS`（前回の `env.txt` は 0 バイト） | `ROS_DOMAIN_ID`・`RMW_IMPLEMENTATION`・`CYCLONEDDS_URI` |
| librealsense / GStreamer の有無 | `dpkg -l \| grep -i -e realsense -e gstreamer` | RGB 取得手段の選択 |
| OpenCV のビルド情報 | lerobot 環境と system Python で `cv2.getBuildInformation()` の V4L2 / GStreamer 行 | 開けない原因の切り分け |
| `pyrealsense2` の実動作 | system Python 3.8 で `import pyrealsense2 as rs; print(rs.__version__)` と `rs.context().devices` | pip 一覧では 2.55.1 入り。**import と実機認識は未確認**（lerobot 環境には無い） |
| `/dev/video4` の占有者 | `fuser -v /dev/video4`（root なら `sudo` 要否を確認） | `videohub_pc4` が握っているか |
| RGB のフォーマット一覧 | `v4l2-ctl` は無いので `python` の `ioctl` か `ffprobe`、または `pyrealsense2` の profiles | 形式・解像度の把握 |
| 標準カメラ（SunplusIT） | `lsusb; ls /dev/v4l/by-id` | 前回は lsusb にも by-id にも出なかった。物理的な接続・電源を先に確認 |
| bad_alloc の再現条件 | 上の注意どおり前後を記録して 1 回だけ | 前回の観測を裏付けるログを残す |
| 取得が意図した全量だったか | 前回の tgz は 707 エントリで壊れてはいない | `g1_map_click_mvp`（2.5GB）・`camera_runs`・`g1_humble` 本体・`*.pcd/*.pt/*.onnx` は**未取得** |

## 3. D435i の RGB（未解決）
仮説（確度は高いが未検証）: `videohub_pc4`（root）が `/dev/video4` を専有し、OpenCV が開けない。
by-id の `index0` は `video4` を指すため、`camera_ctl.sh` はこの専有ノードを選んでしまう。

確認の順:
1. `fuser -v /dev/video4` と `ps aux | grep videohub` で占有を確認。
2. 占有が事実なら、選択肢を比較する（**videohub を止めるのは、他機能への影響が分かってから。勝手に止めない**）:
   - a. 公式経路 `VideoClient.GetImageSample()`（unitree_sdk2py）で RGB を取る。
   - b. `pyrealsense2` で D435i を直接開く（videohub と競合しないか要確認）。
   - c. `video5`（同じ 1.3 側）が開けるか、形式指定で試す。
3. `video2` はグレースケール（3 チャンネル完全一致・平均 100.6）で、**IR と判断**。
   確認のため `/tmp/v2.jpg` を目視（白黒でドット模様＝IR。暗い部屋のカラーなら違う見え方）。
   ただし G1 の再起動で `/tmp` は消えている可能性が高いので、撮り直す。

## 4. カメラ配信スクリプトの既知の注意点
- `camera_stream.py` の Grabber は、開けないカメラを 2 秒間隔で**無限に再 open** する（サーバー稼働中のみ）。
  長時間つけっぱなしにしない。開けないなら `stop` する。
- `camera_ctl.sh` はカメラ名（std / d435i）を by-id の glob で選ぶ。Sunplus が無いと `std` は配信されない。
- 標準カメラの `Sunplus` は、前回は列挙されていない。

## 5. Console 側（G1 なしで済む確認・G1 復帰後に見る点）
- カメラタブ（`#camera`）の見た目は未確認（私は表示確認をしていない。ユーザーの目視待ち）。
- `server.py --camera-url http://<Tailscale の IP>:8081` で、`/camera/std`・`/camera/d435i` が映るか。
- `.gitignore` に `Console/docs/g1_raw/original/` を足すか（`original/` は bash_history 等を含みうる原本コピーで、現状は対象外になっていない）。
- `g1_raw/redacted/` の伏せ字を目視確認してから共有する。
- `g1_analysis.md` と既存の `G1_FINDINGS.md` の重複を整理する。

## 6. 資料の訂正メモ
- 「pyrealsense2 は未導入」は誤り。system Python 3.8 に 2.55.1 が入っている（lerobot 環境には無い）。
- 「bad_alloc・ros2 デーモン 7GB は裏付けなし」は、保存ファイルに無いだけ。会話中の実観測はある（§0）。
