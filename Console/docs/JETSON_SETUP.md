# Jetson へ置いて、動かす手順

最終更新: 2026-10-18。**この手順の全体を Jetson 実機で通したことはまだ無い**（各手順の確認欄を参照）。
実機で確かめる項目は [REAL_ROBOT_CHECKLIST.md](REAL_ROBOT_CHECKLIST.md) の J。

## 0. 前提（Jetson で先に確かめる）
| 何を | 確認コマンド | 備考 |
|---|---|---|
| Python 3.8 以上 | `python3 --version` | ハブは標準ライブラリだけ。pip は要らない |
| Unitree SDK | `ls ~/unitree_sdk2_python` | 別の場所なら `UNITREE_SDK_DIR` で指す |
| 同じ Wi-Fi に付いている | `ip -4 -o addr show dev wlan0` | スマホから開くときだけ。eth0（ロボット内部 LAN）には開かない |
| Nav2 を使うなら | `ls /home/unitree/g1_nav2/g1up.sh` | 警備班の Nav2 一式。無ければ nav2-* タスクは動かない |

## 1. Jetson へ転送するもの
**`Console/` ディレクトリ丸ごと**（`G1_Hackason_UI/Console`）。操作用 PC から:

```bash
rsync -av --exclude '__pycache__' --exclude 'tests/' --exclude 'docs/' \
  Console/ unitree@<JetsonのIP>:~/G1_Hackason/Console/
```

- 置き場所は `~/G1_Hackason/Console`（`jetson/g1-console.service` の `WorkingDirectory` と同じ）。別の場所なら service ファイルも直す。
- 必須: `server.py` `g1console/` `web/` `jetson/` `tasks/`。
- `tests/` はタスク `run-tests` を使うときだけ要る。`docs/` と `tools/` は動作に不要（`tools/` は開発用の単発スクリプト）。
- `data/`（地点 `locations.json`）は、無ければ地点が空で始まる。

### 1.1 カメラと lidar を使うなら、もう 2 か所へ置く（⚠️ 見落としやすい）
`--local` でも、カメラ・lidar の起動・停止は **`~/g1_console_camera/camera_ctl.sh`**、**`~/g1_console_lidar/lidar_ctl.sh`** を呼ぶ（`g1console/camera_proc.py` の `REMOTE_DIR`。`Console/jetson/` の中は見ない）。ctl スクリプトは自分と同じディレクトリの `*_stream.py` を起動する。

```bash
# Jetson 上
mkdir -p ~/g1_console_camera ~/g1_console_lidar
cp ~/G1_Hackason/Console/jetson/camera_ctl.sh ~/G1_Hackason/Console/jetson/camera_stream.py ~/g1_console_camera/
cp ~/G1_Hackason/Console/jetson/lidar_ctl.sh  ~/G1_Hackason/Console/jetson/lidar_stream.py  ~/g1_console_lidar/
chmod +x ~/g1_console_camera/camera_ctl.sh ~/g1_console_lidar/lidar_ctl.sh
```

使わないなら不要（画面のカメラ・lidar のボタンが失敗するだけ）。

## 2. 起動コマンド
Jetson 上で:

```bash
cd ~/G1_Hackason/Console
python3 server.py --local --bind-iface wlan0
```

- `--local` = ssh を使わず、ヘルパー・カメラ・lidar・タスクをこの Jetson 上で動かす。
- `--bind-iface wlan0` = wlan0 の IPv4 でも待ち受ける（127.0.0.1 でも常に待ち受ける）。ループバック以外で待ち受けるので**トークン認証が必須**になる。
- 起動ログに `ログイン用 URL: http://<IP>:18790/login?token=…` が出る。トークンは `~/.g1console/token`（初回に自動生成）。
- 自機のブラウザだけでよいなら `python3 server.py --local`（`http://127.0.0.1:18790`、認証なし）。
- SDK が別の場所なら `UNITREE_SDK_DIR=/path/to/unitree_sdk2_python python3 server.py --local …`。

オプション（必要なときだけ）:
| オプション | 用途 |
|---|---|
| `--nav-adapter http://127.0.0.1:<port>` | Nav2 の ROS アダプタ（読み取り専用）につなぎ、マップタブに地図・現在地を出す。Nav2 稼働中は手動操作が無効になる |
| `--heartbeat-to 127.0.0.1:47311` | マップタブを見ている間だけ Nav2 へ生存信号を送る |
| `--topics-file <path>` | DDS タブからのトピック送信の許可リスト（既定は空 = 何も送れない） |
| `--port <n>` | 既定 18790 |

## 3. スマホ・別 PC から開く
1. 起動ログの `ログイン用 URL` を、同じ Wi-Fi のスマホで一度開く（Cookie が付き、以後トークン不要）。
2. ⚠️ 平文 HTTP。トークンを知っていれば画面から**モード切替（ダンピング・ゼロトルク等）ができる**。物理リモコンが最後の砦。
3. eth0 側のアドレスでは届かない（意図どおり）。

## 4. 常駐させる（任意。人が sudo で行う）
```bash
sudo cp ~/G1_Hackason/Console/jetson/g1-console.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now g1-console
journalctl -u g1-console -n 20      # ログイン用 URL を確認
```
`User`・`WorkingDirectory`・`UNITREE_SDK_DIR` を実環境に合わせて直してから設置する。常駐するのはハブだけで、カメラ・lidar・Nav2・タスクは画面のボタンで起動・停止する（自動再起動しない）。

## 5. Nav2 を使うとき（任意）
- **ハブを起動する環境に `ROS_DISTRO` が設定されていないこと**（`g1up.sh` が要求する。ROS を source した端末から起動しない）。`echo "${ROS_DISTRO:-未設定}"`。
- 順序: タスクタブで `nav2-up-dry`（何も起動しない前提確認）→ 通ったら `nav2-up`。機体は通常の立位で出発位置に置き、人は 2 m 以上離れ、物理リモコンは停止係が持つ。
- `nav2-up` の後も機体は動かない。**発進ゲート（`G1_ARM`）と `enable_navigation` は人が現地で行う**（コンソールは行わない）。
- 止めるとき: 先に人がゲートを閉じ（`sudo`）、`nav2-down`（`-dry` で確認できる）、最後に人が `sudo systemctl stop g1-sdk-bridge`。
- 動作未確認の機能は、ボタンを押すと注意ポップアップが出る。

## 6. 動かす前の確認（実機なし・Jetson でも可）
```bash
cd ~/G1_Hackason/Console
python3 server.py --mock          # 実機なしで画面だけ。作り物の状態
bash tests/run_tests.sh           # tests/ を転送した場合。ロボットには触れない
```

## 7. 未確認（推測で埋めない）
- Jetson（Python 3.8）で `--local` が起動すること、`--bind-iface wlan0` が IP を取れること（J1・J2）。
- `~/g1_console_camera` `~/g1_console_lidar` の配置でカメラ・lidar が起動・停止できること（H の項目と同じ確認）。
- Nav2 のタスクをコンソールから実行したときの挙動（J18〜J21）。
- 上記 rsync のコマンド自体は実行していない（Jetson への ssh 接続先・ユーザー名は例）。
