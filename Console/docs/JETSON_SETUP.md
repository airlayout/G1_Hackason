# Jetson へ置いて、動かす手順

最終更新: 2026-10-18（段階 8: カメラ・lidar の起動・停止を `--local` に対応）。**この手順の全体を Jetson 実機で通したことはまだ無い**（各手順の確認欄を参照）。
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

### 1.1 カメラと lidar（追加の配置は要らない）
`--local` のときは、`Console/jetson/` の `camera_ctl.sh`・`lidar_ctl.sh` をそのまま呼ぶ（`~/g1_console_*` へのコピーは不要。ssh 方式のときだけ従来どおり別 PC 側から `~/g1_console_camera` などを使う）。

- **既定は停止**。ハブを起動しても配信は起動しない。ポートを使うのは、画面でカメラ・lidar の「起動」を押して確認ポップアップを通したときだけ（「停止」で解放）。
- 待ち受けは **`127.0.0.1` のみ**（`CAMERA_BIND` / `LIDAR_BIND` で渡す）。スマホはハブ経由（`/camera`・`/lidar`）で見るので、配信ポート（既定 8081 / 8082）は wlan0・eth0 に出ない。同じ Jetson の他の開発プロジェクトとのポート衝突も、起動したときだけに限られる。ポートは設定タブで変えられる。
- pid とログは `~/.g1console/run/`（`camera_stream.log`・`lidar_stream.log`）。リポジトリの中には何も作らない。
- ハブを止めても配信は止まらない（意図どおり。画面の「停止」で止める）。常駐させたハブを再起動しても、起動済みの配信はそのまま残る。

## 2. 起動コマンド
Jetson 上で:

```bash
cd ~/G1_Hackason/Console
python3 server.py --local --bind-iface wlan0
```

- `--local` = ssh を使わず、ヘルパー・カメラ・lidar・タスクをこの Jetson 上で動かす。
- `--bind-iface wlan0` = wlan0 の IPv4 でも待ち受ける（127.0.0.1 でも常に待ち受ける）。ループバック以外で待ち受けるので**トークン認証が必須**になる。
- 閉じた信頼できるネットワークでだけ、`--no-auth` を足すとトークンを求めなくなる（スマホは URL を開くだけ）。同じネットワークの端末なら誰でも操作できる。既定は認証あり。
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
- Jetson 上でカメラ・lidar が画面から起動・停止できること、停止中は 8081/8082 を誰も使っていないこと、起動中も `127.0.0.1` だけで待ち受けること（`ss -ltn`。J23）。
- Nav2 のタスクをコンソールから実行したときの挙動（J18〜J21）。
- 上記 rsync のコマンド自体は実行していない（Jetson への ssh 接続先・ユーザー名は例）。
