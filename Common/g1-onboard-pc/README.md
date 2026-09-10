# G1 実機内蔵PC（PC2）のフォルダ構造

Unitree G1 実機に内蔵されている PC のうち、`unitree` ユーザーで SSH ログインできる
PC（社内では「PC2」と呼称）のファイルシステム構造を調査したメモ。

## 調査方法

```
ssh g1  # 実機PC2へSSH接続
```

接続先で `ls -la` / `find -maxdepth` 等によりディレクトリツリーを収集した
（2026-09-05 14:52 JST 時点のスナップショット）。

## 基本情報

- OS: Ubuntu（Jetson系、L4T使用）
- ハードウェア: **Jetson Orin NX（ARM64 / aarch64）**。x86_64 PC でビルドした wheel
  はそのままでは入らないため、追加パッケージが必要な場合は `manylinux_aarch64` /
  `linux_aarch64` 版を明示的に取得すること
  （出典: [unitree-g1-physical-ai/voice-conversation/RUNBOOK.md](https://github.com/mayochan32/unitree-g1-physical-ai/blob/main/voice-conversation/RUNBOOK.md)）
- ユーザー: `unitree`（ホームディレクトリ `/home/unitree`）
- ネットワーク:
  - `eth0` が `192.168.123.164/24`（Unitree G1 標準の内部LANアドレス、常時使用）
  - `wlan0` は存在するが **`rfkill` で意図的にブロックされている**。有効化には
    `rfkill unblock all` / `nmcli radio wifi on` / `nmcli device set wlan0 managed yes` /
    `systemctl restart NetworkManager` 等、再起動後も残る恒久的な変更が必要になるため、
    共有機材への配慮として避けること
  - G1本体のマイク multicast は有線セグメント（`192.168.123.x`）内に閉じており、
    ブリッジ/IGMPプロキシを構築しない限り `wlan0` 側には届かない
  - unitree_sdk2py はPC2に最初から入っているため追加インストール不要
    （出典: [g1_bridge/README.md](https://github.com/mayochan32/unitree-g1-physical-ai/blob/main/voice-conversation/g1_bridge/README.md)）

## 共有機材としての運用ルール（他チームとの併用に配慮）

PC2 は複数チームが共有する実機であるため、恒久的な環境変更を避ける運用が徹底されている
（出典: 上記2ファイル）。

- `sudo` を使わない
- パッケージを PC2 本体の Python に `pip install` しない。必要な場合は `/tmp` 上の
  使い捨て venv（例: `python3 -m venv --system-site-packages /tmp/xxx_venv`）に閉じる
- 使い終わったら `/tmp/xxx_venv` 等は明示的に削除する（`/tmp` は再起動で消えるが、
  再起動を待たずに片付ける）
- スクリプト類は `/tmp` に置いて実行する（再起動で自動的に消え、痕跡が残らない）
- 音量など本体設定を変更する API（`AudioClient.SetVolume()` 等）を使った場合は、
  終了時に元の値へ戻す
- `NetworkManager` 等のシステムサービスを再起動しない（他チームの作業中の接続を
  切ってしまう可能性があるため）

## ディレクトリ構造（浅い階層）

```
/
├── home/
│   └── unitree/
│       ├── .bash_history / .bashrc / .profile 等（シェル設定）
│       ├── .ros/                 ROS関連キャッシュ・ログ
│       ├── .ssh/                 SSH鍵（authorized_keys, known_hosts）
│       ├── .vscode-server/       VS Code Remote 用
│       ├── cyclonedds/           CycloneDDS（ソースビルド）
│       ├── cyclonedds_ws/        CycloneDDS ワークスペース
│       ├── Desktop/              Jetson付属のデスクトップショートカット等
│       ├── g1_humble/            ROS2 Humble 関連
│       ├── humble_env/
│       ├── lerobot/              LeRobot（teleop等のサンプル含む）
│       ├── mapping_runs/ mapping_tools/  マッピング作業ディレクトリ・ログ
│       ├── miniconda3/ miniforge3/       Python環境（conda系）
│       ├── nomachine.sh          NoMachine（リモートデスクトップ）関連
│       ├── teleimager/
│       ├── unitree/              Unitree公式ツール一式
│       ├── unitree_sdk2-main/    Unitree SDK2（C++版）
│       ├── unitree_sdk2_python/  Unitree SDK2（Python版）
│       ├── venvs/                Python仮想環境群
│       └── wifi-bt-deb/          Wi-Fi/BTドライバ（rtl8852bu）関連deb
├── opt/
│   ├── nvidia/            Jetson向けNVIDIA製ツール（jetson-io, l4t-*設定 等）
│   ├── ota_package/       OTAアップデート用ペイロード（t19x/t23x）
│   └── ros/               ROS1 Noetic / ROS2 Foxy のインストール先
├── tmp/                   一時ファイル（ソケット、ログ、systemd private tmp 等）
├── unitree/                Unitree純正ソフトウェア一式（実行体・設定）
│   ├── bin/                json_checker 等
│   ├── etc/master_service/ サービス起動制御（cmd/init/plan/prio 等）
│   ├── module/              master_service, video_hub_pc4（カメラ配信）
│   ├── ota/                 OTA関連（backup/pipe/update）
│   ├── robot/pkg/           ロボットパッケージ本体
│   ├── sbin/                key_server, mscli, ota_pipe_cli 等
│   └── var/                 実行時データ・ログ・PIDファイル
├── upgradePythonServer/    アップグレード用Pythonサーバー
├── README.txt
└── version.txt            ファームウェアバージョン情報
```

詳細な深い階層のツリー（フルパス一覧、25,006パス）は
[snapshots/pc2-tree-2026-09-05.md](snapshots/pc2-tree-2026-09-05.md) を参照。

## 用途別の主なパス

| 用途 | パス |
|---|---|
| Unitree純正サービス制御 | `/unitree/etc/master_service/` |
| カメラ映像配信（chest等） | `/unitree/module/video_hub_pc4/` |
| OTAアップデート | `/unitree/ota/`, `/opt/ota_package/` |
| ROS1 (Noetic) | `/opt/ros/noetic/` |
| ROS2 (Foxy) | `/opt/ros/foxy/`, `/home/unitree/g1_humble/` |
| CycloneDDS | `/home/unitree/cyclonedds/`, `/home/unitree/cyclonedds_ws/` |
| Unitree SDK2 (C++ / Python) | `/home/unitree/unitree_sdk2-main/`, `/home/unitree/unitree_sdk2_python/` |
| Wi-Fi/Bluetoothドライバ | `/home/unitree/wifi-bt-deb/` |
| マッピング作業ログ | `/home/unitree/mapping_runs/`, `/home/unitree/mapping_tools/` |

## 注意事項

- 本ドキュメントはパス名・パーミッション・ディレクトリ構造のみを記載しており、
  鍵ファイルやトークンなど**実際の秘匿情報の中身は含まない**。
- `.ssh/`, `.gnupg/` 等の存在するディレクトリ名は記載するが、内容には触れない。
- SSH接続情報（ホスト名・パスワード等）は本ドキュメントに平文で記載しない。
  接続方法は各チームリーダーに確認すること。

## 関連

- [../shared-pc/README.md](../shared-pc/README.md) — G1開発用の共有デスクトップPC
- [../network/README.md](../network/README.md) — G1とのネットワーク接続設定
