# Isaac SimをTailscale経由で遠隔利用する（WebRTC Streaming）

[README.md](./README.md)のSSH/RDP接続に加えて、Isaac SimのGUIそのものを
リモートPC上でサーバーとして起動し、手元のPCから
**Isaac Sim WebRTC Streaming Client**で画面越しに操作する方法。
SSHはコマンド実行向け、RDPはデスクトップ全体の操作向け、この方式は
**Isaac SimのGUIだけを低遅延で使いたいとき**に向いている。

## 前提

- 手元のPC・開発PCの両方がTailnetに参加済み（[README.md](./README.md)参照）
- 開発PC上にIsaac Simがインストール済み

## 手順

### 1. 開発PC側: Isaac Simをstreaming（サーバー）モードで起動

SSHでログインする（[README.md](./README.md)の3-A参照）:

```bash
ssh ubuntu@<開発PCのTailscale IP>
```

Isaac Simのインストール形態によって起動コマンドが異なる。**どちらの形態か
不明な場合は先に以下で確認する**:

```bash
# ビルド版（例: G1プロジェクトのdevuser環境）の場合、このスクリプトが存在する
find / -maxdepth 6 -iname "isaac-sim.streaming.sh" 2>/dev/null

# pip版（venvにisaacsimをインストールした環境）の場合、代わりにこれが存在する
find / -maxdepth 8 -iname "isaacsim.exp.full.streaming.kit" 2>/dev/null
```

#### ビルド版の場合

```bash
cd <Isaac Simのインストール先>/_build/linux-x86_64/release
./isaac-sim.streaming.sh
```

（G1プロジェクトのdevuser環境の場合、パスは`G1_Hackason/CLAUDE.md`に記載）

#### pip版の場合

venv直下の`python.sh`から、streaming用のkitアプリ
（`isaacsim.exp.full.streaming.kit`）を指定して起動する:

```bash
<venvのパス>/python.sh -m isaacsim isaacsim.exp.full.streaming.kit
```

例（2026-09-23時点でOMEN機を確認した際のパス。実際のパスは環境によって
異なるため、上記のfindコマンドで都度確認すること）:

```bash
/home/ubuntu/NVIDIA/env_isaaclab/python.sh -m isaacsim isaacsim.exp.full.streaming.kit
```

> ⚠️ **未検証**: 2026-09-23時点でこのコマンド自体の実機動作確認はできていない
> （作業中にOMEN機の電源が落ち中断）。次回作業時に実際に起動できるか確認し、
> 結果をこのファイルに追記すること。

いずれの場合もウィンドウなしで起動し、WebRTCサーバーとして待ち受ける。
起動に数分かかることがある。

### 2. 手元のPC側: Isaac Sim WebRTC Streaming Clientをインストール

以下のページから、使用中のIsaac Simバージョンに対応した
Isaac Sim WebRTC Streaming Clientをダウンロード・インストールする
（URLはバージョンに応じて変わるので、使用バージョンのドキュメントを開くこと）:

```
https://docs.isaacsim.omniverse.nvidia.com/<バージョン>/installation/download.html
```

### 3. 接続

WebRTC Streaming Clientを起動し、IPアドレス欄に開発PCのTailscale IP
（例: `100.99.102.70`）を入力して接続する。

## トラブルシューティング

- 接続できない場合、まず`tailscale status`で開発PCが`active`になっているか
  確認する（[README.md](./README.md)のトラブルシューティング参照）
- Isaac Sim起動直後は数分間ポートが開かないため、起動ログでエラーが出ていない
  ことを確認してからクライアント側で接続を試みる

## 関連

- [README.md](./README.md) — Tailscaleでの基本的なSSH/RDP接続手順
- [G1_Hackason/CLAUDE.md](../../G1_Hackason/CLAUDE.md) — G1プロジェクトの
  Isaac Sim環境（ビルド版）の詳細パス
