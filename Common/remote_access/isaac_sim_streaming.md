# MacからIsaac Simを使う（Tailscale + WebRTC）

**何をしたいか** | **使う方法**
--- | ---
Ubuntuでコマンドを実行する | [SSH](./README.md#3-a-cuiコマンド作業ssh)
Ubuntuのデスクトップ全体を操作する | [RDP](./README.md#3-b-guiデスクトップ操作リモートデスクトップ)
Isaac Simの画面をMacで見る | このページのWebRTC手順

## 標準Isaac Sim GUIを表示する（実機で映像確認済み）

Ubuntu 24.04のpip版Isaac Sim 6.0.1.0と、Apple Silicon MacのWebRTC Streaming Client 2.0.0で確認した手順。

1. [SSHでUbuntuに接続](./README.md#3-a-cuiコマンド作業ssh)し、GPUと既存のIsaac Simプロセスを確認する。

   ```bash
   nvidia-smi
   pgrep -af 'isaacsim|run_g1_twin.py'
   ```

2. 使用中の人がいなければ、Ubuntuで起動する。準備に数分かかる。

   ```bash
   /home/ubuntu/NVIDIA/env_isaaclab/bin/isaacsim isaacsim.exp.full.streaming --no-window
   ```

3. Macで[Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html)を開き、管理者から案内されたUbuntuのTailscale IPを入力して`Connect`する。旧Omniverse Streaming Appは使わない。

4. `LIVE`とViewport映像を確認する。確認後はUbuntuの起動ターミナルで`Ctrl+C`を押して終了する。

2026-09-28にTailscale経由でLIVE・約60 FPS・1280×720を確認した。暗いグリッドはシーンを開いていない標準GUIの表示。Macからの入力、シーン保存、G1表示はまだ確認していない。

## G1を表示したい場合（未検証）

標準GUIの終了後、Ubuntu側の`IsaacSim_Env/run.sh`と使用するUSDを確認してから、G1用Isaac Labスクリプトの`--livestream 2`を別に試す。Ubuntu上とこのリポジトリでは`run.sh`の既定シーンが異なるため、現時点で共通の起動コマンドは記載しない。結果は[作業記録](./worklogs/2026-09-28.md)に追記する。

## 映らないとき

- まずUbuntuで`nvidia-smi`とIsaac Simの起動ログを確認する。
- 接続試行中にUbuntuで`ss -ltn '( sport = :49100 )'`と`ss -lunp '( sport = :47998 )'`を確認する。TCP 49100は接続、UDP 47998は映像用。
- 接続後に画面が更新されないときはクライアントの`View > Reload`を試す。暗いグリッドが見えるときはシーンの読み込みを確認する。

起動コマンド、ポート、Reloadは[NVIDIA公式のWebRTC手順](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/manual_livestream_clients.html)を参照。TailnetのIPや認証情報は共有資料に記載しない。
