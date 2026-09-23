# Tailscaleによる開発PCへの遠隔接続

開発PC（G1のホストPC / GPUマシンなど）に外出先・自宅から安全にリモート接続し、
SSH開発やGUI確認を行うための手順。エンタメ・警備チーム共通で使う。

## 前提

- 開発PC・接続元PCの両方が同じTailnet（Tailscaleネットワーク）に参加していること
- 開発PC側でTailscaleがインストール済み・ログイン済みであること
- 社内で使うTailscaleアカウント（組織のTailnet）を利用すること
  （個人アカウントで新規Tailnetを作らない。他メンバーと同じTailnetに入る）

## 開発PC側のセットアップ（初回のみ）

### 1. Tailscaleのインストール

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

### 2. ログイン

```bash
sudo tailscale up
```

表示されるURLをブラウザで開き、組織のTailscaleアカウントで認証する。

### 3. SSHを有効化する（Tailscale SSH機能を使う場合）

Tailscale自体にSSH機能があり、これを使うとSSH鍵の配布・管理が不要になる。

```bash
sudo tailscale up --ssh
```

これにより、同一Tailnet内の許可された端末からTailscaleのIDベース認証だけで
SSHログインできるようになる（ACLでのアクセス制御は管理者コンソールで設定）。

### 4. 起動時の自動接続を確認する

```bash
sudo systemctl enable --now tailscaled
```

再起動後も自動的にTailnetへ再接続されることを確認する。

### 5. マシン名の確認

```bash
tailscale status
```

自分の開発PCの名前（`<hostname>.<tailnet-name>.ts.net`）を控えておく。
チームで共有する際はこの名前を使う（IPは固定ではないため名前解決を使う）。

## 接続元PCからの利用方法

### 1. Tailscaleのインストール・ログイン

開発PC側と同じ組織のTailscaleアカウントでログインする。

- macOS/Windows: 公式アプリをインストールしてログイン
- Linux: `curl -fsSL https://tailscale.com/install.sh | sh` → `sudo tailscale up`

### 2. SSH接続

Tailscale SSHを有効化済みの場合:

```bash
ssh <user>@<hostname>.<tailnet-name>.ts.net
```

通常のSSH鍵運用をしている場合も、接続先ホスト名をTailscaleのマシン名に
置き換えるだけで同様に接続できる（`~/.ssh/config`の`HostName`を書き換える）。

### 3. GUI/リモートデスクトップ

VNCやRDPなど既存のリモートデスクトップ手段を使う場合も、接続先ホストを
TailscaleのマシンIP（`tailscale ip -4`で確認）またはマシン名に向ければ、
社内ネットワーク外からでも同様に到達できる。

## 開発の進め方

- リモート接続後の開発フローは、各プロジェクトのCLAUDE.md・SETUP.mdに従う
  （例: [`SETUP.md`](../../SETUP.md)、[`IsaacSim_Env/SETUP.md`](../../IsaacSim_Env/SETUP.md)）
- GPUを使う処理（Isaac Sim等）はリモート接続先の開発PC上で実行し、
  接続元PCは操作端末として使う想定
- 複数人で同じ開発PCに同時接続する場合は、作業前に`tmux`/`screen`等で
  セッションを分離し、他人の作業プロセスを誤って落とさないよう注意する

## トラブルシューティング

### 接続できない

```bash
tailscale status
```

を開発PC・接続元PCの両方で実行し、双方がTailnetに`Connected`状態であることを
確認する。`Offline`や見えない場合は、開発PC側で`tailscaled`が起動しているか
（`systemctl status tailscaled`）を確認する。

### 組織のACLで拒否される

Tailscale管理コンソール（[login.tailscale.com](https://login.tailscale.com/)）の
ACL設定で、自分のアカウント・端末が対象の開発PCへのアクセスを許可されているか
管理者に確認する。

## 関連

- [Common/network/README.md](../network/README.md) — G1本体とのEthernet直結・疎通確認
  （こちらはロボット本体との通信、本ドキュメントは開発PCへの遠隔接続という別レイヤー）
- [SETUP.md](../../SETUP.md) — 操作PC側・G1本体側の環境構築全体の手順
