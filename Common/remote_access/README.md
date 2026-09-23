# Tailscaleによる開発PCへの遠隔接続

開発PC（例: OMEN Ubuntu共有PC）に外出先・自宅から安全にリモート接続し、
SSH開発やGUI操作を行うための手順。エンタメ・警備チーム共通で使う。

管理者（小林）がTailscaleの招待リンクを発行し、各メンバーはそのリンクを
承諾する形でTailnetに参加する運用。個人で新規Tailnetを作る必要はない。

## 前提

- 招待リンクは管理者（小林）が発行する。まだ持っていない場合は本人に連絡する
- 対象の開発PC側は既にTailscaleセットアップ済み（管理者が対応）

## 接続手順（メンバー側）

### 1. 招待リンクを開いてTailscaleに参加する

管理者から共有される招待リンク（例: `https://login.tailscale.com/admin/invite/xxxxxxxxxxxx`）
を開き、案内に従ってTailscaleアカウントを作成・承認する。

### 2. 手元のPCにTailscaleアプリを入れる

- macOS/Windows: [公式サイト](https://tailscale.com/download)からアプリをインストールし、
  手順1で作成したアカウントでログインして有効化する
- Linux: `curl -fsSL https://tailscale.com/install.sh | sh` → `sudo tailscale up`

有効化後、開発PCのTailscale IP（例: `100.99.102.70`。管理者から連絡される）へ
到達できるようになる。

### 3-A. CUI・コマンド作業（SSH）

```bash
ssh ubuntu@<開発PCのTailscale IP>
```

例:

```
% ssh ubuntu@100.99.102.70
ubuntu@100.99.102.70's password:
Welcome to Ubuntu 24.04.4 LTS (GNU/Linux 7.0.0-31-generic x86_64)
...
(base) ubuntu@ubuntu-OMEN-16L-Gaming-Desktop-TG03-0xxx:~$
```

パスワードは管理者から個別に共有されたものを使う。
毎回パスワード入力したくない場合は、通常のSSH鍵運用と同様に鍵を登録できる:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_omen -N ""
ssh-copy-id -i ~/.ssh/id_ed25519_omen.pub ubuntu@<開発PCのTailscale IP>
```

### 3-B. GUI・デスクトップ操作（リモートデスクトップ）

1. RDPアプリを起動する（Mac: Microsoft Remote Desktop / Windows: リモートデスクトップ接続）
2. 接続先に開発PCのTailscale IP（例: `100.99.102.70`）を入力する
3. ユーザー名 `ubuntu` とパスワードを入力してログインする

## 開発の進め方

- リモート接続後の開発フローは、各プロジェクトのCLAUDE.md・SETUP.mdに従う
  （例: [`SETUP.md`](../../SETUP.md)、[`IsaacSim_Env/SETUP.md`](../../IsaacSim_Env/SETUP.md)）
- GPUを使う処理（Isaac Sim等）はリモート接続先の開発PC上で実行し、
  接続元PCは操作端末として使う想定
- **複数人で同じ開発PCに同時接続する可能性がある。** 作業前に`tmux`/`screen`等で
  セッションを分離し、他人の作業プロセスを誤って落とさないよう注意する

## トラブルシューティング

### 接続できない・ログインエラーが出る

```bash
tailscale status
```

を実行し、自分の端末がTailnetに`Connected`状態になっているか確認する。
`Offline`や一覧に開発PCが出てこない場合、または招待リンクの承認・アカウント作成で
つまずいた場合は、管理者（小林）に連絡する。

### パスワードが分からない

管理者（小林）に確認する。招待リンクとは別に個別共有される。

## 関連

- [Common/network/README.md](../network/README.md) — G1本体とのEthernet直結・疎通確認
  （こちらはロボット本体との通信、本ドキュメントは開発PCへの遠隔接続という別レイヤー）
- [SETUP.md](../../SETUP.md) — 操作PC側・G1本体側の環境構築全体の手順
