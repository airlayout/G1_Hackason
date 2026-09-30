# G1 ネットワーク構成・接続手順（ルータ配下 Wi-Fi / Tailscale）

更新: 2026-09-29 / 「確認済み」= 実機で疎通確認

Ethernet 直結（`192.168.123.x`）の手順は [README.md](./README.md) を参照。
本書は、ハッカソン会場でモバイルルータ配下の Wi-Fi から G1 に入る場合の構成と手順。

**ポイント: 全端末（Mac・omen・G1 Jetson）は 5GHz（`Physical_AI_5G`）で接続する。
2.4GHz（`Physical_AI`）は使わない。**

## 1. 構成図

```
                    [ インターネット ]
                           │
                 ┌─────────┴─────────┐
                 │  フリーWi-Fi       │  SSID: Fujitsu_free_Wi-Fi
                 │  (上流回線)        │
                 └─────────┬─────────┘
                           │ デフォルトルート（WAN側）
                 ┌─────────┴─────────┐
                 │  モバイルルータ     │  TP-Link TL-WR1502X
                 │  (ホットスポットモード)
                 │  LAN: 192.168.0.0/24 (GW 192.168.0.1)
                 │  5GHz  : Physical_AI_5G  ← 使う
                 │  2.4GHz: Physical_AI     ← 使わない（混雑）
                 └──┬─────────┬──────────┬──┘
                    │ 5GHz    │ 5GHz     │ 5GHz
         ┌──────────┴──┐ ┌────┴────────┐ ┌┴─────────────────┐
         │ 開発PC(各自) │ │ omen         │ │ G1 / Jetson Orin  │
         │ DHCP(例 .38) │ │ 192.168.0.167│ │ 192.168.0.82      │
         └──────────────┘ └─────────────┘ └──────────────────┘

   Tailscale（tailnet: tail140c19.ts.net）で相互に到達可能
     omen 100.99.102.70 / G1 Jetson 100.78.135.14
```

## 2. 機器一覧

| 機器 | 役割 | 接続 | アドレス | 状態 |
|---|---|---|---|---|
| フリーWi-Fi | 上流回線 | インターネット | - | 確認済み（品質は §6） |
| モバイルルータ TL-WR1502X | LAN構築・中継 | フリーWi-Fiへ上流接続 | 192.168.0.0/24 (GW .1) | 確認済み |
| G1 Jetson Orin | G1の開発PC | ルータ 5GHz | LAN 192.168.0.82 / Tailscale 100.78.135.14 | 確認済み |
| 開発PC（各自の端末） | 操作端末 | ルータ 5GHz | DHCP（例: 192.168.0.38） | 確認済み（Mac で確認） |
| omen | 遠隔開発PC（Isaac Sim 機） | ルータ 5GHz | LAN 192.168.0.167 / Tailscale 100.99.102.70 | 確認済み |

**開発PC は各自の端末で、OS も IP も固定ではない**（Mac / Windows / Linux のいずれも可）。
本書の `192.168.0.38` や Mac 前提のコマンドは一例。IP は DHCP で割り当てられるので、
自分の端末の IP は `ifconfig` / `ipconfig` / `ip addr` 等で確認する。
G1 と omen は上表の IP を使う。

Wi-Fi パスワード・SSH パスワードは、Slack ワークスペース「P.AI Nexus Japan ハッカソン」
（painexusjapan.slack.com）内を検索するか、他のメンバーに聞く（リポジトリには書かない）。

## 3. G1 への接続手順（同一ルータ配下）

1. 端末を `Physical_AI_5G`（5GHz）に接続
2. 疎通確認（Mac / Linux）

   ```bash
   ping -c 5 192.168.0.82
   nc -z 192.168.0.82 22 && echo open
   ```

   Windows（PowerShell）の場合:

   ```powershell
   ping -n 5 192.168.0.82
   Test-NetConnection 192.168.0.82 -Port 22
   ```

3. SSH

   ```bash
   ssh unitree@192.168.0.82
   ```

注意点:

- 別の Wi-Fi（フリーWi-Fi等）に繋いだままでは G1 に届かない。必ずルータ配下へ
- IP は DHCP のため変わる可能性あり。繋がらない場合は `arp -a` かルータ管理画面で再確認
- 初回 ping は ARP 解決待ちで 1〜2 発落ちることがある

## 4. Tailscale 経由のアクセス

tailnet に招待（またはノード共有）されたアカウントのみ到達可能
（招待の流れは [Common/remote_access/README.md](../remote_access/README.md)）。

| 機器 | Tailscale IP | 接続コマンド |
|---|---|---|
| omen | 100.99.102.70 | `ssh ubuntu@100.99.102.70` |
| G1 Jetson | 100.78.135.14 | `ssh unitree@100.78.135.14` |

- 未招待の人は管理者に依頼
- 同一 LAN 内では Tailscale は直接経路（192.168.0.x）を使うので遅延は増えない

## 5. SSH 鍵認証（任意）

Jetson はパスワード認証と鍵認証の両方を受け付ける。鍵を使う人は自分の公開鍵を登録する。

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_g1
ssh-copy-id -i ~/.ssh/id_ed25519_g1.pub unitree@192.168.0.82   # パスワード入力あり
```

`~/.ssh/config` 例:

```
Host g1
  HostName 192.168.0.82
  User unitree
  IdentityFile ~/.ssh/id_ed25519_g1
  IdentitiesOnly yes
```

## 6. 備考: ネット回線品質・問題点

LAN 内（Mac / omen / Jetson 間）は 5GHz で良好。問題は上流のインターネット側。

| 項目 | 2.4GHz | 5GHz |
|---|---|---|
| Mac→Jetson 遅延（平均/最大） | 36 / 447 ms | 15 / 121 ms |
| omen→Jetson 遅延（平均/最大） | 174〜270 / 588〜1457 ms | 12 / 49 ms |
| Jetson→インターネット 遅延 | 149 / 752 ms（ロス2%） | 16 / 70 ms |

- 2.4GHz が遅い原因: ch11 が上流や周辺 AP と重なり混雑 + Wi-Fi 省電力
- 上流（共有フリーWi-Fi）が不安定: インターネット向けは 15〜130 ms と揺れ、
  ダウンロード速度も 0〜110 Mbps で大きく変動する。ロスは無し
- ルータはホットスポットモードのため 5GHz のチャネルを固定できない
  （上流に追従。現在 ch60 = DFS）。上流 AP の切り替えでチャネルが変わることがある
- 大きなファイルのダウンロードは事前に済ませておくと安全
- 将来の改善案: SIM(LTE/5G) または有線 WAN を上流にできるルータへ変更

## 7. 端末側の設定（5GHz を確実に使う）

以下は **Linux 端末（omen など）の場合**。Mac / Windows は OS が国コードを自動設定するため
通常は不要で、`Physical_AI_5G` を選んで繋ぎ、`Physical_AI`（2.4GHz）を自動接続から外せばよい。

国コードを JP にする（Linux）。未設定（`iw reg get` が country 00）だと、5GHz が見えない／
DFS チャネル(52〜64)に繋げない。

```bash
sudo iw reg set JP
echo "options cfg80211 ieee80211_regdom=JP" | sudo tee /etc/modprobe.d/cfg80211-jp.conf   # omen
```

Jetson は `regdom-jp.service`（起動時に `iw reg set JP`）を導入済み。

5GHz 優先・省電力オフ:

```bash
sudo nmcli connection modify "Physical_AI_5G" 802-11-wireless.powersave 2
sudo nmcli connection modify "Physical_AI_5G" connection.autoconnect-priority 100
sudo nmcli connection modify "Physical_AI" connection.autoconnect-priority 1
```

## 8. トラブルシュート

| 症状 | 原因の候補 | 対処 |
|---|---|---|
| G1 に ping が通らない | PC が別 Wi-Fi に接続 | `Physical_AI_5G` に繋ぎ替え |
| 遅延が数百 ms／ジッタ大 | 2.4GHz 接続、省電力 on | 5GHz に接続、§7 |
| 5GHz の SSID が見えない／Jetson が再起動後に繋がらない | 国コード未設定（DFS チャネル不可） | `sudo iw reg set JP`（§7） |
| ネットは使えるが G1 に届かない | ルータの AP 分離 | ルータ設定で無効化 / Tailscale |
| G1 の IP が分からない | DHCP で変更 | `arp -a` / Tailscale IP を使う |
| ルータ配下でネットに出られない | フリーWi-Fiのログイン画面未認証 | ルータ側で認証を通す |
| G1 が Wi-Fi に繋がらない | Jetson の Wi-Fi 未設定 | 有線(192.168.123.x)で入り `nmcli device wifi connect "Physical_AI_5G" password <Slack で確認>` |

## 関連

- [README.md](./README.md) — Ethernet 直結での疎通確認
- [Common/remote_access/README.md](../remote_access/README.md) — Tailscale による開発 PC への遠隔接続
