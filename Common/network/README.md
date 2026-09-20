# G1との通信確認

実機G1にコマンドを送る前に、通信できているかを確認するための手順とスクリプト。
`SimpleWalk/real/`をはじめ、`Perception/`・`Mapping/`・`Navigation/`の実機デプロイでも
共通で使う。

## 前提

- G1本体の電源が入っていること
- 操作PCとG1本体がEthernetケーブルで直結されていること
- G1のIPは`192.168.123.164`固定（有線の場合）

## 手順

### 1. 操作PC側のstatic IPを設定する

`setup_ethernet_for_g1.sh`は、操作PC側のEthernetインターフェース(`enp3s0`)に
G1と同一サブネットのstatic IPを設定する。**アドレスは決め打ちせず、空いているものを
`arping`のDAD(重複アドレス検知)で探して選ぶ。**

```bash
bash Common/network/setup_ethernet_for_g1.sh          # 空きを自動で選ぶ
bash Common/network/setup_ethernet_for_g1.sh 222      # 192.168.123.222 を使う
```

- 「ケーブル接続済みですか？」で`y`
- `sudo`のパスワードが必要
- G1専用の新規接続プロファイル(`g1-link`)を作成して有効化する
  （既存のDHCP接続には触れない。既存プロファイルを直接static化しようとすると
  `ipv4.method`が反映されない不具合があったため、この方式にしている）
- 自動選択の走査範囲は`192.168.123.200`〜`.250`。`.1`・`.120`(LiDAR)・`.161`(PC1)・
  `.164`(PC2)は候補から外れ、明示指定しても拒否される
- 自動選択には`arping`が要る（`sudo apt install -y iputils-arping`）。
  入れない場合はアドレスを引数で明示する

環境に合わせて変えられるもの:

| 環境変数 | 既定 | 用途 |
|---|---|---|
| `G1_IFACE` | `enp3s0` | 有線NIC名 |
| `G1_DHCP_CONN` | `netplan-enp3s0` | `--revert`で戻す先のプロファイル名 |
| `G1_SCAN_FROM` / `G1_SCAN_TO` | `200` / `250` | 自動選択で走査する範囲 |

元のDHCP接続に戻す場合:

```bash
bash Common/network/setup_ethernet_for_g1.sh --revert
```

#### なぜアドレスを決め打ちしないか

**G1のリンクにはG1本体以外の作業者もぶら下がっている。** 以前はこのスクリプトが
`.200`を決め打ちしていたため、全員が同じアドレスを要求して衝突した。

| 日付 | 症状 |
|---|---|
| 2026-08-26 | `エラー: 接続のアクティベーションに失敗: IP 設定を確保できませんでした` |
| 2026-09-06 | macOS側が重複を検知して`.200`を放棄。`ifconfig`にIPv4が無いのに設定は`Manual .200`のまま残り、サービスをoff/onしても戻らなかった |

どちらも**設定だけが残って疎通が無言で消える**ので気付きにくい。現在はスクリプトが
繋ぐ前にDADで確認し、埋まっていれば別のアドレスを選ぶ（明示指定が埋まっていた場合は
そこで止める）。既存の`g1-link`プロファイルがあっても毎回アドレスを入れ直すので、
以前あった「`PC_IP`を書き換えても反映されない」問題も起きない。

リンク上の在席を自分で見たいときは:

```bash
ip neigh show dev enp3s0                            # 応答のあった機器を一覧する
sudo arping -D -c2 -w2 -I enp3s0 192.168.123.222    # 戻り値0なら空き、1なら使用中
```

DADは自分のMACからの応答を無視するので、**自分が既に持っているアドレスは「空き」と出る**。
同じPCで再実行しても自分自身とは衝突しない。

### 2. 疎通確認スクリプトを実行する

```bash
python3 Common/network/check_g1_connectivity.py
```

以下を順に確認する（`lerobot`環境は不要、標準ライブラリのみで動く）:

| 項目 | 確認内容 |
|---|---|
| 1/3 | 操作PCに`192.168.123.0/24`のIPが設定されているか |
| 2/3 | G1(`192.168.123.164`)への`ping`応答 |
| 3/3 | G1の22番ポート(SSH)への到達性 |

最後に`READY`と表示されれば、通信レベルでの疎通は確認できている。

### 3. オプション: SSHログインまで確認する

`sshpass`がインストールされていれば、実際にSSHログインできるかまで確認できる:

```bash
python3 Common/network/check_g1_connectivity.py --ssh-password 123
```

（`123`はUnitreeの工場出荷時デフォルトパスワード。この機体で異なる場合は
実際のパスワードに置き換える）

`sshpass`が無い場合はスキップされるので、自分で`ssh unitree@192.168.123.164`を
手動実行して確認しても良い。

### 4. オプション: ブリッジサーバーのポートまで確認する

G1側で`run_g1_server.py`（DDS-ZMQブリッジ）が起動済みであれば、そのポートへの
到達性も確認できる:

```bash
python3 Common/network/check_g1_connectivity.py --check-bridge-ports
```

| ポート | 用途 |
|---|---|
| 6000 | lowcmd（操作PC→G1への関節コマンド） |
| 6001 | lowstate（G1→操作PCへのロボット状態） |
| 5555 | カメラ映像（`run_g1_server.py --camera`時のみ） |

`run_g1_server.py`が未起動の場合、これらは「到達不可」になるのが正常。

**このオプションは`lerobot`方式（`SimpleWalk/real/walk_forward_real.py`）専用**。
`walk_forward_real.py`は`run_g1_server.py`経由のZMQブリッジでG1と通信するため、
このポート確認が意味を持つ。一方`walk_forward_real_sdk.py`（Unitree SDK標準の
`LocoClient`を使う方式）は操作PCから直接DDS接続するため、これらのZMQポートは
一切使わない。SDK方式の疎通確認は、ping/SSH確認（1〜3）だけで十分。

| | `check_g1_connectivity.py`の対応範囲 |
|---|---|
| `setup_ethernet_for_g1.sh` | 両方式で共通 |
| ping/SSH確認（オプション無し） | 両方式で共通 |
| `--check-bridge-ports` | lerobot方式（`walk_forward_real.py`）専用 |

### 5. WiFi接続の場合

WiFi接続時はIPが可変になるため、`--host`でG1の実際のIPを指定する:

```bash
python3 Common/network/check_g1_connectivity.py --host <WiFiのIP>
```

（G1のWiFiは初期状態で無効。有効化手順は`G1_HuggingFace/README.md`の
「Enable WiFi on the Robot」を参照）

## パスワード無しでSSH接続したい場合

毎回パスワードを入力したくない場合は、SSH鍵を登録する:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_g1 -N ""
ssh-copy-id -i ~/.ssh/id_ed25519_g1.pub unitree@192.168.123.164
```

`~/.ssh/config`に以下を追記すると`ssh g1`だけで接続できるようになる:

```
Host g1
    HostName 192.168.123.164
    User unitree
    IdentityFile ~/.ssh/id_ed25519_g1
    IdentitiesOnly yes
```

## 関連

- [SETUP.md](../../SETUP.md) — 操作PC側・G1本体側の環境構築全体の手順
- [SimpleWalk/real/walk_forward_real.py](../../SimpleWalk/real/walk_forward_real.py) — 疎通確認の後に実行する歩行スクリプトの例
