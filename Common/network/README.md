# G1との通信確認

実機G1にコマンドを送る前に、通信できているかを確認するための手順とスクリプト。
`SimpleWalk/real/`をはじめ、`Perception/`・`Mapping/`・`SLAM/`の実機デプロイでも
共通で使う。

## 前提

- G1本体の電源が入っていること
- 操作PCとG1本体がEthernetケーブルで直結されていること
- G1のIPは`192.168.123.164`固定（有線の場合）

## 手順

### 1. 操作PC側のstatic IPを設定する

`setup_ethernet_for_g1.sh`は、操作PC側のEthernetインターフェース(`enp3s0`)に
G1と同一サブネットのstatic IP(`192.168.123.200/24`)を設定する。

```bash
bash Common/network/setup_ethernet_for_g1.sh
```

- 「ケーブル接続済みですか？」で`y`
- `sudo`のパスワードが必要
- G1専用の新規接続プロファイル(`g1-link`)を作成して有効化する
  （既存のDHCP接続には触れない。既存プロファイルを直接static化しようとすると
  `ipv4.method`が反映されない不具合があったため、この方式にしている）

元のDHCP接続に戻す場合:

```bash
bash Common/network/setup_ethernet_for_g1.sh --revert
```

インターフェース名(`enp3s0`)やIP(`192.168.123.200/24`)が環境と異なる場合は、
スクリプト先頭の変数を書き換える。

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
