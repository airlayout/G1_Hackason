# セットアップ手順

`G1_HuggingFace/`配下（HuggingFace LeRobot + Unitree SDK経由の環境）を新しいマシンで
ゼロから構築するための手順。`SimpleWalk/`・`Perception/`・`Mapping/`・`SLAM/`など、
このリポジトリの機能フォルダはすべてこの環境を共通で使う。

所要時間は環境構築込みで1〜2時間（G1本体側のビルドも含めるとさらに数十分）。

## 全体構成

```
操作PC (このリポジトリをcloneするマシン)          G1本体 (Jetson, Ubuntu 20.04 aarch64)
├─ venv (Python 3.12)                          ├─ conda環境 "lerobot" (Python 3.12)
├─ cyclonedds/ (ソースビルド)                    ├─ cyclonedds/ (ソースビルド、別途必要)
├─ unitree_sdk2_python/                         ├─ unitree_sdk2_python/ (同上)
└─ lerobot/                                     └─ lerobot/ (同上)
        │                                               │
        └── Ethernet(192.168.123.x) ──────────────────┘
             操作PC: walk_forward_real.py 等         G1: run_g1_server.py (常駐)
```

**操作PC側と実機G1側は別々のマシンで、それぞれ独立に環境構築が必要。**
シミュレーションのみ試す場合は操作PC側だけで良い。

## 必要なもの

| 項目 | 要件 | 備考 |
|---|---|---|
| OS（操作PC） | Ubuntu 24.04 | 動作確認環境。他バージョンは未検証 |
| OS（G1本体） | Ubuntu 20.04.6 LTS (aarch64) | 出荷時のまま。Python 3.8.10だが3.9+の構文を使うlerobotは動かないため後述のPython 3.12環境が必要 |
| ディスク | 数GB以上 | torch等を含めるとPython環境だけで2〜3GB程度 |
| ネットワーク | 操作PC: インターネット接続必須 | G1本体: 最初はインターネット無し。操作PC経由で共有する（後述） |

操作PC・G1本体ともに`cmake`/`gcc`/`g++`が必要（CycloneDDSのビルド用）。
Ubuntu標準で入っていることが多いが、無ければ`sudo apt install cmake gcc g++`。

---

## 1. 操作PC側のセットアップ

### 1.1 Python venv

conda不要。素のvenvで十分（pinocchioが必要な機能=`gravity_compensation`を
使わない限り、conda-forge限定のパッケージは不要）。

```bash
cd G1_HuggingFace
python3 -m venv venv
./venv/bin/pip install --upgrade pip
```

### 1.2 CycloneDDS をソースからビルド

```bash
cd G1_HuggingFace
git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds
cd cyclonedds
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install -- -j$(nproc)
```

### 1.3 unitree_sdk2_python

```bash
cd G1_HuggingFace
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
export CYCLONEDDS_HOME=$(pwd)/../cyclonedds/install
../venv/bin/pip install -e .
```

### 1.4 lerobot（`[unitree_g1]` extra込み）

```bash
cd G1_HuggingFace
git clone https://github.com/huggingface/lerobot.git
cd lerobot
export CYCLONEDDS_HOME=$(pwd)/../cyclonedds/install
../venv/bin/pip install -e '.[unitree_g1]'
```

`torch`/`torchvision`/`onnxruntime`等も一緒に入る（CPU版で問題ない。GPUは使わない）。

### 1.5 シミュレーション用の追加パッケージ

`lerobot/unitree-g1-mujoco`（HuggingFace Hub上、trust_remote_code経由で自動取得される
MuJoCo環境）が要求するもの。`[unitree_g1]`extraには含まれないため個別に入れる。

```bash
./venv/bin/pip install mujoco scipy msgpack msgpack-numpy loguru
```

### 1.6 動作確認

```bash
export CYCLONEDDS_HOME=$(pwd)/G1_HuggingFace/cyclonedds/install
./G1_HuggingFace/venv/bin/python SimpleWalk/sim/verify_g1_sim_command.py
```

`RESULT_OK`が出れば成功。初回はHuggingFace Hubから約55MBのMuJoCo環境がダウンロードされる。

**注意（GPU無し環境の場合）**: MuJoCoのレンダリング用に`MUJOCO_GL=osmesa`を
明示的に指定すると、システムに`libOSMesa`が無くて失敗することがある。
`MUJOCO_GL`を**指定しない**（デフォルトのGLFW/Mesa経由）方が、GPUドライバの
状態に関わらず動きやすい。

---

## 2. G1本体とのネットワーク接続

### 2.1 有線Ethernetでの接続

`Common/network/setup_ethernet_for_g1.sh`を使う。G1のIPは`192.168.123.164`固定。

```bash
bash Common/network/setup_ethernet_for_g1.sh
```

操作PC専用の接続プロファイル(`g1-link`)を新規作成し、static IP
(`192.168.123.200/24`)を設定する（既存のDHCP接続には影響しない）。
元に戻す場合は`bash Common/network/setup_ethernet_for_g1.sh --revert`。

### 2.2 疎通確認

```bash
python3 Common/network/check_g1_connectivity.py --ssh-password 123
```

`READY`が出れば、ping・SSH(22番ポート)とも到達できている。
（`123`は工場出荷時デフォルトパスワード。この機体で異なる場合は各自のパスワードに置き換える）

### 2.3 SSH鍵の登録（推奨）

毎回パスワード入力しなくて済むように:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_g1 -N ""
ssh-copy-id -i ~/.ssh/id_ed25519_g1.pub unitree@192.168.123.164
```

`~/.ssh/config`に以下を追記すると`ssh g1`だけで接続できる:

```
Host g1
    HostName 192.168.123.164
    User unitree
    IdentityFile ~/.ssh/id_ed25519_g1
    IdentitiesOnly yes
```

### 2.4 G1本体にインターネットを共有する

G1本体はデフォルトでインターネットに出られない。パッケージのインストール等に
必要なため、操作PCの回線（例: WiFi）を共有する。

**操作PC側**（WiFiインターフェース名・Ethernetインターフェース名は`ip -br a`で確認）:

```bash
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o <WiFiのIF名> -s 192.168.123.0/24 -j MASQUERADE
sudo iptables -A FORWARD -i <WiFiのIF名> -o <EthernetのIF名> -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i <EthernetのIF名> -o <WiFiのIF名> -j ACCEPT
```

**G1側**（`ssh g1`でログインして実行。Ethernetインターフェース名は`ip -br addr show`で確認、通常`eth0`）:

```bash
sudo ip route del default 2>/dev/null || true
sudo ip route add default via 192.168.123.200 dev eth0
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf
ping -c 3 8.8.8.8   # 通ることを確認
```

---

## 3. G1本体側のセットアップ

**この節はすべて`ssh g1`でログインした先（G1本体）で実行する。**

### 3.1 既存環境の確認

G1本体は出荷時のままだと`unitree_sdk2py`（Python版）も`lerobot`も入っていない。
また標準のPythonは3.8.10で、`lerobot`が要求する3.9+の構文（`dict[str, ...]`等）に
対応していないため、**別途Python 3.12環境が必須**。

```bash
python3 --version           # 3.8.10 (これはそのままで良い、変更不要)
which conda                  # 何も出ないはず
```

### 3.2 Miniforge (aarch64) + Python 3.12環境

```bash
wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh -O ~/miniforge_installer.sh
bash ~/miniforge_installer.sh -b -p ~/miniforge3
~/miniforge3/bin/conda create -y -n lerobot python=3.12
```

以降、G1側での作業は毎回:

```bash
source ~/miniforge3/bin/activate lerobot
```

してから行う。

### 3.3 CycloneDDS をソースからビルド（G1本体、aarch64）

操作PC側と同じ手順。**アーキテクチャが違うので、操作PC側でビルドしたものは
流用できず、G1本体上で再度ビルドする必要がある。**

```bash
cd ~
git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds
cd cyclonedds
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install -- -j$(nproc)
```

### 3.4 unitree_sdk2_python（Python 3.12環境内）

```bash
source ~/miniforge3/bin/activate lerobot
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
export CYCLONEDDS_HOME=~/cyclonedds/install
pip install -e .
```

### 3.5 lerobot（`[unitree_g1]` extra込み）

```bash
source ~/miniforge3/bin/activate lerobot
cd ~
git clone https://github.com/huggingface/lerobot.git
cd lerobot
export CYCLONEDDS_HOME=~/cyclonedds/install
pip install -e '.[unitree_g1]'
```

torch含めaarch64向けのwheelがそのまま入る（ビルド不要、数分で完了）。

### 3.6 動作確認（ブリッジサーバーの起動）

```bash
source ~/miniforge3/bin/activate lerobot
cd ~/lerobot
export CYCLONEDDS_HOME=~/cyclonedds/install
export LD_LIBRARY_PATH=~/cyclonedds/install/lib:$LD_LIBRARY_PATH
python -u src/lerobot/robots/unitree_g1/run_g1_server.py
```

`bridge running (lowstate -> zmq, lowcmd -> dds)`と表示されれば成功。
`Ctrl-C`で停止。SSH切断後も動かし続けたい場合は`nohup ... &`でバックグラウンド化する:

```bash
nohup python -u src/lerobot/robots/unitree_g1/run_g1_server.py > ~/g1_server.log 2>&1 &
disown
```

### 3.7 ブリッジポートへの到達性確認（操作PC側から）

```bash
python3 Common/network/check_g1_connectivity.py --check-bridge-ports
```

`lowcmd(6000)`・`lowstate(6001)`が「到達可能」になっていればOK。

---

## 4. つまずきやすい点

- **`nmcli connection modify`で既存プロファイルの`ipv4.method`が反映されない**
  netplan生成のプロファイルで発生した(原因未特定)。既存プロファイルは触らず、
  専用の新規プロファイルを作る方式で回避した(`Common/network/setup_ethernet_for_g1.sh`参照)。
- **`pip install pyzmq`がaarch64+Python3.8でビルド失敗する**
  (`numpy has no attribute 'get_include'`)。`apt install python3-zmq`か、
  Python 3.12環境なら通常のpip installで解決する。
- **`MUJOCO_GL=osmesa`を明示指定すると`libOSMesa`が無くて失敗する**
  ことがある。指定しない（デフォルト）方が動きやすい。
- **G1本体のPython 3.8では`lerobot`をimportすらできない**
  (`dict[str, list[str]]`のような3.9+構文を`__init__.py`が使っている)。
  Python 3.12環境を別途用意する必要がある(本手順の3.2〜3.5)。
- **実機で`disconnect()`すると関節が脱力する** — 支えが無い状態で呼ぶと転倒する。
  `SimpleWalk/real/walk_forward_real.py`の安全確認ゲートを参照。

より詳しい経緯は各機能フォルダの`FAILURES.md`と`G1_HuggingFace/Note`を参照。

## 5. 次に読むもの

- [SimpleWalk/README.md](SimpleWalk/README.md) — 前進歩行（シム・実機とも動作確認済み）
- [G1_HuggingFace/README.md](G1_HuggingFace/README.md) — 検証済みの数値・既知のバグの詳細
- [Common/network/](Common/network/) — 疎通確認・Ethernet設定スクリプト
