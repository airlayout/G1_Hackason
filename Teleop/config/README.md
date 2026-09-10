# Teleop/config — デモ機の設定

## `g1.env.omen` は **git で追跡しない**（`.gitignore:60`）

環境ごとに違う値（NIC 名・IP・機体構成）なので、リポジトリには入れずに
**各デモ機のローカルで持つ**。つど書き換えてよい。

そのぶん **clone し直すと消える。** 復旧手順を下に置いておくこと。

```bash
# 既にデモ機に設定済みのキットがあるなら、そこから取る（いちばん確実）
cp ~/g1-starter-kit/config/g1.env Teleop/config/g1.env.omen

# 無ければ雛形から起こして、下の表の値を入れる
cp Teleop/vendor/g1-starter-kit/config/g1.env.example Teleop/config/g1.env.omen
```

## なぜキットの `config/g1.env` に直接書かないのか

キットの `.gitignore` が `config/g1.env` を除外しているので、そこに書くと
**キットを入れ替えたときに一緒に消える**。1 段外に置いて、
`Teleop/setup.sh`（Phase 2）がキット配下へコピーする形にする。

```bash
cp Teleop/config/g1.env.omen Teleop/vendor/g1-starter-kit/config/g1.env
```

## OMEN（`192.168.123.200`）の値 — 2026-09-10 に実機から回収して確認

雛形から変えるのはこの 4 つだけ。

| キー | 雛形の既定 | OMEN の値 | 意味 |
|---|---|---|---|
| `G1_WIRED_IFACE` | `enp0s31f6` | **`enp129s0`** | 192.168.123.200 が付いている有線 NIC |
| `G1_HOST_IP` | `192.168.123.222` | **`192.168.123.200`** | このPC の固定 IP |
| `G1_ARM` | `auto` | **`G1_29`** | 29DoF 機体（自動判定に頼らず固定） |
| `G1_WIFI_IFACE` | （空） | **`wlp128s20f3`** | Quest と同じ WiFi 側の NIC |

既定のまま使う項目: `G1_ALLOW_WIRELESS_CONTROL=false` / `G1_LIDAR_FLIP=true` /
`ROS_DOMAIN_ID=42`（ロボットの Unitree DDS が domain 0 なので、**必ず 0 以外**にすること）

秘密情報は入っていない（NIC 名・IP・真偽値だけ）。追跡しないのは秘密だからではなく、
**環境ごとに違うから**。

雛形との差分を見るとき:

```bash
diff Teleop/vendor/g1-starter-kit/config/g1.env.example Teleop/config/g1.env.omen
```
