# Teleop/config — デモ機の設定

## `g1.env.omen`

キットの設定 `vendor/g1-starter-kit/config/g1.env` は**キットのネストした `.gitignore` に
入っているため、取り込んでもコミットされない**（`config/g1.env` の 1 行）。
そのままだと OMEN を clone し直したときに設定が消える。

そこで **OMEN で設定済みの実物を `g1.env.omen` として別にコミットし**、
`Teleop/setup.sh`（Phase 2）がキット配下へコピーする。

```bash
cp Teleop/config/g1.env.omen Teleop/vendor/g1-starter-kit/config/g1.env
```

秘密情報は入っていない（NIC 名・IP・真偽値だけ）。

## 雛形から変えてある値（2026-09-10 に OMEN から回収）

| キー | 雛形の既定 | OMEN の値 | 意味 |
|---|---|---|---|
| `G1_WIRED_IFACE` | `enp0s31f6` | **`enp129s0`** | 192.168.123.200 が付いている有線 NIC |
| `G1_HOST_IP` | `192.168.123.222` | **`192.168.123.200`** | このPC の固定 IP |
| `G1_ARM` | `auto` | **`G1_29`** | 29DoF 機体（自動判定に頼らず固定） |
| `G1_WIFI_IFACE` | （空） | **`wlp128s20f3`** | Quest と同じ WiFi 側の NIC |

既定のままの項目: `G1_ALLOW_WIRELESS_CONTROL=false` / `G1_LIDAR_FLIP=true` /
`ROS_DOMAIN_ID=42`（ロボットの Unitree DDS が domain 0 なので、必ず 0 以外にすること）

雛形との差分を見るとき:

```bash
diff Teleop/vendor/g1-starter-kit/config/g1.env.example Teleop/config/g1.env.omen
```

⚠️ 値を変えたら、**デモ機で動いている実物と `g1.env.omen` の両方**を合わせること。
片方だけ直すと clone し直したときに戻る。
