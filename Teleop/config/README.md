# Teleop/config — デモ機の設定

## `g1.env.omen` — ⚠️ まだ入っていない

キットの設定 `vendor/g1-starter-kit/config/g1.env` は**キットのネストした `.gitignore` に
入っているため、取り込んでもコミットされない**（`config/g1.env` の 1 行）。
そのままだと OMEN を clone し直したときに設定が消える。

そこで **OMEN で設定済みの実物を `g1.env.omen` として別にコミットし**、
`Teleop/setup.sh`（Phase 2）がキット配下へコピーする、という形にする。

**2026-09-10 時点で回収できていない。** OMEN 上の `~/g1-starter-kit/config/g1.env` の
読み出しが権限分類器にブロックされたため（`.env` は秘密情報を持ちうるという保護）。
中身に秘密は無い（インターフェース名・IP・真偽値だけ）ことは
`vendor/g1-starter-kit/config/g1.env.example` から分かっている。

### 回収の仕方

```bash
scp ubuntu@192.168.123.200:g1-starter-kit/config/g1.env \
    Teleop/config/g1.env.omen
```

### 入っているはずの値（2026-09-10 に OMEN で確認済みのもの）

```
G1_WIRED_IFACE=enp129s0          # 有線 NIC。192.168.123.200 が付いている側
G1_HOST_IP=192.168.123.200
G1_ARM=G1_29                     # 29DoF 機体
```

他の項目（`G1_LIDAR_FLIP` / `G1_WIFI_IFACE` / `ROS_DOMAIN_ID` / `G1_ALLOW_WIRELESS_CONTROL`）が
既定から動かされているかは**未確認**。回収したら差分を見ること:

```bash
diff vendor/g1-starter-kit/config/g1.env.example Teleop/config/g1.env.omen
```

⚠️ 推測で `g1.env.omen` を書き起こさないこと。デモ機で動いている実物が正。
