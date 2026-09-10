# Demo/ — 当日の入口

Ubuntu ゲーミング PC（`192.168.123.200`）1 台だけを操作して、3 つを見せる。

> 当日の手順書 `Demo/README.html`（図つき）は **Phase 5 で作る**。まだ無い。
> それまではこの README と `bash Demo/demo.sh` を見ること。

```bash
bash Demo/preflight.sh     # まずこれ。何が使えるか列挙して終わる
bash Demo/demo.sh          # 番号を選ぶだけのメニュー
bash Demo/stop.sh          # 全部止める（ターミナルを閉じてしまったとき）
```

| | 中身 | 実機 | 状態 |
|---|---|---|---|
| 項目2 | `01_lidar.sh` LiDAR で計測 | 再生は不要 / ライブは要 | 再生 ✅ 実測 / ライブ ❌ 未検証 |
| 項目3 | `02_isaac_nav2.sh` Isaac Sim + Nav2 | 不要 | ✅ 実測 |
| 項目4 | `03_teleop.sh` Quest でエピソード | **要** | ❌ 未着手（Phase 4） |

## 2026-09-10 に OMEN で実測したこと

**項目3 — 通った。**

```
bash Demo/02_isaac_nav2.sh
  Isaac Sim の起動  16 秒
  Nav2 の起動       24 秒
  RViz2             自動で開いた
  ゴール (5.07, -0.93) → SUCCEEDED / error_code: 0
                          10.5 秒 / リカバリ 0 回 / 誤差 0.14 m
bash Demo/stop.sh
  6 秒で停止・孤児ゼロ
```

**項目2 の再生 — 通った。ただし記録に手当てが要った。**

Humble で録った db3 は Jazzy の `ros2 bag play` が開けない
（`ros2 bag info` は通るので気づきにくい）。`repair_bag_for_jazzy.py` が追記だけで直す。
`lidar_replay.sh` が自動で呼ぶので、当日は意識しなくてよい。
修復後、136,047 msg が再生でき、RViz2 が 3 トピックとも購読した。

**項目4 — 実機が要るので未着手。** キット（`Teleop/vendor/g1-starter-kit/`）は
取り込み先から動くことだけ確認済み（`Teleop/README.md`）。

## 設計の約束

- **設定は `Demo/lib.sh` の 1 箇所だけ。** 値は `Teleop/config/g1.env.omen` から読んで
  二重に持たない。デモ機ごとの上書きは `Demo/demo.env`（追跡しない）
- **`Demo/` は入口だけ。** 実装は領域フォルダが持つ
  （項目2 → `Mapping/real/ubuntu/`、項目3 → `IsaacSim_Env/`、項目4 → `Teleop/`）
- **止め方を再実装しない。** Isaac Sim + Nav2 は `IsaacSim_Env/stop_nav2.sh` が
  正しい signal の向きを知っている。`lib.sh` はそれを呼ぶだけ

## 踏んだ罠（消さないこと）

- **止める signal は送る先で逆になる。** 外側のスクリプトには SIGTERM
  （SIGINT は `SIG_IGN` で黙って無視される）、内側の `ros2 launch` には SIGINT
  （SIGTERM だと子を孤児化する）
- **`pgrep -af` は自分自身と `bash -c` のラッパにマッチする。** 除外を忘れると
  呼び出し元の ssh セッションごと殺す（2026-09-09 に事故。2026-09-10 にも
  `pkill -f apport` で同じことを踏んだ）。だから**待ちは pgrep でなくログの grep**
- **`$(grep -c ... || echo 0)` と書かない。** 0 件のとき grep は「0」を印字したうえで
  exit 1 を返すので `"0\n0"` になる
- **いきなり `kill -9` しない。** RViz2 を kill -9 すると apport が拾いに来て
  ゾンビと apport が残る。SIGINT → SIGTERM → SIGKILL の順に上げる
- **`ping` で機器の生死を見ない。** 「コマンドが無い」と「届かない」が同じ NG になる。
  `lib.sh` の `tcp_probe`（`/dev/tcp`）を使う
- **`ros2 topic hz` は `--no-daemon` を受け付けない**（`list` と `info` と `echo` は通る）
- **macOS の rsync は openrsync。** `--info=progress2` を付けると usage エラーで
  黙って終わる（Mac から記録を押すときに踏む）
