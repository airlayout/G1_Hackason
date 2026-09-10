# Teleop — G1 を座らせて Meta Quest でエピソード記録・再生（デモ項目4）

Quest のコントローラで G1 の腕を動かし、その動きを記録して再生する。
**実体は `vendor/g1-starter-kit/`（外部リポジトリの取り込み）**で、このフォルダ直下の
スクリプトはデモの都合（保存先・タスク名・`Demo/` からの呼ばれ方）だけを引き受ける薄い層。

⚠️ **実機の腕を動かす。`SAFETY.md` を読んでから触ること。**（`SAFETY.md` は Phase 2 で作成）

## 現在の状態（2026-09-10）

Phase 0 が済んだところ。**キットを取り込んだだけで、まだ薄い層は書いていない。**
いまキットを直接使うなら `vendor/g1-starter-kit/README.md` の手順に従うこと。

| ファイル | 状態 |
|---|---|
| `vendor/g1-starter-kit/` | ✅ 取り込み済み（38ファイル / `45ead40`） |
| `config/g1.env.omen` | ⚠️ 未回収（`Teleop/config/README.md` 参照） |
| `SAFETY.md` `setup.sh` `record.sh` `replay.sh` `list.sh` `quest_ap.sh` | ❌ 未着手 |

## なぜ vendor を丸ごとコミットしているか

**このリポジトリの他の場所とはやり方が違う。**
`Mapping/real/vendor/` は外部リポジトリを `.gitignore` して `.repos` で版だけ固定しているが、
ここは中身ごとコミットしている。

理由は、デモ機（Ubuntu 24.04 / ROS 2 Jazzy）向けに**キット本体へ手を入れる見込みがある**ため
（上流の動作確認環境は 22.04 / Humble）。submodule や `.repos` だと手元の修正を持ち回せない。

出所・commit・許諾・上流の取り直し手順は `vendor/g1-starter-kit/UPSTREAM.md` にある。

## 設計上の約束

- キットが既に潰した落とし穴（DoF 自動判定・指令トピック自動選択・腕速度の安全パッチ）を
  **書き直さない**。薄い層から呼ぶだけにする
- `setup/install_apt.sh` は**呼ばない**（`:35` が `jammy` 以外を `die` する。デモ機は 24.04）
- `setup/apply_patches.py` は**省略できない**。これはキットではなく `~/xr_teleoperate` 側を
  書き換えるもの（腕速度 30→2 rad/s の安全パッチ、カメラ無しでも記録できるパッチ）
- 項目4 は **ROS 2 に依存しない**（キットで ROS を使うのは `lidar_view.sh` と
  `record_bag.sh` だけ。テレオペと再生は conda 環境 `tv` の素の Python）
