# 使用した外部リポジトリ

このフォルダのコードは**外部の公開リポジトリを土台にしている**。
それらは別の作者のもので、ライセンスも別なので、このリポジトリには取り込まず
**参照情報と、こちらで当てた差分だけ**を記録する。

セットアップのたびに下記を clone し、`patches/` の内容を適用してから使う。

## 1. g1-starter-kit（立ち上げの土台）

| | |
|---|---|
| URL | https://github.com/yokoshaan/g1-starter-kit |
| 使用コミット | `45ead4009600fe37e307eb1da51d46bcc1b7c4d1`（main, 2026-09-06） |
| 役割 | 疎通確認・機体構成の自動判定・指令経路の判定・テレオペ起動ラッパー |

Quest で G1 を操作し、動きを記録・再生し、頭部LiDARを見るための一式。
実機で踏む落とし穴（指令経路の食い違い、モータ無効、機体DoFの誤判定など）が
あらかじめ潰してある。今回はこのうち**テレオペ部分のみ**を使った。

このキットの価値は、実機でしか分からない前提条件をツール化している点にある。
特に次の3つは自前で書くと確実に時間を溶かす。

- `tools/preflight.py` — 有線疎通・DDS・機体DoF（23/29）の自動判定
- `tools/mode_check.py` — 指令経路（`rt/arm_sdk` / `rt/lowcmd`）とモータ有効状態の判定、`ai` の解除
- `tools/armsdk_probe.py` — 制御権の委譲が効くかを3°だけ動かして確認

## 2. xr_teleoperate（テレオペ本体）

| | |
|---|---|
| URL | https://github.com/unitreerobotics/xr_teleoperate |
| 使用コミット | `845b25a32f7febedf220e830952a7134897adb9d`（2026-08-03） |
| 作者 | Unitree Robotics（公式） |
| 役割 | XRデバイスの姿勢取得、逆運動学、G1への指令送信 |

`g1-starter-kit` は xr_teleoperate を**fork せず、手元のクローンに修正を当てる**方式を
取っている。何を当てるかは `setup/apply_patches.py --list` で確認でき、`--revert` で戻せる。

### 当たっているパッチ（キット同梱）

| 名前 | 内容 |
|---|---|
| `safe-startup` | 起動直後に腕がゼロ姿勢へ飛ぶ速度を 30 → **2 rad/s** に落とす。これが無いと起動した瞬間に両腕が高速で振られる |
| `speed-restore` | 追従開始（`r`）の時点で 30 rad/s へ戻す。`safe-startup` と対で当てる |
| `head-reference` | 腕の制御基準を `head_yaw` → `head_position` に変更。向き基準だと操作中によそを向いたときに左右がズレる |
| `record-camera` | 頭部カメラが無い環境で記録を開始してもクラッシュしないようにする |
| `motion-switcher-retry` | モード判定RPCのタイムアウトを 1.0 → 5.0 秒にし、リトライを入れる |

### こちらで追加したパッチ

| 名前 | 内容 |
|---|---|
| `state-watchdog` | **状態報告が0.2秒途切れたら送信を止めて強制終了する。** 2026-09-30 の事故対策。詳細は [`../real/quest3-g1-20261001/patches/state-watchdog.md`](../real/quest3-g1-20261001/patches/state-watchdog.md) |

**`state-watchdog` が当たっていない環境で実機を動かさないこと。**

## 環境の注意

| 項目 | 値 | 備考 |
|---|---|---|
| OS | Ubuntu 24.04 で動作 | キットのREADMEは「24.04では動きません」と書いているが、conda側がPython 3.10系を独立して持つため実質回避できている |
| Python | 3.10（conda環境 `tv`） | |
| pinocchio | 3.1.0 | **バージョン固定。上げると逆運動学が動かない** |
| numpy | 1.26.4 | 同上 |

## ライセンス

- `g1-starter-kit` — リポジトリの記載に従う（スクリプトとドキュメントは自由に使ってよい旨の記載あり）
- `xr_teleoperate` / `unitree_sdk2py` — Unitree Robotics のライセンスに従う
- `CycloneDDS` — Eclipse

このフォルダに置いているのは**こちらで書いたスクリプトと記録だけ**で、
上記リポジトリのコードは含んでいない。
