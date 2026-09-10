# このディレクトリは外部リポジトリの取り込みです

`g1-starter-kit` は**このリポジトリで書いたものではありません**。
上流をそのままコピーして置いています。

| | |
|---|---|
| 取得元 | https://github.com/yokoshaan/g1-starter-kit |
| commit | `45ead40`（`docs: 実機で踏んだ落とし穴を反映（指令経路・モータ無効・arm_sdk・無線）`） |
| 取得日 | 2026-09-10 |
| 取得元のホスト | `ubuntu@192.168.123.200:~/g1-starter-kit`（clone 済みのものを複製） |
| 作者 | Kouta Yokoyama（`mtside01@gmail.com` / `k.yokoyama@play-robotics.com`） |
| 取り込んだ範囲 | `git ls-files` の 38 ファイルすべて。`.git/` と `config/g1.env` は**含めていない** |

## 許諾

上流に `LICENSE` ファイルは**ありません**。許諾は README の「ライセンスと出典」節の
以下の一文だけです（原文引用）。

> このリポジトリのスクリプトとドキュメントは自由に使ってください。以下は各々のライセンスに従います。
>
> - [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) — Unitree Robotics
> - [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) — Unitree Robotics
> - [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) / [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) — Livox
> - [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) — Eclipse

キットが依存する上記 4 つの外部プロジェクトは**ここには含まれていません**。
それぞれのライセンスに従って各自の環境に導入されます（`setup/install_env.sh` 等）。

## なぜ submodule ではなく丸ごと置いているか

デモ機（OMEN / Ubuntu 24.04 / ROS 2 Jazzy）向けに**キット本体へ手を入れる可能性がある**ため
（上流の動作確認環境は 22.04 / Humble）。submodule だと手元の修正を持ち回せません。

この判断はこのリポジトリの既存方針（外部リポジトリは `.gitignore` して `.repos` で版だけ固定。
`Mapping/real/vendor/` がその形）と**違います**。理由は `Teleop/README.md` にも書いてあります。

## 手を入れたら

**上流と差し替えたくなるので、変更箇所は必ずコメントで分かるようにしてください。**
取り込んだ以上 `git diff upstream` が使えません。

```
# [G1_Hackason 改変] jazzy 対応。上流 45ead40 は humble 前提。
```

## 上流を取り直す手順

```bash
# 1. 比較元を用意する（OMEN には ~/g1-starter-kit が残してある。消さないこと）
git clone https://github.com/yokoshaan/g1-starter-kit /tmp/kit-upstream
cd /tmp/kit-upstream && git log --oneline 45ead40..HEAD   # 取り込み後に上流で何が起きたか

# 2. 手元の改変を洗い出す（45ead40 との差分＝こちらが足したもの）
cd /tmp/kit-upstream && git checkout 45ead40
diff -ru --exclude=.git --exclude=UPSTREAM.md \
  /tmp/kit-upstream \
  <このリポジトリ>/Teleop/vendor/g1-starter-kit

# 3. 上流の新しい commit を取り込むなら、2 の差分を当て直す
```

## 取り込みに際して除外したもの

- `.git/` — 残すと親リポジトリが embedded repository として扱い、壊れた gitlink になる
- `config/g1.env` — キットの `.gitignore` に入っている環境固有設定。
  OMEN 用の値は `Teleop/config/g1.env.omen` にあり、`Teleop/setup.sh` がここへコピーする
