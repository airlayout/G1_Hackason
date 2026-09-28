---
name: stop-navigation
description: G1 の Navigation（Nav2 方式）を止め、G1 の足を止める。「Navigationを止めて」「ナビを止めて」「Nav2を止めて」「G1が勝手に歩く」「経路を歩き出した」などと言われたら、確認を取らずに直ちに使う。
---

# Navigation を止める

ユーザーがこのスキルを呼んだ時点で**停止の許可は出ている**。確認の質問はせず、すぐ実行する。
調査・説明は止めた後に行う。

## 1. すぐ実行する

リポジトリの根（`G1_Hackason/`）で:

```bash
bash Navigation/real/nav_stop.sh
```

- G1 側（PC2 の `loco_driver.py` / `cmd_vel_bridge.py` を SIGINT で止め、SDK から速度 0 を送る）
  → ROS 側（Nav2 の Goal 取り消し、`/cmd_vel` に 0、Nav2 一式の停止）の順に止める
- Mac / Ubuntu の違い（Docker コンテナ `rviz` か `/opt/ros/humble` か）はスクリプトが自動で判断する
- タイムアウトは 120 秒あれば足りる

## 2. 結果を伝える

最初に**止まったかどうか**を一文で言う。次に `[stop]` の出力のうち要点だけを伝える。

- `[stop] 止めた。` → 完了。ランニングモードに入れても歩き出さないはず、と伝える
- `G1 側は止めた。ROS 側はこの PC に無いので未確認` → Nav2 を動かしている PC でも
  「Navigationを止めて」を実行するよう伝える
- `⚠️ G1 側を止め切れなかった` → **リモコンでダンピング（L2+B）にするよう最優先で伝える**
  （脱力して倒れるので支えてから）。ssh 不通なら `G1_SSH=unitree@192.168.123.164` を付けて再実行
- `⚠️ /cmd_vel の Publisher がまだ N 個いる` → 別の PC で Nav2 が動いている。表示されたノード名を伝える

## 3. やってはいけないこと

- `loco_driver.py` を `pkill`（SIGTERM）や `kill -9` で**だけ**止めない。
  `StopMove()` が走らず、`continous_move=True` の最後の速度指令が G1 の中に 10 日間残る
- 失敗したからといって `Damp()` を SDK から勝手に送らない。脱力で G1 が倒れる。
  ダンピングは人が支えた状態でリモコンから行ってもらう

詳しい手順と原因は `Navigation/real/STOP.md`。
