---
name: stop-navigation
description: G1 の Navigation（Nav2。nav2_stable の巡回・Goal、旧 loco_driver 構成も含む）を止め、G1 の足を止める。「Navigationを止めて」「ナビを止めて」「Nav2を止めて」「巡回を止めて」「G1が勝手に歩く」「経路を歩き出した」などと言われたら、確認を取らずに直ちに使う。
---

# Navigation を止める

ユーザーがこのスキルを呼んだ時点で**停止の許可は出ている**。確認の質問はせず、すぐ実行する。
調査・説明は止めた後に行う。

## 1. すぐ実行する

リポジトリの根（`G1_Hackason/`）で:

```bash
bash Navigation/real/nav_stop.sh
```

- G1 の PC2 に ssh し、nav2_stable（`/g1/estop` → 巡回 stop → 走行許可取消 → Goal 取消 → launch 停止）、
  旧 `loco_driver.py`（SIGINT）、SDK からの速度 0、発進ゲートの閉鎖を順に行う
- そのあと操作PCに旧構成の Nav2 があれば止める。Mac / Ubuntu の違いはスクリプトが自動で判断する
- タイムアウトは 180 秒あれば足りる

## 2. 結果を伝える

最初に**止まったかどうか**を一文で言う。次に `[stop]` の出力のうち要点だけを伝える。

- `[stop] 止めた。` → 完了。再開には `g1up.sh` での上げ直しと `/g1/clear_estop` が要る、と添える
- `止めた。ただし発進ゲートが開いたまま` → 表示された `ssh -t ... sudo ...` を**ユーザーに**打ってもらう
  （sudo のパスワードが要る。Claude が代わりに入力しようとしない）
- `⚠️ G1 側を止め切れなかった` → **リモコンでダンピング（L2+B）にするよう最優先で伝える**
  （脱力して倒れるので支えてから）。ssh 不通なら `G1_SSH=unitree@192.168.123.164` を付けて再実行。
  それでも入れなければ、操作PCの `g1_heartbeat_sender` を止めても約 2 秒後に止まる（D-31）
- `⚠️ /cmd_vel の Publisher がまだ N 個いる` → 別の PC で Nav2 が動いている。表示されたノード名を伝える

## 3. やってはいけないこと

- `loco_driver.py` を `pkill`（SIGTERM）や `kill -9` で**だけ**止めない。
  `StopMove()` が走らず、`continous_move=True` の最後の速度指令が G1 の中に 10 日間残る
- 失敗したからといって `Damp()` を SDK から勝手に送らない。脱力で G1 が倒れる。
  ダンピングは人が支えた状態でリモコンから行ってもらう
- 止めた直後に `/g1/clear_estop` や発進ゲートの開放をしない。再開は人の判断で行う

詳しい手順と原因は `Navigation/real/STOP.md`。
