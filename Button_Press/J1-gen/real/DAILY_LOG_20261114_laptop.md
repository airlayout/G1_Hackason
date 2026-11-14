# 2026-11-14 実機日の記録（ノート PC）

> 試すことの一覧は [ToDo_20261114.md](../docs/reports/ToDo_20261114.md)。REPORT は最後にまとめる。
> 時刻は PC2 の時計（ノート PC の時計はこれより 1 時間進んでいる。11/13 と同じ）。

## まとめ

| 項目 | 結果 |
|---|---|
| つなぎ方 | ノート PC（WSL、`eth2` 192.168.0.84）→ 無線の ssh で PC2（`wlan0` 192.168.0.82）。PC2 の有線は `eth0`。ノート PC から github に届かず `git fetch` はできなかった → 手元の `43009c7` を `onboard.sh sync` と `actions/` の rsync で送った |
| 接続 | lowstate 1042 Hz、`mode_machine` 5、腰・腕の mode すべて 1。腰 yaw / roll / pitch = −0.08 / +0.12 / −1.90° |
| 頭カメラ | `videohub_check.py` 5/5 枚（1920x1080、40.8 枚/秒）。**11/13 のような空の画像は無く、再起動は要らなかった**（ロボットの起動から 16 分後） |
| **敬礼 G と F（7）** | **3 回とも最後まで動いて戻った**（G・F・G。ユーザーが確認モードで打った）。**既定は G（`salute`）のまま**にすると決めた（コードの変更なし） |

## 時刻順の記録（PC2 の時計）

| 時刻 | できごと |
|---|---|
| 14:30 ごろ | ロボットの起動（PC2 の uptime から） |
| 14:46 | ssh で PC2 に入れた。`onboard.sh sync`・`actions/` を送った。`check_connection.py`: すべて OK |
| 14:47 | `videohub_check.py` OK → `start_rgbd_server.sh --source videohub --max-fps 5`（5.0 fps。ログ `~/button_press/rgbd_server_20261114_1447.log`） |
| 14:47 | `record.py --label day_watch_salute --mode interval --interval-s 1 --network-interface eth0 --camera-config camera_onboard.yaml` を流し始めた（`_local/button_press/recordings/20261114_144748_day_watch_salute/`） |
| 14:48 ごろ | `salute`・`salute_wrist_bent` の dry-run（どちらも計画どおり） |
| 14:52〜14:56 | **ユーザーが敬礼を 3 回送った**（PC2 で `script -f -a ~/button_press/onboard_logs/user_20261114.log` の中で） |

## 敬礼・G と F（ToDo の 7）

右腕、`--path arm_sdk --network-interface eth0 --execute`、確認モード（`--no-confirm` なし）。腰の上限 8°・腰の kp 120 / kd 2.5（既定）。

| 時刻 | 動作 | 最大の角度・時間 | 追従誤差の最大（上げる / 保つ / 戻る、許容 2.9°） | 腰のずれ（記録の範囲） |
|---|---|---|---|---|
| 14:52 | `salute`（G） | 110.0°・11.5 秒 | 2.32 / 2.28 / 0.75° | yaw −0.2〜0.0°、roll 0.1〜0.4°、pitch −1.0〜+0.4° |
| 14:53 | `salute_wrist_bent`（F） | 105.2°・11.0 秒 | 2.70 / 2.66 / 0.60° | yaw −0.3〜0.0°、roll 0.3〜0.4°、pitch −0.7〜+0.8° |
| 14:55 | `salute`（G） | 110.2°・11.5 秒 | 2.32 / 1.31 / 0.71° | yaw −0.3〜0.0°、roll −0.1〜+0.6°、pitch −2.0〜+0.4° |

- 3 回とも「完了」。腰の上限（8°）に近づいた回は無い（腰のずれは最大 2.4°）
- **追従誤差は 11/13 の簡易版（4〜5°）より小さく、すべて許容内**。F は G より少し大きい（2.7° と 2.3°）
- 腰のずれは `lowstate.csv` の q12〜q14 を、右肩 roll（q23）が開始から 0.2 rad 以上動いている間で取った（確認の Enter 待ちを含む）
- **ユーザーの判断: G（手首まっすぐ）を既定にする**。G はもともと `salute` の既定なので、コードは変えていない
- `salute_elbow_high`（E）は送っていない
