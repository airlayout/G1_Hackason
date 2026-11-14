# 2026-11-01 前後3mm・3条件への変更と検証

標準探索を、認識した位置・手前3mm・奥3mmの3条件へ変更した。
3条件すべてを完走して成功した最初の候補を採用する。探索予算は最大10秒。
障害物余裕3mm、追従誤差10mm、関節・速度・加速度、復帰と解除の基準は維持した。
指先と選択対象の接触は全区間で許可し、対象への接触余裕は実形状から求める参考値とした。
追加中のボトル相対移動補正は標準off。現場計測時の接近点と時間配分を使う。

## 保存した現場シーンでの比較

入力は `LOG/robust-lab-20261101/live/diagnostic_20261101_112438/result.json`。
旧プランナーはGit `95f2afc` から読み込み、同じローカルモデル・保存シーンで順に実行した。
画像から再認識する比較ではなく、モデル構築から候補生成・物理試行までの比較。
旧プランナーの依存するシーン構築処理は現在のものと共通にしている。
各方式1回の測定であり、実行環境・起動時間によって所要時間や評価回数は変わる。

| 方式 | 条件数 | プロセス数 | 探索時間 | 候補数（静的検査を含む） | 完走 / 中止 | 採用 |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 変更前 | 20 | 4 | 9.787秒 | 2 | 24 / 16 | なし |
| 変更後 | 3 | 3 | 2.423秒 | 1 | 3 / 0 | 最初の候補 |

校正に記録された位置不確かさ15mmは変更していない。
通常探索で確認した幅は前後3mmであり、15mmや任意の左右・上下ずれへの対応を確認した結果ではない。
任意のholdout評価は、通常探索とは別に従来の全方向・校正不確かさを含む範囲で行う。

## 保存軌道そのものの再評価

`performance/baseline_candidate.json` と `.npz` の軌道を、生成し直さずに再生した。
2・3条件目の質量・摩擦・関節誤差などのseedは変更前と同じ。

| 条件 | 前方移動 | 横移動 | 障害物余裕 | 最大追従誤差 | 判定 |
| --- | ---: | ---: | ---: | ---: | --- |
| 認識どおり | 47.38mm | −2.59mm | 49.86mm | 6.74mm | 合格 |
| 手前3mm | 50.36mm | −3.68mm | 49.28mm | 6.71mm | 合格 |
| 奥3mm | 44.33mm | 0.80mm | 50.00mm | 6.74mm | 合格 |

参考の接触余裕はそれぞれ17.29 / 15.90 / 17.13mmだった。
以前の負の余裕には、位置の不確かさの重複差し引きと、円筒の上下方向の扱いの問題が含まれていた。

## 実行経路とテスト

骨盤固定のシミュレーションで、RGB-D観測→探索→共通のストリーム実行→結果評価を通した。
上ボタン・下ボタン・ボトルをseed 10000 / 10001で評価し、**6/6成功**。
対象の物理配置は各軸±10mm、校正ずれは各軸±1mm、画像ノイズは±0.5mmのseed生成値を使った。
検出枠はoracle bboxで、YOLOの精度や実機の自由立位を検証したものではない。

| ケース | seed 10000の探索 / 全動作 | seed 10001の探索 / 全動作 |
| --- | --- | --- |
| 上ボタン | 2.176秒 / 30.00秒 | 2.115秒 / 30.00秒 |
| 下ボタン | 4.567秒 / 30.00秒 | 2.084秒 / 30.00秒 |
| ボトル | 6.640秒 / 32.66秒 | 2.327秒 / 32.96秒 |

全体テストは479合格・40スキップ。その後の校正読込の修正は関連92テストで合格。
スキップは依存ライブラリ・実機用SDK等の利用条件によるもの。
新しいテストでは、早い対象接触の許可、指先以外・対象以外の衝突拒否、3mm未満の障害物余裕、
押下不足・過押し・40N超過、未完了・期限切れの不採用、初回合格での終了も確認した。
校正読込の緩和はrobustだけに適用し、従来方式の事前判定は維持する。
模擬SDK・カメラの通信テストはローカルソケットを使用する。

**実機へは接続せず、腕も動かしていない。** 実機での押下成功とは区別する。
集計データは [検証JSON](ROBUST_THREE_CONDITIONS_20261101.json)、生ログは
`_local/robust_push/three_conditions_20261101/` に保存した。

## 再実行

```bash
OPENBLAS_NUM_THREADS=1 _local/button_press_yada/venv/bin/python \
  Manipulation/push_button/sim/compare_robust_field.py \
  --output _local/robust_push/three_conditions_recheck/field

MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 _local/button_press_yada/venv/bin/python \
  Manipulation/push_button/sim/run_robust_suite.py --controllers robust \
  --seeds 10000 10001 --out _local/robust_push/three_conditions_recheck/suite

MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 _local/button_press_yada/venv/bin/python \
  -m pytest Manipulation/push_button/tests -q -p no:cacheprovider
```

通常の実機入口で明示する設定は `--planning-scenarios 3 --planning-position-span-m 0.003 --bottle-feedback off`。
古い試行JSONに `planning_scenarios: 20` が残っている場合はCLIで上書きするか、試行JSONを更新する。
