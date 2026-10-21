# 到着位置の計測レポート

**合成データによる解析検証。実機・歩行・押下の性能を示さない。**

予定試行: 30、記録状況: {'recorded': 29, 'not_recorded': 1}。
対象: `panel_1` / `up`。
navigation結果: {'succeeded': 28, 'aborted': 1, 'unmeasured': 1}。自動到達・停止確認・相対姿勢計測あり: 24。

## 基準と計測品質

基準立位の相対姿勢: 取得済み。最適な押下位置という意味ではない。
時計: `{'id': 'unix', 'synchronized': True, 'max_offset_s': 0.001}`。校正ID: `SYNTHETIC_NOT_REAL`。
パネル原点はボタン中心の視線とパネル面の交点。ボタンの突出量・押込量は測っていない。
パネル+xはロボット側への法線、+zは重力に基づく上向き、+yはz×x。yawは壁を向いた姿勢からの差。
平面残差・校正の不確かさはresults.jsonに保持する。カメラ推定を真値と扱わず、navとの差は不一致量として扱う。
T_base_pelvisを固定値で与えた場合は立位近似。その誤差と身体の揺れがnavとの差に含まれる。

## 偏り・ばらつき

自動到達し、停止後の計測がある試行のみ。欠測・手動介入は下の全試行表に残す。位置は母標準偏差、向きは円平均・円標準偏差。
角度はrad、距離はm。heading差は[-π,π)で集計する。95百分位は初期標本の要約であり運用保証ではない。

| 項目 | n | 平均 | 標準偏差 | 絶対値p95 | 絶対値最大 |
|---|---:|---:|---:|---:|---:|
| delta_normal_m | 24 | -0.00101 | 0.04641 | 0.07037 | 0.07469 |
| delta_lateral_m | 24 | 0.01411 | 0.04394 | 0.07677 | 0.07897 |
| delta_height_m | 24 | 0.00003 | 0.00001 | 0.00004 | 0.00005 |
| delta_heading_rad | 24 | -0.01276 | 0.09331 | 0.14558 | 0.15276 |
| stop_delay_s | 24 | 0.90000 | 0.00000 | 0.90000 | 0.90000 |
| nav_local_discrepancy_m | 24 | 0.01119 | 0.00001 | 0.01120 | 0.01121 |

## 全試行

| 試行 | 記録 | 到達結果 | 手動介入 | 前後差m | 左右差m | 向き差rad | 停止確認s | 品質・欠測理由 |
|---|---|---|---|---:|---:|---:|---:|---|
| route_1_01 | recorded | succeeded | False | 0.02087 | 0.01623 | 0.08964 | 0.90000 | — |
| route_2_01 | recorded | succeeded | False | 0.07469 | 0.05890 | -0.12378 | 0.90000 | — |
| route_3_01 | recorded | succeeded | False | -0.06968 | -0.03319 | -0.05700 | 0.90000 | — |
| route_1_02 | recorded | succeeded | False | -0.03438 | 0.07141 | 0.10123 | 0.90000 | — |
| route_2_02 | recorded | succeeded | False | -0.00226 | 0.03608 | -0.12427 | 0.90000 | — |
| route_3_02 | recorded | succeeded | False | -0.04420 | 0.01772 | -0.02848 | 0.90000 | — |
| route_1_03 | recorded | succeeded | False | -0.01387 | 0.07240 | -0.00141 | 0.90000 | — |
| route_2_03 | recorded | succeeded | False | 0.02021 | 0.05543 | 0.05848 | 0.90000 | — |
| route_3_03 | recorded | succeeded | False | -0.03577 | -0.03703 | -0.08076 | 0.90000 | — |
| route_1_04 | recorded | succeeded | False | -0.00835 | -0.03773 | 0.06752 | 0.90000 | — |
| route_2_04 | recorded | succeeded | False | 0.06024 | 0.07897 | 0.04106 | 0.90000 | — |
| route_3_04 | recorded | succeeded | False | 0.05797 | -0.07755 | -0.07507 | 0.90000 | — |
| route_1_05 | recorded | succeeded | False | 0.02809 | 0.03118 | -0.08435 | 0.90000 | — |
| route_2_05 | recorded | succeeded | False | -0.07049 | -0.01960 | 0.14114 | 0.90000 | — |
| route_3_05 | recorded | succeeded | False | 0.01665 | -0.00831 | -0.05816 | 0.90000 | — |
| route_1_06 | recorded | succeeded | False | -0.05239 | 0.03855 | 0.10298 | 0.90000 | — |
| route_2_06 | recorded | succeeded | False | -0.05483 | 0.03171 | 0.06605 | 0.90000 | — |
| route_3_06 | recorded | succeeded | False | 0.00611 | -0.02096 | -0.06159 | 0.90000 | — |
| route_1_07 | recorded | succeeded | False | 0.05276 | 0.04977 | -0.11975 | 0.90000 | — |
| route_2_07 | recorded | succeeded | False | 0.03561 | 0.04843 | -0.07664 | 0.90000 | — |
| route_3_07 | recorded | succeeded | False | 0.05157 | -0.02275 | -0.15276 | 0.90000 | — |
| route_1_08 | recorded | succeeded | False | 0.05552 | 0.02844 | 0.10049 | 0.90000 | — |
| route_2_08 | recorded | succeeded | False | -0.06814 | 0.02978 | -0.14636 | 0.90000 | — |
| route_3_08 | recorded | succeeded | False | -0.05004 | -0.06934 | 0.11599 | 0.90000 | — |
| route_1_09 | recorded | succeeded | False | 未測定 | 未測定 | 未測定 | 未測定 | body clock mismatch/missing; body stream missing; relative pose is unmeasured; settling not confirmed in recorded window |
| route_2_09 | recorded | aborted | False | 0.04780 | -0.06863 | -0.00586 | 未測定 | — |
| route_3_09 | recorded | succeeded | True | 0.00971 | 0.01477 | 0.05105 | 0.90000 | intervention: synthetic intervention; manual intervention: excluded from automatic-arrival statistics |
| route_1_10 | recorded | succeeded | False | 未測定 | 未測定 | 未測定 | 0.90000 | insufficient panel depth; relative pose is unmeasured |
| route_2_10 | recorded | succeeded | False | 未測定 | 未測定 | 未測定 | 0.90000 | camera exposure clock is unverified; relative pose is unmeasured |
| route_3_10 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |

## 改善候補の整理

- 押下側: 停止後のパネル相対位置・向きの分布を、認識範囲や補正範囲の要求候補として渡す。腰・腕の方式や許容範囲は未決定。
- navigation側: 未到達、停止の遅れ、接近経路による偏り、navと局所観測の不一致を調べる。計測誤差を確認してから原因を分ける。
- この記録だけでは押下可能性を判定しない。実機30試行は初期評価であり、成功率の運用保証ではない。
