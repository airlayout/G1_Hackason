# 到着位置の計測レポート

実機データの記録状況と計測値。未取得・未校正の値は未測定。

予定試行: 30、記録状況: {'not_recorded': 30}。
対象: `panel_1` / `up`。
navigation結果: {'unmeasured': 30}。自動到達・停止確認・相対姿勢計測あり: 0。

## 基準と計測品質

基準立位の相対姿勢: 未測定。最適な押下位置という意味ではない。
時計: `{'id': 'unix', 'synchronized': False, 'max_offset_s': None}`。校正ID: `UNMEASURED`。
パネル原点はボタン中心の視線とパネル面の交点。ボタンの突出量・押込量は測っていない。
パネル+xはロボット側への法線、+zは重力に基づく上向き、+yはz×x。yawは壁を向いた姿勢からの差。
平面残差・校正の不確かさはresults.jsonに保持する。カメラ推定を真値と扱わず、navとの差は不一致量として扱う。
T_base_pelvisを固定値で与えた場合は立位近似。その誤差と身体の揺れがnavとの差に含まれる。

## 偏り・ばらつき

自動到達し、停止後の計測がある試行のみ。欠測・手動介入は下の全試行表に残す。位置は母標準偏差、向きは円平均・円標準偏差。
角度はrad、距離はm。heading差は[-π,π)で集計する。95百分位は初期標本の要約であり運用保証ではない。

| 項目 | n | 平均 | 標準偏差 | 絶対値p95 | 絶対値最大 |
|---|---:|---:|---:|---:|---:|
| delta_normal_m | 0 | 未測定 | 未測定 | 未測定 | 未測定 |
| delta_lateral_m | 0 | 未測定 | 未測定 | 未測定 | 未測定 |
| delta_height_m | 0 | 未測定 | 未測定 | 未測定 | 未測定 |
| delta_heading_rad | 0 | 未測定 | 未測定 | 未測定 | 未測定 |
| stop_delay_s | 0 | 未測定 | 未測定 | 未測定 | 未測定 |
| nav_local_discrepancy_m | 0 | 未測定 | 未測定 | 未測定 | 未測定 |

## 全試行

| 試行 | 記録 | 到達結果 | 手動介入 | 前後差m | 左右差m | 向き差rad | 停止確認s | 品質・欠測理由 |
|---|---|---|---|---:|---:|---:|---:|---|
| route_1_01 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_01 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_01 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_02 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_02 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_02 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_03 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_03 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_03 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_04 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_04 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_04 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_05 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_05 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_05 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_06 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_06 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_06 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_07 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_07 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_07 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_08 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_08 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_08 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_09 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_09 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_09 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_1_10 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_2_10 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |
| route_3_10 | not_recorded | unmeasured | False | 未測定 | 未測定 | 未測定 | 未測定 | attempt has not been recorded |

## 改善候補の整理

- 押下側: 停止後のパネル相対位置・向きの分布を、認識範囲や補正範囲の要求候補として渡す。腰・腕の方式や許容範囲は未決定。
- navigation側: 未到達、停止の遅れ、接近経路による偏り、navと局所観測の不一致を調べる。計測誤差を確認してから原因を分ける。
- この記録だけでは押下可能性を判定しない。実機30試行は初期評価であり、成功率の運用保証ではない。
