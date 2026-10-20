# 外部アセット

`manifest.json`は取得先のcommit・個別ファイルのSHA-256を固定する。
`scripts/fetch_assets.py`で検証して`cache/`へ取得し、原本のURDFや重みは編集しない。
USDへの変換結果も`cache/usd/`へ保存する。

- 身体モデル: [unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description)、`g1_29dof_rev_1_0.urdf`と参照mesh。上流の[LICENSE](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/LICENSE)を参照。
- 検証用ONNXとdeploy設定: [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0)。`--with-demo`指定時だけ取得する。上流READMEのライセンス表記はApache-2.0。

MID-360走査表は[Livox公式シミュレーター](https://github.com/Livox-SDK/livox_laser_simulation/tree/1cce1073633a062b92e30243a4c2920e45551bb5)の scan_mode/mid360.csv を使用する。MIT、著作権表示は licenses/livox_laser_simulation.txt。第1列はTime/sという見出しだが実値は1〜800000の点番号。点頻度200000/sを別に適用する。公開表は4秒分で、個体の取得時刻・ring・反射特性は含まない。既定では表の末尾で繰り返さず、4秒を超える収集を開始前に拒否する。

29関節の身体モデルは全コントローラーで共通。公開ONNXとその入力480次元・履歴5の仕様は、環境の必須条件ではない。
