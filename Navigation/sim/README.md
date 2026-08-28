# Navigation / sim

シミュレーション環境上での検証コードをここに置く。

`slam_operate`のモック（1804/1102に応答し、`rt/slam_info`相当の状態を返す）を用意し、
**実機なしでルート分割器・巡回管理・UIを開発できる状態**を作るのが主目的。
実機G1は1台しかなく歩行検証の時間が限られるため、実機を使わずに進められる範囲を
ここで最大化する。

`Mapping/`に`./mapctl doctor --mock` / `./mapctl start --backend mock`という先例がある
（`Mapping/real/python/g1_mapping/mock.py`）。モックでも実機と同じ状態遷移を通す方針を踏襲する。
