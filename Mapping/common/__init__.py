"""Mapping 機能の sim / real 共通ロジック。

このパッケージは MuJoCo にも Unitree SDK にも依存しない（numpy / scipy のみ）。
シミュレーションで検証したロジックをそのまま実機に持っていけるようにするため、
センサーの取得方法（`sim/mujoco_lidar.py` / 実機の DDS）とは分離してある。
"""
