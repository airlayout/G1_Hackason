# sim_assets

シミュレーション（`pickplace_sim.py`、`/sim`）が読む、SO-101 のロボットモデル。

| 場所 | 内容 |
|---|---|
| [`SO101/`](SO101/README.md) | SO-101 の URDF と MuJoCo（MJCF）の記述。`so101_new_calib.xml` と `scene.xml` を使う。出典とライセンスは `SO101/README.md` と `SO101/LICENSE`。 |

- `SO101/assets/`（3D メッシュ）は `so101_demo/.gitignore` で Git の対象外。別のマシンでは、このフォルダごと手でコピーする必要がある（`prepare.py` は取得しない。取得手順は未整備）。
- 公式モデルとして置いたもの。シミュレーション側の設定は `pickplace_sim.py`（`MODEL_XML` がここの `so101_new_calib.xml` を指す）。
