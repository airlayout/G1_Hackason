# Mapping / sim

MuJoCo 上の G1 で LiDAR 計測を再現し、SLAM のロジックを検証する。

| ファイル | 役割 |
|---|---|
| `scene.py` | マッピング検証用の部屋シーン（壁・柱・箱）を生成し、MuJoCo に読み込ませる |
| `mujoco_lidar.py` | `mj_multiRay` によるレイキャストで 3D LiDAR を模擬する |
| `record_scans.py` | **[エントリポイント]** G1 を部屋の中で歩かせ、スキャンを `scans.npz` に収録する |

地図化そのものは `Mapping/run_slam.py`（sim / real 共通）が行う。
実行方法と実測結果は [Mapping/README.md](../README.md) を参照。

## ここで解いた 3 つの問題

### 1. MuJoCo に LiDAR センサーが無い

`lerobot/unitree-g1-mujoco` の G1 モデルに定義されているセンサーは、関節の
位置・速度・トルク、IMU、足裏の力センサー、そして `head_camera` だけで、
`rangefinder` の類は 1 つも無い。そこで `mujoco.mj_multiRay`（1 点から多数の
レイを飛ばして最初に当たった geom までの距離を返す C 実装）で自前で作った。
11,520 本（32ch × 360 方位）で 1 フレームおよそ 50ms。

### 2. 既定のシーンには地図にするものが何も無い

既定シーン（`assets/scene_43dof.xml`）は無限平面の床と G1 だけ。LiDAR を
回しても床しか返らず、地図として形が残らないうえに、平面 1 枚は水平方向と
ヨー方向を全く拘束しないので ICP がそもそも解けない。壁・柱・箱のある
10m × 8m の部屋を `scene.py` で生成している。

読み込ませ方は「プロセス内で `mujoco.MjModel.from_xml_path` を差し替える」方式。
`UnitreeG1.configure()` はシーンを指定する経路を持たないため何らかの介入が要るが、
`SimpleWalk/sim/patch_mujoco_elastic_band.py` のように HF キャッシュを直接
書き換えると `SimpleWalk/` の実行にも影響してしまう。こちらはプロセス内で完結する。

### 3. LiDAR を頭と同じ高さに付けると点が 1 つも返らない

`torso_link` の頭部 geom はワールド z ≈ 1.12〜1.33m を占めている。同じ
`torso_link` にある `head_camera`（z ≈ 1.27m）と同じ高さに LiDAR を置くと、
センサーが頭の内側に入り、全レイが自分の頭に当たって捨てられる。
既定の取り付け位置は頭の上（z ≈ 1.40m）にしてある。
