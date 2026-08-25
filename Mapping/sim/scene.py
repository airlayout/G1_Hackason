"""マッピング検証用の屋内シーンを組み立て、MuJoCo に読み込ませる。

## なぜ独自シーンが要るのか

`lerobot/unitree-g1-mujoco` の既定シーン（`assets/scene_43dof.xml`）は
無限平面の床と G1 だけで、壁も障害物も無い。この上で LiDAR を回しても
床しか返らず、
  - 地図として意味のある形が何も残らない
  - 平面 1 枚は水平方向・ヨー方向を全く拘束しないので ICP が解けない
という二重の理由で SLAM の検証にならない。壁と柱のある部屋が必須。

## なぜこういう組み込み方をしているのか

`UnitreeG1.configure()` は `make_env("lerobot/unitree-g1-mujoco", trust_remote_code=True)`
を引数無しで呼ぶだけで、シーンを差し替える経路が無い。読み込むシーンは
HF キャッシュ内の `config.yaml` の `ROBOT_SCENE` で決まる。

そこで、キャッシュを書き換えるのではなく **プロセス内で
`mujoco.MjModel.from_xml_path` を差し替える**方式にした。
`SimpleWalk/sim/patch_mujoco_elastic_band.py` はキャッシュの blob を直接
書き換えているが、あれは共有キャッシュを汚す（`SimpleWalk/` の実行にも影響し、
別機能から見ると原因不明の挙動変化になる）。こちらはプロセス内で完結し、
`restore()` で元に戻せる。

なお生成したシーン XML は「作業ディレクトリ + `meshes` へのシンボリックリンク」
の形で置く必要がある。G1 の XML が `<compiler meshdir="meshes"/>` を宣言しており、
MuJoCo はこれを **include 元ではなくメインファイルのあるディレクトリ**から
解決するため、リポジトリ内に置いた XML から絶対パスで include するだけでは
メッシュが見つからず失敗する（実際にこれで一度失敗した）。
"""
from __future__ import annotations

from pathlib import Path

import mujoco

# HF Hub キャッシュ上の MuJoCo 環境。`SimpleWalk/` と同じものを使う。
_HF_REPO_DIR = Path.home() / ".cache" / "huggingface" / "hub" / "models--lerobot--unitree-g1-mujoco"

# 既定シーンが使う 43 自由度（29 関節 + 両手 14）のモデル。ここを 29dof 版に変えると
# アクチュエータ数が bridge 側の期待とずれて壊れるので変えないこと。
_ROBOT_XML_NAME = "g1_29dof_with_hand.xml"

# 部屋の寸法[m]。G1 は原点付近にスポーンするので、原点を中心にした矩形にしている。
ROOM_HALF_X = 5.0
ROOM_HALF_Y = 4.0
WALL_HEIGHT = 2.5
WALL_THICKNESS = 0.05


def find_assets_dir() -> Path:
    """HF キャッシュ内の G1 モデル（`assets/`）を探す。"""
    candidates = sorted(_HF_REPO_DIR.glob(f"snapshots/*/assets/{_ROBOT_XML_NAME}"))
    if not candidates:
        raise FileNotFoundError(
            f"{_ROBOT_XML_NAME} が HF キャッシュに見つからない。まだ一度も MuJoCo 環境を"
            " 起動していない可能性がある。先に SimpleWalk/sim/release_band_and_walk_forward.py"
            " などを一度実行してダウンロードさせること。"
        )
    return candidates[-1].parent


def _obstacle_xml() -> str:
    """部屋の壁と、室内の障害物を MJCF の geom として書き出す。

    障害物は「ICP が解ける幾何」を作るために置いている。壁だけの直方体の部屋は
    向かい合う面が平行なので、コーナーが視野に入らない区間で位置が滑る。
    柱や箱のように向きの違う面を散らしておくと拘束が効く。

    配置は全て壁際（|x| >= 3.6 または |y| >= 2.9）に寄せてある。歩行ポリシーには
    障害物回避が入っていないため、巡回経路上に物を置くとぶつかってその場で
    足踏みしたまま動かなくなる（実際に室内に置いた箱に突っ込んで止まった）。
    """
    walls = [
        # name, pos, size(半寸法)
        ("wall_px", (ROOM_HALF_X, 0.0, WALL_HEIGHT / 2), (WALL_THICKNESS, ROOM_HALF_Y, WALL_HEIGHT / 2)),
        ("wall_nx", (-ROOM_HALF_X, 0.0, WALL_HEIGHT / 2), (WALL_THICKNESS, ROOM_HALF_Y, WALL_HEIGHT / 2)),
        ("wall_py", (0.0, ROOM_HALF_Y, WALL_HEIGHT / 2), (ROOM_HALF_X, WALL_THICKNESS, WALL_HEIGHT / 2)),
        ("wall_ny", (0.0, -ROOM_HALF_Y, WALL_HEIGHT / 2), (ROOM_HALF_X, WALL_THICKNESS, WALL_HEIGHT / 2)),
    ]
    boxes = [
        ("box_shelf", (-4.3, 2.5, 0.9), (0.35, 1.0, 0.9)),
        ("box_crate", (4.2, 2.6, 0.35), (0.5, 0.6, 0.35)),
        ("box_bench", (4.2, -2.6, 0.25), (0.5, 0.9, 0.25)),
        ("box_low", (-4.2, -2.6, 0.2), (0.5, 0.5, 0.2)),
    ]
    pillars = [
        ("pillar_a", (0.0, 3.2, 1.1), (0.25, 1.1)),
        ("pillar_b", (0.0, -3.2, 1.1), (0.25, 1.1)),
    ]

    parts: list[str] = []
    for name, pos, size in walls + boxes:
        parts.append(
            f'    <geom name="{name}" type="box" pos="{pos[0]} {pos[1]} {pos[2]}"'
            f' size="{size[0]} {size[1]} {size[2]}" rgba="0.72 0.72 0.76 1"/>'
        )
    for name, pos, (radius, half_h) in pillars:
        parts.append(
            f'    <geom name="{name}" type="cylinder" pos="{pos[0]} {pos[1]} {pos[2]}"'
            f' size="{radius} {half_h}" rgba="0.62 0.58 0.52 1"/>'
        )
    return "\n".join(parts)


def build_scene(work_dir: Path) -> Path:
    """マッピング用シーン XML を `work_dir` に生成し、そのパスを返す。

    `work_dir` には G1 のメッシュディレクトリへのシンボリックリンクも張る
    （モジュール冒頭の meshdir の説明を参照）。
    """
    assets_dir = find_assets_dir()
    work_dir.mkdir(parents=True, exist_ok=True)

    mesh_link = work_dir / "meshes"
    if mesh_link.is_symlink() or mesh_link.exists():
        mesh_link.unlink()
    mesh_link.symlink_to(assets_dir / "meshes")

    scene_xml = f"""<mujoco model="g1_mapping_room">
  <include file="{assets_dir / _ROBOT_XML_NAME}"/>

  <statistic center="0 0 0.8" extent="6.0"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.4 0.4 0.4" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 2.4" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <site name="com_marker" pos="0.1 0 0" size="0.05" rgba="1 0 0 1" type="sphere"/>
{_obstacle_xml()}
    <camera name="global_view" pos="0.0 -9.5 6.5" xyaxes="1 0 0 0 0.6 0.8" fovy="50"/>
  </worldbody>

  <default>
    <geom friction="1.0"/>
  </default>
</mujoco>
"""
    scene_path = work_dir / "mapping_room_scene.xml"
    scene_path.write_text(scene_xml)
    return scene_path


class SceneOverride:
    """`mujoco.MjModel.from_xml_path` を差し替えて、既定シーンの代わりに部屋シーンを読ませる。

    `robot.connect()` より前に `install()` すること。差し替えは
    「既定シーン（`scene_*.xml`）を読もうとしたとき」だけに限定してあるので、
    ロボット XML 単体を読むような他の呼び出しには影響しない。
    """

    def __init__(self, scene_path: Path) -> None:
        self.scene_path = scene_path
        self._original = mujoco.MjModel.from_xml_path
        self.hit_count = 0

    def install(self) -> None:
        original = self._original
        scene_path = str(self.scene_path)

        def patched(filename, assets=None):  # type: ignore[no-untyped-def]
            if Path(filename).name.startswith("scene_"):
                self.hit_count += 1
                print(f"[scene] 既定シーンを差し替え: {Path(filename).name} -> {scene_path}", flush=True)
                return original(scene_path)
            return original(filename, assets) if assets is not None else original(filename)

        mujoco.MjModel.from_xml_path = patched

    def restore(self) -> None:
        mujoco.MjModel.from_xml_path = self._original
