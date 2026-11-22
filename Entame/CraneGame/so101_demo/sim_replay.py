"""追従の録画ログを、MuJoCo の SO-101 公式モデルで再生し、Rerun の .rrd に書き出す。

左: 録画したカメラ映像。右: 3D のロボット。実測の姿勢（不透明）と、手が求めた目標の姿勢（半透明）を重ねる。
使い方: .venv/bin/python sim_replay.py 20261122-221158-433 [--frames <フレームのフォルダ>] [--open]

関節値（-100〜100）→角度は、lerobot の較正（range_min〜range_max を -100〜100 に対応させ、中央を 0）に従う。
モデルは SO-ARM100 リポジトリの so101_new_calib（各関節の 0 は可動域の中央）で、較正の考え方が一致する。
グリッパーだけは仮定: 0〜100 をモデルの可動域（-10°〜100°）へ線形に対応させる。
"""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

HERE = Path(__file__).parent
MODEL_XML = HERE / "sim_assets" / "SO101" / "so101_new_calib.xml"
CALIBRATION = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/so101.json"
DEBUG = HERE / "debug"
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TICKS_PER_TURN = 4096
FRAME_FPS = 10                      # 録画から切り出すフレームの毎秒枚数（extract の fps と合わせる）
CUR_COLOR = None                    # None = モデル本来の色
WANT_RGBA = (1.0, 0.55, 0.1, 0.35)  # 目標の姿勢（半透明のオレンジ）
PRED_RGBA = (0.1, 0.8, 0.3, 0.45)   # 今のコードを録画の手の入力で動かした予測（半透明の緑）
TRAIL_PRED_RGB = (30, 190, 80)
TRAIL_CUR_RGB = (40, 120, 220)
TRAIL_WANT_RGB = (255, 140, 30)


def load_calibration() -> dict:
    return json.loads(CALIBRATION.read_text())


def joint_radians(cal: dict, joint: str, norm: float) -> float:
    """lerobot の正規化値（-100〜100）をモデルの関節角[rad]へ。中央が 0。"""
    c = cal[joint]
    half_ticks = (c["range_max"] - c["range_min"]) / 2
    return norm / 100 * half_ticks * 2 * math.pi / TICKS_PER_TURN


def apply_pose(model: mujoco.MjModel, data: mujoco.MjData, cal: dict, pose: dict) -> None:
    """pose（関節 → 正規化値）をモデルの qpos に入れて順運動学を計算する。"""
    for joint in ARM_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        data.qpos[model.jnt_qposadr[jid]] = joint_radians(cal, joint, pose[joint])
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper")
    low, high = model.jnt_range[gid]
    data.qpos[model.jnt_qposadr[gid]] = low + pose.get("gripper", 0.0) / 100 * (high - low)
    mujoco.mj_forward(model, data)


def mesh_geoms(model: mujoco.MjModel) -> list:
    """見た目に使うメッシュの geom（衝突用の非表示グループは除く）。"""
    return [g for g in range(model.ngeom)
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and model.geom_group[g] in (0, 1, 2)]


def geom_rgba(model: mujoco.MjModel, gid: int) -> tuple:
    mat = model.geom_matid[gid]
    rgba = model.mat_rgba[mat] if mat >= 0 else model.geom_rgba[gid]
    return tuple(float(x) for x in rgba)


def log_meshes(model: mujoco.MjModel, root: str, rgba_override: tuple | None) -> None:
    """各 geom のメッシュを、形そのままで一度だけ書く（動きは Transform3D で与える）。"""
    for gid in mesh_geoms(model):
        mid = model.geom_dataid[gid]
        v0, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        f0, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        verts = model.mesh_vert[v0:v0 + vn].copy()
        faces = model.mesh_face[f0:f0 + fn].copy()
        rgba = rgba_override or geom_rgba(model, gid)
        rr.log(f"{root}/g{gid}", rr.Mesh3D(vertex_positions=verts, triangle_indices=faces, albedo_factor=rgba), static=True)


def log_transforms(model: mujoco.MjModel, data: mujoco.MjData, root: str) -> None:
    for gid in mesh_geoms(model):
        rr.log(f"{root}/g{gid}", rr.Transform3D(translation=data.geom_xpos[gid], mat3x3=data.geom_xmat[gid].reshape(3, 3)))


def tip_position(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    return data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")].copy()


def read_run(name: str) -> tuple:
    """録画ログから (時刻[s], 実測の姿勢, 目標の姿勢 or None) の列を返す。目標は肩・肘などログに入っている関節だけ。"""
    rows = [json.loads(line) for line in (DEBUG / f"follow-{name}.jsonl").read_text().splitlines()]
    t0 = rows[0]["ts"]
    samples = []
    for r in rows:
        if r.get("ev") == "sample" and r.get("cur"):
            samples.append((r["ts"] - t0, r["cur"], r.get("want") or None))
    return samples


def read_prediction(name: str) -> dict:
    """今のコード・今の limits で、録画した手の入力を流し込んだときの姿勢（{時刻[s]: 姿勢}）。録画時の limits ではない。"""
    from replay import load, replay  # 遅延 import（replay は Desk 一式を読み込むため）
    start, samples = load(DEBUG / f"follow-{name}.jsonl")
    limits = json.loads((HERE / "limits.json").read_text())
    return {r["ts"] - start["ts"]: r["sim"] for r in replay(start, samples, limits=limits)}


def nearest(table: dict, t: float, tol: float = 0.15) -> dict | None:
    if not table:
        return None
    key = min(table, key=lambda k: abs(k - t))
    return table[key] if abs(key - t) <= tol else None


def blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="camera", name="録画（カメラ）"),
            rrb.Spatial3DView(origin="robot", name="SO-101（MuJoCo 公式モデル）  不透明=録画時の実測 / 半透明オレンジ=手が求めた目標 / 半透明の緑=今のコードの予測"),
        ),
        collapse_panels=True,
    )


def frame_path(frames: Path, t: float) -> Path | None:
    index = max(1, round(t * FRAME_FPS) + 1)
    path = frames / f"{index:04d}.jpg"
    return path if path.exists() else None


def build(name: str, frames: Path | None, out: Path) -> None:
    cal = load_calibration()
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    cur_data, want_data, pred_data = (mujoco.MjData(model) for _ in range(3))
    prediction = read_prediction(name)
    rr.init(f"so101-replay-{name}", recording_id=f"so101-replay-{name}")
    rr.save(str(out))
    rr.send_blueprint(blueprint())
    log_meshes(model, "robot/cur", CUR_COLOR)
    log_meshes(model, "robot/want", WANT_RGBA)
    log_meshes(model, "robot/pred", PRED_RGBA)
    rr.log("robot/base_axes", rr.Arrows3D(origins=[[0, 0, 0]] * 3, vectors=[[.08, 0, 0], [0, .08, 0], [0, 0, .08]],
                                           colors=[[255, 60, 60], [60, 200, 60], [60, 60, 255]]), static=True)
    cur_trail, want_trail, pred_trail = [], [], []
    for t, cur, want in read_run(name):
        rr.set_time("sec", duration=t)
        apply_pose(model, cur_data, cal, cur)
        log_transforms(model, cur_data, "robot/cur")
        cur_trail.append(tip_position(model, cur_data))
        pred = nearest(prediction, t)
        if pred:
            apply_pose(model, pred_data, cal, {**cur, **pred})
            log_transforms(model, pred_data, "robot/pred")
            pred_trail.append(tip_position(model, pred_data))
        if want:
            apply_pose(model, want_data, cal, {**cur, **want})
            log_transforms(model, want_data, "robot/want")
            want_trail.append(tip_position(model, want_data))
        else:
            rr.log("robot/want", rr.Clear(recursive=True))
        image = frame_path(frames, t) if frames else None
        if image:
            rr.log("camera", rr.EncodedImage(path=str(image)))
    rr.log("robot/trail_cur", rr.LineStrips3D([np.array(cur_trail)], colors=[TRAIL_CUR_RGB], radii=0.0015), static=True)
    if pred_trail:
        rr.log("robot/trail_pred", rr.LineStrips3D([np.array(pred_trail)], colors=[TRAIL_PRED_RGB], radii=0.0015), static=True)
    if want_trail:
        rr.log("robot/trail_want", rr.LineStrips3D([np.array(want_trail)], colors=[TRAIL_WANT_RGB], radii=0.0015), static=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="例: 20261122-221158-433（debug/follow-<name>.jsonl）")
    ap.add_argument("--frames", type=Path, help="録画から切り出したフレーム（0001.jpg から連番、10fps）のフォルダ")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    out = args.out or DEBUG / f"replay-{args.name}.rrd"
    build(args.name, args.frames, out)
    print(f"[replay] 書き出しました: {out}")


if __name__ == "__main__":
    main()
