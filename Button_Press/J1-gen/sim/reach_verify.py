"""ボトルを押せる範囲を広げる設定（configs/reach_wide.yaml）を、MuJoCo（ボトル・机あり、物理あり）で確かめる。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/reach_verify.py                # 既定の配置の一覧を、今の設定と広げた設定で比べる
    $P Button_Press/J1-gen/sim/reach_verify.py --cases 0.26,-0.15,0.02 --view

配置は 机の縁 x（y = 0 での）、ボトルの左右 y、縁からボトルの手前の面まで gap [m]。机は縁が体の正面に平行、天板の高さは pelvis から
z = 0.0（床から約 0.73 m）。ボトルの押す点は、手前の面の天板から 10 cm 上（real 10/21 に合わせる）を、manual で与える。
開始の腕は 10/21 の実機の立った姿勢（下ろした手が股関節の当たり判定に入る）。
配置ごとに、計画が立つか、押し込みで指先がどこまで進んだか（press_result）、ボトルの動き、腕と机・体・ボトルの接触（最大の力）を出す。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.pipeline_cli import add_common_args  # noqa: E402
from common.sim_scene import BOTTLE, TABLE, bottle_position, bottle_tilt_deg  # noqa: E402
from press_bottle_sim import run_sim  # noqa: E402
from reach_map import REAL_STAND_1021  # noqa: E402

# (縁 x, 左右 y, gap)。体の正面寄り・縁が近い・遠い、今の設定でも通るもの
DEFAULT_CASES = [(0.34, -0.15, 0.02), (0.30, -0.15, 0.02), (0.26, -0.15, 0.02), (0.22, -0.15, 0.02),
                 (0.30, -0.10, 0.02), (0.30, -0.05, 0.02), (0.32, 0.00, 0.02), (0.26, -0.05, 0.02),
                 (0.38, -0.15, 0.03), (0.30, -0.20, 0.02)]
TOP_Z = 0.0
PRESS_H = 0.10
RAD2DEG = 180.0 / np.pi


def scene_for(edge: float, y: float, gap: float) -> dict[str, Any]:
    q = REAL_STAND_1021
    pose = {}
    for base, arm in ((15, q[15:22]), (22, q[22:29])):   # 左腕 15〜21、右腕 22〜28
        for k, v in enumerate(arm):
            pose[base + k] = float(v * RAD2DEG)
    return {
        "detection_pose_deg": pose,
        "table": {"top_center": [edge + 0.20, -0.15, TOP_Z - 0.02], "half_size": [0.20, 0.30, 0.02]},
        "bottle": {"xy": [edge + gap + 0.033, y]},
    }


def make_args(overlay: str | None, point: list[float], label: str, obstacles_file: str | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser()
    add_common_args(p)
    argv = ["--target", "manual", "--point", *map(str, point), "--label", label]
    if obstacles_file:
        argv += ["--obstacles-file", obstacles_file]
    a = p.parse_args(argv)
    a.overlay = overlay
    a.sim_detector = "segmentation"
    a.confirm = False
    a.view = False
    return a


def contact_recorder(stats: dict[str, Any]):
    """tick のたびに、右腕と机・ボトル・体（右腕以外のロボット）の接触の最大の力 [N] を記録する。"""
    import mujoco

    def hook(backend: Any) -> None:
        m, d = backend.model, backend.data
        orig = backend.tick
        f6 = np.zeros(6)

        def cat(body: int) -> str:
            name = m.body(body).name
            if name == TABLE:
                return "desk"
            if name == BOTTLE:
                return "bottle"
            if name.startswith("right_") and any(k in name for k in ("shoulder", "elbow", "wrist")):
                return "arm"
            if name in ("world",):
                return "world"
            return "body"

        def tick(dt: float) -> None:
            orig(dt)
            for i in range(d.ncon):
                c = d.contact[i]
                a, b = cat(int(m.geom_bodyid[c.geom1])), cat(int(m.geom_bodyid[c.geom2]))
                if "world" in (a, b) or "arm" not in (a, b):
                    continue
                other = b if a == "arm" else a
                if other == "arm":
                    continue
                mujoco.mj_contactForce(m, d, i, f6)
                key = f"arm-{other}"
                stats[key] = max(stats.get(key, 0.0), float(abs(f6[0])))

        backend.tick = tick

    return hook


def run_case(edge: float, y: float, gap: float, overlay: str | None, label: str, deep_desk: bool = True) -> dict[str, Any]:
    """deep_desk: 計画の障害物を、実機と同じ作りの深い箱（reach_map.desk_boxes）にする。MuJoCo の机は薄い天板のまま（物理）。
    False なら、シーンの薄い天板だけ（天板の下をくぐる経路が通ってしまう）。"""
    point = [edge + gap, y, TOP_Z + PRESS_H]
    obs_file = None
    if deep_desk:
        import tempfile

        from common.realday import replace_obstacle_group
        from reach_map import desk_boxes

        obs_file = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False).name
        replace_obstacle_group("table", desk_boxes(edge, TOP_Z, 0.0), "", Path(obs_file))
    stats: dict[str, Any] = {}
    start: dict[str, Any] = {}
    rec = contact_recorder(stats)

    def hook(backend: Any) -> None:
        start["pos"] = bottle_position(backend.model, backend.data)
        rec(backend)

    args = make_args(overlay, point, label, obs_file)
    res, backend, logger = run_sim(args, scene_name="sim_scene.yaml", scene_overrides=scene_for(edge, y, gap), on_backend=hook)
    pos = bottle_position(backend.model, backend.data)
    out: dict[str, Any] = {"edge": edge, "y": y, "gap": gap, "overlay": overlay or "-", "code": res.code,
                           "message": res.message[:90], "bottle_moved_cm": round(float(np.linalg.norm((pos - start["pos"])[:2])) * 100, 1),
                           "bottle_tilt_deg": round(bottle_tilt_deg(backend.model, backend.data), 1),
                           "force_N": {k: round(v, 1) for k, v in sorted(stats.items())}, "plan": "", "tip_advance_mm": None}
    ev = Path(logger.dir) / "events.jsonl"
    if ev.exists():
        for line in ev.read_text().splitlines():
            e = json.loads(line)
            if e.get("event") == "plan":
                out["plan"] = e.get("seed", "")
            if e.get("event") == "press_result" and e.get("tip_advance_m") is not None:
                out["tip_advance_mm"] = round(e["tip_advance_m"] * 1000, 1)
                out["planned_mm"] = round(e["planned_m"] * 1000, 1)
                out["stopped"] = bool(e.get("stopped"))
    backend.close()
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cases", nargs="+", help="edge,y,gap（例: 0.26,-0.15,0.02）。既定は一覧")
    ap.add_argument("--overlays", nargs="+", default=["-", "reach_wide.yaml"], help="比べる設定（- は今の設定）")
    ap.add_argument("--thin-desk", action="store_true", help="計画の障害物を、シーンの薄い天板だけにする（既定は実機と同じ深い箱）")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    cases = [tuple(float(v) for v in c.split(",")) for c in args.cases] if args.cases else DEFAULT_CASES
    rows = []
    for edge, y, gap in cases:
        for ov in args.overlays:
            r = run_case(edge, y, gap, None if ov == "-" else ov, f"verify_{edge}_{y}_{ov.split('.')[0]}",
                         deep_desk=not args.thin_desk)
            rows.append(r)
            print(f"[verify] 縁 {edge:.2f} y {y:+.2f} gap {gap * 100:.0f}cm {r['overlay']:16s} → 終了 {r['code']} "
                  f"{r['message'][:40]:40s} 指先 {r['tip_advance_mm']} / 計画 {r.get('planned_mm')} mm、ボトル {r['bottle_moved_cm']} cm・"
                  f"{r['bottle_tilt_deg']}°、力 {r['force_N']} {r['plan']}")
    if args.out:
        args.out.write_text(json.dumps(rows, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
