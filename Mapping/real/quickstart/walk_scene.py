#!/usr/bin/env python3
"""実測地図から作った MuJoCo シーンの中で、G1 を端から端まで歩かせる。

## 何を確かめるためのものか

`pcd_to_mjcf.py` が作ったシーンは、**当たり判定として成立しているか**が
静止した検査（箱の数・塞がる空間）だけでは分からない。実際に学習済みの歩行
ポリシーで横断させて、転ばずに着けるかを見る。地図の使い道そのものの試験である。

## 何を流用しているか（自作していない）

歩行も経路計画も `Dev/Navigation` の実装をそのまま使う。**ここで作り直さない。**

- `sim/g1_walker.py` … `unitree_rl_gym` の学習済み 12DoF ポリシー（`motion.pt`）を
  公式 `deploy_mujoco.py` の制御ループのまま回す
- `nav/occupancy.py` … 点群 → 通行判定格子（機体半径 0.40m ぶん膨張）
- `nav/route.py` … A* ＋ string pulling で折れ線に詰める
- `sim/slam_service.py` … 目標追従の制御則と定数（実測で決まった値なので触らない）

このスクリプトが足しているのは **(1) 実測地図の点群から格子を作る (2) 端から端の
2 点を決める (3) 走らせて記録する** の 3 つだけ。

## 「端から端」の決め方

自由セルの中で**最短経路が最も長くなる 2 点**を、幅優先探索の 2 回掃引で選ぶ。
直線距離で選ぶと壁を挟んだ 2 点になり、実際には短い距離しか歩かないことがある。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/walk_scene.py \\
        runs/20260904T183457_UiS_room_v2 --nav-root <Dev/Navigation の Navigation/>
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial import cKDTree

QUICKSTART = Path(__file__).resolve().parent
sys.path.insert(0, str(QUICKSTART))

from eval_removal import read_trajectory  # noqa: E402
from pcd_to_mjcf import level_cloud  # noqa: E402

DEFAULT_NAV_ROOT = QUICKSTART.parents[2] / "Navigation"
OBSTACLE_Z = (0.15, 2.00)   # pcd_to_mjcf.py の障害物帯と揃える
MAX_RANGE = 12.0            # pcd_to_mjcf.py と同じ切り出し
TIMEOUT_S = 400.0           # 24×31m を 0.45m/s で横断しても十分な余裕
SPAWN_HEIGHT = 0.793
FPS = 20                    # 録画のコマ数[/s]
PLAYBACK_SPEED = 4.0        # 録画を実時間の何倍で見せるか
CAMERA_DISTANCE = 4.5       # 追従カメラの距離[m]
CAMERA_ELEVATION = -22.0


def import_nav(nav_root: Path):
    """`Dev/Navigation` の実装を読み込む。"""
    sys.path.insert(0, str(nav_root.parent))
    sys.path.insert(0, str(nav_root))
    from nav.occupancy import build_grid                     # noqa: PLC0415
    from nav.protocol import Pose2D                          # noqa: PLC0415
    from nav.route import plan_route                         # noqa: PLC0415
    from sim import slam_service                             # noqa: PLC0415
    from sim.g1_walker import G1Walker                       # noqa: PLC0415
    return build_grid, Pose2D, plan_route, slam_service, G1Walker


def farthest_pair(free: np.ndarray) -> "tuple[tuple[int, int], tuple[int, int]]":
    """自由セルの中で、最短経路が最も長くなる 2 点を 2 回掃引で求める。

    直線距離で選ぶと壁を挟んだ 2 点を選んでしまい、「端から端」にならない。

    **掃引は島をまたげないので、最初に一番大きい島を選ぶ。** 実測地図の自由空間は
    壁と未知でいくつもの島に割れる（UiS_room_v3 で 130 個）。走査順で最初に見つけた
    自由セルから広げると、地図の隅にある小部屋に当たったときそこから出られず、
    「端から端」がその小部屋の差し渡しになる（同地図で 95m² の本体を素通りして 4.9m）。
    """
    def sweep(seed: "tuple[int, int]") -> "tuple[tuple[int, int], np.ndarray]":
        distance = np.full(free.shape, -1, np.int32)
        distance[seed] = 0
        queue = deque([seed])
        while queue:
            row, col = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                r, c = row + dr, col + dc
                if 0 <= r < free.shape[0] and 0 <= c < free.shape[1] \
                        and free[r, c] and distance[r, c] < 0:
                    distance[r, c] = distance[row, col] + 1
                    queue.append((r, c))
        flat = int(np.argmax(distance))
        return (flat // free.shape[1], flat % free.shape[1]), distance

    islands, count = ndimage.label(free)
    if count == 0:
        raise ValueError("自由セルが 1 つも無い")
    sizes = np.bincount(islands.ravel())
    sizes[0] = 0                                    # 0 は「自由でない」側の札
    rows, cols = np.nonzero(islands == int(sizes.argmax()))
    first, _ = sweep((int(rows[0]), int(cols[0])))
    second, _ = sweep(first)
    return first, second


def build_scene_with_goals(scene: Path, goals: "list[tuple[float, float]]"):
    """シーンに**目標地点の目印**を差してからコンパイルする。

    見せるためだけの目印なので `contype=0 conaffinity=0` で当たり判定から外す。
    MJCF を書き換えず `MjSpec` で差すのは、`_scene_*.xml` が生成物であり
    実行のたびに作り直されるため。
    """
    import mujoco                                    # noqa: PLC0415

    spec = mujoco.MjSpec.from_file(str(scene))
    for index, (x, y) in enumerate(goals):
        last = index == len(goals) - 1
        body = spec.worldbody.add_body()
        body.name = f"goal_{index}"
        body.pos = [x, y, 0.0]
        post = body.add_geom()
        post.name = f"goal_{index}_post"
        post.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        post.size = [0.10, 0.90, 0.0]
        post.pos = [0.0, 0.0, 0.90]
        post.contype, post.conaffinity = 0, 0
        post.rgba = [1.0, 0.30, 0.15, 0.55] if last else [1.0, 0.78, 0.0, 0.50]
        ring = body.add_geom()
        ring.name = f"goal_{index}_ring"
        ring.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        ring.size = [0.35, 0.01, 0.0]
        ring.pos = [0.0, 0.0, 0.02]
        ring.contype, ring.conaffinity = 0, 0
        ring.rgba = [1.0, 0.30, 0.15, 0.75] if last else [1.0, 0.78, 0.0, 0.65]
    return spec.compile()


def check_scene(walker, npz_path: Path) -> dict:
    """**MJCF が本当にその形を持っているか**を、格子から数え直して照合する。

    HTML は npz の格子から箱を組み直して描いている。MJCF とズレていれば
    「絵では歩けているが実際は違う形の中を歩いた」ことになるので、
    シミュレータが読んだモデルの側から数えて突き合わせる。
    """
    import mujoco                                    # noqa: PLC0415
    sys.path.insert(0, str(QUICKSTART))
    from pcd_to_mjcf import boxes_from_grid          # noqa: PLC0415

    model = walker.model
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "" for i in range(model.ngeom)]
    is_map = np.array([n.startswith("map_") for n in names])
    kinds = model.geom_type[is_map]
    sizes = model.geom_size[is_map]
    mjcf_volume = float((8.0 * np.prod(sizes, axis=1)).sum())

    data = np.load(npz_path)
    boxes = boxes_from_grid(data["occupied"], data["level"], float(data["cell"]), data["origin"])
    cell = float(data["cell"])
    html_volume = float(sum(2 * b[3] * cell * 2 * b[4] for b in boxes))

    hfield = [i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_HFIELD]
    return {
        "mjcf_boxes": int(is_map.sum()),
        "html_boxes": len(boxes),
        "all_boxes": bool((kinds == mujoco.mjtGeom.mjGEOM_BOX).all()),
        "mjcf_volume_m3": round(mjcf_volume, 1),
        "html_volume_m3": round(html_volume, 1),
        "hfield_geoms": len(hfield),
        "hfield_rows": int(model.hfield_nrow[0]) if model.nhfield else 0,
        "hfield_cols": int(model.hfield_ncol[0]) if model.nhfield else 0,
        "grid_rows": int(data["occupied"].shape[0]),
        "grid_cols": int(data["occupied"].shape[1]),
    }


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def place_robot(walker, x: float, y: float, yaw: float, floor_z: float) -> None:
    """キーフレームの置き場所を、こちらが決めた出発点へ差し替える。"""
    data = walker.data
    data.qpos[0:3] = [x, y, SPAWN_HEIGHT + floor_z]
    data.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    data.qvel[:] = 0.0
    walker._mujoco.mj_forward(walker.model, data)   # noqa: SLF001


# その場旋回から前進へ戻す閾値[rad]。**入る閾値と出る閾値を分ける（ヒステリシス）。**
#
# 単一の閾値で「ずれ35°超なら前進を切る」とすると、閾値の近くで前進が 0 と全開を
# 往復する。2026-09-05 の実測では、この発停（速度 0.53→0.04→0.20→0.14→0.48 m/s）が
# 1.5 秒続いたあと歩容が崩れて転んだ。地形でも接触でもなく制御側の問題だった。
RESUME_FORWARD_RAD = math.radians(20.0)


class Recorder:
    """MuJoCo の画面をそのまま録る。**HTML の再生とは別物。**

    HTML は骨盤の x,y,yaw だけを線と矢印で描いた抽象で、機体が本当に脚を運んで
    いるかは写らない。実際に歩けているかを見るには、シミュレータが描いた絵が要る。
    """

    def __init__(self, walker, width: int, height: int) -> None:
        import mujoco                                # noqa: PLC0415
        self._mujoco = mujoco
        self._renderer = mujoco.Renderer(walker.model, height=height, width=width)
        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self._camera.trackbodyid = walker.model.body("pelvis").id
        self._camera.distance = CAMERA_DISTANCE
        self._camera.elevation = CAMERA_ELEVATION
        self.frames: list = []

    def capture(self, walker, yaw: float) -> None:
        # 進行方向の後ろから見る。azimuth は度で、world の +x から反時計回り
        self._camera.azimuth = math.degrees(yaw) - 180.0
        self._renderer.update_scene(walker.data, camera=self._camera)
        self.frames.append(self._renderer.render().copy())


def drive(walker, targets, slam_service, timeout: float, recorder=None) -> dict:
    """目標点列を順に追う。制御則と定数は `slam_service` のものをそのまま使う。

    ヒステリシスだけはここで足している（`slam_service` は区間ごとに向きを
    決め直すので、連続追従に使うと閾値の近くで発停する）。
    """
    tick = slam_service.CONTROL_TICK_S
    history, index, start = [], 0, time.time()
    turning = False
    next_frame = 0.0
    while index < len(targets) and walker.sim_time < timeout:
        pose = walker.pose
        pose = pose() if callable(pose) else pose
        history.append((walker.sim_time, pose.x, pose.y, pose.yaw, walker.height))
        target = targets[index]
        dx, dy = target[0] - pose.x, target[1] - pose.y
        distance = math.hypot(dx, dy)
        if distance <= slam_service.ARRIVAL_TOLERANCE_M:
            index += 1
            continue
        error = wrap(math.atan2(dy, dx) - pose.yaw)
        if turning:
            turning = abs(error) > RESUME_FORWARD_RAD
        else:
            turning = abs(error) > slam_service.TURN_IN_PLACE_RAD
        forward = 0.0 if turning else \
            slam_service.LINEAR_GAIN * distance * slam_service._forward_scale(error)
        walker.set_command(forward, 0.0, slam_service.ANGULAR_GAIN * error)
        walker.step(tick)
        if recorder is not None and walker.sim_time >= next_frame:
            recorder.capture(walker, pose.yaw)
            next_frame += 1.0 / FPS * PLAYBACK_SPEED
        if walker.has_fallen:
            break
    pose = walker.pose
    pose = pose() if callable(pose) else pose
    history.append((walker.sim_time, pose.x, pose.y, pose.yaw, walker.height))
    return {"history": np.array(history), "reached": index >= len(targets),
            "fallen": bool(walker.has_fallen), "sim_s": float(walker.sim_time),
            "wall_s": round(time.time() - start, 1)}


def write_video(frames: list, path: Path, width: int, height: int) -> Path:
    """コマを mp4 にする。ffmpeg に生の RGB を流し込むだけ（中間の PNG を作らない）。"""
    import subprocess                                # noqa: PLC0415

    command = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
        "-framerate", str(FPS), "-i", "-",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28",
        "-movflags", "+faststart", str(path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    for frame in frames:
        process.stdin.write(frame.tobytes())
    process.stdin.close()
    if process.wait() != 0:
        raise SystemExit("ffmpeg が失敗した")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="実測地図のシーンで G1 を端から端まで歩かせる")
    parser.add_argument("session", type=Path)
    parser.add_argument("--map", default="map_octomap_r4.pcd")
    parser.add_argument("--scene", default=None, help="既定は変種に対応する _scene_*.xml")
    parser.add_argument("--nav-root", type=Path, default=DEFAULT_NAV_ROOT,
                        help="nav/ と sim/ を含むディレクトリ（Dev/Navigation の Navigation/）")
    parser.add_argument("--scene-dir", type=Path,
                        default=QUICKSTART.parents[2] / "Navigation/sim/assets/g1_description")
    parser.add_argument("--waypoint", action="append", default=[], metavar="X,Y",
                        help="経由・目標地点。複数可。1つ目が出発点。"
                             "省略時は最短経路が最長になる2点を自動で選ぶ")
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S)
    parser.add_argument("--record", action="store_true",
                        help="MuJoCo の画面を mp4 に録る（ffmpeg が要る）")
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    build_grid, Pose2D, plan_route, slam_service, G1Walker = import_nav(args.nav_root.resolve())

    session = args.session.resolve()
    sim_dir = session / "sim"
    name = json.loads((session / "manifest.json").read_text())["name"]
    variant = Path(args.map).stem.removeprefix("map_")
    scene = args.scene_dir.resolve() / (args.scene or f"_scene_{name}_{variant}.xml")

    points = np.asarray(o3d.io.read_point_cloud(str(session / "map" / args.map)).points)
    points = points[np.isfinite(points).all(axis=1)]
    leveled, rotation, floor_z, _ = level_cloud(points)
    with np.errstate(all="ignore"):
        traj = read_trajectory(session) @ rotation.T
    traj[:, 2] -= floor_z
    leveled = leveled[cKDTree(traj[:, :2]).query(leveled[:, :2])[0] <= MAX_RANGE]

    grid = build_grid(leveled, resolution=0.10, z_min=OBSTACLE_Z[0], z_max=OBSTACLE_Z[1])
    free = ~grid.blocked
    print(f"[grid] {grid.spec.width}×{grid.spec.height} セル / 自由 {int(free.sum()):,}", flush=True)
    # HTML の地点指定に同じ格子を使わせる。別々に作ると GUI と計画がズレる
    np.savez_compressed(sim_dir / "nav_grid.npz",
                        blocked=np.packbits(grid.blocked.ravel()),
                        shape=np.array(grid.blocked.shape),
                        origin=np.array([grid.spec.origin_x, grid.spec.origin_y]),
                        resolution=np.array(grid.spec.resolution))

    if args.waypoint:
        given = [tuple(float(v) for v in w.split(",")[:2]) for w in args.waypoint]
        if len(given) < 2:
            raise SystemExit("--waypoint は出発点を含めて2点以上要る")
        for x, y in given:
            if not grid.is_free(x, y):
                raise SystemExit(f"({x:.2f}, {y:.2f}) は障害物の中か地図の外。"
                                 "壁から 0.40m 以上離れた点を指定する")
        (sx, sy), (gx, gy) = given[0], given[-1]
        waypoints = [Pose2D(float(x), float(y), 0.0) for x, y in given]
    else:
        (r0, c0), (r1, c1) = farthest_pair(free)
        sx, sy = grid.spec.to_world(c0, r0)
        gx, gy = grid.spec.to_world(c1, r1)
        waypoints = [Pose2D(float(sx), float(sy), 0.0), Pose2D(float(gx), float(gy), 0.0)]
    start, goal = waypoints[0], waypoints[-1]
    print(f"[経路] 出発 ({start.x:.2f}, {start.y:.2f}) → 目標 ({goal.x:.2f}, {goal.y:.2f})"
          f"  直線 {math.hypot(gx - sx, gy - sy):.1f} m", flush=True)

    segments = plan_route(grid, waypoints)
    targets = [(s.target.x, s.target.y) for s in segments]
    route = np.array([[start.x, start.y]] + targets)
    planned = float(np.hypot(*np.diff(route, axis=0).T).sum())
    print(f"[経路] 区間 {len(segments)} / 経路長 {planned:.1f} m", flush=True)

    walker = G1Walker(build_scene_with_goals(scene, [(p.x, p.y) for p in waypoints[1:]]))
    yaw = math.atan2(targets[0][1] - start.y, targets[0][0] - start.x)
    place_robot(walker, start.x, start.y, yaw, 0.0)
    print(f"[歩行] {scene.name} を読み込んだ。開始", flush=True)

    recorder = Recorder(walker, args.width, args.height) if args.record else None
    result = drive(walker, targets, slam_service, args.timeout, recorder)
    history = result.pop("history")
    walked = float(np.hypot(*np.diff(history[:, 1:3], axis=0).T).sum())
    final = history[-1, 1:3]
    report = {
        "session": session.name, "variant": variant, "scene": scene.name,
        "start": [round(start.x, 2), round(start.y, 2)],
        "goal": [round(goal.x, 2), round(goal.y, 2)],
        "straight_m": round(float(math.hypot(gx - sx, gy - sy)), 2),
        "planned_m": round(planned, 2), "walked_m": round(walked, 2),
        "segments": len(segments),
        "final_gap_m": round(float(np.hypot(*(final - np.array([goal.x, goal.y])))), 2),
        **result,
    }
    report["check"] = check_scene(walker, sim_dir / f"{variant}.npz")
    check = report["check"]
    agree = (check["mjcf_boxes"] == check["html_boxes"] and check["all_boxes"]
             and abs(check["mjcf_volume_m3"] - check["html_volume_m3"]) < 0.5
             and (check["hfield_rows"], check["hfield_cols"])
             == (check["grid_rows"], check["grid_cols"]))
    print(f"[照合] MJCF の箱 {check['mjcf_boxes']:,} 個 / HTML が描く箱 {check['html_boxes']:,} 個"
          f" / 体積 {check['mjcf_volume_m3']:,.1f} vs {check['html_volume_m3']:,.1f} m³"
          f" / hfield {check['hfield_rows']}×{check['hfield_cols']} = 格子"
          f" {check['grid_rows']}×{check['grid_cols']}  → {'一致' if agree else '不一致'}",
          flush=True)

    if recorder is not None:
        video = write_video(recorder.frames, sim_dir / f"walk_{variant.replace('octomap_', '')}.mp4",
                            args.width, args.height)
        report["video"] = video.name
        print(f"[録画] {video}（{len(recorder.frames)} コマ / "
              f"{video.stat().st_size / 1e6:.1f} MB）", flush=True)

    np.savez_compressed(sim_dir / "walk.npz", history=history.astype(np.float32),
                        route=route.astype(np.float32))
    (sim_dir / "walk.json").write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                       encoding="utf-8")
    verdict = "到達" if report["reached"] else ("転倒" if report["fallen"] else "時間切れ")
    print(f"[結果] {verdict} / 歩いた距離 {walked:.1f} m（経路 {planned:.1f} m）/ "
          f"目標との差 {report['final_gap_m']:.2f} m / "
          f"sim {report['sim_s']:.0f} s を実時間 {report['wall_s']:.1f} s "
          f"({report['sim_s'] / max(report['wall_s'], 1e-9):.0f} 倍速)", flush=True)


if __name__ == "__main__":
    main()
