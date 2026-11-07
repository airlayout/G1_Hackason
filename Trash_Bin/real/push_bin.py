"""ゴミ箱のフラップを押して、頭カメラで中を撮る（実機）。ラボ PC で実行する。手順は real/REAL_PROCEDURE.md。

    P=G1_HuggingFace/venv/bin/python
    # 1. 見つけるだけ（腕は動かさない）。結果の画像（緑の長方形・押す点）と、床から求めた pelvis の高さを出す
    $P Trash_Bin/real/push_bin.py --locate-only
    # 2. dry-run（既定）: 事前確認 → 見つける → 計画 → 指令の計算まで。送信しない
    $P Trash_Bin/real/push_bin.py
    # 3. 空振り: ゴミ箱を置かずに、設定の置き方（シミュレーションで確かめた位置）に向けて同じ腕の動きをする。体の動きを見る
    $P Trash_Bin/real/push_bin.py --air --execute
    # 4. 本番: 事前確認 → 見つける → 押す → 中を撮る → 戻る（各段階で Enter）。腰は動かさない
    $P Trash_Bin/real/push_bin.py --execute

腕を動かす前に、ボタン押し（J1-gen の auto_press.py）と同じ事前確認をする（real/precheck.py）。通らなければ何も送らない。
経路は arm_sdk（プランA）だけ。lowcmd（プランB）はバランス制御が止まるので使わない。
押す流れは Button_Press/J1-gen の PressPipeline（import して使う）。記録は _local/trash_bin/runs/<日時>_<ラベル>/。
押し切った瞬間の頭カメラの画像（中身の確認に使う）は frames/ の下の *_end_color.png。

⚠️ 実機で送る前に: 人がリモコンを持つ（緊急時は L2+B）。最初はハーネスを付ける。
⚠️ シミュレーションは pelvis を固定しているので、腕を前に出したときのバランスは確かめていない（README の「バランスの根拠」）。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Button_Press" / "J1-gen"))
sys.path.insert(0, str(REPO / "Trash_Bin" / "lib"))
sys.path.insert(0, str(REPO / "Trash_Bin" / "sim"))  # 設定の読み込み（bin_scene.load_scene_config）だけ使う

from bin_geometry import BinPose, pose_from_config  # noqa: E402
from bin_scene import load_scene_config  # noqa: E402
from bin_task import BinPipeline, apply_measured_pelvis_height, configure_pipeline, install_leaning  # noqa: E402
from common.arm import joint_limits, make_backend  # noqa: E402
from common.arm.gravity import GravityModel, needs_gravity_model  # noqa: E402
from common.camera_cfg import add_camera_args, apply_head_camera_mount, load_camera_config  # noqa: E402
from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.dds import peer_label  # noqa: E402
from common.pipeline_cli import add_common_args, load_pipeline_config, make_logger  # noqa: E402
from hand_shape import install as install_hand_shape  # noqa: E402
from lean import LeaningCommander  # noqa: E402
from locate_bin import LocateError, draw_detection, locate_bin, measure_pelvis_height  # noqa: E402

EXIT_PRECHECK = 10  # J1-gen の auto_press.py と同じ
MAX_TILT_CHANGE_DEG = 1.0  # 1 回ごとの体の傾きの変化の目安。超えたら次の段階へ進まない（J1-gen が 2026-11-01 に使った値）


def report_body_motion(log_dir: Path, code: int) -> None:
    """記録（events.jsonl の body_motion）から体の傾きの変化を読み、次の段階へ進んでよいかの目安を表示する。"""
    import json

    if code != 0:
        print(f"[push_bin] ⚠️ 途中で中止した（終了コード {code}）。次の段階へ進まない。理由を確かめる")
        return

    ev = None
    try:
        for line in open(log_dir / "events.jsonl", encoding="utf-8"):
            e = json.loads(line)
            if e.get("event") == "body_motion":
                ev = e
    except OSError:
        pass
    if ev is None or ev.get("imu_tilt_max_deg") is None:
        print("[push_bin] 体の動きの記録が無い（体の傾きは画面の表示か body_motion.csv で見る）")
        return
    tilt = float(ev["imu_tilt_max_deg"])
    ok = tilt <= MAX_TILT_CHANGE_DEG
    print(f"[push_bin] 体の傾きの変化の最大: {tilt:.2f}°（目安 {MAX_TILT_CHANGE_DEG}° 以下）→ "
          + ("次の段階へ進んでよい" if ok else "⚠️ 目安を超えた。次の段階へ進まない（ハーネスを外さない・ゴミ箱を置かない）"))


def wait_state(backend, timeout_s: float = 5.0):  # noqa: ANN001,ANN201
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        st = backend.read_state()
        if st is not None:
            return st
        time.sleep(0.05)
    raise RuntimeError(f"{timeout_s} 秒待っても lowstate が届かない（接続・NIC 名・電源を確認）")


def locate_from_frames(src, backend, transform, scene, n: int, logger, max_spread_m: float, max_spread_deg: float):  # noqa: ANN001,ANN201
    """n 枚の深度でゴミ箱を測り、中央値の置き方を返す。ばらつきが大きい・見つからない枚が多いときは LocateError。"""
    poses, last = [], None
    for k in range(n):
        f = src.read_rgbd()
        st = backend.read_state()
        if f is None or st is None:
            print(f"[push_bin] {k + 1} 枚目: フレームか lowstate が無い")
            continue
        q_waist = st.q[12:15].copy()
        try:
            det = locate_bin(f, transform, q_waist, scene)
        except LocateError as e:
            print(f"[push_bin] {k + 1} 枚目: 見つからない（{e}）")
            logger.save_frame(f"locate_fail{k + 1}", f)
            continue
        print(f"[push_bin] {k + 1} 枚目: {det.summary()}")
        poses.append(det.pose)
        last = (f, q_waist, det)
    if len(poses) < max(1, (n + 1) // 2):
        raise LocateError(f"見つかったのは {len(poses)}/{n} 枚。深度が欠けている（鏡面）か、ゴミ箱が写っていない")
    centers = np.array([p.center_xy for p in poses])
    yaws = np.array([p.yaw for p in poses])
    spread = float(np.max(centers.max(axis=0) - centers.min(axis=0)))
    spread_deg = float(np.degrees(yaws.max() - yaws.min()))
    if spread > max_spread_m or spread_deg > max_spread_deg:
        raise LocateError(f"枚ごとのばらつきが大きい（中心 {spread * 100:.1f} cm、向き {spread_deg:.1f}°）")
    pose = BinPose(center_xy=np.median(centers, axis=0), yaw=float(np.median(yaws)))
    return pose, last


def stand_advice(pose: BinPose, scene: dict) -> str:
    """測った置き方と、設定の置き方（シミュレーションで確かめた位置）の差を、立つ位置の直し方の文にする。"""
    nominal = pose_from_config(scene)
    d = pose.center_xy - nominal.center_xy
    dyaw = np.degrees(pose.yaw - nominal.yaw)
    msgs = []
    if abs(d[0]) > 0.02:
        msgs.append(f"ゴミ箱が {abs(d[0]) * 100:.0f} cm {'遠い（前へ出る）' if d[0] > 0 else '近い（下がる）'}")
    if abs(d[1]) > 0.02:
        msgs.append(f"ゴミ箱が {abs(d[1]) * 100:.0f} cm {'左にある（左へ寄る）' if d[1] > 0 else '右にある（右へ寄る）'}")
    if abs(dyaw) > 5:
        msgs.append(f"ゴミ箱が {abs(dyaw):.0f}° {'左回り' if dyaw > 0 else '右回り'}に向いている")
    return "、".join(msgs)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は J1-gen の configs/arm.yaml。lo なら模擬ロボット）")
    add_camera_args(p)
    p.add_argument("--scene", help="ゴミ箱の設定（既定は Trash_Bin/configs/bin_scene.yaml）")
    p.add_argument("--waist-pitch", type=float, help="押す前に腰を前に傾ける角度 [deg]（既定は設定の lean_waist_pitch_deg）")
    p.add_argument("--frames", type=int, default=5, help="ゴミ箱を測る枚数（中央値を使う）")
    p.add_argument("--locate-only", action="store_true", help="見つけるだけ（腕も腰も動かさない）")
    p.add_argument("--air", action="store_true",
                   help="空振り: ゴミ箱を置かずに、設定の置き方に向けて同じ腕の動きをする（深度で探さない。床から pelvis の高さだけ測る）")
    p.add_argument("--lean-only", action="store_true", help="腰だけを傾けて戻す（腕は動かさない）")
    p.add_argument("--lean-hold-s", type=float, default=3.0, help="--lean-only で、傾けたまま保つ秒数")
    p.add_argument("--execute", action="store_true", help="実際に送信する（付けなければ dry-run）")
    p.add_argument("--no-confirm", action="store_true", help="各段階の Enter 待ちを省く")
    add_common_args(p)
    args = p.parse_args()

    scene = load_scene_config(args.scene)
    if args.waist_pitch is not None:
        scene["lean_waist_pitch_deg"] = args.waist_pitch
    # J1-gen の実機日の机の箱（configs/obstacles.yaml）はボトル用なので、--obstacles-file で指定したときだけ読む
    if args.obstacles_file is None:
        args.obstacles_file = ""
    # 腕の重力補償の倍率: 実機は J1-gen と同じ 0.5（2026-10-14 の実測で決めた値。auto_press.py の既定）。
    # 模擬ロボット（lo）は MuJoCo の側で補償しているので 0 のまま（二重になる）
    real_robot = (args.network_interface or "") != "lo"
    if args.gravity_scale is None and real_robot:
        args.gravity_scale = 0.5
    cfg = load_pipeline_config(args)
    cfg.pipeline["log_dir"] = args.log_dir or "_local/trash_bin/runs"
    side = cfg.pipeline.get("arm") or cfg.arm["arm"]
    cfg.arm["arm"] = side
    cam_cfg = load_camera_config(args, quiet=True)
    apply_head_camera_mount(cfg.robot, cam_cfg)
    if scene.get("hand_shape", "box") == "parts":
        install_hand_shape(cfg.robot)
    if args.network_interface:
        cfg.arm["network_interface"] = args.network_interface
    iface = cfg.arm["network_interface"]
    if args.air and args.locate_only:
        p.error("--air と --locate-only は一緒に使えない")
    mode = "見つけるだけ" if args.locate_only else ("腰だけ" if args.lean_only else ("空振り" if args.air else "押す"))
    print(f"[push_bin] {mode}。ネットワークの口: {iface or '（未設定）'}、相手: {peer_label(iface) if iface else '?'}、"
          f"{'送信する（--execute）' if args.execute else 'dry-run（送信しない）'}、腰 {scene.get('lean_waist_pitch_deg') or 0}°、"
          f"腕の重力補償 {cfg.arm['gravity_compensation']['scale']}")

    backend = make_backend(cfg.arm, cfg.robot, dry_run=not args.execute, path="arm_sdk")
    logger = make_logger(cfg, args.label or mode, {"kind": "real", "mode": mode, "execute": args.execute,
                                                   "network_interface": iface, "scene": scene})
    src = None
    code = 0
    try:
        backend.open()
        wait_state(backend)

        if args.lean_only:
            # 腰だけ: 指令を始める → 傾ける → 保つ → 戻す → 終える（lib/lean.py）
            lower, upper = joint_limits(cfg.robot)
            gravity = GravityModel(cfg.robot) if needs_gravity_model(cfg.arm) else None
            lean = float(np.radians(float(scene.get("lean_waist_pitch_deg") or 0.0)))
            lc = scene.get("lean") or {}
            with LeaningCommander(backend, cfg.arm, lower, upper, confirm=args.execute and not args.no_confirm,
                                  gravity=gravity, lean_rad=lean, max_speed_deg_s=float(lc.get("max_speed_deg_s", 5.0)),
                                  settle_s=float(lc.get("settle_s", 1.0)),
                                  max_error_deg=float(lc.get("max_error_deg", 2.0))) as arm:
                print(f"[push_bin] 傾けたまま {args.lean_hold_s} 秒保つ")
                for _ in range(int(args.lean_hold_s / arm.dt)):
                    arm._send_tick()  # noqa: SLF001（J1-gen の保持の 1 周期。腰のずれの確認も入る）
            logger.close(code=0, message="腰だけ")
            return 0

        # 頭カメラの深度でゴミ箱を測る（腕・腰はまだ動かさない。腕はカメラの視野から外しておく）
        from common.camera_rgbd import RgbdZmqSource

        c = load_camera_config(args)["rgbd"]
        src = RgbdZmqSource(c["server_address"], int(c["port"]), int(c["timeout_ms"]))
        src.open()
        if not args.locate_only:
            # 腕を動かす前の事前確認（J1-gen のボタン押しと同じ項目・しきい値。real/precheck.py）
            from precheck import run_prechecks

            checks = run_prechecks(wait_state(backend), src, mock=iface == "lo")
            for ck in checks:
                print(f"[push_bin] 事前確認 {'OK' if ck.ok else 'NG'} {ck.name}: {ck.message}")
            logger.event("precheck", checks=[{"name": ck.name, "ok": ck.ok, "message": ck.message} for ck in checks])
            if not all(ck.ok for ck in checks):
                print("[push_bin] ❌ 事前確認が通らない。腕には何も送らずに終わる")
                logger.close(code=EXIT_PRECHECK, message="事前確認が通らない")
                return EXIT_PRECHECK
        transform = HeadCameraTransform(cfg.robot, np.asarray(cfg.localize["calibration"]["offset_pelvis_m"], dtype=float))
        if args.air:
            # 空振り: ゴミ箱は無いので探さない。シミュレーションで確かめた置き方（設定）に向けて、本番と同じ計画・動きをする。
            # 床から pelvis の高さは測って表示する（台の高さを決める材料。押す点には使わない＝シミュレーションと同じ動きにする）
            pose = pose_from_config(scene)
            frame, st = src.read_rgbd(), wait_state(backend)
            height = measure_pelvis_height(frame, transform, st.q[12:15], scene) if frame is not None else None
            print(f"[push_bin] 空振り。床から求めた pelvis の高さ: "
                  + (f"{height:.3f} m（設定 {scene['pelvis_height_m']:.3f} m）→ 台の高さの目安 "
                     f"{(float(scene['bin']['stand_height_m']) + height - float(scene['pelvis_height_m'])) * 100:.0f} cm"
                     if height is not None else "床が写っていないので測れない"))
            logger.event("air", pelvis_height_m=height)
            if frame is not None:
                logger.save_frame("air_start", frame, waist_q=st.q[12:15])
            target = configure_pipeline(cfg, scene, side, pose, keep_speed=args.speed_mm_s is not None)
            print(f"[push_bin] 押す点（空中）: [{target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}] m。"
                  "ゴミ箱が無いので、計画の深さまで押し込んで保持し、戻る")
            lean_rad = install_leaning(scene)
            pipe = BinPipeline(cfg, backend, logger, src.read_rgbd, None, confirm=args.execute and not args.no_confirm,
                               finger_tilts_deg=scene["push"].get("finger_tilts_deg") or [[0, 0]], lean_rad=lean_rad,
                               extra_seeds_deg=scene["push"].get("ik_seeds_deg"))
            res = pipe.run()
            code = res.code
            report_body_motion(logger.dir, code)
            print(f"[push_bin] 記録: {logger.dir}")
            return code
        pose, (frame, q_waist, det) = locate_from_frames(src, backend, transform, scene, args.frames, logger,
                                                         max_spread_m=0.02, max_spread_deg=3.0)
        apply_measured_pelvis_height(scene, det)  # 押す点の高さを、実機の pelvis の高さに合わせる
        target = configure_pipeline(cfg, scene, side, pose, keep_speed=args.speed_mm_s is not None)
        print(f"[push_bin] ゴミ箱: 中心 [{pose.center_xy[0]:+.3f}, {pose.center_xy[1]:+.3f}] m、向き {np.degrees(pose.yaw):+.1f}°"
              f" → 押す点 [{target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}] m")
        advice = stand_advice(pose, scene)
        if advice:
            print(f"[push_bin] ⚠️ シミュレーションで確かめた位置との差: {advice}（±3 cm・±10° を超えると届かないことがある）")
        logger.event("bin_located", center_xy=pose.center_xy, yaw_deg=float(np.degrees(pose.yaw)), target=target,
                     advice=advice)
        logger.save_frame("locate", frame, waist_q=q_waist)
        logger.save_image("locate_overlay", draw_detection(frame, transform, q_waist, scene, det, target))
        if args.locate_only:
            print(f"[push_bin] 見つけるだけで終わる。重ねた画像: {logger.dir}（locate_overlay）")
            logger.close(code=0, message="見つけるだけ")
            return 0

        lean_rad = install_leaning(scene)
        pipe = BinPipeline(cfg, backend, logger, src.read_rgbd, None, confirm=args.execute and not args.no_confirm,
                           finger_tilts_deg=scene["push"].get("finger_tilts_deg") or [[0, 0]], lean_rad=lean_rad,
                           extra_seeds_deg=scene["push"].get("ik_seeds_deg"))
        pipe.contents_ctx = {"scene": scene, "pose": pose, "transform": transform}  # 保持中に中身を深度で測る
        res = pipe.run()
        code = res.code
        report_body_motion(logger.dir, code)
        from contents import best_of, summary

        print(f"[push_bin] {summary(best_of(pipe.contents))}（画像でも確かめる）")
        print(f"[push_bin] 中の確認用の頭カメラの画像（フラップを押し下げている間）: {logger.dir}/frames/ の"
              " *_end_color.png（押し込みが止まった瞬間）と *_hold1〜3_color.png（保持中に 0.5 秒ごと）")
    except LocateError as e:
        print(f"[push_bin] ❌ ゴミ箱を見つけられない: {e}")
        logger.close(code=2, message=str(e))
        code = 2
    except (ValueError, RuntimeError) as e:  # NIC 名が空、lowstate が来ない、など
        print(f"[push_bin] ❌ {e}")
        logger.close(code=2, message=str(e))
        code = 2
    finally:
        backend.close()
        if src is not None:
            src.close()
    print(f"[push_bin] 記録: {logger.dir}")
    return code


if __name__ == "__main__":
    sys.exit(main())
