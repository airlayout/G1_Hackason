"""Isaac Sim でエレベーター乗り場のシーンを見る・確かめる。run.sh から起動する（直接は起動しない）。

    bash Button_Press/Yada/sim/isaac/run.sh                         # 画面で見る（Shift + ドラッグでボタンを押せる）
    bash Button_Press/Yada/sim/isaac/run.sh --headless --png _local/button_press_yada/isaac_hall
    bash Button_Press/Yada/sim/isaac/run.sh --headless --press up --png _local/button_press_yada/isaac_pressed

G1 は IK（J1-gen の Pinocchio）と同じ公式 URDF（g1_29dof_rev_1_0）から読み込み、pelvis をワールドに固定する。
関節は configs/elevator_hall.yaml の initial_pose_deg の姿勢に保つ（歩行も腕の制御もまだ無いため）。
"""

from __future__ import annotations

import argparse

import sys
from pathlib import Path

# --- Isaac Sim の起動は他の import より先に行う ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Isaac Sim のエレベーター乗り場のシーン")
parser.add_argument("--scene", default="elevator_hall.yaml", help="シーンの設定（configs/ 基準）")
parser.add_argument("--png", default="", help="画像を保存する（パスの先頭。_<カメラ名>.png を付けて保存）")
parser.add_argument("--press", default="", help="このボタンをばねの目標を変えて押し、沈み量と点灯を確かめる（up / down）")
parser.add_argument("--max-steps", type=int, default=0, help="画面で見るとき、この物理ステップ数で終了する（0 なら閉じるまで）")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- ここから下は Isaac Sim 起動後にのみ import できる ---
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.config import REPO_ROOT, load_config  # noqa: E402
from isaac_world import RENDER_EVERY, build_world, save_png  # noqa: E402


def main() -> None:
    w = build_world(load_config(args.scene), args.device, cameras=bool(args.enable_cameras))
    scene, hall = w.scene, w.hall
    print(f"[OK] G1 と乗り場を作った（胴体固定、pelvis = {w.pelvis_world.tolist()}、ボタン: {[b.name for b in scene.buttons]}）")
    w.sim.set_camera_view(eye=(w.pelvis_world + [-1.3, -1.1, 0.9]).tolist(),
                          target=(w.pelvis_world + [0.42, -0.10, 0.1]).tolist())
    print(f"[hall] 関節: {w.hall_art.joint_names}")

    # 置いた直後の揺れが収まるまで進める
    w.step(200)
    print(f"[hall] 沈み量 [mm]: { {n: round(d * 1000, 2) for n, d in hall.depths().items()} }")

    if args.press:
        b = scene.button(args.press)
        hall.set_targets({b.name: b.travel})
        pressed = w.step(150)
        print(f"[press] 押す: 沈み量 {hall.depths()[b.name] * 1000:.2f} mm、点灯 {hall.lit(b.name)}、押された {pressed}")
        hall.set_targets({b.name: 0.0})
        w.step(150)
        print(f"[press] 離す: 沈み量 {hall.depths()[b.name] * 1000:.2f} mm、点灯 {hall.lit(b.name)}")
        others = [n for n in hall.states if n != b.name and hall.lit(n)]
        print(f"[press] ほかに点灯しているボタン: {others}")

    if args.png:
        if not w.cameras:
            print("[NG] --png には --enable_cameras が要る（run.sh は自動で付ける）")
        else:
            w.step(RENDER_EVERY * 5)
            prefix = Path(args.png)
            save_png(w.cameras, prefix if prefix.is_absolute() else REPO_ROOT / prefix)

    if not args.headless:
        print("[view] 画面で見る（Shift + 左ドラッグでボタンを押せる。閉じると終了）")
        n = 0
        while simulation_app.is_running() and (args.max_steps == 0 or n < args.max_steps):
            for name in w.step(RENDER_EVERY):
                print(f"[view] {name} が押された")
            n += RENDER_EVERY
    else:
        print("[INFO] ヘッドレスで起動したので終了する（画面で見るときは --headless を付けずに run.sh で起動する）")


if __name__ == "__main__":
    main()
    # IsaacLab の tutorials と同じく main() の外で閉じる（finally で閉じると例外が隠れる。CLAUDE.md）
    simulation_app.close()
