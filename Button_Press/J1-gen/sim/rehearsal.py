"""実機日のリハーサル: 当日手順書（real/REAL_DAY_PROCEDURE.md）の段階0〜6と故障の場面を、ループバックの
模擬ロボット（sim/sim_robot_server.py）に、実機用のスクリプトを当日と同じ引数で当てて通しで実行する（タスク6）。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/rehearsal.py

当日と違うのは、--network-interface lo と --camera-config camera_sim.yaml を付けることと、人が押す Enter / q を
あらかじめ用意した入力で流し込むことだけ。記録や教えた姿勢は、すべてリハーサルのフォルダに書く（本番の configs や
_local/button_press/runs には書かない）。

結果: _local/button_press/rehearsal/<日時>/report.md（各コマンドの所要時間・終了コード・期待どおりか、段階ごとの合計）と
logs/（各コマンドの出力）。

試せないもの（report.md にも書く）:
- g1-starter-kit の mode_check.py / armsdk_probe.py（実機と同じ domain 0 で動くので、domain 1 の模擬ロボットとはつながらない）
- 人が腕を動かす部分（teach.py / calibrate.py は流れだけ。模擬ロボットの腕は手で動かせないので、記録される値に意味は無い）
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import FEATURE_DIR, REPO_ROOT  # noqa: E402

PY = sys.executable
REAL = FEATURE_DIR / "real"
NET = ["--network-interface", "lo"]
CAM = ["--camera-config", "camera_sim.yaml"]


@dataclass
class Step:
    stage: str
    what: str
    cmd: list[str]
    expect: tuple[int, ...] = (0,)
    stdin: str = ""
    timeout: float = 300.0
    # 出力から拾ってレポートに載せる行（正規表現）
    grab: list[str] = field(default_factory=list)
    background: list[str] | None = None  # 同時に動かすコマンド（収録など）


@dataclass
class Scenario:
    name: str
    faults: list[str]
    steps: list[Step]


def build_scenarios(r: Path) -> list[Scenario]:
    press = [str(REAL / "press_bottle.py"), "--path", "arm_sdk", *NET, *CAM, "--detector", "color",
             "--sim-scene-obstacles", "--seed-pose", "", "--log-dir", str(r / "runs")]
    move = [str(REAL / "move_arm_real.py"), *NET]
    rec = [str(REAL / "record.py"), *NET, *CAM, "--output-dir", str(r / "recordings")]
    track = [r"最大の追従誤差 [0-9.]+°"]
    normal = Scenario("通常", [], [
        Step("0 接続確認", "check_connection.py --rgbd",
             [str(REAL / "check_connection.py"), *NET, *CAM, "--rgbd"], grab=[r"lowstate: \d+ Hz", r"相手: .*"]),
        Step("0 接続確認", "probe_rgbd.py --save",
             [str(REAL / "probe_rgbd.py"), "--camera-config", "camera_sim.yaml", "--seconds", "3", "--save",
              "--out-dir", str(r / "probe")], grab=[r"\d+ フレーム / .*fps）"]),
        Step("1 収録", "record.py（ボトル、全フレーム 20 秒）", [*rec, "--label", "bottle", "--duration-s", "20"],
             grab=[r"終了: .*"]),
        Step("1 収録", "record.py（ボタン、1 秒に 1 枚 15 秒）",
             [*rec, "--label", "button", "--mode", "interval", "--interval-s", "1", "--duration-s", "15"],
             grab=[r"終了: .*"]),
        Step("1 収録", "record.py（Enter で 1 枚 × 3）", [*rec, "--label", "button_enter", "--mode", "enter",
                                                       "--duration-s", "8"], stdin="\n\n\n", grab=[r"終了: .*"]),
        Step("2 経路の判定", "move_arm_real.py arm_sdk（dry-run）", [*move, "--path", "arm_sdk"]),
        Step("2 経路の判定", "move_arm_real.py arm_sdk --execute（確認モード、Enter で進める）",
             [*move, "--path", "arm_sdk", "--execute"], stdin="\n" * 8, grab=track),
        Step("3 ティーチング", "teach.py --name press_bottle（Enter で 1 回記録）",
             [str(REAL / "teach.py"), *NET, "--name", "press_bottle", "--poses-file", str(r / "taught_poses.yaml")],
             stdin="\n\nq\n", grab=[r"指先（FK.*"]),
        Step("3 ティーチング", "teach.py --obstacle table（角を 2 か所）",
             [str(REAL / "teach.py"), *NET, "--obstacle", "table", "--obstacles-file", str(r / "obstacles.yaml")],
             stdin="\n\n\nq\n", grab=[r"箱 table: .*"]),
        Step("3 FK の確認", "fk_check.py --overlay（3 秒）",
             [str(REAL / "fk_check.py"), *NET, *CAM, "--overlay", "--seconds", "3", "--out-dir", str(r / "fk_check")]),
        Step("3 重力補償", "move_arm_real.py lowcmd --gravity-scale 0",
             [*move, "--path", "lowcmd", "--execute", "--no-confirm", "--gravity-scale", "0"], stdin="\n", grab=track),
        Step("3 重力補償", "move_arm_real.py lowcmd --gravity-scale 0.5",
             [*move, "--path", "lowcmd", "--execute", "--no-confirm", "--gravity-scale", "0.5"], stdin="\n", grab=track),
        Step("3 重力補償", "move_arm_real.py lowcmd --gravity-scale 1.0",
             [*move, "--path", "lowcmd", "--execute", "--no-confirm", "--gravity-scale", "1.0"], stdin="\n", grab=track),
        Step("4 較正", "calibrate.py（3 か所。流れの確認のみ）",
             [str(REAL / "calibrate.py"), *NET, *CAM, "--detector", "color", "--out-dir", str(r / "calibration")],
             stdin="\n" + "\n\n" * 3 + "q\n", grab=[r"差の平均 .*"]),
        Step("5 押し込み", "press_bottle.py（dry-run）", [*press, "--label", "dry"],
             grab=[r"対象（深度）: .*", r"結果: .*"]),
        Step("5 押し込み", "press_bottle.py --execute（確認モード、Enter で進める）",
             [*press, "--label", "try1", "--execute"], stdin="\n" * 40, grab=[r"結果: .*"]),
        Step("6 繰り返し", "press_bottle.py --execute（record.py を同時に動かす）",
             [*press, "--label", "try2", "--execute", "--no-confirm"], grab=[r"結果: .*"],
             background=[*rec, "--label", "try2", "--duration-s", "75"]),
        Step("6 繰り返し", "press_bottle.py --target manual（定規モード）",
             [*press, "--label", "manual", "--target", "manual", "--point", "0.388", "-0.20", "0.03", "--execute",
              "--no-confirm"], grab=[r"結果: .*"]),
    ])
    plan_b = Scenario("故障: arm_sdk が効かない → プランB へ切り替え", ["ignore_arm_sdk"], [
        Step("2 経路の判定", "move_arm_real.py arm_sdk（動かないことを検出）",
             [*move, "--path", "arm_sdk", "--execute", "--no-confirm"], expect=(3,), grab=[r"❌ .*"]),
        Step("2 経路の判定", "move_arm_real.py lowcmd（プランB。支持の確認に Enter）",
             [*move, "--path", "lowcmd", "--execute", "--no-confirm"], stdin="\n", grab=track),
        Step("5 押し込み", "press_bottle.py --path lowcmd（プランB で押す）",
             [*[a if a != "arm_sdk" else "lowcmd" for a in press], "--label", "plan_b", "--execute", "--no-confirm"],
             stdin="\n", grab=[r"結果: .*"]),
    ])
    zero = Scenario("故障: ゼロトルク（mode=0）", ["motor_mode0"], [
        Step("0 接続確認", "check_connection.py（mode が 0 と表示）", [str(REAL / "check_connection.py"), *NET],
             expect=(1,), grab=[r"❌ .*"]),
        Step("5 押し込み", "press_bottle.py --execute（送る前に中止）", [*press, "--label", "zero", "--execute",
                                                                   "--no-confirm"], expect=(4,), grab=[r"結果: .*"]),
    ])
    mm = Scenario("故障: mode_machine が 5 でない", ["mode_machine"], [
        Step("5 押し込み", "press_bottle.py --execute（送る前に中止）", [*press, "--label", "mm", "--execute",
                                                                   "--no-confirm"], expect=(4,), grab=[r"結果: .*"]),
    ])
    drop = Scenario("故障: 押し込みの途中で lowstate が途切れる（開始 30 秒後）", ["lowstate_dropout", "--fault-after", "30"], [
        Step("5 押し込み", "press_bottle.py --execute（安全に止まる）", [*press, "--label", "drop", "--execute",
                                                                  "--no-confirm"], expect=(4,), grab=[r"結果: .*"]),
    ])
    sag = Scenario("故障: 押し込みの途中で腰が倒れていく（開始 30 秒後）", ["waist_sag", "--fault-after", "30"], [
        Step("5 押し込み", "press_bottle.py --execute（安全に止まる）", [*press, "--label", "sag", "--execute",
                                                                  "--no-confirm"], expect=(5,), grab=[r"結果: .*"]),
    ])
    return [normal, plan_b, zero, mm, drop, sag]


def fault_args(faults: list[str]) -> list[str]:
    out: list[str] = []
    it = iter(faults)
    for f in it:
        if f.startswith("--"):
            out += [f, next(it)]
        else:
            out += ["--fault", f]
    return out


def run_step(step: Step, log: Path) -> tuple[int, float, str]:
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    bg = None
    if step.background:
        bg = subprocess.Popen([PY, *step.background], cwd=REPO_ROOT, stdout=open(log.with_suffix(".bg.log"), "w"),
                              stderr=subprocess.STDOUT, text=True, env=env)
        time.sleep(2.0)
    t0 = time.monotonic()
    try:
        p = subprocess.run([PY, *step.cmd], cwd=REPO_ROOT, input=step.stdin, capture_output=True, text=True,
                           timeout=step.timeout, env=env)
        code, out = p.returncode, p.stdout + p.stderr
    except subprocess.TimeoutExpired as e:
        code, out = -1, f"タイムアウト（{step.timeout} 秒）\n{e.stdout or ''}"
    dt = time.monotonic() - t0
    if bg is not None:
        bg.wait(timeout=120)
    log.write_text(out, encoding="utf-8")
    notes = []
    for pat in step.grab:
        notes += re.findall(pat, out)
    return code, dt, " / ".join(dict.fromkeys(n.strip() for n in notes))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", help="この名前を含むシナリオだけ実行する")
    args = ap.parse_args()
    root = REPO_ROOT / "_local" / "button_press" / "rehearsal" / time.strftime("%Y%m%d_%H%M%S")
    (root / "logs").mkdir(parents=True)
    rows: list[tuple] = []
    n = 0
    t_all = time.monotonic()
    for sc in build_scenarios(root):
        if args.only and args.only not in sc.name:
            continue
        print(f"[rehearsal] === {sc.name} ===", flush=True)
        slog = open(root / "logs" / f"server_{len(rows):02d}.log", "w")
        server = subprocess.Popen([PY, "-u", str(FEATURE_DIR / "sim" / "sim_robot_server.py"), *fault_args(sc.faults)],
                                  cwd=REPO_ROOT, stdout=slog, stderr=subprocess.STDOUT, text=True)
        time.sleep(5.0)
        try:
            for st in sc.steps:
                n += 1
                name = re.sub(r"[^0-9A-Za-z_]+", "_", st.what)[:40]
                code, dt, note = run_step(st, root / "logs" / f"{n:02d}_{name}.log")
                ok = code in st.expect
                print(f"[rehearsal] {st.stage} | {st.what} | {dt:.1f} 秒 | 終了コード {code}"
                      f"（期待 {st.expect}）{'OK' if ok else '❌'} | {note}", flush=True)
                rows.append((sc.name, st.stage, st.what, dt, code, st.expect, ok, note, n))
        finally:
            server.terminate()
            server.wait(timeout=15)
            slog.close()
    total = time.monotonic() - t_all

    lines = ["# 実機日のリハーサル（ループバックの模擬ロボット）", "",
             f"- 日時: {root.name}、全体 {total / 60:.1f} 分（模擬ロボットの起動待ちを含む）",
             f"- 結果: {sum(r[6] for r in rows)} / {len(rows)} 件が期待どおり", "",
             "| # | 場面 | 段階 | 内容 | 秒 | 終了コード | 期待 | 結果 | メモ |", "|---|---|---|---|---|---|---|---|---|"]
    for sc, stage, what, dt, code, exp, ok, note, i in rows:
        lines.append(f"| {i} | {sc} | {stage} | {what} | {dt:.1f} | {code} | {'/'.join(map(str, exp))} | "
                     f"{'OK' if ok else '❌'} | {note.replace('|', '/')} |")
    lines += ["", "## 段階ごとの合計（通常の場面のみ）", "", "| 段階 | 秒 |", "|---|---|"]
    stages: dict[str, float] = {}
    for sc, stage, _, dt, *_ in rows:
        if sc == "通常":
            stages[stage] = stages.get(stage, 0.0) + dt
    for stage, dt in stages.items():
        lines.append(f"| {stage} | {dt:.0f} |")
    lines += ["", "## 試せなかったもの", "",
              "- mode_check.py / armsdk_probe.py（domain 0 で動くので、domain 1 の模擬ロボットとはつながらない）",
              "- 人が腕を動かす部分（teach.py / calibrate.py は流れだけ。記録された値に意味は無い）",
              "- 本物のカメラ（YOLO、RealSense の深度）と、本物の腕の力（押し込み・重力の効き方）"]
    (root / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[rehearsal] レポート: {root / 'report.md'}")
    return 0 if all(r[6] for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
