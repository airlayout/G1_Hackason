#!/usr/bin/env python3
"""[G1_Hackason 改変] stick-pause／record-loco-cmd／xr-stale-stop の自己テスト（機体・Quest 不要）。

修正済みの teleop_hand_and_arm.py から該当の部分だけを抜き出し、偽の機体・偽の時計・偽の Quest で
1 周期ずつ回して振る舞いを確かめる。setup/apply_patches.py で当てた後に流す。

    python3 tools/stick_pause_selftest.py                       # ~/xr_teleoperate を見る
    python3 tools/stick_pause_selftest.py --file <teleop_hand_and_arm.py> --televuer <televuer.py>

確かめること（stick-pause・record-loco-cmd）:
  1. 0.5 秒未満の押し込みでは切り替わらない
  2. 0.5 秒押し続けると一時停止: 歩きの指令は 0、腕は止めた瞬間の実測の角度
  3. 押し続けても再び切り替わらない（離すまで次の切り替えをしない）
  4. 一時停止中は Damp も腕の帰還も呼ばれない
  5. 再開すると腕の速さを 2 rad/s に絞り、2 秒後に 30 rad/s へ戻す
  6. 記録の歩きの指令は、機体へ送った値と同じ（一時停止中は 0）
確かめること（xr-stale-stop。当たっていれば）:
  7. 最初のデータが届く前は止めない
  8. 0.3 秒届かないと一時停止（最後のスティックが前でも歩かない）
  9. 届かない間・届き始めて押したままでは再開しない。離して押し直すと再開する
  10. televuer が手とコントローラの両方で届いた時刻を残す
  11. （xr-stale-log が当たっていれば）途切れの長さが [xr-gap] でログに残る
"""

import argparse
import ast
import sys
import textwrap
import types
from pathlib import Path

import numpy as np

TELEOP = Path.home() / "xr_teleoperate" / "teleop"
DEFAULT_FILE = TELEOP / "teleop_hand_and_arm.py"
DEFAULT_TELEVUER = TELEOP / "televuer" / "src" / "televuer" / "televuer.py"
DT = 0.02  # 1 周期の秒数（テスト用）


def extract(text: str, start: str, end: str, include_end: bool) -> str:
    """start を含む行から end を含む行まで（include_end で end 行を含めるか）を字下げを外して返す。"""
    lines = text.splitlines(keepends=True)
    i = next(k for k, ln in enumerate(lines) if start in ln)
    j = next(k for k in range(i + 1, len(lines)) if end in lines[k])
    return textwrap.dedent("".join(lines[i:j + 1 if include_end else j]))


class FakeClock:
    """time.time() と time.monotonic() の両方を同じ偽の時刻で返す。"""

    def __init__(self) -> None:
        self.now = 1000.0

    def time(self) -> float:
        return self.now

    def monotonic(self) -> float:
        return self.now


class FakeArm:
    def __init__(self) -> None:
        self.measured = np.linspace(0.1, 1.4, 14)
        self.sent_q = None
        self.velocity_limit = 30.0
        self.go_home_called = False

    def get_current_dual_arm_q(self) -> np.ndarray:
        return self.measured.copy()

    def get_current_dual_arm_dq(self) -> np.ndarray:
        return np.zeros(14)

    def get_current_motor_q(self) -> np.ndarray:
        return np.zeros(29)

    def ctrl_dual_arm(self, q, tauff) -> None:
        self.sent_q = np.array(q)

    def set_arm_velocity_limit(self, v: float) -> None:
        self.velocity_limit = float(v)

    def ctrl_dual_arm_go_home(self) -> None:
        self.go_home_called = True


class FakeLoco:
    def __init__(self) -> None:
        self.moves = []
        self.damp_called = False

    def Move(self, vx: float, vy: float, vyaw: float) -> None:
        self.moves.append((vx, vy, vyaw))

    def Enter_Damp_Mode(self) -> None:
        self.damp_called = True


class FakeIK:
    def solve_ik(self, lw, rw, q, dq):
        # 操作者の手の位置に相当する解。止めた角度とは違う値にしておく。
        return np.full(14, 0.9), np.full(14, 0.05)


def make_tele(both_pressed: bool, stick_fwd: float, ready: bool) -> types.SimpleNamespace:
    return types.SimpleNamespace(
        right_ctrl_aButton=False,
        left_ctrl_thumbstick=both_pressed,
        right_ctrl_thumbstick=both_pressed,
        left_ctrl_thumbstickValue=np.array([0.0, -stick_fwd]),   # 前は (0, -1)
        right_ctrl_thumbstickValue=np.array([0.0, 0.0]),
        left_wrist_pose=None,
        right_wrist_pose=None,
        motion_data_ready=ready,
    )


def check_televuer(path: Path, check) -> None:
    """televuer が手とコントローラの両方で届いた時刻を残すか（文字列と構文で見る）。"""
    if not path.exists():
        check(False, f"televuer.py が見つからない: {path}")
        return
    text = path.read_text(encoding="utf-8")
    ast.parse(text)
    check(text.count("self.last_motion_time_shared.value = time.monotonic()") == 2,
          "televuer: 手とコントローラの 2 か所で届いた時刻を残す")
    check("def last_motion_time(self):" in text and "\nimport time\n" in text,
          "televuer: last_motion_time を読める（time を読み込んでいる）")


def main() -> int:
    p = argparse.ArgumentParser(description="stick-pause／record-loco-cmd／xr-stale-stop の自己テスト")
    p.add_argument("--file", type=Path, default=DEFAULT_FILE)
    p.add_argument("--televuer", type=Path, default=DEFAULT_TELEVUER)
    args_cli = p.parse_args()
    text = args_cli.file.read_text(encoding="utf-8")
    for marker in ("PATCH(stick-pause)", "PATCH(record-loco-cmd)"):
        if marker not in text:
            print(f"[自己テスト] ✗ {marker} が当たっていません: {args_cli.file}")
            return 1
    has_stale = "PATCH(xr-stale-stop)" in text

    init_src = extract(text, "PATCH(stick-pause): 両スティック長押し", "while not STOP:", False)
    # 11-04: 修正 seated-controls が塊の頭の if を書き換えるので、その上の上流のコメントから抜き出す
    high_src = extract(text, "# high level control", "# get current robot state data.", False)
    ik_src = extract(text, "sol_q, sol_tauff  = arm_ik.solve_ik", "arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)", True)
    rec_src = extract(text, "PATCH(record-loco-cmd): 手なしでも", "current_body_action = []", True)
    rec_src = "if True:\n" + textwrap.indent(rec_src, "    ")

    clock, arm, loco = FakeClock(), FakeArm(), FakeLoco()
    infos = []
    tvuer = types.SimpleNamespace(last_motion_time=0.0)
    ns = {
        "time": clock, "arm_ctrl": arm, "loco_wrapper": loco, "arm_ik": FakeIK(),
        "tv_wrapper": types.SimpleNamespace(tvuer=tvuer),
        "args": types.SimpleNamespace(input_mode="controller", motion=True, loco_fwd_max=0.3,  # 前の上限は loco-fwd-max の既定
                                      no_loco=False),  # 足あり（修正 no-loco の既定）
        "logger_mp": types.SimpleNamespace(warning=lambda m: print(f"    （記録係）{m}"),
                                           info=lambda m: (infos.append(m), print(f"    （記録係）{m}")),
                                           debug=lambda m: None),
        "START": True, "STOP": False,
        "current_lr_arm_q": None, "current_lr_arm_dq": None,
        "time_ik_start": 0.0,
    }
    exec(init_src, ns)
    code_high, code_ik, code_rec = (compile(s, n, "exec") for s, n in
                                    ((high_src, "high"), (ik_src, "ik"), (rec_src, "rec")))

    def step(both: bool, fwd: float = 1.0, fresh: bool = True, ready: bool = True):
        if fresh and ready:
            tvuer.last_motion_time = clock.now        # Quest からこの周期にデータが届いた
        ns["tele_data"] = make_tele(both, fwd, ready)
        exec(code_high, ns)
        exec(code_ik, ns)
        exec(code_rec, ns)
        clock.now += DT
        return loco.moves[-1], arm.sent_q.copy(), list(ns["current_body_action"])

    fails = []

    def check(cond: bool, msg: str) -> None:
        print(f"  {'✓' if cond else '✗'} {msg}")
        if not cond:
            fails.append(msg)

    # 7. 最初のデータが届く前は止めない（xr-stale-stop が当たっていれば意味がある）
    for _ in range(int(1.0 / DT)):
        mv, q, rec = step(False, fwd=0.0, ready=False)
    check(not ns["stick_paused"], "最初のデータが届く前は一時停止しない")

    # 1. 0.4 秒押して離す → 切り替わらない
    for _ in range(int(0.4 / DT)):
        mv, q, rec = step(True)
    mv, q, rec = step(False)
    check(not ns["stick_paused"], "0.4 秒の押し込みでは一時停止しない")
    check(abs(mv[0] - 0.3) < 1e-9, f"通常時は前に 0.3 m/s を送る（{mv[0]:.2f}）")
    check(rec == list(mv), "記録の歩きの指令＝送った値（通常時）")

    # 2. 0.6 秒押し続ける → 一時停止
    arm.measured = np.linspace(-0.3, 0.3, 14)          # 止めた瞬間の実測
    for _ in range(int(0.6 / DT)):
        mv, q, rec = step(True)
    check(ns["stick_paused"], "0.6 秒押し続けると一時停止する")
    check(mv == (0.0, 0.0, 0.0), "一時停止中は歩きの指令が 0")
    check(np.allclose(q, np.linspace(-0.3, 0.3, 14)), "腕は止めた瞬間の実測の角度を送る（IK の解ではない）")
    check(rec == [0.0, 0.0, 0.0], "記録の歩きの指令も 0")

    # 3. さらに 2 秒押し続ける → 切り替わらない
    for _ in range(int(2.0 / DT)):
        mv, q, rec = step(True)
    check(ns["stick_paused"], "押し続けても再び切り替わらない")

    # 4. 離して扉に押されても腕は固定のまま
    arm.measured = np.linspace(-0.25, 0.35, 14)
    for _ in range(10):
        mv, q, rec = step(False)
    check(np.allclose(q, np.linspace(-0.3, 0.3, 14)), "離しても一時停止中は止めた角度のまま")
    check(not loco.damp_called and not arm.go_home_called, "脱力も腕の帰還も呼ばれない")

    # 5. 再開 → 2 rad/s に絞り、2 秒後に 30 rad/s
    for _ in range(int(0.6 / DT)):
        mv, q, rec = step(True)
    check(not ns["stick_paused"], "もう一度 0.6 秒押すと再開する")
    check(arm.velocity_limit == 2.0, "再開直後は腕を 2 rad/s に絞る")
    check(np.allclose(q, 0.9), "再開後は IK の解（操作者の手）に追従する")
    for _ in range(int(2.2 / DT)):
        mv, q, rec = step(False)
    check(arm.velocity_limit == 30.0, "2 秒後に 30 rad/s へ戻す")
    check(abs(mv[0] - 0.3) < 1e-9 and rec == list(mv), "再開後は歩きの指令が戻り、記録とも一致")

    if has_stale:
        print("  ― Quest の途切れ（xr-stale-stop）―")
        # 8. 左スティックを前に倒したまま Quest が途切れる
        arm.measured = np.linspace(0.2, 0.5, 14)
        for _ in range(int(0.2 / DT)):
            mv, q, rec = step(False, fwd=1.0, fresh=False)
        check(not ns["stick_paused"], "0.2 秒の途切れでは止めない")
        for _ in range(int(0.3 / DT)):
            mv, q, rec = step(False, fwd=1.0, fresh=False)
        check(ns["stick_paused"], "0.3 秒を超えて途切れると一時停止する")
        check(mv == (0.0, 0.0, 0.0) and rec == [0.0, 0.0, 0.0],
              "最後のスティックが前のままでも歩かない（記録も 0）")
        check(np.allclose(q, np.linspace(0.2, 0.5, 14)), "腕は止めた瞬間の実測の角度")

        # 9. 途切れている間、両スティックが押された古い値のままでも再開しない
        for _ in range(int(1.0 / DT)):
            mv, q, rec = step(True, fresh=False)
        check(ns["stick_paused"], "途切れている間は古い押し込みで再開しない")
        # 届き始めたが、押したまま → 再開しない（一度離す必要がある）
        for _ in range(int(1.0 / DT)):
            mv, q, rec = step(True, fresh=True)
        check(ns["stick_paused"], "届き始めても押したままでは再開しない")
        mv, q, rec = step(False, fresh=True)
        for _ in range(int(0.6 / DT)):
            mv, q, rec = step(True, fresh=True)
        check(not ns["stick_paused"], "一度離して 0.6 秒押し直すと再開する")
        check_televuer(args_cli.televuer, check)
        if "PATCH(xr-stale-log)" in text:
            gaps = [float(m.split("データが ")[1].split(" 秒")[0]) for m in infos if "[xr-gap]" in m]
            check(len(gaps) >= 1 and max(gaps) > 0.3,
                  f"途切れの長さがログに残る（[xr-gap] {', '.join(f'{g:.2f}' for g in gaps)} 秒）")
    else:
        print("  （xr-stale-stop は当たっていないので途切れの確かめは省く）")

    print(f"\n[自己テスト] {'合格' if not fails else f'不合格 {len(fails)} 件'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
