#!/usr/bin/env python3
"""2026-11-04: 遠隔操作・再生を始めてよい機体の状態かを、形（立った形・座位）ごとに確かめる。PC2 の上で動く。

teleop_start.sh・scripts/pc2_teleop.sh（start・replay・check）から呼ばれる。読むだけ（機体には何も送らない）。

    python3 form_check.py --form standing              # 立った形（--motion。足あり・足なし）: 純正が通常運転 500/501 なら 0
    python3 form_check.py --form seated                # 座位の遠隔操作: 純正がダンピング（FSM 1）か、止まっていてモータが有効なら 0
    python3 form_check.py --form seated --for replay   # 座位の再生: 純正が止まっていてモータが有効なら 0（ダンピングでは断る）
    python3 form_check.py --form both                  # check 用: 全部の判定を出す（いつも 0）

11-04 朝、純正がゼロトルク（FSM 0）のまま立った形で起こし、go しても腕が動かなかった。キットの mode_check.py は
FSM 0 でも「腕を動かせる状態です」と出す（モータの mode しか見ない）ので、ここでは純正の FSM で決める。
純正が動いているかは、歩きの RPC（GetFsmId）と切り替えの RPC（CheckMode）の 2 つで見る。どちらかが答えれば動いている。
「止まっている」は、切り替えの RPC が空の名前を答えたか、保持係が保っている（純正が動いていれば黙る）ときだけとする。
両方の RPC が答えないだけでは「止まっている」としない（PC2 が混んで答えが遅れただけかもしれず、立っている機体の純正を
止めて崩す。11-04 の見直しの指摘）。
座位の再生は純正を止めない。止めてから再生の最初の指令までの数秒、誰も姿勢を保たないため（同じ指摘）。
座位の遠隔操作を 1 回起こして quit した後（純正が止まり、保持係が保っている）に流す。
"""

import argparse
import contextlib
import os
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
# 配った先（~/teleop_tools/）では同じフォルダ、リポジトリではキットの tools/ に loco_replay.py がある
for p in (HERE, HERE.parents[1] / "vendor" / "g1-starter-kit" / "tools"):
    if (p / "loco_replay.py").exists():
        sys.path.insert(0, str(p))
        break
sys.path.insert(0, str(HERE))
import loco_replay  # noqa: E402
import seat_hold  # noqa: E402

DAMP_FSM, ZERO_TORQUE_FSM = 1, 0
FSM_ATTEMPTS = 3
SWITCHER_TIMEOUT_S, SWITCHER_ATTEMPTS = 1.0, 2
STATE_WAIT_S = 2.0
Verdict = Tuple[bool, str]


def stock_running(fsm: Optional[int], name: Optional[str]) -> bool:
    """純正が動いているか（歩きの RPC か切り替えの RPC のどちらかが答えた）。"""
    return fsm is not None or bool(name)


def judge_standing(fsm: Optional[int], name: Optional[str], holder_pid: Optional[int],
                   state_seen: bool = True) -> Verdict:
    """立った形（--motion: 純正が立たせたまま腕を借りる）を始めてよいか。"""
    if holder_pid is not None:
        return False, f"座位の保持係が動いている（PID {holder_pid}）。座位を終えるなら pc2_teleop.sh release"
    running = stock_running(fsm, name)
    if not running and not state_seen:
        return False, "機体の状態（rt/lowstate）が来ない（機体の電源・PC2 の eth0）"
    if not running:
        return False, "純正が答えない（座位の後で止まっている？）。立たせるなら pc2_teleop.sh restore で純正を戻すか、電源を入れ直す"
    if fsm in loco_replay.WALKING_FSM_IDS:
        return True, f"純正が通常運転（FSM {fsm}）"
    if fsm is None:
        return False, "純正は動いているが FSM が読めない"
    if fsm == ZERO_TORQUE_FSM:
        return False, "純正がゼロトルク（FSM 0）。--motion では腕が動かない。立たせてリモコンで通常運転（R1+X）に"
    return False, f"純正が通常運転ではない（FSM {fsm}）。--motion では腕が動かない。リモコンで R1+X"


def judge_seated(fsm: Optional[int], name: Optional[str], modes: Optional[np.ndarray],
                 holder: Optional[str], for_replay: bool = False) -> Verdict:
    """座位（--motion なし: 純正を止めて rt/lowcmd で腕だけ。脚と腰は固める）を始めてよいか。

    holder は保持係の状態（seat_hold.holder_state()）。遠隔操作は起動のときに自分で純正を止める。再生は止めない。
    """
    if stock_running(fsm, name):
        if fsm == DAMP_FSM and not for_replay:
            return True, "純正がダンピング（FSM 1）。遠隔操作が起動のときに純正を止める"
        if fsm == DAMP_FSM:
            return False, ("純正がダンピング（FSM 1）のまま。座位の再生は、座位の遠隔操作を 1 回起こして quit した後"
                           "（純正が止まり、保持係が保っている）に流す")
        if fsm is None:
            return False, "純正は動いているが FSM が読めない"
        if fsm == ZERO_TORQUE_FSM:
            return False, "純正がゼロトルク（FSM 0）でモータが無効。座らせたまま、リモコンの L2+B でダンピングに"
        return False, (f"純正が FSM {fsm}（立っている・動いている）。この形は純正を止めるので崩れる。"
                       "座らせてから L2+B でダンピングに")
    if modes is None:
        return False, "機体の状態（rt/lowstate）が来ない（機体の電源・PC2 の eth0）"
    if name is None and holder != "hold":
        return False, ("純正が止まっているか確かめられない（RPC が答えず、保持係も保っていない）。"
                       "立っているかもしれないので止めない。続けるなら pc2_teleop.sh restore で純正を戻し、ダンピングから")
    off = int(np.sum(modes[:seat_hold.N_BODY] != 1))
    if off:
        return False, (f"純正は止まっているが、モータが {off}/{seat_hold.N_BODY} 関節で無効（ゼロトルク）。"
                       "リモコンの L2+B でダンピングに（効かなければ pc2_teleop.sh restore で純正を戻す）")
    return True, "純正は止まっていて、モータは有効" + ("（保持係が保っている）" if holder == "hold" else "")


def read_modes(wait_s: float = STATE_WAIT_S) -> Optional[np.ndarray]:
    """rt/lowstate を 1 件読んで、実関節のモータの mode（1 が有効）を返す。来なければ None。

    聞き役（listener）を付けずに 1 件だけ取りに行く。付けると、閉じた直後に届いた 1 件で SDK が
    「[Reader] take sample error」を出す（状態は 500 件/秒で届き続ける。11-04 の PC2 の check で出た）。
    """
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()
    msg = sub.Read(wait_s)                        # 来なければ SDK が print して None（print は main で捨てる）
    if msg is None or not hasattr(msg, "motor_state"):
        return None
    return seat_hold.body_from_state(msg).mode


def stock_state() -> Tuple[Optional[int], Optional[str]]:
    """純正の (FSM, 切り替えの名前)。読めなければ None。"""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    fsm = None
    client = loco_replay.make_loco_client()
    for _ in range(FSM_ATTEMPTS):
        fsm = loco_replay.fsm_id(client)
        if fsm is not None:
            break
    name = None
    msc = MotionSwitcherClient()
    msc.SetTimeout(SWITCHER_TIMEOUT_S)
    msc.Init()
    for _ in range(SWITCHER_ATTEMPTS):
        try:
            status, result = msc.CheckMode()
            if status == 0 and result is not None:
                name = result.get("name") or ""
                break
        except Exception:  # noqa: BLE001  散発の失敗は試し直す（読めなければ None）
            pass
    return fsm, name


def main() -> int:
    p = argparse.ArgumentParser(description="遠隔操作・再生を始めてよい機体の状態かを形ごとに確かめる")
    p.add_argument("--form", choices=["standing", "seated", "both"], required=True)
    p.add_argument("--for", dest="purpose", choices=["teleop", "replay"], default="teleop",
                   help="座位で何を始めるか（再生は純正を止めないので、止まっている状態でだけ通す）")
    p.add_argument("--iface", default="eth0")
    a = p.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    # SDK の print（RPC が答えないたびの「[ClientStub] send request error」など）は捨てる。座位の後は純正が止まって
    # 必ず出て、誤りに見える。答えたかどうかは下の 1 行と ✓/✗ に出す。例外はそのまま上がる
    with open(os.devnull, "w") as quiet, contextlib.redirect_stdout(quiet):
        ChannelFactoryInitialize(0, a.iface)
        modes = read_modes()
        fsm, name = stock_state()
    holder_pid, holder = seat_hold.running_pid(), seat_hold.holder_state()
    print(f"[form] 純正: {'動いている' if stock_running(fsm, name) else '答えない'}（FSM {fsm}・切り替え {name!r}）・"
          f"モータの有効 {'?' if modes is None else int(np.sum(modes == 1))}/{seat_hold.N_BODY}・"
          f"保持係 {f'PID {holder_pid}（{holder}）' if holder_pid else 'なし'}")

    verdicts = {}
    if a.form in ("standing", "both"):
        verdicts["立った形（足あり・足なし）"] = judge_standing(fsm, name, holder_pid, modes is not None)
    if a.form == "both" or (a.form == "seated" and a.purpose == "teleop"):
        verdicts["座位の遠隔操作（start --seated）"] = judge_seated(fsm, name, modes, holder)
    if a.form == "both" or (a.form == "seated" and a.purpose == "replay"):
        verdicts["座位の再生（replay --seated）"] = judge_seated(fsm, name, modes, holder, for_replay=True)
    for label, (ok, text) in verdicts.items():
        print(f"[form] {'✓' if ok else '✗'} {label}: {text}")
    return 0 if a.form == "both" or all(v[0] for v in verdicts.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
