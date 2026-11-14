"""腕の定型動作を外から 1 行で呼ぶ入口。

Python から:
    from actions.api import perform
    perform("salute")                                   # dry-run（何も送らない）
    perform("point", execute=True, confirm=False)       # 実機に送る（人が G1 のそばでリモコンを持っているとき）
    perform("salute", path="sim", execute=True, confirm=False)   # sim（MuJoCo）

戻り値は終了コード（0 = 完了、130 = 中止、2 = 目標を拒否、3/4/5 = 失敗。actions/runner.py と同じ）。

実機で安全を確かめた動作（VERIFIED_ACTIONS）だけ呼べる。それ以外は allow_unverified=True を付けたときだけ。
execute の既定は False。実機に送るのは、人がそばにいて明示したときだけ（緊急時はリモコンの L2+B）。
"""

from __future__ import annotations

import os

from .poses import ACTIONS
from .runner import main

# 実機で動かして安全を確かめた動作（2026-11-14）。point_level は見送り（体の傾きが大きい）
VERIFIED_ACTIONS: tuple[str, ...] = ("salute", "point")


def perform(name: str, *, path: str = "arm_sdk", execute: bool = False, confirm: bool = True,
            network_interface: str | None = None, allow_unverified: bool = False) -> int:
    """name の動作を実行する。execute=False なら dry-run。confirm=True なら各段階で Enter を待つ（execute のときだけ効く）。"""
    if name not in ACTIONS:
        raise ValueError(f"動作が無い: {name}（{', '.join(sorted(ACTIONS))}）")
    if name not in VERIFIED_ACTIONS and not allow_unverified:
        raise ValueError(f"{name} は実機で確かめていない動作。使うなら allow_unverified=True（確かめた動作: {', '.join(VERIFIED_ACTIONS)}）")
    if path not in ("sim", "arm_sdk"):
        raise ValueError(f"path は sim か arm_sdk: {path}")
    argv = ["--action", name, "--path", path]
    nic = network_interface or os.environ.get("G1_NIC")
    if nic and path == "arm_sdk":
        argv += ["--network-interface", nic]
    if execute:
        argv.append("--execute")
        if not confirm:
            argv.append("--no-confirm")
    return main(argv)


def cli(argv: list[str] | None = None) -> int:
    import argparse

    p = argparse.ArgumentParser(description="敬礼・指さしを 1 コマンドで動かす（既定は dry-run）")
    p.add_argument("action", choices=sorted(ACTIONS), help=f"動作（実機で確かめた: {', '.join(VERIFIED_ACTIONS)}）")
    p.add_argument("--sim", action="store_true", help="sim（MuJoCo）で動かす（既定は実機の arm_sdk）")
    p.add_argument("--execute", action="store_true", help="実際に送る（付けなければ dry-run。sim でも付ける）")
    p.add_argument("--no-confirm", action="store_true", help="各段階の Enter 待ちを省く")
    p.add_argument("--nic", help="G1 につないでいる NIC（既定は環境変数 G1_NIC、無ければ configs/arm.yaml）")
    p.add_argument("--allow-unverified", action="store_true", help="実機で確かめていない動作も許す")
    a = p.parse_args(argv)
    try:
        return perform(a.action, path="sim" if a.sim else "arm_sdk", execute=a.execute, confirm=not a.no_confirm,
                       network_interface=a.nic, allow_unverified=a.allow_unverified)
    except ValueError as e:
        print(f"[action] {e}")
        return 2
