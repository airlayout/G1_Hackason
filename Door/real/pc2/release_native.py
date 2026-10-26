"""純正の制御（ai などのモーションサービス）を SDK で外して、door_run_real.py を起こせる状態にする（2026-10-23）。

なぜ: 機体を再起動した後しばらくは、リモコンが届かず L2+B・L2+R2 が効かないことがある（10-16・10-23）。そのとき door_run_real.py は
「純正の歩行が動いている」と言って起動しない。10-23 は Damp（脱力）の要求は応答が無かったが、ReleaseMode で外れた。

⚠️ 外すと機体は脱力する。必ずウィンチで吊り、体重を綱が受けていることを人が確かめてから --yes を付けて起こす。
使い方（PC2）: python3 release_native.py --yes
"""
import argparse
import sys
import time

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yes", action="store_true", help="ウィンチで吊ってあることを確かめた")
    ap.add_argument("--iface", default="eth0")
    a = ap.parse_args()
    if not a.yes:
        print("[release] ⚠️ 外すと機体は脱力する。ウィンチで吊ってから --yes を付けて起こす")
        return 2
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    ChannelFactoryInitialize(0, a.iface)
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    loco = LocoClient()
    loco.SetTimeout(5.0)
    loco.Init()
    print("[release] 今の制御:", msc.CheckMode(), flush=True)
    print("[release] Damp（応答が無いことがある）:", loco.Damp(), flush=True)
    time.sleep(1.0)
    for k in range(5):
        code, res = msc.CheckMode()
        name = (res or {}).get("name", "") if code == 0 else None
        if code == 0 and not name:
            print("[release] 純正の制御が外れた。door_start.sh で起こせる", flush=True)
            return 0
        print(f"[release] まだ '{name}'（code {code}）: ReleaseMode", msc.ReleaseMode(), flush=True)
        time.sleep(2.0)
    print("[release] ⛔ 外れなかった。リモコン（L2+B → L2+R2）を試す", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
