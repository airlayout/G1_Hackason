#!/usr/bin/env python3
"""G1 との疎通と機体構成を確認する（読み取り専用 / ロボットは動きません）。

テレオペを起動する前に、これで「有線が通っているか / 相手は誰か / 何DoF機か」を
確定させる。一切 publish しないので、これでロボットが動くことはない。

使い方:
    ./scripts/preflight.sh              # ラッパー経由（設定を自動で読む）
    python3 tools/preflight.py --iface enp0s31f6

    --iface     有線インターフェース名（既定: config/g1.env の G1_WIRED_IFACE）
    --subnet    スキャンする /24（既定: 有線 NIC の実 IP から導出）
    --duration  DDS 受信サンプリング秒数 (default: 3.0)
    --skip-scan ping スイープを省略して DDS だけ見る
    --print-arm 判定結果（G1_23 / G1_29）だけを出力する（スクリプトから使う用）
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

# Unitree G1 の既定構成。個体によって違うことがあるので「既定」と明記する。
KNOWN_HOSTS = {
    161: "制御演算ユニット (既定)",
    164: "開発演算ユニット PC2 (既定)",
}

# unitree_hg の LowState_ モータ配列インデックス（robot_arm.py の JointIndex と一致）
WAIST_JOINTS = [(12, "WaistYaw"), (13, "WaistRoll"), (14, "WaistPitch")]
LEFT_ARM = [(15, "ShoulderPitch"), (16, "ShoulderRoll"), (17, "ShoulderYaw"),
            (18, "Elbow"), (19, "WristRoll"), (20, "WristPitch"), (21, "WristYaw")]
RIGHT_ARM = [(22, "ShoulderPitch"), (23, "ShoulderRoll"), (24, "ShoulderYaw"),
             (25, "Elbow"), (26, "WristRoll"), (27, "WristPitch"), (28, "WristYaw")]


def default_iface():
    """config/g1.env の G1_WIRED_IFACE を既定値として使う。"""
    env = Path(__file__).resolve().parent.parent / "config" / "g1.env"
    if env.exists():
        for line in env.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith("G1_WIRED_IFACE=") and not line.startswith("#"):
                value = line.split("=", 1)[1].strip()
                if value:
                    return value
    return "enp0s31f6"


def section(title):
    print(f"\n\033[1m=== {title} ===\033[0m")


def ok(msg):
    print(f"  \033[32m[OK]\033[0m   {msg}")


def warn(msg):
    print(f"  \033[33m[注意]\033[0m {msg}")


def ng(msg):
    print(f"  \033[31m[NG]\033[0m   {msg}")


def host_ipv4(iface):
    """インターフェースに実際に付いている IPv4 アドレスを返す（無ければ None）。"""
    out = subprocess.run(["ip", "-4", "-br", "addr", "show", iface],
                         capture_output=True, text=True).stdout
    for token in out.split():
        if "/" in token and token.count(".") == 3:
            return token.split("/")[0]
    return None


def check_link(iface):
    """有線リンクとホストIPを確認。リンクが上がっていれば True。"""
    section(f"1. 有線インターフェース {iface}")

    operstate = Path(f"/sys/class/net/{iface}/operstate")
    if not operstate.exists():
        ng(f"{iface} が存在しない。`ip -br addr` で正しい名前を確認すること。")
        return False

    state = operstate.read_text().strip()
    addrs = subprocess.run(["ip", "-4", "-br", "addr", "show", iface],
                           capture_output=True, text=True).stdout.strip()
    print(f"  link state : {state}")
    print(f"  addr       : {addrs or '(なし)'}")

    if state != "up":
        ng("リンクが上がっていない。RJ45 の抜き差し / G1 の起動完了待ち。")
        return False
    if "192.168.123." not in addrs:
        ng("192.168.123.x の静的IP が付いていない。docs/02-network.md の手順で設定する。\n"
           "     NetworkManager 管理下なら nmcli で（ip addr add では消される）:\n"
           "       sudo nmcli con up 'Wired connection 1'\n"
           "     （`ip addr add` では NetworkManager に消されるので不可）")
        return False

    if Path(f"/sys/class/net/{iface}/wireless").exists() or \
       Path(f"/sys/class/net/{iface}/phy80211").exists():
        warn("このインターフェースは無線です。読み取りは動きますが、受信レートや\n"
             "     取りこぼしが有線より不安定になります。ロボットを動かす用途では\n"
             "     有線を使ってください（docs/02-network.md の「なぜ有線か」）。")

    ok("リンク UP・静的IP あり")
    return True


def scan_subnet(iface, subnet, self_ip=None):
    """/24 を並列 ping して生きているホストを列挙する（root 不要）。"""
    section(f"2. {subnet}.0/24 スキャン")
    print("  ping スイープ中 ... ", end="", flush=True)

    procs = {
        i: subprocess.Popen(["ping", "-c", "1", "-W", "1", "-I", iface, f"{subnet}.{i}"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for i in range(1, 255)
    }
    alive = sorted(i for i, p in procs.items() if p.wait() == 0)
    print("完了")

    # ARP テーブルから MAC も引く（応答が無くても ARP は返る機器があるため）
    neigh = subprocess.run(["ip", "neigh", "show", "dev", iface],
                           capture_output=True, text=True).stdout
    macs = {}
    for line in neigh.splitlines():
        parts = line.split()
        if len(parts) >= 5 and parts[0].startswith(subnet) and "lladdr" in parts:
            macs[int(parts[0].rsplit(".", 1)[1])] = parts[parts.index("lladdr") + 1]

    found = sorted(set(alive) | set(macs))
    if not found:
        ng("応答するホストが1つも無い。G1 の起動完了待ち / ケーブル確認。")
        return []

    for i in found:
        if self_ip and f"{subnet}.{i}" == self_ip:
            label = "このPC (ホスト)"
        else:
            label = KNOWN_HOSTS.get(i, "\033[33m未知のホスト\033[0m")
        mac = f"  mac={macs[i]}" if i in macs else ""
        mark = "ping応答" if i in alive else "ARPのみ"
        print(f"  {subnet}.{i:<4} {mark}  {label}{mac}")

    self_last = int(self_ip.rsplit(".", 1)[1]) if self_ip else None
    robots = [i for i in found if i != self_last]
    if 161 in robots:
        ok("制御演算ユニット .161 を確認（既定構成どおり）")
    elif robots:
        warn(f"既定の .161 が居ない。この個体の制御ユニットは {subnet}.{robots[0]} の可能性。\n"
             "     → DDS が通れば teleop 自体は IP 非依存なので影響なし。")
    return found


def check_dds(iface, duration):
    """rt/lowstate を購読して受信状況と DoF 構成を判定する（購読のみ・送信なし）。"""
    section("3. DDS 疎通 + 機体構成の判定")

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as e:
        ng(f"unitree_sdk2py を import できない ({e})。\n"
           "     conda env `tv` が有効か確認: conda activate tv")
        return None

    samples = []
    ChannelFactoryInitialize(0, iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda msg: samples.append(msg), 10)

    print(f"  rt/lowstate を {duration:.0f} 秒間 購読中 ... ", end="", flush=True)
    time.sleep(duration)
    print("完了")

    if not samples:
        ng("lowstate を1件も受信できない。原因の切り分け:\n"
           "     - ロボットの起動が完了していない（起動直後は数十秒かかる）\n"
           "     - インターフェース名の指定違い（--iface）\n"
           "     - HG系ではない機体（unitree_go IDL の機体）\n"
           "     ※ 仕様書 §7 のとおり、DDS 初回ハンドシェイクは自己判断で回避策を試さず報告すること")
        return None

    latest = samples[-1]
    hz = len(samples) / duration
    ok(f"受信 {len(samples)} 件 / {hz:.0f} Hz  tick={latest.tick}")
    rpy = latest.imu_state.rpy
    print(f"  IMU rpy    : ({rpy[0]:+.3f}, {rpy[1]:+.3f}, {rpy[2]:+.3f}) rad")

    def is_live(idx):
        """実装されているモータは電圧・温度を返す。欠番スロットは全ゼロ。"""
        m = latest.motor_state[idx]
        return m.vol > 0.1 or m.temperature[0] > 0

    def report(name, joints):
        live = [n for i, n in joints if is_live(i)]
        dead = [n for i, n in joints if not is_live(i)]
        print(f"  {name:<8}: {len(live)} 軸稼働  {live}")
        if dead:
            print(f"            (無効: {dead})")
        return len(live)

    n_waist = report("腰", WAIST_JOINTS)
    n_left = report("左腕", LEFT_ARM)
    n_right = report("右腕", RIGHT_ARM)

    if n_left == n_right == 7 and n_waist == 3:
        arm = "G1_29"
    elif n_left == n_right == 5 and n_waist == 1:
        arm = "G1_23"
    else:
        arm = None

    if arm:
        ok(f"機体構成の判定: \033[1m--arm={arm}\033[0m "
           f"(腕 {n_left}+{n_right} 軸 / 腰 {n_waist} 軸)")
    else:
        warn(f"既知の構成に一致しない（腕 {n_left}+{n_right} 軸 / 腰 {n_waist} 軸）。\n"
             "     モータが未通電だと全ゼロに見える。ロボットの電源とモータ有効化を確認し、\n"
             "     それでも一致しなければ手首の軸数を目視（1軸=G1_23 / 3軸=G1_29）。")
    return arm


def main():
    p = argparse.ArgumentParser(description="G1 事前確認（読み取り専用）")
    p.add_argument("--iface", default=None)
    p.add_argument("--print-arm", action="store_true",
                   help="判定結果だけを出力（スクリプトから呼ぶ用）")
    p.add_argument("--subnet", default=None,
                   help="スキャンする /24（既定: 有線 NIC の実 IP から導出）")
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("--skip-scan", action="store_true")
    args = p.parse_args()
    if args.iface is None:
        args.iface = default_iface()

    if args.print_arm:
        # 静かに判定だけして機体名を返す
        import contextlib, io
        with contextlib.redirect_stdout(io.StringIO()):
            arm = check_dds(args.iface, args.duration)
        print(arm or "unknown")
        sys.exit(0 if arm else 1)

    print("\033[1mG1 事前確認 — 読み取り専用（ロボットは動きません）\033[0m")

    if not check_link(args.iface):
        sys.exit(1)

    self_ip = host_ipv4(args.iface)
    subnet = args.subnet
    if subnet is None and self_ip:
        subnet = self_ip.rsplit(".", 1)[0]   # 実測 IP から /24 を導く
    subnet = subnet or "192.168.123"

    if not args.skip_scan:
        scan_subnet(args.iface, subnet, self_ip)

    arm = check_dds(args.iface, args.duration)

    section("4. 次のステップ")
    if arm is None:
        print("  機体を判定できませんでした。上の [NG]/[注意] を解消してから再実行してください。")
        sys.exit(1)
    print(f"""
  機体は {arm} と判定されました。config/g1.env に書いておくと毎回の判定を省けます:

      G1_ARM={arm}

  テレオペを始める:
      ./scripts/teleop.sh                  （記録なし）
      ./scripts/teleop.sh --record         （動きを記録する）

  ⚠️ 起動直後に両腕がゼロ姿勢へ動きます。可動範囲をクリアにしてから実行してください。""")


if __name__ == "__main__":
    main()
