#!/usr/bin/env python3
"""Livox LiDAR の機種・IP・シリアルを自動検出する（読み取り専用）。

UDP 56000 番へ探索要求（cmd_id=0x0000）をブロードキャストし、LiDAR が返す
応答から機種・IP・シリアルを読む。Livox-SDK2 の DeviceManager::Detection() と
同じ手順で、**問い合わせるだけ**なので LiDAR にもロボットにも書き込まない。

なぜこれが必要か:
    G1 に載る Livox は **Mid-360** と **Mid-360S** があり、見た目では区別できない。
    そして **ドライバの設定ファイル形式が両者で違う**。間違った方を使うと
    ドライバは「初期化成功」と出したまま LiDAR を検出せず、トピックも作らず、
    エラーも出さない。原因が非常に分かりにくいので、先にこれで確定させる。

使い方:
    # 何が繋がっているか調べる
    python3 tools/lidar_probe.py

    # 検出結果から設定ファイルを生成する
    python3 tools/lidar_probe.py --write-config

    # 点群の向き（上下逆さま搭載かどうか）を実データで判定する
    #   ※ ROS 環境を source し、ドライバを起動した状態で実行
    python3 tools/lidar_probe.py --orientation

依存: 標準ライブラリのみ（--orientation のみ ROS 2 と numpy が必要）
"""

import argparse
import ipaddress
import json
import socket
import struct
import sys
import time
import zlib
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DISCOVERY_PORT = 56000

# Livox-SDK2 sdk_core/comm/{define.h,sdk_protocol.h} の値と同じ
SOF = 0xAA
SDK_VERSION = 0
CMD_ID_LIDAR_SEARCH = 0x0000
CMD_TYPE_CMD = 0
CMD_TYPE_ACK = 1
SENDER_HOST = 0
SENDER_LIDAR = 1
HEADER_LEN = 24          # sof..crc32_d
CRC16_COVER = 18         # crc16 は先頭 18 バイト（rsvd まで）を対象にする
ACK_DATA_LEN = 24        # 機器情報の本体長
ACK_TOTAL_LEN = HEADER_LEN + ACK_DATA_LEN

# Livox-SDK2 include/livox_lidar_def.h の LivoxLidarDeviceType と同じ対応。
# 設定ファイル形式が違うものは CONFIG_STYLE で分けている。
DEVICE_TYPES = {
    0:  ("Hub", None),
    1:  ("Mid-40", None),
    2:  ("Tele-15", None),
    3:  ("Horizon", None),
    6:  ("Mid-70", None),
    7:  ("Avia", None),
    9:  ("Mid-360", "MID360"),
    10: ("Industrial HAP", "HAP"),
    15: ("HAP", "HAP"),
    16: ("PA", None),
    35: ("Mid-360S", "MID360s"),
    40: ("Avia2", "AVIA2"),
}


def crc16_ccitt_false(data):
    """CRC-16/CCITT-FALSE。Livox の crc16_h と同じ（poly=0x1021 init=0xffff）。

    FastCRC16::ccitt() の実装コメントにある
    poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1
    と同一。
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc32_livox(data):
    """Livox の crc32_d。FastCRC32::crc32() は zlib と同じ多項式・初期値・反転。"""
    return zlib.crc32(data) & 0xFFFFFFFF


def build_discovery_request(seq):
    """LiDAR 探索要求（cmd_id=0x0000）を組み立てる。データ部は空。

    Livox-SDK2 の DeviceManager::Detection() と同じ内容。ホストがこれを
    255.255.255.255:56000 へブロードキャストし、LiDAR が ACK を返す。
    これを送らないと、LiDAR は自分から名乗らない。
    """
    header = struct.pack(
        "<BBHIHBB6s",
        SOF, SDK_VERSION, HEADER_LEN, seq & 0xFFFFFFFF,
        CMD_ID_LIDAR_SEARCH, CMD_TYPE_CMD, SENDER_HOST, b"\x00" * 6)
    assert len(header) == CRC16_COVER, len(header)
    # データ長 0 のときは crc32 を 0 にする（SDK の Pack と同じ）
    return header + struct.pack("<HI", crc16_ccitt_false(header), 0)


def decode_broadcast(data):
    """Livox の検出ブロードキャストを解読して dict を返す。読めなければ None。

    SDK2 の制御フレーム構造:
        0      sof (0xAA)
        1      version
        2-3    length (uint16 LE)
        4-7    seq_num
        8-9    cmd_id
        10     cmd_type
        11     sender_type
        12-17  reserved
        18-19  crc16
        20-23  crc32
        24-    データ本体
    データ本体（機器情報）:
        24     ret_code
        25     dev_type
        26-41  シリアル番号（16 バイト・NUL 終端）
        42-45  LiDAR の IP アドレス
        46-47  コマンドポート (uint16 LE)
    """
    if len(data) != ACK_TOTAL_LEN or data[0] != SOF:
        return None
    length = struct.unpack_from("<H", data, 2)[0]
    if length != len(data):
        return None

    cmd_id, cmd_type, sender_type = struct.unpack_from("<HBB", data, 8)
    if cmd_id != CMD_ID_LIDAR_SEARCH:
        return None
    if cmd_type != CMD_TYPE_ACK or sender_type != SENDER_LIDAR:
        return None          # ホスト自身が出した探索要求などは弾く

    crc16_h, crc32_d = struct.unpack_from("<HI", data, 18)
    if crc16_h != crc16_ccitt_false(data[:CRC16_COVER]):
        return None
    body = data[HEADER_LEN:]
    if len(body) != ACK_DATA_LEN or crc32_d != crc32_livox(body):
        return None

    ret_code = body[0]
    if ret_code != 0:
        return None          # エラー応答は採用しない

    dev_type = body[1]
    sn = body[2:18].split(b"\x00")[0].decode("ascii", "replace")
    ip = ".".join(str(b) for b in body[18:22])
    cmd_port = struct.unpack_from("<H", body, 22)[0]
    try:
        ipaddress.IPv4Address(ip)
    except ValueError:
        return None
    name, config_style = DEVICE_TYPES.get(dev_type, (f"未知 (dev_type={dev_type})", None))

    return {"dev_type": dev_type, "model": name, "config_style": config_style,
            "sn": sn, "ip": ip, "cmd_port": cmd_port}


def probe(seconds, host_ip=None):
    """探索要求をブロードキャストしながら 56000 番で ACK を待ち、LiDAR 一覧を返す。

    ⚠️ 待ち受けるだけでは検出できない。LiDAR は接続先が未確定でも自発的には
    名乗らず、ホストからの探索要求（cmd_id=0x0000）に ACK で応答する方式のため。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.bind(("", DISCOVERY_PORT))
    except OSError as e:
        sys.exit(f"エラー: UDP {DISCOVERY_PORT} を開けません ({e})\n"
                 "  LiDAR ドライバが起動していると重複します。先に止めてから実行してください。")
    sock.settimeout(0.2)

    # 全体ブロードキャストに加え、有線側のサブネットブロードキャストにも送る
    targets = ["255.255.255.255"]
    if host_ip:
        try:
            net = ipaddress.IPv4Network(f"{host_ip}/24", strict=False)
            targets.append(str(net.broadcast_address))
        except ValueError:
            pass

    found, others, seq = {}, set(), 1
    deadline = time.monotonic() + seconds
    next_send = 0.0
    print(f"探索要求をブロードキャストしながら UDP {DISCOVERY_PORT} を "
          f"{seconds:.0f} 秒間待ち受けます ...")
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            request = build_discovery_request(seq)
            seq += 1
            for target in targets:
                try:
                    sock.sendto(request, (target, DISCOVERY_PORT))
                except OSError:
                    pass
            next_send = now + 1.0
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        info = decode_broadcast(data)
        if info:
            found[info["ip"]] = info
        elif addr[0] != host_ip:
            others.add(f"{addr[0]} ({len(data)}B)")
    sock.close()

    if others:
        print(f"  （LiDAR の応答以外も見えました: {', '.join(sorted(others))}）")
    return sorted(found.values(), key=lambda d: d["ip"])


def pick_device(devices, want_serial, want_ip):
    """使う LiDAR を 1 台に確定する。複数見つかったら明示指定を要求する。

    応答の順番はネットワークのタイミング次第なので、「最初に応答した機器」を
    黙って選ぶと、実行ごとに別の LiDAR 用の設定を作ってしまう。
    """
    if want_serial:
        hit = [d for d in devices if d["sn"] == want_serial]
        if not hit:
            sys.exit(f"エラー: シリアル {want_serial} の LiDAR は見つかりませんでした。")
        return hit[0]
    if want_ip:
        hit = [d for d in devices if d["ip"] == want_ip]
        if not hit:
            sys.exit(f"エラー: IP {want_ip} の LiDAR は見つかりませんでした。")
        return hit[0]
    if len(devices) == 1:
        return devices[0]
    print("\n  複数の LiDAR が見つかりました。どれを使うか指定してください:\n")
    for d in devices:
        print(f"    --lidar-ip {d['ip']}    （{d['model']} / SN {d['sn']}）")
    sys.exit("\n  --lidar-ip か --serial で 1 台を指定して、もう一度実行してください。")


def report(devices, host_ip):
    if not devices:
        print("""
  LiDAR が見つかりませんでした。考えられる原因:

   1. 有線が繋がっていない／ロボットの起動が完了していない
      → ip -br addr で リンクが up か、ping で機体に届くか確認

   2. LiDAR が既に他のホストに接続済みで、探索要求に応答しない
      → ロボットを再起動し、起動直後にもう一度これを実行する

   3. ドライバが起動していて 56000 を占有している
      → 先にドライバを止める（このツールと同じポートを使う）

   4. このPC が LiDAR と同じサブネット (192.168.123.0/24) にいない
      → tools/preflight.py で有線設定を確認
""")
        return None

    print(f"\n  {len(devices)} 台の LiDAR を検出しました\n")
    for d in devices:
        print(f"    機種       : {d['model']}  (dev_type={d['dev_type']})")
        print(f"    IP         : {d['ip']}")
        print(f"    シリアル   : {d['sn']}")
        print(f"    設定形式   : {d['config_style'] or '未対応'}")
        print()

    if len(devices) > 1:
        print("  ⚠️ 複数台あります。--write-config には --lidar-ip / --serial の指定が必要です。\n")
        return None

    primary = devices[0]
    if primary["config_style"] is None:
        print("  ⚠️ このキットが設定ファイルを用意していない機種です。"
              "livox_ros_driver2 の config/ にある雛形を手で用意してください。")
        return primary

    print(f"  この機種の設定形式は {primary['config_style']} です。")
    if host_ip:
        print(f"  LiDAR のデータ送信先（このPC）: {host_ip}")
    print("\n  --write-config を付けると config/livox/active.json を生成します:")
    print("      python3 tools/lidar_probe.py --write-config")
    return primary


def write_config(device, host_ip):
    """検出した機種に合う設定ファイルを config/livox/active.json として生成する。"""
    try:
        ipaddress.IPv4Address(host_ip)
    except ValueError:
        sys.exit(f"エラー: host IP の形式が不正です: {host_ip}")
    style = device["config_style"]
    template = REPO / "config" / "livox" / f"{style}_config.json.in"
    if not template.exists():
        sys.exit(f"エラー: 雛形がありません: {template}")

    cfg = json.loads(template.read_text(encoding="utf-8"))

    # 機種ごとにキー名も構造も違う。ここを間違えると LiDAR は無反応のまま。
    # 「知らないキーを拾う」推定はテンプレートにメタデータが増えると壊れるので、
    # 雛形の中から host_net_info を持つセクションを明示的に探す。
    sections = [k for k, v in cfg.items()
                if isinstance(v, dict) and "host_net_info" in v]
    if len(sections) != 1:
        sys.exit(f"エラー: 雛形 {template.name} の構造が想定と違います"
                 f"（host_net_info を持つセクション: {sections}）")
    section = sections[0]
    host = cfg[section]["host_net_info"]
    if isinstance(host, list):          # MID360s / AVIA2 系: 配列 + host_ip
        for h in host:
            h["host_ip"] = host_ip
    else:                               # MID360 系: オブジェクト + チャネル別 IP
        for key in list(host):
            if key.endswith("_ip") and host[key]:
                host[key] = host_ip
    cfg["lidar_configs"][0]["ip"] = device["ip"]
    cfg["_comment"] = [
        f"tools/lidar_probe.py が自動生成 (機種 {device['model']} / SN {device['sn']})。",
        "手で編集せず、環境が変わったら再生成すること。",
        f"host_ip は有線 NIC の実 IP ({host_ip}) と一致していないと点群が届かない。",
    ]

    # 一時ファイルに書いてから置き換える（途中で切れても壊れた設定を残さない）
    out = REPO / "config" / "livox" / "active.json"
    tmp = out.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(out)
    print(f"\n  生成しました: {out}")
    print(f"    機種 {device['model']} / LiDAR {device['ip']} / host {host_ip}")
    print("  これで ./scripts/lidar_view.sh が使えます。")


def check_orientation():
    """点群の z 分布から、LiDAR が上下逆さまに搭載されているかを判定する。

    G1 は機種や個体で搭載向きが違うことがある。床は最も点が集まる平面なので、
    それがセンサ座標の上側にあれば逆さま搭載と分かる。
    """
    try:
        import numpy as np
        import rclpy
        import sensor_msgs_py.point_cloud2 as pc2
        from sensor_msgs.msg import PointCloud2
    except ImportError as e:
        sys.exit(f"エラー: ROS 2 環境が有効でありません ({e})\n"
                 "  source /opt/ros/humble/setup.bash してから実行してください。")

    rclpy.init()
    node = rclpy.create_node("lidar_orientation")
    got = []

    def cb(msg):
        if got:
            return
        # 空の点群だと np.array([]) が 1 次元になり axis=1 の計算で落ちるため、
        # 必ず (N, 3) に整形してから扱う。inf/NaN も除外する。
        pts = np.asarray([[p[0], p[1], p[2]] for p in pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)],
            dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return
        # 未反射は (0,0,0) で返るので除外しないと分布が原点に張り付く
        got.append(pts[np.linalg.norm(pts, axis=1) > 0.3])

    node.create_subscription(PointCloud2, "/livox/lidar", cb, 10)
    print("/livox/lidar を待っています（ドライバを起動しておくこと）...")
    deadline = time.time() + 20
    while not got and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    if not got or len(got[0]) < 200:
        sys.exit("有効な点が足りません（判定には 200 点以上必要）。\n"
                 "  ./scripts/lidar_view.sh が動いていて点群が出ているか確認してください。")

    z = got[0][:, 2]
    hist, edges = np.histogram(z, bins=80)
    peak = float((edges[hist.argmax()] + edges[hist.argmax() + 1]) / 2)
    print(f"\n  点数 {len(z)}   z 中央値 {float(np.median(z)):+.2f} m")
    print(f"  最も点が集中する z = {peak:+.2f} m（床とみなす）")
    if peak > 0.3:
        print("  → 床がセンサより上にある = **上下逆さま搭載**")
        print("     config/g1.env は  G1_LIDAR_FLIP=true  が正しい")
    elif peak < -0.3:
        print("  → 床がセンサより下にある = 正立搭載")
        print("     config/g1.env は  G1_LIDAR_FLIP=false  が正しい")
    else:
        print("  → 判断できません（周囲が開けすぎ／壁が主体）。")
        print("     RViz で見て、天井と床が逆さまでないか目視で決めてください。")


def read_host_ip(iface_hint=None):
    """config/g1.env の G1_HOST_IP を読む。無ければ有線 NIC から推測する。"""
    env = REPO / "config" / "g1.env"
    if env.exists():
        for line in env.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith("G1_HOST_IP=") and not line.startswith("#"):
                value = line.split("=", 1)[1].strip()
                if value:
                    return value
    # 192.168.123.x を持つインターフェースを探す
    import subprocess
    out = subprocess.run(["ip", "-4", "-br", "addr"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        for token in line.split():
            if token.startswith("192.168.123.") and "/" in token:
                return token.split("/")[0]
    return None


def main():
    p = argparse.ArgumentParser(description="Livox LiDAR の機種・IP を自動検出")
    p.add_argument("--seconds", type=float, default=8.0, help="待ち受け秒数 (default: 8)")
    p.add_argument("--write-config", action="store_true",
                   help="検出結果から config/livox/active.json を生成する")
    p.add_argument("--host-ip", help="LiDAR のデータ送信先（既定: config/g1.env か有線NICから推測）")
    p.add_argument("--lidar-ip", help="複数台あるとき、使う LiDAR を IP で指定")
    p.add_argument("--serial", help="複数台あるとき、使う LiDAR をシリアルで指定")
    p.add_argument("--orientation", action="store_true",
                   help="点群の向き（上下逆さま搭載か）を実データで判定（ROS 環境と稼働中のドライバが必要）")
    args = p.parse_args()

    if args.orientation:
        check_orientation()
        return

    print("Livox LiDAR 検出 — 読み取り専用（LiDAR にもロボットにも書き込みません）\n")
    host_ip = args.host_ip or read_host_ip()
    if not host_ip:
        print("  ⚠️ 有線 NIC の IP が分かりません。先に tools/preflight.py を実行してください。\n")

    devices = probe(args.seconds, host_ip)
    primary = report(devices, host_ip)

    if args.write_config:
        if not devices:
            sys.exit(1)
        primary = pick_device(devices, args.serial, args.lidar_ip)
        if primary["config_style"] is None:
            sys.exit("エラー: この機種の設定雛形をこのキットは持っていません。")
        if not host_ip:
            sys.exit("エラー: --host-ip を指定するか config/g1.env の G1_HOST_IP を埋めてください。")
        write_config(primary, host_ip)


if __name__ == "__main__":
    main()
