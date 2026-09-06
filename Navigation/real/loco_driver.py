#!/usr/bin/env python3
"""受け取った速度指令を G1 の足に渡す。**PC2 の system python3.8 で動かす。**

## なぜ 2 プロセスに割れているのか

PC2 では **rclpy と unitree_sdk2py が同じ Python に同居できない**（2026-09-06 実測）。

| 環境 | rclpy | unitree_sdk2py |
|---|---|---|
| pixi の Python 3.11（`~/g1_humble`） | OK | **NG**（`No module named 'cyclonedds'`） |
| system の Python 3.8 | **NG** | OK |

Mac のコンテナに SDK を入れる道も試したが、`cyclonedds` の Python バインディングが
ビルドできなかった（C ライブラリ 0.10.5 は在るが、pip のビルドが通らない）。
そこで **ROS 側と SDK 側を別プロセスにし、localhost の UDP で繋ぐ。**
新しい依存を足さずに済み、しかも**止められる側にウォッチドッグを置ける。**

    [Nav2] --/cmd_vel--> [cmd_vel_bridge.py (rclpy/py3.11)] --UDP--> [これ (SDK/py3.8)] --> 足

## 安全の作りは全部こちら側にある

ROS 側が落ちても、UDP が届かなくなっても、**このプロセスが自分で止める。**

| 機構 | 中身 |
|---|---|
| ウォッチドッグ | 最後の指令から `--timeout` 秒でこれ以上動かさず `StopMove()` |
| 速度クランプ | vx / vy / vyaw に上限。**G1 の安全な速度域は未知**（公式値は Go2/Go2_W 用） |
| 発進ゲート | `--arm` を付けない限り SDK を呼ばない |
| 後退の禁止 | 後方センサ上部が死角（2026-09-06 実測）。既定で vx<0 を捨てる |

## 使い方

    # ① まず素振り。SDK を呼ばず、届いた指令を表示するだけ
    ssh g1 'python3 ~/nav_tools/loco_driver.py --dry-run'

    # ② 実機。人が支え、停止手段を持った状態で
    ssh -t g1 'python3 ~/nav_tools/loco_driver.py --network-interface eth0 --arm'
"""
import argparse
import json
import socket
import sys
import time

DEFAULT_PORT = 47600
DEFAULT_MAX_VX = 0.30       # [m/s] ⚠️ 仮。実機で下から上げて決める
DEFAULT_MAX_VY = 0.20       # [m/s]
DEFAULT_MAX_VYAW = 0.50     # [rad/s]
DEFAULT_TIMEOUT = 0.5       # [s] 無指令でこれを超えたら止める


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class LocoDriver:
    def __init__(self, args) -> None:
        self._args = args
        self._client = None
        self._last_seen = None
        self._moving = False
        if not args.dry_run:
            self._init_loco(args.network_interface)

    def _init_loco(self, iface: str) -> None:
        sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        ChannelFactoryInitialize(0, iface)
        self._client = LocoClient()
        self._client.SetTimeout(10.0)
        self._client.Init()
        print("[driver] LocoClient を初期化した（iface={}）".format(iface), flush=True)

    def apply(self, vx: float, vy: float, vyaw: float) -> None:
        vx = clamp(vx, self._args.max_vx)
        vy = clamp(vy, self._args.max_vy)
        vyaw = clamp(vyaw, self._args.max_vyaw)
        if vx < 0.0 and not self._args.allow_reverse:
            vx = 0.0

        self._last_seen = time.time()
        if not self._args.arm:
            return
        if self._args.dry_run:
            print("[driver] (素振り) vx={:+.3f} vy={:+.3f} vyaw={:+.3f}".format(vx, vy, vyaw),
                  flush=True)
            self._moving = True
            return
        self._client.Move(vx, vy, vyaw, continous_move=True)
        self._moving = True

    def watchdog(self) -> None:
        """指令が途切れたら止める。**ここが最後の砦。**"""
        if not self._moving or self._last_seen is None:
            return
        age = time.time() - self._last_seen
        if age <= self._args.timeout:
            return
        if not self._args.dry_run and self._client is not None:
            self._client.StopMove()
        self._moving = False
        print("[driver] 停止した: {:.2f}s 指令が来なかった".format(age), flush=True)

    def stop(self, why: str) -> None:
        if not self._args.dry_run and self._client is not None:
            self._client.StopMove()
        self._moving = False
        print("[driver] 停止した: {}".format(why), flush=True)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--network-interface", default="eth0")
    p.add_argument("--max-vx", type=float, default=DEFAULT_MAX_VX)
    p.add_argument("--max-vy", type=float, default=DEFAULT_MAX_VY)
    p.add_argument("--max-vyaw", type=float, default=DEFAULT_MAX_VYAW)
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    p.add_argument("--allow-reverse", action="store_true",
                   help="後退を許す。**後方センサ上部が死角なので既定では禁止**")
    p.add_argument("--arm", action="store_true", help="発進を許可する。付けなければ SDK を呼ばない")
    p.add_argument("--dry-run", action="store_true", help="SDK を呼ばず、届いた指令を表示するだけ")
    args = p.parse_args(argv)

    driver = LocoDriver(args)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    # ウォッチドッグを回すため、受信は必ずタイムアウトさせる
    sock.settimeout(args.timeout / 4.0)

    print("[driver] 127.0.0.1:{} で待受 / 上限 vx={:.2f} vy={:.2f} vyaw={:.2f} / "
          "無指令 {:.2f}s で停止 / {}".format(
              args.port, args.max_vx, args.max_vy, args.max_vyaw, args.timeout,
              "**素振り**" if args.dry_run
              else ("発進可" if args.arm else "発進ゲート閉（--arm が無い）")), flush=True)

    try:
        while True:
            try:
                payload, _ = sock.recvfrom(256)
            except socket.timeout:
                driver.watchdog()
                continue
            try:
                cmd = json.loads(payload.decode("utf-8"))
                driver.apply(float(cmd["vx"]), float(cmd["vy"]), float(cmd["vyaw"]))
            except Exception as error:
                # 壊れた指令で動かすより黙って捨てるほうが安全。ただし記録は残す
                print("[driver] 指令を捨てた: {}".format(error), file=sys.stderr, flush=True)
            driver.watchdog()
    except KeyboardInterrupt:
        pass
    finally:
        driver.stop("終了する")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
