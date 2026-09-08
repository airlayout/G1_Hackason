#!/usr/bin/env python3
"""記録した走行から、**門番の候補が失敗を止められたか**をオフラインで測る。

## なぜ門番が要るか

実機には真値が無い。そして**品質指標は嘘をつく**: MOLA-LO は ICP 品質 0.85
（平常より高い）を出しながら真値から 14.7 m 外れたことがある。
だから「品質が下がったら止める」は効かない。**壊れ方そのものを見る**必要がある。

## 実機で実装できる条件だけを使う

真値を使う門番は sim でしか動かないので**候補にしない**。真値は
「門番が正しく鳴ったか」の採点にだけ使う。

| 候補 | 見るもの | 実機で取れるか | 2026-09-08 の判定 |
|---|---|---|---|
| G1 | 測位器が出す補正 map->odom の跳び | 取れる | **却下** |
| G2 | 推定位置が事前地図の障害物に入る | 取れる | **却下** |
| G3 | 指令したのに odom が並進しない | 取れる | 却下（旋回を誤判定） |
| G4 | 指令したのに並進も旋回もしない | 取れる | **これだけ使える** |

## 判定の根拠（実測。推測ではない）

**G1 却下（2 つの理由）:**
1. `eval_guard_real.py` で実機の記録に当てたら **862/5202 窓＝16.6 % 誤報**。
   この記録は MOLA-LO が真値から 0.023〜0.030 m ＝ **測位は正しかった**もの。
   内蔵 SLAM の odom と MOLA-LO は同じ区間の走行距離が 138.77 m 対 153.41 m
   （10.6 % 違う）ので、2 m 窓ではそれだけで 0.21 m 動く。しきい 0.3 m は埋もれる
2. sim の失敗は**跳びではなく、じわじわ滑った**。ずれが 0.17→2.40 m に開く間、
   0.5 s あたりの map->odom の動きは最大 0.081 m（yaw は −10°→−33° を 70 s かけて連続）。
   **跳び検出では原理的に捕まらない**

**G2 却下:** 推定位置は 2.4 m 外れていても事前地図の自由空間に居続けた。一度も鳴らない。

**G3 却下:** ゴール到着時の**その場旋回**（yaw_rate 0.8 rad/s、vx 0、10 s で 31〜48 度
回りながら 0.12 m）を「嵌った」と誤判定する。並進のしきい値を 0.15 m 以下に
下げないと誤報し、そのぶん検知が遅れた。

**G4 が使える:** 旋回も進捗と数えれば誤報しない。しきい値は
`sweep_guard_thresholds.py` で決める（窓 8 s / 移動 0.15 m / 旋回 10 度で
成功記録に対し 5 倍の余裕）。

## ⚠️ G4 でも「発散そのもの」は捕まらない

G4 が鳴るのは走行 1 で **108 s**。乗り上げは **83 s**、ずれが 1 m を超えたのは **37 s**。
つまり **G4 は被害を止めるだけで、発散を防げない**。
もっと早く鳴らそうとすると、失敗した窓（0.381 m / 25.9 度）と
成功した窓（0.416 m / 33.5 度）が重なって分離できない。

**発散そのものを検知するには、この記録に入っていない量が要る**
（推定位置でのスキャンと地図の残差）。次に記録を取るときは `/scan` を必ず含める。

使い方:
    python3 eval_guards.py --record <dir>/navigation.json --map <map>/nav_map_clean.yaml
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

# --- G1: 測位器の補正の跳び ---
# 短い窓のあいだに map->odom がこれ以上動いたら「吸い付き直した」と見る
G1_WINDOW_M = 2.0      # 窓の長さ（odom の走行距離）
G1_TRANS_M = 0.30      # 並進の跳びの上限
G1_ROT_DEG = 10.0      # 回転の跳びの上限

# --- G2: 推定位置が障害物に入る ---
G2_ROBOT_RADIUS_M = 0.25

# --- G3 / G4: 指令したのに動かない ---
G3_WINDOW_S = 15.0     # この長さの窓で見る
G3_CMD_MIN = 0.05      # 指令がこれ以上出ていた割合が
G3_CMD_FRAC = 0.8      # これ以上の窓で
G3_MOVED_M = 0.15      # odom がこれしか動いていなければ「嵌った」

# G4 は旋回も進捗と数える。sweep_guard_thresholds.py が選んだ値
G4_WINDOW_S = 8.0
G4_MOVED_M = 0.15
G4_TURNED_DEG = 10.0

# 採点用（真値を使う。門番の入力ではない）
FAIL_ERR_M = 1.0       # ずれがこれを超えたら「もう失敗している」


def read_pgm(path: Path) -> np.ndarray:
    """P5 の PGM を読む。コメント行を飛ばす。"""
    data = path.read_bytes()
    fields: list[bytes] = []
    i = 0
    while len(fields) < 4:
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while i < len(data) and data[i : i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j : j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    magic, w, h, maxv = fields[0], int(fields[1]), int(fields[2]), int(fields[3])
    assert magic == b"P5", magic
    assert maxv == 255, maxv
    i += 1  # ヘッダ末尾の空白 1 文字
    return np.frombuffer(data[i : i + w * h], dtype=np.uint8).reshape(h, w)


class OccMap:
    """map_server と同じ約束で事前地図を引く。"""

    def __init__(self, yaml_path: Path) -> None:
        text = yaml_path.read_text()
        conf = {}
        for line in text.splitlines():
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            conf[k.strip()] = v.strip()
        self.res = float(conf["resolution"])
        ox, oy, _ = json.loads(conf["origin"])
        self.origin = (ox, oy)
        self.occ_thresh = float(conf.get("occupied_thresh", 0.65))
        img = read_pgm(yaml_path.parent / conf["image"])
        # negate=0 なら occ = (255 - value) / 255
        self.occupied = ((255 - img.astype(np.float32)) / 255.0) > self.occ_thresh
        self.h, self.w = self.occupied.shape

    def is_blocked(self, x: float, y: float, radius: float) -> bool:
        """(x, y) から radius 以内に占有セルがあるか。地図の外は塞がっている扱い。"""
        cx = (x - self.origin[0]) / self.res
        cy = (y - self.origin[1]) / self.res
        r = radius / self.res
        c0, c1 = int(math.floor(cx - r)), int(math.ceil(cx + r))
        r0, r1 = int(math.floor(cy - r)), int(math.ceil(cy + r))
        if c0 < 0 or r0 < 0 or c1 >= self.w or r1 >= self.h:
            return True
        # 画像の行 0 は y が最大の側
        top = self.h - 1 - r1
        bot = self.h - 1 - r0
        return bool(self.occupied[top : bot + 1, c0 : c1 + 1].any())


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def map_odom(amcl: list, truth: list) -> tuple[float, float, float]:
    """T_mo = T_mb ∘ T_ob^-1。**測位器が出している補正そのもの。**

    実機ではこれを TF から直接引ける（真値の引き算ではない）。
    ここでは記録に map->odom を残していないので、記録済みの
    map->base_link（amcl）と odom->base_link（truth）から復元する。
    """
    mx, my, mth = amcl
    tx, ty, tth = truth
    dth = wrap(mth - tth)
    c, s = math.cos(dth), math.sin(dth)
    return (mx - (c * tx - s * ty), my - (s * tx + c * ty), dth)


def split_runs(track: list[dict]) -> list[tuple[int, int]]:
    """goal が変わるところで走行を切る。**同じゴールへ 2 回投げた場合は切れない**
    ので、指令が長く途切れた点も境界として扱う。"""
    bounds = [0]
    for i in range(1, len(track)):
        if track[i]["goal"] != track[i - 1]["goal"]:
            bounds.append(i)
        elif track[i]["t"] - track[i - 1]["t"] > 5.0:
            bounds.append(i)  # 走行の切れ目（次のゴールを選ぶ間）
    bounds.append(len(track))
    return [(bounds[k], bounds[k + 1]) for k in range(len(bounds) - 1)]


def err_of(s: dict) -> float | None:
    """真値と推定のずれ。**採点用**。門番はこれを見られない。"""
    if not s["truth"] or not s["amcl"]:
        return None
    return math.hypot(s["truth"][0] - s["amcl"][0], s["truth"][1] - s["amcl"][1])


def analyse(track: list[dict], omap: OccMap | None) -> list[dict]:
    out = []
    for run_i, (a, b) in enumerate(split_runs(track), 1):
        seg = track[a:b]
        t0 = seg[0]["t"]
        # odom（真値）の走行距離。実機では脚オドメトリの積算に相当する
        dist = [0.0]
        for i in range(1, len(seg)):
            p, q = seg[i - 1]["truth"], seg[i]["truth"]
            dist.append(dist[-1] + (math.hypot(q[0] - p[0], q[1] - p[1])
                                    if p and q else 0.0))
        mo = [map_odom(s["amcl"], s["truth"]) if s["amcl"] and s["truth"]
              else None for s in seg]

        turn = [0.0]
        for i in range(1, len(seg)):
            p, q = seg[i - 1]["truth"], seg[i]["truth"]
            turn.append(turn[-1] + (abs(wrap(q[2] - p[2])) if p and q else 0.0))

        trips: dict[str, dict | None] = {"G1": None, "G2": None,
                                         "G3": None, "G4": None}

        # G1: 窓 G1_WINDOW_M のあいだの map->odom の跳び
        for i in range(len(seg)):
            if mo[i] is None:
                continue
            j = i
            while j > 0 and dist[i] - dist[j] < G1_WINDOW_M:
                j -= 1
            if mo[j] is None or dist[i] - dist[j] < G1_WINDOW_M:
                continue
            dt = math.hypot(mo[i][0] - mo[j][0], mo[i][1] - mo[j][1])
            dr = abs(math.degrees(wrap(mo[i][2] - mo[j][2])))
            if dt > G1_TRANS_M or dr > G1_ROT_DEG:
                trips["G1"] = {"i": i, "t": seg[i]["t"] - t0,
                               "detail": f"並進 {dt:.2f} m / 回転 {dr:.1f} 度"}
                break

        # G2: 推定位置が事前地図の障害物に入る
        if omap is not None:
            for i, s in enumerate(seg):
                if not s["amcl"]:
                    continue
                if omap.is_blocked(s["amcl"][0], s["amcl"][1], G2_ROBOT_RADIUS_M):
                    trips["G2"] = {"i": i, "t": s["t"] - t0,
                                   "detail": f"推定 ({s['amcl'][0]:+.2f}, "
                                             f"{s['amcl'][1]:+.2f}) が障害物内"}
                    break

        # G3: 指令が出ているのに odom が動かない
        for i in range(len(seg)):
            j = i
            while j > 0 and seg[i]["t"] - seg[j]["t"] < G3_WINDOW_S:
                j -= 1
            if seg[i]["t"] - seg[j]["t"] < G3_WINDOW_S:
                continue
            win = seg[j : i + 1]
            commanded = sum(
                1 for s in win
                if math.hypot(s["cmd"][0], s["cmd"][1]) > G3_CMD_MIN
                or abs(s["cmd"][2]) > G3_CMD_MIN)
            if commanded / len(win) < G3_CMD_FRAC:
                continue
            if dist[i] - dist[j] < G3_MOVED_M:
                trips["G3"] = {"i": i, "t": seg[i]["t"] - t0,
                               "detail": f"{G3_WINDOW_S:.0f} s 指令が出て "
                                         f"{dist[i] - dist[j]:.2f} m しか進まず"}
                break

        # G4: 指令が出ているのに並進も旋回もしない（**これだけ使える**）
        for i in range(len(seg)):
            if seg[i]["t"] - t0 < 5.0 + G4_WINDOW_S:
                continue           # 起動猶予。指令が出ても歩行はまだ加速していない
            j = i
            while j > 0 and seg[i]["t"] - seg[j]["t"] < G4_WINDOW_S:
                j -= 1
            if seg[i]["t"] - seg[j]["t"] < G4_WINDOW_S:
                continue
            win = seg[j : i + 1]
            commanded = sum(
                1 for s in win
                if math.hypot(s["cmd"][0], s["cmd"][1]) > G3_CMD_MIN
                or abs(s["cmd"][2]) > G3_CMD_MIN)
            if commanded / len(win) < G3_CMD_FRAC:
                continue
            d, r = dist[i] - dist[j], math.degrees(turn[i] - turn[j])
            if d < G4_MOVED_M and r < G4_TURNED_DEG:
                trips["G4"] = {"i": i, "t": seg[i]["t"] - t0,
                               "detail": f"{G4_WINDOW_S:.0f} s 指令が出て "
                                         f"{d:.2f} m / {r:.1f} 度しか動かず"}
                break

        errs = [err_of(s) for s in seg]
        first_fail = next((i for i, e in enumerate(errs)
                           if e is not None and e > FAIL_ERR_M), None)
        zs = [s["sim_z"] for s in seg if s["sim_z"] is not None]
        first_climb = next((i for i, s in enumerate(seg)
                            if s["sim_z"] is not None and s["sim_z"] > 0.75), None)

        out.append({
            "run": run_i, "n": len(seg), "dur": seg[-1]["t"] - t0,
            "goal": seg[0]["goal"], "dist": dist[-1],
            "err_start": errs[0], "err_end": errs[-1],
            "err_max": max((e for e in errs if e is not None), default=None),
            "z_range": (min(zs), max(zs)) if zs else None,
            "first_fail": first_fail,
            "first_fail_t": seg[first_fail]["t"] - t0 if first_fail is not None else None,
            "first_climb_t": seg[first_climb]["t"] - t0 if first_climb is not None else None,
            "trips": trips,
            "errs": errs, "t_rel": [s["t"] - t0 for s in seg],
        })
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", required=True)
    ap.add_argument("--map", help="事前地図の yaml（G2 に必要）")
    args = ap.parse_args()

    rec = json.loads(Path(args.record).read_text())
    omap = OccMap(Path(args.map)) if args.map else None
    if omap is not None:
        print(f"[map] {omap.w}x{omap.h} セル / {omap.res} m / "
              f"占有 {int(omap.occupied.sum())} セル")
    runs = analyse(rec["track"], omap)

    print(f"\n記録: {len(rec['track'])} サンプル / 到達 "
          f"{sum(rec['results'])}/{len(rec['results'])}\n")
    for r in runs:
        print(f"== 走行 {r['run']}  {r['dur']:.0f} s / {r['n']} サンプル / "
              f"odom {r['dist']:.2f} m  ゴール "
              f"({r['goal'][0]:+.2f}, {r['goal'][1]:+.2f}) ==")
        print(f"   ずれ  開始 {r['err_start']:.2f} m → 最大 {r['err_max']:.2f} m "
              f"→ 終了 {r['err_end']:.2f} m")
        if r["z_range"]:
            print(f"   真値 z  {r['z_range'][0]:.2f} 〜 {r['z_range'][1]:.2f} m"
                  + (f"（{r['first_climb_t']:.0f} s に 0.75 m 超え＝乗り上げ）"
                     if r["first_climb_t"] is not None else ""))
        if r["first_fail_t"] is not None:
            print(f"   ずれが {FAIL_ERR_M:.1f} m を超えたのは {r['first_fail_t']:.0f} s")
        else:
            print(f"   ずれは {FAIL_ERR_M:.1f} m を超えなかった")
        for name in ("G1", "G2", "G3", "G4"):
            tr = r["trips"][name]
            if tr is None:
                print(f"   {name}: 鳴らなかった")
                continue
            e = r["errs"][tr["i"]]
            lead = (r["first_fail_t"] - tr["t"]) if r["first_fail_t"] is not None else None
            msg = (f"   {name}: {tr['t']:.0f} s に鳴った"
                   f"（そのときのずれ {e:.2f} m / {tr['detail']}）")
            if lead is not None:
                msg += f" 失敗の {lead:+.0f} s 前"
            print(msg)
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
