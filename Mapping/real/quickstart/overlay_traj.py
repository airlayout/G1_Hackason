#!/usr/bin/env python3
"""**再生した候補の軌跡を、事前地図への重畳で採点する。**脚 odom と独立な基準が要るため。

    Navigation/.venv/bin/python quickstart/overlay_traj.py runs/click4_20260913 \\
        --traj "B. AMCL:runs/_replay/click4_amcl" \\
        --traj "D. GLIM:runs/_replay/click4_glim" --window auto

## なぜ要るのか（2026-09-15）

`eval_traj.py` の M1 / M3 / M4 / 脚odom差 は**脚 odom と循環する候補がある**。
AMCL の鎖は `map -(AMCL)-> odom -(dog_odom_to_tf)-> base_link` なので、
**高周波の動きは脚 odom そのもの**で、AMCL が触るのは `map -> odom` だけ。しかも
`update_min_d = 0.20 m` なので 0.2 m 動くまで補正が走らない。
つまり「脚 odom との差」は**脚 odom と脚 odom を比べている**に近い。
09-15 の静止で AMCL に 0.000 m の自動満点が出たのと同じ構造である。

重畳は**事前地図**という脚 odom と無関係な基準に対する量なので、この循環が無い。
**重畳を入れて初めて候補が同じ土俵に乗る。**

## 何を測るか（`pc2/still_report.py` と同じ帯・同じ許容）

| 列 | 定義 |
|---|---|
| 既定帯 許容0   | 床上 0.02〜1.82 m の点が、地図の占有セルに**ぴったり**乗った割合 |
| 壁の帯 許容±1 | 床上 1.30〜1.82 m の点が、占有セルを ±1 セル(10 cm)膨らませた所に乗った割合 |

⚠️ **1 つの数字で判断しないこと。** 既定帯は床の反射を「外れ」と数えるので、
同じ姿勢が既定 44.4 % / 壁の帯 91.9 % に割れる（2026-09-15 実測）。合否は壁の帯で見る。
⚠️ **この部屋のこの場所では 2D 重畳は姿勢を 2 m より細かく決められない**
（種のまわり ±4 m を掃くと 96 % 台が 2.4 m の幅で並ぶ）。順位の**審判**には使えない。
「循環していない side-check」として読むこと。

## 入力

- 位置決め: 候補の軌跡（`map -> base_link`）。再生の出力ディレクトリか TUM ファイル。
- スキャン: **元の記録から**読む（再生の出力に点群は録っていない）。
- 取付: 元の記録の `/tf_static` の `base_link -> livox_frame`。

⚠️ スキャンは 1 枚ずつ流す（1.3 GB の bag を丸ごとメモリに載せない）。
⚠️ 記録には `frame_id='map'` / `point_step=48` の異物が混ざる。**両方**で弾く。
"""
from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure_overlay as mo  # noqa: E402
from eval_traj import open_bag, read_static_tf, resample_hz, to_base_link  # noqa: E402
from tf_chain import resolve  # noqa: E402

# `pc2/still_report.py` と同じ帯（base_link 系＝床上の高さ）
WALL_LO, WALL_HI = 1.30, 1.82
DEF_LO, DEF_HI = 0.02, 1.82


def stream_scans(bag_dir: Path):
    """(t, 生の点群[livox_frame]) を 1 枚ずつ返す。異物は frame_id と point_step で弾く。"""
    db = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}
    n_bad = 0
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?",
                               (tid["/utlidar/cloud_livox_mid360"],)):
        r = mo.Cdr(blob)
        sec, nsec = r.i32(), r.u32()
        fid = r.st()
        r.u32(); r.u32()
        for _ in range(r.u32()):
            r.st(); r.u32(); r.u8(); r.u32()
        r.u8()
        ps = r.u32(); r.u32(); dl = r.u32()
        if fid != "livox_frame" or ps != mo.LIVOX_POINT_STEP:
            n_bad += 1
            continue
        raw = np.frombuffer(blob, dtype=np.uint8, count=dl, offset=r.a)
        xyz = raw.reshape(-1, mo.LIVOX_POINT_STEP)[:, :12].copy().view(np.float32).reshape(-1, 3)
        yield sec + nsec * 1e-9, xyz.astype(np.float64)
    if n_bad:
        print(f"  （異物を {n_bad} 件弾いた）", flush=True)


def load_traj(path: Path, hz: float, st=None) -> np.ndarray:
    """再生の出力ディレクトリ（`map -> base_link` を鎖で辿る）か TUM を (N,8) で返す。"""
    if path.is_dir():
        con, tid = open_bag(path)
        traj, desc = resolve(con, tid)
        if len(traj) < 2:
            raise SystemExit(f"{path}: {desc}")
        print(f"    鎖 {desc}")
    else:
        traj = np.loadtxt(path)
    if st is not None:
        # ⚠️ `livox_frame` の軌跡を `base_link` へ落とす。掛け忘れると**落ちずに数字だけ悪くなる**。
        #    GLIM は `base_link` という名前で出しているが中身は `livox_frame` である
        #    （実測: z +1.278 m・roll -178.8°。MOLA∘取付 の +1.234 m・-179.3° と一致）。
        traj = to_base_link(traj, st)
    traj = resample_hz(traj, hz)
    # ⚠️ Slerp は時刻が**狭義単調増加**でないと例外を出す。鎖の合成は同じ時刻を
    #    2 度出すことがあるので、ここで必ず潰す
    _, keep = np.unique(traj[:, 0], return_index=True)
    return traj[np.sort(keep)]


class Scorer:
    """軌跡 1 本ぶんの集計。スキャンは外から 1 枚ずつ渡す。"""

    def __init__(self, name: str, traj: np.ndarray) -> None:
        self.name = name
        self.t = traj[:, 0]
        self.xyz = traj[:, 1:4]
        self.slerp = Slerp(self.t, Rotation.from_quat(traj[:, 4:8]))
        self.tot = self.hit_def = self.tot_def = self.hit_wall = self.tot_wall = 0

    def covers(self, ts: float) -> bool:
        return bool(self.t[0] <= ts <= self.t[-1])

    def add(self, ts: float, p_base: np.ndarray, occ, occ_tol, res, ox, oy) -> None:
        h, w = occ.shape
        t = np.array([np.interp(ts, self.t, self.xyz[:, i]) for i in range(3)])
        q = (self.slerp(ts).as_matrix() @ p_base.T).T + t
        for lo, hi, grid, key in ((DEF_LO, DEF_HI, occ, "def"),
                                  (WALL_LO, WALL_HI, occ_tol, "wall")):
            s = q[(q[:, 2] >= lo) & (q[:, 2] <= hi)]
            if not len(s):
                continue
            ix = np.floor((s[:, 0] - ox) / res).astype(int)
            iy = np.floor((s[:, 1] - oy) / res).astype(int)
            m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            if not m.any():
                continue
            # ⚠️ read_map は PGM を反転せずに返す。行は h-1-iy（09-11 にここを落とした）
            n_hit = int(grid[h - 1 - iy[m], ix[m]].sum())
            if key == "def":
                self.tot_def += int(m.sum()); self.hit_def += n_hit
            else:
                self.tot_wall += int(m.sum()); self.hit_wall += n_hit

    def report(self) -> tuple[float, float]:
        d = 100.0 * self.hit_def / self.tot_def if self.tot_def else float("nan")
        w = 100.0 * self.hit_wall / self.tot_wall if self.tot_wall else float("nan")
        return d, w


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path, help="元の記録（スキャンと取付をここから読む）")
    ap.add_argument("--traj", action="append", default=[], metavar='"名前:path"',
                    help="再生の出力ディレクトリか TUM。省略すると記録の /tf（C0）")
    ap.add_argument("--ref", type=Path, default=None)
    ap.add_argument("--window", default="full", metavar="A:B",
                    help="記録の頭からの秒。既定 full")
    ap.add_argument("--traj-hz", type=float, default=10.0)
    ap.add_argument("--traj-frame", default="base_link", choices=("base_link", "livox"),
                    help="livox = 軌跡が `map -> livox_frame`（GLIM がこれ）。"
                         "こちらで `base_link` へ落とす")
    ap.add_argument("--every", type=int, default=1, help="スキャンを何枚に 1 枚使うか")
    a = ap.parse_args()

    bag = a.run / "bag" if (a.run / "bag").is_dir() else a.run
    ref = a.ref or (Path(__file__).resolve().parents[1]
                    / "runs/20260906T135940_UiS_room_v3/map/nav_map_ref.yaml")
    occ, res, ox, oy = mo.read_map(ref)[:4]
    occ_tol = binary_dilation(occ, np.ones((3, 3), bool))
    print(f"事前地図 {occ.shape[1]}x{occ.shape[0]} / res {res} m / 占有 {int(occ.sum())} セル")

    con, tid = open_bag(a.run)
    st = read_static_tf(con, tid)
    if st is None:
        raise SystemExit("/tf_static に base_link->livox_frame が無い")
    R_bl_lv, t_bl_lv = Rotation.from_quat(st[1]).as_matrix(), np.asarray(st[0])
    print(f"取付 base_link->livox_frame xyz={tuple(round(v, 3) for v in t_bl_lv)}")

    specs = a.traj or [f"C0 bag内 /tf:{a.run}"]
    scorers = []
    for s in specs:
        name, _, path = s.rpartition(":")
        print(f"  軌跡 {name or path}")
        scorers.append(Scorer(name or path,
                              load_traj(Path(path), a.traj_hz,
                                        st if a.traj_frame == "livox" else None)))

    t_first = min(sc.t[0] for sc in scorers)
    if a.window != "full":
        w0, w1 = (float(v) for v in a.window.split(":"))
    else:
        w0, w1 = -1e18, 1e18

    # 窓は「記録の頭から」の秒なので、記録の先頭時刻が要る
    t_rec0 = next(stream_scans(bag))[0]
    lo, hi = (t_rec0 + w0, t_rec0 + w1) if a.window != "full" else (t_first, 1e18)

    used = 0
    for n, (ts, p) in enumerate(stream_scans(bag)):
        if n % a.every:
            continue
        if not (lo <= ts <= hi):
            continue
        p = p[np.isfinite(p).all(1)]
        d = np.linalg.norm(p, axis=1)
        p = p[(d > mo.RANGE_MIN) & (d < mo.RANGE_MAX)]
        if not len(p):
            continue
        p_base = (R_bl_lv @ p.T).T + t_bl_lv
        for sc in scorers:
            if sc.covers(ts):
                sc.add(ts, p_base, occ, occ_tol, res, ox, oy)
        used += 1
        if used % 500 == 0:
            print(f"  ... {used} 枚", flush=True)

    print(f"\n=== 重畳（{a.run.name} / 窓 {a.window} / スキャン {used} 枚）===")
    print(f"  {'候補':<28}{'既定帯 許容0':>14}{'壁の帯 許容±1':>16}{'評価点(壁)':>14}")
    for sc in scorers:
        dd, ww = sc.report()
        print(f"  {sc.name:<28}{dd:13.1f} %{ww:15.1f} %{sc.tot_wall:14,}")
    print("\n  ⚠️ 合否は壁の帯（> 80 %）で見る。既定帯は床の反射を「外れ」と数える。")
    print("  ⚠️ 2D 重畳は姿勢を 2 m より細かく決められない。順位の審判には使えない。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
