#!/usr/bin/env python3
"""ゴースト除去の候補を **1 枚のスコアカード**で採点する。全案の共通の物差し。

## 2026-09-13 に 2 つ直した

**① 主指標を「壁の帯」にした**（ユーザーの判断）。09-12 の守りは「全体の重畳 73.8% 以上」
だったが、これは**床が点の大半を占めて「乗った」を稼ぐ**指標で、**ゴーストを消すと構造上
必ず下がる**。実測の内訳では候補 R=0.50 の損失 2.1 pt のうち **58% が床**で、
**壁は 1 点も失っていない**。

**② 合否はフィルタが見ない記録で測る**。09-12 の `still_..._r1` はフィルタ側が
「6 日後にも在るセルは残す」保護に使う。**同じ記録で合否も測ると素通りする**
（2026-09-13 に実測。机の守りが 13.2% 落ちたので追ったら、消えていたのは
「09-10 には 0.64 m の物が在り、09-12 には床しか無い」＝**動かされた家具**だった）。

→ **合否は `stage_20260910T084053`（別の日の静止）だけで測る。**
09-12 の数字は継続性のために並べて出すが、合否には使わない。

## 守り

| 守り | 中身（すべて 09-10 の記録） | 線 |
|---|---|---|
| ②壁の帯【主】 | 床上 1.3 m 超の重畳 | 基準 −0.4 pt 以内 |
| ③机の帯 | 床上 0.5-1.2 m の重畳 | 基準 −1.5 pt 以内 |
| ④ずらし | (0,0) が ±0.20 m の四方より強い | OK |

削減率（A / B / 机 / 壁）と全体の重畳は**報告のみ**。
机の削減は「動かされた家具」でも上がるので、合否には使わない（上記の実測）。
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

REAL = Path(__file__).resolve().parents[2]          # .../Mapping/real
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(REAL / "quickstart"))
sys.path.insert(0, str(HERE))

from measure_overlay import world_points, count_hits, read_map   # noqa: E402
from pcd_to_occupancy import build_grid, write_map               # noqa: E402
from filters import _cellkey, FZ, RES                            # noqa: E402

SESSION = REAL / "runs" / "20260906T135940_UiS_room_v3"
# 出力物（キャッシュ・候補地図・ログ）はここ。**`runs/` は .gitignore の対象**なので、
# 道具は quickstart 側に置き、成果物だけ runs に置く（2026-09-13。
# 前回はこの一式が /private/tmp にしか無く、危うく消えるところだった）
WORK = REAL / "runs" / "ghost_eval_20260912"
JUDGE_BAG = REAL / "runs" / "stage_20260910T084053" / "bag"        # 合否はこれだけで出す
REF_BAG = REAL / "runs" / "still_20260912T085449_r1" / "bag"       # 継続性の参考／フィルタが使う
JUDGE_CACHE = WORK / "live_stage_0910.npz"
REF_CACHE = WORK / "live_still_r1.npz"

BAND = (0.15, 1.80)
SHIFT = 0.20

# ⚠️ 入力は **raw**。map_full_p.txt は 2026-09-12 反復 2 で**棄却された**持続性フィルタの
# 出力（raw − 187,427 点）。渡すと占有セル 19,714 の別地図が黙って出る
RAW_TXT = SESSION / "map" / "ref" / "map_full_raw.txt"
TRAJ = SESSION / "mola_floor0" / "traj.txt"

REGIONS = {                       # 2026-09-12 にユーザーが画面上で囲んだ 2 箇所
    "A_廊下":     (-6.4, 5.0, 14.1, 18.9),
    "B_開けた床": (-3.8, 1.7, -2.6,  3.7),
}
DESK = (0.0, 5.0, 0.0, 5.0)
GUARD = dict(wall_drop_max=0.4, deskband_drop_max=1.5)


def load_map() -> tuple[np.ndarray, np.ndarray]:
    p = pd.read_csv(RAW_TXT, sep=r"\s+")[["x", "y", "z"]].to_numpy(np.float64)
    tr = np.loadtxt(TRAJ, usecols=(1, 2))
    d, _ = cKDTree(tr).query(p[:, :2], workers=-1)
    return p, d


def live_points(bag: Path = REF_BAG, cache: Path = REF_CACHE) -> np.ndarray:
    if cache.exists():
        return np.load(cache)["xyz"]
    xyz, used, _, _ = world_points(bag, verbose=False)
    np.savez_compressed(cache, xyz=xyz.astype(np.float32), used=used)
    print("ライブをキャッシュ: {} 枚 / {:,} 点 -> {}".format(used, len(xyz), cache.name))
    return xyz


def cell_hmax(xyz: np.ndarray) -> dict:
    """セルごとの「床上の最高点」。見ていないセルは入らない。"""
    h = xyz[:, 2] - FZ
    k = _cellkey(xyz[:, :2])
    o = np.argsort(k)
    ks, vs = k[o], h[o]
    u, i = np.unique(ks, return_index=True)
    return dict(zip(u.tolist(), np.maximum.reduceat(vs, i).tolist()))


def groups(p: np.ndarray, d: np.ndarray, judge_hmax: dict) -> dict[str, np.ndarray]:
    """報告に使う点の群。机は**別の日の記録で裏の取れたセル**に限る。

    ⚠️ 09-12 の定義は「軌跡から 1 m 超」だった。2026-09-13 に汚染を実測:
    その群の 296 点は高さ中央 1.07 m（本物の机は 0.70 m）、8 セルだけで、
    うち 7 セルはライブが**床 0.03 m しか見ていない** ＝ 軌跡 1.1 m 脇に立っていた人。
    """
    h = p[:, 2] - FZ
    out = {}
    for n, (x0, x1, y0, y1) in REGIONS.items():
        out[n] = ((p[:, 0] >= x0) & (p[:, 0] <= x1) & (p[:, 1] >= y0) & (p[:, 1] <= y1)
                  & (h >= 0.30) & (h <= 1.80))
    x0, x1, y0, y1 = DESK
    box = ((p[:, 0] >= x0) & (p[:, 0] <= x1) & (p[:, 1] >= y0) & (p[:, 1] <= y1)
           & (h >= 0.20) & (h <= 1.20))
    k = _cellkey(p[:, :2])
    out["机"] = box & np.array([judge_hmax.get(int(v), -9.0) >= 0.30 for v in k])
    out["壁"] = (h >= 1.90) & (h <= 2.30)
    return out


class _Bag:
    def __init__(self, xyz):
        self.xyz = xyz.astype(np.float64)
        h = self.xyz[:, 2] - FZ
        self.wall = h >= 1.30
        self.desk = (h >= 0.50) & (h <= 1.20)


class Scorecard:
    HEAD = ("{:<22}{:>9}{:>10}{:>9}{:>9}{:>9}{:>8}{:>7}{:>7}{:>7}{:>7}"
            .format("候補", "占有セル", "壁の帯▲", "机の帯", "全体", "全体", "ずらし",
                    "A減", "B減", "机減", "判定"))
    HEAD2 = ("{:<22}{:>9}{:>10}{:>9}{:>9}{:>9}{:>8}{:>7}{:>7}{:>7}{:>7}"
             .format("", "", "(09-10)", "(09-10)", "(09-10)", "(09-12)", "", "", "", "", ""))

    def __init__(self) -> None:
        self.judge = _Bag(live_points(JUDGE_BAG, JUDGE_CACHE))
        self.ref = _Bag(live_points(REF_BAG, REF_CACHE))
        self.judge_hmax = cell_hmax(self.judge.xyz)
        self.p, self.d = load_map()
        self.g = groups(self.p, self.d, self.judge_hmax)
        self.base: dict | None = None

    @staticmethod
    def _pct(xy, occ, res, ox, oy, dx=0.0, dy=0.0) -> float:
        hit, tot = count_hits(xy, occ, res, ox, oy, dx, dy)
        return 100.0 * hit / tot if tot else 0.0

    def evaluate(self, name: str, keep: np.ndarray, out_dir: Path,
                 save_txt: bool = False) -> dict:
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = out_dir / name
        img, lo = build_grid(self.p[keep], RES, BAND, 0.30, 1, verbose=False)
        write_map(img, lo, RES, stem)
        occ, res, ox, oy = read_map(stem.with_suffix(".yaml"))
        J, R = self.judge, self.ref

        r = dict(name=name, cells=int((img == 0).sum()), points=int(keep.sum()),
                 wall=self._pct(J.xyz[J.wall], occ, res, ox, oy),
                 deskband=self._pct(J.xyz[J.desk], occ, res, ox, oy),
                 total=self._pct(J.xyz, occ, res, ox, oy),
                 total0912=self._pct(R.xyz, occ, res, ox, oy))
        worst = max(self._pct(J.xyz, occ, res, ox, oy, dx, dy)
                    for dx, dy in ((SHIFT, 0), (-SHIFT, 0), (0, SHIFT), (0, -SHIFT)))
        r["shift_ok"] = r["total"] > worst
        drop = ~keep
        for n, m in self.g.items():
            r["drop_" + n] = 100.0 * (drop & m).sum() / max(m.sum(), 1)
        r["drop_all"] = 100.0 * drop.mean()

        if self.base is None:
            self.base = r
            r["verdict"] = "基準"
        else:
            r["verdict"] = "OK" if (
                r["wall"] >= self.base["wall"] - GUARD["wall_drop_max"]
                and r["deskband"] >= self.base["deskband"] - GUARD["deskband_drop_max"]
                and r["shift_ok"]) else "NG"
        if save_txt:
            np.savetxt(stem.with_suffix(".txt"), self.p[keep],
                       header="x y z", comments="", fmt="%.6f")
        return r

    @staticmethod
    def row(r: dict) -> str:
        return ("{:<22}{:>9,}{:>9.1f}%{:>8.1f}%{:>8.1f}%{:>8.1f}%{:>8}"
                "{:>6.1f}%{:>6.1f}%{:>6.1f}%{:>7}".format(
                    r["name"], r["cells"], r["wall"], r["deskband"], r["total"],
                    r["total0912"], "OK" if r["shift_ok"] else "NG",
                    r["drop_A_廊下"], r["drop_B_開けた床"], r["drop_机"], r["verdict"]))

    def header(self) -> str:
        return "\n" + self.HEAD + "\n" + self.HEAD2

    @staticmethod
    def rule() -> str:
        return ("守り（すべて 09-10 の記録で測る）: ②壁の帯 >= 基準 -{:.1f}pt【主】/ "
                "③机の帯 >= 基準 -{:.1f}pt / ④ずらし OK\n"
                "  ＊09-12 の全体・机の削減は**報告のみ**（フィルタが 09-12 を見るので合否に使えない）"
                .format(GUARD["wall_drop_max"], GUARD["deskband_drop_max"]))


if __name__ == "__main__":
    sc = Scorecard()
    print(sc.header())
    print(Scorecard.row(sc.evaluate("00_before", np.ones(len(sc.p), bool),
                                    WORK / "cand")))
    print("\n" + Scorecard.rule())
