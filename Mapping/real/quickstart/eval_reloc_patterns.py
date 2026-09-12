#!/usr/bin/env python3
"""段 8 — 再定位を記録の 6 パターンで検証する（門）。

## なぜ要るのか

`--relocalize` は素直な 1 枚では 5/5 正しく判定した（段 7b）。だが素材が
**静止の対照の 1 枚だけ**で、実際に直したい状況（歩行のすべった終端・机が動いた）は
一度も当てていない。**ここが実機・Isaac Sim へ進む門**（計画書 §2.6）。

## 6 パターン

| # | 内容 | 素材 | 合格 |
|---|---|---|---|
| 1 | 小さなずれ +0.5 m | 静止 | 真値に戻り「採用」 |
| 2 | 中程度 +1.5 m | 静止 | 同上 |
| 3 | 大きい +3 m（ROI の端） | 静止 | 同上、または**「自信なし」**（黙って間違えない） |
| 4 | **実際の失敗からの復帰** r1/r2/r3 のすべった終端 | 歩行 3 本 | 真値が無いのでゲートと重畳で見る |
| 5 | **机が動いた**（z 0.2〜1.2 m を 1 m 平行移動） | 静止（合成） | 真値に戻り「採用」 |
| 6 | **誤報** 正しい姿勢を渡す | 静止 | **同じ姿勢を返し、勝手に飛ばない** |

## 決めごと

- **r0（ゲートの基準）は静止の対照で 1 回較正し、全パターンで使い回す。**
  これが実際の運用（信頼できるときに較正して、外れたときに使う）と同じ形。
  パターンごとに較正し直すと、**歩行のすべった姿勢で較正してしまい**基準が壊れる
- 探索は全点、ゲートは壁の帯（段 6）。`--relocalize` が中で固定している

使い方:
    Navigation/.venv/bin/python quickstart/eval_reloc_patterns.py \\
        --out runs/reloc_eval_20260911
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]                      # .../physical_ai
REAL = HERE.parent                          # .../Mapping/real
VENV = REPO / "G1_Hackason/Navigation/.venv/bin/python"
SESSION = "20260906T135940_UiS_room_v3"
CONTAINER = "rviz"
WORK = "/work/G1_Hackason/Mapping/real"
PROBE = "/tmp/reloc_build/bin/reloc_probe"

STILL = "stage_20260910T084053"
WALKS = ("stage_20260910T182639_r1", "stage_20260910T182639_r2", "stage_20260910T182639_r3")

BAND_LO = 1.3            # 3D ゲートに使う壁の帯の下限 [m]
DESK_BAND = (0.2, 1.2)   # 机の高さ帯（動かす対象）
ROI = 3.0
# ⚠️ **二重ゲート**（段 8b）。3D 残差（map.mm）だけでは誤採用する。
#    2026-09-11 の段 8 で r3 が残差 1.02 倍（較正値より良い）で採用されたのに
#    2D 重畳は 36.9%（真値基準 73.4%）だった。**3D と 2D は別の地図**なので、
#    片方が壊れてももう片方で気づける。**食い違ったら棄却する。**
OVERLAY_FRAC = 0.80      # 重畳が「真値に置いたときの値」の何割を下回ったら棄却するか
# ⚠️ **ここだけ旧 nav_map のまま残してある**（2026-09-12）。重畳の基準は他では
#    nav_map_ref（make_ref_map.py 製）に移したが、段 8b の「誤採用 9/9 ゼロ」は
#    この地図で測った値なので、**基準を替えると根拠も測り直しになる**。
#    O0 は実行時に測り直すので替えても動くが、9/9 の再現を確認してからにすること。
REF_MAP = "map/old/nav_map.yaml"     # ⚠️ 間引いていない旧 nav_map（clean だと天井 43.9%）
# 現実的な「机 1 台」。帯を丸ごと動かすと 44% の点が動き、それは机ではなく部屋の半分
DESK_ONE = (-0.5, 0.5, 1.0)          # base_link 系の中心と半径 [m]（実測で最も密な塊）


def run(cmd: list, **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def in_container(args: list) -> subprocess.CompletedProcess:
    """コンテナで reloc_probe を叩く（Mac に MRPT は無い）。"""
    return run(["docker", "exec", "-u", "ubuntu", CONTAINER, "bash", "-c",
                "source /opt/ros/humble/setup.bash && " + " ".join(args)])


def export_scan(bag: Path, out: Path, extra: list) -> dict:
    """1 スキャンを base_link 系で出し、メタ（推定姿勢を含む）を返す。"""
    cmd = [str(VENV), str(HERE / "export_scan.py"), str(bag), "--out", str(out)] + extra
    r = run(cmd, cwd=REAL)
    if r.returncode != 0:
        raise SystemExit("export_scan が落ちた:\n{}\n{}".format(r.stdout, r.stderr))
    return json.loads(out.with_suffix(".json").read_text())


def probe(scan: Path, center: tuple, out_stem: Path, r0: float,
          truth: tuple | None) -> dict:
    """--relocalize を 1 回回して JSON を読む。"""
    rel = lambda p: str(p).replace(str(REAL), WORK)     # noqa: E731
    args = [PROBE, "--map", "{}/runs/{}/mola_floor0/map.mm".format(WORK, SESSION),
            "--scan", rel(scan), "--relocalize",
            "--center", "{:.4f}".format(center[0]), "{:.4f}".format(center[1]),
            "--roi", str(ROI), "--r0", "{:.5f}".format(r0),
            "--band-lo", str(BAND_LO), "--top", "1",
            "--json", rel(out_stem.with_suffix(".json")),
            "--dump-grid", rel(out_stem.with_suffix(".grid"))]
    if truth is not None:
        args += ["--truth", "{:.4f}".format(truth[0]), "{:.4f}".format(truth[1]),
                 "{:.3f}".format(truth[2])]
    r = in_container(args)
    path = out_stem.with_suffix(".json")
    if not path.exists():
        raise SystemExit("reloc_probe が json を書かなかった:\n{}\n{}".format(r.stdout, r.stderr))
    return json.loads(path.read_text())


def overlay_at(scan_xyz: Path, pose, ref_map: Path, _cache={}) -> float:
    """返った姿勢での 2D 重畳 [%]。**3D 残差とは別の地図**を見る（二重ゲートの片側）。"""
    sys.path.insert(0, str(HERE))
    from measure_overlay import read_map          # noqa: E402
    from overlay_at_pose import overlay           # noqa: E402
    if "map" not in _cache:
        _cache["map"] = read_map(ref_map)
    if scan_xyz not in _cache:
        _cache[scan_xyz] = np.loadtxt(scan_xyz)
    occ, res, ox, oy = _cache["map"]
    return overlay(_cache[scan_xyz], pose, occ, res, ox, oy)["hit_pct"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True, help="書き出し先（runs/ の下）")
    ap.add_argument("--still-index", type=int, default=45)
    a = ap.parse_args()
    out = (REAL / a.out) if not a.out.is_absolute() else a.out
    out.mkdir(parents=True, exist_ok=True)

    # ── 素材を作る ────────────────────────────────────────────────────
    print("== 素材 ==")
    still = export_scan(REAL / "runs" / STILL / "bag", out / "still_full",
                        ["--index", str(a.still_index)])
    t = still["truth_map_base_link"]
    truth = (t["x"], t["y"], t["yaw_deg"])
    print("  静止の対照 {} 点 / 真値 ({:.3f}, {:.3f}, {:.2f} deg)".format(
        still["points"], *truth))

    desk = export_scan(REAL / "runs" / STILL / "bag", out / "still_desk",
                       ["--index", str(a.still_index),
                        "--move-band", str(DESK_BAND[0]), str(DESK_BAND[1]), "1.0", "0.0"])
    print("  机（帯を丸ごと・過酷）{} 点 / {}".format(desk["points"], desk["synth"]))
    desk1 = export_scan(REAL / "runs" / STILL / "bag", out / "still_desk1",
                        ["--index", str(a.still_index), "--move-region",
                         str(DESK_ONE[0]), str(DESK_ONE[1]), str(DESK_ONE[2]),
                         str(DESK_BAND[0]), str(DESK_BAND[1]), "1.0", "0.0"])
    print("  机（1 台ぶん・現実的）{} 点 / {}".format(desk1["points"], desk1["synth"]))

    walks = {}
    for w in WALKS:
        # すべった**終端**が欲しいので、使える最後のスキャンを取る
        meta = export_scan(REAL / "runs" / w / "bag", out / "walk_{}".format(w[-2:]),
                           ["--index", "-1"])
        walks[w] = meta
    print()

    # ── r0 を 1 回だけ較正して使い回す ────────────────────────────────
    print("== ゲートの較正（静止の対照。全パターンで使い回す）==")
    cal = in_container([PROBE, "--map", "{}/runs/{}/mola_floor0/map.mm".format(WORK, SESSION),
                        "--scan", str(out / "still_full.xyz").replace(str(REAL), WORK),
                        "--relocalize", "--center", "{:.4f}".format(truth[0]),
                        "{:.4f}".format(truth[1]), "--roi", "0.5",
                        "--trusted-pose", "{:.4f}".format(truth[0]),
                        "{:.4f}".format(truth[1]), "{:.3f}".format(truth[2]),
                        "--band-lo", str(BAND_LO), "--top", "1"])
    r0 = None
    for line in cal.stdout.splitlines():
        if "基準 r0" in line:
            r0 = float(line.split("=")[1].split("m")[0])
    if r0 is None:
        raise SystemExit("r0 を較正できなかった:\n{}\n{}".format(cal.stdout, cal.stderr))
    print("  r0 = {:.5f} m（帯 z>{} m）".format(r0, BAND_LO))
    print()

    # ── パターンを回す ────────────────────────────────────────────────
    def off(dx, dy):
        return (truth[0] + dx, truth[1] + dy)

    patterns = [
        ("1", "小さなずれ +0.5 m / +10°", out / "still_full.xyz", off(0.5, 0.0), truth,
         "戻って採用"),
        ("2", "中程度 +1.5 m / +45°", out / "still_full.xyz",
         off(1.5 * math.cos(math.radians(45)), 1.5 * math.sin(math.radians(45))), truth,
         "戻って採用"),
        ("3", "大きい +3 m / 180°（ROI の端）", out / "still_full.xyz", off(3.0, 0.0), truth,
         "戻って採用、または自信なし"),
        ("5a", "机 1 台が動いた（現実的）", out / "still_desk1.xyz", off(0.5, 0.0), truth,
         "戻って採用"),
        ("5b", "帯を丸ごと動かした（過酷）", out / "still_desk.xyz", off(0.5, 0.0), truth,
         "戻るか、自信なし"),
        ("6", "誤報（正しい姿勢を渡す）", out / "still_full.xyz", off(0.0, 0.0), truth,
         "同じ姿勢・飛ばない"),
    ]
    for w in WALKS:
        m = walks[w]["truth_map_base_link"]
        patterns.append(("4" + w[-1], "実際の失敗からの復帰 {}".format(w[-2:]),
                         out / "walk_{}.xyz".format(w[-2:]), (m["x"], m["y"]), None,
                         "ゲートが判断（真値なし）"))

    # ── 2D 側の較正: 同じスキャンを真値に置いたときの重畳 ────────────────
    ref_map = REAL / "runs" / SESSION / REF_MAP
    o0 = overlay_at(out / "still_full.xyz", truth, ref_map)
    print("== 2D の較正（二重ゲートのもう片側）==")
    print("  真値に置いたときの重畳 O0 = {:.1f}%（基準地図 {}）".format(o0, REF_MAP))
    print("  ⚠️ 91 枚の平均 78.2% ではなく**この 1 枚の値**。棄却線は O0 x {:.2f} = {:.1f}%"
          .format(OVERLAY_FRAC, OVERLAY_FRAC * o0))
    print()

    results = []
    print("== パターン ==")
    hdr = "{:<5} {:<30} {:>8} {:>6} {:>8} {:>7}  {}"
    print(hdr.format("#", "内容", "真値差", "r/r0", "重畳", "対 O0", "二重ゲートの判定"))
    for pid, name, scan, center, tr, expect in patterns:
        stem = out / "pattern{}".format(pid)
        res = probe(scan, center, stem, r0, tr)
        # ── 二重ゲート: 3D が通っても 2D が食い違えば棄却 ──────────────
        ov = overlay_at(scan, res["pose"], ref_map)
        frac = ov / o0 if o0 else 0.0
        ok3d = res["accepted"]
        ok2d = frac >= OVERLAY_FRAC
        if ok3d and ok2d:
            verdict = "採用"
        elif ok3d and not ok2d:
            verdict = "棄却（3D は通ったが 2D が食い違う）"
        else:
            verdict = res["verdict"]
        res.update({"id": pid, "name": name, "expect": expect, "scan": scan.name,
                    "believed": list(center), "overlay_pct": ov, "overlay_frac": frac,
                    "gate3d": ok3d, "gate2d": ok2d, "final": verdict})
        results.append(res)
        print(hdr.format(pid, name[:28],
                         "{:.2f}".format(res["error_m"]) if "error_m" in res else "—",
                         "{:.2f}".format(res["ratio"]),
                         "{:.1f}%".format(ov), "{:.2f}".format(frac), verdict))

    (out / "patterns.json").write_text(json.dumps(
        {"r0": r0, "o0": o0, "overlay_frac": OVERLAY_FRAC, "ref_map": REF_MAP,
         "band_lo": BAND_LO, "roi": ROI, "truth": list(truth),
         "still_index": a.still_index, "results": results},
        ensure_ascii=False, indent=2))
    print("\n書いた: {}".format(out / "patterns.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
