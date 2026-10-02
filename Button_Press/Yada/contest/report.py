"""評価の結果（evaluate.py が保存した JSON）から、弱点のレポート（Markdown）を作る。

    P=~/miniconda3/envs/lerobot/bin/python
    $P Button_Press/Yada/contest/report.py _local/button_press_yada/results/<結果>.json
    $P Button_Press/Yada/contest/report.py <結果1>.json <結果2>.json      # 複数をまとめる（同じエージェントの結果）

レポートの中身:
1. 成功率と、どこで失敗したか（runner.classify_stage の分類）
2. 乱しを 1 種類ずつ入れた評価（evaluate.py --ablation の結果があれば）: 種類ごとの成功率と、乱しなしとの差
3. 条件ごとの成功率: 条件の値で試行を半分に分け（中央値で）、小さい側と大きい側の成功率を比べる。差が大きい順
4. 失敗した試行の一覧: 目立って厳しかった条件と、再現のコマンド

注意: 3 は「一緒に変わった条件」の影響も混ざる（相関であって原因とは限らない）。原因を確かめるには 2 を使う。
試行が少ないと差はたまたまのことがある（目安として、条件 1 つにつき数十試行）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT  # noqa: E402
from common.realism import FEATURES, GROUP_LABELS  # noqa: E402

REPORT_DIR = REPO_ROOT / "_local" / "button_press_yada" / "reports"

STAGE_LABELS: dict[str, tuple[str, str]] = {
    "success": ("成功", ""),
    "wrong_button": ("違うボタンを押した", "▲▼の見分け、指示の理解"),
    "error": ("エラー（例外・出力の形の誤り）", "エージェントのログを見る"),
    "no_motion": ("腕をほとんど動かさなかった", "ボタンを見つけられなかった・計画できなかった可能性（動く前の段階）"),
    "collision": ("壁・扉・盤に強くぶつかった", "手前の位置、押す向き、腕の通り道"),
    "touched_not_pressed": ("ボタンに触れたが、点灯する深さまで沈まなかった", "押し込みの深さ・力、指先の位置のずれ"),
    "near_miss": ("指先がボタンの近く（3 cm 以内）まで来たが、触れなかった", "位置の推定の誤差、腕の追従の誤差"),
    "not_reached": ("指先がボタンに近づけなかった", "位置の推定の大きな誤り、IK・届く範囲"),
    "unknown": ("分類できない（指先の位置を測れないシミュレーター）", ""),
}
BASIC_FEATURES: dict[str, str] = {
    # 本番に似た乗り場（elevator_prod.yaml）
    "wall.front_x": "壁までの距離 [m]",
    "column.protrusion": "柱の出っ張り [m]",
    "column.width": "柱の幅 [m]",
    "column.center_y": "柱の左右の位置 [m]（負 = 右）",
    "door.width": "扉の幅 [m]",
    "buttons.common.radius": "ボタンの半径 [m]",
    "buttons.common.protrusion": "ボタンの出っ張り [m]",
    "button_up_height": "一般用 ▲ の高さ（床から）[m]",
    "button_spacing": "▲ と ▼ の間隔 [m]",
    "button_wc_up_height": "車いす用 ▲ の高さ（床から）[m]",
    "floor_brightness": "床の明るさの倍率",
    "column_brightness": "柱の明るさの倍率",
    # 壁に付いた盤（elevator_hall.yaml）
    "wall_front_x": "壁までの距離 [m]",
    "panel_center_y": "盤の左右の位置 [m]（負 = 右）",
    "panel_center_height": "盤の高さ（床から）[m]",
}
SMALL_N = 20


def load_results(paths: list[Path]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    results: list[dict[str, Any]] = []
    meta: dict[str, Any] = {"files": [str(p) for p in paths]}
    for p in paths:
        d = json.loads(Path(p).read_text())
        for k in ("agent", "sim", "eval_set"):
            meta.setdefault(k, d.get(k))
        if d.get("ablation"):
            meta["ablation"] = True
        results += d["results"]
    return results, meta


def _rate(rs: list[dict[str, Any]]) -> float:
    return sum(r["outcome"] == "success" for r in rs) / len(rs) if rs else float("nan")


def _pct(x: float) -> str:
    return "-" if np.isnan(x) else f"{100 * x:.0f}%"


def condition_effects(results: list[dict[str, Any]], keys: list[str] | None = None) -> list[dict[str, Any]]:
    """条件ごとに、中央値で試行を 2 つに分けたときの成功率の差。差（の大きさ）が大きい順。keys で条件を絞れる。"""
    labels = {**BASIC_FEATURES, **{k: v[0] for k, v in FEATURES.items()}}
    if keys is not None:
        labels = {k: v for k, v in labels.items() if k in keys}
    rows = []
    for key, label in labels.items():
        vals = [(r.get("extra", {}).get("conditions", {}).get(key), r) for r in results]
        vals = [(v, r) for v, r in vals if v is not None]
        if len(vals) < 4:
            continue
        x = np.array([v for v, _ in vals], dtype=float)
        if np.ptp(x) <= 1e-12:
            continue  # この条件は変えていない
        med = float(np.median(x))
        low = [r for v, r in vals if v <= med]
        high = [r for v, r in vals if v > med]
        if not low or not high:
            continue
        rl, rh = _rate(low), _rate(high)
        rows.append({"key": key, "label": label, "median": med, "n_low": len(low), "n_high": len(high),
                     "rate_low": rl, "rate_high": rh, "diff": rh - rl})
    rows.sort(key=lambda r: -abs(r["diff"]))
    return rows


def severe_conditions(r: dict[str, Any], results: list[dict[str, Any]], top: int = 3) -> list[str]:
    """この試行で、ほかの試行と比べて特に厳しかった乱し（上位 20% に入るもの）。"""
    c = r.get("extra", {}).get("conditions", {})
    out = []
    for key, (label, _) in FEATURES.items():
        if key not in c:
            continue
        allv = np.array([x["extra"]["conditions"][key] for x in results if key in x.get("extra", {}).get("conditions", {})])
        if np.ptp(allv) <= 1e-12:
            continue
        pct = float((allv < c[key]).mean())
        if pct >= 0.8:
            out.append((pct, f"{label} = {c[key]:.3g}（上位 {100 * (1 - pct):.0f}%）"))
    out.sort(reverse=True)
    return [s for _, s in out[:top]]


def repro_command(meta: dict[str, Any], r: dict[str, Any]) -> str:
    sim = meta.get("sim") or r.get("sim")
    script = "evaluate_isaac.sh --viz kit" if sim == "isaac" else "evaluate.py --view"
    runner = "bash Button_Press/Yada/contest/" if sim == "isaac" else "$P Button_Press/Yada/contest/"
    s = f"{runner}{script} --agent {meta.get('agent')} --set {r.get('extra', {}).get('eval_set') or meta.get('eval_set') or 'basic'} --seed {r['seed']}"
    if r.get("extra", {}).get("ablation") not in (None, "all"):
        s += f" --ablation-only {r['extra']['ablation']}"
    return s


def build_report(results: list[dict[str, Any]], meta: dict[str, Any]) -> str:
    L: list[str] = []
    n = len(results)
    main = [r for r in results if r.get("extra", {}).get("ablation") in (None, "all")]
    L.append("# 弱点のレポート")
    L.append("")
    L.append(f"- エージェント: `{meta.get('agent')}`")
    L.append(f"- シミュレーター: {meta.get('sim')}、評価セット: {meta.get('eval_set')}")
    L.append(f"- 試行: {n}" + (f"（乱し {len({r.get('extra', {}).get('ablation') for r in results})} 通り × "
                               f"{len(main)}）" if meta.get("ablation") and main else "")
             + f"（作成 {time.strftime('%Y-%m-%d %H:%M')}）")
    if main:
        ok = [r for r in main if r["outcome"] == "success"]
        mt = f"、成功時の平均 {np.mean([r['time_s'] for r in ok]):.2f} 秒" if ok else ""
        L.append(f"- **成功率 {_pct(_rate(main))}**（{len(ok)} / {len(main)}{mt}）"
                 + ("。すべての乱しを入れた試行の値" if meta.get("ablation") else ""))
    L.append("")

    L.append("## 1. どこで失敗したか")
    L.append("")
    L.append("| 分類 | 回数 | 見直すところ（目安） |")
    L.append("|---|---|---|")
    stages: dict[str, int] = {}
    for r in main or results:
        stages[r.get("stage") or "unknown"] = stages.get(r.get("stage") or "unknown", 0) + 1
    for k in STAGE_LABELS:
        if k in stages:
            label, hint = STAGE_LABELS[k]
            L.append(f"| {label} | {stages[k]} | {hint} |")
    L.append("")

    if meta.get("ablation"):
        L.append("## 2. 乱しを 1 種類ずつ入れた評価（原因の切り分け）")
        L.append("")
        L.append("その種類の乱しだけを入れ、ほかは理想の値にして評価した。「乱しなし」より成功率が下がった種類が、弱点の原因。")
        L.append("")
        L.append("| 乱しの種類 | 成功率 | 乱しなしとの差 | 主な失敗 |")
        L.append("|---|---|---|---|")
        groups: dict[str, list[dict[str, Any]]] = {}
        for r in results:
            groups.setdefault(r.get("extra", {}).get("ablation", "all"), []).append(r)
        base = _rate(groups.get("none", []))
        order = ["none", *[g for g in GROUP_LABELS if g not in ("none", "all")], "all"]
        rows = []
        for g in order:
            if g not in groups:
                continue
            rate = _rate(groups[g])
            fails: dict[str, int] = {}
            for r in groups[g]:
                if r["outcome"] != "success":
                    fails[r.get("stage", "unknown")] = fails.get(r.get("stage", "unknown"), 0) + 1
            top = max(fails, key=fails.get) if fails else ""
            diff = "" if g == "none" or np.isnan(base) else f"{100 * (rate - base):+.0f} pt"
            rows.append((g, rate, diff, STAGE_LABELS.get(top, ("", ""))[0] if top else "-"))
        for g, rate, diff, top in rows:
            mark = " **←**" if diff and float(diff.split()[0]) <= -20 else ""
            L.append(f"| {GROUP_LABELS.get(g, g)} | {_pct(rate)}（{len(groups[g])} 試行） | {diff}{mark} | {top} |")
        L.append("")

    sec = 3 if meta.get("ablation") else 2
    L.append(f"## {sec}. 条件ごとの成功率")
    L.append("")
    if meta.get("ablation"):
        # 乱しを 1 種類だけ入れた試行の中で、その種類の値と成功の関係を見る（ほかの乱しが混ざらないので、原因に近い）
        L.append("乱しを 1 種類だけ入れた試行の中で、その乱しの値で試行を半分に分け（中央値で）、成功率を比べた。")
        L.append("「大きい側」で下がっていれば、その値のあたりから弱くなる。")
        L.append("")
        groups_r: dict[str, list[dict[str, Any]]] = {}
        for r in results:
            groups_r.setdefault(r.get("extra", {}).get("ablation", "all"), []).append(r)
        any_row = False
        for g, rs in groups_r.items():
            if g in ("none", "all") or _rate(rs) >= 1.0:
                continue
            keys = [k for k, (_, grp) in FEATURES.items() if grp == g]
            eff = condition_effects(rs, keys)
            if not eff:
                continue
            any_row = True
            L.append(f"**{GROUP_LABELS.get(g, g)}**（成功率 {_pct(_rate(rs))}、{len(rs)} 試行）")
            L.append("")
            L.append("| 条件 | 中央値 | 小さい側の成功率 | 大きい側の成功率 | 差 |")
            L.append("|---|---|---|---|---|")
            for e in eff:
                L.append(f"| {e['label']} | {e['median']:.3g} | {_pct(e['rate_low'])}（{e['n_low']}） | "
                         f"{_pct(e['rate_high'])}（{e['n_high']}） | {100 * e['diff']:+.0f} pt |")
            L.append("")
        if not any_row:
            L.append("成功率が下がった乱しが無い。")
    else:
        eff = condition_effects(main or results)
        if not eff:
            L.append("条件を変えた試行が足りないので、集計していない。")
        else:
            L.append("条件の値で試行を半分に分け（中央値で）、小さい側と大きい側の成功率を比べた。差の大きい順。")
            L.append("一緒に変わったほかの条件の影響も混ざるので、原因とは限らない（原因の切り分けは `evaluate.py --ablation`）。")
            if len(main or results) < SMALL_N:
                L.append(f"**試行が {len(main or results)} 回しかないので、差はたまたまのことがある。**")
            L.append("")
            L.append("| 条件 | 中央値 | 小さい側の成功率 | 大きい側の成功率 | 差 |")
            L.append("|---|---|---|---|---|")
            for e in eff:
                mark = " **←**" if abs(e["diff"]) >= 0.3 else ""
                L.append(f"| {e['label']} | {e['median']:.3g} | {_pct(e['rate_low'])}（{e['n_low']}） | "
                         f"{_pct(e['rate_high'])}（{e['n_high']}） | {100 * e['diff']:+.0f} pt{mark} |")
    L.append("")

    fails = [r for r in (main or results) if r["outcome"] != "success"]
    L.append(f"## {sec + 1}. 失敗した試行")
    L.append("")
    if not fails:
        L.append("なし。")
    else:
        L.append("再現のコマンドの `$P` は MuJoCo の Python（例: `~/miniconda3/envs/lerobot/bin/python`）。")
        L.append("")
        for r in fails[:30]:
            d = r.get("extra", {}).get("diag", {})
            dist = d.get("min_tip_dist_target_m")
            facts = [STAGE_LABELS.get(r.get("stage", ""), (r.get("stage", ""), ""))[0]]
            if dist is not None:
                facts.append(f"指先とボタンの最も近い距離 {1000 * dist:.0f} mm")
            if d.get("max_depth_target_mm"):
                facts.append(f"ボタンの最大の沈み {d['max_depth_target_mm']:.1f} mm")
            if r.get("max_contact_force_n"):
                facts.append(f"接触力 {r['max_contact_force_n']:.0f} N")
            sev = severe_conditions(r, main or results)
            L.append(f"- **seed {r['seed']}**（{r['target']}）: " + "、".join(facts))
            if sev:
                L.append(f"  - 厳しかった条件: " + "、".join(sev))
            L.append(f"  - 再現: `{repro_command(meta, r)}`")
        if len(fails) > 30:
            L.append(f"- ほか {len(fails) - 30} 試行")
    L.append("")
    return "\n".join(L)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", nargs="+", type=Path, help="evaluate.py が保存した結果の JSON")
    ap.add_argument("--out", default="", help="レポートの保存先（既定は _local/button_press_yada/reports/）")
    args = ap.parse_args()
    results, meta = load_results(args.results)
    md = build_report(results, meta)
    out = Path(args.out) if args.out else REPORT_DIR / f"{args.results[0].stem}.md"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(md, encoding="utf-8")
    print(md)
    print(f"[report] 保存した: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
