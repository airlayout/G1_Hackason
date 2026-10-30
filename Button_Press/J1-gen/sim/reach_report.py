"""押せる範囲の地図（reach_map.py・elevator_stand_map.py の map.csv）を、上から見た図と数字の表にまとめる。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/reach_report.py \\
        --bottle 既定=_local/button_press/reach_map/20261029_001221 \\
        --bottle 広げた設定=_local/button_press/reach_map/20261030_wide \\
        --elevator _local/button_press/stand_map/20261029_005937

出力: _local/button_press/reach_report/<日時>/report.html（ブラウザで開く）と summary.md（数字だけ）。
図はどれも「上から見た図」。ロボットは図の下にいて、上（前）を向いている。左右は見たままの向き（図の左がロボットの左）。
- ボトル: 縦 = 机の手前の縁までの距離、横 = ボトルの左右。全体の図は、縁からボトルまでの距離・天板の高さ・縁の傾きの
  組み合わせ（既定の格子で 16 通り）のうち、いくつで「計画が通り、頭カメラの画像にも入る」かを数で出す
- エレベーター: 縦 = 柱（ボタン）までの距離、横 = ボタンが体の中心からどちらにあるか。全体の図は、柱の面の傾き（5 通り）のうち、いくつで通るか
matplotlib は日本語の字体が無いと文字化けするので、図は SVG を HTML に直接書く（ブラウザが日本語を表示する）。
"""

from __future__ import annotations

import argparse
import csv
import html
import time
from collections import Counter
from pathlib import Path
from typing import Callable

REPO = Path(__file__).resolve().parents[3]

# マスの色（通る / 計画は通るがカメラに入らない / 机・柱に当たる / 体に当たる / 届かないなど）
COLORS = {"ok": "#2e9d5b", "novis": "#e3b23c", "desk": "#9aa5b8", "body": "#c96a6a", "other": "#d9d9d9"}
LABELS = {"ok": "通る（計画もカメラも）", "novis": "計画は通るが、カメラに入らない", "desk": "机・柱・壁に当たる",
          "body": "体に当たる", "other": "届かない・関節が急に動く・作業空間の外"}
STAGE_TEXT = {"W": "作業空間の外", "R": "手前の姿勢に届かない", "A": "手前の姿勢で当たる", "V": "経由の姿勢で当たる",
              "1": "開始→経由の途中で当たる", "2": "経由→手前の途中で当たる", "D": "開始→手前の途中で当たる（経由なし）", "P": "押し込みで届かない",
              "Q": "押し込みで当たる", "J": "関節が急に動く", "K": "手前へ動く途中で柱・壁に当たる"}
CELL = 34


def load(path: Path) -> list[dict[str, str]]:
    """map.csv を読む（フォルダを渡したら中の map.csv）。"""
    p = path / "map.csv" if path.is_dir() else path
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def status(row: dict[str, str]) -> str:
    """1 通りの結果を、色分けの区分にする。"""
    if row["plan_ok"] == "1":
        visible = row["visible"] == "1" if "visible" in row else (row["vis_buttons"] == "1" and row["vis_region"] == "1")
        return "ok" if visible else "novis"
    if row["partner"] in ("desk", "wall"):
        return "desk"
    if row["partner"] == "body":
        return "body"
    return "other"


def side(y: float) -> str:
    """左右を言葉にする（y は左が +）。"""
    return "中央" if abs(y) < 1e-9 else (f"左 {y * 100:.0f} cm" if y > 0 else f"右 {-y * 100:.0f} cm")


def svg_grid(xs: list[float], ys: list[float], cell: Callable[[float, float], tuple[str, str]],
             xlabel: str, ylabel: str) -> str:
    """上から見た図（縦 = 前方 x、上ほど遠い。横 = 左右 y、左ほどロボットの左）を SVG で書く。
    cell(x, y) は (色, マスの中の文字)。"""
    xs_d, ys_d = sorted(xs, reverse=True), sorted(ys, reverse=True)
    left, top = 58, 8
    w, h = left + CELL * len(ys_d) + 8, top + CELL * len(xs_d) + 56
    out = [f'<svg width="{w}" height="{h}" viewBox="0 0 {w} {h}" role="img">']
    for i, x in enumerate(xs_d):
        yy = top + i * CELL
        out.append(f'<text x="{left - 6}" y="{yy + CELL / 2 + 4}" text-anchor="end" class="ax">{x * 100:.0f}</text>')
        for j, y in enumerate(ys_d):
            color, text = cell(x, y)
            xx = left + j * CELL
            # マスは数が多い（1 万個ほど）ので、大きさ・角の丸み・文字の位置は CSS（.c・.cv）に任せて短く書く
            out.append(f'<rect class="c" x="{xx}" y="{yy}" fill="{color}"/>')
            if text:
                out.append(f'<text class="cv" x="{xx + (CELL - 2) // 2}" y="{yy + CELL // 2 + 3}">{html.escape(text)}</text>')
    by = top + CELL * len(xs_d)
    for j, y in enumerate(ys_d):
        out.append(f'<text x="{left + j * CELL + CELL / 2 - 1}" y="{by + 14}" text-anchor="middle" class="ax">{side(y).replace(" cm", "").replace(" ", "")}</text>')
    out.append(f'<text x="{left + CELL * len(ys_d) / 2}" y="{by + 32}" text-anchor="middle" class="axl">{html.escape(ylabel)}</text>')
    out.append(f'<text x="{left + CELL * len(ys_d) / 2}" y="{by + 50}" text-anchor="middle" class="axl">▲ ロボット（前＝図の上）</text>')
    out.append(f'<text x="12" y="{top + CELL * len(xs_d) / 2}" text-anchor="middle" class="axl" '
               f'transform="rotate(-90 12 {top + CELL * len(xs_d) / 2})">{html.escape(xlabel)}</text>')
    out.append("</svg>")
    return "".join(out)


def fraction_color(n: int, total: int) -> str:
    """通った割合を緑の濃さにする（0 は灰色）。"""
    if n == 0:
        return COLORS["other"]
    a = 0.25 + 0.75 * n / total
    return f"rgba(46,157,91,{a:.2f})"


def reason_counts(rows: list[dict[str, str]]) -> list[tuple[str, int]]:
    c = Counter()
    for r in rows:
        if r["plan_ok"] != "1":
            who = {"desk": "机", "wall": "柱・壁", "body": "体"}.get(r["partner"], "")
            c[STAGE_TEXT.get(r["stage"], r["stage"]) + (f"（{who}）" if who else "")] += 1
        elif status(r) == "novis":
            c["計画は通るが、カメラに入らない"] += 1
    return c.most_common()


def ranges_by_y(rows: list[dict[str, str]], xkey: str, ys: list[float], groups: int) -> list[tuple[float, str, str]]:
    """左右ごとに、「全部の条件で通る」距離と「どれか 1 つでも通る」距離を cm の範囲で出す。"""
    xs = sorted({float(r[xkey]) for r in rows})
    step = min(b - a for a, b in zip(xs, xs[1:])) if len(xs) > 1 else 1.0
    out = []
    for y in sorted(ys, reverse=True):
        per_x: dict[float, int] = Counter()
        for r in rows:
            if float(r["y"]) == y and status(r) == "ok":
                per_x[float(r[xkey])] += 1
        out.append((y, spans([x for x, n in per_x.items() if n == groups], step),
                    spans(list(per_x), step)))
    return out


def spans(vals: list[float], step: float) -> str:
    """距離の一覧を、続いている範囲ごとに cm で書く（例: 18〜24・28〜40 cm）。間が抜けていれば分けて書く。"""
    if not vals:
        return "なし"
    vals = sorted(vals)
    groups = [[vals[0]]]
    for v in vals[1:]:
        if v - groups[-1][-1] > step * 1.5:
            groups.append([v])
        else:
            groups[-1].append(v)
    return "・".join(f"{g[0] * 100:.0f}" if len(g) == 1 else f"{g[0] * 100:.0f}〜{g[-1] * 100:.0f}" for g in groups) + " cm"


def legend() -> str:
    return '<div class="legend">' + "".join(
        f'<span><i style="background:{COLORS[k]}"></i>{v}</span>' for k, v in LABELS.items()) + "</div>"


def bottle_section(name: str, rows: list[dict[str, str]], md: list[str]) -> str:
    edges = sorted({float(r["edge"]) for r in rows})
    ys = sorted({float(r["y"]) for r in rows})
    conds = sorted({(float(r.get("slope", 0) or 0), float(r["top"]), float(r["gap"])) for r in rows})
    ok = sum(status(r) == "ok" for r in rows)
    plan = sum(r["plan_ok"] == "1" for r in rows)
    idx = {(float(r["edge"]), float(r["y"]), float(r.get("slope", 0) or 0), float(r["top"]), float(r["gap"])): r for r in rows}

    def overview(x: float, y: float) -> tuple[str, str]:
        n = sum(status(idx[(x, y, *c)]) == "ok" for c in conds if (x, y, *c) in idx)
        return fraction_color(n, len(conds)), str(n)

    parts = [f"<h3>{html.escape(name)}</h3>",
             f'<p class="num"><b>{ok}</b> / {len(rows)} 通りで通る（計画が通るのは {plan}。差の {plan - ok} はボトルが頭カメラの画像から出る）</p>',
             '<div class="row"><div>',
             f"<h4>全体（{len(conds)} 条件のうち通った数）</h4>",
             svg_grid(edges, ys, overview, "机の手前の縁までの距離 [cm]", "ボトルの左右 [cm]"),
             "</div><div><h4>断られた理由</h4><table>" +
             "".join(f"<tr><td>{html.escape(k)}</td><td class='n'>{v}</td></tr>" for k, v in reason_counts(rows)) +
             "</table><h4>ボトルの左右ごとの範囲（机の縁）</h4><table><tr><th>ボトル</th><th>全部の条件で通る</th><th>どれかで通る</th></tr>" +
             "".join(f"<tr><td>{side(y)}</td><td>{a}</td><td>{b}</td></tr>" for y, a, b in ranges_by_y(rows, "edge", ys, len(conds))) +
             "</table></div></div>"]
    tops = sorted({c[1] for c in conds})
    md_out: list[str] = []
    if len(tops) > 2:
        parts.append(height_block(rows, idx, conds, edges, ys, tops, md_out))
    parts.append(f"<details><summary>条件ごとの図（{len(conds)} 枚）</summary>{legend()}<div class='grid'>")
    for c in conds:
        slope, top, gap = c

        def cell(x: float, y: float, c=c) -> tuple[str, str]:
            r = idx.get((x, y, *c))
            if r is None:
                return "#fff", ""
            s = status(r)
            text = "" if s == "ok" else ("o" if s == "novis" else r["stage"])
            return COLORS[s], text
        parts.append(f"<div><h5>縁の傾き {slope:+.3f}・天板 z {top:+.2f} m・縁からボトル {gap * 100:.0f} cm</h5>"
                     f"{svg_grid(edges, ys, cell, '机の縁 [cm]', 'ボトルの左右 [cm]')}</div>")
    parts.append("</div></details>")
    md += [f"### ボトル: {name}", "", f"- 通る: **{ok} / {len(rows)}**（計画が通るのは {plan}。差はカメラの画像から出る分）", "",
           "| ボトルの左右 | 全部の条件で通る机の縁 | どれかで通る机の縁 |", "|---|---|---|"]
    md += [f"| {side(y)} | {a} | {b} |" for y, a, b in ranges_by_y(rows, "edge", ys, len(conds))]
    md += ["", "断られた理由: " + "、".join(f"{k} {v}" for k, v in reason_counts(rows)), ""] + md_out
    return "".join(parts)


def height_block(rows: list[dict[str, str]], idx: dict, conds: list[tuple[float, float, float]], edges: list[float],
                 ys: list[float], tops: list[float], md: list[str]) -> str:
    """机の高さ（天板）ごとの全体の図と、高さ × 左右の範囲の表。床からの高さは pelvis が床から約 0.73 m として換算する。"""
    parts = ["<h4>机の高さごと</h4><p class='note'>床からの高さは、pelvis が床から約 0.73 m として換算。"
             "各図は、その高さでの条件（縁からボトルまでの距離 × 縁の傾き）のうち通った数。</p><div class='grid'>"]
    head = "".join(f"<th>{t + 0.73:.2f} m</th>" for t in tops)
    count_row, rng_rows = [], {y: [] for y in sorted(ys, reverse=True)}
    for t in tops:
        sub = [c for c in conds if c[1] == t]
        rs = [r for r in rows if float(r["top"]) == t]

        def overview(x: float, y: float, sub=sub) -> tuple[str, str]:
            n = sum(status(idx[(x, y, *c)]) == "ok" for c in sub if (x, y, *c) in idx)
            return fraction_color(n, len(sub)), str(n)
        ok = sum(status(r) == "ok" for r in rs)
        nov = sum(status(r) == "novis" for r in rs)
        parts.append(f"<div><h5>床から {t + 0.73:.2f} m（天板 z {t:+.2f}）: {ok}/{len(rs)}</h5>"
                     f"{svg_grid(edges, ys, overview, '机の縁 [cm]', 'ボトルの左右 [cm]')}</div>")
        count_row.append(f"{ok}/{len(rs)}（カメラ外 {nov}）")
        for y, _a, b in ranges_by_y(rs, "edge", ys, len(sub)):
            rng_rows[y].append(b)
    parts.append("</div><h4>机の高さ × ボトルの左右: どれかの条件で通る机の縁</h4><table><tr><th></th>" + head + "</tr>"
                 + "<tr><td>通る数</td>" + "".join(f"<td>{c}</td>" for c in count_row) + "</tr>"
                 + "".join(f"<tr><td>{side(y)}</td>" + "".join(f"<td>{v}</td>" for v in vs) + "</tr>" for y, vs in rng_rows.items())
                 + "</table>")
    md += ["#### 机の高さごと（床から。どれかの条件で通る机の縁）", "",
           "| | " + " | ".join(f"{t + 0.73:.2f} m" for t in tops) + " |", "|---" * (len(tops) + 1) + "|",
           "| 通る数 | " + " | ".join(count_row) + " |"]
    md += [f"| {side(y)} | " + " | ".join(vs) + " |" for y, vs in rng_rows.items()]
    md.append("")
    return "".join(parts)


def elevator_section(rows: list[dict[str, str]], md: list[str]) -> str:
    parts: list[str] = []
    md += ["### エレベーター（車いす用の ▲▼、立つ位置）", ""]
    for target, tname in (("down", "▼（床から約 87 cm）"), ("up", "▲（床から約 93.5 cm）")):
        rs = [r for r in rows if r["target"] == target]
        if not rs:
            continue
        xs = sorted({float(r["x"]) for r in rs})
        ys = sorted({float(r["y"]) for r in rs})
        tilts = sorted({float(r["tilt"]) for r in rs})
        idx = {(float(r["x"]), float(r["y"]), float(r["tilt"])): r for r in rs}
        ok = sum(status(r) == "ok" for r in rs)

        def overview(x: float, y: float) -> tuple[str, str]:
            n = sum(status(idx[(x, y, t)]) == "ok" for t in tilts if (x, y, t) in idx)
            rec = 0.29 <= x <= 0.40 and abs(y) <= 0.06 + 1e-9
            return fraction_color(n, len(tilts)), str(n) + ("*" if rec else "")
        parts += [f"<h3>{tname}</h3>", f'<p class="num"><b>{ok}</b> / {len(rs)} 通りで通る</p>', '<div class="row"><div>',
                  f"<h4>全体（柱の面の傾き {len(tilts)} 通りのうち通った数。* は今の勧める範囲）</h4>",
                  svg_grid(xs, ys, overview, "柱（ボタン）までの距離 [cm]", "ボタンが体の中心からどちらか [cm]"),
                  "</div><div><h4>断られた理由</h4><table>" +
                  "".join(f"<tr><td>{html.escape(k)}</td><td class='n'>{v}</td></tr>" for k, v in reason_counts(rs)) +
                  "</table><h4>左右ごとの範囲（柱までの距離）</h4><table><tr><th>ボタン</th><th>全部の傾きで通る</th><th>どれかで通る</th></tr>" +
                  "".join(f"<tr><td>{side(y)}</td><td>{a}</td><td>{b}</td></tr>" for y, a, b in ranges_by_y(rs, "x", ys, len(tilts))) +
                  "</table></div></div>", f"<details><summary>傾きごとの図（{len(tilts)} 枚）</summary>{legend()}<div class='grid'>"]
        for t in tilts:
            def cell(x: float, y: float, t=t) -> tuple[str, str]:
                r = idx.get((x, y, t))
                if r is None:
                    return "#fff", ""
                s = status(r)
                text = "" if s == "ok" else ("o" if s == "novis" else r["stage"])
                return COLORS[s], text
            parts.append(f"<div><h5>柱の面の傾き {t:+.0f}°</h5>{svg_grid(xs, ys, cell, '柱までの距離 [cm]', 'ボタンの左右 [cm]')}</div>")
        parts.append("</div></details>")
        md += [f"#### {tname}", "", f"- 通る: **{ok} / {len(rs)}**", "",
               "| ボタンの左右 | 全部の傾きで通る距離 | どれかで通る距離 |", "|---|---|---|"]
        md += [f"| {side(y)} | {a} | {b} |" for y, a, b in ranges_by_y(rs, "x", ys, len(tilts))]
        md += ["", "断られた理由: " + "、".join(f"{k} {v}" for k, v in reason_counts(rs)), ""]
    return "".join(parts)


CSS = """
:root{--bg:#fff;--fg:#1f2328;--mut:#59636e;--line:#d0d7de}
@media (prefers-color-scheme:dark){:root{--bg:#16181d;--fg:#e6e8eb;--mut:#9aa4af;--line:#3a3f47}}
body{background:var(--bg);color:var(--fg);font-family:system-ui,"Noto Sans JP","Yu Gothic UI",sans-serif;margin:0 auto;max-width:1200px;padding:16px;line-height:1.6}
h2{border-bottom:1px solid var(--line);padding-bottom:4px;margin-top:40px}h5{margin:8px 0 2px;font-weight:600}
.row{display:flex;flex-wrap:wrap;gap:32px;align-items:flex-start}.grid{display:flex;flex-wrap:wrap;gap:16px}
.num{font-size:1.1em}.num b{font-size:1.5em}
table{border-collapse:collapse;font-size:.92em}td,th{border:1px solid var(--line);padding:3px 8px;text-align:left}td.n{text-align:right}
.ax{font-size:11px;fill:var(--mut)}.axl{font-size:12px;fill:var(--fg)}.c{width:32px;height:32px;rx:3px}.cv{font-size:11px;fill:#111;pointer-events:none;text-anchor:middle}
.legend{display:flex;flex-wrap:wrap;gap:14px;margin:8px 0;font-size:.9em}.legend i{display:inline-block;width:12px;height:12px;border-radius:2px;margin-right:4px;vertical-align:-1px}
.note{color:var(--mut);font-size:.92em}details{margin:12px 0}summary{cursor:pointer;font-weight:600}
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bottle", action="append", default=[], metavar="名前=フォルダ", help="reach_map.py の結果（何個でも）")
    ap.add_argument("--elevator", type=Path, help="elevator_stand_map.py の結果")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    out = args.out or REPO / "_local/button_press/reach_report" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)

    md = ["# 押せる範囲のまとめ", "", f"作成: {time.strftime('%Y-%m-%d %H:%M')}。計画だけ（IK・衝突・経由の姿勢）と、頭カメラ（URDF の約 48° 下向き）に入るか。"
          "MuJoCo・実機では未確認。図は report.html。", ""]
    body = [f"<h1>押せる範囲のまとめ</h1><p class='note'>作成 {time.strftime('%Y-%m-%d %H:%M')}。計画だけ（IK・腕と体・机の衝突・経由の姿勢）と、"
            "頭カメラ（URDF の約 48° 下向き、640×360）に入るかで判定。MuJoCo・実機では未確認。"
            "図はどれも上から見た図で、ロボットは図の下から上（前）を向いている。マスの中の英字は断られた段階（凡例は条件ごとの図の上）。</p>"]
    if args.bottle:
        body.append("<h2>ボトル（机の縁とボトルの位置）</h2><p class='note'>縦 = pelvis から机の手前の縁まで（ボトルの手前の面は縁から 0〜12 cm 奥）。"
                    "横 = ボトルの左右。条件 = 縁からボトルまでの距離 × 天板の高さ × 縁の傾き。</p>")
        md += ["## ボトル", ""]
        for spec in args.bottle:
            name, _, path = spec.partition("=")
            rows = load(REPO / path if not Path(path).is_absolute() else Path(path))
            body.append(bottle_section(name, rows, md))
            print(f"[reach_report] ボトル {name}: {len(rows)} 通り")
    if args.elevator:
        p = args.elevator if args.elevator.is_absolute() else REPO / args.elevator
        rows = load(p)
        body.append("<h2>エレベーター（立つ位置）</h2><p class='note'>縦 = pelvis から柱の面（ボタン）まで。横 = ボタンが体の中心からどちらにあるか。"
                    "条件 = 柱の面の傾き（−10〜+10°）。</p>")
        md += ["## エレベーター", ""]
        body.append(elevator_section(rows, md))
        print(f"[reach_report] エレベーター: {len(rows)} 通り")
    page = (f"<!doctype html><html lang='ja'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            f"<title>押せる範囲のまとめ</title><style>{CSS}</style></head><body>{''.join(body)}</body></html>")
    (out / "report.html").write_text(page, encoding="utf-8")
    (out / "summary.md").write_text("\n".join(md), encoding="utf-8")
    print(f"[reach_report] 保存: {out}/report.html・summary.md")


if __name__ == "__main__":
    main()
