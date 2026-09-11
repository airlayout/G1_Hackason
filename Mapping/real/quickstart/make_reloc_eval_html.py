#!/usr/bin/env python3
"""段 8 の検証結果を作業ログの HTML にまとめる（動画を埋め込む）。

数字は `patterns.json` から読む。**手で書き写さない**（写し間違いが一番こわい）。

    Navigation/.venv/bin/python quickstart/make_reloc_eval_html.py \\
        runs/reloc_eval_20260911 \\
        --out ../../../docs/作業ログ/2026-09-11_2_Global-Localization_6パターン検証.html
"""
from __future__ import annotations

import argparse
import html
import json
import os
import sys
from pathlib import Path

CSS = """
:root{--ground:#f6f8fa;--surface:#fff;--surface-2:#eef1f6;--ink:#111620;--ink-2:#46505f;
--ink-3:#6b7686;--line:#dce1e9;--accent:#0f6fc4;--accent-soft:#e7f1fb;--warn:#b1500f;
--warn-soft:#fbeee2;--good:#14714a;--good-soft:#e3f2ea;--bad:#a8202c;--bad-soft:#f9e6e7;
--sans:"IBM Plex Sans JP",-apple-system,"Hiragino Sans","Noto Sans JP",sans-serif;
--mono:"IBM Plex Mono",ui-monospace,Menlo,monospace;--wide:1180px;--r:6px}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--ground:#0e1116;
--surface:#151a22;--surface-2:#1b212b;--ink:#e6ebf2;--ink-2:#aab4c2;--ink-3:#7c8593;
--line:#28313d;--accent:#4da3e8;--accent-soft:#12263a;--warn:#e08a3c;--warn-soft:#2e2015;
--good:#4cbf8d;--good-soft:#12291f;--bad:#e8737d;--bad-soft:#2e1618}}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--ink);font-family:var(--sans);
line-height:1.75;font-size:16px}
main{max-width:var(--wide);margin:0 auto;padding:2.5rem 1.25rem 5rem}
h1{font-size:1.7rem;line-height:1.4;margin:0 0 .4rem}
h2{font-size:1.2rem;margin:2.6rem 0 .8rem;padding-top:.8rem;border-top:1px solid var(--line)}
h3{font-size:1.02rem;margin:1.6rem 0 .5rem}
.meta{color:var(--ink-3);font-size:.9rem;margin-bottom:1.6rem}
.lead{background:var(--surface);border:1px solid var(--line);border-left:4px solid var(--accent);
border-radius:var(--r);padding:1rem 1.2rem;margin:1.2rem 0 2rem}
.verdict{font-weight:700;font-size:1.05rem}
table{border-collapse:collapse;width:100%;font-size:.9rem;margin:.8rem 0 1.4rem;
background:var(--surface);border:1px solid var(--line);border-radius:var(--r);overflow:hidden}
th,td{padding:.5rem .7rem;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
th{background:var(--surface-2);font-weight:600;white-space:nowrap}
td.n{font-family:var(--mono);text-align:right;white-space:nowrap}
tr:last-child td{border-bottom:none}
.ok{color:var(--good);font-weight:600}.ng{color:var(--bad);font-weight:600}
.wk{color:var(--warn);font-weight:600}
.card{background:var(--surface);border:1px solid var(--line);border-radius:var(--r);
padding:1rem 1.1rem;margin:1.4rem 0}
.card.pass{border-left:4px solid var(--good)}
.card.fail{border-left:4px solid var(--bad)}
video{width:100%;max-width:980px;border:1px solid var(--line);border-radius:var(--r);
background:#000;display:block;margin:.6rem 0}
.note{background:var(--warn-soft);border-left:3px solid var(--warn);padding:.65rem .9rem;
border-radius:4px;margin:.8rem 0;font-size:.92rem}
code{font-family:var(--mono);font-size:.88em;background:var(--surface-2);padding:.1em .35em;
border-radius:3px}
.scroll{overflow-x:auto}
"""


def cls(r: dict) -> str:
    return "ok" if (r["gate3d"] and r["gate2d"]) else "ng"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    base = a.dir.resolve()
    meta = json.loads((base / "patterns.json").read_text())
    out = a.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    vids = {p.name.split("_")[0].replace("pattern", ""): p for p in sorted(base.glob("pattern*.mp4"))}
    rel = os.path.relpath(base, out.parent)

    accepted = [r for r in meta["results"] if r["gate3d"] and r["gate2d"]]
    wrong_accept = [r for r in accepted
                    if r.get("error_m", 0) > 1.0 or r["overlay_frac"] < meta["overlay_frac"]]

    e = html.escape
    P = []
    P.append("<!doctype html><html lang=ja><head><meta charset=utf-8>")
    P.append('<meta name=viewport content="width=device-width, initial-scale=1">')
    P.append("<title>Global Localization — 記録での 6 パターン検証（段 8）</title>")
    P.append('<link rel=preconnect href="https://fonts.googleapis.com">')
    P.append('<link rel=stylesheet href="https://fonts.googleapis.com/css2?'
             'family=IBM+Plex+Sans+JP:wght@400;500;600;700&family=IBM+Plex+Mono:wght@400;500&display=swap">')
    P.append("<style>" + CSS + "</style></head><body><main>")
    P.append("<h1>Global Localization — 記録での 6 パターン検証（段 8 / 8b）</h1>")
    P.append('<p class=meta>2026-09-11 ／ 素材: 静止の対照 '
             '<code>stage_20260910T084053</code> と歩行 3 本 '
             '<code>stage_20260910T182639_r{1,2,3}</code> ／ 機体は不要（記録のみ）</p>')

    # ── 結論を先頭に ──
    P.append('<div class=lead><p class=verdict>結論: '
             '<span class=ng>門はまだ通っていない。</span>'
             'ただし<span class=ok>「黙って間違える」失敗は 9/9 で消えた。</span></p>')
    P.append("<ul>")
    P.append("<li><b>安全性（誤採用ゼロ）</b>: 二重ゲートで 9/9。"
             "段 8 では r3 が <code>r/r0 1.02</code>（較正値より良い）で採用されていたが、"
             "2D 重畳が 36.9% と食い違うため棄却されるようになった。</li>")
    P.append("<li><b>復帰</b>: 静止系（1・2・3・6）は 4/4 で真値水準に戻る。"
             "<b>机 1 台が動いた場合（5a）も戻る。</b></li>")
    P.append("<li><b>戻れないもの</b>: 帯を丸ごと動かした過酷版（5b）と、"
             "<b>歩行のすべった終端（r1/r2/r3）は 0/3</b>。"
             "ただし全部ゲートが止めたので、間違った場所で再開はしない。</li>")
    P.append("<li><b>原因は特定済み</b>: 到達しうる重畳の天井の位置が、"
             "信じていた姿勢から r1 8.6 m / r2 8.0 m / r3 4.0 m ＝ <b>ROI ±3 m の外</b>。"
             "ROI を 10 m に広げると一致率が 1〜14% に落ちる（探索が地図の外へ出る）ので、"
             "<b>地図の有効域に探索を閉じ込める</b>のが次の一手。</li>")
    P.append("</ul></div>")

    # ── 二重ゲートの説明 ──
    P.append("<h2>判定の作り — 二重ゲート</h2>")
    P.append("<p>探索は「一番尤もらしい格子点」を必ず返す。つまり<b>返ってきたこと自体は何の保証でもない</b>。"
             "そこで、<b>別々の地図を見る 2 つのゲート</b>で採否を決める。片方が壊れてももう片方で気づける。</p>")
    P.append("<div class=scroll><table><tr><th>ゲート</th><th>見る地図</th><th>使う点</th>"
             "<th>基準</th><th>棄却</th></tr>")
    P.append("<tr><td>① 3D 残差</td><td><code>map.mm</code>（MOLA が測位に使う 3D）</td>"
             "<td>壁の帯 z&gt;{:.1f} m</td><td class=n>r0 = {:.4f} m</td>"
             "<td class=n>r &gt; {:.1f} × r0</td></tr>".format(
                 meta["band_lo"], meta["r0"], meta["results"][0]["gate_n"]))
    P.append("<tr><td>② 2D 重畳</td><td>旧 <code>nav_map</code>（間引いていない 2D）</td>"
             "<td>帯 0.02〜1.82 m</td><td class=n>O0 = {:.1f} %</td>"
             "<td class=n>重畳 &lt; {:.2f} × O0</td></tr>".format(
                 meta["o0"], meta["overlay_frac"]))
    P.append("</table></div>")
    P.append('<div class=note>⚠️ <b>O0 は「同じ 1 枚のスキャンを真値に置いたときの値」（{:.1f}%）。</b>'
             '91 枚の平均 78.2% ではない。1 枚と平均を混ぜると判定が甘くなる。</div>'.format(meta["o0"]))
    P.append('<div class=note>⚠️ <b>探索は全点、ゲートは壁の帯。使う点群が逆になる。</b>'
             '帯で探索すると 1.6〜2.9 m 外し、全点でゲートすると 0.5 m の誤りが 1.45 倍にしか'
             'ならず弾けない（段 6 実測）。</div>')

    # ── 一覧表 ──
    P.append("<h2>一覧</h2><div class=scroll><table>")
    P.append("<tr><th>#</th><th>内容</th><th>真値との差</th><th>① r/r0</th>"
             "<th>② 重畳</th><th>対 O0</th><th>探索</th><th>判定</th></tr>")
    for r in meta["results"]:
        err = "{:.2f} m".format(r["error_m"]) if "error_m" in r else "—"
        P.append("<tr><td class=n>{}</td><td>{}</td><td class=n>{}</td><td class=n>{:.2f}</td>"
                 "<td class=n>{:.1f} %</td><td class=n>{:.2f}</td><td class=n>{:.2f} s</td>"
                 "<td class={}>{}</td></tr>".format(
                     e(r["id"]), e(r["name"]), err, r["ratio"], r["overlay_pct"],
                     r["overlay_frac"], r["search_seconds"], cls(r), e(r["final"])))
    P.append("</table></div>")

    # ── 各パターン ──
    P.append("<h2>1 パターン = 1 本</h2>")
    P.append("<p>左が事前地図（灰＝占有）。<b>赤丸＝門番が鳴った時点で信じていた姿勢</b>、"
             "<b>青丸＝探索が返した姿勢</b>、<b>緑星＝真値</b>。点群は置いた姿勢での重畳で、"
             "<span class=ok>緑＝占有セルに乗った</span> / <span class=ng>赤＝外れた</span>。"
             "右は尤度の格子（探索が何を見て決めたか）。</p>")
    for r in meta["results"]:
        v = vids.get(r["id"])
        ok = r["gate3d"] and r["gate2d"]
        P.append('<div class="card {}">'.format("pass" if ok else "fail"))
        P.append("<h3>パターン {} — {}</h3>".format(e(r["id"]), e(r["name"])))
        P.append("<p>期待していたこと: {} ／ 結果: <span class={}>{}</span></p>".format(
            e(r["expect"]), cls(r), e(r["final"])))
        if v:
            P.append('<video controls preload=metadata src="{}/{}"></video>'.format(
                e(rel), e(v.name)))
        P.append("<div class=scroll><table>")
        P.append("<tr><th>信じていた姿勢</th><td class=n>({:.2f}, {:.2f})</td>"
                 "<th>探索が返した姿勢</th><td class=n>({:.2f}, {:.2f}, {:.1f}°)</td></tr>".format(
                     r["believed"][0], r["believed"][1], r["pose"][0], r["pose"][1], r["pose"][2]))
        if "truth" in r:
            P.append("<tr><th>真値</th><td class=n>({:.2f}, {:.2f}, {:.1f}°)</td>"
                     "<th>真値との差</th><td class=n>{:.3f} m / {:.2f}°</td></tr>".format(
                         r["truth"][0], r["truth"][1], r["truth"][2],
                         r["error_m"], r["error_deg"]))
        P.append("<tr><th>① 3D 残差</th><td class=n>{:.4f} m（r0 の {:.2f} 倍）</td>"
                 "<th>一致率</th><td class=n>{:.1f} %</td></tr>".format(
                     r["residual"], r["ratio"], 100 * r["match_rate"]))
        P.append("<tr><th>② 2D 重畳</th><td class=n>{:.1f} %（O0 の {:.2f}）</td>"
                 "<th>探索の所要</th><td class=n>{:.2f} s</td></tr>".format(
                     r["overlay_pct"], r["overlay_frac"], r["search_seconds"]))
        P.append("</table></div></div>")

    P.append("<h2>この先</h2><ul>")
    P.append("<li><b>段 8 をやり直して門を通す</b>には、歩行のすべった終端から戻れるようにする必要がある。"
             "探索を地図の有効域に閉じ込めたうえで ROI を広げるのが次の一手。</li>")
    P.append("<li>この記録には <b>IMU が入っていない</b>（2026-09-11 に判明）ため、"
             "再生は 09-10 の live の再現にならない。機体が使えるようになったら "
             "<b>IMU 入りの記録 1 本</b>と<b>静止の対照を数本</b>取るのが最優先。</li>")
    P.append("<li>本計画は回り道で、目的は "
             "<code>2026-09-09_2-first-short-walk-and-map-goals.md</code> の段 3 ＝ "
             "<b>RViz2 の 2D Goal Pose をクリックして G1 を歩かせる</b>こと。</li>")
    P.append("</ul></main></body></html>")

    out.write_text("\n".join(P), encoding="utf-8")
    print("書いた: {}".format(out))
    print("  動画 {} 本 / 採用 {} 件（うち疑わしい採用 {} 件）".format(
        len(vids), len(accepted), len(wrong_accept)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
