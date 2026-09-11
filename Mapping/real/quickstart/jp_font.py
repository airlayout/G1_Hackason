#!/usr/bin/env python3
"""matplotlib の図に日本語を出すためのフォント選び。

## なぜ独立したファイルなのか

同じ 10 行が `plot_gs_comparison.py` / `plot_gs_intensity.py` /
`identify_follower_mask.py` / `render_inflation_sweep.py` に散っていた。
そのうち `plot_gs_comparison.py` は **gsplat を import する**ので、
Navigation の venv からは読めない（2026-09-11 に踏んだ）。
描画ヘルパを重い解析スクリプトに置いておくと使い回せない。

⚠️ 残る 3 つはまだ各自のコピーを持っている。触る機会があればここへ寄せる。
"""
from __future__ import annotations

# 上から順に試す。macOS は Hiragino、Linux は Noto を想定
CANDIDATES = ("Hiragino Sans", "Hiragino Kaku Gothic ProN",
              "Noto Sans CJK JP", "IPAexGothic", "Arial Unicode MS")

# ⚠️ **日本語の等幅フォントは期待しないこと。** 2026-09-11 にこの Mac で確認した結果、
# Osaka-Mono / HackGen / Source Han Mono / Noto Sans Mono CJK JP / BIZ UDGothic は
# どれも入っていない（あるのは Hiragino Sans と Menlo だけ）。
# `family="monospace"` を指定すると Menlo が選ばれ、**日本語が全部豆腐になる**。
# 数字を揃えたいときは family を指定せず、書式（{:>6.1f} など）で桁を揃える。
MONO_CANDIDATES = ("Osaka-Mono", "HackGen", "Source Han Mono",
                   "Noto Sans Mono CJK JP", "BIZ UDGothic")


def japanese_mono():
    """日本語が出せる等幅フォント。**無ければ None**（上の注記のとおり普通は無い）。"""
    from matplotlib import font_manager
    available = {f.name for f in font_manager.fontManager.ttflist}
    for candidate in MONO_CANDIDATES:
        if candidate in available:
            return candidate
    return None


def japanese_font():
    """使えるフォントを 1 つ選んで rcParams に入れ、**選んだ名前を返す**。

    ⚠️ 見つからないまま描くと日本語が全部豆腐になる。呼び出し側が気づけるよう
    None を返す（黙って既定のまま進めない）。
    """
    import matplotlib
    from matplotlib import font_manager

    available = {f.name for f in font_manager.fontManager.ttflist}
    for candidate in CANDIDATES:
        if candidate in available:
            matplotlib.rcParams["font.family"] = candidate
            return candidate
    return None
