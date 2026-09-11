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
