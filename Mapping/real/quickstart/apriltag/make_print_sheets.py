#!/usr/bin/env python3
"""AprilTag の貼り紙と、カメラ校正ボードの PDF を作る。

## なぜ自前で作るのか

ネットで拾った PNG を印刷すると**一辺の実寸が分からなくなる**。姿勢推定は
「黒い正方形の一辺 [m]」を入力に取るので、ここが 1 mm 違うと距離が同じ割合で
ずれる（3 m で 160 mm のタグなら 1 mm の誤りが 19 mm の距離誤差になる）。
だから**寸法を紙に刷り込み、等倍で刷れたことを紙の上で確かめられる**ものを作る。

## 出るもの

- `tags_<family>_<size>mm_<paper>.pdf` … 1 ページ 1 枚。表紙に貼り方の指示
- `charuco_<paper>.pdf` … 内部パラメータ（fx, fy, cx, cy）を測る校正ボード

## 使い方

    Navigation/.venv/bin/python make_print_sheets.py --out-dir ./print
    Navigation/.venv/bin/python make_print_sheets.py --paper a3 --tag-mm 240 --ids 0-7

⚠️ **印刷時は「実際のサイズ」「100%」を選ぶ。**「用紙に合わせる」は倍率が変わる。
刷ったら紙の上の 100 mm 定規をものさしで当てて確かめること。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
# ⚠️ 既定の Type3 は PostScript 名をそのまま PDF の Name にするため、
# Hiragino のように名前が日本語のフォントで UnicodeEncodeError になる。42 にする。
matplotlib.rcParams["pdf.fonttype"] = 42
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
try:
    from jp_font import japanese_font
except ImportError:  # 単体で持ち出したとき用
    def japanese_font():
        return None

PAPERS_MM = {"a4": (210.0, 297.0), "a3": (297.0, 420.0)}
FAMILIES = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
}
MM_PER_INCH = 25.4
# 校正ボード。A4 横（297x210）で 224x140 mm。下に定規を置く余白を残す。
CHARUCO_SQUARES = (8, 5)
CHARUCO_SQUARE_MM = 28.0
CHARUCO_MARKER_MM = 21.0
CHARUCO_DPI = 600


def parse_ids(text: str) -> list[int]:
    """`0-7` `0,3,5` `0-3,10` のどれでも ID の一覧にする。"""

    ids: list[int] = []
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk[1:]:
            start, _, end = chunk.partition("-")
            ids.extend(range(int(start), int(end) + 1))
        else:
            ids.append(int(chunk))
    return ids


def merge_runs(grid: np.ndarray) -> list[tuple[int, int, int, int]]:
    """0/1 の格子を、極大な長方形の集合へまとめる。

    モジュールを 1 個ずつ長方形で描くと、PDF ビューアによっては境目に白い髪の毛が
    出る。まとめておけば境目そのものが減る。返すのは (col, row, width, height)。
    """

    taken = np.zeros_like(grid, dtype=bool)
    rectangles: list[tuple[int, int, int, int]] = []
    rows, cols = grid.shape
    for row in range(rows):
        for col in range(cols):
            if grid[row, col] == 0 or taken[row, col]:
                continue
            width = 0
            while col + width < cols and grid[row, col + width] == 1 and not taken[row, col + width]:
                width += 1
            height = 1
            while row + height < rows and np.all(grid[row + height, col:col + width] == 1) \
                    and not np.any(taken[row + height, col:col + width]):
                height += 1
            taken[row:row + height, col:col + width] = True
            rectangles.append((col, row, width, height))
    return rectangles


def new_page(pdf: PdfPages, width_mm: float, height_mm: float):
    """紙 1 枚ぶんの図を作る。座標は mm、左下が原点。"""

    figure = plt.figure(figsize=(width_mm / MM_PER_INCH, height_mm / MM_PER_INCH))
    axes = figure.add_axes((0.0, 0.0, 1.0, 1.0))
    axes.set_xlim(0.0, width_mm)
    axes.set_ylim(0.0, height_mm)
    axes.set_aspect("equal")
    axes.axis("off")
    return figure, axes


def draw_scale_bar(axes, x_mm: float, y_mm: float, length_mm: float = 100.0) -> None:
    """等倍で刷れたことを紙の上で確かめるための定規。"""

    axes.plot([x_mm, x_mm + length_mm], [y_mm, y_mm], color="0.25", lw=0.8,
              solid_capstyle="butt")
    for offset in range(0, int(length_mm) + 1, 10):
        tall = offset % 50 == 0
        axes.plot([x_mm + offset, x_mm + offset], [y_mm, y_mm + (3.5 if tall else 2.0)],
                  color="0.25", lw=0.8)
    axes.text(x_mm + length_mm / 2, y_mm - 4.5,
              f"{length_mm:.0f} mm — ものさしで確かめる（合わなければ等倍で刷り直す）",
              ha="center", va="top", fontsize=7, color="0.25")


def draw_tag_page(pdf: PdfPages, dictionary, family: str, tag_id: int,
                  tag_mm: float, paper: str) -> None:
    """タグ 1 枚のページを描く。"""

    width_mm, height_mm = PAPERS_MM[paper]
    figure, axes = new_page(pdf, width_mm, height_mm)

    # 36h11 は 6x6 のデータ + 1 モジュールの黒枠 = 8x8 モジュール。
    modules = dictionary.markerSize + 2
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, modules, borderBits=1)
    grid = (marker == 0).astype(np.uint8)  # 黒いモジュールを 1 にする
    module_mm = tag_mm / modules

    center_x = width_mm / 2.0
    center_y = height_mm / 2.0 + 12.0
    left = center_x - tag_mm / 2.0
    bottom = center_y - tag_mm / 2.0

    for col, row, span_x, span_y in merge_runs(grid):
        axes.add_patch(Rectangle(
            (left + col * module_mm, bottom + (modules - row - span_y) * module_mm),
            span_x * module_mm, span_y * module_mm,
            facecolor="black", edgecolor="none", antialiased=False))

    # 白の余白（1 モジュール）の外に、中心と四隅を測るための印を置く。
    quiet_mm = module_mm
    mark = 6.0
    gap = quiet_mm + 2.0
    for (x0, y0, x1, y1) in (
        (center_x, bottom - gap, center_x, bottom - gap - mark),
        (center_x, bottom + tag_mm + gap, center_x, bottom + tag_mm + gap + mark),
        (left - gap, center_y, left - gap - mark, center_y),
        (left + tag_mm + gap, center_y, left + tag_mm + gap + mark, center_y),
    ):
        axes.plot([x0, x1], [y0, y1], color="0.55", lw=0.6)

    axes.text(center_x, bottom + tag_mm + gap + mark + 5.0,
              f"{family}   ID {tag_id}", ha="center", va="bottom",
              fontsize=13, color="black")
    axes.text(center_x, bottom - gap - mark - 6.0,
              f"黒い正方形の一辺 = {tag_mm:.1f} mm（白の余白は含まない）",
              ha="center", va="top", fontsize=9.5, color="black")
    axes.text(center_x, bottom - gap - mark - 13.0,
              "貼ったら 貼付記録シート に ID・中心の高さ・向き を書く",
              ha="center", va="top", fontsize=7.5, color="0.35")
    draw_scale_bar(axes, center_x - 50.0, 22.0)
    pdf.savefig(figure)
    plt.close(figure)


def draw_record_page(pdf: PdfPages, ids: list[int], paper: str) -> None:
    """貼った場所を書き留める表。あとで地図に登録するときの元になる。"""

    width_mm, height_mm = PAPERS_MM[paper]
    figure, axes = new_page(pdf, width_mm, height_mm)
    axes.text(18.0, height_mm - 22.0, "貼付記録シート", fontsize=16, va="top")
    axes.text(18.0, height_mm - 33.0,
              "貼ったその場で書く。高さは 床からタグの中心まで。向きは「タグが向いている方角」",
              fontsize=8.5, va="top", color="0.35")

    columns = [("ID", 14.0), ("貼った面（壁/机/柱/床）", 52.0), ("中心の高さ mm", 30.0),
               ("向き（北/東/…や目印）", 46.0), ("備考", 32.0)]
    x = 18.0
    top = height_mm - 44.0
    row_h = 11.0
    for name, span in columns:
        axes.text(x + 2.0, top + 3.0, name, fontsize=8, va="bottom", color="0.2")
        x += span
    total = sum(span for _, span in columns)
    for index in range(len(ids) + 1):
        y = top - index * row_h
        axes.plot([18.0, 18.0 + total], [y, y], color="0.6", lw=0.5)
    x = 18.0
    for _, span in columns:
        axes.plot([x, x], [top, top - len(ids) * row_h], color="0.6", lw=0.5)
        x += span
    axes.plot([x, x], [top, top - len(ids) * row_h], color="0.6", lw=0.5)
    for index, tag_id in enumerate(ids):
        axes.text(20.0, top - index * row_h - row_h / 2, str(tag_id),
                  fontsize=9, va="center", color="0.2")

    y = top - len(ids) * row_h - 18.0
    for text in (
        "地図に登録するときの手順（この紙は捨てない）",
        "  1. ロボットを起こし、測位が合っていることを 重畳 で確かめる",
        "  2. タグが見える場所へ立たせ、record_tag_session.sh で 10 秒録る",
        "  3. register_tags.py が タグの地図座標を出す。この紙の高さと突き合わせる",
    ):
        axes.text(18.0, y, text, fontsize=8.5, va="top", color="0.2")
        y -= 7.0
    pdf.savefig(figure)
    plt.close(figure)


def draw_cover_page(pdf: PdfPages, family: str, tag_mm: float, ids: list[int],
                    paper: str, max_range_m: float) -> None:
    """貼り方の指示を 1 ページ目に置く。紙束だけ持って現場に行けるようにする。"""

    width_mm, height_mm = PAPERS_MM[paper]
    figure, axes = new_page(pdf, width_mm, height_mm)
    lines = [
        ("t", "AprilTag 貼付の手引き"),
        ("s", f"{family} / 一辺 {tag_mm:.0f} mm / ID {ids[0]}〜{ids[-1]} の {len(ids)} 枚"),
        ("b", "1. 印刷"),
        ("n", "・「実際のサイズ」「100%」で刷る。「用紙に合わせる」は倍率が変わるので使わない"),
        ("n", "・刷ったら各ページ下の 100 mm 定規をものさしで当てる。合わなければ刷り直す"),
        ("n", "・光沢紙は照明が映り込んで読めなくなる。普通紙かマット紙を使う"),
        ("b", "2. 貼り方は 2 通り（2026-09-15 に実測した値）"),
        ("n", "・床に平らに置く … 機体の前 2.0 m まで 検出率 100%・誤差 8 mm。2.5 m から落ちる"),
        ("n", "・立てて貼る　　 … 3.5 m まで 検出率 100%。机の側面・壁の下部・柱"),
        ("n", "・床置きは奥行きが潰れるので、紙を大きくしても距離は伸びない"),
        ("b", "3. 立てるときの高さ  ← いちばん効く"),
        ("n", "・タグの中心を 床から 0.3〜0.7 m にする"),
        ("n", "・0.9 m を超えると 3 m 以遠でカメラの画面から外れる（頭カメラは下向き）"),
        ("b", "4. 床に 2 枚置く場合（推奨の最小構成）"),
        ("n", "・機体の前 1.6〜2.4 m、横に 1.0〜2.5 m 離して置く → 2 枚同時に視野へ入る"),
        ("n", "・置いたら 2 枚の中心どうしの間隔を巻尺で測って書き留める"),
        ("n", "　（登録した地図座標と突き合わせる。1 枚だとこの検算ができない）"),
        ("b", "5. 共通"),
        ("n", "・四辺をテープで留めて平らにする。中央だけ留めると膨らむ"),
        ("n", "・同じ ID を 2 枚貼らない。1 枚ずつ別の場所へ"),
        ("n", "・床置きは踏まれる・椅子で隠れる。人の動線を外す"),
        ("n", "・貼ったら次ページ以降と最終ページの記録シートに書く"),
        ("b", "やってはいけないこと"),
        ("n", "・拡大縮小して刷る / ラミネートする（照り返しで読めない）"),
        ("n", "・窓ぎわの直射日光が当たる面に貼る / 動く物（椅子・台車）に貼る"),
    ]
    y = height_mm - 26.0
    for kind, text in lines:
        if kind == "t":
            axes.text(20.0, y, text, fontsize=17, va="top"); y -= 11.0
        elif kind == "s":
            axes.text(20.0, y, text, fontsize=10, va="top", color="0.35"); y -= 13.0
        elif kind == "b":
            y -= 3.0
            axes.text(20.0, y, text, fontsize=11.5, va="top"); y -= 8.0
        else:
            axes.text(23.0, y, text, fontsize=9, va="top", color="0.15"); y -= 6.6
    pdf.savefig(figure)
    plt.close(figure)


def draw_charuco_page(pdf: PdfPages, paper: str) -> Path | None:
    """内部パラメータを測る ChArUco ボードを 1 ページ描く（横向き）。"""

    height_mm, width_mm = PAPERS_MM[paper]  # 横向き
    figure, axes = new_page(pdf, width_mm, height_mm)
    board = cv2.aruco.CharucoBoard(
        CHARUCO_SQUARES, CHARUCO_SQUARE_MM / 1000.0, CHARUCO_MARKER_MM / 1000.0,
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    board_w_mm = CHARUCO_SQUARES[0] * CHARUCO_SQUARE_MM
    board_h_mm = CHARUCO_SQUARES[1] * CHARUCO_SQUARE_MM
    pixels = (int(round(board_w_mm / MM_PER_INCH * CHARUCO_DPI)),
              int(round(board_h_mm / MM_PER_INCH * CHARUCO_DPI)))
    image = board.generateImage(pixels)

    left = (width_mm - board_w_mm) / 2.0
    bottom = (height_mm - board_h_mm) / 2.0 + 14.0
    axes.imshow(image, cmap="gray", vmin=0, vmax=255, interpolation="none",
                extent=(left, left + board_w_mm, bottom, bottom + board_h_mm),
                zorder=2)
    axes.text(width_mm / 2, bottom + board_h_mm + 6.0,
              f"ChArUco {CHARUCO_SQUARES[0]}x{CHARUCO_SQUARES[1]}  "
              f"square {CHARUCO_SQUARE_MM:.0f} mm / marker {CHARUCO_MARKER_MM:.0f} mm  "
              "(DICT_4X4_50)",
              ha="center", va="bottom", fontsize=10)
    axes.text(width_mm / 2, bottom - 7.0,
              "カメラの内部パラメータを測る板。硬い板に貼って たわませない",
              ha="center", va="top", fontsize=9, color="0.2")
    draw_scale_bar(axes, width_mm / 2 - 50.0, bottom - 24.0)
    pdf.savefig(figure)
    plt.close(figure)
    return None


def estimate_range(tag_mm: float, image_width: int, horizontal_fov_deg: float,
                   min_pixels: float) -> float:
    """タグ一辺が min_pixels に写る距離。"""

    focal_px = (image_width / 2.0) / np.tan(np.deg2rad(horizontal_fov_deg) / 2.0)
    return float(focal_px * (tag_mm / 1000.0) / min_pixels)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", default="print", help="PDF の置き場")
    parser.add_argument("--family", default="36h11", choices=sorted(FAMILIES))
    parser.add_argument("--ids", default="0-7", help="例 0-7 / 0,3,5 / 0-3,10")
    parser.add_argument("--paper", default="a4", choices=sorted(PAPERS_MM))
    parser.add_argument("--tag-mm", type=float, default=None,
                        help="黒い正方形の一辺 [mm]。既定は用紙ごとの推奨値")
    parser.add_argument("--image-width", type=int, default=1280,
                        help="読める距離の見積りに使う横画素数")
    parser.add_argument("--hfov-deg", type=float, default=87.0,
                        help="読める距離の見積りに使う水平画角")
    parser.add_argument("--no-charuco", action="store_true")
    arguments = parser.parse_args()

    japanese_font()
    tag_mm = arguments.tag_mm or {"a4": 160.0, "a3": 240.0}[arguments.paper]
    paper_w = PAPERS_MM[arguments.paper][0]
    if tag_mm * (1.0 + 2.0 / (6 + 2)) > paper_w:
        parser.error(f"一辺 {tag_mm:.0f} mm は {arguments.paper} に白の余白ごと入らない "
                     f"（上限 {paper_w / 1.25:.0f} mm）")

    ids = parse_ids(arguments.ids)
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[arguments.family])
    if max(ids) >= dictionary.bytesList.shape[0]:
        parser.error(f"{arguments.family} の ID は 0〜{dictionary.bytesList.shape[0] - 1}")

    out_dir = Path(arguments.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # 2026-09-15 に実機の IR 画像へ合成して測った限界（range_study.py）。
    # 24 px が読める下限、30 px が余裕を見た目安。
    safe_range = estimate_range(tag_mm, arguments.image_width, arguments.hfov_deg, 30.0)
    limit_range = estimate_range(tag_mm, arguments.image_width, arguments.hfov_deg, 24.0)

    tag_pdf = out_dir / f"tags_{arguments.family}_{tag_mm:.0f}mm_{arguments.paper}.pdf"
    with PdfPages(tag_pdf) as pdf:
        draw_cover_page(pdf, arguments.family, tag_mm, ids, arguments.paper, safe_range)
        for tag_id in ids:
            draw_tag_page(pdf, dictionary, arguments.family, tag_id, tag_mm,
                          arguments.paper)
        draw_record_page(pdf, ids, arguments.paper)
    print(f"[OK] {tag_pdf}  … 表紙 + タグ {len(ids)} 枚 + 貼付記録シート")
    print(f"     一辺 {tag_mm:.0f} mm / {arguments.image_width} px / 画角 {arguments.hfov_deg:.0f}deg "
          f"→ 確実に読めるのは {safe_range:.1f} m まで（限界 {limit_range:.1f} m）")

    if not arguments.no_charuco:
        charuco_pdf = out_dir / f"charuco_{arguments.paper}.pdf"
        with PdfPages(charuco_pdf) as pdf:
            draw_charuco_page(pdf, arguments.paper)
        print(f"[OK] {charuco_pdf}  … 校正ボード 1 枚")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
