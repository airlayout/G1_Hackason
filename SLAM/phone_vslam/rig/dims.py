"""スマホ・カバー・体・GoPro の寸法（単位 mm）と、その出典。

座標の決め方（スマホを入れた袋の中）:
    x: 左右（着た人の右が +）。スマホの幅の向き
    y: 前後（着た人の前が +）。スマホの背面（カメラ）が +y を向く
    z: 上下（上が +）。スマホの下の端が z=0

「背面から見て」の座標（Apple の図面の書き方）は、背面の左上を原点に右へ bx、下へ by。
背面を前から見るので、背面から見た左は着た人の右（+x）になる。
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Lens:
    """背面のレンズ 1 つ。円錐は「この中に物を置くと写り込む」範囲（Apple の keep-out cone）。"""

    name: str
    bx: float  # 背面から見て左の端からの距離
    by: float  # 上の端からの距離
    cone_deg: float  # 円錐の全角
    apex_from_front: float  # 円錐の頂点の、画面のガラス（前の面）からの距離（図の "KEEPOUT CONE TO COVER GLASS"）


@dataclass(frozen=True)
class Button:
    """側面のボタン 1 つ。side は画面から見た側（'L' 左・'R' 右）。from/to は上の端からの距離。"""

    name: str
    side: str
    top_from: float
    top_to: float


@dataclass(frozen=True)
class Phone:
    name: str
    height: float
    width: float
    thickness: float  # 本体の厚み（カメラの出っ張りを除く）
    turret: float  # カメラの出っ張りの高さ（背面から）
    cam_box: tuple[float, float, float, float]  # 背面から見たカメラの範囲 (bx0, by0, bx1, by1)
    lenses: tuple[Lens, ...]
    buttons: tuple[Button, ...]
    official_parts: str  # どこまでが公式の数字か
    source: str


@dataclass(frozen=True)
class Case:
    """カバーを付けた外形。official が False のものは見積もり（試し刷りで確かめる）。"""

    height: float
    width: float
    thickness: float
    official: bool
    source: str


# --- iPhone 11 Pro（Apple 公式の寸法図）-----------------------------------------------
# Accessory Design Guidelines for Apple Devices（2020-06-22 版）39.3 iPhone 11 Pro（PDF 145 ページ）。
# 今の版（2026）の Dimensional Drawings からは iPhone 11 Pro が外されたため、Wayback Machine の
# 2020-10-12 の写し（https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf）から読んだ。
IPHONE_11_PRO = Phone(
    name="iPhone 11 Pro",
    height=143.99,
    width=71.36,
    thickness=8.10,
    turret=1.21,  # REAR CAMERA TURRET HEIGHT
    cam_box=(3.23, 3.23, 30.59, 32.71),  # Detail D の出っ張りの外形
    lenses=(
        Lens("広角（主）", 12.04, 12.04, 79.0, 5.91),  # REAR CAMERA 1
        Lens("望遠", 12.04, 27.06, 45.0, 2.93),  # REAR CAMERA 2
        Lens("超広角", 25.04, 19.55, 122.0, 7.12),  # REAR CAMERA 3
    ),
    buttons=(
        # 画面から見て左: 消音スイッチ 24.72・音量 + 37.84・音量 − 51.37（中心）。長さは ±5.71／±5.68
        Button("消音スイッチ", "L", 24.72 - 5.71, 24.72 + 5.71),
        Button("音量 +", "L", 37.84 - 5.68, 37.84 + 5.68),
        Button("音量 −", "L", 51.37 - 5.68, 51.37 + 5.68),
        # 画面から見て右: サイドボタン 44.61（中心）±8.77
        Button("サイドボタン", "R", 44.61 - 8.77, 44.61 + 8.77),
    ),
    official_parts="外形・カメラ・ボタンすべて公式",
    source="Apple Accessory Design Guidelines 2020-06-22 §39.3",
)

# --- Pixel 7a（外形だけ公式）------------------------------------------------------------
# 外形は Google の Pixel 7a tech specs（support.google.com/pixelphone/answer/7158570）。
# カメラの帯とボタンの位置は Google が公開していない（Made for Google の取引先向け）。
# → 袋はカメラの帯（上の 40 mm）とボタンの帯（上から 15〜105 mm の右側）を広く避ける形にし、
#   正確な位置に頼らない。下の値は「避ける範囲」で、位置そのものではない。
PIXEL_7A = Phone(
    name="Pixel 7a",
    height=152.0,
    width=72.9,
    thickness=9.0,
    turret=1.5,  # 公式なし。見積もり
    cam_box=(0.0, 8.0, 72.9, 40.0),  # 公式なし。帯は幅いっぱい・上の方にある
    lenses=(),
    buttons=(Button("電源・音量（範囲の見積もり）", "R", 15.0, 105.0),),
    official_parts="外形だけ公式（カメラ・ボタンの位置は非公開）",
    source="Google Pixel 7a tech specs",
)

# --- カバー ---------------------------------------------------------------------------
# Google 純正 Pixel 7a Case: 長さ 156.9・横 77.4・高さ 11.5 mm・31.3 g（Google ストアの商品ページ）
PIXEL_7A_CASE = Case(156.9, 77.4, 11.5, official=True, source="Google ストア Pixel 7a Case")

# iPhone 11 Pro のカバーの外形は公式の数字が無い。Google 純正カバーの足し分
# （縦 +4.9・横 +4.5・厚み +2.5）を iPhone 11 Pro の本体に足した見積もり。
IPHONE_11_PRO_CASE = Case(
    IPHONE_11_PRO.height + 4.9,
    IPHONE_11_PRO.width + 4.5,
    IPHONE_11_PRO.thickness + 2.5,
    official=False,
    source="見積もり（Google 純正カバーの足し分を流用）",
)


# --- 締め具 ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Hardware:
    hole: float = 5.3  # M5 のボルトが通る穴
    nut_af: float = 8.3  # M5 ナットの二面幅（8 mm）＋隙間
    nut_depth: float = 4.2  # ナットの厚み（4 mm）＋隙間
    strap_w: float = 26.0  # 25 mm の帯が通る穴
    strap_t: float = 4.0


HW = Hardware()


@dataclass(frozen=True)
class Printer:
    """Bambu Lab A2L の造形範囲。"""

    x: float = 330.0
    y: float = 320.0
    z: float = 325.0


A2L = Printer()


@dataclass(frozen=True)
class PhoneSlot:
    """袋 1 つ分の設定。side は胸の板の上で左（−1）か右（+1）か。外側＝回す軸の側。"""

    phone: Phone
    case: Case
    side: int  # 左 −1・右 +1（着た人から見て）
    label: str = field(default="")


LEFT = PhoneSlot(IPHONE_11_PRO, IPHONE_11_PRO_CASE, side=-1, label="iphone11pro_left")
RIGHT = PhoneSlot(PIXEL_7A, PIXEL_7A_CASE, side=+1, label="pixel7a_right")
