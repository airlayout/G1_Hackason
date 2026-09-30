"""G1 の FSM ID 定義（Mac 側サーバと Jetson 側ヘルパーで共有する情報）。

出典: unitree_sdk2_python の g1_loco_client.py（0/1/3/500/702/706）と、
Unitree 公式マニュアル系ドキュメント（501 = 腰3自由度版の通常モード）。
実機で GetFsmId が 501 を返すことは確認済み。
"""

# 表示用ラベル（GetFsmId の値 → 名前）
FSM_LABELS = {
    0: "ゼロトルク",
    1: "ダンピング",
    3: "座位",
    500: "通常歩行（腰1軸）",
    501: "通常歩行（腰3軸）",
    702: "床から起立",
    706: "しゃがみ⇔起立",
}

# 切り替えボタン（順序どおりに表示）。danger=True は歩行中に押すと警告を出す。
# 「準備(ロック立位)」「Run(801)」「デバッグ」は ID/手順が未確認のため未搭載。
BUTTONS = [
    {"id": 1, "label": "ダンピング", "danger": True},
    {"id": 0, "label": "ゼロトルク", "danger": True},
    {"id": 3, "label": "座位", "danger": False},
    {"id": 706, "label": "しゃがみ⇔起立", "danger": False},
    {"id": 702, "label": "床から起立", "danger": False},
    {"id": 501, "label": "通常歩行", "danger": False},
]

ALLOWED_IDS = frozenset(b["id"] for b in BUTTONS)
