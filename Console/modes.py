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

# 未実装のモード切替（画面に「未実装」で無効表示する）。理由は REMOTE_CONTROLLER.md 参照。
PLANNED_MODES = [
    {"label": "準備（固定立位）", "note": "FSM ID が未確認"},
    {"label": "Run", "note": "FSM ID が未確認（801 は資料 1 件のみ）"},
    {"label": "デバッグモード", "note": "切替手順が未確認"},
]

# 未実装のアクション（リモコンの SELECT+Y/A/X 相当）。SDK では LocoClient.SetTaskId で実現する見込み。
PLANNED_ACTIONS = [
    {"label": "手を振る", "note": "SetTaskId(0)"},
    {"label": "握手", "note": "SetTaskId(2/3)"},
    {"label": "振り向いて手を振る", "note": "SetTaskId(1)"},
]

# 基本機能の状態。REMOTE_CONTROLLER.md の表と同じ内容を保つこと。
# implemented: 実装済みか / verified: 実機（G1 電源オン、コンソール経由）で試したか
FEATURES = [
    {"id": "connection", "label": "接続状態表示", "implemented": True, "verified": False},
    {"id": "reconnect", "label": "自動再接続", "implemented": True, "verified": False},
    {"id": "mode", "label": "モード変更", "implemented": True, "verified": False,
     "note": "ダンピング/ゼロトルク/座位/しゃがみ⇔起立/床から起立/通常歩行のみ"},
    {"id": "action", "label": "アクション実行", "implemented": False, "verified": False},
    {"id": "move", "label": "矢印キーで移動", "implemented": False, "verified": False},
    {"id": "battery", "label": "バッテリー表示", "implemented": False, "verified": False,
     "note": "取得方法（トピック）が未確認"},
]
