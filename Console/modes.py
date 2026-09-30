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
    {"id": "battery", "label": "バッテリー表示", "implemented": True, "verified": False,
     "note": "rt/lf/bmsstate。電圧・電流の単位は推定"},
    {"id": "audio", "label": "音声・LED（音量・頭部 LED・読み上げ）", "implemented": True, "verified": False,
     "note": "実機で音量取得・LED・読み上げを送信済み（デバッグモード）。LED は読み戻し不可で目視、読み上げは rt/audio_msg の play_state で開始/終了を観測可。ユーザー確認は未完"},
    {"id": "sensors", "label": "デバッグ情報表示（IMU・関節・指令・オドメトリ・Jetson・DDS 受信状況）", "implemented": True,
     "verified": False, "note": "lidar・カメラは対象外。関節名の対応・メインボード value の意味は未確認"},
]


# 音声・LED（モーションに関係しない書き込み）。範囲外は helper に渡さず弾く。
TTS_MAX_CHARS = 100
TTS_SPEAKERS = (0, 1)  # SDK のサンプルで使われる値。0=中国語, 1=英語（と推定、実機で未確認）


def validate_audio(kind: str, body: dict) -> dict:
    """POST 本文を検証し、helper に渡すリクエストを返す。不正なら ValueError。"""
    def integer(key, lo, hi):
        v = body[key]
        if isinstance(v, bool) or not isinstance(v, int) or not lo <= v <= hi:
            raise ValueError("%s は %d〜%d の整数" % (key, lo, hi))
        return v
    try:
        if kind == "volume":
            return {"op": "audio_volume", "volume": integer("volume", 0, 100)}
        if kind == "led":
            return {"op": "audio_led", "r": integer("r", 0, 255), "g": integer("g", 0, 255), "b": integer("b", 0, 255)}
        if kind == "tts":
            text = body["text"]
            if not isinstance(text, str) or not text.strip() or len(text) > TTS_MAX_CHARS:
                raise ValueError("text は 1〜%d 文字" % TTS_MAX_CHARS)
            speaker = integer("speaker_id", min(TTS_SPEAKERS), max(TTS_SPEAKERS))
            return {"op": "audio_tts", "text": text, "speaker_id": speaker}
    except KeyError as exc:
        raise ValueError("%s がありません" % exc.args[0])
    raise ValueError("unknown audio kind")
