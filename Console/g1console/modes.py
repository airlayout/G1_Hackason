"""G1 の FSM ID 定義（操作用 PC 側サーバと Jetson 側ヘルパーで共有する情報）。

出典: unitree_sdk2_python の g1_loco_client.py（0/1/3/500/702/706）と、
Unitree 公式マニュアル系ドキュメント（501 = 腰3自由度版の通常モード）。
実機で GetFsmId が 501 を返した記録: 別スクリプトによる 1 回と、2026-10-18 のコンソール経由（リモコンで通常モードにした後）。
"""

# 表示用ラベル（GetFsmId の値 → 名前）
FSM_LABELS = {
    0: "ゼロトルク",
    1: "ダンピング",
    3: "座位",
    4: "固定立位（準備）",  # 2026-10-18 実機: リモコンで固定立位にした直後に 4 を確認（1 回のみ）
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

# 未実装のモード切替（画面に「未実装」で無効表示する）。理由は docs/REMOTE_CONTROLLER.md 参照。
PLANNED_MODES = [
    {"label": "準備（固定立位）", "note": "FSM ID が未確認"},
    {"label": "Run", "note": "FSM ID が未確認（801 は資料 1 件のみ）"},
    {"label": "デバッグモード", "note": "切替手順が未確認"},
]

# アクション（リモコンの SELECT+Y/A/X 相当）。LocoClient.SetTaskId で実行する。task_id と効果は実機未検証。
# 握手は開始（2）と終了（3）の 2 段階。終了を送れるよう両方を載せる。
ACTIONS = [
    {"id": "wave", "label": "手を振る", "task_id": 0},
    {"id": "wave_turn", "label": "振り向いて手を振る", "task_id": 1},
    {"id": "shake_start", "label": "握手（開始）", "task_id": 2},
    {"id": "shake_end", "label": "握手（終了）", "task_id": 3},
]
PLANNED_ACTIONS = []  # 未実装のアクションは今は無い（API の形は保つ）

# 基本機能の状態。REMOTE_CONTROLLER.md の表と同じ内容を保つこと。
# implemented: 実装済みか / verified: 実機（G1 電源オン、コンソール経由）で試したか
FEATURES = [
    {"id": "connection", "label": "接続状態表示", "implemented": True, "verified": False},
    {"id": "reconnect", "label": "自動再接続", "implemented": True, "verified": False},
    {"id": "mode", "label": "モード変更", "implemented": True, "verified": False,
     "note": "ダンピング/ゼロトルク/座位/しゃがみ⇔起立/床から起立/通常歩行のみ"},
    {"id": "action", "label": "アクション実行", "implemented": True, "verified": False,
     "note": "手を振る・握手。SetTaskId の番号と効果は実機未検証。--local のときだけ"},
    {"id": "move", "label": "移動（ジョイスティック・矢印キー）", "implemented": True, "verified": False,
     "note": "--local のときだけ。接続断は 0.5 秒で停止。速度上限・FSM 500/501 での動作は実機未検証"},
    {"id": "tasks", "label": "タスク実行（定義済みコマンドの起動・停止・ログ）", "implemented": True, "verified": False,
     "note": "Console/tasks/*.json の argv だけを実行する。Mac のテストでは確認済みだが、Jetson 上での実行は未確認"},
    {"id": "camera", "label": "カメラの映像表示と、配信の起動・停止", "implemented": True, "verified": False,
     "note": "--local のときだけ起動・停止できる。Jetson 上での起動・停止は未確認"},
    {"id": "lidar", "label": "lidar の点群表示と、配信の起動・停止", "implemented": True, "verified": False,
     "note": "--local のときだけ起動・停止できる。Jetson 上での起動・停止は未確認"},
    {"id": "nav2ctl", "label": "Nav2 の起動・停止（タスクとして g1up.sh と撤収の一部を実行）", "implemented": True, "verified": False,
     "note": "tasks/nav2-*.json。警備班の手順を呼ぶだけ。発進ゲート（G1_ARM）と enable_navigation は自動で行わない。コンソールからの実行は未確認"},
    {"id": "nav", "label": "Nav2 の地図・現在地表示と、稼働中の手動操作の排他", "implemented": True, "verified": False,
     "note": "--nav-adapter（loopback の ROS アダプタ）を読むだけ。Goal や巡回の操作は送らない。実アダプタ・RViz との位置合わせは未確認"},
    {"id": "locations", "label": "地点（名前つき座標）の登録・編集と、タスクからの参照", "implemented": True, "verified": False,
     "note": "Console/data/locations.json。タスクの argv の {loc:NAME.x} で参照。座標系が Nav2 の map と一致するかは実機で未確認"},
    {"id": "topics", "label": "トピック通信（購読は自由、送信は許可リストだけ）", "implemented": True, "verified": False,
     "note": "--local のときだけ。運動・関節・腕・RPC は常に送信拒否。許可リストは --topics-file（既定は空）。Jetson の DDS では未確認"},
    {"id": "battery", "label": "バッテリー表示", "implemented": True, "verified": False,
     "note": "rt/lf/bmsstate。電圧・電流の単位は推定"},
    {"id": "audio", "label": "音声・LED（音量・頭部 LED・読み上げ）", "implemented": True, "verified": False,
     "note": "実機で音量取得・LED・読み上げを送信済み（デバッグモード）。LED は読み戻し不可で目視、読み上げは rt/audio_msg の play_state で開始/終了を観測可。ユーザー確認は未完"},
    {"id": "sensors", "label": "デバッグ情報表示（IMU・関節・指令・オドメトリ・Jetson・DDS 受信状況）", "implemented": True,
     "verified": False, "note": "lidar・カメラは対象外。関節名の対応・メインボード value の意味は未確認"},
]


# 音声・LED（モーションに関係しない書き込み）。範囲外は helper に渡さず弾く。
TTS_MAX_CHARS = 100
TTS_SPEAKERS = (0, 1)  # 0=中国語, 1=英語。実機確認: 話者 1 は鳴る・話者 0 は無音だった（2026-10-01）


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


# 関節名は SDK の並び順（脚 12・腰 3・腕 14）による対応。実機での突き合わせは未確認。
JOINT_NAMES = [
    "左股 pitch", "左股 roll", "左股 yaw", "左膝", "左足首 pitch", "左足首 roll",
    "右股 pitch", "右股 roll", "右股 yaw", "右膝", "右足首 pitch", "右足首 roll",
    "腰 yaw", "腰 roll", "腰 pitch",
    "左肩 pitch", "左肩 roll", "左肩 yaw", "左肘", "左手首 roll", "左手首 pitch", "左手首 yaw",
    "右肩 pitch", "右肩 roll", "右肩 yaw", "右肘", "右手首 roll", "右手首 pitch", "右手首 yaw",
]

# タブ名 = URL ハッシュ = API 名。AI もこの名前で画面と API を対応づけられる。
TABS = ("ops", "motion", "state", "joints", "audio", "tasks", "map", "camera", "dds", "settings")
