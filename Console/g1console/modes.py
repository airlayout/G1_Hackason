"""G1 の FSM ID 定義（操作用 PC 側サーバと Jetson 側ヘルパーで共有する情報）。

出典: unitree_sdk2_python の g1_loco_client.py（0/1/3/500/702/706）と、
Unitree 公式マニュアル系ドキュメント（501 = 腰3自由度版の通常モード）。
実機で GetFsmId が 501 を返した記録: 別スクリプトによる 1 回と、2026-10-18 のコンソール経由（リモコンで通常モードにした後）。
"""

# 表示用ラベル（GetFsmId の値 → 名前）
FSM_LABELS = {
    0: "Zero Torque",
    1: "Damping",
    3: "Seated Mode",
    4: "Locked Standing",  # 2026-10-18 リモコンで入れた直後に 4 を確認。2026-10-24 にもリモコン経由で再現
    500: "Regular Mode (1-axis waist)",
    501: "Regular Mode (3-axis waist)",
    702: "Lie -> Stand",
    706: "Squat <-> Stand",
    # 2026-10-24 リモコン R1+R2+X（キー値 1041）で入り、約 1 秒で Damping に戻った。ロボットが「robot rising」「damp mode」と発音。
    # 名前は Abra Robotics の表の「Adapt rise」による推定。吊り下げ中のため、実際の起き上がりは未観測。ボタンは置かない
    1001: "Adapt rise",
}

# 切り替えボタン（順序どおりに表示）。danger=True は歩行中に押すと警告を出す。
# モード名は公式の英語表記のまま使う（翻訳しない）。
# 「Running Mode(801)」「デバッグ」は ID/手順が未確認のため未搭載。
BUTTONS = [
    {"id": 1, "label": "Damping", "danger": True},
    {"id": 0, "label": "Zero Torque", "danger": True},
    {"id": 4, "label": "Locked Standing", "danger": False},  # ID はリモコン経由で確認。コンソールからの送信は未確認
    {"id": 3, "label": "Seated Mode", "danger": False},
    {"id": 706, "label": "Squat <-> Stand", "danger": False},
    {"id": 702, "label": "Lie -> Stand", "danger": False},
    {"id": 501, "label": "Regular Mode", "danger": False},
]

ALLOWED_IDS = frozenset(b["id"] for b in BUTTONS)

# 速度モード（LocoClient.SetSpeedMode）。値の意味（どちらが低速か）は資料が無く未確認。
# 試験で確かめるまで、中立な名前で出す。確かめたら label だけ書き換える。読み戻しの手段は無い。
SPEED_BUTTONS = [
    {"value": 0, "label": "Speed 0（試験）"},
    {"value": 1, "label": "Speed 1（試験）"},
]
SPEED_VALUES = frozenset(b["value"] for b in SPEED_BUTTONS)
SPEED_FSM_IDS = (500, 501)  # 歩行（Regular Mode）のときだけ押せる

# 現在の FSM → 画面で押せるボタン。公式の状態遷移図（2026-10-24 利用者提示）に沿う。
# Zero Torque へは Damping からだけ、Zero Torque からは Damping だけ。基本は Damping から他のモードへ。
# 緊急用に、どの状態からでも Damping は押せる。図に無い遷移は載せない。
# 706 は Damping から送っても動かない（実機）。Regular Mode から送ると動く（2026-10-24。POST 数からの読みで、画面の遷移表示は未確認）。
# 図の「Squat」(L1+DOWN) の FSM ID は未確認のため、ボタンが無い。ここに無い現在値は Damping だけ押せる。
TRANSITIONS = {
    0: (1,),
    1: (0, 4, 3, 702),  # 706 は Damping からは何度送っても動かなかった（2026-10-24）。Regular Mode から動く

    4: (1, 3, 501),
    3: (1, 4),
    501: (1, 3, 706),  # 2026-10-24 コンソールで Damping→4→501→706 を確認（706 は最後に Damping へ自動遷移）
    500: (1, 3, 706),
    706: (1, 501),
    702: (1, 501),
}
FALLBACK_TRANSITION = (1,)

# 未実装のモード切替（画面に「未実装」で無効表示する）。理由は docs/REMOTE_CONTROLLER.md 参照。
PLANNED_MODES = [
    {"label": "Running Mode", "note": "FSM ID が未確認（801 は資料 1 件のみ）"},
    {"label": "デバッグモード", "note": "切替手順が未確認"},
]

# アクション。G1 本体のアーム動作サービスに action_id を送って実行する（腕の動作を再生するだけで、歩行は止めない）。
# 番号は実機の GetActionList（2026-10-24）で確認した。SDK ヘッダの対応表は right kiss を 12 としているが、実機は 13。
# 純正リモコン（handshake / wave hand / face wave / right kiss / clap）との対応: right kiss=13・clap=17・handshake=27 は名前が一致。
# wave hand と face wave は候補が複数あり、どれが同じ動きかは実機で確かめる（未確認）。
# 握手などは動作後に腕を保持することがある。release_arm（99）で戻す。
ACTIONS = [
    {"id": "wave_under_head", "label": "手を振る（胸の前・wave_under_head）", "arm_id": 25},
    {"id": "wave_above_head", "label": "手を振る（頭の上・wave_above_head）", "arm_id": 26},
    {"id": "turn_back_wave", "label": "振り向いて手を振る（turn_back_wave）", "arm_id": 1},
    {"id": "shake_hand", "label": "握手（shake_hand）", "arm_id": 27},
    {"id": "blow_kiss_right", "label": "右手の投げキス（blow_kiss_with_right_hand）", "arm_id": 13},
    {"id": "clap", "label": "拍手（clamp）", "arm_id": 17},
    {"id": "release_arm", "label": "腕を戻す（release_arm）", "arm_id": 99},
]
PLANNED_ACTIONS = []  # 未実装のアクションは今は無い（API の形は保つ）

# 基本機能の状態。REMOTE_CONTROLLER.md の表と同じ内容を保つこと。
# implemented: 実装済みか / verified: 実機（G1 電源オン、コンソール経由）で試したか
FEATURES = [
    {"id": "connection", "label": "接続状態表示", "implemented": True, "verified": False},
    {"id": "reconnect", "label": "自動再接続", "implemented": True, "verified": False},
    {"id": "mode", "label": "モード変更", "implemented": True, "verified": False,
     "note": "Damping / Zero Torque / Seated Mode / Squat <-> Stand / Lie -> Stand / Regular Mode のみ。2026-10-24 Zero Torque→Damping をコンソール経由で実機確認（1.1 秒）。他の遷移は未確認"},
    {"id": "speed", "label": "速度モード（Speed 0 / Speed 1）", "implemented": True, "verified": False,
     "note": "SetSpeedMode の値の意味（低速・高速）は資料が無く未確認。2026-10-24 の試験は、ボタンを押しても反応が無く、中断（サーバーのログに送信の記録なし）。読み戻しの手段は無い"},
    {"id": "action", "label": "アクション実行", "implemented": True, "verified": False,
     "note": "手を振る・握手・投げキス・拍手など。番号は実機の一覧で確認済み。2026-10-24 実機で全7項目（25/26/1/27/13/17/99）の動作を確認。リモコン表記（wave hand / face wave）との対応は未整理。--local のときだけ"},
    {"id": "move", "label": "移動（ジョイスティック・矢印キー）", "implemented": True, "verified": True,
     "note": "--local のときだけ。2026-10-24 利用者が実機（Regular Mode）で移動の成功を確認。接続断は 0.5 秒で停止。速度上限は未検証"},
    {"id": "tasks", "label": "タスク実行（定義済みコマンドの起動・停止・ログ）", "implemented": True, "verified": False,
     "note": "Console/tasks/*.json の argv だけを実行する。Mac のテストでは確認済みだが、Jetson 上での実行は未確認"},
    {"id": "camera", "label": "カメラの映像表示と、配信の起動・停止", "implemented": True, "verified": False,
     "note": "--local のときだけ起動・停止できる。Jetson 上での起動・停止は未確認"},
    {"id": "lidar", "label": "lidar の点群表示と、配信の起動・停止", "implemented": True, "verified": False,
     "note": "--local のときだけ起動・停止できる。Jetson 上での起動・停止は未確認"},
    {"id": "lidar_sensor", "label": "LiDAR 本体の停止（待機）・起動（計測）", "implemented": True, "verified": False,
     "note": "--local のときだけ。Jetson から LiDAR(192.168.123.120) へ Livox 制御で work_tgt_mode を IDLE/SAMPLING に切り替える。待機中は点群が来ないので Nav2・マッピングは使えない。電源を入れ直すと計測に戻る。ハブ経由の操作は未確認（Jetson からの直接送信は 2026-10-24 に確認）"},
    {"id": "hubstop", "label": "ハブ（このコンソール）の停止", "implemented": True, "verified": False,
     "note": "設定タブのボタン。止めると画面から再起動できない（Jetson で行う）。タスク実行中・操作権の保持中は止めない。実機（systemd 配下を含む）では未確認"},
    {"id": "nav2ctl", "label": "Nav2 の起動・停止（タスクとして g1up.sh と撤収の一部を実行）", "implemented": True, "verified": False,
     "note": "tasks/nav2-*.json。警備班の手順を呼ぶだけ。発進ゲート（G1_ARM）と enable_navigation は自動で行わない。コンソールからの実行は未確認"},
    {"id": "nav", "label": "Nav2 の地図・現在地表示と、稼働中の手動操作の排他", "implemented": True, "verified": False,
     "note": "--nav-adapter（loopback の ROS アダプタ）を読むだけ。Goal や巡回の操作は送らない。実アダプタ・RViz との位置合わせは未確認"},
    {"id": "locations", "label": "地点（名前つき座標）の登録・編集と、タスクからの参照", "implemented": True, "verified": False,
     "note": "Console/data/locations.json。タスクの argv の {loc:NAME.x} で参照。座標系が Nav2 の map と一致するかは実機で未確認"},
    {"id": "topics", "label": "トピック通信（購読は自由、送信は許可リストだけ）", "implemented": True, "verified": False,
     "note": "--local のときだけ。運動・関節・腕・RPC は常に送信拒否。許可リストは --topics-file（既定は空）。Jetson の DDS では未確認"},
    {"id": "battery", "label": "バッテリー表示", "implemented": True, "verified": True,
     "note": "rt/lf/bmsstate。2026-10-18 利用者が実機で確認済みと申告（比較した数値は記録なし）"},
    {"id": "audio", "label": "音声・LED（音量・頭部 LED・読み上げ）", "implemented": True, "verified": False,
     "note": "実機で音量取得・LED・読み上げを送信済み（デバッグモード）。LED は読み戻し不可で目視、読み上げは rt/audio_msg の play_state で開始/終了を観測可。ユーザー確認は未完"},
    {"id": "telemetry", "label": "姿勢（IMU）・オドメトリ・Jetson・メインボード・リモコン／非常停止／サービスの表示", "implemented": True,
     "verified": True, "note": "2026-10-18 利用者が実機で値が取れている（受信・表示）ことを確認と申告。メインボード value の意味・単位・リモコン押下での変化は未確認"},
    {"id": "sensors", "label": "デバッグ情報表示（関節・指令・DDS 受信状況）", "implemented": True,
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
TABS = ("ops", "motion", "action", "state", "joints", "audio", "tasks", "map", "camera", "dds", "settings")
