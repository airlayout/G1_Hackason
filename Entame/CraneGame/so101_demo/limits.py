"""モーター別の制限。値はすべて「可動範囲に対する%」（画面の表示と同じ）。

- speed: 追従（手の動きに合わせる）での最大速度（%/秒）
- step : 1回の指令で、今の位置から動かしてよい最大量（%）。LeRobot の max_relative_target に当たる
- min / max: 動かしてよい範囲の下限・上限（%）。範囲の外へは、指令を丸める

保存ファイル（limits.json）は起動時に自動で読む。無い・壊れているときは既定値。
"""
import json
import math

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# 項目ごとの (下限, 上限)。画面のスライダーの範囲にもなる
RANGES = {"speed": (1.0, 150.0), "step": (0.25, 15.0), "min": (0.0, 100.0), "max": (0.0, 100.0)}
MIN_SPAN = 5.0     # 下限と上限の最小の幅（%）
# 既定値は、制限を設定できるようにする前の固定値と同じ（腕は毎秒 22、グリッパーは毎秒 45 の内部単位、1回 3.0）
ARM_DEFAULT = {"speed": 11.0, "step": 1.5, "min": 0.0, "max": 100.0}
GRIPPER_DEFAULT = {"speed": 45.0, "step": 3.0, "min": 0.0, "max": 100.0}


def scale(joint):
    """% を内部の値（関節 -100〜100、グリッパー 0〜100）の幅にする倍率。"""
    return 1.0 if joint == "gripper" else 2.0


def raw(joint, percent):
    """可動範囲に対する % を、内部の値にする。"""
    return percent if joint == "gripper" else percent * 2 - 100


def default_limits():
    return {j: dict(GRIPPER_DEFAULT if j == "gripper" else ARM_DEFAULT) for j in JOINTS}


def parse_limits(data, base):
    """base に data（関節 → 項目 → 値。一部だけでもよい）を重ねた、検証済みの新しい制限を返す。"""
    if not isinstance(data, dict):
        raise ValueError("制限はオブジェクトで指定してください")
    merged = {j: dict(v) for j, v in base.items()}
    for joint, fields in data.items():
        if joint not in merged or not isinstance(fields, dict):
            raise ValueError(f"不明な関節: {joint}")
        for key, value in fields.items():
            if key not in RANGES:
                raise ValueError(f"不明な項目: {key}")
            low, high = RANGES[key]
            if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{joint} の {key} は {low:g}〜{high:g} で指定してください")
            merged[joint][key] = float(value)
    for joint, v in merged.items():
        if v["max"] - v["min"] < MIN_SPAN:
            raise ValueError(f"{joint} の上限と下限は {MIN_SPAN:g}% 以上離してください")
    return merged


def load_limits(path):
    """保存済みの制限を読む。ファイルが無ければ既定値。壊れていれば、理由を出して既定値で続ける。"""
    try:
        text = path.read_text()
    except FileNotFoundError:
        return default_limits()
    except OSError as exc:
        print(f"[limits] {path.name} を読めません（{exc}）。既定値を使います", flush=True)
        return default_limits()
    try:
        return parse_limits(json.loads(text), default_limits())
    except ValueError as exc:
        print(f"[limits] {path.name} が不正です（{exc}）。既定値を使います", flush=True)
        return default_limits()


def save_limits(limits, path):
    path.write_text(json.dumps(limits, indent=1, ensure_ascii=False))


def clamp_pose(pose, limits):
    """指令の姿勢（内部の値）を、関節ごとの可動範囲の中に丸める。渡された関節だけ返す。"""
    return {j: min(raw(j, limits[j]["max"]), max(raw(j, limits[j]["min"]), v)) for j, v in pose.items()}


def step_raw(joint, limits):
    return limits[joint]["step"] * scale(joint)


def speed_raw(joint, limits):
    return limits[joint]["speed"] * scale(joint)
