"""初期姿勢（追従の基準）。値は limits.py と同じ内部の値（関節 -100〜100）。

グリッパーが机に接した、モーターを OFF にしても保てる姿勢を、人が実機で作って記録する。
追従（登録不要）では、手を画面の中央に置いた状態がこの姿勢に当たり、手を動かすと、動かせる下限・上限へ向かう。
保存ファイル（home.json）は起動時に自動で読む。無い・壊れているときは「未登録」（範囲の真ん中が基準）。
"""
import json
import math

from limits import JOINTS, scale

HOME_JOINTS = JOINTS[:-1]        # グリッパーは含めない（指の開きで動かす）
START_TOLERANCE = 10.0           # 追従の開始時に、初期姿勢から離れていてよい量（可動範囲に対する %）
NAMES = {"shoulder_pan": "台座", "shoulder_lift": "肩", "elbow_flex": "肘",
         "wrist_flex": "手首の曲げ", "wrist_roll": "手首の回転"}


def capture(pose):
    """今の姿勢から、初期姿勢を作る。"""
    return {j: float(pose[j]) for j in HOME_JOINTS}


def parse_home(data):
    """保存データを検証して、初期姿勢にする。"""
    if not isinstance(data, dict) or set(data) != set(HOME_JOINTS):
        raise ValueError("初期姿勢は " + "、".join(HOME_JOINTS) + " をすべて含めてください")
    for joint, value in data.items():
        if type(value) not in (float, int) or not math.isfinite(value) or not -100 <= value <= 100:
            raise ValueError(f"{joint} は -100〜100 で指定してください")
    return {j: float(data[j]) for j in HOME_JOINTS}


def load_home(path):
    """保存済みの初期姿勢を読む。無ければ None。壊れていれば、理由を出して None で続ける。"""
    try:
        text = path.read_text()
    except FileNotFoundError:
        return None
    except OSError as exc:
        print(f"[home] {path.name} を読めません（{exc}）。初期姿勢は未登録として扱います", flush=True)
        return None
    try:
        return parse_home(json.loads(text))
    except ValueError as exc:
        print(f"[home] {path.name} が不正です（{exc}）。初期姿勢は未登録として扱います", flush=True)
        return None


def save_home(home, path):
    path.write_text(json.dumps(home, indent=1, ensure_ascii=False))


def neutral_map(value, low, high, home=None):
    """入力 -1〜1 を、下限 low〜上限 high（内部の値）の中へ写す。0 が基準（home。無ければ範囲の真ん中）、
    +1 が上限、-1 が下限。基準が範囲の外なら、範囲の端に寄せる。"""
    base = (low+high) / 2 if home is None else min(high, max(low, home))
    return base + (high-base)*value if value >= 0 else base + (base-low)*value


def input_of(position, low, high, home=None):
    """neutral_map の逆。関節の位置 position（内部の値）を、入力 -1〜1 の尺度にする（手の入力と比べる用）。
    基準の側に余地がない（基準が範囲の端）ときは 0。"""
    base = (low+high) / 2 if home is None else min(high, max(low, home))
    room = high-base if position >= base else base-low
    return max(-1.0, min(1.0, (position-base) / room)) if room > 0 else 0.0


def far_joints(pose, home):
    """初期姿勢から START_TOLERANCE より離れている関節の名前（日本語）。"""
    return [NAMES[j] for j in HOME_JOINTS if abs(pose[j]-home[j]) > START_TOLERANCE*scale(j)]
