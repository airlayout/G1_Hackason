"""手のひらの上下（骨格版の追従中に、人へ指示する）。ロボットの手首の回転の値は、limits.py と同じ内部の値（-100〜100）。

「下」は初期姿勢の手首の回転（未登録なら追従を始めたときの値）。「上」は、人がモーターOFFで手首を回して作り、
「今の手首を『手のひら上』にする」で記録する（実機で測るまで、値は推測しない）。保存ファイル（palm.json）は起動時に読む。
"""
import json
import math

PALM_STATES = ("down", "up")
PALM_NAMES = {"down": "下", "up": "上"}
MIN_GAP = 20.0        # 「上」と「下」の手首の回転の差がこれ未満なら、登録のし間違いとみなす（内部の値）


def parse_palm_up(data):
    """保存データを検証して、「手のひら上」の手首の回転にする。"""
    value = data.get("wrist_roll") if isinstance(data, dict) else None
    if type(value) not in (float, int) or not math.isfinite(value) or not -100 <= value <= 100:
        raise ValueError("wrist_roll は -100〜100 で指定してください")
    return float(value)


def load_palm_up(path):
    """保存済みの値を読む。無ければ None。壊れていれば、理由を出して None で続ける。"""
    try:
        text = path.read_text()
    except FileNotFoundError:
        return None
    except OSError as exc:
        print(f"[palm] {path.name} を読めません（{exc}）。手のひら上は未登録として扱います", flush=True)
        return None
    try:
        return parse_palm_up(json.loads(text))
    except ValueError as exc:
        print(f"[palm] {path.name} が不正です（{exc}）。手のひら上は未登録として扱います", flush=True)
        return None


def save_palm_up(value, path):
    path.write_text(json.dumps({"wrist_roll": value}, indent=1))
