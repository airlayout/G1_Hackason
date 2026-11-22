"""教示（ティーチング）の記録。人の姿勢 H1〜H10 を1つずつ選び、ロボットを手で動かして、その関節値を保存する。

保存するのは、H ごとの「人の角度（pan・lift・elbow）」と「ロボットの関節値（limits.py と同じ内部の値）」の組。
あとで、人の角度 → ロボットの関節値の対応（IK）を解くための元データになる。値は実機で測ったものだけを入れる（推測しない）。
保存ファイル（teach.json）は起動時に読む。無い・壊れているときは空。
"""
import json
import math
import os
import time

from limits import JOINTS

# 人の姿勢。角度は skeleton.py と同じ定義（度）。pan が None のものは、上腕がほぼ垂直で横向きが測れない（対応づけに pan を使わない）
HUMAN_POSES = (
    {"id": "H1", "name": "腕を体の横に下ろす", "pan": None, "lift": 10, "elbow": 170},
    {"id": "H2", "name": "上腕を下ろし、肘を直角に", "pan": None, "lift": 20, "elbow": 90},
    {"id": "H3", "name": "上腕を少し上げ、肘は直角（基準）", "pan": 0, "lift": 50, "elbow": 90},
    {"id": "H4", "name": "腕を前へ水平に伸ばす", "pan": 0, "lift": 90, "elbow": 170},
    {"id": "H5", "name": "腕を前へ水平に上げ、肘を直角に", "pan": 0, "lift": 90, "elbow": 90},
    {"id": "H6", "name": "腕を真横へ水平に伸ばす", "pan": 70, "lift": 90, "elbow": 170},
    {"id": "H7", "name": "腕を斜め前・外側へ上げて伸ばす", "pan": 40, "lift": 90, "elbow": 150},
    {"id": "H8", "name": "腕を斜め前・内側へ（体の前を横切る）", "pan": -40, "lift": 60, "elbow": 120},
    {"id": "H9", "name": "肘を深く曲げ、手を肩へ近づける", "pan": 0, "lift": 40, "elbow": 40},
    {"id": "H10", "name": "腕を斜め上へ高く伸ばす", "pan": 0, "lift": 135, "elbow": 170},
)
HUMAN_IDS = tuple(p["id"] for p in HUMAN_POSES)
SAME_POSE = 0.5       # 直前に保存した姿勢との差がこれ未満なら「動かしていないのでは」と知らせる（内部の値）


def parse_pose(pose):
    """ロボットの関節値を検証して、全関節の新しい辞書にする。"""
    if not isinstance(pose, dict) or set(pose) != set(JOINTS):
        raise ValueError("姿勢は " + "、".join(JOINTS) + " をすべて含めてください")
    out = {}
    for joint in JOINTS:
        value = pose[joint]
        low = 0 if joint == "gripper" else -100
        if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= 100:
            raise ValueError(f"{joint} は {low}〜100 で指定してください")
        out[joint] = float(value)
    return out


def parse_teach(data):
    """保存データ（H の id → {"robot": 姿勢, "time": 文字列}）を検証して、新しい辞書にする。"""
    if not isinstance(data, dict):
        raise ValueError("教示データはオブジェクトで指定してください")
    out = {}
    for key, entry in data.items():
        if key not in HUMAN_IDS:
            raise ValueError(f"不明な姿勢: {key}")
        if not isinstance(entry, dict):
            raise ValueError(f"{key} の形式が不正です")
        out[key] = {"robot": parse_pose(entry.get("robot")), "time": str(entry.get("time", ""))}
    return out


def load_teach(path):
    """保存済みの教示を読む。無ければ空。壊れていれば、理由を出して空で続ける。"""
    try:
        text = path.read_text()
    except FileNotFoundError:
        return {}
    except OSError as exc:
        print(f"[teach] {path.name} を読めません（{exc}）。教示は空として扱います", flush=True)
        return {}
    try:
        return parse_teach(json.loads(text))
    except ValueError as exc:
        print(f"[teach] {path.name} が不正です（{exc}）。教示は空として扱います", flush=True)
        return {}


def save_teach(teach, path):
    """途中で壊れないよう、一時ファイルに書いてから置き換える。"""
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(teach, indent=1, ensure_ascii=False))
    os.replace(temp, path)


def entry(pose):
    return {"robot": parse_pose(pose), "time": time.strftime("%Y-%m-%d %H:%M:%S")}


def nearest_gap(teach, key, pose):
    """key 以外で保存済みの姿勢のうち、pose にいちばん近いものとの、関節ごとの差の最大値。無ければ None。"""
    gaps = [max(abs(e["robot"][j] - pose[j]) for j in JOINTS[:-1]) for k, e in teach.items() if k != key]
    return min(gaps) if gaps else None
