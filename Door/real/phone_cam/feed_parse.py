"""PC2 の走らせ役の画面の出力（logs/console_latest.log）の 1 行ずつから、撮影のページに重ねる「今の状態」を作る。

読む行（door_run_stock.py・door_run_staged.py・door_script.py が出す形。11-12 の記録で確かめた）:
  [door_stock] READY → DOOR（180.0 秒・段に分けて（段 0 から））      段の移り
  [door_stock] READY    傾き  2.2° |dq| ... 扉 nan° 周期 20.4 ms（計算 6.3 ms） 腕 最高 55 °C（関節 15）   0.5 秒ごとの状態
  [script] 2a → 2b（手のひら−当てる点 前 -28.4 横 +0.9 上 -39.2 cm・扉 0.0°）   台本の小段の移り
  [script] 触れた… ／ [script] 下げた… ／ [staged] … ／ d を受け付けない… ／ d を待っている…   出来事
  [ana] …                                                              回が止まった時の要約（何行も続く）
状態は凍った dataclass で、1 行ごとに作り直す（apply_line）。
"""
from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass, replace
from typing import Dict, Optional, Tuple

MAX_EVENTS = 12          # 出来事は新しい方からこれだけ持つ
ANA_GAP_S = 3.0          # [ana] の行がこれ以上空いたら、次の [ana] は新しい要約
ACTIVE = ("ARMS_IN", "READY", "DOOR", "POST")  # 自動で録る間（door_run_stock.ACTIVE のうち腕を借りている所）

SUB_NAMES: Dict[str, str] = {  # door_script.SUBS の意味（door_script.py の冒頭の説明）
    "2a": "待つ", "2b": "構える", "2c": "下ろして触れる", "2d": "押し下げる",
    "3a": "掛け金を外す", "3c": "押しながら歩く", "done": "済み", "retreat": "やり直し", "failed": "失敗",
}

RE_TRANSITION = re.compile(r"^\[door_stock\] ([A-Z_]+) → ([A-Z_]+)(?:（(.*)）)?\s*$")
RE_STATUS = re.compile(r"^\[door_stock\] ([A-Z_]+)\s+傾き\s+([-\d.]+)°")
RE_DOOR = re.compile(r"扉 (nan|[-+\d.]+)°")
RE_PERIOD = re.compile(r"周期 ([\d.]+) ms（計算 ([\d.]+) ms）")
RE_TEMP = re.compile(r"腕 最高 (\d+) °C（関節 (\d+)）")
RE_FEET = re.compile(r"足 L (\d+) N・R (\d+) N")
RE_SUB = re.compile(r"^\[script\] (\w+) → (\w+)（(.*)）\s*$")
EVENT_KEYS = ("[script]", "[staged]", "受け付けない", "待っている", "⚠️", "手を扉から離す", "起動しない", "読み直した")


@dataclass(frozen=True)
class RobotState:
    phase: str = ""
    sub: str = ""
    sub_detail: str = ""
    door_deg: float = math.nan
    tilt_deg: float = math.nan
    period_ms: float = math.nan
    calc_ms: float = math.nan
    temp_c: float = math.nan
    temp_joint: int = -1
    feet_n: Tuple[float, float] = (math.nan, math.nan)
    status_line: str = ""
    events: Tuple[Tuple[float, str], ...] = ()
    ana: Tuple[str, ...] = ()
    ana_t: float = math.nan
    tries: int = 0      # READY → DOOR の数（回の番号）
    arms: int = 0       # ARMS_IN に入った数（a を打つたびに 1 つ。撮影のページは 1 つにつき 1 本録る）
    updated: float = math.nan


def _num(s: str) -> float:
    try:
        return float(s)
    except ValueError:
        return math.nan


def _event(st: RobotState, t: float, text: str) -> RobotState:
    return replace(st, events=(st.events + ((t, text),))[-MAX_EVENTS:])


def _status(st: RobotState, line: str, phase: str, tilt: str) -> RobotState:
    upd: dict = dict(phase=phase, tilt_deg=_num(tilt), status_line=line)
    m = RE_DOOR.search(line)
    if m:
        upd["door_deg"] = _num(m.group(1))
    m = RE_PERIOD.search(line)
    if m:
        upd.update(period_ms=_num(m.group(1)), calc_ms=_num(m.group(2)))
    m = RE_TEMP.search(line)
    if m:
        upd.update(temp_c=_num(m.group(1)), temp_joint=int(m.group(2)))
    m = RE_FEET.search(line)
    if m:
        upd["feet_n"] = (_num(m.group(1)), _num(m.group(2)))
    return replace(st, **upd)


def apply_line(st: RobotState, line: str, t: float) -> RobotState:
    """1 行を読んだ後の状態（t は Mac の時刻）"""
    line = line.rstrip("\r\n")
    if not line.strip():
        return st
    st = replace(st, updated=t)
    m = RE_TRANSITION.match(line)
    if m:
        old, new, why = m.group(1), m.group(2), m.group(3) or ""
        st = replace(st, phase=new,
                     tries=st.tries + (1 if new == "DOOR" else 0),
                     arms=st.arms + (1 if new == "ARMS_IN" else 0),
                     sub="" if new == "DOOR" else st.sub, sub_detail="" if new == "DOOR" else st.sub_detail)
        return _event(st, t, f"{old} → {new}" + (f"（{why}）" if why else ""))
    m = RE_STATUS.match(line)
    if m:
        return _status(st, line, m.group(1), m.group(2))
    if line.startswith("[ana]"):
        fresh = (not st.ana) or not (t - st.ana_t <= ANA_GAP_S)
        return replace(st, ana=((line,) if fresh else st.ana + (line,)), ana_t=t)
    m = RE_SUB.match(line)
    if m:
        st = replace(st, sub=m.group(2), sub_detail=m.group(3))
    if any(k in line for k in EVENT_KEYS):
        return _event(st, t, line)
    return st


def sub_label(sub: str) -> str:
    return f"{sub} {SUB_NAMES[sub]}" if sub in SUB_NAMES else sub


def to_json(st: RobotState, now: Optional[float] = None) -> dict:
    """撮影のページへ返す形（NaN は null に）"""
    d = asdict(st)
    for k, v in list(d.items()):
        if isinstance(v, float) and v != v:
            d[k] = None
    d["feet_n"] = [None if v != v else v for v in st.feet_n]
    d["events"] = [[t, s] for t, s in st.events]
    d["ana"] = list(st.ana)
    d["sub_label"] = sub_label(st.sub)
    d["active"] = st.phase in ACTIVE
    if now is not None and st.updated == st.updated:
        d["age_s"] = now - st.updated
    return d
