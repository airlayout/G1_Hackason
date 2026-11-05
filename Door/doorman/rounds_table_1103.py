"""11-03: 締め始め 0.9 の連鎖（chain/1102_1704）の回ごとの表を出す（作業ログ 2026-10-29 の 12 章）。
評価・線を固定した評価（fixed_level_1102.tsv）・扉の等級ごとの成功（bygrade.py）・20 rad/s での失敗の型（term_analysis.py の per_env.json）。
使い方（OMEN の ~/work_inoue/doorman で）: python rounds_table_1103.py > rounds_table_1103.tsv
"""
import csv
import json
import os
import re
import subprocess
from collections import Counter

PY = os.path.expanduser("~/miniforge3/envs/doorman6/bin/python")
TOOLS = os.environ.get("TOOLS", os.path.expanduser("~/work_inoue/doorman"))
CHAIN = "chain/1102_1704"
GRADES = ("等級1", "等級2", "等級3", "実機に近い m0≤16")


def kinds(rec):
    p = rec + "/per_env.json"
    if not os.path.exists(p):
        subprocess.run([PY, f"{TOOLS}/term_analysis.py", rec], capture_output=True)
    rows = json.load(open(p))
    fails = [x for x in rows if not x["success"]]
    k = Counter(x["kind"] for x in fails)
    s0 = sum(1 for x in fails if x["kind"] == "持ち時間切れ" and x["end_stage"] == 0)
    return k, s0


def grade(rec):
    out = subprocess.run([PY, f"{TOOLS}/bygrade.py", rec], capture_output=True, text=True).stdout
    g = {}
    for line in out.splitlines():
        m = re.match(r"\s*(等級1|等級2|等級3|実機に近い m0≤16)\s+(\d+)/\s*(\d+)", line)
        if m:
            g[m.group(1)] = f"{m.group(2)}/{m.group(3)}"
    return g


fixed = {}
for r in csv.reader(open("fixed_level_1102.tsv"), delimiter="\t"):
    if r[0] == CHAIN:
        fixed.setdefault(int(r[1]), {})[r[4]] = (r[5], r[7])
print("回\t反復\t評価\t5.0固定\t20固定\t" + "\t".join(GRADES) + "\t90°\t持ち時間\tうち近づく\t4m\t速さ")
for r in csv.reader(open(f"{CHAIN}/results.tsv"), delimiter="\t"):
    if r[0] != "f3sc":
        continue
    rd, it, succ, rec = int(r[2]), r[3], r[4], r[7]
    f = fixed.get(rd, {})
    l5 = f.get("0.25", ("-", None))[0]
    if "1.0" in f:
        l20, r20 = f["1.0"]
    elif rd >= 24:
        l20, r20 = "=", rec  # 線が 1.0 に戻った後は保存の線の評価が 20 rad/s の評価
    else:
        l20, r20 = "-", None
    line = [str(rd), it, succ, l5, l20]
    if r20:
        k, s0 = kinds(r20)
        g = grade(r20)
        line += [g.get(x, "") for x in GRADES]
        line += [str(k.get("扉が 90°", 0)), str(k.get("持ち時間切れ", 0)), str(s0), str(k.get("扉から 4 m", 0)), str(k.get("関節の速さ", 0))]
    print("\t".join(line))
