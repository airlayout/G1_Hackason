"""11-13: s0_script_1113.sh の結果（JSON Lines）を表にする。python3 s0_show_1113.py <jsonl> [--hmin 0.95]
回ごと: 最後の小段・止まり方・扉とレバーの最大・3c の間の pelvis の前後と向きの動き・手の力（見積もりと接触センサ）"""
import argparse
import collections
import json
import statistics as stt

ap = argparse.ArgumentParser()
ap.add_argument("jsonl")
ap.add_argument("--hmin", type=float, default=0.95, help="レバーの高さの下限 [m]（Room 4 は 0.99。低い扉は台本の最初の経由点に届かない）")
a = ap.parse_args()
rows = [json.loads(l) for l in open(a.jsonl) if l.strip()]
use = [r for r in rows if r.get("lever_height", 1.0) >= a.hmin]
print(f"回 {len(rows)}・うちレバー {a.hmin} m 以上 {len(use)}")


def pose_at(r, sub):
    for p in r.get("sub_pose", []):
        if p[0] == sub:
            return p
    return None


last = collections.Counter(r.get("last_sub", "") for r in use)
print("最後の小段:", dict(last))
print("扉 3° 以上:", sum(r["door_max_deg"] >= 3 for r in use), "・10° 以上:", sum(r["door_max_deg"] >= 10 for r in use),
      "・30° 以上:", sum(r["door_max_deg"] >= 29.5 for r in use), "・レバー 30° 以上:", sum(r["lever_max_deg"] >= 30 for r in use))
stops = collections.Counter((r.get("stop") or "（時間切れ）").split("（")[0] for r in use)
print("止まり方:")
for k, v in stops.most_common(8):
    print(f"  {v:3d}  {k}")
print("\n env  最後  扉最大°  3c に入った時 x・向き → 最後 x・向き    3c の手の力 前後 見積もり/接触  上下 見積もり/接触  相関(前後)")
for r in use:
    p3c, pend = pose_at(r, "3c"), (r.get("sub_pose") or [None])[-1]
    hf = r.get("hand_force", {}).get("3c") or r.get("hand_force", {}).get("3a") or {}
    s = f" {r['env']:3d}  {r.get('last_sub',''):6s} {r['door_max_deg']:5.1f}  "
    if p3c and pend:
        s += f"{p3c[2]:+.2f} {p3c[4]:+5.1f}° → {pend[2]:+.2f} {pend[4]:+5.1f}°  "
    else:
        s += " " * 34
    if hf:
        e, t = hf["est_med"], hf["true_med"]
        s += f"   {e[0]:+5.1f} / {t[0]:+5.1f}       {e[2]:+5.1f} / {t[2]:+5.1f}      {hf['corr'][0]}"
    print(s)
# 手の力の答え合わせ（小段ごと・全回の中央）
print("\n手の力の答え合わせ（扉の座標・N・回ごとの中央値の中央）:")
for sub in ("2c", "2d", "3a", "3c"):
    d = [r["hand_force"][sub] for r in use if sub in r.get("hand_force", {})]
    if not d:
        continue
    med = lambda key, k: stt.median(x[key][k] for x in d)
    cs = [x["corr"][0] for x in d if x["corr"][0] is not None]
    print(f"  {sub}: {len(d)} 回・前後 見積もり {med('est_med',0):+.1f}／接触 {med('true_med',0):+.1f}・上下 見積もり {med('est_med',2):+.1f}／接触 {med('true_med',2):+.1f}"
          f"・差の中央 前後 {med('abs_err_med',0):.1f}・上下 {med('abs_err_med',2):.1f}・前後の相関の中央 {stt.median(cs) if cs else float('nan'):.2f}")
    dp = [x for x in d if "palm_est_med" in x]
    if dp:
        mp = lambda key, k: stt.median(x[key][k] for x in dp)
        print(f"       手のひらの原点で解くと: 前後 {mp('palm_est_med',0):+.1f}・上下 {mp('palm_est_med',2):+.1f}・差の中央 前後 {mp('palm_abs_err_med',0):.1f}・上下 {mp('palm_abs_err_med',2):.1f}")
