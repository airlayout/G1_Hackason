"""学習の連鎖の回ごとの評価を、できた順に当たり方まで数えて 1 行ずつ出す（10-19 夜。exp_touch3_1019.sh の見張り用）。

使い方（OMEN の ~/work_inoue/doorman で）: python touch_round_summary.py chain/1019_1819 [chain/...]
回の記録に touch_peak.json が無ければ作る（力は物理 1 回ごとの最大）。λ の記録（touch_logs/lambda_<名前>.json.log）があれば、その回の終わりの値も出す
"""
import glob
import json
import os
import sys

from touch_metrics import episodes, load, summarize


def lam_at(name: str, rec: str) -> str:
    """評価の記録ができた時刻より前に書かれた、最後の λ"""
    path = f"touch_logs/lambda_{name}.json.log"
    if not os.path.exists(path):
        return ""
    rows = [json.loads(x) for x in open(path) if x.strip()]
    if not rows:
        return ""
    # 回の区切りは記録の時刻で取れないので、最後の値を出す（λ の記録は学習中だけ書かれる）
    return f" λ={rows[-1]['lambda']:.0f} 違反の割合（学習中）={rows[-1]['rate']:.2f}"


def main() -> int:
    for chain in sys.argv[1:]:
        tsv = os.path.join(chain, "results.tsv")
        if not os.path.exists(tsv):
            continue
        for line in open(tsv).read().splitlines()[1:]:
            stage, _task, rnd, it, succ, rate, _ck, rec = line.split("\t")
            out = os.path.join(rec, "touch_peak.json")
            how = "物理 1 回ごとの最大"
            if not os.path.exists(out):
                if not glob.glob(os.path.join(rec, "states_*.npz")):
                    continue
                r = load(rec)
                if "hand_lever_force_peak" not in r:  # 0057 より前の記録
                    out, how = os.path.join(rec, "touch_snapshot_split.json"), "写し"
                eps = episodes(r, "auto")
                json.dump({"summary": summarize(eps), "per_episode": eps}, open(out, "w"), ensure_ascii=False, indent=1)
            s = json.load(open(out))["summary"]
            print(
                f"[{stage} 回 {rnd}・反復 {it}]（{how}）成功 {float(rate) * 100:.1f}%"
                f" 当たりの山15N以下 {s['当たりの山が15N以下の割合'] * 100:.1f}%"
                f" 山 {s['当たりの山[N]'].get('中央')}/{s['当たりの山[N]'].get('9割')} N"
                f" 直前の速さ {s['触れる直前の速さ[m/s]'].get('中央')} m/s"
                f" 動かす間 {s['動かす間の力[N]'].get('中央')} N"
                f" 底突き {s['底突きの力[N]'].get('中央')} N"
                f" 内訳 {s['15N超えの内訳']}"
                f"{lam_at(stage, rec)}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
