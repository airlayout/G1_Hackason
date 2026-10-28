"""押している間に体（pelvis）がどれだけ動いたかを、実機の記録から出す（ラボ PC で。ロボットには何も送らない）。

実機の G1 は、腕を伸ばしたり押した反動を受けたりすると、足側のバランス制御が腰の位置を直す。シミュレーションは
腰を固定しているので、この動きは見えない。対象の位置は押す前に pelvis 座標で測っているので、そのあとに腰が動くと
そのぶん狙いがずれる。求め方は common/body_motion.py。

    P=G1_HuggingFace/venv/bin/python
    # ボトル（全体をつなぐスクリプトの記録。終わったときにも同じ内容を表示している）
    $P Button_Press/J1-gen/real/body_motion.py _local/button_press/runs/<日時>_<ラベル>
    # エレベーター（contest_agent を J1GEN_BODY_LOG を付けて動かしたときの記録）
    $P Button_Press/J1-gen/real/body_motion.py _local/button_press/body_log/<日時>_<up|down>

基準は、ボトルは対象を測った時刻の直前、エレベーターは ▲▼ を探していた最後の周期（look の最後）。
結果は記録のフォルダの body_motion.csv にも書く（行ごとの値。グラフにするとき用）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.body_motion import LegKinematics, analyze, analyze_run_dir, read_lowstate_csv, write_csv  # noqa: E402
from common.config import load_config  # noqa: E402


def analyze_body_log(folder: Path, legs: LegKinematics, point: np.ndarray | None):  # type: ignore[no-untyped-def]
    t, q, imu, labels = read_lowstate_csv(folder / "body_log.csv")
    looks = [i for i, p in enumerate(labels) if p == "look"]
    ref = looks[-1] if looks else 0
    meta = folder / "meta.json"
    if point is None and meta.exists():
        point = np.asarray(json.loads(meta.read_text())["target_point"], dtype=float)
    return analyze(legs, t, q, imu, ref=ref, point=point, labels=labels)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", type=Path, help="記録のフォルダ（lowstate.csv か body_log.csv があるところ）")
    ap.add_argument("--point", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="狙いのずれを見る点（pelvis 座標 [m]）。省略すると記録の対象の点")
    a = ap.parse_args()

    legs = LegKinematics(load_config("robot.yaml"))
    point = None if a.point is None else np.asarray(a.point, dtype=float)
    if (a.folder / "body_log.csv").exists():
        m = analyze_body_log(a.folder, legs, point)
    elif (a.folder / "lowstate.csv").exists():
        m = analyze_run_dir(a.folder, legs, point)
    else:
        print(f"[body_motion] {a.folder} に lowstate.csv も body_log.csv も無い")
        return 1
    print(m.text())
    write_csv(a.folder / "body_motion.csv", m)
    print(f"[body_motion] 行ごとの値: {a.folder / 'body_motion.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
