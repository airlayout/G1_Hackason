"""2 台以上の軌跡が「1 つの地図」に正しく載っているかを正解と比べる（TUM のように正解が同じ座標の物）。

    python tests/eval_multicam.py est_a.txt:gt_a.txt est_b.txt:gt_b.txt [...]

推定・正解とも TUM の形（時刻 x y z qx qy qz qw）。時刻は同じ時計（TUM は rgb.txt の時刻）。
出す物:
  joint   全部の台の組をまとめて 1 つの Sim3 で正解に合わせたときの、台ごとの位置の誤差（RMSE）
  own     台ごとに別々の Sim3 で合わせたときの誤差（その台の軌跡の形だけの良さ）
joint が own とほぼ同じなら、2 台目は地図の正しい所に載っている。joint だけ大きければ、
2 台の間の重ね方（合流）がずれている。
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from eval_traj import umeyama  # noqa: E402

MAX_DT = 0.02


def associate(est: np.ndarray, gt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    idx = np.searchsorted(gt[:, 0], est[:, 0]).clip(1, len(gt) - 1)
    near = np.where(np.abs(gt[idx - 1, 0] - est[:, 0]) < np.abs(gt[idx, 0] - est[:, 0]), idx - 1, idx)
    ok = np.abs(gt[near, 0] - est[:, 0]) < MAX_DT
    return est[ok, 1:4], gt[near[ok], 1:4]


def rmse(e: np.ndarray, g: np.ndarray, s: float, R: np.ndarray, t: np.ndarray) -> float:
    return float(np.sqrt(((s * e @ R.T + t - g) ** 2).sum(1).mean()))


def evaluate(pairs: list[tuple[np.ndarray, np.ndarray]]) -> dict:
    E = np.concatenate([e for e, _ in pairs])
    G = np.concatenate([g for _, g in pairs])
    s, R, t = umeyama(E, G, with_scale=True)
    out = {"scale_joint": round(s, 4), "cams": []}
    for e, g in pairs:
        so, Ro, to = umeyama(e, g, with_scale=True)
        out["cams"].append({
            "n_matched": int(len(e)),
            "ate_joint_m": round(rmse(e, g, s, R, t), 4),
            "ate_own_m": round(rmse(e, g, so, Ro, to), 4),
            "path_gt_m": round(float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum()), 3),
        })
    return out


def main() -> None:
    pairs = []
    for arg in sys.argv[1:]:
        est_path, gt_path = arg.split(":")
        e, g = associate(np.atleast_2d(np.loadtxt(est_path)), np.atleast_2d(np.loadtxt(gt_path)))
        if len(e) < 3:
            raise SystemExit(f"{est_path}: 正解と組にできた点が {len(e)} しかない（時刻の時計が違う？）")
        pairs.append((e, g))
    res = evaluate(pairs)
    for arg, cam in zip(sys.argv[1:], res["cams"]):
        cam["est"] = arg.split(":")[0]
    print(json.dumps(res, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
