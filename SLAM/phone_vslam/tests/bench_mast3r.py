"""1 こまの追跡と、後ろの最適化の照合 1 回分の時間を測る（生の速さを上げる前後の比べ）。GPU を使う。

    cd ~/work_inoue/phone_slam/MASt3R-SLAM
    TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 python ../phone_vslam/tests/bench_mast3r.py <録画.mp4> [--n 20]

比べる形:
  fp32       上流と同じ（読み取り・復号は fp32＋TF32、頭は fp32）
  fp16       fast_mast3r.enable()（読み取り・復号・頭の DPT と MLP を fp16、後処理は fp32。cudnn.benchmark）
測る物:
  encode     画像の読み取り部 1 こま
  decode     復号部＋頭 1 組（2 枚分）
  track      FrameTracker.track の全体（推論＋対応＋姿勢の解き）
  sym x4     後ろの最適化が新しいキーフレームごとにする照合（検索で見つけた 4 組・両向き）
fp16 は fp32 との点の位置のずれ・姿勢のずれ・対応の割合も出す。
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import numpy as np
import torch

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(pathlib.Path.cwd()))


def timed(fn, n: int):
    out = fn()  # 温め（cudnn.benchmark の選びもここで済ませる）
    out = fn()
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    for _ in range(n):
        out = fn()
    torch.cuda.synchronize()
    return (time.perf_counter() - t0) / n * 1000, out


def run_variant(name, ctx, model, new, kf, kfs4, keyframes, fr_img, n):
    from mast3r_slam.mast3r_utils import decoder, mast3r_match_asymmetric, mast3r_match_symmetric
    from mast3r_slam.tracker import FrameTracker

    fr = new(fr_img)
    t_enc, (feat, pos, _) = timed(lambda: model._encode_image(fr.img, fr.img_true_shape), n)
    fr.feat, fr.pos = feat.float(), pos
    t_dec, _ = timed(lambda: decoder(model, fr.feat, kf.feat, fr.pos, kf.pos, fr.img_true_shape, kf.img_true_shape), n)
    t_match, out = timed(lambda: mast3r_match_asymmetric(model, new(fr_img), kf), n)
    tracker = FrameTracker(model, keyframes, "cuda:0")

    def track():
        f = new(fr_img)
        return tracker.track(f), f

    t_track, ((_, _, _), f) = timed(track, n)
    fi = torch.cat([k.feat for k in kfs4[:4]])
    pi = torch.cat([k.pos for k in kfs4[:4]])
    fj = torch.cat([kf.feat] * 4)
    pj = torch.cat([kf.pos] * 4)
    shapes = [k.img_true_shape for k in kfs4[:4]]
    t_sym, _ = timed(lambda: mast3r_match_symmetric(model, fi, pi, fj, pj, shapes, [kf.img_true_shape] * 4), max(3, n // 4))
    valid = out[1]
    print(
        f"[測る] {name}: encode {t_enc:5.1f}・decode {t_dec:5.1f}・encode+decode+match {t_match:5.1f}・"
        f"track {t_track:5.1f} ms（≒ {1000 / t_track:4.1f} fps）・sym x4 {t_sym:6.1f} ms・対応 {valid.float().mean():.3f}",
        flush=True,
    )
    return dict(X=out[2].float(), valid=valid, T=f.T_WC.data.clone())


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("video")
    p.add_argument("--n", type=int, default=20)
    p.add_argument("--gap", type=int, default=10, help="キーフレームといまのこまの間のこま数")
    args = p.parse_args()

    import lietorch
    import torch.multiprocessing as mp
    from mast3r_slam.config import load_config
    from mast3r_slam.frame import SharedKeyframes, create_frame
    from mast3r_slam.mast3r_utils import load_mast3r, mast3r_inference_mono

    from video_file import FfmpegVideoReader

    torch.backends.cuda.matmul.allow_tf32 = True
    torch.set_grad_enabled(False)
    load_config("config/base.yaml")
    model = load_mast3r(device="cuda:0")

    r = FfmpegVideoReader(args.video)
    imgs = []
    while len(imgs) <= 200 + args.gap and (it := r.read()) is not None:
        imgs.append(it[1])
    r.close()
    as_f = lambda i: imgs[i].astype(np.float32) / 255  # noqa: E731
    I = lietorch.Sim3.Identity(1, device="cuda:0")
    new = lambda img: create_frame(0, img, I, img_size=512, device="cuda:0")  # noqa: E731

    def make_kf(img):
        k = new(img)
        X, C = mast3r_inference_mono(model, k)
        k.update_pointmap(X, C)
        return k

    kf = make_kf(as_f(200))
    kfs4 = [make_kf(as_f(i)) for i in (150, 160, 170, 180)]
    h, w = kf.img.shape[-2:]
    keyframes = SharedKeyframes(mp.Manager(), h, w, buffer=4)
    keyframes.append(kf)
    fr_img = as_f(200 + args.gap)
    print(f"[測る] 画 {w}x{h}・トークン {h * w // 256}・{args.n} 回の平均", flush=True)

    ref = run_variant("fp32", None, model, new, kf, kfs4, keyframes, fr_img, args.n)

    import fast_mast3r

    fast_mast3r.enable()
    # キーフレームの読み取りも fp16 で作り直す（生では全部の画が fp16 の読み取りを通る）
    kf16 = make_kf(as_f(200))
    kfs16 = [make_kf(as_f(i)) for i in (150, 160, 170, 180)]
    keyframes16 = SharedKeyframes(mp.Manager(), h, w, buffer=4)
    keyframes16.append(kf16)
    got = run_variant("fp16", None, model, new, kf16, kfs16, keyframes16, fr_img, args.n)

    d = (got["X"] - ref["X"]).norm(dim=-1)
    scale = ref["X"].norm(dim=-1).median()
    dT = (got["T"] - ref["T"]).abs()
    print(
        f"[測る] fp16 と fp32 の差: 点の位置 中央値 {float(d.median() / scale) * 100:.2f}%・9 割 {float(d.quantile(0.9) / scale) * 100:.2f}%"
        f"（距離の中央値に対して）・姿勢 位置 {float(dT[0, :3].abs().max()):.4f}・回転 {float(dT[0, 3:7].abs().max()):.4f}・"
        f"対応の割合 {float(ref['valid'].float().mean()):.3f} → {float(got['valid'].float().mean()):.3f}",
        flush=True,
    )


if __name__ == "__main__":
    main()
