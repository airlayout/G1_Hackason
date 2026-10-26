"""段 6 (b): 映像の生徒の 1 ステップの推論にかかる時間を CPU で測る（PC2 の lerobot の環境の torch で動かす）。

export_vision_student.py の書き出し（student_*.ts・golden.npz）を読み、まず 3 本とも golden の答えと同じかを確かめてから、
  full       画像＋観測 → 行動（制御の 1 ステップ全部。毎ステップ画像を見るとき）
  full_paced 同じ計算を、実機と同じく 20 ms ごとに休みを入れて（休みの間に CPU の周波数や糸が冷めると遅くなる）
  encoder    画像 → 画像の特徴（重い部分）
  recurrent  観測＋画像の特徴 → 行動（軽い部分。画像を間引くときに毎ステップ回す方）
  pre        カメラの灰色 640×360 → 384×216 の 3 チャンネル（cv2 がある時だけ）
  sizes      画像の部分だけを小さい画像で（既定 144×256・108×192。学習し直す前の見積もり用）
を測る。LSTM の状態は持ち回す。測った後にもう一度 golden を確かめる（最適化された後の計算でも同じ答えか）。
機体には何も送らない（DDS も触らない）。

使い方: OMP_NUM_THREADS=4 python bench_vision_student.py --dir <書き出し> --threads 4 --out t4.json
      python bench_vision_student.py --summary <結果のフォルダ>    # t*.json と trt_*.log を表にする
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import re
import time
from pathlib import Path

import numpy as np

CONTROL_HZ = 50.0  # 生徒と HOMIE の制御の周期（シミュレータの 0.02 s）
TOL = 1e-3  # 別の CPU（ARM）では足し算の順が変わるので書き出し時（1e-4）より緩める
TOL_TRT_FP16 = 5e-2  # fp16 の TensorRT の画像の特徴の差の目安（精度の参考。速さの判定には使わない）
CAM_W, CAM_H = 640, 360  # rec_ir.py の記録の大きさ


def stats(ms: list[float]) -> dict:
    a = np.asarray(ms)
    return {"n": int(a.size), "median": float(np.median(a)), "p90": float(np.percentile(a, 90)),
            "p99": float(np.percentile(a, 99)), "max": float(a.max()), "mean": float(a.mean())}


def timed(fn, iters: int, warmup: int, period_ms: float | None = None) -> list[float]:
    """fn の時間だけを測る。period_ms を渡すと、その周期の拍子に合わせて呼ぶ（間は寝る）"""
    for i in range(warmup):
        fn(i)
    out = []
    t_next = time.perf_counter()
    for i in range(iters):
        t0 = time.perf_counter_ns()
        fn(i)
        out.append((time.perf_counter_ns() - t0) / 1e6)
        if period_ms is not None:
            t_next += period_ms / 1000.0
            time.sleep(max(0.0, t_next - time.perf_counter()))
    return out


def check_golden(torch, full, enc, rec, g: dict, layers: int, hidden: int) -> dict:
    """全部・画像の部分・記憶の部分のそれぞれで、書き出した時の答え（golden）との差の最大"""
    zeros = lambda: torch.zeros(layers, 1, hidden)  # noqa: E731
    h, c, h2, c2 = zeros(), zeros(), zeros(), zeros()
    err = {"full": 0.0, "encoder": 0.0, "recurrent": 0.0}
    for t in range(len(g["obs"])):
        img = torch.from_numpy(g["imgs_u8"][t : t + 1])
        obs = torch.from_numpy(g["obs"][t : t + 1])
        a, h, c = full(img, obs, h, c)
        err["full"] = max(err["full"], float(np.abs(a.numpy().reshape(-1) - g["actions"][t]).max()))
        lat = enc(img)
        err["encoder"] = max(err["encoder"], float(np.abs(lat.numpy().reshape(-1) - g["latent"][t]).max()))
        a2, h2, c2 = rec(obs, torch.from_numpy(g["latent"][t : t + 1]), h2, c2)
        err["recurrent"] = max(err["recurrent"], float(np.abs(a2.numpy().reshape(-1) - g["actions"][t]).max()))
    return err


def bench(a) -> dict:
    import torch

    torch.set_num_threads(a.threads)
    torch.set_num_interop_threads(1)
    meta = json.loads((a.dir / "meta.json").read_text())
    layers, hidden = meta["lstm"]["layers"], meta["lstm"]["hidden"]
    g = dict(np.load(a.dir / "golden.npz"))
    full = torch.jit.load(str(a.dir / "student_full.ts")).eval()
    enc = torch.jit.load(str(a.dir / "student_encoder.ts")).eval()
    rec = torch.jit.load(str(a.dir / "student_recurrent.ts")).eval()
    res = {
        "host": platform.node(), "machine": platform.machine(), "python": platform.python_version(),
        "torch": torch.__version__, "threads": torch.get_num_threads(), "cpu_count": os.cpu_count(),
        "omp_num_threads": os.environ.get("OMP_NUM_THREADS"), "loadavg_before": os.getloadavg(),
    }
    with torch.inference_mode():
        res["golden_before"] = check_golden(torch, full, enc, rec, g, layers, hidden)
        imgs = [torch.from_numpy(x[None]) for x in g["imgs_u8"]]
        obs = [torch.from_numpy(x[None]) for x in g["obs"]]
        n = len(imgs)
        st = {"h": torch.zeros(layers, 1, hidden), "c": torch.zeros(layers, 1, hidden)}
        lat = enc(imgs[0])

        def step_full(i: int) -> None:
            _, st["h"], st["c"] = full(imgs[i % n], obs[i % n], st["h"], st["c"])

        def step_enc(i: int) -> None:
            enc(imgs[i % n])

        def step_rec(i: int) -> None:
            _, st["h"], st["c"] = rec(obs[i % n], lat, st["h"], st["c"])

        res["full"] = stats(timed(step_full, a.iters, a.warmup))
        res["full_paced"] = stats(timed(step_full, a.iters, a.warmup, period_ms=1000.0 / CONTROL_HZ)) if a.paced else None
        res["encoder"] = stats(timed(step_enc, a.iters, a.warmup))
        res["recurrent"] = stats(timed(step_rec, a.iters * 5, a.warmup))
        res["encoder_sizes"] = bench_sizes(torch, enc, a)
        res["golden_after"] = check_golden(torch, full, enc, rec, g, layers, hidden)
    errs = list(res["golden_before"].values()) + list(res["golden_after"].values())
    res["golden_max_err"] = max(errs)
    res["golden_ok"] = res["golden_max_err"] <= TOL
    res["pre"] = bench_preprocess(a.iters)
    res["loadavg_after"] = os.getloadavg()
    return res


def bench_sizes(torch, enc, a) -> dict:
    """小さい画像での画像の部分。失敗しても他の結果は残す"""
    out = {}
    for hw in a.sizes.split(",") if a.sizes else []:
        try:
            h, w = (int(v) for v in hw.split("x"))
            small = torch.randint(0, 256, (1, h, w, 3), dtype=torch.uint8)
            out[hw] = stats(timed(lambda i: enc(small), max(a.iters // 2, 10), 5))
        except Exception as e:  # noqa: BLE001 - 見積もり用の付け足し。失敗は記録して続ける
            out[hw] = {"error": f"{type(e).__name__}: {e}"[:200]}
    return out


def bench_preprocess(iters: int) -> dict | None:
    """カメラの灰色の画像を網の入力の大きさの 3 チャンネルに直す手間（実機で毎回かかる分）"""
    try:
        import cv2
    except ImportError:
        return None
    rng = np.random.default_rng(0)
    frames = [rng.integers(0, 256, size=(CAM_H, CAM_W), dtype=np.uint8) for _ in range(8)]

    def pre(i: int) -> None:
        small = cv2.resize(frames[i % 8], (384, 216), interpolation=cv2.INTER_AREA)
        cv2.cvtColor(small, cv2.COLOR_GRAY2RGB)

    return stats(timed(pre, iters, 10))


def parse_trt(path: Path) -> dict | None:
    """trtexec の結果の行（GPU Compute Time の median と 99 パーセンタイル）を読む"""
    txt = path.read_text(errors="replace")
    m = re.search(r"GPU Compute Time:.*?median = ([\d.]+) ms.*?percentile\(99%\) = ([\d.]+) ms", txt)
    if not m:
        return None
    return {"median": float(m.group(1)), "p99": float(m.group(2))}


def trt_latent_err(path: Path, latent0: np.ndarray) -> float | None:
    """trtexec --exportOutput の JSON（[{name, dimensions, values}]）の画像の特徴を golden と比べる"""
    try:
        outs = json.loads(path.read_text())
        vals = next(o["values"] for o in outs if o.get("name") == "latent")
        return float(np.abs(np.asarray(vals, dtype=np.float32).reshape(-1) - latent0.reshape(-1)).max())
    except (OSError, ValueError, KeyError, StopIteration, TypeError):
        return None


def _fmt(s: dict | None) -> str:
    return "—" if s is None else f"{s['median']:.1f}・{s['p99']:.1f}・{s['max']:.1f}"


def _worst_p99(r: dict) -> float:
    """休みなしと 20 ms ごとの遅い方（実機に近い方を逃さない）"""
    return max(r["full"]["p99"], (r.get("full_paced") or r["full"])["p99"])


def summary(root: Path) -> None:
    period = 1000.0 / CONTROL_HZ
    rows = [json.loads(p.read_text()) for p in sorted(root.glob("t*.json"), key=lambda q: int(re.sub(r"\D", "", q.stem) or 0))]
    if not rows:
        print("結果（t*.json）が無い")
        return
    r0 = rows[0]
    print(f"機械 {r0['host']}（{r0['machine']}・{r0['cpu_count']} コア）・Python {r0['python']}・torch {r0['torch']}")
    print(f"制御の周期 {period:.0f} ms（{CONTROL_HZ:.0f} Hz）に対して。単位 ms（中央・99%・最大）")
    print("| 糸 | golden | 全部（休みなし） | 全部（20 ms ごと） | 画像の部分 | 記憶の部分 | 前処理 |")
    print("|---|---|---|---|---|---|---|")
    for r in rows:
        ok = "✔" if r["golden_ok"] else f"✘ {r['golden_max_err']:.1e}"
        print(f"| {r['threads']} | {ok} | {_fmt(r['full'])} | {_fmt(r.get('full_paced'))} | {_fmt(r['encoder'])} "
              f"| {_fmt(r['recurrent'])} | {_fmt(r['pre'])} |")
    for hw in rows[0].get("encoder_sizes", {}):
        cells = "・".join(f"糸 {r['threads']} " + (f"{s['median']:.1f}" if "median" in s else "失敗")
                         for r in rows for s in [r.get("encoder_sizes", {}).get(hw, {})] if s)
        print(f"画像の部分だけ {hw}（中央 ms）: {cells}")
    golden = root.parent / "golden.npz"
    latent0 = np.load(golden)["latent"][0] if golden.exists() else None
    for p in sorted(root.glob("trt_*.log")):
        prec = p.stem[4:]
        t = parse_trt(p)
        line = f"TensorRT {prec}: " + ("読めない（ログを見る）" if t is None else f"中央 {t['median']:.2f} ms・99% {t['p99']:.2f} ms（GPU の計算だけ）")
        e = trt_latent_err(root / f"trt_{prec}_out.json", latent0) if latent0 is not None else None
        line += "・golden との差 " + ("読めない" if e is None else f"{e:.1e}" + ("" if e <= (TOL_TRT_FP16 if prec == "fp16" else TOL) else " ⚠️大きい"))
        print(line)
    if not all(r["golden_ok"] for r in rows):
        print("⚠️ golden が合わない糸がある＝書き出した網と違う計算をしている。下の判定は速さだけの参考")
    print("⚠️ 判定は生徒の計算だけ（HOMIE・カメラの取り込み・DDS は入っていない）。糸 8 は CPU を全部使う構成で、実機では制御や記録と取り合う")
    best = min(rows, key=_worst_p99)
    if _worst_p99(best) <= period:
        print(f"⇒ 毎ステップ画像を見ても {period:.0f} ms に収まる（糸 {best['threads']} で 99% {_worst_p99(best):.1f} ms）")
        return
    rec_best = min(rows, key=lambda r: r["recurrent"]["p99"])
    enc_best = min(rows, key=lambda r: r["encoder"]["median"])
    hz = 1000.0 / enc_best["encoder"]["median"]
    print(f"⇒ 毎ステップ画像は {period:.0f} ms に収まらない（最良でも 99% {_worst_p99(best):.1f} ms）。"
          f"記憶の部分は 99% {rec_best['recurrent']['p99']:.2f} ms、画像の部分だけなら CPU で中央値で約 {hz:.1f} Hz")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=Path)
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--paced", action="store_true", help="20 ms ごとに休みを入れた測り方も足す")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--sizes", default="144x256,108x192", help="画像の部分だけを測る小さい画像（縦x横、カンマ区切り。空で測らない）")
    ap.add_argument("--summary", type=Path)
    a = ap.parse_args()
    if a.summary:
        summary(a.summary)
        return
    if a.dir is None or a.out is None:
        ap.error("--dir と --out が要る（表だけなら --summary）")
    res = bench(a)
    a.out.write_text(json.dumps(res, ensure_ascii=False, indent=1))
    f, e, r = res["full"], res["encoder"], res["recurrent"]
    paced = res.get("full_paced")
    print(f"[bench] 糸 {res['threads']}: golden {'OK' if res['golden_ok'] else 'NG'}（差 {res['golden_max_err']:.1e}）"
          f"・全部 中央 {f['median']:.1f}/99% {f['p99']:.1f} ms" + (f"（20 ms ごと 99% {paced['p99']:.1f}）" if paced else "")
          + f"・画像 {e['median']:.1f} ms・記憶 {r['median']:.2f} ms")
    if not res["golden_ok"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
