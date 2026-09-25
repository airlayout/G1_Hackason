#!/usr/bin/env python3
"""G1本体(Jetson)でYOLOの推論性能を計測するスクリプト。

実機に触れる時間は限られるため、**実機で確かめないと分からない数字**だけを
1回の実行でまとめて取る。操作PCでは意味が無い(操作PCの性能が分かるだけ)。

このリポジトリの他のファイルには依存しない(common/ をimportしない)。
G1本体にこのファイル1つをコピーすれば動く。

計測する内容:
  1. 実行環境      … Jetsonの型番/JetPack/CPU数/メモリ/torchのCUDA対応
  2. 推論時間      … モデル × 入力サイズ × スレッド数 の中央値
  3. メモリ使用量  … プロセスのRSS(と、GPUを使う場合はGPUメモリ)
  4. 連続動作      … 一定時間回し続けたときの速度低下(発熱による性能低下の確認)

**スレッド数を変えて測るのは、PerceptionがCPUを占有できないため。**
Mapping/歩行制御などと同居する前提で、「2スレッドに絞ったら何msか」を知っておく。

使い方(G1本体側。conda環境を有効化してから):
  source ~/miniforge3/bin/activate lerobot
  pip install ultralytics          # 未インストールの場合
  python bench_onboard.py --out ~/bench_result.json

  # 重みを事前に操作PCから渡しておく場合(実機がインターネットに出られないとき):
  #   操作PC側: scp yolo26n.pt unitree@192.168.123.164:~/
  #   G1側:     python bench_onboard.py --weights ~/yolo26n.pt
"""
import argparse
import json
import platform
import statistics
import subprocess
import time
from pathlib import Path

import numpy as np


def read_text(path: str) -> str:
    """存在しないファイルやアクセスできないファイルは空文字で返す"""
    try:
        return Path(path).read_text(errors="ignore").strip()
    except OSError:
        return ""


def run_cmd(cmd: list[str]) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def rss_mb() -> float:
    """このプロセスが使っている物理メモリ(MB)"""
    for line in read_text("/proc/self/status").splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / 1024
    return float("nan")


def cpu_temp_c() -> float:
    """CPUの温度(℃)。取れなければ nan。発熱による性能低下の確認に使う"""
    temps = []
    for zone in sorted(Path("/sys/devices/virtual/thermal").glob("thermal_zone*")):
        t = read_text(str(zone / "temp"))
        if t.isdigit():
            temps.append(int(t) / 1000)
    return max(temps) if temps else float("nan")


def collect_environment() -> dict:
    """実機の素性を記録する。型番やJetPackの版が分からないと結果を解釈できない"""
    import torch

    env = {
        "machine": platform.machine(),
        "python": platform.python_version(),
        "os": read_text("/etc/os-release").splitlines()[0] if read_text("/etc/os-release") else "",
        # Jetson固有。デスクトップでは空になる
        "device_model": read_text("/proc/device-tree/model").replace("\x00", ""),
        "nv_tegra_release": read_text("/etc/nv_tegra_release"),
        "jetpack_l4t": read_text("/etc/nv_tegra_release").split(",")[0] if read_text("/etc/nv_tegra_release") else "",
        "nvpmodel": run_cmd(["nvpmodel", "-q"]),   # 電力モード(取れないことが多い)
        "cpu_count": __import__("os").cpu_count(),
        "mem_total_kb": next((l.split()[1] for l in read_text("/proc/meminfo").splitlines()
                              if l.startswith("MemTotal:")), ""),
        "torch": torch.__version__,
        "cuda_available": torch.cuda.is_available(),
        "cuda_device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "",
    }
    return env


def make_image(width: int, height: int) -> np.ndarray:
    """計測用の画像。G1のカメラと同じ大きさで作る。

    推論時間は画像の中身ではなく大きさでほぼ決まるため、実写でなくてよい。
    ultralyticsのサンプル画像があればそれを使い、無ければ模様を作る。
    """
    try:
        import cv2
        from ultralytics.utils import ASSETS

        sample = Path(ASSETS) / "bus.jpg"
        if sample.exists():
            return cv2.resize(cv2.imread(str(sample)), (width, height))
    except Exception:
        pass
    rng = np.random.default_rng(0)
    return rng.integers(0, 255, (height, width, 3), dtype=np.uint8)


def measure(model, image, imgsz: int, repeat: int, use_cuda: bool) -> list[float]:
    """推論時間(ミリ秒)を repeat 回測って返す。最初の数回はウォームアップとして捨てる"""
    import torch

    for _ in range(3):
        model.predict(image, imgsz=imgsz, classes=[0], conf=0.5, verbose=False)
    if use_cuda:
        torch.cuda.synchronize()

    times = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        model.predict(image, imgsz=imgsz, classes=[0], conf=0.5, verbose=False)
        if use_cuda:
            # GPUは非同期に動くため、待たないと実際より速く見える
            torch.cuda.synchronize()
        times.append((time.perf_counter() - t0) * 1000)
    return times


def soak_test(model, image, imgsz: int, seconds: float, use_cuda: bool) -> dict:
    """一定時間回し続けて、前半と後半で速度が変わるかを見る(発熱による性能低下の確認)"""
    import torch

    times, start = [], time.perf_counter()
    while time.perf_counter() - start < seconds:
        t0 = time.perf_counter()
        model.predict(image, imgsz=imgsz, classes=[0], conf=0.5, verbose=False)
        if use_cuda:
            torch.cuda.synchronize()
        times.append((time.perf_counter() - t0) * 1000)
    if len(times) < 4:
        return {"frames": len(times)}
    half = len(times) // 2
    return {
        "frames": len(times),
        "first_half_ms": round(statistics.median(times[:half]), 1),
        "second_half_ms": round(statistics.median(times[half:]), 1),
        "slowdown_pct": round((statistics.median(times[half:]) / statistics.median(times[:half]) - 1) * 100, 1),
        "temp_after_c": round(cpu_temp_c(), 1),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default="yolo26n.pt",
                        help="計測するモデル。カンマ区切りで複数可(例: yolo26n.pt,yolo26s.pt)")
    parser.add_argument("--sizes", default="640,320", help="入力サイズ。カンマ区切り")
    parser.add_argument("--threads", default="0,4,2,1",
                        help="CPUスレッド数。0は制限なし。Perceptionが他タスクとCPUを分け合う前提で複数測る")
    parser.add_argument("--width", type=int, default=640, help="計測画像の幅(G1のカメラに合わせる)")
    parser.add_argument("--height", type=int, default=480, help="計測画像の高さ")
    parser.add_argument("--repeat", type=int, default=15, help="1条件あたりの計測回数")
    parser.add_argument("--soak", type=float, default=60.0, help="連続動作の秒数(0で省略)")
    parser.add_argument("--out", default="", help="結果をJSONで保存するパス")
    args = parser.parse_args()

    try:
        import torch
        from ultralytics import YOLO
    except ImportError as e:
        print(f"[bench] エラー: 必要なパッケージがありません: {e}")
        print("[bench]   conda環境を有効化してから `pip install ultralytics` してください:")
        print("[bench]   source ~/miniforge3/bin/activate lerobot")
        return 1

    env = collect_environment()
    print("=== 実行環境 ===")
    for k, v in env.items():
        if v not in ("", None):
            print(f"  {k:18s}: {v}")
    if not env["cuda_available"]:
        print("\n  ⚠ torch から GPU が見えていません。以降の数値は CPU のものです。")
        print("    Jetson で GPU を使うには、JetPack の版に合う NVIDIA 提供の torch が必要です")
        print("    (pip の標準 wheel は aarch64 でも CPU 専用)。")
    print(f"  開始時のCPU温度      : {cpu_temp_c():.1f} ℃")

    use_cuda = bool(env["cuda_available"])
    image = make_image(args.width, args.height)
    print(f"\n計測画像: {args.width}x{args.height} (G1のカメラ相当)")

    results = {"environment": env, "image": [args.width, args.height], "runs": [], "soak": {}}

    for weights in args.weights.split(","):
        weights = weights.strip()
        try:
            model = YOLO(weights)
        except Exception as e:
            print(f"\n[bench] {weights} を読み込めませんでした: {e}")
            print("[bench]   重みは初回にインターネットから自動取得されます。")
            print("[bench]   実機が外に出られない場合は、操作PCから scp で渡して --weights でパスを指定してください。")
            continue
        if use_cuda:
            model.to("cuda")

        print(f"\n########## {weights} ##########")
        print(f"{'入力':>6s} {'スレッド':>7s} {'中央値':>9s} {'最小':>8s} {'最大':>8s} {'メモリ':>9s}")
        for imgsz in (int(s) for s in args.sizes.split(",")):
            for nthreads in (int(t) for t in args.threads.split(",")):
                if nthreads > 0:
                    torch.set_num_threads(nthreads)
                    label = f"{nthreads}"
                else:
                    torch.set_num_threads(env["cpu_count"] or 1)
                    label = "制限なし"
                ts = measure(model, image, imgsz, args.repeat, use_cuda)
                med = statistics.median(ts)
                print(f"{imgsz:>6d} {label:>7s} {med:>7.0f}ms {min(ts):>6.0f}ms {max(ts):>6.0f}ms {rss_mb():>7.0f}MB")
                results["runs"].append({
                    "weights": weights, "imgsz": imgsz, "threads": label,
                    "median_ms": round(med, 1), "min_ms": round(min(ts), 1), "max_ms": round(max(ts), 1),
                    "fps_at_median": round(1000 / med, 1), "rss_mb": round(rss_mb(), 1),
                })

        if args.soak > 0:
            torch.set_num_threads(env["cpu_count"] or 1)
            imgsz0 = int(args.sizes.split(",")[0])
            print(f"\n  連続動作({args.soak:.0f}秒, 入力{imgsz0}, スレッド制限なし)…")
            s = soak_test(model, image, imgsz0, args.soak, use_cuda)
            results["soak"][weights] = s
            if "slowdown_pct" in s:
                print(f"    {s['frames']}回実行  前半 {s['first_half_ms']}ms → 後半 {s['second_half_ms']}ms "
                      f"({s['slowdown_pct']:+.1f}%)  終了時 {s['temp_after_c']}℃")
                if s["slowdown_pct"] > 15:
                    print("    ⚠ 後半で15%以上遅くなっています。発熱による性能低下の可能性があります。")

    if args.out:
        Path(args.out).write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n結果を保存しました: {args.out}")
        print("このファイルを操作PCへ持ち帰って記録に残してください(scp等)。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
