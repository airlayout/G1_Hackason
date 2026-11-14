"""録った音（real/mic_record.py の WAV）から、エレベーターの到着の音（高めのチャイム）を探し、スペクトログラムの図を作る。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/chime_analyze.py _local/button_press/sound/<日時>.wav
    # しきい値・帯を変えて試す（到着の音の高さが分かったら、帯を狭めると他の音と取り違えにくい）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/chime_analyze.py <wav> --band 2000 4000 --min-db 12

出力: 見つけた高い音（時刻・長さ・高さ・ふだんより何 dB 強いか）と、<wav と同じ名前>_spec.png（上: 音の波形、
下: スペクトログラム。見つけた区間に枠）。WAV と同じ名前の .json（mic_record.py が書く）があれば、録り始めの時刻も出し、
押した記録（_local/button_press/auto/*.json）と時刻で並べられるようにする。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.chime import detect_tones, level_dbfs, load_wav, spectrogram  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("wav", type=Path, nargs="+")
    p.add_argument("--band", type=float, nargs=2, default=[1000.0, 6000.0], metavar=("LOW_HZ", "HIGH_HZ"))
    p.add_argument("--min-db", type=float, default=15.0, help="ふだん（その周波数の中央値）より何 dB 強ければ音とみなすか")
    p.add_argument("--min-s", type=float, default=0.1, help="音とみなす長さの下限 [秒]")
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args()

    for path in args.wav:
        x, sr = load_wav(path)
        meta_path = path.with_suffix(".json")
        t0 = json.loads(meta_path.read_text()).get("first_packet_epoch") if meta_path.exists() else None
        print(f"[chime] {path.name}: {len(x) / sr:.1f} 秒、{sr} Hz、大きさ {level_dbfs(x):.1f} dBFS"
              + (f"、録り始め {time.strftime('%H:%M:%S', time.localtime(t0))}" if t0 else ""))
        tones = detect_tones(x, sr, tuple(args.band), args.min_db, args.min_s)
        for tone in tones:
            clock = f"（{time.strftime('%H:%M:%S', time.localtime(t0 + tone.start_s))}）" if t0 else ""
            print(f"[chime]   高い音: {tone.start_s:.2f}〜{tone.end_s:.2f} s{clock}、{tone.duration_s:.2f} 秒、"
                  f"{tone.peak_hz:.0f} Hz、ふだんより {tone.above_db:.0f} dB")
        if not tones:
            print(f"[chime]   高い音は見つからなかった（{args.band[0]:.0f}〜{args.band[1]:.0f} Hz、{args.min_db:.0f} dB、{args.min_s} 秒）")
        if not args.no_plot:
            out = plot(path, x, sr, tones, args.band)
            print(f"[chime]   図: {out}")
    return 0


def plot(path: Path, x, sr: int, tones: list, band: list[float]) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    t, f, db = spectrogram(x, sr)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True, gridspec_kw={"height_ratios": [1, 3]})
    a1.plot(np.arange(len(x)) / sr, x, lw=0.4)
    a1.set_ylabel("amplitude")
    a2.pcolormesh(t, f, db.T, shading="auto", vmin=np.percentile(db, 5), vmax=np.percentile(db, 99.5), cmap="magma")
    for y in band:
        a2.axhline(y, color="cyan", lw=0.6, ls="--")
    for tone in tones:
        a2.add_patch(plt.Rectangle((tone.start_s, band[0]), tone.duration_s, band[1] - band[0], fill=False, ec="lime", lw=1.2))
    a2.set_ylabel("Hz")
    a2.set_xlabel("s")
    a1.set_title(path.name)
    fig.tight_layout()
    out = path.with_name(path.stem + "_spec.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


if __name__ == "__main__":
    sys.exit(main())
