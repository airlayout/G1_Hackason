"""押す流れで G1 に喋らせる言葉（configs/voice.yaml）を、WAV にして PC2 に置く・試しに鳴らす（ノート PC で実行する）。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/real/voice_phrases.py make           # VOICEVOX で WAV を作る（Voice/out/phrases/<名前>.wav）
    $P Button_Press/J1-gen/real/voice_phrases.py send           # g1_speaker.py と WAV を PC2 の pc2.dir に送る
    $P Button_Press/J1-gen/real/voice_phrases.py play lit       # 1 つ PC2 で鳴らす（ssh 経由）
    $P Button_Press/J1-gen/real/voice_phrases.py play --all     # 全部を順に鳴らす

- PC2 の /tmp は再起動で消えるので、send はその日に 1 回（G1 を起動し直したらもう一度）
- make は VOICEVOX を入れた venv の python（既定 ~/voicevox_core/venv/bin/python。環境変数 VOICEVOX_PY で変える）で
  Voice/voicevox_speak.py --batch を 1 回動かす。その venv には PyYAML が無いので、言葉は JSON にして渡す
- PC2 の宛先は voice.yaml の pc2.host（環境変数 PC2 か --host で上書き）
- 押す流れで喋らせるのは real/auto_press.py --voice local / ssh
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.announce import speaker_command  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402

VOICE_DIR = REPO_ROOT / "Voice"
OUT_DIR = VOICE_DIR / "out" / "phrases"  # Voice/.gitignore で git 管理外
DEFAULT_VOICEVOX_PY = Path.home() / "voicevox_core" / "venv" / "bin" / "python"


def phrases_of(cfg: dict) -> dict[str, str]:
    """喋る言葉（文が空のものは除く）。"""
    return {str(k): str(v) for k, v in (cfg.get("phrases") or {}).items() if v and str(v).strip()}


def make(cfg: dict) -> int:
    py = Path(os.environ.get("VOICEVOX_PY") or DEFAULT_VOICEVOX_PY)
    if not py.exists():
        print(f"[voice] VOICEVOX の python が無い: {py}（入れ方は Voice/VOICEVOX.md。環境変数 VOICEVOX_PY で変えられる）")
        return 1
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    batch = OUT_DIR.parent / "phrases.json"
    batch.write_text(json.dumps(phrases_of(cfg), ensure_ascii=False, indent=2), encoding="utf-8")
    cmd = [str(py), str(VOICE_DIR / "voicevox_speak.py"), "--batch", str(batch), "--out-dir", str(OUT_DIR)]
    for key, v in (cfg.get("voice") or {}).items():
        if v is not None:
            cmd += [f"--{key}", str(v)]
    print(f"[voice] $ {' '.join(cmd)}")
    return subprocess.run(cmd).returncode


def send(cfg: dict, host: str) -> int:
    keys = list(phrases_of(cfg))
    wavs = [OUT_DIR / f"{k}.wav" for k in keys]
    missing = [w.name for w in wavs if not w.exists()]
    if missing:
        print(f"[voice] WAV が無い: {', '.join(missing)}（先に make）")
        return 1
    d = str(cfg["pc2"]["dir"])
    ssh = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
    for cmd in (["ssh", *ssh, host, f"mkdir -p {d}"],
                ["scp", *ssh, str(VOICE_DIR / "g1_speaker.py"), *map(str, wavs), f"{host}:{d}/"]):
        print(f"[voice] $ {' '.join(cmd)}")
        if subprocess.run(cmd, timeout=120).returncode != 0:
            print("[voice] ❌ 送れなかった（PC2 に ssh できるか、宛先 --host を確かめる）")
            return 1
    print(f"[voice] {len(wavs)} 個を {host}:{d}/ に送った: {', '.join(keys)}")
    return 0


def play(cfg: dict, host: str, keys: list[str]) -> int:
    os.environ["PC2"] = host  # speaker_command の ssh の宛先
    phrases = phrases_of(cfg)
    code = 0
    for k in keys:
        if k not in phrases:
            print(f"[voice] voice.yaml に無い名前: {k}（{', '.join(phrases)}）")
            return 1
        print(f"[voice] 「{phrases[k]}」（{k}）")
        r = subprocess.run(speaker_command("ssh", cfg, k), timeout=float(cfg["pc2"].get("timeout_s", 20)) + 10)
        if r.returncode != 0:
            print(f"[voice] ❌ 鳴らせなかった（終了コード {r.returncode}。send を済ませたか）")
            code = r.returncode
    return code


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=["make", "send", "play"])
    p.add_argument("keys", nargs="*", help="play: 鳴らす言葉の名前（voice.yaml の phrases の左側）")
    p.add_argument("--all", action="store_true", help="play: 全部を順に鳴らす")
    p.add_argument("--host", default=None, help="PC2 の ssh 宛先（既定は環境変数 PC2、無ければ voice.yaml の pc2.host）")
    p.add_argument("--config", default="voice.yaml", help="言葉の設定（configs/ の下）")
    args = p.parse_args()
    cfg = load_config(args.config)
    host = args.host or os.environ.get("PC2") or str(cfg["pc2"]["host"])
    if args.command == "make":
        return make(cfg)
    if args.command == "send":
        return send(cfg, host)
    keys = list(phrases_of(cfg)) if args.all else args.keys
    if not keys:
        p.error("play には名前か --all を付ける")
    return play(cfg, host, keys)


if __name__ == "__main__":
    sys.exit(main())
