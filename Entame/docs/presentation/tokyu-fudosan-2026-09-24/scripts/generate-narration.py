#!/usr/bin/env python3
"""slides.html に埋め込まれた台本(talk-script)から、VOICEVOX ENGINEを使って
スライドごとのナレーションWAVを生成する。

前提: ローカルでVOICEVOX ENGINEが起動していること。
  docker run --rm -p 50021:50021 voicevox/voicevox_engine:cpu-ubuntu20.04-latest

台本の正本は slides.html 内の talk-script であり、このスクリプトは
それを読み取って音声を再生成するだけ。台本を直したら必ず再実行する。
"""

import json
import re
import sys
import urllib.parse
import urllib.request
from pathlib import Path

ENGINE_URL = "http://127.0.0.1:50021"
SPEAKER = 2
SPEED_SCALE = 0.84  # 本番の話速目安270〜300字/分に合わせた補正。既定速度は速すぎる。

PROJECT_DIR = Path(__file__).resolve().parent.parent
SLIDES_HTML = PROJECT_DIR / "slides.html"
AUDIO_DIR = PROJECT_DIR / "audio"

SLIDE_RE = re.compile(
    r'<span class="slide-number mono">(\d+)</span>.*?'
    r'<div class="talk-script">.*?<p>(.*?)</p>',
    re.S,
)

CHARS_PER_MINUTE = 280  # 本文表示用の推定秒数計算に使う目安(1分あたり文字数)


def check_engine_alive() -> None:
    try:
        with urllib.request.urlopen(f"{ENGINE_URL}/version", timeout=5) as res:
            version = res.read().decode("utf-8")
        print(f"[OK] VOICEVOX ENGINE 起動確認 (version={version})")
    except Exception as e:
        print(
            "[ERROR] VOICEVOX ENGINEに接続できませんでした。\n"
            f"  対象URL: {ENGINE_URL}\n"
            f"  詳細: {e}\n"
            "  以下のコマンドでENGINEを起動してください:\n"
            "    docker run --rm -p 50021:50021 voicevox/voicevox_engine:cpu-ubuntu20.04-latest",
            file=sys.stderr,
        )
        sys.exit(1)


def strip_tags(body: str) -> str:
    text = re.sub(r"<[^>]+>", "", body)
    return re.sub(r"\s+", "", text).strip()


def extract_slides() -> list[tuple[str, str]]:
    if not SLIDES_HTML.exists():
        print(f"[ERROR] {SLIDES_HTML} が見つかりません。", file=sys.stderr)
        sys.exit(1)

    html = SLIDES_HTML.read_text(encoding="utf-8")
    matches = SLIDE_RE.findall(html)

    if not matches:
        print(
            "[ERROR] slides.html から1件も台本を抽出できませんでした。"
            " slide-number / talk-script の構造を確認してください。",
            file=sys.stderr,
        )
        sys.exit(1)

    slides = []
    for number, raw_body in matches:
        text = strip_tags(raw_body)
        if not text:
            print(f"[ERROR] スライド{number}の台本が空です。", file=sys.stderr)
            sys.exit(1)
        slides.append((number, text))
    return slides


def synthesize(text: str) -> bytes:
    query_url = f"{ENGINE_URL}/audio_query?speaker={SPEAKER}&text={urllib.parse.quote(text)}"
    req = urllib.request.Request(query_url, method="POST")
    with urllib.request.urlopen(req, timeout=30) as res:
        query = json.loads(res.read().decode("utf-8"))

    query["speedScale"] = SPEED_SCALE

    synth_url = f"{ENGINE_URL}/synthesis?speaker={SPEAKER}"
    body = json.dumps(query).encode("utf-8")
    req = urllib.request.Request(
        synth_url, data=body, method="POST", headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=60) as res:
        return res.read()


def main() -> None:
    check_engine_alive()

    slides = extract_slides()
    AUDIO_DIR.mkdir(parents=True, exist_ok=True)

    for number, text in slides:
        char_count = len(text)
        est_seconds = char_count / CHARS_PER_MINUTE * 60
        print(f"[slide {number}] {char_count}文字 / 推定{est_seconds:.1f}秒")

        wav_bytes = synthesize(text)
        out_path = AUDIO_DIR / f"slide_{int(number)}.wav"
        out_path.write_bytes(wav_bytes)
        print(f"  -> {out_path}")

    print(f"[DONE] {len(slides)}件のナレーションを生成しました。")


if __name__ == "__main__":
    main()
