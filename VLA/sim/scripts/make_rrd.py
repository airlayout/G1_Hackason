"""rollout の mp4 を Rerun の .rrd にまとめる。エピソードごとにタブで切り替えて見る。

各タブ: 左=ego（G1 の目線）/ 中=第三者視点 / 右=情報。動画は左右 2 分割の 1 枚絵なので、半分に割って記録する。
mp4 を Rerun に直接渡すと環境によって再生が止まるため、PyAV で復号して JPEG 画像として記録する。
エピソード一覧（名前・mp4・checkpoint・結果・メモ）は、マニフェスト JSON（既定は results/episodes.json）が持つ。

使い方:
  python3 -I make_rrd.py [--manifest M.json] [--videos-dir DIR] [--out OUT.rrd]
  rerun OUT.rrd          # Viewer で開く
依存: av rerun-sdk（VLA/sim/requirements-viewer.txt）
"""
import argparse
import json
import pathlib

import av
import rerun as rr
import rerun.blueprint as rrb

HERE = pathlib.Path(__file__).resolve().parent
RESULTS = HERE.parent / "results"
FPS = 20.0  # rollout の動画は 20fps
CAPTION_PX = 36  # 動画の下端に焼き込まれた指示文の帯。切り落とす
ICON = {"SUCCESS": "✅", "FAIL": "❌"}


def find_video(videos_dir: pathlib.Path, name: str) -> pathlib.Path:
    """videos_dir 以下（サブディレクトリ含む）から mp4 をファイル名で探す。"""
    hits = sorted(videos_dir.rglob(name))
    if not hits:
        raise SystemExit(f"[error] {name} が {videos_dir} 以下に無い。--videos-dir を確認")
    return hits[0]


def episode_tab(name: str, result: str) -> rrb.Horizontal:
    return rrb.Horizontal(
        rrb.Spatial2DView(origin=f"g1/{name}/ego_camera", name="ego（G1の目線）"),
        rrb.Spatial2DView(origin=f"g1/{name}/third_person", name="第三者視点"),
        rrb.TextDocumentView(origin=f"g1/{name}/info", name="情報"),
        column_shares=[4, 4, 2],
        name=f"{ICON.get(result, '❔')} {name}",
    )


def send_layout(episodes: list) -> None:
    # 成功のタブを先頭にする。最初の画面で成功が見える。
    ordered = sorted(episodes, key=lambda e: e["result"] != "SUCCESS")
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Vertical(
                rrb.TextDocumentView(origin="g1/instruction", name="指示文"),
                rrb.Tabs(
                    *[episode_tab(e["name"], e["result"]) for e in ordered],
                    rrb.TextDocumentView(origin="g1/summary", name="📋 まとめ"),
                ),
                row_shares=[1, 9],
            ),
            rrb.TimePanel(timeline="sim_time"),
        )
    )


def log_episode(ep: dict, mp4: pathlib.Path, stride: int, jpeg_q: int) -> int:
    name = ep["name"]
    info = f"checkpoint: {ep['checkpoint']}\nresult: {ep['result']}"
    if ep.get("note"):
        info += f"\n\n{ep['note']}"
    rr.log(f"g1/{name}/info", rr.TextDocument(info), static=True)
    n = 0
    with av.open(str(mp4)) as c:
        for i, f in enumerate(c.decode(video=0)):
            if i % stride:
                continue
            img = f.to_ndarray(format="rgb24")
            h, w, _ = img.shape
            half = w // 2
            rr.set_time("step", sequence=i)
            rr.set_time("sim_time", duration=i / FPS)
            rr.log(f"g1/{name}/ego_camera", rr.Image(img[: h - CAPTION_PX, :half]).compress(jpeg_quality=jpeg_q))
            rr.log(f"g1/{name}/third_person", rr.Image(img[: h - CAPTION_PX, half:]).compress(jpeg_quality=jpeg_q))
            n += 1
    return n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", type=pathlib.Path, default=RESULTS / "episodes.json")
    ap.add_argument("--videos-dir", type=pathlib.Path, default=RESULTS / "videos", help="mp4 を探すディレクトリ")
    ap.add_argument("--out", type=pathlib.Path, default=RESULTS / "g1_compare_tabs.rrd", help="出力する .rrd")
    ap.add_argument("--stride", type=int, default=3, help="何フレームごとに記録するか（既定 3 = 約 6.7fps）")
    ap.add_argument("--jpeg-quality", type=int, default=85)
    args = ap.parse_args()

    manifest = json.loads(args.manifest.read_text())
    episodes = manifest["episodes"]
    videos = {e["name"]: find_video(args.videos_dir.expanduser(), e["video"]) for e in episodes}

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rr.init("g1_vla_sim", spawn=False)
    rr.save(args.out)
    send_layout(episodes)
    rr.log("g1/instruction", rr.TextDocument(manifest["instruction"]), static=True)
    rr.log("g1/summary", rr.TextDocument("\n".join(manifest["summary"])), static=True)
    for ep in episodes:
        n = log_episode(ep, videos[ep["name"]], args.stride, args.jpeg_quality)
        print(f"[rrd] {ep['name']}: {n} frame pairs")
    print(f"[rrd] -> {args.out}")


if __name__ == "__main__":
    main()
