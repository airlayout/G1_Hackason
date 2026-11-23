"""スマホの映像で MASt3R-SLAM を回し、地図を Rerun のウェブ表示に出す。

    # 生の映像（MediaMTX の cam1）
    python run_phone_slam.py --source rtsp://127.0.0.1:8554/cam1
    # 録画を、生と同じく間に合わないこまを捨てながら回す（段 1 の試し）
    python run_phone_slam.py --source rec/cam1.mkv --realtime
    # TUM などの上流の dataset もそのまま渡せる
    python run_phone_slam.py --source datasets/tum/rgbd_dataset_freiburg1_desk

上流の main.py の主ループと同じ手順を、取り込み口と表示だけ差し替えて回す。
後ろの最適化（run_backend）は上流の main.py の物をそのまま使う。
較正は使わない（use_calib: False）。iPhone と Pixel のレンズの違いもそのまま受ける。
"""

from __future__ import annotations

import argparse
import collections
import os
import pathlib
import signal
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_REPO = pathlib.Path(os.environ.get("MAST3R_REPO", HERE.parent / "MASt3R-SLAM"))
FPS_WINDOW = 30
WAIT_LOG_EVERY_S = 10.0
LIVE_WAIT_S = 10.0  # 生の流れが止まったとき、この間隔で知らせながら待つ
VIZ_STALL_S = 60.0  # 表示のプロセスの心拍がこれだけ止まったら起こし直す
VIZ_CHECK_EVERY_S = 10.0
VIZ_MAX_RESTARTS = 3


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", required=True, help="rtsp://…・録画のファイル・dataset のフォルダ")
    p.add_argument("--repo", default=str(DEFAULT_REPO), help="MASt3R-SLAM の置き場所")
    p.add_argument("--config", default="config/base.yaml", help="--repo からの相対でもよい")
    p.add_argument("--realtime", action="store_true", help="録画を生の流れと同じく扱う（遅れたこまを捨てる）")
    p.add_argument("--stride", type=int, default=1, help="録画を realtime なしで回すときの間引き")
    p.add_argument("--kf-buffer", type=int, default=512, help="キーフレームの箱の数（GPU の記憶 約 6.5 MB／個）")
    p.add_argument("--save-dir", default="", help="終わるときに軌跡（traj.txt）と点群（map.ply）を書く")
    p.add_argument("--no-viz", action="store_true")
    p.add_argument("--hosts", default="", help="表示の URL に載せる IP（カンマ区切り）")
    p.add_argument("--pts-stride", type=int, default=2)
    p.add_argument("--conf-thresh", type=float, default=1.5)
    p.add_argument("--small-map-kf", type=int, default=10, help="これより少ないキーフレームの地図は見失ったら作り直す")
    p.add_argument("--reset-small-s", type=float, default=3.0, help="小さい地図を見失ってから作り直すまでの秒")
    p.add_argument("--reset-large-s", type=float, default=0.0, help="大きい地図も作り直すまでの秒（0 は作り直さず再測位を待つ）")
    p.add_argument("--reset-file", default="", help="このファイルが現れたら地図を作り直す（start_slam.sh reset）")
    return p.parse_args()


def make_source(src: str, realtime: bool, stride: int):
    from mast3r_slam.dataloader import load_dataset

    from phone_source import LiveRtspDataset, VideoFileDataset

    if src.startswith(("rtsp://", "rtsps://")):
        return LiveRtspDataset(src, frame_timeout_s=LIVE_WAIT_S)
    if pathlib.Path(src).suffix.lower() in {".mp4", ".mkv", ".mov", ".webm", ".avi"}:
        return VideoFileDataset(src, realtime=realtime, stride=stride)
    return load_dataset(src)  # 上流の dataset（TUM など）


class VizSupervisor:
    """表示のプロセスを起こし、心拍（status["viz_beat"]）が止まったら起こし直す。

    11-17 の試しで、表示のプロセスが rr.serve_web の中で戻らなくなった（rerun-io/rerun#8417。
    rerun_viz で呼ぶ順を直した）。また起きても SLAM は動き続けるので、表示だけを起こし直す。
    rerun_viz は SIGUSR1 で止まった所を記録に出す。
    起こし直した表示は、共有の地図から全部のキーフレームを送り直す。

    表示は映像を待たずに先に起こす（スマホが送る前からページを開ける）。共有の地図は、
    最初のこまで大きさが決まってから attach で渡す（CUDA の共有テンソルは torch の Queue で渡せる）。
    """

    def __init__(self, mp, target, cfg: dict, status, opts) -> None:
        self.mp = mp
        self.target = target
        self.cfg = cfg
        self.status = status
        self.opts = opts
        self.proc = None
        self.queue = None
        self.shared: tuple | None = None
        self.started_at = 0.0
        self.checked_at = 0.0
        self.restarts = 0

    def start(self) -> None:
        self.queue = self.mp.Queue()
        self.proc = self.mp.Process(
            target=self.target, args=(self.cfg, self.queue, self.status, self.opts), name="viz"
        )
        self.proc.start()
        self.started_at = time.time()
        if self.shared is not None:
            self.queue.put(self.shared)

    def attach(self, states, keyframes) -> None:
        self.shared = (states, keyframes)
        self.queue.put(self.shared)

    def check(self) -> None:
        now = time.time()
        if self.proc is None or now - self.checked_at < VIZ_CHECK_EVERY_S:
            return
        self.checked_at = now
        beat = max(float(self.status.get("viz_beat", 0.0)), self.started_at)
        dead = not self.proc.is_alive()  # 落ちた（11-17 に GPU のエラーで落ちた）ならすぐ起こし直す
        if not dead and now - beat < VIZ_STALL_S:
            return
        if self.restarts >= VIZ_MAX_RESTARTS:
            print(f"[SLAM] 表示が止まった（{now - beat:.0f} 秒）。起こし直しは {VIZ_MAX_RESTARTS} 回で打ち止め", flush=True)
            self.proc = None
            return
        self.restarts += 1
        why = f"落ちた（終了コード {self.proc.exitcode}）" if dead else f"{now - beat:.0f} 秒止まった"
        print(f"[SLAM] 表示が{why}。起こし直す（{self.restarts} 回目）", flush=True)
        if not dead:
            os.kill(self.proc.pid, signal.SIGUSR1)  # 止まった所を記録に出してから
            time.sleep(1.0)
        self.proc.kill()
        self.proc.join(timeout=10)
        self.start()

    def stop(self) -> None:
        if self.proc is None:
            return
        self.proc.join(timeout=5)
        if self.proc.is_alive():
            self.proc.terminate()


class MapResetter:
    """見失ったまま戻れない地図を捨て、いまのこまから作り直す（上流の MASt3R-SLAM には無い）。

    上流は見失うと「前に見た所」への再測位しか試さない。11-17 の iPhone の試しで、最初のこま
    （間近のモニターの黒い裏とぶれた床）で地図を始め、2 こま目で見失い、キーフレーム 1 枚の地図に
    2,400 回再測位を試して戻れなかった。小さい地図はすぐ捨てて作り直す。大きい地図は既定では
    捨てない（前に歩いた所へ戻れば再測位できる）。

    作り直しは、後ろの最適化のプロセスを止め（因子グラフと検索の記憶を持っている）、共有の
    キーフレームと状態を空にし、INIT に戻して後ろのプロセスを起こし直す。
    """

    def __init__(self, args, start_backend, states, keyframes, Mode, status) -> None:
        self.args = args
        self.start_backend = start_backend
        self.states = states
        self.keyframes = keyframes
        self.Mode = Mode
        self.status = status
        self.backend = start_backend()
        self.lost_since: float | None = None
        self.resets = 0
        self.reset_file = pathlib.Path(args.reset_file) if args.reset_file else None

    def reason(self, mode) -> str:
        """作り直すべきなら理由を返す。要らなければ空"""
        if self.reset_file is not None and self.reset_file.exists():
            self.reset_file.unlink(missing_ok=True)
            return "requested"
        if mode != self.Mode.RELOC:
            self.lost_since = None
            self.status["lost_s"] = 0.0
            return ""
        now = time.time()
        if self.lost_since is None:
            self.lost_since = now
        lost = now - self.lost_since
        self.status["lost_s"] = lost
        n = len(self.keyframes)
        if n < self.args.small_map_kf and lost >= self.args.reset_small_s:
            return f"lost {lost:.0f}s with {n} keyframes"
        if self.args.reset_large_s > 0 and lost >= self.args.reset_large_s:
            return f"lost {lost:.0f}s"
        return ""

    def reset(self, why: str) -> None:
        self.states.set_mode(self.Mode.TERMINATED)  # 後ろのプロセスはこれを見て抜ける
        self.backend.join(timeout=30)
        if self.backend.is_alive():
            self.backend.terminate()
            self.backend.join(timeout=5)
        kf, st = self.keyframes, self.states
        with kf.lock:
            kf.n_size.value = 0
            kf.is_dirty[:] = False
        with st.lock:
            del st.global_optimizer_tasks[:]
            del st.edges_ii[:]
            del st.edges_jj[:]
            st.reloc_sem.value = 0
            st.paused.value = 0
        st.set_mode(self.Mode.INIT)
        self.backend = self.start_backend()
        self.resets += 1
        self.lost_since = None
        note = f"map reset #{self.resets} ({why})"
        self.status.update(note=note, lost_s=0.0)
        print(f"[SLAM] 地図を作り直す: {note}", flush=True)


class StopFlag:
    """Ctrl-C と SIGTERM で主ループを止め、保存してから終わる"""

    def __init__(self) -> None:
        self.stop = False
        signal.signal(signal.SIGINT, self._handle)
        signal.signal(signal.SIGTERM, self._handle)

    def _handle(self, signum, frame) -> None:  # noqa: ARG002
        print(f"[SLAM] 止めの合図（{signal.Signals(signum).name}）。保存して終わる", flush=True)
        self.stop = True


def source_status(dataset) -> dict:
    stats = getattr(dataset, "stats", None)
    if stats is None:
        return {}
    return {
        "recv_fps": stats.receive_fps,
        "recv_shape": tuple(stats.last_shape),
        "received": stats.received,
        "used": stats.used,
        "reconnects": stats.reconnects,
    }


def main() -> None:
    args = parse_args()
    repo = pathlib.Path(args.repo).resolve()
    os.chdir(repo)  # 上流は重み（checkpoints/）を相対で読む
    sys.path.insert(0, str(repo))
    sys.path.insert(0, str(HERE))

    import lietorch
    import torch
    import torch.multiprocessing as mp

    import main as upstream  # 上流の main.py（run_backend を使う）
    import mast3r_slam.evaluate as evaluate
    from mast3r_slam.config import config, load_config
    from mast3r_slam.frame import Mode, SharedKeyframes, SharedStates, create_frame
    from mast3r_slam.mast3r_utils import load_mast3r, mast3r_inference_mono
    from mast3r_slam.tracker import FrameTracker
    from rerun_viz import VizOptions, run_rerun_viz, viewer_urls

    mp.set_start_method("spawn")
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.set_grad_enabled(False)
    device = "cuda:0"

    load_config(args.config)
    if config["use_calib"]:
        raise SystemExit("[SLAM] 較正ありの設定は使わない（use_calib: False の設定を渡す）")
    stop = StopFlag()
    manager = mp.Manager()
    status = manager.dict(source=args.source, kf_buffer=args.kf_buffer, stop=False, waiting=True)

    viz = None
    if not args.no_viz:  # 表示は映像を待たずに先に起こす
        hosts = tuple(x for x in args.hosts.split(",") if x) or ("127.0.0.1",)
        import viewer_hub

        src_name = args.source.rstrip("/").rsplit("/", 1)[-1]
        opts = VizOptions(
            name=src_name,
            hosts=hosts,
            pts_stride=args.pts_stride,
            conf_thresh=args.conf_thresh,
            recording_id=viewer_hub.run_recording_id(f"phone_vslam mono {src_name}"),
        )
        viz = VizSupervisor(mp, run_rerun_viz, config, status, opts)
        viz.start()
        for url in viewer_urls(opts):
            print(f"[SLAM] 表示: {url}", flush=True)

    # 重みも映像を待たずに読む（送り始めたらすぐ追跡が始まる）
    model = load_mast3r(device=device)
    model.share_memory()

    dataset = make_source(args.source, args.realtime, args.stride)
    dataset.subsample(config["dataset"]["subsample"])
    print(f"[SLAM] 最初のこまを待つ: {args.source}", flush=True)
    while True:  # スマホが送り始めるまで待つ
        try:
            h, w = dataset.get_img_shape()[0]
            break
        except TimeoutError as e:
            if stop.stop:
                status["stop"] = True
                if viz is not None:
                    viz.stop()
                return
            if viz is not None:
                viz.check()
            print(f"[SLAM] {e}", flush=True)
    print(f"[SLAM] SLAM の画の大きさ {w}x{h}", flush=True)

    keyframes = SharedKeyframes(manager, h, w, buffer=args.kf_buffer)
    states = SharedStates(manager, h, w)
    if viz is not None:
        viz.attach(states, keyframes)

    def start_backend():
        proc = mp.Process(target=upstream.run_backend, args=(config, model, states, keyframes, None), name="backend")
        proc.start()
        return proc

    tracker = FrameTracker(model, keyframes, device)
    resetter = MapResetter(args, start_backend, states, keyframes, Mode, status)

    i = 0
    stamps: collections.deque[float] = collections.deque(maxlen=FPS_WINDOW)
    last_wait_log = 0.0
    paused = False
    while not stop.stop:
        mode = states.get_mode()
        if i >= len(dataset):
            break
        try:
            _, img = dataset[i]
        except StopIteration:
            break  # 録画の終わり
        except TimeoutError as e:  # 生の流れが止まった。つながり直すのを待つ
            # 後ろのプロセスは見失った状態だと、新しいこまが無くても最後のこまの再測位を休みなく
            # 繰り返して GPU を使い続ける（上流の作り）。映像が来ない間は一時停止させる
            if not paused:
                states.pause()
                paused = True
            status.update(waiting=True, **source_status(dataset))
            if time.time() - last_wait_log > WAIT_LOG_EVERY_S:
                print(f"[SLAM] {e}", flush=True)
                last_wait_log = time.time()
            continue
        status["waiting"] = False
        if paused:
            states.unpause()
            paused = False
        why = resetter.reason(mode)
        if why:
            resetter.reset(why)
            tracker = FrameTracker(model, keyframes, device)  # 追跡の持ち越し（前のこまの対応）も捨てる
            mode = states.get_mode()

        T_WC = lietorch.Sim3.Identity(1, device=device) if mode == Mode.INIT else states.get_frame().T_WC
        frame = create_frame(i, img, T_WC, img_size=dataset.img_size, device=device)

        add_new_kf = False
        if mode == Mode.INIT:
            X_init, C_init = mast3r_inference_mono(model, frame)
            frame.update_pointmap(X_init, C_init)
            keyframes.append(frame)
            states.queue_global_optimization(len(keyframes) - 1)
            states.set_mode(Mode.TRACKING)
            states.set_frame(frame)
        elif mode == Mode.TRACKING:
            add_new_kf, _, try_reloc = tracker.track(frame)
            if try_reloc:
                states.set_mode(Mode.RELOC)
            states.set_frame(frame)
        elif mode == Mode.RELOC:
            X, C = mast3r_inference_mono(model, frame)
            frame.update_pointmap(X, C)
            states.set_frame(frame)
            states.queue_reloc()
        else:
            raise RuntimeError(f"知らない状態: {mode}")

        if add_new_kf:
            if len(keyframes) >= keyframes.buffer - 1:
                print(f"[SLAM] キーフレームが上限（{keyframes.buffer}）。--kf-buffer を増やす", flush=True)
                status["note"] = "keyframe buffer full - stopped"
                break
            keyframes.append(frame)
            states.queue_global_optimization(len(keyframes) - 1)

        if viz is not None:
            viz.check()
        stamps.append(time.time())
        fps = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
        status.update(slam_fps=fps, frame_idx=i, **source_status(dataset))
        if i % 30 == 0:
            print(f"[SLAM] こま {i}  {fps:.1f} fps  キーフレーム {len(keyframes)}  状態 {states.get_mode().name}", flush=True)
        i += 1

    states.set_mode(Mode.TERMINATED)
    resetter.backend.join(timeout=30)
    if args.save_dir and len(keyframes) > 0:
        save_dir = pathlib.Path(args.save_dir).expanduser().resolve()
        evaluate.save_traj(save_dir, "traj.txt", dataset.timestamps, keyframes)
        evaluate.save_reconstruction(save_dir, "map.ply", keyframes, args.conf_thresh)
        print(f"[SLAM] 保存した: {save_dir}/traj.txt・map.ply（キーフレーム {len(keyframes)}）", flush=True)
    status["stop"] = True
    if viz is not None:
        viz.stop()
    if hasattr(dataset, "grabber"):
        dataset.grabber.close()
    print(f"[SLAM] 終わり（こま {i}・キーフレーム {len(keyframes)}）", flush=True)


if __name__ == "__main__":
    main()
