"""2 台以上のスマホの映像で 1 つの地図を作る（Multicam-MASt3R-SLAM を直して、生の映像につないだ物）。

    # 生の映像（MediaMTX の cam1・cam2）
    python run_multicam_slam.py --source rtsp://127.0.0.1:8554/cam1,rtsp://127.0.0.1:8554/cam2
    # 録画（生と同じく、間に合わないこまを捨てる）
    python run_multicam_slam.py --source rec/cam1.mp4,rec/cam2.mp4 --realtime
    # 写真のフォルダ・TUM（こまを順に 1 枚ずつ交互に。試し用）
    python run_multicam_slam.py --source datasets/tum/..._desk,datasets/tum/..._desk2

両手に 1 台ずつ持って歩いてよい（2 台の間の位置は決めなくてよい・較正しない）。
はじめに 2 台で同じ所を写すと、2 台目がすぐ地図に重なる。重なった後は別々に歩いてよい。
2 台目は、地図の中の「見たことのある所」を写すと重なる（検索で見つける）。

大きさ（何 m か）は分からない（単眼と同じ）。2 台の地図の大きさは揃う。
保存（--save-dir）: traj_<名前>.txt（カメラごとのキーフレームの軌跡）・map.ply（全部の点）。
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import os
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_REPO = pathlib.Path(os.environ.get("MULTICAM_REPO", HERE.parent / "Multicam-MASt3R-SLAM"))
# 見失った（合流前の）カメラが再測位を試す間隔（その間のこまは捨てる）。再測位はキーフレームの箱の錠を握ったまま
# 照合する（上流と同じ。途中で別のカメラのキーフレームが入ると、失敗時の pop_last がそれを消してしまうため）。
# その間はもう一方のカメラの追跡も待つので、試す回数を絞る
RELOC_PERIOD_S = 1.0
IDLE_SLEEP_S = 0.003
WAIT_LOG_EVERY_S = 10.0
FPS_WINDOW = 20
FPS_STALE_S = 3.0
# 地図を始める・重ねる（再測位）前の関門。11-17 の実機で、伏せた Pixel の真っ暗な画で地図を始めては
# 見失って作り直す、を 9 回繰り返した。暗い画・模様の無い画（白い壁・天井）では始めない・試さない
GATE_LONG_SIDE = 320
MIN_MEAN_BRIGHTNESS = 25.0  # 0..255
MIN_GATE_CORNERS = 60  # 長い辺 320 に縮めた画の角の数
VIDEO_SUFFIXES = {".mp4", ".mkv", ".mov", ".webm", ".avi"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", required=True, help="カンマ区切りで 2 つ以上（rtsp://…・録画・写真のフォルダ）")
    p.add_argument("--names", default="", help="カメラの名前（カンマ区切り。既定は rtsp の名前かファイル名）")
    p.add_argument("--repo", default=str(DEFAULT_REPO), help="Multicam-MASt3R-SLAM の置き場所")
    p.add_argument("--config", default="config/base.yaml", help="--repo からの相対でもよい")
    p.add_argument("--realtime", action="store_true", help="録画を生の流れと同じく扱う（遅れたこまを捨てる）")
    p.add_argument("--stride", type=int, default=1, help="録画・フォルダを順に回すときの間引き")
    p.add_argument("--max-frames", type=int, default=0, help="フォルダを順に回すとき、1 台あたりこの枚数で止める（0 は全部）")
    p.add_argument("--kf-buffer", type=int, default=512, help="キーフレームの箱の数（GPU の記憶 約 6.5 MB／個）")
    p.add_argument("--save-dir", default="")
    p.add_argument("--no-viz", action="store_true")
    p.add_argument("--hosts", default="", help="表示の URL に載せる IP（カンマ区切り）")
    # 表示に送る点の間引き。2 台はキーフレームが 1 台の倍の速さで増えるので 3（1 枚 約 1.6 万点）。
    # 2 にすると TUM の 2 本の試しで 30 秒に 26〜43 MB を送った
    p.add_argument("--pts-stride", type=int, default=3)
    p.add_argument("--conf-thresh", type=float, default=1.5)
    p.add_argument("--small-map-kf", type=int, default=10, help="これより少ないキーフレームの地図は、全部の台が見失ったら作り直す")
    p.add_argument("--reset-small-s", type=float, default=3.0)
    p.add_argument("--reset-large-s", type=float, default=0.0)
    p.add_argument("--reset-file", default="")
    # 速さ（11-17 に測った: tests/bench_mast3r.py）
    p.add_argument("--no-fp16", dest="fp16", action="store_false",
                   help="MASt3R の推論を上流と同じ fp32 で回す（既定は fast_mast3r の fp16。追跡 92.6→73.1 ms・点のずれ 0.09%%）")
    p.add_argument("--recent-skip", type=int, default=4,
                   help="後ろの最適化で、検索で見つけた同じカメラの直前この枚数を照合しない（0 で上流と同じく全部）")
    # 既定は緩い再測位（11-18 の実機の録画で、終わりに始めの机へ戻った所を、厳しい形では検索 3 枚のうち 2 枚しか
    # 合わず重なれなかった。緩い形で重なってループが閉じ、始めと終わりの間が 9 → 0.4 になった）
    p.add_argument("--reloc-strict", dest="reloc_loose", action="store_false",
                   help="見失った後の再測位を上流と同じ厳しい形にする（検索で見つけた 3 枚すべてが合えば戻る）")
    p.add_argument("--reloc-loose", dest="reloc_loose", action="store_true", default=True,
                   help="（既定）検索で見つけた 3 枚のうち --reloc-min-kept 枚以上合えば戻る")
    p.add_argument("--reloc-min-kept", type=int, default=2,
                   help="緩い再測位で、照合が合う相手が何枚以上なら重ねるか（地図のキーフレームがそれより少ない間はその数）")
    p.add_argument("--reloc-buddy", type=int, default=2,
                   help="見失ったカメラの再測位の相手に、もう一方のスマホの最新この枚数のキーフレームを加える（0 で加えない）")
    p.add_argument("--reloc-min-match", type=float, default=0.0,
                   help="再測位の照合の割合の下限（0 は設定のまま＝0.3）")
    p.add_argument("--img-size", type=int, default=512,
                   help="MASt3R に渡す画の長い辺（上流は 512。384 にすると画素が約 0.56 倍で軽い）")
    p.add_argument("--gn-iters", type=int, default=5,
                   help="生の間の全体の最適化の反復（0 で上流の設定のまま＝10）")
    p.add_argument("--final-gn-iters", type=int, default=20,
                   help="止めて保存する前に、全体をこの反復でもう一度最適化する（0 でしない）")
    return p.parse_args()


def source_name(src: str) -> str:
    return pathlib.Path(src.rstrip("/")).stem if "://" not in src else src.rstrip("/").rsplit("/", 1)[-1]


# ---------------------------------------------------------------- 取り込み口


class LiveFeed:
    """生の流れ。新しいこまが来ていれば返す（待たない）"""

    def __init__(self, url: str) -> None:
        from rtsp_grabber import LatestFrameGrabber

        self.url = url
        self.grabber = LatestFrameGrabber(url)
        self.seq = 0
        self.stats = self.grabber.stats

    def poll(self):
        item = self.grabber.latest()
        if item is None or item[1] == self.seq:
            return None
        t, self.seq, bgr = item
        return t, bgr[:, :, ::-1]  # RGB（uint8）

    def close(self) -> None:
        self.grabber.close()


class VideoFeed:
    """録画を ffmpeg で順に読む（途中で解像度が変わる iPhone の録画も読める。video_file.py）。

    realtime なら生の流れと同じく、壁の時計に合わせて間に合わないこまを捨てる。そうでなければ stride おきに全部
    """

    def __init__(self, path: str, realtime: bool, stride: int, max_frames: int) -> None:
        from rtsp_grabber import SourceStats
        from video_file import FfmpegVideoReader

        self.reader = FfmpegVideoReader(path)
        self.realtime = realtime
        self.stride = max(1, stride)
        self.limit = max_frames if max_frames > 0 else None
        self.used = 0
        self.wall0: float | None = None
        self.ahead = None  # 生と同じ速さで読むとき、まだ時刻の来ていない次のこま
        self.stats = SourceStats()
        self.stats.last_shape = tuple(self.reader.out_wh[::-1])

    def ended(self) -> StopIteration:
        """読み終わりの知らせ（何こま目で、ffmpeg がどう終わったか）"""
        r = self.reader
        wall = f"・読み始めから {time.time() - self.wall0:.1f} 秒" if self.wall0 else ""
        print(f"[取り込み] {r.path} の終わり: {r.n} こま・録画の中 {r.last_t:.1f} 秒{wall}・ffmpeg の終わり {r.proc.poll()}", flush=True)
        return StopIteration()

    def poll(self):
        if self.limit is not None and self.used >= self.limit:
            raise StopIteration
        if self.realtime:
            # 壁の時計までに来ているこまのうち、いちばん新しい 1 こまを渡す（生の流れの「最新のこま」と同じ）。
            # まだ時刻の来ていないこまは渡さず取っておく。前は呼ばれるたびに少なくとも 1 こま進めていたので、
            # 主ループが空回りで速く呼ぶと録画が早送りになった（11-17: 67 秒の録画が 35 秒で終わった）
            now = time.time()
            self.wall0 = self.wall0 or now
            target = now - self.wall0
            item = None
            while True:
                nxt = self.ahead if self.ahead is not None else self.reader.read()
                self.ahead = None
                if nxt is None:
                    if item is not None:
                        break  # 最後のこまは渡す。終わりは次に呼ばれたときに知らせる
                    raise self.ended()
                if nxt[0] > target:
                    self.ahead = nxt
                    break
                item = nxt
            if item is None:
                return None
        else:
            for _ in range(self.stride):
                item = self.reader.read()
                if item is None:
                    raise self.ended()
        self.used += 1
        self.stats.received = self.reader.n
        return item

    def close(self) -> None:
        self.reader.close()


class DatasetFeed:
    """写真のフォルダ・TUM など上流の dataset を、呼ばれるたびに次のこまを返す口にする"""

    def __init__(self, src: str, stride: int, max_frames: int) -> None:
        from mast3r_slam.dataloader import load_dataset

        self.ds = load_dataset(src)
        self.ds.subsample(stride)
        self.stats = None
        self.i = 0
        self.limit = max_frames if max_frames > 0 else None

    def poll(self):
        if self.i >= len(self.ds) or (self.limit is not None and self.i >= self.limit):
            raise StopIteration
        img = self.ds.read_img(self.i)  # RGB（uint8）
        t = self.ds.get_timestamp(self.i)
        self.i += 1
        return float(t), img

    def close(self) -> None:
        pass


def make_feed(src: str, args):
    if src.startswith(("rtsp://", "rtsps://")):
        return LiveFeed(src)
    if pathlib.Path(src).suffix.lower() in VIDEO_SUFFIXES:
        return VideoFeed(src, args.realtime, args.stride, args.max_frames)
    return DatasetFeed(src, args.stride, args.max_frames)


# ---------------------------------------------------------------- カメラごとの状態（主のプロセスだけが持つ）


@dataclasses.dataclass
class Cam:
    idx: int
    name: str
    feed: object
    tracker: object = None
    T_WC: object = None  # 前のこまの姿勢（追跡の始め）
    last_mode: object = None
    reloc_at: float = float("-inf")
    offline: bool = False  # 録画・写真を順に回す（生と同じ速さではない）。再測位の間隔を録画の中の時刻で数える
    lost_since: float | None = None
    received: int = 0
    used: int = 0
    stamps: collections.deque = dataclasses.field(default_factory=lambda: collections.deque(maxlen=FPS_WINDOW))
    ended: bool = False
    problem: str = ""  # 地図を始める・重ねるのに向かない理由（表示に出す）

    def fps(self) -> float:
        s = self.stamps
        if len(s) < 2 or s[-1] <= s[0] or time.time() - s[-1] > FPS_STALE_S:
            return 0.0  # こまが来ていない（送信が止まった）のに前の値を出し続けない
        return (len(s) - 1) / (s[-1] - s[0])


def conform_to(img: np.ndarray, ref_hw: tuple[int, int]) -> np.ndarray:
    """2 台目以降のこまを、地図の箱の大きさ（最初のこまの向き・縦横比・大きさ）に揃える"""
    from phone_source import FrameConformer

    c = FrameConformer()
    c.ref_hw = ref_hw
    return c(img)


def view_problem(img_rgb: np.ndarray) -> str:
    """地図を始める・重ねるのに向かない画なら理由（英語・表示に出す）。向いていれば空"""
    import cv2

    h, w = img_rgb.shape[:2]
    k = GATE_LONG_SIDE / max(h, w)
    g = cv2.cvtColor(cv2.resize(img_rgb, (max(1, int(w * k)), max(1, int(h * k))), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2GRAY)
    if float(g.mean()) < MIN_MEAN_BRIGHTNESS:
        return "too dark"
    pts = cv2.goodFeaturesToTrack(g, maxCorners=500, qualityLevel=0.01, minDistance=5)
    if pts is None or len(pts) < MIN_GATE_CORNERS:
        return "no texture"
    return ""


class MultiCamSlam:
    """主ループ。カメラを順に見て、新しいこまがあれば追跡・地図を始める・合流（再測位）を回す"""

    def __init__(self, args, cams: list[Cam], ctx) -> None:
        self.args = args
        self.cams = cams
        self.ctx = ctx  # 遅く読み込む物（torch・mast3r_slam）をまとめた入れ物
        self.timestamps: list[float] = []
        self.ref_hw: tuple[int, int] | None = None
        self.n_frames = 0
        self.lost_all_since: float | None = None
        self.resets = 0
        self.t_first: float | None = None  # 最初のこまを受けた時刻（速さのまとめに使う）

    # ---- 地図の箱と後ろのプロセス

    def setup_map(self, h: int, w: int) -> None:
        c = self.ctx
        self.keyframes = c.CamKeyframes(c.manager, h, w, buffer=self.args.kf_buffer)
        self.states = c.SharedStates(c.manager, h, w, num_cams=len(self.cams))
        self.reloc_pending = c.mp.Array("i", len(self.cams), lock=False)
        for cam in self.cams:
            cam.tracker = c.FrameTracker(c.model, self.keyframes, c.device)
        self.backend = self.start_backend()

    def start_backend(self):
        c = self.ctx
        self.states.set_mode(c.Mode.TRACKING)  # 全体の状態は「止める」の合図にだけ使う（カメラごとの状態は camera_modes）
        proc = c.mp.Process(
            target=c.run_backend_multicam,
            args=(c.config, c.model, self.states, self.keyframes, self.reloc_pending, c.device,
                  self.args.fp16, self.args.recent_skip, self.args.gn_iters, self.args.final_gn_iters),
            name="backend",
        )
        proc.start()
        return proc

    def stop_backend(self) -> None:
        self.states.set_mode(self.ctx.Mode.TERMINATED)
        self.backend.join(timeout=120)  # 後ろは終わる前に全体を解き直す（--final-gn-iters。大きい地図で数秒）
        if self.backend.is_alive():
            self.backend.terminate()
            self.backend.join(timeout=5)

    def reset_map(self, why: str) -> None:
        """全部の台が見失った小さな地図を捨て、次に来たこまから作り直す"""
        c = self.ctx
        self.stop_backend()
        kf, st = self.keyframes, self.states
        with kf.lock:
            kf.n_size.value = 0
            kf.is_dirty[:] = False
        with st.lock:
            del st.global_optimizer_tasks[:]
            del st.edges_ii[:]
            del st.edges_jj[:]
            st.paused.value = 0
        for cam in self.cams:
            st.set_camera_mode(c.Mode.INIT, cam.idx)
            self.reloc_pending[cam.idx] = 0
            cam.tracker = c.FrameTracker(c.model, kf, c.device)
            cam.T_WC = None
            cam.lost_since = None
        self.backend = self.start_backend()
        self.resets += 1
        self.lost_all_since = None
        note = f"map reset #{self.resets} ({why})"
        self.ctx.status["note"] = note
        print(f"[SLAM] 地図を作り直す: {note}", flush=True)

    # ---- 1 こま

    def new_frame(self, cam: Cam, t: float, img_rgb: np.ndarray, T_WC):
        c = self.ctx
        img = img_rgb.astype(np.float32) / 255.0
        fid = len(self.timestamps)
        self.timestamps.append(t)
        return c.create_frame(fid, img, T_WC, img_size=self.args.img_size, device=c.device, cameraID=cam.idx)

    def kf_pose(self, cam: Cam):
        i = self.keyframes.last_index(cam.idx)
        return None if i is None else self.ctx.lietorch.Sim3(self.keyframes.T_WC[i].clone())

    def step_camera(self, cam: Cam, t: float, img_rgb: np.ndarray) -> None:
        c = self.ctx
        Mode = c.Mode
        st, kf = self.states, self.keyframes
        mode = st.get_camera_mode(cam.idx)
        if mode == Mode.TRACKING and cam.last_mode != Mode.TRACKING:
            # いま地図に重なった（合流・再測位）。重ねたキーフレームの姿勢から追跡を始める
            cam.T_WC = self.kf_pose(cam)
            cam.tracker.reset_idx_f2k()
            print(f"[SLAM] {cam.name} が地図に重なった（キーフレーム {len(kf)}）", flush=True)
        cam.last_mode = mode

        if mode in (Mode.INIT, Mode.RELOC):
            cam.problem = view_problem(img_rgb)
            if cam.problem:
                return  # 暗い・模様が無い画では地図を始めない・重ねようとしない
        else:
            cam.problem = ""

        if mode == Mode.INIT and len(kf) == 0:
            # 地図がまだ無い。このカメラで始める
            frame = self.new_frame(cam, t, img_rgb, c.identity_pose(c.device))
            X, C = c.mast3r_inference_mono(c.model, frame)
            frame.update_pointmap(X, C)
            kf.append(frame)
            st.queue_global_optimization(len(kf) - 1)
            st.set_frame(frame, cam.idx)
            st.set_camera_mode(Mode.TRACKING, cam.idx)
            cam.last_mode = Mode.TRACKING
            cam.T_WC = frame.T_WC
            print(f"[SLAM] {cam.name} で地図を始めた", flush=True)
            return

        if mode in (Mode.INIT, Mode.RELOC):
            # 合流前・見失った: 間隔を空けて、地図の中の見たことのある所を探す（後ろのプロセスが試す）。
            # 間隔は、生は壁の時計、録画を順に回すときは録画の中の時刻で数える（壁の時計で数えると、見失っている間は
            # 処理するこまが無いので録画を 1 秒に数十こまで読み飛ばし、11-18 に残り 20 秒を 2〜3 回しか試さずに終えた）
            now = t if cam.offline else time.time()
            if self.reloc_pending[cam.idx] or now - cam.reloc_at < RELOC_PERIOD_S:
                return
            T0 = cam.T_WC if cam.T_WC is not None else c.identity_pose(c.device)
            frame = self.new_frame(cam, t, img_rgb, T0)
            X, C = c.mast3r_inference_mono(c.model, frame)
            frame.update_pointmap(X, C)
            st.set_frame(frame, cam.idx)
            cam.reloc_at = now
            self.reloc_pending[cam.idx] = 1
            return

        # 追跡（前のこまの姿勢から始める）
        frame = self.new_frame(cam, t, img_rgb, cam.T_WC if cam.T_WC is not None else self.kf_pose(cam))
        add_kf, _, try_reloc = cam.tracker.track(frame)
        if try_reloc:
            st.set_camera_mode(Mode.RELOC, cam.idx)
            cam.last_mode = Mode.RELOC
            print(f"[SLAM] {cam.name} が見失った（再測位を待つ）", flush=True)
        st.set_frame(frame, cam.idx)
        cam.T_WC = frame.T_WC
        if add_kf:
            if len(kf) >= kf.buffer - 1:
                raise MemoryError(f"キーフレームが上限（{kf.buffer}）。--kf-buffer を増やす")
            kf.append(frame)
            st.queue_global_optimization(len(kf) - 1)

    # ---- 状態の知らせ・作り直し

    def update_status(self, cam: Cam) -> None:
        Mode = self.ctx.Mode
        mode = self.states.get_camera_mode(cam.idx)
        now = time.time()
        if mode == Mode.RELOC:
            cam.lost_since = cam.lost_since or now
        else:
            cam.lost_since = None
        s = cam.feed.stats
        self.ctx.status[f"cam_{cam.name}"] = {
            "slam_fps": cam.fps(),
            "received": cam.received,
            "used": cam.used,
            "lost_s": now - cam.lost_since if cam.lost_since else 0.0,
            "problem": cam.problem,
            "recv_fps": getattr(s, "receive_fps", 0.0) if s is not None else 0.0,
            "recv_shape": tuple(getattr(s, "last_shape", (0, 0))) if s is not None else (0, 0),
        }

    def maybe_reset(self) -> None:
        Mode = self.ctx.Mode
        reset_file = pathlib.Path(self.args.reset_file) if self.args.reset_file else None
        if reset_file is not None and reset_file.exists():
            reset_file.unlink(missing_ok=True)
            self.reset_map("requested")
            return
        n = len(self.keyframes)
        if n == 0:
            return
        modes = [self.states.get_camera_mode(cam.idx) for cam in self.cams]
        if any(m == Mode.TRACKING for m in modes):
            self.lost_all_since = None
            return
        now = time.time()
        self.lost_all_since = self.lost_all_since or now
        lost = now - self.lost_all_since
        if n < self.args.small_map_kf and lost >= self.args.reset_small_s:
            self.reset_map(f"all phones lost {lost:.0f}s with {n} keyframes")
        elif self.args.reset_large_s > 0 and lost >= self.args.reset_large_s:
            self.reset_map(f"all phones lost {lost:.0f}s")

    # ---- 主ループ

    def run(self, stop, viz) -> None:
        last_log = time.time()
        while not stop.stop:
            got = False
            for cam in self.cams:
                if cam.ended:
                    continue
                if cam.offline and self.ref_hw is not None and self.reloc_pending[cam.idx]:
                    continue  # 録画を順に回すときは、再測位の結果が出るまでこまを読み進めない（生と同じく試す回数を保つ）
                try:
                    item = cam.feed.poll()
                except StopIteration:
                    cam.ended = True
                    print(f"[SLAM] {cam.name} の録画が終わった", flush=True)
                    continue
                if item is None:
                    continue
                got = True
                cam.received += 1
                t, img = item
                if self.ref_hw is None:
                    self.t_first = time.time()
                    self.ref_hw = img.shape[:2]
                    h, w = self.ctx.img_hw_for(img, self.args.img_size)
                    print(f"[SLAM] 地図の箱 {w}x{h}（最初のこま {img.shape[1]}x{img.shape[0]}・{cam.name}）", flush=True)
                    self.setup_map(h, w)
                    if viz is not None:
                        viz.attach(self.states, self.keyframes)
                    self.ctx.status["waiting"] = False
                img = conform_to(img, self.ref_hw)
                before = len(self.timestamps)
                self.step_camera(cam, t, img)
                if len(self.timestamps) > before:
                    cam.used += 1
                    cam.stamps.append(time.time())
                self.update_status(cam)
            if all(cam.ended for cam in self.cams):
                break
            if self.ref_hw is not None:
                self.maybe_reset()
            if viz is not None:
                viz.check()
            if time.time() - last_log >= WAIT_LOG_EVERY_S:
                last_log = time.time()
                self.log_progress()
            if not got:
                time.sleep(IDLE_SLEEP_S)

    def log_progress(self) -> None:
        if self.ref_hw is None:
            print("[SLAM] 映像を待っている: " + "・".join(f"{c.name} {c.received} こま" for c in self.cams), flush=True)
            return
        ids = self.keyframes.cam_ids()
        parts = []
        for cam in self.cams:
            mode = self.states.get_camera_mode(cam.idx)
            parts.append(f"{cam.name} {mode.name} {cam.fps():.1f} fps kf {int((ids == cam.idx).sum())}")
        print(f"[SLAM] {' ・ '.join(parts)} ・ 合計 kf {len(ids)}", flush=True)

    def summary(self) -> str:
        """台ごとに、使ったこま・受けたこま・平均の fps（最初のこまから終わりまで）"""
        if self.t_first is None:
            return "こまが来なかった"
        secs = max(time.time() - self.t_first, 1e-6)
        return "・".join(f"{c.name} 使った {c.used}/受けた {c.received}（{c.used / secs:.1f} fps）" for c in self.cams) + f"・{secs:.0f} 秒"

    def save(self, save_dir: pathlib.Path) -> None:
        if self.ref_hw is None or len(self.keyframes) == 0:
            return
        c = self.ctx
        names = [cam.name for cam in self.cams]
        paths = c.save_camera_trajs(save_dir, self.timestamps, self.keyframes, names)
        n = c.save_map_with_conf(save_dir, self.keyframes, self.args.conf_thresh)
        print(f"[SLAM] 保存した: {save_dir}（{'・'.join(p.name for p in paths)}・map.ply {n:,} 点（確からしさ入り）・"
              f"キーフレーム {len(self.keyframes)}）", flush=True)


class Ctx:
    """mast3r_slam と torch を、作業の置き場所を決めてから読み込んで入れておく"""

    def __init__(self, args) -> None:
        import lietorch
        import torch
        import torch.multiprocessing as mp

        import mast3r_slam.evaluate as evaluate
        from mast3r_slam.config import config, load_config
        from mast3r_slam.frame import Mode, SharedStates, create_frame
        from mast3r_slam.mast3r_utils import load_mast3r, mast3r_inference_mono, resize_img
        from mast3r_slam.tracker import FrameTracker

        from multicam_core import CamKeyframes, identity_pose, run_backend_multicam, save_camera_trajs, save_map_with_conf

        mp.set_start_method("spawn")
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.set_grad_enabled(False)
        load_config(args.config)
        if config["use_calib"]:
            raise SystemExit("[SLAM] 較正ありの設定は使わない（use_calib: False の設定を渡す）")
        self.lietorch, self.torch, self.mp = lietorch, torch, mp
        self.evaluate, self.config, self.Mode = evaluate, config, Mode
        self.SharedStates, self.create_frame, self.FrameTracker = SharedStates, create_frame, FrameTracker
        self.load_mast3r, self.mast3r_inference_mono, self.resize_img = load_mast3r, mast3r_inference_mono, resize_img
        self.CamKeyframes, self.identity_pose = CamKeyframes, identity_pose
        self.run_backend_multicam, self.save_camera_trajs = run_backend_multicam, save_camera_trajs
        self.save_map_with_conf = save_map_with_conf
        self.device = "cuda:0"
        self.manager = mp.Manager()
        self.model = None
        self.status = None

    def img_hw_for(self, img_rgb: np.ndarray, size: int = 512) -> tuple[int, int]:
        """MASt3R に渡す大きさ（上流の get_img_shape と同じく resize_img で決まる）"""
        import mast3r_slam.mast3r_utils as mu

        out = mu.resize_img(img_rgb.astype(np.float32) / 255.0, size)
        return tuple(out["img"][0].shape[1:])


def main() -> None:
    args = parse_args()
    sources = [s for s in args.source.split(",") if s]
    if len(sources) < 2:
        raise SystemExit("[SLAM] --source は 2 つ以上（1 台なら run_phone_slam.py）")
    names = [n for n in args.names.split(",") if n] or [source_name(s) for s in sources]
    if len(names) != len(sources) or len(set(names)) != len(names):
        raise SystemExit(f"[SLAM] カメラの名前が源と合わない・重なっている: {names}")
    repo = pathlib.Path(args.repo).resolve()
    os.chdir(repo)  # 重み（checkpoints/）を相対で読む
    sys.path.insert(0, str(repo))
    sys.path.insert(0, str(HERE))

    from rerun_viz import VizOptions, viewer_urls
    from multicam_viz import run_multicam_viz
    from run_phone_slam import StopFlag, VizSupervisor

    ctx = Ctx(args)
    if args.reloc_loose:
        ctx.config["reloc"]["strict"] = False
    if args.reloc_min_match > 0:
        ctx.config["reloc"]["min_match_frac"] = args.reloc_min_match
    ctx.config["reloc"]["min_kept"] = args.reloc_min_kept
    ctx.config["reloc"]["buddy"] = args.reloc_buddy
    print(f"[SLAM] 再測位: {f'緩い（{args.reloc_min_kept} 枚以上合えば）' if not ctx.config['reloc']['strict'] else '厳しい（全部合えば）'}・"
          f"照合の割合 {ctx.config['reloc']['min_match_frac']} 以上・もう一方の最新 {args.reloc_buddy} 枚も相手に", flush=True)
    stop = StopFlag()
    ctx.status = ctx.manager.dict(source=args.source, kf_buffer=args.kf_buffer, stop=False, waiting=True)
    viz = None
    if not args.no_viz:
        hosts = tuple(x for x in args.hosts.split(",") if x) or ("127.0.0.1",)
        import viewer_hub
        from multicam_viz import APP_ID

        opts = VizOptions(name="multicam", cams=tuple(names), hosts=hosts, pts_stride=args.pts_stride, conf_thresh=args.conf_thresh,
                          recording_id=viewer_hub.run_recording_id(APP_ID))
        print(f"[SLAM] 表示の記録: {opts.recording_id}", flush=True)
        viz = VizSupervisor(ctx.mp, run_multicam_viz, ctx.config, ctx.status, opts)
        viz.start()
        for url in viewer_urls(opts):
            print(f"[SLAM] 表示: {url}", flush=True)

    if args.fp16:
        import fast_mast3r

        fast_mast3r.enable()
        print("[SLAM] MASt3R の推論は fp16（fast_mast3r）", flush=True)
    if args.img_size != 512:
        import fast_mast3r

        fast_mast3r.allow_any_image_size()
        print(f"[SLAM] MASt3R に渡す画の長い辺は {args.img_size}", flush=True)
    ctx.model = ctx.load_mast3r(device=ctx.device)
    ctx.model.share_memory()
    cams = [Cam(i, name, make_feed(src, args)) for i, (name, src) in enumerate(zip(names, sources))]
    for cam in cams:
        cam.offline = isinstance(cam.feed, DatasetFeed) or (isinstance(cam.feed, VideoFeed) and not args.realtime)
    print("[SLAM] カメラ: " + "・".join(f"{c.name}={s}" for c, s in zip(cams, sources)), flush=True)
    slam = MultiCamSlam(args, cams, ctx)
    try:
        slam.run(stop, viz)
    finally:
        print(f"[SLAM] 速さのまとめ: {slam.summary()}", flush=True)
        if slam.ref_hw is not None:
            slam.stop_backend()
            if args.save_dir:
                slam.save(pathlib.Path(args.save_dir).expanduser().resolve())
        ctx.status["stop"] = True
        if viz is not None:
            viz.stop()
        for cam in cams:
            cam.feed.close()
        print(f"[SLAM] 終わり（こま {len(slam.timestamps)}・地図の作り直し {slam.resets} 回）", flush=True)


if __name__ == "__main__":
    main()
