"""2 台以上のスマホで 1 つの地図を作る部品（Multicam-MASt3R-SLAM の派生版の上に、その不具合を直して載せる）。

元にした物:
- MASt3R-SLAM（rmurai0610/MASt3R-SLAM e6f4e3d）: 追跡・照合・大域の最適化・検索（重み・CUDA の部品）
- Multicam-MASt3R-SLAM（johnkimdy, UIST 2025, 71058ea）: キーフレームにカメラの番号を持たせ、
  追跡は「同じカメラの直前のキーフレーム」と比べる（frame.py・tracker.py。差分はこの 2 つだけ）

派生版の main.py は、そのまま使うと次の所で地図が壊れる。ここで直す:
1. 2 台目の始まりは、照合の割合が低くても追跡に進んでいた
   → 2 台目は「地図の中の見たことのある所」を検索で見つけて合流する（再測位と同じ手順・厳しい判定）
2. 2 台目の追跡の初めの姿勢が、毎こま単位行列だった → そのカメラの前のこまの姿勢から始める
3. 2 台目が見失っても再測位しない（コメントで外されていた） → どのカメラも見失えば再測位する
4. 後ろの最適化の「直前のキーフレーム」とのつなぎが、番号が 1 つ前＝別のカメラのキーフレームでも
   張られ、しかも照合が悪くても捨てられなかった（上流の add_factors は ii == jj-1 を必ず残す）
   → 必ず残すつなぎは「同じカメラの直前のキーフレーム」だけ。検索で見つけたつなぎ（別のカメラとの
   合流もここ）は照合の割合で選ぶ
5. 再測位で入れたキーフレームのカメラの番号が 0 になっていた（SharedStates.get_frame が番号を写さない）

基準のカメラは決めない。地図が空のとき最初にこまが来たカメラが地図を始め、ほかは合流する。
"""

from __future__ import annotations

import pathlib
import time

import lietorch
import numpy as np
import torch
from mast3r_slam.config import config, set_global_config
from mast3r_slam.frame import Mode, SharedKeyframes
from mast3r_slam.global_opt import FactorGraph
from mast3r_slam.lietorch_utils import as_SE3
from mast3r_slam.mast3r_utils import load_retriever

BACKEND_IDLE_S = 0.01
MAX_BATCH_KF = 8  # 後ろが遅れたとき、まとめて 1 回の最適化で片づけるキーフレームの上限


class CamKeyframes(SharedKeyframes):
    """カメラの番号の引き方を速くした SharedKeyframes。

    派生版の last_keyframe は後ろから 1 枚ずつ self[i] を作って番号を見る。self[i] のたびに
    Manager の錠（プロセス間の往復）を取るので、キーフレームが増えるほど追跡が遅くなる。
    番号の配列（cam_id）をまとめて引く。
    """

    def last_index(self, camid: int, before: int | None = None) -> int | None:
        with self.lock:
            n = self.n_size.value if before is None else min(before, self.n_size.value)
            if n <= 0:
                return None
            hits = torch.nonzero(self.cam_id[:n] == camid)
            return int(hits[-1]) if len(hits) else None

    def last_keyframe(self, camid: int = 0):
        with self.lock:
            i = self.last_index(camid)
            return None if i is None else (self[i], i)

    def cam_ids(self) -> np.ndarray:
        with self.lock:
            return self.cam_id[: self.n_size.value].cpu().numpy().copy()


def reloc_required(min_kept: int, n_map_keyframes: int) -> int:
    """緩い再測位で重なるのに要る「照合が合った相手」の数。

    11-18 の実機で、1 枚だけ合った重なりが床のカーペットの縞と白い L 字の印（階のどこにでもある）に引かれて
    別の場所に重なった（2 台そろって 12 m ほど飛んだ）。2 枚以上を要る。ただし地図にキーフレームが
    min_kept より少ない間は、その枚数でよい（始めに 2 台で同じ所を止まって写すとキーフレームが増えず、
    2 台目がいつまでも合流できない）。
    """
    return max(1, min(min_kept, n_map_keyframes))


def truncate_factors(factor_graph, n: int) -> None:
    """FactorGraph のつなぎを先頭 n 本に戻す（add_factors で足した分を取り消す）"""
    for name in ("ii", "jj", "idx_ii2jj", "idx_jj2ii", "valid_match_j", "valid_match_i", "Q_ii2jj", "Q_jj2ii"):
        setattr(factor_graph, name, getattr(factor_graph, name)[:n])


def relocalize(frame, keyframes, factor_graph, retrieval_database) -> bool:
    """上流の main.relocalization と同じ手順（検索 → 照合 → 判定 → 大域の最適化）。

    判定は設定の reloc.strict 次第。上流の既定（strict）は検索で見つけた k 枚すべてと照合の割合が min_match_frac 以上
    のときだけ重ねる。緩い形（run_multicam_slam.py の既定）では reloc.min_kept 枚以上が合えば重ね、合わない相手との
    つなぎは捨てる（11-18 の実機の録画で、始めの机に戻った所を厳しい形では 3 枚のうち 2 枚しか合わず重なれなかった。
    1 枚でよいとすると床の模様で別の所に重なった）。

    違いは、加えるキーフレームにカメラの番号が付いたまま入ること（呼ぶ側で frame.camera_ID を入れる）。
    2 台目の合流もこれで行う（初めは地図の中の見たことのある所に重ねる）。
    """
    with keyframes.lock:
        kf_idx = retrieval_database.update(
            frame,
            add_after_query=False,
            k=config["retrieval"]["k"],
            min_thresh=config["retrieval"]["min_thresh"],
        )
        # もう一方のスマホの最新のキーフレームも相手に加え、先に並べる（姿勢の始めもそこから取る）
        buddy = recent_other_cameras(keyframes.cam_ids(), int(frame.camera_ID), int(config["reloc"].get("buddy", 0)))
        kf_idx = list(dict.fromkeys(buddy + [int(j) for j in kf_idx]))
        if not kf_idx:
            return False
        keyframes.append(frame)
        n_kf = len(keyframes)
        kf_idx = list(kf_idx)
        n_before = int(factor_graph.ii.numel())
        ok = factor_graph.add_factors(
            [n_kf - 1] * len(kf_idx),
            kf_idx,
            config["reloc"]["min_match_frac"],
            is_reloc=config["reloc"]["strict"],
        )
        if not ok:
            keyframes.pop_last()
            return False
        # 照合が合って残った相手（strict なら検索で見つけた全部。緩めた形では合った物だけ）
        kept = [int(j) for j in factor_graph.jj[n_before:].tolist()] or kf_idx
        need = reloc_required(int(config["reloc"].get("min_kept", 1)), n_kf - 1)
        if not config["reloc"]["strict"] and len(kept) < need:
            # 合った相手が少ない＝似た模様の別の所の恐れ。張ったつなぎを戻して断る
            truncate_factors(factor_graph, n_before)
            keyframes.pop_last()
            print(f"[後ろ] カメラ {frame.camera_ID} の再測位を断った（合ったのは {kept}・検索 {kf_idx}・{need} 枚以上が要る）", flush=True)
            return False
        retrieval_database.update(
            frame,
            add_after_query=True,
            k=config["retrieval"]["k"],
            min_thresh=config["retrieval"]["min_thresh"],
        )
        # 始めの姿勢は「実際に合った相手」から（上流は検索の 1 番目から。緩めた形では 1 番目が合わない相手のことがある）
        keyframes.T_WC[n_kf - 1] = keyframes.T_WC[kept[0]].clone()
        extra = "" if kept == kf_idx else f"（検索 {kf_idx} のうち合った物）"
        print(f"[後ろ] カメラ {frame.camera_ID} を地図に重ねた（キーフレーム {n_kf - 1} ↔ {kept}{extra}）", flush=True)
    factor_graph.solve_GN_rays()
    return True


def recent_other_cameras(cam_ids: np.ndarray, cam: int, n: int) -> list[int]:
    """カメラ cam 以外の各カメラの、最新 n 枚のキーフレームの番号（新しい順）。

    2 台は両手に持って一緒に歩くので、片方が見失っている間も、もう一方の最新のキーフレームは目の前とほぼ同じ景色。
    再測位の相手に加えると戻りやすい（11-18 の録画で、iPhone は検索の相手が 1〜2 枚しか無く 28 秒戻れなかった）。
    時刻も場所も近い相手なので、床の模様で遠くの別の所に重なる恐れは無い。
    """
    if n <= 0 or len(cam_ids) == 0:
        return []
    out: list[int] = []
    for c in np.unique(cam_ids):
        if int(c) == cam:
            continue
        out += [int(i) for i in np.flatnonzero(cam_ids == c)[-n:][::-1]]
    return out


def recent_same_camera(cam_ids: np.ndarray, cam: int, before: int, n: int) -> set[int]:
    """カメラ cam の、before より前の最新 n 枚のキーフレームの番号"""
    if n <= 0:
        return set()
    return {int(i) for i in np.flatnonzero(cam_ids[:before] == cam)[-n:]}


def add_keyframe_factors(factor_graph, retrieval_database, keyframes, idx: int, recent_skip: int = 0) -> list[int]:
    """新しいキーフレーム idx のつなぎを張る。返り値は検索で見つけた（合流・ループの）相手。

    recent_skip > 0 なら、検索で見つけた相手のうち「同じカメラの直前 recent_skip 枚」を外す。それらは前進の鎖で
    もうつながっていて、照合（1 組で復号 2 回）を足しても地図はほとんど変わらない。後ろの照合は新しいキーフレーム
    ごとに 4 組で 323〜424 ms（11-17 の測り）と追跡の 4 こま分より重く、2 台の生の速さを半分にしていた。
    別のカメラのキーフレーム（合流）と、同じカメラの古いキーフレーム（ループ）は外さない。
    """
    frame = keyframes[idx]
    cam = int(frame.camera_ID)
    prev = keyframes.last_index(cam, before=idx)
    if prev is not None:
        # 同じカメラの直前のキーフレームとは必ずつなぐ（上流の「1 つ前」と同じ役。下限 0 で必ず残る）
        factor_graph.add_factors([prev], [idx], 0.0)
    found = retrieval_database.update(
        frame,
        add_after_query=True,
        k=config["retrieval"]["k"],
        min_thresh=config["retrieval"]["min_thresh"],
    )
    skip = recent_same_camera(keyframes.cam_ids(), cam, idx, recent_skip)
    others = sorted({int(j) for j in found} - {idx, prev} - skip)
    if others:
        # [idx] を先に置く: 上流の add_factors は ii == jj-1 の組を照合が悪くても残すので、
        # 新しい方を ii にしてその扱いに入らないようにする（つなぎは両向きに使われるので向きは関係ない）
        factor_graph.add_factors([idx] * len(others), others, config["local_opt"]["min_match_frac"])
    return others


class BackendTiming:
    """後ろの最適化の 1 枚ごとの時間を every 枚ごとにまとめて記録に出す（生の速さを取り合う相手を測る）"""

    def __init__(self, every: int = 10) -> None:
        self.every = every
        self.rows: list[tuple[float, float, int, int]] = []

    def add(self, t_factors: float, t_solve: float, n_found: int, n_edges: int, n_kf: int = 1) -> None:
        before = sum(r[4] for r in self.rows)
        self.rows.append((t_factors, t_solve, n_found, n_edges, n_kf))
        after = before + n_kf
        if after // self.every == before // self.every:
            return
        last = np.array(self.rows[-self.every:], dtype=float)
        kf = last[:, 4].sum()
        print(
            f"[後ろ] キーフレーム {after} 枚目まで: 1 枚あたり つなぎ {last[:, 0].sum() / kf * 1000:.0f} ms・"
            f"最適化 1 回 {last[:, 1].mean() * 1000:.0f} ms（{kf / len(last):.1f} 枚ずつ）・検索の相手 {last[:, 2].sum() / kf:.1f}・"
            f"つなぎの総数 {int(last[-1, 3])}",
            flush=True,
        )


def run_backend_multicam(
    cfg: dict,
    model,
    states,
    keyframes,
    reloc_pending,
    device: str = "cuda:0",
    fp16: bool = False,
    recent_skip: int = 0,
    gn_iters: int = 0,
    final_iters: int = 0,
) -> None:
    """後ろの最適化のプロセス。新しいキーフレームのつなぎと大域の最適化、カメラごとの再測位（合流）を回す。

    reloc_pending[c] が 1 なら、states のカメラ c のこまで再測位を試し、終わったら 0 に戻す
    （主のプロセスは 0 になるまで次のこまを置かない＝同じこまを何度も試して GPU を使い続けない）。
    """
    set_global_config(cfg)
    if fp16:
        import fast_mast3r  # spawn の子はクラスを読み直すので、ここでも差し込む

        fast_mast3r.enable()
    factor_graph = FactorGraph(model, keyframes, None, device)
    retrieval_database = load_retriever(model)
    n_cams = len(reloc_pending)
    timing = BackendTiming()
    # 生の間の全体の最適化の反復（0 は設定のまま＝上流は 10）。前の答えから始めるので 1 枚分の変化は少ない反復で収まる。
    # 反復ごとの手間は「つなぎの数 × 1 枚の画素」で、地図が育つと 1 回 600 ms を超えた（11-17・60 枚）
    live_cfg = dict(config["local_opt"], max_iters=gn_iters) if gn_iters > 0 else config["local_opt"]
    while states.get_mode() != Mode.TERMINATED:
        if states.is_paused():
            time.sleep(BACKEND_IDLE_S)
            continue
        busy = False
        for cam in range(n_cams):
            if not reloc_pending[cam]:
                continue
            busy = True
            frame = states.get_frame(cam)
            frame.camera_ID = cam
            if relocalize(frame, keyframes, factor_graph, retrieval_database):
                states.set_camera_mode(Mode.TRACKING, cam)
            reloc_pending[cam] = 0

        with states.lock:
            tasks = list(states.global_optimizer_tasks[:MAX_BATCH_KF])
        if not tasks:
            if not busy:
                time.sleep(BACKEND_IDLE_S)
            continue
        # 溜まっていた新しいキーフレームのつなぎを全部張ってから、最適化は 1 回だけ（後ろが遅れたときに追いつく）
        t0 = time.perf_counter()
        n_found = 0
        for idx in tasks:
            found = add_keyframe_factors(factor_graph, retrieval_database, keyframes, idx, recent_skip)
            n_found += len(found)
            if found:
                print(f"[後ろ] キーフレーム {idx} を検索でつないだ: {found}", flush=True)
        with states.lock:
            states.edges_ii[:] = factor_graph.ii.cpu().tolist()  # .cpu() で照合の終わりまで待つ
            states.edges_jj[:] = factor_graph.jj.cpu().tolist()
        t1 = time.perf_counter()
        factor_graph.cfg = live_cfg
        factor_graph.solve_GN_rays()
        torch.cuda.synchronize()
        timing.add(t1 - t0, time.perf_counter() - t1, n_found, int(factor_graph.ii.numel()), len(tasks))
        with states.lock:
            del states.global_optimizer_tasks[: len(tasks)]

    # 止めるときに全体をもう一度、反復を多めに最適化してから保存させる（生の間は反復を絞っている）
    if final_iters > 0 and factor_graph.ii.numel() > 0:
        t0 = time.perf_counter()
        factor_graph.cfg = dict(config["local_opt"], max_iters=final_iters)
        factor_graph.solve_GN_rays()
        torch.cuda.synchronize()
        print(f"[後ろ] 終わりの全体の最適化（反復 {final_iters}・つなぎ {int(factor_graph.ii.numel())}）: "
              f"{(time.perf_counter() - t0) * 1000:.0f} ms", flush=True)


def save_camera_trajs(save_dir: pathlib.Path, timestamps: list[float], keyframes, names: list[str]) -> list[pathlib.Path]:
    """カメラごとの軌跡（TUM の形: 時刻 x y z qx qy qz qw）を traj_<名前>.txt に書く。時刻は受けた秒"""
    save_dir.mkdir(parents=True, exist_ok=True)
    rows: dict[int, list[str]] = {c: [] for c in range(len(names))}
    for i in range(len(keyframes)):
        kf = keyframes[i]
        T = as_SE3(kf.T_WC).data.cpu().numpy().reshape(-1)
        t = timestamps[kf.frame_id] if kf.frame_id < len(timestamps) else float(kf.frame_id)
        rows.setdefault(int(kf.camera_ID), []).append(f"{t:.6f} " + " ".join(f"{v:.6f}" for v in T))
    out = []
    for c, lines in rows.items():
        if not lines or c >= len(names):
            continue
        p = save_dir / f"traj_{names[c]}.txt"
        p.write_text("\n".join(lines) + "\n")
        out.append(p)
    return out


def save_map_with_conf(save_dir: pathlib.Path, keyframes, conf_min: float, name: str = "map.ply") -> int:
    """全部のキーフレームの点を世界の座標で map.ply に書く。点ごとの確からしさ（conf）も入れる。

    上流の save_reconstruction と同じ点（T_WC で X_canon を動かし、確からしさは get_average_conf）。違いは conf の欄。
    後から view_result.py --conf-thresh で絞り直せる（遠くの点ほど奥行きが不確かで視線の向きに扇のように広がり、
    確からしさが低い。11-17・11-18 の実機の地図で目立った）。conf_min より低い点は書かない。書いた点の数を返す。
    """
    from plyfile import PlyData, PlyElement

    save_dir.mkdir(parents=True, exist_ok=True)
    parts = []
    for i in range(len(keyframes)):
        kf = keyframes[i]
        pW = kf.T_WC.act(kf.X_canon).cpu().numpy().reshape(-1, 3)
        rgb = (kf.uimg.cpu().numpy() * 255).astype(np.uint8).reshape(-1, 3)
        conf = kf.get_average_conf().cpu().numpy().astype(np.float32).reshape(-1)
        keep = conf > conf_min
        parts.append((pW[keep], rgb[keep], conf[keep]))
    xyz = np.concatenate([q[0] for q in parts]).astype(np.float32)
    rgb = np.concatenate([q[1] for q in parts])
    conf = np.concatenate([q[2] for q in parts])
    v = np.empty(len(xyz), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1"), ("conf", "f4")])
    v["x"], v["y"], v["z"] = xyz.T
    v["red"], v["green"], v["blue"] = rgb.T
    v["conf"] = conf
    PlyData([PlyElement.describe(v, "vertex")]).write(str(save_dir / name))
    return len(v)


def identity_pose(device: str) -> lietorch.Sim3:
    return lietorch.Sim3.Identity(1, device=device)
