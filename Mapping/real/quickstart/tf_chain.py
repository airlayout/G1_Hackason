#!/usr/bin/env python3
"""**`/tf` の鎖を辿って `map -> base_link` を組み立てる。候補ごとに鎖の形が違うため。**

## なぜ要るのか（2026-09-15 に 2 度踏んだ）

測位候補は `map` から `base_link` までを出す点だけが共通で、**途中の形は全部違う**。

    MOLA        map -> base_link                                  （直接）
    AMCL        map -> odom -> base_link                          （odom は別ノードが出す）
    FAST_LIO    map -> odom -> camera_init -> body -> base_link    （静的 2 本を含む）

直接の対だけを見る実装は、AMCL と FAST_LIO に対して**空を返す**。
呼び出し側はそれを「この候補は測位を出していない」と誤報する——**落ちないので気づけない**。
だから **frame のグラフを組んで BFS で辿る**。ここが唯一の入口である。

⚠️ **`/tf_static` も辺として入れること。** FAST_LIO の `odom->camera_init` と `body->base_link` は静的で、
これを落とすと鎖が切れる。

⚠️ **辺は逆向きにも使える。** BFS は無向で探し、逆向きに使う辺は姿勢を反転して掛ける。
"""
from __future__ import annotations

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation


def read_edges(con, tid: dict[str, int]) -> dict[tuple[str, str], np.ndarray]:
    """`/tf` と `/tf_static` の全部の辺を (parent, child) -> (N, 8) で返す。

    列は [t, x, y, z, qx, qy, qz, qw]。
    """
    from measure_overlay import Cdr  # 遅延 import（この関数だけの依存）

    seen: dict[tuple[str, str], list] = {}
    for topic in ("/tf", "/tf_static"):
        if topic not in tid:
            continue
        for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid[topic],)):
            r = Cdr(blob)
            for _ in range(r.u32()):
                sec, nsec = r.i32(), r.u32()
                parent, child = r.st(), r.st()
                tr = [r.f64() for _ in range(3)]
                q = [r.f64() for _ in range(4)]
                seen.setdefault((parent, child), []).append([sec + nsec * 1e-9, *tr, *q])
    return {k: np.array(sorted(v)) for k, v in seen.items() if v}


def find_path(edges, src: str, dst: str) -> list[tuple[tuple[str, str], bool]] | None:
    """src -> dst の辺の並びを返す。bool は「その辺を逆向きに使うか」。"""
    adj: dict[str, list] = {}
    for (p, c) in edges:
        adj.setdefault(p, []).append((c, (p, c), False))
        adj.setdefault(c, []).append((p, (p, c), True))
    prev: dict[str, tuple] = {src: ()}
    q = deque([src])
    while q:
        f = q.popleft()
        if f == dst:
            break
        for nxt, key, inv in adj.get(f, []):
            if nxt not in prev:
                prev[nxt] = (f, key, inv)
                q.append(nxt)
    if dst not in prev:
        return None
    out, f = [], dst
    while prev[f]:
        f, key, inv = prev[f]
        out.append((key, inv))
    return list(reversed(out))


def compose(edges, path, times) -> np.ndarray:
    """path に沿って合成し、各時刻の姿勢を (N, 8) で返す。

    各辺はその時刻に**最も近い標本**を使う（静的な辺は標本 1 個なので常にそれ）。
    """
    out = []
    for t in times:
        R, tr = Rotation.identity(), np.zeros(3)
        for key, inv in path:
            a = edges[key]
            k = int(np.argmin(np.abs(a[:, 0] - t))) if len(a) > 1 else 0
            Re, te = Rotation.from_quat(a[k, 4:8]), a[k, 1:4]
            if inv:
                Re, te = Re.inv(), -Re.inv().apply(te)
            tr = R.apply(te) + tr
            R = R * Re
        out.append([t, *tr, *R.as_quat()])
    return np.array(out) if out else np.empty((0, 8))


def resolve(con, tid: dict[str, int], parent: str = "map",
            child: str = "base_link") -> tuple[np.ndarray, str]:
    """`parent -> child` を鎖で組み立てて (N, 8) と、辿った経路の説明を返す。

    直接の辺があればそれを返す（合成のコストを掛けない）。
    """
    edges = read_edges(con, tid)
    direct = edges.get((parent, child))
    if direct is not None and len(direct) >= 2:
        return direct, f"{parent} -> {child}"
    path = find_path(edges, parent, child)
    if path is None:
        return np.empty((0, 8)), f"{parent} から {child} へ辿れない"
    dyn = max((edges[k] for k, _ in path), key=len)
    if len(dyn) < 2:
        return np.empty((0, 8)), "動的な辺が無い"
    desc = " -> ".join([parent] + [(k[0] if inv else k[1]) for k, inv in path])
    return compose(edges, path, dyn[:, 0]), desc
