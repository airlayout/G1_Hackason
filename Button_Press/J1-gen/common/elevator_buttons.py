"""エレベーターの呼びボタン（一般用の ▲▼）を、頭カメラの RGB + 深度から見つける。

    finder = ButtonFinder.from_config(robot_cfg, elevator_cfg)
    call = finder.find(rgb, depth_m, K, q_waist)       # 見つからなければ ButtonNotFound
    call.button("up").center, call.normal              # pelvis 座標

対象は、扉の右の柱に縦に並ぶ呼びボタン（上から 一般用 ▲、一般用 ▼、車いすの印、車いす用 ▲、車いす用 ▼）。
押してよいのは一般用の 2 つだけ（車いす用を押すと「違うボタン」になる）。

流れ:
1. 候補を画像で探す（2 通り。設定の detector で選ぶ）
   - classic: まわりより明るい、色の無い小さな丸（灰色のボタン）。中の黒い記号（▲▼）の穴は埋める
   - yolo: 自分で学習した YOLO（クラス up / down。sim/train_button_yolo.py）。重みは _local/ に置く
2. 丸の中の黒い記号から、▲ か ▼ かを読む（三角形の重心は、底辺の側に寄る。▲ なら下に寄る）
3. 深度で 3 次元の位置（pelvis 座標）にし、まわりの柱の面に平面を当てはめて、押す向き（面の法線）を求める
4. 柱ごとに上から並べ、「いちばん上の 2 つ」の間隔と記号が一般用の ▲▼ に合うときだけ採用する
   - ▲ が画面の外に切れていると、▼ と車いす用の ▲ が上の 2 つになるが、間隔が合わないので採用しない
   - 記号が読めたのに順番と合わない（上が ▼ など）ときも採用しない

公開のエレベーターボタンの YOLO（Kshaw17-web/End-to-end-elevator-button-detection）は、リポジトリに
ライセンスの記載が無く使用の許可が無いため使わない（2026-10-07 確認。学習データの Roboflow の
データセットは CC BY 4.0）。

画像処理は numpy と OpenCV だけを使う。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from .camera_geometry import HeadCameraTransform

ARROWS = ("up", "down")


class ButtonNotFound(RuntimeError):
    """一般用の ▲▼ を見つけられない（見つけたが条件に合わない）。"""


@dataclass
class Candidate2D:
    """画像の中のボタンの候補。"""

    bbox: tuple[int, int, int, int]  # (x1, y1, x2, y2)。x2, y2 は含まない
    mask: np.ndarray  # (H, W) bool。ボタンの面（記号の穴を埋めたもの）
    arrow: str | None  # "up" / "down" / None（読めない）
    score: float = 1.0
    source: str = "classic"


@dataclass
class Button3D:
    """3 次元の位置（pelvis 座標）が分かったボタン。"""

    center: np.ndarray  # ボタンの面の中心 [m]
    diameter_m: float
    arrow: str | None
    bbox: tuple[int, int, int, int]
    n_points: int
    name: str = ""  # 選んだあとに "up" / "down"


@dataclass
class CallButtons:
    """一般用の ▲▼ と、柱の面の向き。"""

    up: Button3D
    down: Button3D
    normal: np.ndarray  # 押す向き（柱の面に向かう単位ベクトル、pelvis 座標）
    others: list[Button3D] = field(default_factory=list)  # 採用しなかったボタン（記録用）

    def button(self, name: str) -> Button3D:
        if name not in ARROWS:
            raise ValueError(f"ボタンは up / down: {name}")
        return self.up if name == "up" else self.down

    def summary(self) -> str:
        def f(b: Button3D) -> str:
            return f"{b.name}={np.round(b.center, 3).tolist()}（直径 {b.diameter_m * 100:.1f} cm、記号 {b.arrow}）"

        return f"{f(self.up)}、{f(self.down)}、押す向き {np.round(self.normal, 3).tolist()}"


class CandidateDetector(Protocol):
    def detect(self, rgb: np.ndarray) -> list[Candidate2D]: ...


# ---- 画像の処理 -----------------------------------------------------------------


def _fill_holes(mask: np.ndarray) -> np.ndarray:
    """mask の中の穴（外側とつながっていない False の領域）を埋める。"""
    import cv2

    h, w = mask.shape
    pad = np.zeros((h + 2, w + 2), np.uint8)
    pad[1:-1, 1:-1] = mask.astype(np.uint8)
    flood = pad.copy()
    cv2.floodFill(flood, np.zeros((h + 4, w + 4), np.uint8), (0, 0), 1)
    return mask | (flood[1:-1, 1:-1] == 0)


def read_arrow(gray: np.ndarray, bbox: tuple[int, int, int, int], inner: float = 0.55, min_pixels: int = 8,
               min_offset: float = 0.05) -> str | None:
    """ボタン（枠 bbox）の中の黒い記号から、▲ か ▼ かを読む。読めなければ None。

    三角形の重心は、底辺から高さの 1/3 の所にある。記号を囲む箱の中心より重心が下（画像の v が大きい）なら、
    底辺が下の ▲。斜め上から見ていても、上下の順は変わらない。
    見るのは枠に内接する楕円の内側（半径 inner 倍）だけ。ボタンの縁は照明で影になり、そこまで入れると
    影を記号と取り違えた（2026-10-07、MuJoCo の評価環境の 22 個で、縁まで入れると 5 個を読み違え、7 個を読めなかった。
    内側だけにすると 22 個すべて読めた）。
    """
    x1, y1, x2, y2 = bbox
    w, h = x2 - x1, y2 - y1
    if w < 4 or h < 4:
        return None
    v, u = np.mgrid[y1:y2, x1:x2]
    cx, cy = (x1 + x2 - 1) / 2, (y1 + y2 - 1) / 2
    face = ((u - cx) / (w / 2 * inner)) ** 2 + ((v - cy) / (h / 2 * inner)) ** 2 <= 1
    g = gray[y1:y2, x1:x2]
    vals = g[face]
    if vals.size < 4 * min_pixels:
        return None
    dark = face & (g < 0.6 * float(np.median(vals)))
    if int(dark.sum()) < min_pixels:
        return None
    v = v[dark]
    v0, v1 = float(v.min()), float(v.max())
    if v1 - v0 < 2:
        return None
    offset = (float(v.mean()) - 0.5 * (v0 + v1)) / (v1 - v0)
    if offset > min_offset:
        return "up"
    if offset < -min_offset:
        return "down"
    return None


class ClassicButtonDetector:
    """まわりより明るい、色の無い小さな丸を探す（MuJoCo の評価環境のボタンに合わせた方法）。

    まわりの明るさは、局所の平均（background_px 四方）で見る。柱は照明で上が明るく写るので、
    画像全体の明るさを基準にすると、上のボタンが柱と区別できなかった（Yada の見本のエージェントと同じ考え方）。
    """

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.background_px = int(cfg.get("background_px", 121))
        self.min_contrast = float(cfg.get("min_contrast", 18.0))
        self.max_chroma = float(cfg.get("max_chroma", 40.0))
        self.min_area_px = int(cfg.get("min_area_px", 40))
        self.max_area_px = int(cfg.get("max_area_px", 20000))
        self.min_fill = float(cfg.get("min_fill", 0.5))
        self.max_aspect = float(cfg.get("max_aspect", 2.5))

    def detect(self, rgb: np.ndarray) -> list[Candidate2D]:
        import cv2

        img = np.asarray(rgb, dtype=np.float32)
        gray = img.mean(axis=2)
        chroma = img.max(axis=2) - img.min(axis=2)
        k = self.background_px
        background = cv2.blur(gray, (k, k), borderType=cv2.BORDER_REFLECT)
        spots = (gray - background > self.min_contrast) & (chroma < self.max_chroma)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(spots.astype(np.uint8), connectivity=8)
        out: list[Candidate2D] = []
        for i in range(1, n):
            x, y, w, h, _ = (int(v) for v in stats[i])
            if not (1.0 / self.max_aspect <= w / max(h, 1) <= self.max_aspect):
                continue
            sub = _fill_holes(labels[y:y + h, x:x + w] == i)
            area = int(sub.sum())
            if not (self.min_area_px <= area <= self.max_area_px) or area < self.min_fill * w * h:
                continue
            face = np.zeros(spots.shape, bool)
            face[y:y + h, x:x + w] = sub
            out.append(Candidate2D(bbox=(x, y, x + w, y + h), mask=face, arrow=read_arrow(gray, (x, y, x + w, y + h)),
                                   score=float(area) / (w * h), source="classic"))
        return out


class YoloButtonDetector:
    """自分で学習した YOLO（クラス up / down）。ボタンの面は、枠に内接する楕円とする。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        from .config import resolve_repo_path
        from .perception_bridge import perception

        weights = resolve_repo_path(cfg["weights"])
        if not weights.is_file():
            raise FileNotFoundError(f"ボタンの YOLO の重みが無い: {weights}（sim/train_button_yolo.py で作る）")
        yolo = perception("detector.yolo_detector")
        self._det = yolo.YoloDetector(model_name=str(weights), classes=None,
                                      confidence_threshold=float(cfg.get("confidence", 0.4)),
                                      device=str(cfg.get("device", "auto")))

    def detect(self, rgb: np.ndarray) -> list[Candidate2D]:
        h, w = rgb.shape[:2]
        v, u = np.mgrid[0:h, 0:w]
        # Perception の YoloDetector は BGR（OpenCV の並び）を受け取る
        out: list[Candidate2D] = []
        for d in self._det.detect(np.ascontiguousarray(rgb[:, :, ::-1])):
            x1, y1, x2, y2 = d.bbox
            cx, cy, rx, ry = (x1 + x2) / 2, (y1 + y2) / 2, max((x2 - x1) / 2, 1), max((y2 - y1) / 2, 1)
            face = ((u - cx) / rx) ** 2 + ((v - cy) / ry) ** 2 <= 1.0
            box = (int(np.floor(x1)), int(np.floor(y1)), int(np.ceil(x2)), int(np.ceil(y2)))
            out.append(Candidate2D(bbox=box, mask=face, arrow=d.class_name if d.class_name in ARROWS else None,
                                   score=float(d.confidence), source="yolo"))
        return out


def make_candidate_detector(cfg: dict[str, Any]) -> CandidateDetector:
    kind = cfg.get("detector", "classic")
    if kind == "classic":
        return ClassicButtonDetector(cfg.get("classic", {}))
    if kind == "yolo":
        return YoloButtonDetector(cfg["yolo"])
    raise ValueError(f"detector は classic / yolo: {kind}")


# ---- 3 次元 ---------------------------------------------------------------------


def pelvis_points(depth_m: np.ndarray, K: np.ndarray, rot: np.ndarray, pos: np.ndarray) -> np.ndarray:
    """深度の各画素の 3 次元の位置 (H, W, 3)（pelvis 座標）。"""
    z = np.asarray(depth_m, dtype=np.float64)
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    p = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], axis=-1)
    return p @ rot.T + pos


def fit_plane(points: np.ndarray, iters: int = 2, keep_m: float = 0.006) -> tuple[np.ndarray, np.ndarray, float]:
    """点に平面を当てはめる（外れた点を除いて当てはめ直す）。(法線, 平面上の点, 当てはまった割合)。"""
    pts = np.asarray(points, dtype=np.float64)
    keep = np.ones(len(pts), bool)
    n = np.array([1.0, 0.0, 0.0])
    c = pts.mean(axis=0)
    for _ in range(iters + 1):
        sel = pts[keep]
        c = sel.mean(axis=0)
        _, _, vt = np.linalg.svd(sel - c, full_matrices=False)
        n = vt[-1]
        dist = np.abs((pts - c) @ n)
        keep = dist < max(keep_m, 2.5 * float(np.median(dist)))
    return n, c, float(keep.mean())


class ButtonFinder:
    def __init__(self, cam_tf: HeadCameraTransform, detector: CandidateDetector, cfg: dict[str, Any]) -> None:
        self.cam_tf = cam_tf
        self.detector = detector
        self.cfg = cfg
        self.last_candidates: list[Candidate2D] = []
        self.last_buttons: list[Button3D] = []

    @classmethod
    def from_config(cls, robot_cfg: dict[str, Any], cfg: dict[str, Any],
                    detector: CandidateDetector | None = None) -> "ButtonFinder":
        return cls(HeadCameraTransform(robot_cfg), detector or make_candidate_detector(cfg), cfg)

    def find(self, rgb: np.ndarray, depth_m: np.ndarray, K: np.ndarray, q_waist: np.ndarray) -> CallButtons:
        rot, pos = self.cam_tf.pelvis_from_optical(np.asarray(q_waist, dtype=float))
        depth = np.asarray(depth_m, dtype=np.float64)
        P = pelvis_points(depth, np.asarray(K, dtype=float), rot, pos)
        g = self.cfg["geometry"]
        valid = (depth > float(g["min_depth_m"])) & (depth < float(g["max_depth_m"]))
        cands = self.detector.detect(rgb)
        self.last_candidates = cands
        buttons = []
        for c in cands:
            b = self._locate(c, P, valid)
            if b is not None:
                buttons.append(b)
        self.last_buttons = buttons
        up, down = select_call_pair(buttons, g)
        normal = self._column_normal(P, valid, cands, up, down)
        others = [b for b in buttons if b is not up and b is not down]
        return CallButtons(up=up, down=down, normal=normal, others=others)

    def _locate(self, c: Candidate2D, P: np.ndarray, valid: np.ndarray) -> Button3D | None:
        g = self.cfg["geometry"]
        m = c.mask & valid
        if int(m.sum()) < int(g["min_points"]):
            return None
        pts = P[m]
        # 面の中心: 前後（x）は中央値（縁の外れた深度に強い）、左右・上下は平均
        center = np.array([np.median(pts[:, 0]), pts[:, 1].mean(), pts[:, 2].mean()])
        # 大きさ: 左右の幅（画像の面の範囲は検出で決まっているので、端から端まで）
        diameter = float(np.ptp(pts[:, 1]))
        if not (float(g["min_diameter_m"]) <= diameter <= float(g["max_diameter_m"])):
            return None
        return Button3D(center=center, diameter_m=diameter, arrow=c.arrow, bbox=c.bbox, n_points=int(m.sum()))

    def _column_normal(self, P: np.ndarray, valid: np.ndarray, cands: list[Candidate2D],
                       up: Button3D, down: Button3D) -> np.ndarray:
        """2 つのボタンのまわりの柱の面に平面を当てはめ、面に向かう向きを返す。"""
        g = self.cfg["geometry"]
        mid = 0.5 * (up.center + down.center)
        r = float(g["normal_region_m"])
        near = valid & (np.abs(P[..., 1] - mid[1]) < r) & (np.abs(P[..., 2] - mid[2]) < r + 0.5 * abs(
            up.center[2] - down.center[2]))
        # ボタンの面（少し出っ張っている）と、柱より手前の物（手など）を除く
        for c in cands:
            near &= ~c.mask
        near &= np.abs(P[..., 0] - mid[0]) < float(g["normal_depth_band_m"])
        pts = P[near]
        if len(pts) < int(g["normal_min_points"]):
            raise ButtonNotFound(f"柱の面の点が足りない（{len(pts)} 点）")
        n, _, inlier = fit_plane(pts)
        if n[0] < 0:
            n = -n
        if inlier < float(g["normal_min_inlier"]) or n[0] < np.cos(np.radians(float(g["normal_max_tilt_deg"]))):
            raise ButtonNotFound(f"柱の面の向きが求まらない（法線 {np.round(n, 3).tolist()}、当てはまり {inlier:.0%}）")
        return n


def select_call_pair(buttons: list[Button3D], g: dict[str, Any]) -> tuple[Button3D, Button3D]:
    """柱ごとに上から並べ、いちばん上の 2 つが一般用の ▲▼ の並びに合えば返す。"""
    if len(buttons) < 2:
        raise ButtonNotFound(f"ボタンが {len(buttons)} 個しか見つからない")
    col_tol = float(g["column_tolerance_m"])
    dz_lo, dz_hi = (float(v) for v in g["pair_spacing_m"])
    columns: list[list[Button3D]] = []
    for b in sorted(buttons, key=lambda b: -b.center[2]):
        for col in columns:
            if abs(col[0].center[1] - b.center[1]) < col_tol:
                col.append(b)
                break
        else:
            columns.append([b])
    reasons = []
    pairs = []
    for col in columns:
        if len(col) < 2:
            continue
        top, second = col[0], col[1]
        dz = float(top.center[2] - second.center[2])
        if not (dz_lo <= dz <= dz_hi):
            reasons.append(f"上の 2 つの間隔 {dz * 100:.1f} cm が {dz_lo * 100:.0f}〜{dz_hi * 100:.0f} cm の外")
            continue
        if (top.arrow is not None and top.arrow != "up") or (second.arrow is not None and second.arrow != "down"):
            reasons.append(f"記号の順番が合わない（上 {top.arrow}、下 {second.arrow}）")
            continue
        pairs.append((top, second))
    if len(pairs) != 1:
        why = "、".join(reasons) if reasons else "柱にボタンが 2 つ並んでいない"
        raise ButtonNotFound(f"一般用の ▲▼ が 1 組に決まらない（{len(pairs)} 組）: {why}")
    up, down = pairs[0]
    up.name, down.name = "up", "down"
    return up, down


# ---- 複数のフレーム ---------------------------------------------------------------


class ButtonTracker:
    """新しいフレームで続けて測り、ばらつきが小さければ中央値を返す。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.frames = int(cfg["frames"])
        self.max_spread_m = float(cfg["max_spread_m"])
        self.max_normal_spread_deg = float(cfg["max_normal_spread_deg"])
        self.samples: list[CallButtons] = []

    def reset(self) -> None:
        self.samples.clear()

    def add(self, call: CallButtons) -> CallButtons | None:
        """1 フレーム分を足す。そろってばらつきが小さければ、まとめた結果を返す（そうでなければ None）。"""
        self.samples.append(call)
        if len(self.samples) < self.frames:
            return None
        s = self.samples[-self.frames:]
        ups = np.array([c.up.center for c in s])
        downs = np.array([c.down.center for c in s])
        normals = np.array([c.normal for c in s])
        up_c, down_c = np.median(ups, axis=0), np.median(downs, axis=0)
        n = np.median(normals, axis=0)
        n /= np.linalg.norm(n)
        spread = max(np.linalg.norm(ups - up_c, axis=1).max(), np.linalg.norm(downs - down_c, axis=1).max())
        ang = float(np.degrees(np.arccos(np.clip(normals @ n, -1, 1))).max())
        if spread > self.max_spread_m or ang > self.max_normal_spread_deg:
            # 古いほうから捨てて、次のフレームを待つ
            self.samples.pop(0)
            return None
        last = s[-1]
        up = Button3D(up_c, float(np.median([c.up.diameter_m for c in s])), last.up.arrow, last.up.bbox,
                      last.up.n_points, "up")
        down = Button3D(down_c, float(np.median([c.down.diameter_m for c in s])), last.down.arrow, last.down.bbox,
                        last.down.n_points, "down")
        return CallButtons(up=up, down=down, normal=n, others=last.others)
