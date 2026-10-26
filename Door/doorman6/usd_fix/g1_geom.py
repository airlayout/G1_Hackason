"""G1（29 関節・rev_1_0）の USD を読み、順運動学（FK）と当たりの形どうしの符号つき距離を扱う共通の部品（読むだけ）。

- 座標は列ベクトルの約束（p_world = R @ p + t）。剛体の座標は USD の剛体の prim の座標（＝URDF のリンクの座標）
- 当たりの形は PhysX と同じく凸の形として扱う。Mesh(convexHull) は凸包、Cylinder・Sphere・Cube はそのまま
- 凸包の頂点が 64（PhysX の hullVertexLimit の既定）を超えるメッシュは、PhysX が既定で
  「元の点の OBB を、元の凸包の面で削っていく」外側の近似にする（PxConvexFlag::ePLANE_SHIFTING の説明にある既定の方法）。
  hull="physx" ではそれをまねた近似（physx_like_hull）、hull="true" では真の凸包を使う。正しい値は Isaac Sim で測る。
  ⚠️ 正解の記録（自己衝突ありの PhysX で取った姿勢）を再生すると、physx 風では既存の形どうし（胴↔右上腕）が 13.6 mm・428 コマ
  食い込む＝PhysX が実際に使った形より大きすぎる。真の凸包では 4.4 mm 以下。PhysX の実際の形は真の凸包に近いとみなし、
  physx 風は上限の見積りとして並べて出す
- 距離は python-fcl（GJK/EPA）の符号つき距離 [m]。負＝食い込みの深さ

実行は Mac: uvx --with usd-core --with numpy --with scipy --with python-fcl python <script>
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import fcl
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial import ConvexHull, HalfspaceIntersection

DOF: Tuple[str, ...] = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint",
    "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
# VIRAL6 の rl/config/robot/g1/g1_29dof.yaml の init_state.default_joint_angles（Isaac の既定の姿勢。ほかは 0）
DEFAULT_JOINT_ANGLES: Dict[str, float] = {
    "left_hip_pitch_joint": -0.1, "left_knee_joint": 0.3, "left_ankle_pitch_joint": -0.2,
    "right_hip_pitch_joint": -0.1, "right_knee_joint": 0.3, "right_ankle_pitch_joint": -0.2,
}
HULL_VERTEX_LIMIT = 64  # PhysX の convexHull の頂点の上限の既定（PhysxConvexHullCollisionAPI の hullVertexLimit）


def default_q() -> np.ndarray:
    return np.array([DEFAULT_JOINT_ANGLES.get(n, 0.0) for n in DOF])


# ---------- 形の下ごしらえ ----------

def read_stl(path: str) -> np.ndarray:
    """STL（バイナリ・ASCII）の頂点 [N,3]（三角形ごとに重複あり）"""
    with open(path, "rb") as f:
        data = f.read()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        pts = [list(map(float, ln.split()[1:4])) for ln in data.decode("ascii", "ignore").splitlines()
               if ln.strip().startswith("vertex")]
        return np.array(pts, dtype=float)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(data, dtype=rec, count=n, offset=84)["v"].reshape(-1, 3).astype(float)


def hull_gap(P: np.ndarray, Q: np.ndarray) -> float:
    """P の凸包の頂点が Q の凸包から外へ出る最大の距離 [m]（0＝P は Q の中）"""
    eq = ConvexHull(Q).equations
    Hp = P[ConvexHull(P).vertices]
    return float(max((Hp @ eq[:, :3].T + eq[:, 3]).max(axis=1).max(), 0.0))


def reduce_hull(points: np.ndarray, limit: int = HULL_VERTEX_LIMIT) -> Tuple[np.ndarray, float]:
    """凸包を頂点 limit 個以下に間引く（元の頂点から選ぶので内側の近似）。外へ最も出ている元の頂点を 1 つずつ足していく。

    返り値: (間引いた凸包の頂点, 元の凸包の頂点が間引いた凸包から外へ出る最大の距離 [m])
    """
    H = points[ConvexHull(points).vertices]
    if len(H) <= limit:
        return H, 0.0
    i0 = int(np.argmin(H[:, 0]))
    i1 = int(np.argmax(np.linalg.norm(H - H[i0], axis=1)))
    i2 = int(np.argmax(np.linalg.norm(np.cross(H - H[i0], H[i1] - H[i0]), axis=1)))
    nrm = np.cross(H[i1] - H[i0], H[i2] - H[i0])
    i3 = int(np.argmax(np.abs((H - H[i0]) @ nrm)))
    idx = [i0, i1, i2, i3]
    while True:
        hull = ConvexHull(H[idx])
        out = (H @ hull.equations[:, :3].T + hull.equations[:, 3]).max(axis=1)
        k = int(np.argmax(out))
        if out[k] <= 1e-7 or len(hull.vertices) >= limit:
            return H[idx][hull.vertices], float(max(out.max(), 0.0))
        idx.append(k)


def _halfspace_vertices(planes: np.ndarray, interior: np.ndarray) -> np.ndarray:
    """n·p + d <= 0 の交わりの頂点（重なりをまとめる）"""
    hs = HalfspaceIntersection(planes, interior)
    V = np.unique(np.round(hs.intersections, 10), axis=0)
    return V[ConvexHull(V).vertices]


def physx_like_hull(points: np.ndarray, limit: int = HULL_VERTEX_LIMIT, eps: float = 1e-4) -> np.ndarray:
    """PhysX の既定の頂点の上限の扱いをまねた外側の近似（元の点の OBB を、元の凸包の面で、相対的に最も深く削る面から
    順に削り、頂点が limit を超える手前で止める）。頂点が limit 以下なら真の凸包をそのまま返す。"""
    hull = ConvexHull(points)
    H = points[hull.vertices]
    if len(H) <= limit:
        return H
    planes = np.unique(np.round(hull.equations, 9), axis=0)
    c = H.mean(axis=0)
    _, R = np.linalg.eigh(np.cov((H - c).T))
    loc = (H - c) @ R
    lo, hi = loc.min(axis=0), loc.max(axis=0)
    cur = []
    for k in range(3):
        n = R[:, k]
        cur.append(np.r_[n, -(hi[k] + n @ c)])
        cur.append(np.r_[-n, lo[k] + n @ c])
    cur = np.array(cur)
    V = _halfspace_vertices(cur, c)
    while True:
        s = V @ planes[:, :3].T + planes[:, 3]
        dmax = np.maximum(s.max(axis=0), 0.0)
        dr = dmax - np.minimum(s.min(axis=0), 0.0)
        rel = dmax / np.maximum(dr, 1e-12)
        k = int(np.argmax(rel))
        if rel[k] <= eps:
            return V
        nxt = np.vstack([cur, planes[k]])
        V2 = _halfspace_vertices(nxt, c)
        if len(V2) > limit:
            return V
        cur, V = nxt, V2


def hull_faces(V: np.ndarray) -> np.ndarray:
    """凸包の三角形（外向きの順＝USD の rightHanded）[F,3]"""
    hull = ConvexHull(V)
    faces = hull.simplices.copy()
    for f, eq in zip(faces, hull.equations):
        a, b, c = V[f]
        if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
            f[1], f[2] = f[2], f[1]
    return faces


# ---------- USD を読む ----------

def _mat(xc: UsdGeom.XformCache, prim: Usd.Prim) -> np.ndarray:
    m = xc.GetLocalToWorldTransform(prim)
    return np.array([[m[i][j] for j in range(4)] for i in range(4)]).T


def quat_wxyz_to_R(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _T(R: np.ndarray, p: Sequence[float]) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


_AXIS_TO_Z = {"Z": np.eye(3), "X": np.array([[0., 0, 1], [0, 1, 0], [-1, 0, 0]]),
              "Y": np.array([[1., 0, 0], [0, 0, 1], [0, -1, 0]])}  # fcl の円柱の軸（Z）を USD の axis へ向ける


@dataclass(frozen=True)
class Joint:
    name: str
    b0: str
    b1: str
    A: np.ndarray  # body0 から見た関節の座標
    B_inv: np.ndarray  # 関節の座標から body1 へ
    axis: str
    lower: float  # [rad]
    upper: float


@dataclass(frozen=True)
class Collider:
    """1 つの凸の当たりの形（剛体の座標）"""
    link: str
    path: str
    kind: str  # convex / cylinder / sphere / box
    T: np.ndarray  # 剛体の座標から見た形の座標（回転と並進）
    size: Tuple[float, ...]  # cylinder (r, h)・sphere (r,)・box (x, y, z)・convex ()
    V: Optional[np.ndarray]  # convex の頂点（剛体の座標）
    n_src_hull: int  # convex の元の凸包の頂点の数（参考）
    center: np.ndarray  # 外接球（剛体の座標）
    radius: float

    @property
    def label(self) -> str:
        tail = "/".join(self.path.split("/")[-2:])
        return f"{self.link}[{tail}]"


@dataclass
class Robot:
    links0: Dict[str, np.ndarray]  # 全関節 0 での剛体の世界の座標
    joints: List[Joint]
    colliders: List[Collider] = field(default_factory=list)
    parent: Dict[str, str] = field(default_factory=dict)

    def adjacent(self, a: str, b: str) -> bool:
        """関節で直接つながった親子か（PhysX の関節の自動の除外）"""
        return self.parent.get(a) == b or self.parent.get(b) == a

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        jd = {j.name: j for j in self.joints}
        return np.array([jd[n].lower for n in DOF]), np.array([jd[n].upper for n in DOF])

    def collider_pairs(self) -> List[Tuple[int, int]]:
        C = self.colliders
        return [(i, j) for i in range(len(C)) for j in range(i + 1, len(C))
                if C[i].link != C[j].link and not self.adjacent(C[i].link, C[j].link)]


def _bound(V: np.ndarray) -> Tuple[np.ndarray, float]:
    c = (V.min(axis=0) + V.max(axis=0)) / 2
    return c, float(np.linalg.norm(V - c, axis=1).max())


def _make_collider(link: str, prim: Usd.Prim, M: np.ndarray, hull: str) -> Collider:
    t = prim.GetTypeName()
    path = str(prim.GetPath())
    S = np.linalg.norm(M[:3, :3], axis=0)
    R = M[:3, :3] / S
    if t == "Mesh":
        approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        if approx != "convexHull":
            raise ValueError(f"{path}: approximation={approx}（convexHull だけを扱う）")
        P = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        P = P @ M[:3, :3].T + M[:3, 3]
        n_src = len(ConvexHull(P).vertices)
        V = physx_like_hull(P) if hull == "physx" else P[ConvexHull(P).vertices]
        c, r = _bound(V)
        return Collider(link, path, "convex", np.eye(4), (), V, n_src, c, r)
    if not np.allclose(S, S[0], rtol=1e-6) and t != "Cube":
        raise ValueError(f"{path}: 一様でない拡大 {S}")
    if t == "Cylinder":
        g = UsdGeom.Cylinder(prim)
        r, h = g.GetRadiusAttr().Get() * S[0], g.GetHeightAttr().Get() * S[0]
        T = _T(R @ _AXIS_TO_Z[g.GetAxisAttr().Get()], M[:3, 3])
        return Collider(link, path, "cylinder", T, (r, h), None, 0, M[:3, 3].copy(), math.hypot(r, h / 2))
    if t == "Sphere":
        r = UsdGeom.Sphere(prim).GetRadiusAttr().Get() * S[0]
        return Collider(link, path, "sphere", _T(R, M[:3, 3]), (r,), None, 0, M[:3, 3].copy(), r)
    if t == "Cube":
        dims = tuple(float(x) for x in UsdGeom.Cube(prim).GetSizeAttr().Get() * S)
        return Collider(link, path, "box", _T(R, M[:3, 3]), dims, None, 0, M[:3, 3].copy(),
                        float(np.linalg.norm(dims) / 2))
    raise ValueError(f"知らない当たりの形 {t} {path}")


def load_robot(usd_path: str, hull: str = "physx") -> Robot:
    """合成したステージ（instance proxy まで）から剛体・回転関節・当たりの形を読む。hull は physx / true"""
    stage = Usd.Stage.Open(usd_path)
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    prims = list(stage.Traverse(pred))
    bodies = {p.GetName(): p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    links0 = {k: _mat(xc, p) for k, p in bodies.items()}
    robot = Robot(links0, [])

    def owner(prim: Usd.Prim) -> Optional[str]:
        p = prim
        while p and p.IsValid():
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                return p.GetName()
            p = p.GetParent()
        return None

    for prim in prims:
        if prim.IsA(UsdPhysics.Joint):
            if not prim.IsA(UsdPhysics.RevoluteJoint):
                raise ValueError(f"回転以外の関節 {prim.GetPath()} {prim.GetTypeName()}")
            j = UsdPhysics.RevoluteJoint(prim)
            b0 = str(j.GetBody0Rel().GetTargets()[0]).split("/")[-1]
            b1 = str(j.GetBody1Rel().GetTargets()[0]).split("/")[-1]
            q0, q1 = j.GetLocalRot0Attr().Get(), j.GetLocalRot1Attr().Get()
            A = _T(quat_wxyz_to_R([q0.GetReal(), *q0.GetImaginary()]), list(j.GetLocalPos0Attr().Get()))
            B = _T(quat_wxyz_to_R([q1.GetReal(), *q1.GetImaginary()]), list(j.GetLocalPos1Attr().Get()))
            robot.joints.append(Joint(prim.GetName(), b0, b1, A, np.linalg.inv(B), j.GetAxisAttr().Get(),
                                      math.radians(j.GetLowerLimitAttr().Get()),
                                      math.radians(j.GetUpperLimitAttr().Get())))
            robot.parent[b1] = b0
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        link = owner(prim)
        if link is None:
            raise ValueError(f"剛体の外の当たりの形 {prim.GetPath()}")
        M = np.linalg.inv(links0[link]) @ _mat(xc, prim)
        robot.colliders.append(_make_collider(link, prim, M, hull))
    missing = set(DOF) - {j.name for j in robot.joints}
    if missing:
        raise ValueError(f"USD に関節が無い: {sorted(missing)}")
    return robot


# ---------- FK ----------

def _rot(axis: str, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    R = {"X": [[1, 0, 0], [0, c, -s], [0, s, c]], "Y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
         "Z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
    return _T(np.array(R, dtype=float), np.zeros(3))


def fk(robot: Robot, q: Sequence[float]) -> Dict[str, np.ndarray]:
    """pelvis を全関節 0 のときの位置に置いた FK。q は DOF の順の 29。
    body1 = body0 · localPose0 · Rot(axis, q) · localPose1⁻¹（UsdPhysics の回転関節の定義）"""
    qd = dict(zip(DOF, q))
    W = {"pelvis": robot.links0["pelvis"]}
    pending = list(robot.joints)
    while pending:
        rest = [j for j in pending if j.b0 not in W]
        for j in pending:
            if j.b0 in W:
                W[j.b1] = W[j.b0] @ j.A @ _rot(j.axis, qd.get(j.name, 0.0)) @ j.B_inv
        if len(rest) == len(pending):
            raise RuntimeError("関節の木がつながらない")
        pending = rest
    return W


# ---------- 符号つき距離（python-fcl） ----------

def _fcl_geom(c: Collider):
    if c.kind == "convex":
        h = ConvexHull(c.V)
        faces = hull_faces(c.V)
        remap = {v: i for i, v in enumerate(h.vertices)}
        V = c.V[h.vertices]
        flat = np.concatenate([[3, remap[a], remap[b], remap[d]] for a, b, d in faces]).astype(np.int64)
        return fcl.Convex(V, len(faces), flat)
    if c.kind == "cylinder":
        return fcl.Cylinder(c.size[0], c.size[1])
    if c.kind == "sphere":
        return fcl.Sphere(c.size[0])
    if c.kind == "box":
        return fcl.Box(*c.size)
    raise ValueError(c.kind)


class Distance:
    """当たりの形どうしの符号つき距離 [m]（負＝食い込み）"""

    def __init__(self, robot: Robot):
        self.robot = robot
        self.objs = [fcl.CollisionObject(_fcl_geom(c), fcl.Transform()) for c in robot.colliders]
        self.pairs = robot.collider_pairs()

    def place(self, W: Dict[str, np.ndarray]) -> None:
        for o, c in zip(self.objs, self.robot.colliders):
            M = W[c.link] @ c.T
            o.setTransform(fcl.Transform(M[:3, :3], M[:3, 3]))

    def pair(self, i: int, j: int) -> float:
        req = fcl.DistanceRequest(enable_signed_distance=True)
        res = fcl.DistanceResult()
        return float(fcl.distance(self.objs[i], self.objs[j], req, res))

    def scan(self, W: Dict[str, np.ndarray], pairs: Optional[Iterable[Tuple[int, int]]] = None,
             within: float = 0.01) -> Dict[Tuple[int, int], float]:
        """外接球で within [m] より近い組だけ距離を測る"""
        self.place(W)
        C = self.robot.colliders
        cw = [W[c.link][:3, :3] @ c.center + W[c.link][:3, 3] for c in C]
        out = {}
        for i, j in (self.pairs if pairs is None else pairs):
            if np.linalg.norm(cw[i] - cw[j]) <= C[i].radius + C[j].radius + within:
                out[(i, j)] = self.pair(i, j)
        return out
