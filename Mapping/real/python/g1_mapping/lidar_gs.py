"""LiDAR だけで 3D Gaussian Splatting を回す（RGB を使わない）。

## なぜ普通の 3DGS がそのまま使えないのか

3DGS の公開実装（INRIA / gsplat / nerfstudio）は **ピンホールカメラの RGB 画像**を
前提にしている。この G1 の入力は Mid-360 の生スキャンで、

* この記録に画像が無い（記録したのは点群 2 種・odom・IMU の 4 トピックだけ）。
  ただし **機体にはカメラがあり、生の Livox トピックには intensity も入っている**。
  「色が使えない」のではなく、この記録では手元に無いだけである（2026-09-05 訂正）。
* 水平 360° を見る。ピンホールでは 1 枚に収まらない。
* 「画素」ではなく **1 本ずつの光線と、その距離**が観測値である。

そこで本モジュールは、画像ラスタライザではなく **レイ単位の Gaussian 合成**で書いてある。
これは 3DGRT / LiDAR-RT 系が使う定式化で、2D の EWA 近似を経由しないぶん
LiDAR には素直で、ピンホール／360° の問題も起きない。

## レイ 1 本に対するガウシアンの応答

ガウシアン（平均 μ・共分散 Σ・不透明度 o）と、光線 x(t) = p + t·d（d は単位ベクトル）。
Δ = μ − p、Σ⁻¹ = MᵀM（M = S⁻¹Rᵀ）と置くと、

    a = M·d,  b = M·Δ
    t* = (a·b) / |a|²                     … 光線上で応答が最大になる距離
    q  = |b|² − (a·b)² / |a|²             … そのときのマハラノビス距離²
    α  = o · exp(−q/2)

t* の昇順に手前から合成する（T は透過率）:

    depth = Σ αᵢ Tᵢ t*ᵢ ,  acc = Σ αᵢ Tᵢ ,  Tᵢ₊₁ = Tᵢ (1 − αᵢ)

## どのガウシアンがどの光線に効くか

センサ位置を中心に方位角・仰角で等間隔のビンを切り（range image と同じ考え方）、
各ガウシアンを角度 footprint（既定 2σ）が覆うビンすべてに登録する。光線は自分のビンから
手前 K 個を取る。これは 3DGS のタイル分割を球面に置き換えたもので、
「タイル内はガウシアン中心の奥行き順」という近似も本家と同じである。

## 動的な点が消える理屈

OctoMap は「後からそこを光線が通り抜けた＝そのとき空だった」を log-odds の票で数える。
こちらは同じ証拠を**連続量の勾配**で受ける。追従者の居た場所は数秒後には光線が
通り抜けるので、そのガウシアンを残したままだと描画距離が手前に寄って depth loss が増える。
損失を下げる向きは「そのガウシアンの α を下げる」なので、不透明度が落ちて刈られる。
机は光線が必ず天板で止まるため、下げる圧力がかからない。

**光線が一度も当たらなかったガウシアンには勾配が来ない**ので、初期不透明度のまま残る。
これは OctoMap の「未知 → 残す」と同じ挙動である（そろえるため、既定では
opacity reset を行わない。有効にすると未観測のガウシアンまで道連れに死ぬ）。

## 借りているもの

刈り込みと分割は gsplat（`gsplat.strategy.ops`）の実装をそのまま呼ぶ。
パラメータを増減させるときに Adam の内部状態を食い違わせないのが面倒な箇所で、
そこは自前で書かない。`ops` の規約に合わせて `scales` は log、`opacities` は logit、
`quats` は wxyz で持つ。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import torch
import torch.nn.functional as F
from torch import Tensor

from gsplat.utils import normalized_quat_to_rotmat

EPS = 1e-9
MAX_ALPHA = 0.999
# 1 つのガウシアンが覆える球面ビンの上限。センサのすぐ近くにあるガウシアンは
# 見込み角が大きく（σ=4.6cm・距離1m で 2σ が 5.3°）ビン数が跳ね上がるので、
# 上限を超えたら footprint を相似に縮める。中心 1 ビンに畳むと、光線から
# 少し外れたところに中心があるガウシアンを取り落とす。
MAX_CELLS_PER_GAUSSIAN = 1024
# 反射強度を [0,1] に落とすときの割り算。実測の最大は 155 なので余裕を見て 255 で固定する
# （記録ごとに変わる正規化を入れると、別の記録と比べられなくなる）
INTENSITY_SCALE = 255.0
# 3DGS の .ply で色を SH 0 次に入れるときの係数（INRIA の実装と同じ）
SH_C0 = 0.28209479177387814


@dataclass(frozen=True)
class RenderConfig:
    """球面ビンの切り方と、1 本の光線が見るガウシアンの数。"""

    n_azimuth: int = 360            # 1.0°
    n_elevation: int = 90           # 2.0°
    max_per_ray: int = 64           # K。手前から数えてこの数だけ合成する
    near: float = 0.05              # これより手前の応答は捨てる[m]
    sigma: float = 2.0              # footprint を何σまで広げるか

    @property
    def n_bins(self) -> int:
        return self.n_azimuth * self.n_elevation


# ────────────────────────────────────────────────────────────────
# パラメータの初期化
# ────────────────────────────────────────────────────────────────


def _rotmat_to_quat_wxyz(rotations: np.ndarray) -> np.ndarray:
    """(N,3,3) の回転行列を (N,4) の四元数 wxyz にする（gsplat の並び）。"""
    m = rotations
    trace = m[:, 0, 0] + m[:, 1, 1] + m[:, 2, 2]
    quaternions = np.empty((len(m), 4), dtype=np.float64)

    # trace が正なら w から、そうでなければ最大の対角要素から作る（数値安定のため）
    positive = trace > 0
    s = np.sqrt(np.maximum(trace[positive] + 1.0, EPS)) * 2
    quaternions[positive, 0] = 0.25 * s
    quaternions[positive, 1] = (m[positive, 2, 1] - m[positive, 1, 2]) / s
    quaternions[positive, 2] = (m[positive, 0, 2] - m[positive, 2, 0]) / s
    quaternions[positive, 3] = (m[positive, 1, 0] - m[positive, 0, 1]) / s

    rest = np.where(~positive)[0]
    for i in rest:
        diagonal = np.array([m[i, 0, 0], m[i, 1, 1], m[i, 2, 2]])
        k = int(diagonal.argmax())
        a, b = (k + 1) % 3, (k + 2) % 3
        s = math.sqrt(max(m[i, k, k] - m[i, a, a] - m[i, b, b] + 1.0, EPS)) * 2
        quaternions[i, 0] = (m[i, b, a] - m[i, a, b]) / s
        quaternions[i, 1 + k] = 0.25 * s
        quaternions[i, 1 + a] = (m[i, a, k] + m[i, k, a]) / s
        quaternions[i, 1 + b] = (m[i, b, k] + m[i, k, b]) / s

    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    return quaternions / np.maximum(norms, EPS)


def init_from_points(
    points: np.ndarray,
    *,
    neighbors: int = 10,
    init_opacity: float = 0.3,
    min_scale: float = 0.01,
    max_scale: float = 0.20,
    anisotropic: bool = True,
    intensities: "np.ndarray | None" = None,
) -> "dict[str, np.ndarray]":
    """点群 1 点につき 1 個のガウシアンを置く。

    `anisotropic=True` のときは近傍の主成分分析で姿勢と 3 軸のスケールを決める。
    壁や天板の上では平たい円盤になるので、等方な球を置くより表面に沿う。
    LiDAR-GS 系が初期化に使うのと同じ手口である。
    """
    from scipy.spatial import cKDTree

    tree = cKDTree(points)
    k = min(neighbors, len(points) - 1) + 1
    distances, indices = tree.query(points, k=k, workers=-1)

    if not anisotropic:
        spacing = distances[:, 1:].mean(axis=1)
        scales = np.repeat(spacing[:, None], 3, axis=1)
        quaternions = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (len(points), 1))
    else:
        neighborhood = points[indices]                             # (N, k, 3)
        centered = neighborhood - neighborhood.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", centered, centered) / max(k - 1, 1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)     # 昇順
        # eigh は昇順なので降順に入れ替える。列が固有ベクトル。
        eigenvalues = eigenvalues[:, ::-1]
        eigenvectors = eigenvectors[:, :, ::-1]
        # 右手系にそろえる（det=-1 なら第3軸を反転）
        determinants = np.linalg.det(eigenvectors)
        eigenvectors[determinants < 0, :, 2] *= -1
        scales = np.sqrt(np.maximum(eigenvalues, 0.0))
        quaternions = _rotmat_to_quat_wxyz(eigenvectors)

    scales = np.clip(scales, min_scale, max_scale)
    arrays = {
        "means": points.astype(np.float32),
        "scales": np.log(scales).astype(np.float32),
        "quats": quaternions.astype(np.float32),
        "opacities": np.full(len(points), math.log(init_opacity / (1 - init_opacity)),
                             dtype=np.float32),
    }
    if intensities is not None:
        # 反射強度を 1 チャンネルの「色」として持たせる。logit で持つのは
        # 不透明度と同じ理由（sigmoid で [0,1] に閉じ込め、勾配が飽和しにくい）
        normalized = np.clip(intensities / INTENSITY_SCALE, 1e-3, 1 - 1e-3)
        arrays["colors"] = np.log(normalized / (1 - normalized)).astype(np.float32)[:, None]
    return arrays


def to_parameters(arrays: "dict[str, np.ndarray]", device: str) -> "dict[str, torch.nn.Parameter]":
    return {
        name: torch.nn.Parameter(torch.from_numpy(value).to(device))
        for name, value in arrays.items()
    }


# ────────────────────────────────────────────────────────────────
# 描画
# ────────────────────────────────────────────────────────────────


def precision_matrices(quats: Tensor, log_scales: Tensor) -> Tensor:
    """M = S⁻¹Rᵀ を返す。Σ⁻¹ = MᵀM である。

    Σ = R S S Rᵀ なので Σ⁻¹ = R S⁻¹ S⁻¹ Rᵀ = (S⁻¹Rᵀ)ᵀ(S⁻¹Rᵀ)。
    3x3 を毎回逆行列にかけるより、この形で持って行列ベクトル積 2 回で済ませる。
    """
    rotations = normalized_quat_to_rotmat(F.normalize(quats, dim=-1))   # [N,3,3]
    inverse_scales = torch.exp(-log_scales)                             # [N,3]
    return inverse_scales[:, :, None] * rotations.transpose(1, 2)


COARSE = 4      # 間引き用の粗ビンは、描画用ビンの何倍粗いか


@torch.no_grad()
def _cull_to_ray_directions(
    azimuth: Tensor,
    elevation: Tensor,
    angular_radius: Tensor,
    ray_bins: Tensor,
    config: RenderConfig,
) -> Tensor:
    """光線が一本も飛んでいない向きのガウシアンを落とす。

    Mid-360 の垂直 FOV は 59°（−7°〜+52°）しかないので、球面の 2/3 には
    そもそも光線が無い。粗いビンで占有を作り 1 マス膨張させて、そこに中心が
    入っていないものを捨てる。粗ビンより広い footprint を持つもの（＝センサに
    近いガウシアン）は膨張 1 マスでは足りないので、無条件に残す。
    """
    n_azimuth, n_elevation = config.n_azimuth, config.n_elevation
    # 割り切れないと最終行がはみ出すので切り上げで確保する
    coarse_az = -(-n_azimuth // COARSE)
    coarse_el = -(-n_elevation // COARSE)

    ray_az = (ray_bins % n_azimuth) // COARSE
    ray_el = (ray_bins // n_azimuth) // COARSE
    occupied = torch.zeros(coarse_el * coarse_az, dtype=torch.float32, device=azimuth.device)
    occupied[ray_el * coarse_az + ray_az] = 1.0

    # 1 マス膨張させる。方位は巡回するので左右を回り込ませてから max を取る
    grid = occupied.view(1, 1, coarse_el, coarse_az)
    wrapped = torch.cat([grid[..., -1:], grid, grid[..., :1]], dim=-1)
    wrapped = torch.nn.functional.pad(wrapped, (0, 0, 1, 1), mode="replicate")
    dilated = torch.nn.functional.max_pool2d(wrapped, kernel_size=3, stride=1).view(-1) > 0

    az_cell = (((azimuth + math.pi) / (2 * math.pi) * n_azimuth).long()
               .clamp(0, n_azimuth - 1)) // COARSE
    el_cell = (((elevation + math.pi / 2) / math.pi * n_elevation).long()
               .clamp(0, n_elevation - 1)) // COARSE
    inside = dilated[el_cell * coarse_az + az_cell]
    wide = angular_radius > (COARSE * math.pi / n_elevation)
    return inside | wide


@torch.no_grad()
def associate(
    means: Tensor,
    log_scales: Tensor,
    origin: Tensor,
    ray_directions: Tensor,
    config: RenderConfig,
) -> "tuple[Tensor, Tensor]":
    """各光線に効きそうなガウシアンを、手前から最大 K 個ずつ集める。

    返り値は (候補の id [R,K], 有効フラグ [R,K])。
    球面ビンに登録してから光線ごとに引く。3DGS のタイル分割の球面版で、
    「ビン内はガウシアン中心の距離順」という並べ方も本家と同じである。
    """
    device = means.device
    n_azimuth, n_elevation = config.n_azimuth, config.n_elevation

    # 光線がどのビンに落ちるかを先に出す
    ray_distance = ray_directions.norm(dim=-1).clamp_min(EPS)
    ray_azimuth = torch.atan2(ray_directions[:, 1], ray_directions[:, 0])
    ray_elevation = torch.asin((ray_directions[:, 2] / ray_distance).clamp(-1.0, 1.0))
    ray_az_cell = (((ray_azimuth + math.pi) / (2 * math.pi) * n_azimuth).long()
                   .clamp(0, n_azimuth - 1))
    ray_el_cell = (((ray_elevation + math.pi / 2) / math.pi * n_elevation).long()
                   .clamp(0, n_elevation - 1))
    ray_bins = ray_el_cell * n_azimuth + ray_az_cell               # [R]

    delta = means - origin                                       # [N,3]
    distance = delta.norm(dim=-1).clamp_min(EPS)
    azimuth = torch.atan2(delta[:, 1], delta[:, 0])              # [-π, π]
    elevation = torch.asin((delta[:, 2] / distance).clamp(-1.0, 1.0))

    # Mid-360 の垂直 FOV は 59° しかないので、地図の大半は「光線が一本も飛んでいない
    # 向き」にある。粗いビンで先に落としておくとペア数が半分以下になる。
    # 角度半径が粗ビンより大きいものは落とさない（膨張 1 マスでは足りないため）。
    kept = _cull_to_ray_directions(
        azimuth, elevation, angular_radius=torch.asin(
            (config.sigma * torch.exp(log_scales).max(dim=-1).values / distance).clamp(max=0.999)),
        ray_bins=ray_bins, config=config)
    global_ids = torch.where(kept)[0]
    delta, distance = delta[kept], distance[kept]
    azimuth, elevation = azimuth[kept], elevation[kept]
    log_scales = log_scales[kept]

    # config.sigma σ ぶんの角度半径。センサがガウシアンの中に入り込んでいる場合は
    # asin の引数が 1 を超えるので 0.999（≒89.9°）で頭打ちにし、あとは下の
    # MAX_CELLS_PER_GAUSSIAN で相似に縮める。
    reach = config.sigma * torch.exp(log_scales).max(dim=-1).values
    angular = torch.asin((reach / distance).clamp(max=0.999))
    widen = 1.0 / torch.cos(elevation).clamp_min(0.02)           # 極付近は方位を広げる

    azimuth_px = (azimuth + math.pi) / (2 * math.pi) * n_azimuth
    elevation_px = (elevation + math.pi / 2) / math.pi * n_elevation
    radius_az = (angular * widen) / (2 * math.pi) * n_azimuth
    radius_el = angular / math.pi * n_elevation

    # 上限を超える footprint は相似に縮める（中心は保つ）
    cells = (2 * radius_az + 1) * (2 * radius_el + 1)
    shrink = torch.where(cells > MAX_CELLS_PER_GAUSSIAN,
                         torch.sqrt(MAX_CELLS_PER_GAUSSIAN / cells.clamp_min(EPS)),
                         torch.ones_like(cells))
    radius_az = radius_az * shrink
    radius_el = radius_el * shrink

    az_low = torch.floor(azimuth_px - radius_az).long()
    az_high = torch.ceil(azimuth_px + radius_az).long()
    el_low = torch.floor(elevation_px - radius_el).long().clamp(0, n_elevation - 1)
    el_high = torch.ceil(elevation_px + radius_el).long().clamp(0, n_elevation - 1)

    az_count = (az_high - az_low + 1).clamp(1, n_azimuth)
    el_count = (el_high - el_low + 1).clamp(1, n_elevation)

    counts = az_count * el_count                                  # [N]
    total = int(counts.sum().item())
    if total == 0:
        empty = torch.zeros((len(ray_directions), config.max_per_ray),
                            dtype=torch.long, device=device)
        return empty, torch.zeros_like(empty, dtype=torch.bool)

    gaussian_ids = torch.repeat_interleave(
        torch.arange(len(distance), device=device), counts)         # [P]（間引き後の番号）
    starts = torch.cumsum(counts, 0) - counts
    local = torch.arange(total, device=device) - torch.repeat_interleave(starts, counts)
    az_offset = local % az_count[gaussian_ids]
    el_offset = local // az_count[gaussian_ids]
    del local
    az_cell = (az_low[gaussian_ids] + az_offset) % n_azimuth
    el_cell = (el_low[gaussian_ids] + el_offset).clamp(0, n_elevation - 1)
    del az_offset, el_offset
    bin_ids = el_cell * n_azimuth + az_cell                        # [P]
    del az_cell, el_cell

    # (ビン, 距離) の順に並べる。int64 の上位にビン、下位に距離を量子化して詰める。
    # ここが一番メモリを食うので、要らなくなった中間結果はその場で捨てる。
    depth_key = (distance[gaussian_ids] / 200.0).clamp(0.0, 1.0)
    depth_key = (depth_key * ((1 << 30) - 1)).long()
    sort_key = bin_ids.long() << 32 | depth_key
    del depth_key
    order = sort_key.argsort()
    del sort_key
    sorted_bins = bin_ids[order]
    sorted_gaussians = gaussian_ids[order]
    del bin_ids, gaussian_ids, order

    # 光線ごとに自分のビンの先頭 K 個を取る
    low = torch.searchsorted(sorted_bins.contiguous(), ray_bins.contiguous(), right=False)
    high = torch.searchsorted(sorted_bins.contiguous(), (ray_bins + 1).contiguous(), right=False)
    slots = torch.arange(config.max_per_ray, device=device)
    index = low[:, None] + slots[None, :]
    valid = index < high[:, None]
    index = index.clamp(max=max(total - 1, 0))
    return global_ids[sorted_gaussians[index]], valid


def render_rays(
    params: "dict[str, torch.nn.Parameter]",
    origin: Tensor,
    ray_directions: Tensor,
    config: RenderConfig,
    candidates: "tuple[Tensor, Tensor] | None" = None,
    with_color: bool = False,
    free_space: "Tensor | None" = None,
    free_space_margin: float = 0.10,
    free_space_max_range: float = 3.0,
) -> "tuple[Tensor, Tensor, Tensor] | tuple[Tensor, Tensor, Tensor, Tensor]":
    """光線ごとの (期待距離, 累積不透明度, 各ガウシアンの寄与重み) を返す。

    `with_color=True` なら 4 つめに合成した色（＝反射強度）も返す。
    `free_space`（光線ごとの実測距離）を渡すと、最後に**自由空間の罰則**も返す——
    実測より手前にあるガウシアンの α の和で、透過率による重み付けをしない。

    `ray_directions` は単位ベクトルでなくてよい（正規化する）。
    返す距離は α 合成した期待値そのままで、`acc` で割っていない。

    1 スキャンぶんの対応付けは 2.3 GB を一時的に確保する（実測・720x180・2σ）。
    方位で区切って分割処理する案も試したが、間引きが区画ごとに全ガウシアンを
    走査するので 2 倍遅くなるうえ、区画の境目で最大 33 cm ずれたので採らなかった。
    """
    means = params["means"]
    log_scales = params["scales"]
    directions = F.normalize(ray_directions, dim=-1)

    if candidates is None:
        candidates = associate(means.detach(), log_scales.detach(), origin, directions, config)
    gaussian_ids, valid = candidates

    # 全 N 個ぶん作らず、拾われたものだけ組み立てる。667k 個の回転行列を毎回
    # 作ると autograd がそのぶん中間結果を抱えるので、ここは効く。
    rays, per_ray = gaussian_ids.shape
    flat = gaussian_ids.reshape(-1)
    selected = precision_matrices(params["quats"][flat], log_scales[flat])
    selected = selected.reshape(rays, per_ray, 3, 3)                    # [R,K,3,3]
    delta = means[flat].reshape(rays, per_ray, 3) - origin              # [R,K,3]

    a = torch.einsum("rkij,rj->rki", selected, directions)              # M·d
    b = torch.einsum("rkij,rkj->rki", selected, delta)                  # M·Δ
    aa = (a * a).sum(-1).clamp_min(EPS)
    ab = (a * b).sum(-1)
    bb = (b * b).sum(-1)

    t_star = ab / aa
    quadratic = (bb - ab * ab / aa).clamp_min(0.0)
    response = torch.exp(-0.5 * quadratic)
    opacity = torch.sigmoid(params["opacities"])[gaussian_ids]
    alpha = (opacity * response).clamp(max=MAX_ALPHA)
    alpha = alpha * (valid & (t_star > config.near)).to(alpha.dtype)

    # 手前から合成する
    order = t_star.argsort(dim=1)
    alpha = torch.gather(alpha, 1, order)
    distance = torch.gather(t_star, 1, order)
    transmittance = torch.cumprod(
        torch.cat([torch.ones_like(alpha[:, :1]), 1.0 - alpha[:, :-1]], dim=1), dim=1)
    weight = alpha * transmittance
    depth, accumulation = (weight * distance).sum(1), weight.sum(1)

    if free_space is not None:
        # ── 自由空間の罰則 ──────────────────────────────────
        # depth loss は α 合成した結果にしか効かないので、手前が濃いと
        # 透過率が尽きて奥のガウシアンに勾配が届かない。OctoMap が
        # 「光線が通ったセル全部」に空の票を入れるのに対し、これは
        # 「手前の 1 個」にしか効かないということ。ここを埋める。
        #
        # 実測距離より手前にあるガウシアンは、その光線から見れば空でなければ
        # おかしい。透過率で重み付けせず α をそのまま罰する。
        #
        # max_range を切るのは OctoMap と同じ理由。Mid-360 は垂直 FOV −7° で
        # 床を斜入射でしか見ないので、長い光線が床のガウシアンを舐めて
        # 「空」と判定してしまう（OctoMap では天井 56.7 % が消えた）。
        limit = torch.minimum(free_space - free_space_margin,
                              torch.full_like(free_space, free_space_max_range))
        ahead = (distance < limit[:, None]) & (distance > config.near)
        penalty = (alpha * ahead.to(alpha.dtype)).sum(1)
    else:
        penalty = None

    if not with_color:
        return (depth, accumulation, weight) if penalty is None else (
            depth, accumulation, weight, penalty)
    color = torch.sigmoid(params["colors"])[gaussian_ids].squeeze(-1)   # [R,K]
    color = torch.gather(color, 1, order)
    composited = (weight * color).sum(1)
    if penalty is None:
        return depth, accumulation, weight, composited
    return depth, accumulation, weight, composited, penalty


@torch.no_grad()
def query_density(
    params: "dict[str, torch.nn.Parameter]",
    points: Tensor,
    *,
    neighbors: int = 8,
    chunk: int = 100_000,
) -> Tensor:
    """各点における「一番強く効いているガウシアンの α」を返す。

    掃除後の地図を作るとき、点が学習後のガウシアン場に支えられているかを見る。
    OctoMap の `getLabels()`（占有 / 空 / 未知）に相当する問い合わせである。
    """
    from scipy.spatial import cKDTree

    means_np = params["means"].detach().cpu().numpy()
    tree = cKDTree(means_np)
    precision = precision_matrices(params["quats"], params["scales"])
    opacity = torch.sigmoid(params["opacities"])

    out = torch.empty(len(points), device=points.device)
    for start in range(0, len(points), chunk):
        stop = min(start + chunk, len(points))
        block = points[start:stop]
        _, indices = tree.query(block.cpu().numpy(), k=neighbors, workers=-1)
        # k=1 のとき cKDTree は (N,) を返す。常に (N, k) に揃える
        indices = np.atleast_2d(np.ascontiguousarray(indices))
        if indices.shape[0] != len(block):
            indices = indices.T
        ids = torch.from_numpy(np.ascontiguousarray(indices)).long().to(points.device)
        delta = block[:, None, :] - params["means"][ids]
        b = torch.einsum("bkij,bkj->bki", precision[ids], delta)
        out[start:stop] = (opacity[ids] * torch.exp(-0.5 * (b * b).sum(-1))).max(dim=1).values
    return out


# ────────────────────────────────────────────────────────────────
# 3DGS の .ply 読み書き（INRIA 形式）
# ────────────────────────────────────────────────────────────────

# 3DGS の .ply は「テキストのヘッダ + little-endian の生レコード」なので、
# numpy の構造化配列だけで足りる。`plyfile` は GPLv3 なので入れない。
SPLAT_FIELDS = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4"),
                ("f_dc_0", "<f4"), ("f_dc_1", "<f4"), ("f_dc_2", "<f4"),
                ("opacity", "<f4"),
                ("scale_0", "<f4"), ("scale_1", "<f4"), ("scale_2", "<f4"),
                ("rot_0", "<f4"), ("rot_1", "<f4"), ("rot_2", "<f4"), ("rot_3", "<f4")]


def write_splat_ply(path, params: "dict[str, torch.nn.Parameter]") -> None:
    """どの 3DGS ビューアでも開ける .ply を書く（INRIA の並び）。

    色が無いので f_dc は SH 0 次の中間灰色に固定する（DC = (色 − 0.5) / 0.28209479）。
    形と不透明度だけを持つスプラットになる。
    """
    means = params["means"].detach().cpu().numpy().astype(np.float32)
    scales = params["scales"].detach().cpu().numpy().astype(np.float32)
    quats = F.normalize(params["quats"].detach(), dim=-1).cpu().numpy().astype(np.float32)
    opacities = params["opacities"].detach().cpu().numpy().astype(np.float32)

    records = np.zeros(len(means), dtype=SPLAT_FIELDS)
    records["x"], records["y"], records["z"] = means.T
    if "colors" in params:
        # 反射強度を SH 0 次に入れる。3 チャンネル同値なので、ビューアでは灰色階調になる
        grey = torch.sigmoid(params["colors"].detach()).cpu().numpy().reshape(-1)
        dc = ((grey - 0.5) / SH_C0).astype(np.float32)
        records["f_dc_0"] = records["f_dc_1"] = records["f_dc_2"] = dc
    else:
        records["f_dc_0"] = records["f_dc_1"] = records["f_dc_2"] = 0.0   # 中間灰色
    records["opacity"] = opacities
    records["scale_0"], records["scale_1"], records["scale_2"] = scales.T
    records["rot_0"], records["rot_1"], records["rot_2"], records["rot_3"] = quats.T

    header = ["ply", "format binary_little_endian 1.0", f"element vertex {len(records)}"]
    header += [f"property float {name}" for name, _ in SPLAT_FIELDS]
    header += ["end_header", ""]
    with open(path, "wb") as stream:
        stream.write("\n".join(header).encode("ascii"))
        stream.write(records.tobytes())


def read_splat_ply(path) -> "dict[str, np.ndarray]":
    """`write_splat_ply` が書いたものを読み戻す。図を作るときに使う。

    色（f_dc）は捨てる。こちらは形と不透明度しか持っていないため。
    """
    with open(path, "rb") as stream:
        names, count = [], 0
        while True:
            line = stream.readline().decode("ascii").strip()
            if not line:
                raise ValueError(f"end_header が見つからない: {path}")
            if line.startswith("element vertex"):
                count = int(line.split()[-1])
            elif line.startswith("property float"):
                names.append(line.split()[-1])
            elif line == "end_header":
                break
            elif line.startswith("format") and "binary_little_endian" not in line:
                raise ValueError(f"binary_little_endian のみ対応: {line}")
        records = np.frombuffer(stream.read(count * 4 * len(names)),
                                dtype=[(name, "<f4") for name in names], count=count)

    column = lambda name: np.ascontiguousarray(records[name])          # noqa: E731
    loaded = {
        "means": np.stack([column("x"), column("y"), column("z")], axis=1),
        "scales": np.stack([column(f"scale_{i}") for i in range(3)], axis=1),
        "quats": np.stack([column(f"rot_{i}") for i in range(4)], axis=1),
        "opacities": column("opacity"),
    }
    grey = column("f_dc_0") * SH_C0 + 0.5
    if np.ptp(grey) > 1e-6:      # 一様な灰色なら「色なし」で書かれたもの
        clipped = np.clip(grey, 1e-3, 1 - 1e-3)
        loaded["colors"] = np.log(clipped / (1 - clipped)).astype(np.float32)[:, None]
    return loaded
