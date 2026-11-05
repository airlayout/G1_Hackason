"""11-05: 頭のカラーの画像だけで扉を開ける生徒（VIRAL6_vis の vision_door_student.VisionDoorStudent）を実機で回す計算。
numpy だけ（PC2 の Python 3.8 でも動く）。計画は docs/plan/2026-11-05-g1-door-vision-rl.md。

生徒は f3sc（状態入力の生徒）の観測 151 の並びをそのまま使い、扉の 3 項目（19 個）だけを画像から当てた値に差し替える:
  1. door_core.build_actor_obs で観測 151 を作る（鏡写しの後。扉の項目は使わないので何が入っていてもよい）
  2. 扉の頭: [画像の特徴 128, 頭の重力 3, 胴の重力 3, 腰 yaw・roll・pitch 3] → 扉の値 17（鏡写しの側）
  3. 扉の 3 項目へ差し込む: door_angle_lidar = [角度, 1（測れた）, angle_age]・door_frame_loc 9・door_geometry 7
  4. 生徒の正規化（f3sc の値の写し）→ [正規化した 151, 特徴 128, 頭の重力 3] を LSTM 2 層 → MLP → 行動 17（鏡写しの側）
door_core.step(..., policy=VisionActor(...)) で使う。行動の後（積み上げ・HOMIE・指令）は f3sc と同じ。

追う段（11-05、d3 から。meta の vision.tracker_hidden > 0）: 2 と 3 の間で、2 の当てを差し込んだ観測を 4 と同じ形に直した入力
[正規化した 151, 特徴, 頭の重力] を記憶つきの LSTM 1 層に通し、扉の値を「2 の当て + 出口 × 許す誤差」に当て直してから差し込む。
追う段の (h, c) は door_core.CoreState.policy_state で持ち越す（"door" の周期だけ進む・reset_actor で 0 に戻る）。

頭の重力（vis_head_gravity）: シミュレータは torso_link の向きにカメラの俯角（torso の y 軸まわりに下向き）を掛けた座標で
重力の向きを見る（door_vision_feat.head_gravity）。実機は pelvis の IMU と腰の 3 関節（Rz·Rx·Ry、door_lidar_msg と同じ）で
torso の向きを組み、その日に測った俯角を掛ける。歩く間の頭の頷きは入れていない（学習は頷き 0〜4° と雑音 0.02 に耐える形）。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

import door_core as dc

DOOR_TERMS = ("door_angle_lidar", "door_frame_loc", "door_geometry")
DOOR_DIM = 17
AUX_DIM = 9
WAIST = slice(12, 15)  # SDK の関節の番号 12〜14: waist_yaw・waist_roll・waist_pitch


@dataclass(frozen=True)
class VisionParams:
    lstm: Tuple[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray], ...]  # 1 層目の入力は 151 + 特徴 + 3
    actor_mlp: dc.Mlp
    head: dc.Mlp  # [特徴, 手掛かり 9] → 正規化した扉の値 17
    head_out_mean: np.ndarray
    head_out_std: np.ndarray
    t_mean: np.ndarray  # [151] 生徒の正規化（f3sc の値の写し）
    t_var: np.ndarray
    angle_age: float
    feat_dim: int
    door_slices: Dict[str, Tuple[int, int]]  # 観測 151 の中の扉の 3 項目
    pg_slice: Tuple[int, int]  # projected_gravity
    waist_slice: Tuple[int, int]  # dof_pos の腰 3
    hg_mirror_M: np.ndarray  # [3, 3] 頭の重力の鏡写し（door_mirror.py の vis_head_gravity）
    meta: Dict  # 書き出しの記録（カメラの画角・解像度・俯角・画像の網）
    tracker: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None  # 追う段の LSTM 1 層（無ければ None）
    tracker_out: Optional[Tuple[np.ndarray, np.ndarray]] = None  # [17, H]・[17] 当て直しの出口
    door_tol: Optional[np.ndarray] = None  # [17] 出口の単位（項目ごとの許す誤差）


def load_vision(path: str, p: dc.Params) -> VisionParams:
    """export_vision_student.py が書いた npz を読む。p は同じ npz を door_core.load_params で読んだもの"""
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    vis = meta.get("vision")
    if vis is None:
        raise ValueError(f"{path} は画像の生徒の書き出しでない（meta に vision が無い）")
    lstm = tuple(
        (z[f"vis_lstm{i}_w_ih"], z[f"vis_lstm{i}_w_hh"], z[f"vis_lstm{i}_b_ih"], z[f"vis_lstm{i}_b_hh"])
        for i in range(dc.LSTM_LAYERS)
    )
    feat_dim = int(vis["feat_dim"])
    want = p.obs_dim + feat_dim + 3
    if lstm[0][0].shape[1] != want:
        raise ValueError(f"LSTM の入力の幅 {lstm[0][0].shape[1]} が 観測 {p.obs_dim}＋特徴 {feat_dim}＋3 と合わない")
    dof0 = p.layout["dof_pos"][0]
    trk = {}
    if int(vis.get("tracker_hidden", 0)) > 0:
        trk = dict(
            tracker=tuple(z[f"vis_trk_{k}"] for k in ("w_ih", "w_hh", "b_ih", "b_hh")),
            tracker_out=(z["vis_trk_out_w"], z["vis_trk_out_b"]),
            door_tol=z["vis_door_tol"],
        )
        if trk["tracker"][0].shape[1] != want:
            raise ValueError(f"追う段の入力の幅 {trk['tracker'][0].shape[1]} が {want} と合わない")
    return VisionParams(
        lstm=lstm,
        actor_mlp=dc._mlp_from(z, "vis_actor", 4, "silu"),
        head=dc._mlp_from(z, "vis_head", int(vis["head_layers"]), "silu"),
        head_out_mean=z["vis_head_out_mean"],
        head_out_std=z["vis_head_out_std"],
        t_mean=z["vis_t_mean"],
        t_var=z["vis_t_var"],
        angle_age=float(vis["angle_age"]),
        feat_dim=feat_dim,
        door_slices={t: p.layout[t] for t in DOOR_TERMS},
        pg_slice=p.layout["projected_gravity"],
        waist_slice=(dof0 + WAIST.start, dof0 + WAIST.stop),
        hg_mirror_M=z["vis_hg_mirror_M"],
        meta=vis,
        **trk,
    )


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    """四元数（w, x, y, z、機体 → 世界）の回転行列"""
    w, x, y, z = (float(v) for v in np.asarray(q, dtype=float) / np.linalg.norm(q))
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def head_gravity(quat_pelvis_wxyz: np.ndarray, q_waist: np.ndarray, pitch_rad: float) -> np.ndarray:
    """頭（カメラの体の座標。前 +x・上 +z。torso の y 軸まわりに pitch_rad 下を向く）から見た重力の向き（実の向き）。
    直立・腰 0 なら (sin θ, 0, −cos θ)（door_vision_feat.head_gravity と同じ）"""
    r_torso = quat_wxyz_to_matrix(quat_pelvis_wxyz) @ _rz(q_waist[0]) @ _rx(q_waist[1]) @ _ry(q_waist[2])
    r_head = r_torso @ _ry(pitch_rad)
    return r_head.T @ np.array([0.0, 0.0, -1.0])


def floor_tilt_deg(hg: np.ndarray) -> float:
    """頭の重力の向き → 頭の床からの俯角 [°]（READY の表示と同じ式。鏡写しは y だけなので、どちらの向きでも同じ）"""
    return math.degrees(math.atan2(hg[0], -hg[2]))


def pitch_for_floor_tilt(quat_pelvis_wxyz: np.ndarray, q_waist: np.ndarray, tilt_deg: float, pitch0_rad: float) -> float:
    """頭の床からの俯角が tilt_deg（カメラ D435i の加速度計で測った値。pc2/cam_tilt.py）になる torso から見た俯角 [rad]。
    床からの俯角は torso からの俯角に胴の傾きを足したものでほぼ線形なので、差を足し込む不動点を 3 回で十分（誤差 1e-6° 以下）"""
    pitch = pitch0_rad
    for _ in range(3):
        pitch += math.radians(tilt_deg - floor_tilt_deg(head_gravity(quat_pelvis_wxyz, q_waist, pitch)))
    return pitch


def head_pitch_out_of_range(vp: VisionParams, pitch_rad: float) -> Optional[str]:
    """torso からの俯角が学習の乱択（meta の camera.pitch_range_deg）の外なら理由、内なら None。
    11-06 朝: 首は受動で、一晩で頭が 53° → 84°（画面は自分の足）に垂れていた。その画像で方策を回さない"""
    lo, hi = (float(v) for v in vp.meta["camera"]["pitch_range_deg"])
    deg = math.degrees(pitch_rad)
    if lo <= deg <= hi:
        return None
    return f"頭の俯角 {deg:.1f}° が学習の範囲 {lo:.0f}〜{hi:.0f}° の外（頭が垂れて／上がっていないか見る。直したら測り直して起こす）"


def camera_pitch_deg(vp: VisionParams) -> float:
    """学習の取付のカメラの俯角 [°]（camera_rot_offset は torso の y 軸まわりの回転だけ。wxyz）"""
    w, x, y, z = vp.meta["camera"]["rot_offset_wxyz"]
    if abs(x) > 1e-6 or abs(z) > 1e-6:
        raise ValueError(f"カメラの取付が y 軸まわりだけの回転でない: {vp.meta['camera']['rot_offset_wxyz']}")
    return math.degrees(2.0 * math.atan2(y, w))


def mirror_head_gravity(vp: VisionParams, hg: np.ndarray, mirror: bool) -> np.ndarray:
    """方策が見る向きへ（把手が左の扉なら鏡写し＝y を反転）"""
    return vp.hg_mirror_M @ hg if mirror else np.asarray(hg, dtype=float)


def door_values(vp: VisionParams, obs: np.ndarray, feat: np.ndarray, hg_m: np.ndarray) -> np.ndarray:
    """扉の頭: 観測 151（鏡写しの後）から胴の重力と腰を取り、特徴・頭の重力と合わせて扉の値 17（鏡写しの側）を当てる"""
    aux = np.concatenate([hg_m, obs[vp.pg_slice[0]:vp.pg_slice[1]], obs[vp.waist_slice[0]:vp.waist_slice[1]]])
    return vp.head(np.concatenate([feat, aux])) * vp.head_out_std + vp.head_out_mean


def insert_door(vp: VisionParams, obs: np.ndarray, door17: np.ndarray) -> np.ndarray:
    """観測 151 の扉の 3 項目を、当てた値で置き換えた写しを返す"""
    out = np.array(obs, dtype=float)
    a, b = vp.door_slices["door_angle_lidar"]
    out[a:b] = [door17[0], 1.0, vp.angle_age]
    a, b = vp.door_slices["door_frame_loc"]
    out[a:b] = door17[1:10]
    a, b = vp.door_slices["door_geometry"]
    out[a:b] = door17[10:17]
    return out


def door_from_obs(vp: VisionParams, obs: np.ndarray) -> np.ndarray:
    """観測 151 に差し込んだ扉の値 17 を取り出す（記録・表示用。鏡写しの側）"""
    parts = [obs[slice(*vp.door_slices[t])] for t in DOOR_TERMS]
    return np.concatenate([parts[0][:1], parts[1], parts[2]])


def door_real(p: dc.Params, vp: VisionParams, obs: np.ndarray) -> Tuple[float, np.ndarray]:
    """観測 151 の扉の値を実の向きへ戻す（表示用）: (扉の角度 rad, pelvis から見た開口の中心 xyz [m])"""
    x = obs
    if p.mirror:  # 鏡写しは 2 回で戻る（export_door_policy.py が確かめている）ので同じ写像で戻せる
        x = p.obs_mirror_M @ obs + p.obs_mirror_b
    a, _ = vp.door_slices["door_angle_lidar"]
    f, _ = vp.door_slices["door_frame_loc"]
    return float(x[a]), np.array(x[f:f + 3])


@dataclass(frozen=True)
class VisionActor:
    """door_core.step の policy。周期ごとに今の特徴と頭の重力（鏡写しの側）で作る"""

    vp: VisionParams
    feat: np.ndarray  # [feat_dim]
    hg_m: np.ndarray  # [3]

    def _lstm_in(self, obs: np.ndarray) -> np.ndarray:
        """[正規化した観測 151, 特徴, 頭の重力]（LSTM と追う段の入力）"""
        vp = self.vp
        x = np.clip((obs - vp.t_mean) / np.sqrt(vp.t_var + 1e-5), -5.0, 5.0)
        return np.concatenate([x, self.feat, self.hg_m])

    def prepare(self, obs: np.ndarray, state=None):
        """扉の値を当てて観測に差し込む → (観測, 追う段の次の状態)。追う段が無ければ状態はそのまま"""
        vp = self.vp
        head = door_values(vp, obs, self.feat, self.hg_m)
        obs_head = insert_door(vp, obs, head)
        if vp.tracker is None:
            return obs_head, state
        hidden = vp.tracker[1].shape[1]
        h, c = state if state is not None else (np.zeros((1, hidden)), np.zeros((1, hidden)))
        out, h, c = dc.lstm_mlp((vp.tracker,), lambda x: x, self._lstm_in(obs_head), h, c)
        door = head + (vp.tracker_out[0] @ out + vp.tracker_out[1]) * vp.door_tol
        return insert_door(vp, obs, door), (h, c)

    def forward(self, obs: np.ndarray, h: np.ndarray, cell: np.ndarray):
        return dc.lstm_mlp(self.vp.lstm, self.vp.actor_mlp, self._lstm_in(obs), h, cell)
