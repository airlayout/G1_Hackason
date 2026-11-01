"""URDFの順運動学。学習入力の手首姿勢を求め、SDK順の関節を名前で対応させる。"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .config import finite_array


def rotation(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)


class WristKinematics:
    def __init__(self, urdf: Path, names: list[str], base: str, tips: list[str]):
        root = ET.parse(urdf).getroot()
        joints = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
        self.names = names
        self.chains = []
        for tip in tips:
            chain = []
            current = tip
            while current != base:
                if current not in joints:
                    raise ValueError(f"URDFで{base}から{tip}へ辿れません")
                joint = joints[current]
                origin = joint.find("origin")
                xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
                rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
                transform = np.eye(4)
                transform[:3, 3] = xyz
                transform[:3, :3] = (
                    rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
                )
                axis = joint.find("axis")
                if joint.get("type") not in {"fixed", "revolute", "continuous"}:
                    raise ValueError("手首のchainには回転関節と固定関節だけを使います")
                index = None if joint.get("type") == "fixed" else names.index(joint.get("name"))
                direction = np.fromstring(axis.get("xyz") if axis is not None else "1 0 0", sep=" ")
                chain.append((transform, index, direction))
                current = joint.find("parent").get("link")
            self.chains.append(chain[::-1])

    def poses(self, positions) -> list[np.ndarray]:
        q = finite_array(positions, (29,), "q")
        result = []
        for chain in self.chains:
            pose = np.eye(4)
            for origin, index, axis in chain:
                pose = pose @ origin
                if index is not None:
                    turn = np.eye(4)
                    turn[:3, :3] = rotation(axis, q[index])
                    pose = pose @ turn
            # GR00TのPose形式: 回転行列の最初の2行（列ではない）。
            result.append(np.concatenate([pose[:3, 3], pose[:2, :3].reshape(-1)]).astype(np.float32))
        return result
