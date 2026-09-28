"""公式モデルの読み込みと、頭カメラ追加の確認（タスク0）。"""

from __future__ import annotations

import unittest

import numpy as np

from common.config import load_config
from common.robot_model import JOINT_NAMES, NUM_MOTORS, load_model, rpy_to_quat_wxyz


class TestRobotModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import mujoco

        cls.mujoco = mujoco
        cls.cfg = load_config("robot.yaml")
        cls.model = load_model(cls.cfg, fixed_base=True)

    def test_joint_order_matches_motor_index(self) -> None:
        """胴体固定時の関節が29個で、並びが motor_cmd の番号と一致する。"""
        m = self.model
        self.assertEqual(m.njnt, NUM_MOTORS)
        self.assertEqual(m.nq, NUM_MOTORS)
        names = tuple(self.mujoco.mj_id2name(m, self.mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt))
        self.assertEqual(names, JOINT_NAMES)
        # アクチュエータ（motor）の並びも同じ
        act = tuple(m.joint(m.actuator_trnid[i, 0]).name for i in range(m.nu))
        self.assertEqual(act, JOINT_NAMES)

    def test_floating_base_kept_when_not_fixed(self) -> None:
        m = load_model(self.cfg, fixed_base=False)
        self.assertEqual(m.nq, NUM_MOTORS + 7)

    def test_head_camera_pose_in_torso(self) -> None:
        """頭カメラが torso_link 基準で設定値の位置にあり、光軸が link の x 軸（前方）を向く。"""
        mj = self.mujoco
        m = self.model
        d = mj.MjData(m)
        mj.mj_forward(m, d)
        torso = m.body("torso_link").id
        cam = m.camera(self.cfg["head_camera"]["name"]).id

        r_torso = d.xmat[torso].reshape(3, 3)
        p_rel = r_torso.T @ (d.cam_xpos[cam] - d.xpos[torso])
        np.testing.assert_allclose(p_rel, self.cfg["head_camera"]["xyz"], atol=1e-9)

        # 期待値: link の x 軸を RPY で回したもの。MuJoCo のカメラは -z 方向を見る。
        q = rpy_to_quat_wxyz(*self.cfg["head_camera"]["rpy"])
        r_link = np.zeros(9)
        mj.mju_quat2Mat(r_link, q)
        expected_forward = r_link.reshape(3, 3)[:, 0]
        r_cam_rel = r_torso.T @ d.cam_xmat[cam].reshape(3, 3)
        np.testing.assert_allclose(-r_cam_rel[:, 2], expected_forward, atol=1e-9)
        # 下向き（pitch 約 47.6°）なので、光軸の z 成分は負
        self.assertLess(expected_forward[2], -0.7)

    def test_head_camera_renders(self) -> None:
        """頭カメラで画像を描画できる（描画環境が無い場合はスキップ）。"""
        mj = self.mujoco
        cam_cfg = self.cfg["head_camera"]
        try:
            renderer = mj.Renderer(self.model, cam_cfg["height"], cam_cfg["width"])
        except Exception as e:  # noqa: BLE001  描画環境（OpenGL）が無いマシンでは失敗する
            self.skipTest(f"描画環境が無い: {e}")
        d = mj.MjData(self.model)
        mj.mj_forward(self.model, d)
        renderer.update_scene(d, camera=cam_cfg["name"])
        img = renderer.render()
        self.assertEqual(img.shape, (cam_cfg["height"], cam_cfg["width"], 3))
        renderer.close()


if __name__ == "__main__":
    unittest.main()
