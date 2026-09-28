"""実機用の LowCmd_ の組み立て（common/arm/backend_dds.py の fill_lowcmd）を確かめる（タスク1）。

DDS には接続しない。unitree_sdk2py が入っていない環境（CI など）ではスキップする。
"""

from __future__ import annotations

import unittest

import numpy as np

from common.arm.backend_dds import active_joints, fill_lowcmd
from common.arm.types import JointCommand
from common.robot_model import ARM_SDK_WEIGHT_IDX, NUM_MOTORS

try:
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    HAVE_SDK = True
except ImportError:
    HAVE_SDK = False


def arm_command(weight: float) -> JointCommand:
    c = JointCommand()
    c.q[:] = np.linspace(-1, 1, NUM_MOTORS)
    c.kp[15:29] = 60.0
    c.kd[15:29] = 1.5
    c.weight = weight
    return c


@unittest.skipUnless(HAVE_SDK, "unitree_sdk2py が無い")
class TestFillLowCmd(unittest.TestCase):
    def test_arm_sdk_message(self) -> None:
        msg = unitree_hg_msg_dds__LowCmd_()
        cmd = arm_command(0.4)
        fill_lowcmd(msg, cmd, mode_machine=5, use_weight=True)
        self.assertEqual(msg.mode_machine, 5)
        self.assertEqual(msg.mode_pr, 0)
        self.assertAlmostEqual(msg.motor_cmd[ARM_SDK_WEIGHT_IDX].q, 0.4, places=6)
        for i in range(15, 29):
            self.assertEqual(msg.motor_cmd[i].mode, 1)
            self.assertAlmostEqual(msg.motor_cmd[i].q, cmd.q[i], places=6)
            self.assertAlmostEqual(msg.motor_cmd[i].kp, 60.0)
        for i in range(0, 15):  # 指令していない関節は触らない
            self.assertEqual(msg.motor_cmd[i].mode, 0)
            self.assertEqual(msg.motor_cmd[i].kp, 0.0)
        msg.crc = CRC().Crc(msg)
        self.assertNotEqual(msg.crc, 0)

    def test_weight_is_clipped(self) -> None:
        msg = unitree_hg_msg_dds__LowCmd_()
        fill_lowcmd(msg, arm_command(1.7), mode_machine=5, use_weight=True)
        self.assertAlmostEqual(msg.motor_cmd[ARM_SDK_WEIGHT_IDX].q, 1.0)

    def test_lowcmd_does_not_touch_weight_slot(self) -> None:
        msg = unitree_hg_msg_dds__LowCmd_()
        cmd = arm_command(1.0)
        cmd.kp[:] = 40.0
        fill_lowcmd(msg, cmd, mode_machine=5, use_weight=False)
        self.assertEqual(msg.motor_cmd[ARM_SDK_WEIGHT_IDX].q, 0.0)
        self.assertTrue(all(msg.motor_cmd[i].mode == 1 for i in range(NUM_MOTORS)))


class TestActiveJoints(unittest.TestCase):
    def test_only_joints_with_gain(self) -> None:
        self.assertEqual(active_joints(arm_command(1.0)), list(range(15, 29)))


if __name__ == "__main__":
    unittest.main()
