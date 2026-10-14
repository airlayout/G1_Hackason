"""DoorMan の g1_43dof.yaml（Dex3 の指つき）から、指の無い G1 29 関節版の g1_29dof.yaml を作る。

元の DoorMan（NVlabs/GR00T-VisualSim2Real doorman @ 016c70c）の 43 要素の配列を手で切ると間違えやすいので、
先頭 29 要素（脚 12・腰 3・腕 14。並びは 43 関節版の先頭と同じ）を機械的に取り出す。

手の剛体は、29 関節の USD（g1_29dof_rev_1_0.usd）では left/right_wrist_yaw_link に統合されている
（rubber hand は剛体でない Xform）。Dex3 の palm と同じ座標にするためのオフセットも書く
（43 関節版の left_hand_palm_joint の位置 (0.0415, ±0.003, 0)、回転は単位）。

使い方:
    python make_g1_29dof_yaml.py <VIRAL>/gr00t/rl/config/robot/g1/g1_43dof.yaml \
        <VIRAL>/gr00t/rl/config/robot/g1/g1_29dof.yaml
"""

import sys
from pathlib import Path

import yaml

NUM_DOF = 29
NUM_BODIES = 30

# どこからも読まれない指の設定（2026-10-06 の洗い出し C 節）。消す。
UNUSED_FINGER_KEYS = [
    "overcurling_dof_names",
    "right_hand_dof_names",
    "left_hand_dof_names",
    "upper_left_arm_dof_names",
    "upper_right_arm_dof_names",
    "thumb_tip_link",
    "right_hand_palm_link",
    "finger_tip_links",
    "finger_dof_configuration_overcurling",
    "finger_dof_configuration_straight",
    "right_fingers_palm_body_names",
]

# 関節ごとの配列（43 要素）。先頭 29 要素に切る。
PER_DOF_LIST_KEYS = [
    "dof_pos_lower_limit_list",
    "dof_pos_upper_limit_list",
    "dof_pos_humanly_lower_limit_list",
    "dof_pos_humanly_upper_limit_list",
    "dof_vel_limit_list",
    "dof_effort_limit_list",
    "dof_armature_list",
    "dof_joint_friction_list",
]

HEADER = """# @package _global_
#
# G1 29 関節・指の動かない純正のエンドハンド（rubber hand）。
# G1_Hackason/Door/tools/make_g1_29dof_yaml.py が g1_43dof.yaml から生成した。手で直さず、生成し直すこと。
#
# - 関節は 43 関節版の先頭 29（脚 12・腰 3・腕 14）。HOMIE が見る 29 関節と同じ並び
# - 手の剛体は left/right_wrist_yaw_link（rubber hand はこの剛体に統合済み）
# - left/right_hand_palm_offset は wrist_yaw_link から見た Dex3 の palm の原点（座標の向きも同じ）
"""


def _is_hand_name(name: str) -> bool:
    return "_hand_" in name or name.endswith("_hand") or name.startswith("hand_")


def convert(src: dict) -> dict:
    robot = dict(src["robot"])

    dof_names = list(robot["dof_names"][:NUM_DOF])
    if any(_is_hand_name(n) for n in dof_names):
        raise ValueError(f"先頭 {NUM_DOF} 関節に指が混ざっている: {dof_names}")

    robot["num_bodies"] = NUM_BODIES
    robot["dof_obs_size"] = NUM_DOF
    robot["actions_dim"] = NUM_DOF
    robot["upper_body_actions_dim"] = len(robot["arm_dof_names"])
    robot["dof_names"] = dof_names
    robot["upper_dof_names"] = list(robot["arm_dof_names"])

    for key in UNUSED_FINGER_KEYS:
        robot.pop(key, None)

    for key in PER_DOF_LIST_KEYS:
        values = robot[key]
        if len(values) < NUM_DOF:
            raise ValueError(f"{key} の長さが {len(values)}（{NUM_DOF} 未満）")
        robot[key] = list(values[:NUM_DOF])

    robot["finger_primitive"] = {"num_non_primitive_dof": NUM_DOF, "primitive_action_map": {}}

    body_names = list(robot["body_names"][:NUM_BODIES])
    if body_names[-2:] != ["left_wrist_yaw_link", "right_wrist_yaw_link"]:
        raise ValueError(f"剛体の並びが想定と違う: 末尾 {body_names[-2:]}")
    robot["body_names"] = body_names
    robot["arm_body_names"] = [n for n in robot["arm_body_names"] if not _is_hand_name(n)]

    robot["left_hand_body_names"] = ["left_wrist_yaw_link"]
    robot["right_hand_body_names"] = ["right_wrist_yaw_link"]
    # 接触センサの filter の並び（gr00t/rl/utils/hand_links.py が読む）
    robot["hand_contact_body_names"] = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
    robot["left_hand_palm_side_direction"] = ["-y"]
    robot["right_hand_palm_side_direction"] = ["+y"]
    robot["left_hand_palm_body"] = "left_wrist_yaw_link"
    robot["right_hand_palm_body"] = "right_wrist_yaw_link"
    robot["left_hand_palm_offset"] = [0.0415, 0.003, 0.0]
    robot["right_hand_palm_offset"] = [0.0415, -0.003, 0.0]
    # 29 関節の USD では head_link・d435_link は torso_link の子の Xform（剛体でない）
    robot["head_frame_body"] = "torso_link"
    robot["head_frame_offset"] = [0.0039635, 0.0, -0.044]

    init_state = dict(robot["init_state"])
    init_state["default_joint_angles"] = {
        k: v for k, v in init_state["default_joint_angles"].items() if not _is_hand_name(k)
    }
    robot["init_state"] = init_state

    control = dict(robot["control"])
    for gain in ("stiffness", "damping"):
        control[gain] = {k: v for k, v in control[gain].items() if not k.startswith("hand_")}
    robot["control"] = control

    asset = dict(robot["asset"])
    asset["robot_type"] = "g1_29dof_rev_1_0"
    asset["usd_file"] = "g1/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"
    robot["asset"] = asset

    out = {k: v for k, v in src.items() if k != "robot"}
    out["robot"] = robot
    return out


def main() -> None:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src_path, dst_path = Path(sys.argv[1]), Path(sys.argv[2])
    src = yaml.safe_load(src_path.read_text(encoding="utf-8"))
    out = convert(src)
    body = yaml.safe_dump(out, allow_unicode=True, sort_keys=False, width=120)
    dst_path.write_text(HEADER + "\n" + body, encoding="utf-8")
    print(f"[make_g1_29dof] {dst_path} を書いた")


if __name__ == "__main__":
    main()
