"""パッチ 0089（11-02・回が終わった理由を記録に残す・選んだ台だけを動画にする）の試験。
Isaac Sim は起こさない（扉の環境から関数を取り出し、偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 「途中の打ち切り」は、評価の記録に終わった理由が無く、10-30 から消去法で「上半身の関節の速さ 20 rad/s 越え」と見ていた。
  記録するときだけ、終わった台に理由のビット・段階・上半身の関節の速さの最大とその関節を残す（判定は変えない）
- simulator.config.render_env_ids で、その台の動画だけを書く（256 台の評価のまま、失敗した台だけを動画にする）。
  Isaac Lab 3.0 のカメラは台の数だけ要るので、描くのは全部の台のまま
"""
from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

ENV = Path("gr00t/rl/envs/door/door_open_homie.py")
RECORDER = Path("gr00t/rl/utils/state_recorder.py")
SUBSET = Path("gr00t/rl/utils/render_subset.py")

TERM = {
    "TERM_OVERTIME": 1,
    "TERM_TIMEOUT": 2,
    "TERM_BASE": 4,
    "TERM_FAR": 8,
    "TERM_OVERSPEED": 16,
    "TERM_HOLD_OPEN": 32,
}
UPPER = [15, 16, 17, 18]  # 上半身（指を除く）の dof の番号（偽）
HOLD_OPEN_DEG = 90.0


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _door_class():
    """扉の環境の _check_termination・_note_term_reason と TERM_* を、偽の親の下のクラスとして組み直す。
    親の _check_termination は、偽の環境の base_reset_in を reset_buf に写すだけ。reset_buf は本物（base_task.py）と同じく
    整数（0/1）の入れ物にする（真偽で試すと、整数のまま台を選ぶ誤りが見えない。11-02 に実物で 0 番と 1 番の台にだけ書かれた）"""
    tree = ast.parse(ENV.read_text(encoding="utf-8"))
    door_cls = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.ClassDef)
        and any(isinstance(m, ast.FunctionDef) and m.name == "_note_term_reason" for m in n.body)
    )
    body = []
    for node in door_cls.body:
        if isinstance(node, ast.FunctionDef) and node.name in ("_check_termination", "_note_term_reason"):
            node.decorator_list = []
            body.append(node)
        elif isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id.startswith("TERM_") for t in node.targets
        ):
            body.append(node)
    src = ast.unparse(ast.Module(body=body, type_ignores=[]))
    module_src = (
        "class _Parent:\n"
        "    def _check_termination(self):\n"
        "        self.reset_buf = self.base_reset_in.long()\n"
        "\n"
        "class Door(_Parent):\n" + "\n".join("    " + line for line in src.splitlines()) + "\n"
    )
    namespace = {"torch": torch}
    exec(compile(module_src, str(ENV), "exec"), namespace)
    return namespace["Door"]


def _env(*, recording: bool, n: int = 7) -> object:
    """台 0: 終わらない・1: 関節の速さ・2: 扉が 90°・3: 段階の持ち時間切れ・4: 完了・5: 倒れ・6: 扉から 4 m"""
    env = _door_class()()
    env.device = "cpu"
    env.num_envs = n
    env.num_stages = 6
    env.config = {"door_hold_open_angle_deg": HOLD_OPEN_DEG}
    env.termination_level = torch.tensor(1.0)
    env.episode_length_buf = torch.full((n,), 100)
    env.stage_buf = torch.tensor([0, 4, 5, 2, 5, 0, 4])
    env.time_in_stage_buf = torch.tensor([10, 10, 10, 200, 10, 10, 10])
    env.max_stage_time = torch.tensor([500, 200, 200, 200, 600, 400])
    env.stage_complete_cond_func = lambda: torch.tensor([False, False, False, False, True, False, False])
    env.time_out_buf = torch.tensor([False, False, False, False, True, False, False])
    # 親（土台と段階）が立てる打ち切り: 持ち時間切れ・完了・倒れ
    env.base_reset_in = torch.tensor([False, False, False, True, True, True, False])
    rel = torch.zeros(n, 3)
    rel[6, 0] = 4.5
    env.relative_door_pos_buf = rel
    dof_vel = torch.zeros(n, 29)
    dof_vel[:, 16] = 3.0
    dof_vel[1, 17] = -25.0  # 台 1 だけ 20 rad/s 越え（向きは負）
    hinge = torch.zeros(n, 3)
    hinge[2, 0] = 91.0 * torch.pi / 180.0
    door = SimpleNamespace(data=SimpleNamespace(joint_pos=SimpleNamespace(torch=hinge)))
    env.simulator = SimpleNamespace(dof_vel=dof_vel, scene=SimpleNamespace(articulations={"door": door}))
    env._upper_non_finger_dof_idx = UPPER
    env.term_reason_buf = torch.full((n,), -1.0)
    env.term_stage_buf = torch.full((n,), -1.0)
    env.term_upper_vel_buf = torch.zeros(n)
    env.term_upper_joint_buf = torch.full((n,), -1.0)
    env.term_count_buf = torch.zeros(n)
    env.state_recorder = object() if recording else None
    return env


def test_reasons_are_split_per_condition():
    env = _env(recording=True)
    env._check_termination()
    assert env.term_reason_buf.tolist() == [-1.0, 16.0, 32.0, 1.0, 2.0, 4.0, 8.0]
    assert env.term_stage_buf.tolist() == [-1.0, 4.0, 5.0, 2.0, 5.0, 0.0, 4.0]
    assert env.term_count_buf.tolist() == [0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


def test_overspeed_keeps_the_fastest_joint_and_its_speed():
    env = _env(recording=True)
    env._check_termination()
    assert env.term_upper_vel_buf[1].item() == pytest.approx(25.0)
    assert env.term_upper_joint_buf[1].item() == 17.0
    # 速くない台は、その歩でいちばん速い関節（3 rad/s）を残す
    assert env.term_upper_vel_buf[2].item() == pytest.approx(3.0)
    assert env.term_upper_joint_buf[2].item() == 16.0


def test_values_carry_over_until_the_next_end():
    env = _env(recording=True)
    env._check_termination()
    # 次の歩: 台 1 は戻された後で何も起きない。台 0 が倒れる
    env.base_reset_in = torch.tensor([True, False, False, False, False, False, False])
    env.time_out_buf = torch.zeros(7, dtype=torch.bool)
    env.time_in_stage_buf = torch.zeros(7, dtype=torch.long)
    env.stage_complete_cond_func = lambda: torch.zeros(7, dtype=torch.bool)
    env.simulator.dof_vel[1, 17] = 0.0
    env.simulator.scene.articulations["door"].data.joint_pos.torch[2, 0] = 0.0
    env.relative_door_pos_buf[6, 0] = 0.0
    env._check_termination()
    assert env.term_reason_buf.tolist() == [4.0, 16.0, 32.0, 1.0, 2.0, 4.0, 8.0]
    assert env.term_count_buf.tolist() == [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


@pytest.mark.parametrize("recording", [False, True])
def test_reset_is_the_same_with_and_without_recording(recording):
    env = _env(recording=recording)
    env._check_termination()
    assert env.reset_buf.dtype == torch.long
    assert env.reset_buf.bool().tolist() == [False, True, True, True, True, True, True]


def test_nothing_is_noted_without_the_recorder():
    env = _env(recording=False)
    env._check_termination()
    assert env.term_reason_buf.tolist() == [-1.0] * 7
    assert env.term_count_buf.tolist() == [0.0] * 7


def test_recorder_writes_reasons_and_the_current_fastest_joint():
    recorder = _load(RECORDER, "state_recorder_0089")
    env = _env(recording=True)
    env._check_termination()
    row = recorder.StateRecorder._term_reason(env, 3)
    assert row["term_reason"].tolist() == [-1.0, 16.0, 32.0]
    assert row["term_upper_joint"].tolist() == [-1.0, 17.0, 16.0]
    assert row["upper_vel_max"].tolist() == pytest.approx([3.0, 25.0, 3.0])
    assert row["upper_vel_joint"].tolist() == [16.0, 17.0, 16.0]
    assert set(row) == {"term_reason", "term_stage", "term_upper_vel", "term_upper_joint", "term_count",
                        "upper_vel_max", "upper_vel_joint"}


def test_render_all_envs_by_default():
    subset = _load(SUBSET, "render_subset_0089")
    assert subset.render_view_env_ids(None, 4) == [0, 1, 2, 3]
    assert subset.render_view_env_ids([], 4) == [0, 1, 2, 3]


def test_render_subset_is_sorted_and_unique():
    subset = _load(SUBSET, "render_subset_0089")
    assert subset.render_view_env_ids([12, 3, 3], 256) == [3, 12]


def test_render_subset_rejects_ids_outside_the_envs():
    subset = _load(SUBSET, "render_subset_0089")
    with pytest.raises(ValueError, match="256"):
        subset.render_view_env_ids([3, 256], 256)

