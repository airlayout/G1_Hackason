"""評価の記録から、指定した台の最初の 1 回（count_success.py が数える回）だけを MuJoCo で描いて mp4 にする。
使い方: python render_first_episode.py <sim2sim> <mjcf> <記録> <出力先> <名前:台>..."""
import sys
from pathlib import Path
import mujoco, numpy as np
sys.path.insert(0, sys.argv[1])
from replay_states import VIDEO_SIZE, add_door, load_record, render_episode, split_episodes  # noqa: E402

mjcf, record, out_dir = Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4])
static, states = load_record(record)
out_dir.mkdir(parents=True, exist_ok=True)
for spec_arg in sys.argv[5:]:
    name, k = spec_arg.rsplit(":", 1); k = int(k)
    spec = mujoco.MjSpec.from_file(str(mjcf))
    add_door(spec, float(static["door_width"][k]), float(static["door_height"][k]), float(static["door_handle_height"][k]),
             float(static["door_handle_width"][k]), int(round(float(static["door_open_lr"][k]))))
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, VIDEO_SIZE[0])
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, VIDEO_SIZE[1])
    model = spec.compile(); data = mujoco.MjData(model)
    qadr = np.array([model.joint(str(n)).qposadr[0] for n in static["dof_names"]])
    s, e = split_episodes(states["episode_length"][:, k])[0]
    fps = int(round(1.0 / float(static["dt"])))
    out = out_dir / f"{name}.mp4"
    render_episode(model, data, states, k, (s, e), qadr, model.joint("door_hinge").qposadr[0], model.joint("door_handle_joint").qposadr[0],
                   out, fps, (35.0, -20.0, 4.0))
    print(f"[render] {out} steps={e - s} max_hinge={np.degrees(states['door_dof_pos'][s:e, k, 0].max()):.1f}")
