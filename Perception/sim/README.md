# Perception / sim

MuJoCoシミュレーション上での検証コード。

## 前提

MuJoCoシム側で、`head_camera`をZMQ配信する状態になっていること。
`G1_HuggingFace/README.md`の`lerobot-teleoperate`の例のように、
`--robot.cameras='{"...": {"type": "zmq", "server_address": "localhost", "port": 5555, "camera_name": "head_camera", ...}}'`
を付けて起動すればよい。

## 実行

```bash
source ../../G1_HuggingFace/venv/bin/activate  # 未activateの場合
pip install -r ../requirements.txt             # 未インストールの場合

python run_sim.py
# 別のconfigを使う場合: python run_sim.py --config configs/config.yaml
# フレーム数を絞って動作確認する場合: python run_sim.py --max-frames 30
```

検出結果は`outputs/`にJSON Lines(`run.jsonl`)・CSV(`run.csv`)として出力される
（`.gitignore`で追跡対象外）。
