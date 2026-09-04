# Perception / real

実機G1上での実行コード。

## 前提

実機G1側で、カメラ配信付きのサーバを起動しておくこと（`G1_HuggingFace/README.md`の
「実機への接続」を参照）:

```bash
# G1本体側(SSH接続後)
cd ~/lerobot
python src/lerobot/robots/unitree_g1/run_g1_server.py --camera
```

操作PC側は`configs/config.yaml`の`source.zmq.server_address`をG1のIP
（既定`192.168.123.164`）に合わせる。ネットワーク疎通確認は`Common/network/`を参照。

## 実行

```bash
source ../../G1_HuggingFace/venv/bin/activate  # 未activateの場合
pip install -r ../requirements.txt             # 未インストールの場合

python run_real.py --server-address 192.168.123.164
# フレーム数を絞って動作確認する場合: python run_real.py --max-frames 30
```

検出結果は`outputs/`にJSON Lines(`run.jsonl`)・CSV(`run.csv`)として出力される
（`.gitignore`で追跡対象外）。
