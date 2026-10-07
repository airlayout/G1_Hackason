#!/usr/bin/env python3
"""既存のボタン押下処理を Yada の採点環境で評価する。"""

import argparse
import os
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from yada_paths import AGENT, REPO_ROOT, find_yada_root


def build_command(root, backend, evaluation_args):
    if any(a.split("=", 1)[0] in ("--agent", "--client") for a in evaluation_args):
        raise ValueError("この入口は Manipulation/push_button のエージェントを評価します")
    if backend == "dds":
        unsupported = ("--set", "--workers", "--report", "--ablation", "--ablation-only", "--trials")
        if any(a.split("=", 1)[0] in unsupported for a in evaluation_args):
            raise ValueError("DDS方式は basic のみです。realistic・並列・弱点レポートは --backend api を使ってください")
        client = shlex.join([sys.executable, str(Path(__file__).with_name("run_yada_dds.py")),
                             "--yada-root", str(root)])
        return [sys.executable, str(root / "contest/evaluate_dds.py"), "--client", client, *evaluation_args]
    return [sys.executable, str(root / "contest/evaluate.py"), "--agent", str(AGENT), *evaluation_args]


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--backend", choices=("api", "dds"), default="api")
    parser.add_argument("--yada-root", help="Yadaディレクトリ（省略時は通常配置か準備済みのスナップショット）")
    parser.add_argument("--weights", help="YOLO重み。省略時はカラー画像から検出する")
    args, evaluation_args = parser.parse_known_args()
    try:
        root = find_yada_root(args.yada_root)
        env = {**os.environ, "PYTHONUNBUFFERED": "1"}
        if args.weights:
            weights = Path(args.weights).expanduser().resolve()
            if not weights.is_file():
                raise FileNotFoundError(f"YOLO重みがありません: {weights}")
            env["G1_YADA_WEIGHTS"] = str(weights)
        else:
            env.pop("G1_YADA_WEIGHTS", None)
        if "--out" not in evaluation_args and not any(a.startswith("--out=") for a in evaluation_args):
            name = f"push_button_{args.backend}_{datetime.now():%Y%m%d_%H%M%S}.json"
            evaluation_args += ["--out", str(REPO_ROOT / "_local/button_press_yada/results" / name)]
        command = build_command(root, args.backend, evaluation_args)
        if args.backend == "dds":
            try:
                from unitree_sdk2py.utils.crc import CRC

                CRC()
            except (ImportError, OSError) as exc:
                raise ValueError(f"DDS用SDKまたはCRCライブラリがありません。公式SDKをソースから -e で導入してください: {exc}") from exc
    except (ValueError, FileNotFoundError) as exc:
        parser.error(str(exc))
    print(f"採点コード: {root}\n検出: {'YOLO' if args.weights else 'カラー画像'}", flush=True)
    code = subprocess.call(command, cwd=root.parents[1], env=env)
    if code:
        return code
    # 元の評価器はエージェントの失敗でも終了コード0を返す。自動試験では結果も検査する。
    output = next((a.split("=", 1)[1] for a in evaluation_args if a.startswith("--out=")), None)
    if output is None:
        output = evaluation_args[evaluation_args.index("--out") + 1]
    import json

    output_path = Path(output)
    if not output_path.is_absolute():
        output_path = root.parents[1] / output_path
    result = json.loads(output_path.read_text())
    return 0 if result["summary"]["trials"] > 0 and result["summary"]["success_rate"] == 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
