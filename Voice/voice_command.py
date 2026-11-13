#!/usr/bin/env python3
"""音声 → EmbeddingGemma 2 → タスク選択。既定はコマンド表示のみ。"""

from __future__ import annotations

import argparse
import json
import shlex
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import soundfile as sf

from Voice.audio_input import load_audio, record_audio
from Voice.embedding_gemma import DEFAULT_MODEL, EmbeddingGemma
from Voice.pipeline import encode_references
from Voice.task_actions import DEFAULT_ROBOT_PYTHON, REPO_ROOT, TASK_ARGS, build_command, run_action
from Voice.task_router import TaskRouter, load_catalog
from Voice.tool_client import DEFAULT_SOCKET, ToolClient
from Voice.tool_runtime import ToolRuntime, default_backends, load_tools

DEFAULT_CONFIG = Path(__file__).resolve().parent / "configs/tasks.json"


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--audio", type=Path, help="WAV / FLAC 等の音声ファイル")
    source.add_argument("--record-seconds", type=float, help="操作 PC のマイクを指定秒数だけ録音")
    parser.add_argument("--mic-device", help="sounddevice の入力デバイス名")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--model", default=DEFAULT_MODEL, help="EmbeddingGemma 2 のモデル ID またはローカルパス")
    parser.add_argument("--device", default="auto", help="auto / cpu / cuda:0 等")
    parser.add_argument("--embed-only", action="store_true", help="音声ベクトルの保存までで終了")
    parser.add_argument("--list-tasks", action="store_true", help="タスク定義を表示して終了（モデルの読み込みなし）")
    parser.add_argument("--execute", action="store_true", help="選んだタスクを実機で実行（既定はコマンド表示）")
    parser.add_argument("--robot-python", type=Path, default=DEFAULT_ROBOT_PYTHON)
    parser.add_argument("--network-interface", help="G1 につないでいる NIC を実行処理へ渡す")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "_local/voice/runs")
    parser.add_argument("--save-audio", type=Path, help="整形済みの入力音声を指定先の WAV に保存（既存ファイルは上書きしない）")
    parser.add_argument("--tools", type=Path, help="汎用ツール定義を使う（省略時は従来のボタン押し呼び出し）")
    parser.add_argument("--tool-socket", type=Path, help="G1 内のツールサーバーの Unix socket")
    return parser


def run(args: argparse.Namespace) -> int:
    catalog = None if args.embed_only else load_catalog(args.config)
    runtime = None
    if not args.embed_only and (args.tools or args.tool_socket):
        runtime = ToolRuntime(load_tools(args.tools or DEFAULT_CONFIG.with_name("tools.json")), default_backends())
    if catalog:
        for task in catalog.tasks:
            if task.action is not None:
                if runtime:
                    runtime.validate(task.action, task.arguments)
                elif task.action not in TASK_ARGS:
                    raise ValueError(f"{task.task_id}: 未登録の実行処理です: {task.action}")
    if args.audio:
        clip = load_audio(args.audio)
    else:
        print(f"[voice] {args.record_seconds} 秒録音します", file=sys.stderr, flush=True)
        clip = record_audio(args.record_seconds, args.mic_device)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ_") + uuid4().hex[:8]
    run_dir = args.output_dir.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True)
    record = {
        "started_at": datetime.now(timezone.utc).isoformat(),
        "model": args.model, "execute": args.execute,
        "source": str(args.audio.expanduser().resolve()) if args.audio else "microphone",
        "sample_rate": clip.sample_rate, "duration_seconds": clip.duration,
    }
    try:
        if not args.audio:
            sf.write(run_dir / "input.wav", clip.samples, clip.sample_rate)
        if args.save_audio:
            saved_audio = args.save_audio.expanduser().absolute()
            saved_audio.parent.mkdir(parents=True, exist_ok=True)
            with saved_audio.open("xb") as stream:
                sf.write(stream, clip.samples, clip.sample_rate, format="WAV")
            record["saved_audio"] = str(saved_audio)
        print(f"[voice] モデルを読み込みます: {args.model}", file=sys.stderr, flush=True)
        encoder = EmbeddingGemma(args.model, args.device)
        record["model_revision"] = encoder.revision
        record["device"] = str(encoder.device)
        vector = encoder.encode_audio(clip)
        vector_path = run_dir / "audio_embedding.npy"
        np.save(vector_path, vector, allow_pickle=False)
        record["embedding_path"] = str(vector_path)
        record["embedding_shape"] = list(vector.shape)
        print(f"[voice] 音声ベクトルを保存しました: {vector_path}", flush=True)
        if args.embed_only:
            record["status"] = "embedded"
            return 0
        document_vectors = encode_references(encoder, catalog)
        decision = TaskRouter(catalog, document_vectors).route(vector)
        record["catalog"] = asdict(catalog)
        record["decision"] = asdict(decision)
        print("[voice] 候補: " + ", ".join(f"{item.task_id}={item.score:.3f}" for item in decision.candidates))
        if decision.task_id is None:
            record["status"] = "rejected"
            print(f"[voice] 実行を見送りました: {decision.reason}")
            return 3
        action = next(task.action for task in catalog.tasks if task.task_id == decision.task_id)
        if runtime:
            task = next(task for task in catalog.tasks if task.task_id == decision.task_id)
            if args.execute:
                client = ToolClient(args.tool_socket or DEFAULT_SOCKET, timeout_s=runtime.specs[action].timeout_s + 3)
                metadata = client.request("list")
                if metadata.get("execute_enabled") is not True or metadata.get("tools") != runtime.describe():
                    raise ValueError("ツールサーバーのモード・定義が一致しません")
                result = client.call(action, task.arguments, execute=True)
            else:
                result = asdict(runtime.call(action, task.arguments))
            record["tool_result"], record["status"] = result, result["status"]
            print("[voice] ツール: " + json.dumps(result, ensure_ascii=False))
            return 0 if result.get("success") is True else 4
        command = build_command(action, args.robot_python, args.network_interface, execute=args.execute)
        record["command"] = list(command)
        print(f"[voice] 選択: {decision.task_id} ({decision.candidates[0].score:.3f})")
        print(f"[voice] {'実行' if args.execute else 'dry-run'}: {shlex.join(command)}", flush=True)
        # 起動前にも選択結果を残し、実行中に異常終了した場合に追えるようにする。
        record["status"] = "executing" if args.execute else "dry_run"
        (run_dir / "result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        result = run_action(action, args.robot_python, args.network_interface, execute=args.execute)
        record["action"] = asdict(result)
        record["status"] = result.status
        return 0 if result.returncode in (None, 0) else 4
    except KeyboardInterrupt:
        record["status"] = "interrupted"
        raise
    except Exception as exc:
        record["status"], record["error"] = "error", str(exc)
        raise
    finally:
        record["finished_at"] = datetime.now(timezone.utc).isoformat()
        result_path = run_dir / "result.json"
        result_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[voice] 記録: {result_path}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)
    if args.embed_only and args.execute:
        parser.error("--embed-only と --execute は同時に指定できません")
    if not args.list_tasks and args.audio is None and args.record_seconds is None:
        parser.error("--audio または --record-seconds を指定してください")
    try:
        if args.list_tasks:
            catalog = load_catalog(args.config)
            for task in catalog.tasks:
                print(f"{task.task_id}: {task.descriptions[0]} (action={task.action})")
            return 0
        return run(args)
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f"[voice] エラー: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
