#!/usr/bin/env python3
"""G1 上でマイクを常時受信し、音声区間をベクトル化して登録済みツールを呼ぶ。"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import queue
import signal
import socket
import sys
import threading
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import soundfile as sf

from Voice.audio_input import AudioClip, load_audio
from Voice.embedding_gemma import DEFAULT_MODEL, EmbeddingGemma
from Voice.g1_mic import MIC_GROUP, MIC_PORT, decode_pcm, interface_ip, open_mic
from Voice.pipeline import encode_references
from Voice.segmenter import EnergySegmenter, Segment, SegmentConfig
from Voice.task_router import TaskRouter, load_catalog
from Voice.tool_client import DEFAULT_SOCKET, ToolClient
from Voice.tool_runtime import ToolRuntime, default_backends, load_tools

HERE = Path(__file__).resolve().parent


class VoiceController:
    def __init__(self, encoder: EmbeddingGemma, router: TaskRouter, runtime: ToolRuntime,
                 output_dir: Path, client: ToolClient | None = None, *, max_age_s: float = 5.0,
                 cooldown_s: float = 2.0, clock=time.monotonic, enabled=lambda: True) -> None:
        self.encoder, self.router, self.runtime = encoder, router, runtime
        self.output_dir, self.client = output_dir, client
        self.max_age_s, self.cooldown_s, self.clock, self.enabled = max_age_s, cooldown_s, clock, enabled
        self.last_execution: dict[str, float] = {}
        for task in router.catalog.tasks:
            if task.action is not None:
                runtime.validate(task.action, task.arguments)

    def fresh(self, segment: Segment) -> bool:
        return self.enabled() and 0 <= self.clock() - segment.ended_at <= self.max_age_s

    def process(self, segment: Segment) -> dict:
        if not self.fresh(segment):
            print("[voice] 音声区間を見送りました: 期限切れ、または待受停止", flush=True)
            return {"status": "expired"}
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ_") + uuid4().hex[:8]
        folder = self.output_dir / run_id
        folder.mkdir(parents=True)
        record = {"started_at": datetime.now(timezone.utc).isoformat(), "model": self.encoder.model_name,
                  "model_revision": self.encoder.revision, "catalog": asdict(self.router.catalog)}
        try:
            sf.write(folder / "input.wav", segment.samples, 16000)
            vector = self.encoder.encode_audio(AudioClip(segment.samples))
            np.save(folder / "audio_embedding.npy", vector, allow_pickle=False)
            decision = self.router.route(vector)
            record["decision"] = asdict(decision)
            if not self.fresh(segment):
                record["status"] = "expired"
            elif decision.task_id is None:
                record["status"] = "rejected"
            else:
                task = next(t for t in self.router.catalog.tasks if t.task_id == decision.task_id)
                if self.clock() - self.last_execution.get(task.action, float("-inf")) < self.cooldown_s:
                    record["status"] = "cooldown"
                else:
                    self.last_execution[task.action] = self.clock()
                    if self.client is None:
                        result = asdict(self.runtime.call(task.action, task.arguments))
                    else:
                        result = self.client.call(task.action, task.arguments, execute=True,
                                                  max_age_s=max(0.001, self.max_age_s - (self.clock() - segment.ended_at)))
                    record["tool_result"], record["status"] = result, result["status"]
            print("[voice] " + json.dumps({k: record[k] for k in ("status", "decision")}, ensure_ascii=False), flush=True)
            return record
        except Exception as exc:
            record["status"], record["error"] = "error", str(exc)
            raise
        finally:
            record["finished_at"] = datetime.now(timezone.utc).isoformat()
            (folder / "result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")


class ContinuousListener:
    def __init__(self, controller: VoiceController, segmenter: EnergySegmenter, health_client: ToolClient | None = None,
                 *, packet_timeout_s: float = 1.0, processing_timeout_s: float = 30.0) -> None:
        self.controller, self.segmenter, self.health_client = controller, segmenter, health_client
        self.packet_timeout_s, self.processing_timeout_s = packet_timeout_s, processing_timeout_s
        self.pending: queue.Queue[Segment] = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.capture_done = threading.Event()
        self.last_packet = self.worker_seen = float("-inf")
        self.busy_until = 0.0
        self.failure: Exception | None = None
        self.session = uuid4().hex
        self.worker = threading.Thread(target=self._worker, name="voice-inference", daemon=True)
        self.reporter = threading.Thread(target=self._report, name="voice-health-reporter", daemon=True)
        self.controller.enabled = lambda: not self.stop.is_set() and (
            (self.capture_done.is_set() and controller.client is None)
            or time.monotonic() - self.last_packet < self.packet_timeout_s)

    def start(self) -> None:
        self.worker.start()
        self.reporter.start()

    def feed(self, samples: np.ndarray, now: float) -> None:
        # UDP は複数フレームをまとめて運ぶ。現在のパケットが含む音声時間を
        # 差し引き、実際に音声が欠けた時間だけを受信途切れとして判定する。
        packet_duration = len(samples) / self.segmenter.config.sample_rate
        if now - self.last_packet - packet_duration > 0.1:
            self.segmenter.reset()
            while not self.pending.empty():
                try:
                    self.pending.get_nowait()
                except queue.Empty:
                    break
        self.last_packet = now
        for segment in self.segmenter.feed(samples, now):
            try:
                self.pending.put_nowait(segment)
            except queue.Full:
                try:
                    self.pending.get_nowait()
                except queue.Empty:
                    pass
                self.pending.put_nowait(segment)

    def healthy(self, now: float) -> bool:
        worker_progress = now < self.busy_until if self.busy_until else now - self.worker_seen < 1.0
        return not self.stop.is_set() and self.worker.is_alive() and now - self.last_packet < self.packet_timeout_s and worker_progress

    def _worker(self) -> None:
        while not self.stop.is_set():
            self.worker_seen = time.monotonic()
            try:
                segment = self.pending.get(timeout=0.1)
            except queue.Empty:
                if self.capture_done.is_set():
                    return
                continue
            self.busy_until = time.monotonic() + self.processing_timeout_s
            try:
                self.controller.process(segment)
            except Exception as exc:
                self.failure = exc
                self.stop.set()
            finally:
                self.busy_until = 0.0

    def _report(self) -> None:
        while not self.stop.wait(0.2):
            if self.health_client is not None:
                try:
                    response = self.health_client.health(self.session, self.healthy(time.monotonic()))
                    if response.get("success") is not True:
                        raise RuntimeError("ツールサーバーが生存報告を受理しませんでした")
                except Exception as exc:
                    self.failure = exc
                    self.stop.set()

    def close(self) -> None:
        self.stop.set()
        self.reporter.join(timeout=1)
        if self.health_client is not None:
            try:
                self.health_client.health(self.session, False)
            except Exception:
                pass
        self.worker.join(timeout=1)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=HERE / "configs/tasks.patrol.json")
    parser.add_argument("--tools", type=Path, default=HERE / "configs/tools.json")
    parser.add_argument("--segment-config", type=Path, default=HERE / "configs/segment.json")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--allow-download", action="store_true", help="未取得モデルのネット取得を許可（既定はローカルのみ）")
    parser.add_argument("--torch-threads", type=int, default=2)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--local-ip")
    parser.add_argument("--group", default=MIC_GROUP)
    parser.add_argument("--port", type=int, default=MIC_PORT)
    parser.add_argument("--socket", type=Path, default=DEFAULT_SOCKET)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--replay", type=Path, help="WAV を実時間で再生して常時入力を検証（実行不可）")
    parser.add_argument("--max-age", type=float, default=5.0)
    parser.add_argument("--processing-timeout", type=float, default=30.0)
    parser.add_argument("--cooldown", type=float, default=2.0)
    parser.add_argument("--output-dir", type=Path, default=HERE.parent / "_local/voice/onboard-runs")
    args = parser.parse_args(argv)
    if args.replay and args.execute:
        parser.error("--replay は dry-run 専用です")
    if args.torch_threads < 1 or any(not math.isfinite(v) or v <= 0 for v in (args.max_age, args.processing_timeout, args.cooldown)):
        parser.error("スレッド数と各期限は正の有限値にしてください")
    config = SegmentConfig(**json.loads(args.segment_config.read_text()))
    runtime = ToolRuntime(load_tools(args.tools), default_backends())
    catalog = load_catalog(args.config)
    for task in catalog.tasks:
        if task.action is not None:
            runtime.validate(task.action, task.arguments)
    client = None
    if args.execute:
        client = ToolClient(args.socket, timeout_s=max(spec.timeout_s for spec in runtime.specs.values()) + 3)
        remote = client.request("list")
        if remote.get("execute_enabled") is not True or remote.get("tools") != runtime.describe():
            raise ValueError("実行サーバーのモード・ツール定義が一致しません")
    lock_path = args.socket.parent / "listener.lock"
    lock_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with lock_path.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        import torch
        torch.set_num_threads(args.torch_threads)
        print("[voice] モデルと登録例を読み込みます", flush=True)
        encoder = EmbeddingGemma(args.model, args.device, local_files_only=not args.allow_download)
        router = TaskRouter(catalog, encode_references(encoder, catalog))
        controller = VoiceController(encoder, router, runtime, args.output_dir.expanduser(), client,
                                     max_age_s=args.max_age, cooldown_s=args.cooldown)
        listener = ContinuousListener(controller, EnergySegmenter(config),
                                      ToolClient(args.socket, timeout_s=0.5) if client else None,
                                      processing_timeout_s=args.processing_timeout)
        listener.start()
        try:
            if args.replay:
                clip = load_audio(args.replay)
                # ファイル末尾に無音が無い場合も終了判定まで流す。
                samples = np.concatenate((clip.samples, np.zeros(config.sample_rate * config.end_ms // 1000)))
                frame_size = listener.segmenter.frame_samples
                for start in range(0, len(samples), frame_size):
                    if listener.stop.is_set():
                        break
                    listener.feed(samples[start:start + frame_size], time.monotonic())
                    time.sleep(config.frame_ms / 1000)
                listener.capture_done.set()
                listener.worker.join(timeout=args.processing_timeout)
                if listener.worker.is_alive():
                    raise RuntimeError("音声処理が期限内に完了しませんでした")
            else:
                local_ip = args.local_ip or interface_ip(args.interface)
                with open_mic(local_ip, args.group, args.port) as mic:
                    print(f"[voice] G1 マイク待受: {args.group}:{args.port} via {local_ip}", flush=True)
                    last_display = 0.0
                    while not listener.stop.is_set():
                        try:
                            data, _ = mic.recvfrom(65536)
                            listener.feed(decode_pcm(data), time.monotonic())
                        except socket.timeout:
                            if time.monotonic() - listener.last_packet > listener.packet_timeout_s:
                                listener.segmenter.reset()
                        except ValueError as exc:
                            listener.segmenter.reset()
                            print(f"[voice] 入力を破棄: {exc}", file=sys.stderr)
                        if time.monotonic() - last_display > 2:
                            last_display = time.monotonic()
                            print(f"[voice] 音量={listener.segmenter.last_dbfs:.1f} dBFS healthy={listener.healthy(last_display)}", flush=True)
            if listener.failure:
                raise listener.failure
        finally:
            listener.close()
    return 0


if __name__ == "__main__":
    def stop_on_signal(signum: int, frame: object) -> None:
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_on_signal)
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (ValueError, RuntimeError, OSError, ImportError) as error:
        print(f"[voice] エラー: {error}", file=sys.stderr)
        raise SystemExit(2)
