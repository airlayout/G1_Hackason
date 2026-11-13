from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import soundfile as sf

from Voice.task_actions import ActionResult
from Voice.task_router import Task, TaskCatalog
from Voice.voice_command import encode_references, main


class VoiceCommandTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.audio = self.root / "input.wav"
        sf.write(self.audio, np.full(4_000, 0.1), 16_000)
        self.config = self.root / "tasks.json"
        self.config.write_text(json.dumps({
            "min_similarity": 0.5, "min_margin": 0.08,
            "tasks": [
                {"id": "bottle", "action": "press_bottle", "descriptions": ["ボトルを押す"]},
                {"id": "down", "action": "press_down", "descriptions": ["下ボタンを押す"]},
                {"id": "none", "action": None, "descriptions": ["待つ"]},
            ],
        }), encoding="utf-8")
        self.args = ["--audio", str(self.audio), "--config", str(self.config), "--output-dir", str(self.root / "runs")]
        self.encoder = Mock(device="cpu", revision="test")
        self.encoder.encode_audio.return_value = np.array([1, 0, 0], dtype=np.float32)
        self.encoder.encode_documents.return_value = np.eye(3)
        self.addCleanup(patch.stopall)
        self.backend = patch("Voice.voice_command.EmbeddingGemma", return_value=self.encoder).start()
        self.stdout = io.StringIO()
        self.stderr = io.StringIO()

    def invoke(self, args: list[str]) -> int:
        with contextlib.redirect_stdout(self.stdout), contextlib.redirect_stderr(self.stderr):
            return main(args)

    def result(self) -> dict:
        return json.loads(next((self.root / "runs").glob("*/result.json")).read_text())

    def test_default_dry_run_exports_vector_and_never_starts_robot(self) -> None:
        with patch("Voice.task_actions.subprocess.Popen") as spawn:
            self.assertEqual(self.invoke(self.args), 0)
        spawn.assert_not_called()
        result = self.result()
        self.assertEqual(result["decision"]["task_id"], "bottle")
        self.assertEqual(result["status"], "dry_run")
        np.testing.assert_array_equal(np.load(result["embedding_path"], allow_pickle=False), [1, 0, 0])

    def test_embed_only_does_not_load_task_config_or_compare_documents(self) -> None:
        self.config.unlink()
        with patch("Voice.voice_command.run_action") as action:
            self.assertEqual(self.invoke([*self.args, "--embed-only"]), 0)
        self.encoder.encode_documents.assert_not_called()
        action.assert_not_called()
        self.assertEqual(self.result()["status"], "embedded")

    def test_ambiguous_input_never_executes_even_with_execute_flag(self) -> None:
        self.encoder.encode_audio.return_value = np.array([1, 1, 0], dtype=np.float32)
        with patch("Voice.voice_command.run_action") as action:
            self.assertEqual(self.invoke([*self.args, "--execute"]), 3)
        action.assert_not_called()
        self.assertEqual(self.result()["decision"]["reason"], "ambiguous")

    def test_explicit_execute_passes_selected_action_to_control_environment(self) -> None:
        with patch("Voice.voice_command.run_action", return_value=ActionResult(("test",), "process_exited", 7)) as action:
            self.assertEqual(self.invoke([*self.args, "--execute", "--robot-python", "/robot/python"]), 4)
        self.assertEqual(action.call_args.args[:2], ("press_bottle", Path("/robot/python")))
        self.assertTrue(action.call_args.kwargs["execute"])
        self.assertEqual(self.result()["action"]["returncode"], 7)

    def test_silence_is_rejected_before_model_loading(self) -> None:
        sf.write(self.audio, np.zeros(4_000), 16_000)
        self.assertEqual(self.invoke(self.args), 2)
        self.backend.assert_not_called()

    def test_model_failure_is_recorded(self) -> None:
        self.encoder.encode_audio.side_effect = RuntimeError("model failed")
        self.assertEqual(self.invoke(self.args), 2)
        self.assertEqual(self.result()["status"], "error")
        self.assertIn("model failed", self.result()["error"])

    def test_list_tasks_does_not_load_model(self) -> None:
        self.assertEqual(self.invoke(["--list-tasks", "--config", str(self.config)]), 0)
        self.backend.assert_not_called()
        self.assertIn("bottle", self.stdout.getvalue())

    def test_registered_audio_replaces_text_for_that_task(self) -> None:
        catalog = TaskCatalog((
            Task("bottle", ("説明文1", "説明文2"), "press_bottle", (str(self.audio),)),
            Task("none", ("待つ",), None),
        ), 0.75, 0.08)
        self.encoder.encode_documents.return_value = np.array([[0, 1, 0]])
        vectors = encode_references(self.encoder, catalog)
        self.assertEqual(vectors.shape, (2, 3))
        np.testing.assert_array_equal(vectors[0], [1, 0, 0])
        np.testing.assert_array_equal(vectors[1], [0, 1, 0])
        self.encoder.encode_documents.assert_called_once_with(["待つ"])

    def test_save_audio_writes_normalized_wav_without_overwriting(self) -> None:
        saved = self.root / "reference.wav"
        args = [*self.args, "--embed-only", "--save-audio", str(saved)]
        self.assertEqual(self.invoke(args), 0)
        samples, rate = sf.read(saved)
        self.assertEqual(rate, 16_000)
        self.assertEqual(samples.shape, (4_000,))
        self.assertEqual(self.invoke(args), 2)

    def test_embedding_only_cannot_execute(self) -> None:
        with self.assertRaises(SystemExit) as error:
            self.invoke([*self.args, "--embed-only", "--execute"])
        self.assertEqual(error.exception.code, 2)
        self.backend.assert_not_called()

    def test_generic_tools_dry_run_does_not_use_legacy_action_or_spawn(self) -> None:
        tools = self.root / "tools.json"
        tools.write_text(json.dumps({"tools": [{
            "name": "press_bottle", "backend": "command",
            "config": {"argv": ["/does/not/exist"]},
            "input_schema": {"type": "object", "additionalProperties": False},
        }, {
            "name": "press_down", "backend": "command",
            "config": {"argv": ["/does/not/exist"]},
            "input_schema": {"type": "object", "additionalProperties": False},
        }]}))
        with patch("Voice.voice_command.run_action") as legacy, patch("Voice.tool_runtime.subprocess.Popen") as spawn:
            self.assertEqual(self.invoke([*self.args, "--tools", str(tools)]), 0)
        legacy.assert_not_called()
        spawn.assert_not_called()
        self.assertEqual(self.result()["tool_result"]["status"], "dry_run")
