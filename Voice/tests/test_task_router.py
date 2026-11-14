from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from Voice.task_router import Task, TaskCatalog, TaskRouter, load_catalog


class TaskRouterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.catalog = TaskCatalog((
            Task("bottle", ("ボトルを押す", "ペットボトルを押す"), "press_bottle"),
            Task("down", ("下ボタンを押す",), "press_down"),
            Task("no_action", ("何もしない",), None),
        ), min_similarity=0.5, min_margin=0.08)
        self.vectors = np.array([[1, 0, 0], [0.99, 0.01, 0], [0, 1, 0], [0, 0, 1]])
        self.router = TaskRouter(self.catalog, self.vectors)

    def test_aliases_of_same_task_do_not_count_as_competing_tasks(self) -> None:
        decision = self.router.route(np.array([1, 0, 0]))
        self.assertEqual(decision.task_id, "bottle")
        self.assertEqual(decision.reason, "accepted")
        self.assertEqual(len(decision.candidates), 3)

    def test_close_different_tasks_are_rejected(self) -> None:
        decision = self.router.route(np.array([1, 1, 0]))
        self.assertIsNone(decision.task_id)
        self.assertEqual(decision.reason, "ambiguous")

    def test_zero_margin_accepts_highest_ranked_close_candidate(self) -> None:
        catalog = TaskCatalog(self.catalog.tasks, min_similarity=0.65, min_margin=0)
        decision = TaskRouter(catalog, self.vectors).route(np.array([1, 1, 0]))
        self.assertEqual(decision.task_id, "bottle")
        self.assertEqual(decision.reason, "accepted")

    def test_low_similarity_is_rejected(self) -> None:
        self.assertEqual(self.router.route(np.array([-1, -1, -1])).reason, "low_similarity")

    def test_no_action_winner_is_never_executable(self) -> None:
        decision = self.router.route(np.array([0, 0, 1]))
        self.assertIsNone(decision.task_id)
        self.assertEqual(decision.reason, "no_action")

    def test_magnitude_does_not_change_cosine_ranking(self) -> None:
        decision = TaskRouter(self.catalog, self.vectors * 7).route(np.array([0, 20, 0]))
        self.assertEqual(decision.task_id, "down")

    def test_invalid_embeddings_fail_closed(self) -> None:
        for vector in ([0, 0, 0], [float("nan"), 1, 0], [1, 0], [[1, 0, 0]]):
            with self.subTest(vector=vector), self.assertRaises(ValueError):
                self.router.route(np.array(vector))
        with self.assertRaises(ValueError):
            TaskRouter(self.catalog, np.zeros((4, 3)))
        with self.assertRaises(ValueError):
            TaskRouter(self.catalog, np.ones((3, 3)))


class CatalogTests(unittest.TestCase):
    def test_zero_margin_is_valid(self) -> None:
        source = Path(__file__).resolve().parents[1] / "configs/tasks.all.json"
        data = json.loads(source.read_text())
        data["min_margin"] = 0
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tasks.json"
            path.write_text(json.dumps(data))
            self.assertEqual(load_catalog(path).min_margin, 0)

    def test_default_catalog_is_valid(self) -> None:
        catalog = load_catalog(Path(__file__).resolve().parents[1] / "configs/tasks.json")
        self.assertEqual([task.task_id for task in catalog.tasks], ["press_bottle", "press_down", "no_action"])

    def test_reference_paths_are_relative_to_config(self) -> None:
        source = Path(__file__).resolve().parents[1] / "configs/tasks.json"
        data = json.loads(source.read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tasks.json"
            recording = Path(tmp) / "recording.wav"
            recording.touch()
            data["tasks"][0]["reference_audio"] = ["recording.wav"]
            path.write_text(json.dumps(data))
            catalog = load_catalog(path)
            self.assertEqual(catalog.tasks[0].reference_audio, (str(recording),))
            recording.unlink()
            with self.assertRaisesRegex(ValueError, "登録音声がありません"):
                load_catalog(path)

    def test_invalid_catalogs_are_rejected(self) -> None:
        valid = {
            "min_similarity": 0.5, "min_margin": 0.08,
            "tasks": [{"id": "one", "descriptions": ["一つ"], "action": "press_bottle"},
                      {"id": "none", "descriptions": ["待つ"], "action": None}],
        }
        cases = []
        for key, value in (("min_similarity", float("nan")), ("min_margin", -0.01), ("min_margin", True), ("tasks", [])):
            cases.append({**valid, key: value})
        cases.append({**valid, "tasks": [valid["tasks"][0], valid["tasks"][0]]})
        cases.append({**valid, "tasks": [{"id": "bad", "descriptions": [], "action": None}, valid["tasks"][1]]})
        cases.append({**valid, "tasks": [{"id": "bad", "descriptions": ["説明"]}, valid["tasks"][1]]})
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tasks.json"
            for data in cases:
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.subTest(data=data), self.assertRaises(ValueError):
                    load_catalog(path)
