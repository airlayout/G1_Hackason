import json
import tempfile
import unittest
from pathlib import Path

import sim_log


def body(session="abc123", records=({"t": 1},)):
    return json.dumps({"session": session, "records": list(records)}).encode()


class SimLogTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.log = sim_log.SimLog(Path(self.tmp.name) / "debug")

    def tearDown(self):
        self.tmp.cleanup()

    def files(self):
        return sorted((Path(self.tmp.name) / "debug").glob("sim-camera-*.jsonl"))

    def test_records_are_appended_as_jsonl_to_one_file_per_session(self):
        self.assertEqual(self.log.append(body(records=({"t": 1}, {"t": 2, "メモ": "日本語"}))), 2)
        self.assertEqual(self.log.append(body(records=({"t": 3},))), 1)
        self.assertEqual(len(self.files()), 1)
        rows = [json.loads(line) for line in self.files()[0].read_text(encoding="utf-8").splitlines()]
        self.assertEqual([r["t"] for r in rows], [1, 2, 3])
        self.assertEqual(rows[1]["メモ"], "日本語")
        self.log.append(body(session="zzz999"))
        self.assertEqual(len(self.files()), 2)

    def test_bad_input_is_rejected_and_nothing_written(self):
        bad = [b"not json", b"[]", body(session="../x"), body(session="AB"), json.dumps({"session": "abc123"}).encode(),
               body(records=()), body(records=[{"t": 1}] * (sim_log.BATCH_RECORDS_MAX + 1)),
               json.dumps({"session": "abc123", "records": [1]}).encode()]
        for b in bad:
            with self.assertRaises(ValueError, msg=b[:40]):
                self.log.append(b)
        self.assertEqual(self.files(), [])

    def test_file_size_is_capped(self):
        original = sim_log.FILE_BYTES_MAX
        sim_log.FILE_BYTES_MAX = 100
        try:
            self.log.append(body(records=({"pad": "x" * 60},)))
            with self.assertRaises(ValueError):
                self.log.append(body(records=({"pad": "x" * 60},)))
        finally:
            sim_log.FILE_BYTES_MAX = original


if __name__ == "__main__":
    unittest.main()
