"""bench_vision_student.py の、torch が要らない部分（統計・TensorRT のログの読み取り・表と判定）の試験"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pc2"))
import bench_vision_student as b  # noqa: E402

# trtexec の結果の行（TensorRT 8.5 は --percentile の 1 つだけ、8.6 以降は 90/95/99 を並べる）
TRT85 = ("[10/21/2026-17:00:00] [I] GPU Compute Time: min = 2.10 ms, max = 3.20 ms, mean = 2.30 ms, "
         "median = 2.25 ms, percentile(99%) = 2.90 ms, total compute time = 2.99 s\n")
TRT86 = ("[I] GPU Compute Time: min = 2.1 ms, max = 3.2 ms, mean = 2.3 ms, median = 2.4 ms, "
         "percentile(90%) = 2.6 ms, percentile(95%) = 2.7 ms, percentile(99%) = 3.0 ms\n")


def _row(threads: int, full_p99: float, enc_med: float, rec_p99: float = 0.5, ok: bool = True) -> dict:
    s = lambda m, p: {"n": 10, "median": m, "p90": p, "p99": p, "max": p, "mean": m}  # noqa: E731
    return {"host": "pc2", "machine": "aarch64", "cpu_count": 8, "python": "3.12", "torch": "2.11",
            "threads": threads, "golden_ok": ok, "golden_max_err": 0.0 if ok else 1.0,
            "full": s(full_p99 - 1, full_p99), "encoder": s(enc_med, enc_med + 1),
            "recurrent": s(rec_p99 - 0.1, rec_p99), "pre": None}


def test_stats_percentiles():
    r = b.stats([1.0] * 98 + [10.0, 20.0])
    assert r["median"] == 1.0 and r["max"] == 20.0 and r["n"] == 100
    assert r["p99"] > r["p90"]


@pytest.mark.parametrize("text, med, p99", [(TRT85, 2.25, 2.90), (TRT86, 2.4, 3.0)])
def test_parse_trt_reads_both_versions(tmp_path, text, med, p99):
    p = tmp_path / "trt_fp16.log"
    p.write_text("[I] Latency: min = 9 ms, median = 9 ms, percentile(99%) = 9 ms\n" + text)
    assert b.parse_trt(p) == {"median": med, "p99": p99}  # Latency の行でなく GPU Compute Time の行を読む


def test_parse_trt_returns_none_on_failed_build(tmp_path):
    p = tmp_path / "trt_fp32.log"
    p.write_text("[E] Error[4]: network has dynamic or shape inputs\n")
    assert b.parse_trt(p) is None


def _write(tmp_path, rows):
    for r in rows:
        (tmp_path / f"t{r['threads']}.json").write_text(json.dumps(r))


def test_summary_pass_when_full_step_fits(tmp_path, capsys):
    _write(tmp_path, [_row(1, 60.0, 50.0), _row(8, 15.0, 12.0)])
    b.summary(tmp_path)
    out = capsys.readouterr().out
    assert "収まる（糸 8" in out


def test_summary_rows_sorted_numerically(tmp_path, capsys):
    _write(tmp_path, [_row(2, 40.0, 35.0), _row(10, 30.0, 25.0)])
    b.summary(tmp_path)
    out = capsys.readouterr().out
    assert out.index("| 2 |") < out.index("| 10 |")


def test_summary_fail_reports_encoder_rate(tmp_path, capsys):
    _write(tmp_path, [_row(4, 70.0, 50.0, rec_p99=0.8), _row(8, 45.0, 40.0, rec_p99=0.6)])
    b.summary(tmp_path)
    out = capsys.readouterr().out
    assert "収まらない（最良でも 99% 45.0 ms）" in out
    assert "99% 0.60 ms" in out and "約 25.0 Hz" in out


def test_summary_marks_golden_mismatch(tmp_path, capsys):
    _write(tmp_path, [_row(4, 15.0, 12.0, ok=False)])
    b.summary(tmp_path)
    out = capsys.readouterr().out
    assert "✘" in out and "golden が合わない" in out


def test_summary_lists_smaller_image_sizes(tmp_path, capsys):
    r = _row(8, 45.0, 40.0)
    r["encoder_sizes"] = {"144x256": {"n": 5, "median": 18.0, "p90": 19, "p99": 19, "max": 19, "mean": 18}}
    _write(tmp_path, [r])
    b.summary(tmp_path)
    assert "画像の部分だけ 144x256（中央 ms）: 糸 8 18.0" in capsys.readouterr().out


def test_verdict_uses_the_slower_of_continuous_and_paced(tmp_path, capsys):
    r = _row(8, 15.0, 12.0)
    r["full_paced"] = {"n": 10, "median": 21.0, "p90": 24.0, "p99": 25.0, "max": 26.0, "mean": 21.0}
    _write(tmp_path, [r])
    b.summary(tmp_path)
    out = capsys.readouterr().out
    assert "収まらない（最良でも 99% 25.0 ms）" in out  # 休みなしの 15 ms でなく、20 ms ごとの 25 ms で判定する


def test_summary_marks_failed_size(tmp_path, capsys):
    r = _row(8, 45.0, 40.0)
    r["encoder_sizes"] = {"108x192": {"error": "RuntimeError: shape"}}
    _write(tmp_path, [r])
    b.summary(tmp_path)
    assert "画像の部分だけ 108x192（中央 ms）: 糸 8 失敗" in capsys.readouterr().out


def test_trt_latent_err_reads_exported_json(tmp_path):
    p = tmp_path / "trt_fp16_out.json"
    p.write_text(json.dumps([{"name": "latent", "dimensions": "1x3", "values": [0.1, 0.2, 0.35]}]))
    assert b.trt_latent_err(p, np.array([0.1, 0.2, 0.3], dtype=np.float32)) == pytest.approx(0.05, abs=1e-6)
    assert b.trt_latent_err(tmp_path / "missing.json", np.zeros(3)) is None


def test_timed_paced_keeps_the_period():
    import time

    t0 = time.perf_counter()
    ms = b.timed(lambda i: None, iters=5, warmup=0, period_ms=20.0)
    assert len(ms) == 5 and max(ms) < 5.0  # 測るのは呼んだ時間だけ
    assert time.perf_counter() - t0 >= 0.09  # 拍子どおり 20 ms ごとに呼んでいる（5 回で 0.1 s 近く）
