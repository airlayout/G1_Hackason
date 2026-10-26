"""stage_chain.sh・f3_keepalive_1026.sh の「学習が進まないのに回し続ける」の試験（10-27）。GPU も Isaac Sim も使わない。

10-26 21 時から 10-27 07:30 まで、学習の反復が上限（algo.trl.num_total_batches=50000）に届いた後、学習は始まるたびに
40 秒で終わり、stage_chain.sh は「チェックポイントが増えていない」と書いて次の回へ進むだけだった（10 時間の空回り）。

  python -m pytest -q G1_Hackason/Door/doorman/tests/test_stage_chain_stall.py

作り物: 一時的な HOME に OMEN と同じ並び（~/work_inoue/doorman・doorman6）を作り、学習（run_train_segments.sh）と
評価（eval_from_start.sh）と python（torch.load を作り物にする）を差し替える。台本は Linux（OMEN）向けなので、
Mac にない GNU の書き方（date -d・find -printf・stat -c）は小さな代わりの道具で受ける。
"""
from __future__ import annotations

import os
import shutil
import stat
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]

# --- GNU だけの書き方の代わり（Mac で試すため。OMEN では本物が使われる） ---------------------------------------------------
SHIM_DATE = r'''#!/usr/bin/env python3
import datetime, re, subprocess, sys
a = sys.argv[1:]
if "-d" in a:
    i = a.index("-d"); m = re.match(r"\+(\d+) min", a[i + 1])
    t = datetime.datetime.now() + datetime.timedelta(minutes=int(m.group(1)))
    fmt = [x for x in a if x.startswith("+")][-1][1:]
    print(t.strftime(fmt)); sys.exit(0)
sys.exit(subprocess.call(["/bin/date"] + a))
'''
SHIM_FIND = r'''#!/usr/bin/env python3
import os, subprocess, sys
a = sys.argv[1:]
if "-printf" in a:
    root, name = a[0], a[a.index("-name") + 1]
    for d, _, fs in os.walk(root):
        for f in fs:
            if f == name:
                p = os.path.join(d, f); print(f"{os.path.getmtime(p)} {p}")
    sys.exit(0)
sys.exit(subprocess.call(["/usr/bin/find"] + a))
'''
SHIM_STAT = r'''#!/usr/bin/env python3
import os, subprocess, sys
a = sys.argv[1:]
if a[:2] == ["-c", "%Y"]:
    print(int(os.path.getmtime(a[2]))); sys.exit(0)
sys.exit(subprocess.call(["/usr/bin/stat"] + a))
'''
# python の代わり: 本物の python3 に、torch.load が「<ckpt>.iter に書いた反復」を返す作り物の torch を足す
FAKE_TORCH = '''import types
def load(path, map_location=None, weights_only=None):
    return {"state": types.SimpleNamespace(global_step=int(open(str(path) + ".iter").read()))}
'''
# 学習の代わり: STUB_MODE で振る舞いを選ぶ。progress＝新しい last.pt を作る／stall＝始まるが重みは増えない／wait＝空くのを待つだけ
STUB_TRAIN = r'''#!/usr/bin/env bash
LOGROOT="$WORK/$REPO/logs_rl/$PROJECT"
case "${STUB_MODE:-progress}" in
  progress)
    n=$(ls "$LOGROOT" 2>/dev/null | wc -l | tr -d " "); d="$LOGROOT/run_$n"; mkdir -p "$d"
    prev=$(cat "$HOME/last_iter" 2>/dev/null || echo 45150); it=$((prev + 400)); echo "$it" > "$HOME/last_iter"
    sleep 1; touch "$d/last.pt"; echo "$it" > "$d/last.pt.iter"; cp "$HOME/config.yaml" "$HOME/meta.yaml" "$d/"
    echo "=== start seg=0 out=x"; echo "=== all_done reason=deadline" ;;
  stall) echo "=== start seg=0 out=x"; echo "=== end seg=0 reason=exited"; echo "=== all_done reason=exited" ;;
  wait) echo "=== wait machine free (need 1MB) seg=0"; echo "=== all_done reason=deadline segments=0" ;;
esac
'''
STUB_EVAL = r'''#!/usr/bin/env bash
out="$HOME/work_inoue/doorman/records/r_$RANDOM$RANDOM"; mkdir -p "$out"
echo "{\"success_rate\": ${STUB_RATE:-0.5}, \"success\": 128, \"episodes_from_stage0\": 256}" > "$out/summary.json"
echo "=== eval_from_start out=$out success=128/256 rc=0"
'''


def _write(path: Path, text: str, executable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    if executable:
        path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


@pytest.fixture()
def omen(tmp_path):
    """OMEN と同じ並びの一時的な HOME"""
    home = tmp_path / "home"
    base = home / "work_inoue" / "doorman"
    base.mkdir(parents=True)
    for name in ("stage_chain.sh", "exp_f3_retrain_1026.sh", "f3_keepalive_1026.sh"):
        shutil.copy(HERE / name, base / name)
        (base / name).chmod(0o755)
    _write(base / "run_train_segments.sh", STUB_TRAIN, True)
    _write(base / "eval_from_start.sh", STUB_EVAL, True)
    _write(home / "work_inoue" / "doorman6" / "simfix_args.sh", "#!/usr/bin/env bash\necho '++env.config.a=1 ++env.config.b=[1,2]'\n", True)
    shim = tmp_path / "shim"
    _write(shim / "date", SHIM_DATE, True)
    _write(shim / "find", SHIM_FIND, True)
    _write(shim / "stat", SHIM_STAT, True)
    fake = tmp_path / "fake_torch"
    _write(fake / "torch" / "__init__.py", FAKE_TORCH)
    py = home / "miniforge3" / "envs" / "doorman6" / "bin" / "python"
    _write(py, f"#!/usr/bin/env bash\nPYTHONPATH={fake} exec {sys.executable} \"$@\"\n", True)
    ck = base / "ckpt" / "start" / "last.pt"
    _write(ck, "")
    _write(Path(str(ck) + ".iter"), "45150")
    for f in ("config.yaml", "meta.yaml"):
        _write(ck.parent / f, "x: 1\n")
        _write(home / f, "x: 1\n")
    env = dict(os.environ, HOME=str(home), PATH=f"{shim}:{os.environ['PATH']}", IDLE_GATE="0")
    return {"home": home, "base": base, "ck": ck, "env": env}


def _chain(omen, mode: str, rounds: int, **extra):
    env = dict(omen["env"], STUB_MODE=mode, CHAIN_REPO="R", PAUSE_FILE=str(omen["home"] / "P"), STOP_FILE=str(omen["home"] / "S"),
               PASS_RATE="1.1", EVAL_N="8", **extra)
    r = subprocess.run(["bash", "./stage_chain.sh", str(omen["ck"]), f"f3:door:{rounds}:1:through:++a=1"],
                       cwd=omen["base"], env=env, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120, check=False)
    log = max((omen["base"] / "chain").glob("*/chain.log")).read_text(encoding="utf-8")
    return r, log


def test_stalled_training_stops_the_chain_after_three_rounds(omen):
    """学習が始まるのに重みが増えない回が 3 回続いたら止まり、理由（反復の上限か起動の失敗）を書く"""
    r, log = _chain(omen, "stall", rounds=10)
    assert r.returncode == 1
    assert "chain_done result=stalled_at_f3" in log
    assert log.count("学習は始まったのに重みが増えていない") == 3
    assert "num_total_batches" in log
    assert "round=4/10" not in log  # 4 回目は始めない


def test_waiting_for_the_machine_is_not_a_stall(omen):
    """空くのを待っただけの回（学習が始まっていない）は数えず、決めた回数まで回る"""
    _, log = _chain(omen, "wait", rounds=5)
    assert "stalled" not in log
    assert log.count("空くのを待っていて学習は始まらなかった") == 5
    assert "chain_done result=fail_at_f3" in log


def test_progress_resets_and_keeps_going(omen):
    """重みが増える回は今までどおり評価して次へ進む"""
    _, log = _chain(omen, "progress", rounds=3)
    assert log.count("success=128/256") == 3
    assert "stalled" not in log


def test_exp_f3_refuses_to_start_at_the_iteration_cap(omen):
    """始める前に重みの反復が上限（NUM_TOTAL_BATCHES）に届いていたら、始めずに理由を書いて終わる"""
    env = dict(omen["env"], CK=str(omen["ck"]), NUM_TOTAL_BATCHES="45150", STUB_MODE="progress")
    r = subprocess.run(["bash", "./exp_f3_retrain_1026.sh"], cwd=omen["base"], env=env, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120, check=False)
    assert r.returncode == 1
    assert "上限" in r.stdout and "45150" in r.stdout
    assert not (omen["base"] / "chain").exists()


def test_keepalive_does_not_restart_after_a_stall(omen):
    """連鎖が「学習が進まない」で止まったときは、見張りは始め直さずに終わる"""
    chain = omen["base"] / "chain" / "1027_0000"
    chain.mkdir(parents=True)
    (chain / "chain.log").write_text(f"=== chain_done result=stalled_at_f3 last_ckpt={omen['ck']}\n", encoding="utf-8")
    (chain / "results.tsv").write_text("stage\n", encoding="utf-8")
    r = subprocess.run(["bash", "./f3_keepalive_1026.sh"], cwd=omen["base"], env=omen["env"], capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=60, check=False)
    assert r.returncode == 1
    assert "進まない" in r.stdout
    assert len(list((omen["base"] / "chain").iterdir())) == 1  # 新しい連鎖を始めていない
