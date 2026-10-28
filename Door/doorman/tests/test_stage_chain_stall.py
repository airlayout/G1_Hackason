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
echo "n=$1 mem_max=${MEM_MAX:-}" >> "$HOME/train_calls"  # 台数とメモリの上限が届いたか（10-28）
case "${STUB_MODE:-progress}" in
  progress)
    n=$(ls "$LOGROOT" 2>/dev/null | wc -l | tr -d " "); d="$LOGROOT/run_$n"; mkdir -p "$d"
    prev=$(cat "$HOME/last_iter" 2>/dev/null || echo 45150); it=$((prev + 400)); echo "$it" > "$HOME/last_iter"
    sleep 1; touch "$d/last.pt"; echo "$it" > "$d/last.pt.iter"; cp "$HOME/config.yaml" "$HOME/meta.yaml" "$d/"
    [ -n "${STUB_STOP_AFTER:-}" ] && touch "$STUB_STOP_AFTER"  # 学習の途中で人が止め札を置いた
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
    for name in ("stage_chain.sh", "exp_f3_retrain_1026.sh", "f3_keepalive_1026.sh", "exp_f3bc_1027.sh", "exp_f3_phases_1028.sh"):
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


def test_two_chains_started_in_the_same_minute_keep_their_own_results(omen):
    """同じ分に始めた 2 つの連鎖が、同じ置き場の results.tsv を上書きしない（10-28。段 C〜E は results.tsv から回を数え直す）"""
    _chain(omen, "progress", rounds=1)
    _chain(omen, "progress", rounds=1)
    dirs = sorted((omen["base"] / "chain").iterdir())
    assert len(dirs) == 2
    for d in dirs:
        assert len((d / "results.tsv").read_text(encoding="utf-8").splitlines()) == 2  # 見出し＋1 回


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


# --- 10-27 夕: 直し 2・3 を順に試す台本（exp_f3bc_1027.sh）と、学習の引数を足す TRAIN_MORE -------------------------------------
def _all_chain_logs(omen) -> str:
    return "".join(p.read_text(encoding="utf-8") for p in sorted((omen["base"] / "chain").glob("*/chain.log")))


def test_exp_f3_appends_train_more_to_the_training_args(omen):
    """TRAIN_MORE に書いた引数が学習に渡る（stage_chain の extra= に出る）"""
    env = dict(omen["env"], CK=str(omen["ck"]), STUB_MODE="progress", ROUNDS="1", PASS_RATE="1.1", NAME="f3b",
               TRAIN_MORE="++algo.config.max_noise_std=0.26")
    r = subprocess.run(["bash", "./exp_f3_retrain_1026.sh"], cwd=omen["base"], env=env, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=120, check=False)
    log = _all_chain_logs(omen)
    assert "stage=f3b" in log
    assert "++algo.config.max_noise_std=0.26" in log
    assert "name=f3b" in r.stdout and "more=++algo.config.max_noise_std=0.26" in r.stdout


def test_exp_f3_refuses_train_more_with_a_colon(omen):
    """「:」は段の区切りなので、TRAIN_MORE に入っていたら始めない"""
    env = dict(omen["env"], CK=str(omen["ck"]), STUB_MODE="progress", TRAIN_MORE="++a.b=c:d")
    r = subprocess.run(["bash", "./exp_f3_retrain_1026.sh"], cwd=omen["base"], env=env, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=120, check=False)
    assert r.returncode == 1
    assert "TRAIN_MORE" in r.stdout
    assert not (omen["base"] / "chain").exists()


def test_f3bc_runs_noise_cap_then_noise_cap_plus_walk_away_from_the_same_best(omen):
    """段 A（雑音の上限だけ）→ 段 B（上限＋歩き去りの罰）の順に、どちらも控えた最良の重みから始める"""
    best_dir = omen["base"] / "ckpt" / "f3_best"
    env = dict(omen["env"], STUB_MODE="progress", BEST_SRC=str(omen["ck"]), BEST_DIR=str(best_dir),
               ROUNDS_A="1", ROUNDS_B="1", NO_KEEPALIVE="1")
    r = subprocess.run(["bash", "./exp_f3bc_1027.sh"], cwd=omen["base"], env=env, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=240, check=False)
    assert r.returncode == 0, r.stdout + r.stderr
    assert (best_dir / "last.pt").exists() and (best_dir / "config.yaml").exists()
    log = _all_chain_logs(omen)
    a = log.index("chain start")
    b = log.index("chain start", a + 1)  # 段 B の連鎖の始めの行（段の指定がそのまま書かれる）
    assert "stage=f3b" in log[a:b] and "stage=f3c" in log[b:]
    assert a < b
    assert "walk_away_in_swing" not in log[a:b]  # 段 A には罰を入れない
    assert "++algo.config.max_noise_std=0.26" in log[a:b]
    assert "++algo.config.max_noise_std=0.26" in log[b:] and "++rewards.reward_scales.walk_away_in_swing=-12.0" in log[b:]
    assert log.count(f"resume={best_dir}/last.pt") == 2  # 2 つの段とも最良の重みから
    assert "見張りを始めずに終わる" in r.stdout


def test_f3bc_does_not_start_stage_b_when_stopped(omen):
    """段 A の途中で止め札が置かれたら、段 B を始めない（始めるときに残っている札は stage_chain.sh が消すので、途中で置く）"""
    stop = omen["home"] / "work_inoue" / "STOP_DOOR_F3"
    env = dict(omen["env"], STUB_MODE="progress", BEST_SRC=str(omen["ck"]), BEST_DIR=str(omen["base"] / "ckpt" / "f3_best"),
               ROUNDS_A="2", ROUNDS_B="1", NO_KEEPALIVE="1", STUB_STOP_AFTER=str(stop))
    r = subprocess.run(["bash", "./exp_f3bc_1027.sh"], cwd=omen["base"], env=env, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=240, check=False)
    assert r.returncode == 0
    assert "段 B を始めずに終わる" in r.stdout
    assert "stage=f3c" not in _all_chain_logs(omen)


# --- 10-28: 候補 1〜3 を 1 つずつ足す台本（exp_f3_phases_1028.sh） ----------------------------------------------------------------
def _phases(omen, **extra):
    env = dict(omen["env"], STUB_MODE="progress", BEST=str(omen["ck"]), ROUNDS_C="1", ROUNDS_D="1", ROUNDS_E="1", NO_KEEPALIVE="1")
    env.update(extra)
    return subprocess.run(["bash", "./exp_f3_phases_1028.sh"], cwd=omen["base"], env=env, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", timeout=360, check=False)


def _chain_parts(log: str) -> list[str]:
    """chain.log をつないだ文字列を、連鎖の始めの行ごとに切る"""
    idx = [i for i in range(len(log)) if log.startswith("chain start", i)]
    return [log[a:b] for a, b in zip(idx, [*idx[1:], len(log)])]


def test_phases_add_one_fix_at_a_time_from_the_same_best(omen):
    """段 C（罰を早く強く）→ 段 D（＋持ち時間 600）→ 段 E（＋近づく報酬を止める）の順に、どれも同じ重みから始める"""
    r = _phases(omen)
    assert r.returncode == 0, r.stdout + r.stderr
    c, d, e = _chain_parts(_all_chain_logs(omen))
    assert "stage=f3d" in c and "stage=f3e" in d and "stage=f3f" in e
    for part in (c, d, e):
        assert "++algo.config.max_noise_std=0.26" in part
        assert "++env.config.walk_away_start_m=1.0" in part and "++rewards.reward_scales.walk_away_in_swing=-40.0" in part
        assert f"resume={omen['ck']}" in part  # どの段も同じ出発点から
    # 持ち時間の並びのカンマは「|」で渡し、学習の引数ではカンマに戻る
    assert "max_stage_time=[250" not in c  # 評価の持ち時間（[500,…]）は連鎖の始めの行にいつも出るので、学習の方だけ見る
    assert "++env.config.max_stage_time=[250,100,100,100,600,200]" in d and "++env.config.max_stage_time=[250,100,100,100,600,200]" in e
    assert "target_root_distance_swing_scale" not in c + d
    assert "++env.config.target_root_distance_swing_scale=0.0" in e
    assert "見張りを始めずに終わる" in r.stdout


def test_phases_stop_before_the_next_phase(omen):
    """段 C の途中で止め札が置かれたら、段 D も見張りも始めない"""
    stop = omen["home"] / "work_inoue" / "STOP_DOOR_F3"
    r = _phases(omen, ROUNDS_C="2", STUB_STOP_AFTER=str(stop))
    assert r.returncode == 0
    assert "次の段も見張りも始めずに終わる" in r.stdout
    log = _all_chain_logs(omen)
    assert "stage=f3d" in log and "stage=f3e" not in log


@pytest.mark.parametrize(("extra", "want", "sfx"), [({}, "n=512 mem_max=9G", ""),
                                                     ({"NUM_ENVS": "4096", "MEM_MAX": "12G"}, "n=4096 mem_max=12G", "n4096")])
def test_phases_pass_the_number_of_envs_to_every_training(omen, extra, want, sfx):
    """台数とメモリの上限は、段 C〜E のどの学習にも届き、512 台でなければ段の名前に台数が付く（10-28。省くと元の 512 台・9G）"""
    r = _phases(omen, **extra)
    assert r.returncode == 0, r.stdout + r.stderr
    calls = (omen["home"] / "train_calls").read_text(encoding="utf-8").splitlines()
    assert calls == [want] * 3
    assert want in r.stdout  # 段の始めの行にも出す
    c, d, e = _chain_parts(_all_chain_logs(omen))
    assert f"stage=f3d{sfx} " in c and f"stage=f3e{sfx} " in d and f"stage=f3f{sfx} " in e
    # 重みの置き場も段の名前で分かれる（512 台の回の door_f3d と混ざらない）
    logs = omen["home"] / "work_inoue" / "doorman6" / "VIRAL6_fix" / "logs_rl"
    assert sorted(p.name for p in logs.iterdir()) == [f"door_f3d{sfx}", f"door_f3e{sfx}", f"door_f3f{sfx}"]


def _rounds(omen) -> dict[str, list[list[str]]]:
    """全部の連鎖の results.tsv の回を、段の名前ごとに（連鎖の置き場の名前の順に）"""
    out: dict[str, list[list[str]]] = {}
    for p in sorted((omen["base"] / "chain").glob("*/results.tsv")):
        for row in p.read_text(encoding="utf-8").splitlines()[1:]:
            cols = row.split("\t")
            out.setdefault(cols[0], []).append(cols)
    return out


def _prior_chain(omen, stage: str, rates: list[float]) -> Path:
    """前に回して止めた段の回を、古い名前の連鎖の置き場に置く（重みも作る）。いちばん新しい回の重みを返す"""
    d = omen["base"] / "chain" / "0101_0000"
    d.mkdir(parents=True)
    rows = ["stage\ttask\tround\titer\tsuccess\trate\tckpt\trecord"]
    ck = omen["ck"]
    for i, rate in enumerate(rates, 1):
        ck = omen["base"] / "ckpt" / f"prior{i}" / "last.pt"
        _write(ck, "")
        _write(Path(f"{ck}.iter"), str(40000 + i))  # 続き（作り物の学習は 45,150 から）より小さい反復
        rows.append(f"{stage}\tdoor\t{i}\t{40000 + i}\t0/256\t{rate}\t{ck}\tnone")
    (d / "results.tsv").write_text("\n".join(rows) + "\n", encoding="utf-8")
    return ck


def test_phases_move_on_after_three_rounds_when_the_difference_is_clear(omen):
    """3 回の平均が比べる相手から 5 ポイント以上離れていれば、最大の回数を待たずに次の段へ（10-28 ユーザー「おすすめで進めて」）"""
    r = _phases(omen, ROUNDS_C="6", ROUNDS_D="1")  # 作り物の評価は 50%。段 C は 512 台の段 C の 74.7% と比べる
    assert r.returncode == 0, r.stdout + r.stderr
    rounds = _rounds(omen)
    assert [len(rounds[s]) for s in ("f3d", "f3e", "f3f")] == [3, 1, 1]
    assert "判定 f3d: 3 回の平均 50.0% 対 74.7%（差 24.7）→ 次の段へ" in r.stdout


def test_phases_extend_to_the_max_rounds_when_the_difference_is_unclear(omen):
    """差が 5 ポイントより小さければ、その段の最後の重みから最大の回数まで続ける"""
    r = _phases(omen, ROUNDS_C="4", ROUNDS_D="1", REF_C="52")
    assert r.returncode == 0, r.stdout + r.stderr
    rounds = _rounds(omen)
    assert len(rounds["f3d"]) == 4
    parts = _chain_parts(_all_chain_logs(omen))
    assert len(parts) == 4  # 段 C の 3 回・その続きの 1 回・段 D・段 E
    assert f"resume={rounds['f3d'][2][6]}" in parts[1]  # 続きは 3 回目の重みから
    assert "判定 f3d: 3 回の平均 50.0% 対 52%（差 2.0）→ 4 回まで続ける" in r.stdout


def test_phases_continue_a_stopped_phase_from_its_last_checkpoint(omen):
    """止めて組み直したときは前の連鎖の回も数え、足りない回だけを前の最後の重みから回す（10-28 に 4,096 台の段 C を組み直した）"""
    last = _prior_chain(omen, "f3d", [0.95, 0.95])
    r = _phases(omen, ROUNDS_C="6", ROUNDS_D="1")
    assert r.returncode == 0, r.stdout + r.stderr
    assert len(_rounds(omen)["f3d"]) == 3  # 前の 2 回＋1 回。平均 80.0% は 74.7% から 5.3 離れているので次の段へ
    parts = _chain_parts(_all_chain_logs(omen))
    assert f"resume={last}" in parts[0]
    assert f"resume={omen['ck']}" in parts[1]  # 段 D は最良の重みから
    assert "→ 次の段へ" in r.stdout


def test_phases_from_phase_d_compare_with_the_rounds_of_phase_c(omen):
    """FROM_PHASE=D なら段 C は回さず、段 D の 3 回を段 C の回の平均と比べる"""
    _prior_chain(omen, "f3d", [0.5, 0.5, 0.5])
    r = _phases(omen, FROM_PHASE="D", ROUNDS_D="4")
    assert r.returncode == 0, r.stdout + r.stderr
    rounds = _rounds(omen)
    assert [len(rounds[s]) for s in ("f3d", "f3e", "f3f")] == [3, 4, 1]
    assert "段 f3d を" not in r.stdout
    assert "判定 f3e: 3 回の平均 50.0% 対 f3d の 50.0%（差 0.0）→ 4 回まで続ける" in r.stdout


def test_phases_refuse_to_start_without_the_best_checkpoint(omen):
    """出発点の重みが無ければ、連鎖を始めずに終わる"""
    r = _phases(omen, BEST=str(omen["base"] / "ckpt" / "none" / "last.pt"))
    assert r.returncode == 1
    assert "出発点の重みが無い" in r.stdout
    assert not (omen["base"] / "chain").exists()
