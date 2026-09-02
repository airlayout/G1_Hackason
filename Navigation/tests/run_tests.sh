#!/usr/bin/env bash
# Navigation のテスト。実機・PC2・DDS のいずれも不要。
# scripts/ci/run_all_tests.sh が自動で見つけて実行する。
#
# 依存は 2 段になっている:
#
# | 段 | 要るもの | どのテスト |
# |---|---|---|
# | 必須 | numpy / scipy / scikit-image | protocol, occupancy, route, rooms, mission |
# | 任意 | mujoco / torch / mujoco-lidar | sim（歩行ポリシーを実際に回す） |
#
# 必須のほうは無ければここで入れる。CI は素の Python 3.12 で来るため
# （`.github/workflows/ci.yml` は pip install を持たず、
#  フォルダごとの run_tests.sh が自分の依存を面倒みる約束になっている）。
# 任意のほうは 500MB を超えるので**入れない**。無ければ該当テストは skip され、
# 何件 skip したかが出力に残る。緑＝全部検証した、ではない。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVIGATION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export PYTHONPATH="$NAVIGATION_DIR${PYTHONPATH:+:$PYTHONPATH}"

if ! python3 -c "import numpy, scipy, skimage" 2>/dev/null; then
    echo "--- numpy / scipy / scikit-image が無いので入れる ---"
    python3 -m pip install --quiet --disable-pip-version-check numpy scipy scikit-image
fi

python3 -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
