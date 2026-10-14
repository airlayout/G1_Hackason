"""スクリプトを走らせながら、一定時間ごとに全スレッドの Python の呼び出しの履歴をファイルに書き出す。

OMEN は ptrace が制限されていて py-spy で外から覗けないので、中から faulthandler で書き出す。
使い方: STACK_FILE=stacks.txt STACK_EVERY=120 python stackdump_run.py <script.py> [引数...]
"""
import faulthandler
import os
import runpy
import sys

stack_file = open(os.environ.get("STACK_FILE", "stacks.txt"), "w")
faulthandler.dump_traceback_later(int(os.environ.get("STACK_EVERY", "120")), repeat=True, file=stack_file)
script = sys.argv[1]
sys.argv = sys.argv[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(script)))
runpy.run_path(script, run_name="__main__")
