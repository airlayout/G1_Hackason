"""評価環境を使っていて、スムーズにいかなかったこと（困りごと）を記録する。どのブランチからでも書ける。

    P=~/miniconda3/envs/lerobot/bin/python          # Python 3.8 以上ならどれでもよい（標準のライブラリだけを使う）
    $P Button_Press/Yada/tools/trouble_log.py add             # 質問に答えて 1 件書く（コミットまで）
    $P Button_Press/Yada/tools/trouble_log.py push            # 書いたものを GitHub に送る
    $P Button_Press/Yada/tools/trouble_log.py list            # 一覧（全員の分）
    $P Button_Press/Yada/tools/trouble_log.py show <番号>     # 1 件を表示する
    $P Button_Press/Yada/tools/trouble_log.py add --title "..." --category isaac --what "..." --happened "..."  # 質問なしで書く

しくみ:
- ログは、ログ専用のブランチ log/eval-env の eval_env_log/ に、1 件 = 1 ファイルで置く
  （ファイル名は「日時_書いた人_分類_ランダムな 4 文字.md」なので、別々の人が同時に書いてもぶつからない。
  英数字だけにしている: 日本語のファイル名は、macOS が濁点などを分けて保存するため、git で別のファイルに見えることがある）
- 書き込みは、手元の別のフォルダ（_local/trouble_log_worktree。git の worktree）でログ専用のブランチを開いて行う。
  今作業しているブランチは切り替えず、今のファイルにも触らない
- 一覧のファイル（索引）は置かない（全員で 1 つのファイルを書き換えると、ぶつかるため）。list で、そのつど作る
- コミットとプッシュは別のコマンド（リポジトリの決まり: コミットは手元、プッシュは全員への公開なので、分けて確かめる）

GitHub では https://github.com/airlayout/G1_Hackason/tree/log/eval-env/eval_env_log で見られる。

このスクリプトが無いブランチで作業しているときの取り出し方（ブランチは切り替えない。リポジトリの中で実行する）:
    git fetch origin log/eval-env && git show origin/log/eval-env:trouble_log.py > /tmp/trouble_log.py
    python /tmp/trouble_log.py add
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import platform
import re
import subprocess
import sys
from pathlib import Path

LOG_BRANCH = "log/eval-env"
TOOL_NAME = "trouble_log.py"  # ログ専用のブランチの一番上に置く、このスクリプトの写し
LOG_DIR = "eval_env_log"
REMOTE = "origin"

CATEGORIES = {
    "setup": "準備（ライブラリ・モデル・環境の作り方）",
    "docs": "文書（GUIDE.md などが分かりにくい・間違っている）",
    "mujoco": "MuJoCo での評価",
    "isaac": "Isaac Sim での評価",
    "sim_dds": "模擬 G1（DDS・実機と同じ口）",
    "agent": "エージェントの作り方（interface、観測、動作）",
    "report": "弱点のレポート・結果のファイル",
    "real": "実機に持っていくとき",
    "other": "その他",
}
SEVERITIES = {
    "blocker": "進めなくなった",
    "major": "回り道して進めた（時間がかかった）",
    "minor": "少し迷った・気になった",
}
STATUSES = ("未対応", "対応中", "対応済み", "対応しない")

README = f"""# 評価環境の困りごとのログ

`Button_Press/Yada` の評価環境を使っていて、スムーズにいかなかったことを記録する場所（ブランチ `{LOG_BRANCH}`）。
「質問した」「回り道した」「文書と違った」など、小さなことでも書いてほしい。改善の材料にする。

## 書き方

どのブランチで作業していても、次のコマンドで書ける（今のブランチは切り替わらない）。

```bash
P=~/miniconda3/envs/lerobot/bin/python
$P Button_Press/Yada/tools/trouble_log.py add      # 質問に答えて 1 件書く（コミットまで）
$P Button_Press/Yada/tools/trouble_log.py push     # GitHub に送る
$P Button_Press/Yada/tools/trouble_log.py list     # 一覧
```

`Button_Press/Yada/tools/trouble_log.py` が無いブランチでも、ブランチを切り替えずに、このブランチに置いた写し
（一番上の `trouble_log.py`）を取り出して使える（リポジトリの中で実行する）:

```bash
git fetch origin {LOG_BRANCH} && git show origin/{LOG_BRANCH}:trouble_log.py > /tmp/trouble_log.py
python /tmp/trouble_log.py add
```

スクリプトを使わずに、GitHub のこのブランチの `{LOG_DIR}/` の下へ、直接ファイルを足してもよい（1 件 = 1 ファイル、
ファイル名は `日時_書いた人_分類_ランダムな4文字.md` のように英数字だけにする。下の項目をそろえる）。

## 1 件のファイルの中身

先頭の項目（`key: value`）は、一覧（`trouble_log.py list`）で使う。

| 項目 | 内容 |
|---|---|
| `title` | 題名（1 行） |
| `date`, `author` | 書いた日時と人（自動） |
| `category` | {", ".join(CATEGORIES)} |
| `severity` | blocker（進めなくなった）/ major（回り道した）/ minor（少し迷った） |
| `status` | {" / ".join(STATUSES)}（対応した人が書き換える） |
| `branch`, `commit` | 書いたときに作業していたブランチと、そのコミット（自動） |

本文: 何をしようとしたか / 何が起きたか / 期待していたこと / どう回避したか / 再現のコマンド / エラーの文 / 環境（自動）。

## ぶつからないための決まり

- 1 件 = 1 ファイル。ほかの人のファイルは、`status` と「対応」の欄を書き換えるときだけ触る
- 一覧のファイルは置かない（`list` で作る）
"""


# ---- git ----------------------------------------------------------------------

def git(args: list[str], cwd: Path, check: bool = True, capture: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["git", *args], cwd=cwd, check=check, text=True,
                          stdout=subprocess.PIPE if capture else None, stderr=subprocess.PIPE if capture else None)


def repo_root() -> Path:
    """作業中のリポジトリ（の一番上）。実行した場所から探し、だめならこのスクリプトの場所から探す。

    実行した場所を先に見るので、このスクリプトをリポジトリの外（/tmp など）に置いても、リポジトリの中で実行すれば動く
    （スクリプトが入っていないブランチで作業しているとき。下の「取り出し方」）。
    """
    for where in (Path.cwd(), Path(__file__).resolve().parent):
        out = subprocess.run(["git", "rev-parse", "--show-toplevel"], text=True, capture_output=True, cwd=where)
        if out.returncode == 0:
            root = Path(out.stdout.strip())
            # ログ専用のブランチの作業用のフォルダの中で実行されたら、その外側（作業中のリポジトリ）を使う
            if root.name == "trouble_log_worktree" and root.parent.name == "_local":
                return root.parent.parent
            return root
    sys.exit("[trouble_log] git のリポジトリの中で実行してください")


def worktree_dir(root: Path) -> Path:
    return root / "_local" / "trouble_log_worktree"


def remote_has_branch(root: Path) -> bool:
    out = git(["ls-remote", "--heads", REMOTE, LOG_BRANCH], root, check=False)
    return out.returncode == 0 and bool(out.stdout.strip())


def ensure_worktree(root: Path, create_if_missing: bool = False) -> Path:
    """ログ専用のブランチを、手元の別のフォルダに開く（無ければ GitHub から取ってくる。そこにも無ければ作る）。"""
    wt = worktree_dir(root)
    if (wt / ".git").exists():
        return wt
    git(["worktree", "prune"], root, check=False)
    wt.parent.mkdir(parents=True, exist_ok=True)
    local = git(["rev-parse", "--verify", "--quiet", f"refs/heads/{LOG_BRANCH}"], root, check=False).returncode == 0
    if not local and remote_has_branch(root):
        git(["fetch", REMOTE, f"{LOG_BRANCH}:{LOG_BRANCH}"], root)
        local = True
    if local:
        git(["worktree", "add", str(wt), LOG_BRANCH], root)
        return wt
    if not create_if_missing:
        sys.exit(f"[trouble_log] ログのブランチ {LOG_BRANCH} が、手元にも GitHub にも無い。"
                 f"作る場合は `trouble_log.py init` を実行する")
    # どのブランチの履歴も引き継がない、ログだけのブランチを作る
    git(["worktree", "add", "--detach", str(wt)], root)
    git(["checkout", "--orphan", LOG_BRANCH], wt)
    git(["rm", "-rf", "--quiet", "."], wt, check=False)
    (wt / LOG_DIR).mkdir(exist_ok=True)
    (wt / LOG_DIR / ".gitkeep").write_text("")
    (wt / "README.md").write_text(README, encoding="utf-8")
    (wt / TOOL_NAME).write_text(Path(__file__).read_text(encoding="utf-8"), encoding="utf-8")
    git(["add", "README.md", TOOL_NAME, f"{LOG_DIR}/.gitkeep"], wt)
    git(["commit", "-q", "-m", "評価環境の困りごとのログを始める（README と eval_env_log/）"], wt)
    print(f"[trouble_log] ログのブランチ {LOG_BRANCH} を作った（まだ GitHub には送っていない。push で送る）")
    return wt


def sync(wt: Path, root: Path) -> None:
    """GitHub の最新を取り込む（ファイルは 1 件ずつ別なので、ぶつからない）。"""
    if remote_has_branch(root):
        git(["fetch", "--quiet", REMOTE, LOG_BRANCH], wt, check=False)
        git(["rebase", "--quiet", f"{REMOTE}/{LOG_BRANCH}"], wt, check=False)


# ---- 1 件のファイル -------------------------------------------------------------

def environment_lines(root: Path) -> list[str]:
    lines = [f"- OS: {platform.platform()}", f"- Python（このスクリプト）: {platform.python_version()}"]
    try:
        lines.append(f"- CPU のコア: {len(os.sched_getaffinity(0))}")
    except AttributeError:
        pass
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], capture_output=True,
                             text=True, timeout=10)
        if out.returncode == 0 and out.stdout.strip():
            lines.append(f"- GPU: {', '.join(s.strip() for s in out.stdout.splitlines())}")
    except Exception:
        pass
    return lines


def ask(prompt: str, default: str = "", multiline: bool = False) -> str:
    if multiline:
        print(f"{prompt}（複数行。空の行で終わり）")
        lines = []
        while True:
            try:
                s = input("  > ")
            except EOFError:
                break
            if not s:
                break
            lines.append(s)
        return "\n".join(lines) or default
    try:
        s = input(f"{prompt}{f'（既定 {default}）' if default else ''}: ").strip()
    except EOFError:
        s = ""
    return s or default


def choose(prompt: str, options: dict[str, str], default: str) -> str:
    print(prompt)
    keys = list(options)
    for i, k in enumerate(keys, 1):
        print(f"  {i}. {k:<8} {options[k]}")
    while True:
        s = ask("  番号か名前", default)
        if s in options:
            return s
        if s.isdigit() and 1 <= int(s) <= len(keys):
            return keys[int(s) - 1]
        print("  一覧の番号か名前を入れてください")


def build_entry(a: argparse.Namespace, root: Path) -> tuple[str, str]:
    """(ファイル名, 中身)。"""
    now = dt.datetime.now()
    author = a.author or git(["config", "user.name"], root, check=False).stdout.strip() or os.environ.get("USER", "someone")
    branch = git(["branch", "--show-current"], root, check=False).stdout.strip() or "(detached)"
    commit = git(["rev-parse", "--short", "HEAD"], root, check=False).stdout.strip()
    meta = [f"title: {a.title}", f"date: {now:%Y-%m-%d %H:%M}", f"author: {author}", f"category: {a.category}",
            f"severity: {a.severity}", f"status: 未対応", f"branch: {branch}", f"commit: {commit}"]
    sec = [
        ("何をしようとしたか", a.what),
        ("何が起きたか", a.happened),
        ("期待していたこと", a.expected),
        ("どう回避したか（できたなら）", a.workaround),
        ("再現のコマンド", f"```bash\n{a.command}\n```" if a.command else ""),
        ("エラーの文", f"```\n{a.error}\n```" if a.error else ""),
    ]
    body = ["---", *meta, "---", "", f"# {a.title}", ""]
    for h, v in sec:
        body += [f"## {h}", "", v or "（なし）", ""]
    body += ["## 環境（自動）", "", *environment_lines(root), "", "## 対応（対応した人が書く）", "", "（まだ）", ""]
    safe_author = re.sub(r"[^A-Za-z0-9.-]+", "", author)[:20] or "someone"
    name = f"{now:%Y%m%d-%H%M%S}_{safe_author}_{a.category}_{os.urandom(2).hex()}.md"
    return name, "\n".join(body)


def parse_meta(text: str) -> dict[str, str]:
    m = re.match(r"---\n(.*?)\n---", text, re.S)
    out: dict[str, str] = {}
    if m:
        for line in m.group(1).splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                out[k.strip()] = v.strip()
    return out


# ---- コマンド -------------------------------------------------------------------

def cmd_init(a: argparse.Namespace) -> int:
    root = repo_root()
    ensure_worktree(root, create_if_missing=True)
    return 0


def cmd_add(a: argparse.Namespace) -> int:
    root = repo_root()
    wt = ensure_worktree(root)
    sync(wt, root)
    interactive = not a.title and sys.stdin.isatty()
    if interactive:
        print("[trouble_log] 困ったことを 1 件書きます（Ctrl+C でやめる）")
        a.title = ask("題名（1 行）")
        a.category = choose("どこで困ったか", CATEGORIES, a.category or "other")
        a.severity = choose("どのくらい困ったか", SEVERITIES, a.severity or "minor")
        a.what = a.what or ask("何をしようとしたか", multiline=True)
        a.happened = a.happened or ask("何が起きたか", multiline=True)
        a.expected = a.expected or ask("期待していたこと", multiline=True)
        a.workaround = a.workaround or ask("どう回避したか（できたなら）", multiline=True)
        a.command = a.command or ask("再現のコマンド（あれば）")
        a.error = a.error or ask("エラーの文（あれば。貼り付け）", multiline=True)
    if not a.title:
        print("[trouble_log] 題名（--title）が要る")
        return 2
    a.category = a.category or "other"
    a.severity = a.severity or "minor"
    if a.category not in CATEGORIES or a.severity not in SEVERITIES:
        print(f"[trouble_log] category は {list(CATEGORIES)}、severity は {list(SEVERITIES)} のどれか")
        return 2
    if a.error_file:
        a.error = (a.error + "\n" if a.error else "") + Path(a.error_file).read_text(errors="replace")[-4000:]
    name, text = build_entry(a, root)
    path = wt / LOG_DIR / name
    path.parent.mkdir(exist_ok=True)
    path.write_text(text, encoding="utf-8")
    git(["add", str(path.relative_to(wt))], wt)
    git(["commit", "-q", "-m", f"困りごと: {a.title}"], wt)
    print(f"[trouble_log] 書いてコミットした: {LOG_DIR}/{name}")
    print(f"[trouble_log] 中身は {path}（直すなら編集して `trouble_log.py commit`）")
    print("[trouble_log] GitHub に送るには: trouble_log.py push")
    return 0


def cmd_update_tool(a: argparse.Namespace) -> int:
    """ログ専用のブランチに置いた、このスクリプトの写しと README を新しくする（スクリプトを直したときに使う）。"""
    root = repo_root()
    wt = ensure_worktree(root)
    sync(wt, root)
    (wt / TOOL_NAME).write_text(Path(__file__).read_text(encoding="utf-8"), encoding="utf-8")
    (wt / "README.md").write_text(README, encoding="utf-8")
    git(["add", TOOL_NAME, "README.md"], wt)
    if not git(["diff", "--cached", "--quiet"], wt, check=False).returncode:
        print("[trouble_log] 写しは最新")
        return 0
    git(["commit", "-q", "-m", "困りごとのログ: スクリプトの写しと README を更新"], wt)
    print("[trouble_log] 写しを更新してコミットした。GitHub に送るには: trouble_log.py push")
    return 0


def cmd_commit(a: argparse.Namespace) -> int:
    """手で編集した（status を書き換えた、など）ものをコミットする。"""
    root = repo_root()
    wt = ensure_worktree(root)
    git(["add", "-A", LOG_DIR], wt)
    if not git(["diff", "--cached", "--quiet"], wt, check=False).returncode:
        print("[trouble_log] 変更が無い")
        return 0
    git(["commit", "-q", "-m", a.message or "困りごとのログを更新"], wt)
    print("[trouble_log] コミットした。GitHub に送るには: trouble_log.py push")
    return 0


def cmd_push(a: argparse.Namespace) -> int:
    root = repo_root()
    wt = ensure_worktree(root)
    ahead = git(["rev-list", "--count", f"{REMOTE}/{LOG_BRANCH}..HEAD"], wt, check=False)
    for attempt in range(3):
        out = git(["push", "-u", REMOTE, f"HEAD:{LOG_BRANCH}"], wt, check=False)
        if out.returncode == 0:
            n = f"（{ahead.stdout.strip()} 件のコミット）" if ahead.returncode == 0 else "（初めて送った）"
            print(f"[trouble_log] GitHub に送った{n}: {LOG_BRANCH}")
            return 0
        # ほかの人が先に送っていた: 取り込んでからもう一度（ファイルは別なので、ぶつからない）
        print("[trouble_log] ほかの人の書き込みを取り込んで、送り直す")
        sync(wt, root)
    print(f"[trouble_log] 送れなかった:\n{out.stderr}")
    return 1


def _entries(root: Path) -> list[tuple[str, dict[str, str], str]]:
    wt = ensure_worktree(root)
    sync(wt, root)
    out = []
    for p in sorted((wt / LOG_DIR).glob("*.md")):
        text = p.read_text(encoding="utf-8", errors="replace")
        out.append((p.name, parse_meta(text), text))
    return out


def cmd_list(a: argparse.Namespace) -> int:
    root = repo_root()
    rows = _entries(root)
    if a.status:
        rows = [r for r in rows if r[1].get("status") == a.status]
    if a.category:
        rows = [r for r in rows if r[1].get("category") == a.category]
    if not rows:
        print("[trouble_log] まだ 1 件も無い")
        return 0
    print(f"{'番号':>4}  {'日時':<16}  {'状態':<6}  {'重さ':<7}  {'分類':<8}  {'書いた人':<12}  題名")
    for i, (name, m, _) in enumerate(rows, 1):
        print(f"{i:>4}  {m.get('date', ''):<16}  {m.get('status', ''):<6}  {m.get('severity', ''):<7}  "
              f"{m.get('category', ''):<8}  {m.get('author', '')[:12]:<12}  {m.get('title', name)}")
    counts: dict[str, int] = {}
    for _, m, _ in rows:
        counts[m.get("status", "?")] = counts.get(m.get("status", "?"), 0) + 1
    print(f"[trouble_log] {len(rows)} 件（{counts}）。1 件を見るには: trouble_log.py show <番号>")
    return 0


def cmd_show(a: argparse.Namespace) -> int:
    root = repo_root()
    rows = _entries(root)
    if not 1 <= a.number <= len(rows):
        print(f"[trouble_log] 番号は 1〜{len(rows)}")
        return 2
    name, _, text = rows[a.number - 1]
    print(f"# ファイル: {worktree_dir(root) / LOG_DIR / name}\n")
    print(text)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("init", help="ログのブランチを作る（最初の 1 回だけ。ふつうは不要）").set_defaults(func=cmd_init)
    p = sub.add_parser("add", help="1 件書いてコミットする（引数が無ければ質問する）")
    p.add_argument("--title", default="")
    p.add_argument("--category", choices=list(CATEGORIES))
    p.add_argument("--severity", choices=list(SEVERITIES))
    p.add_argument("--what", default="", help="何をしようとしたか")
    p.add_argument("--happened", default="", help="何が起きたか")
    p.add_argument("--expected", default="", help="期待していたこと")
    p.add_argument("--workaround", default="", help="どう回避したか")
    p.add_argument("--command", default="", help="再現のコマンド")
    p.add_argument("--error", default="", help="エラーの文")
    p.add_argument("--error-file", default="", help="エラーの入ったログのファイル（最後の 4000 文字を入れる）")
    p.add_argument("--author", default="", help="書いた人（既定は git の user.name）")
    p.set_defaults(func=cmd_add)
    p = sub.add_parser("commit", help="手で編集した（status を書き換えた、など）ものをコミットする")
    p.add_argument("-m", "--message", default="")
    p.set_defaults(func=cmd_commit)
    sub.add_parser("push", help="コミットしたものを GitHub に送る").set_defaults(func=cmd_push)
    sub.add_parser("update-tool", help="ログ専用のブランチに置いた、このスクリプトの写しと README を新しくする"
                   ).set_defaults(func=cmd_update_tool)
    p = sub.add_parser("list", help="一覧（全員の分。GitHub の最新を取り込んでから）")
    p.add_argument("--status", choices=list(STATUSES))
    p.add_argument("--category", choices=list(CATEGORIES))
    p.set_defaults(func=cmd_list)
    p = sub.add_parser("show", help="1 件を表示する（番号は list の番号）")
    p.add_argument("number", type=int)
    p.set_defaults(func=cmd_show)
    a = ap.parse_args()
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
