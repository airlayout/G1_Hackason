"""採点環境を通常配置または隔離したスナップショットから探す。"""

import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
CACHE = REPO_ROOT / "_local/button_press_yada/evaluation_env"
AGENT = Path(__file__).resolve().parent / "yada_agent"


def find_yada_root(explicit=None):
    root = Path(explicit).expanduser().resolve() if explicit else REPO_ROOT / "Button_Press/Yada"
    if not explicit and not (root / "contest/interface.py").is_file():
        current = CACHE / "current.json"
        if not current.is_file():
            raise FileNotFoundError("採点環境がありません。先に sim/prepare_yada.py を実行してください")
        root = CACHE / json.loads(current.read_text())["commit"] / "Button_Press/Yada"
    if not (root / "contest/interface.py").is_file():
        raise FileNotFoundError(f"Yada の採点環境がありません: {root}")
    return root
