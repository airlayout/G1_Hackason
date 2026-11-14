"""敬礼・指さしを 1 コマンドで動かす（actions/api.py の cli）。

    bash Button_Press/J1-gen/real/act.sh salute              # dry-run
    bash Button_Press/J1-gen/real/act.sh point --execute     # 実機に送る（人が打つ。確認モード）
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from actions.api import cli  # noqa: E402

if __name__ == "__main__":
    sys.exit(cli())
