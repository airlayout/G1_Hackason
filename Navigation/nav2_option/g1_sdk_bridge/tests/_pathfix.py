"""テストからパッケージ化していない親ディレクトリのモジュールをimportするための小道具。

このプロトタイプは colcon workspace 化していない(Planning.md Q3、暫定判断)ため、
`protocol.py` 等はただのモジュールとして親ディレクトリに置かれている。
各テストファイルの先頭で `import _pathfix` するだけで済むようにする。
"""

import os
import sys

_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)
