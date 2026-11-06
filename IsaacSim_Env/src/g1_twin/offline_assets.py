"""ネットに出られない機で、Isaac のアセットをローカルのキャッシュから読ませる。

## なぜ要るか

IsaacLab は床（GroundPlane）も G1 の USD も NVIDIA の S3 から取る。
デモ機（192.168.123.200）はインターネットに出られないので、そのままだと

    FileNotFoundError: Unable to open the usd file at path:
      https://omniverse-content-production.s3-.../Isaac/Environments/Grid/default_environment.usd

で落ちる（2026-09-17 実測）。一方この機には Isaac Sim が過去に取得した
アセットのキャッシュが ``~/NVIDIA/.isaac_asset_cache`` に 674MB 残っており、
**URL と同じ階層構造**（<root>/Isaac/Environments/... ）で並んでいる。
だからルートの文字列を差し替えるだけで、床も G1 も Warehouse もそのまま通る。

## なぜこの形か

参照先は carb 設定ではなく、``IsaacLab/apps/isaaclab.python.kit`` の
``persistent.isaac.asset_root.cloud`` を **import 時に文字列として読んだ**
モジュール変数（isaaclab/utils/assets.py の ``_parse_kit_asset_root``）。
したがって:

- ``--kit_args`` では変えられない（carb を見ていない）
- IsaacLab 側のファイルを書き換える必要は無い。import 時に決まるだけなので、
  **十分早いうちに Python 側で差し替えれば足りる**

IsaacLab の中は触らない方針（CLAUDE.md）なので後者を採る。

## ⚠️ import 順だけでは足りない（2026-09-17 実測）

``use_local_assets()`` を ``isaaclab.app`` より先に呼び、``isaaclab.utils.assets``
が 1 個しか無く（sys.modules のキーも id も同一）、``from_files_cfg`` が
**差し替えより後に** import されることまで確かめても、
``from_files_cfg.ISAAC_NUCLEUS_DIR`` は S3 のままだった。
Kit の拡張機構が絡むため、from-import で配られた先まで import 順では追えない。

したがって差し替えは 2 段構えにする:

1. ``use_local_assets()`` … モジュール変数を差し替える。
   呼び出し時に読む側（runner.py の ``assets_root`` 等）はこれで直る。
2. ``localize()`` … **使う直前に**文字列を読み替える。
   cfg のクラス属性へ焼き込まれてしまったもの（``GroundPlaneCfg.usd_path``、
   ``G1_CFG.spawn.usd_path``）はこちらで直す。
"""

from __future__ import annotations

import os
from pathlib import Path

# キャッシュの中で、ルートからの相対位置が分かっている 1 個。
# これがあるディレクトリを「アセットルート」と判定する。
_MARKER = Path("Isaac/Environments/Grid/default_environment.usd")

# Isaac Sim がダウンロードしたアセットを置く場所。バージョン番号（6.0 等）が
# 1 階層挟まるので、ここでは親までを指して下を走査する。
_CACHE_TAIL = Path(".isaac_asset_cache/Assets/Isaac")

# 明示的にクラウドを使いたいときの合図（ネットのある機で従来どおり動かす）。
_FORCE_CLOUD = "cloud"

# use_local_assets() が差し替えた前後のルート。localize() が読み替えに使う。
_cloud_root = ""
_local_root = ""


def _cache_parents() -> list[Path]:
    """アセットキャッシュがありそうな親ディレクトリを、確からしい順に返す。"""
    parents = []
    # env.sh が指す Isaac Sim の隣（この構成では ~/NVIDIA/env_isaaclab の隣）
    isaac_sim = os.environ.get("ISAAC_SIM", "")
    if isaac_sim:
        parents.append(Path(isaac_sim).parent / _CACHE_TAIL)
    parents.append(Path.home() / "NVIDIA" / _CACHE_TAIL)
    return parents


def find_local_asset_root() -> str:
    """ローカルのアセットルートを返す。見つからなければ空文字。

    Returns:
        ルートの絶対パス。``G1_ASSET_ROOT`` で明示された場合はそれを検証して返す。
    """
    explicit = os.environ.get("G1_ASSET_ROOT", "")
    if explicit == _FORCE_CLOUD:
        return ""
    if explicit:
        return explicit if (Path(explicit) / _MARKER).is_file() else ""

    for parent in _cache_parents():
        if not parent.is_dir():
            continue
        # 新しいバージョンから見る（6.0 と 5.0 が並んでいることがある）
        for root in sorted(parent.iterdir(), reverse=True):
            if (root / _MARKER).is_file():
                return str(root)
    return ""


def use_local_assets() -> str:
    """アセットの参照先をローカルのキャッシュへ差し替える。

    ⚠️ ``isaaclab.app`` / ``isaaclab.sim`` の import より先に呼ぶこと。

    Returns:
        差し替えたルート。差し替えなかったときは空文字。
    """
    global _cloud_root, _local_root

    root = find_local_asset_root()
    if not root:
        return ""

    # 遅延 import。この 1 個は stdlib しか使わないので Isaac Sim 起動前でも通る。
    import isaaclab.utils.assets as assets

    # 差し替える前の値を控える。localize() はこれを目印に読み替える。
    _cloud_root = assets.NUCLEUS_ASSET_ROOT_DIR
    _local_root = root

    # assets.py が持つ 4 個すべてを、同じ導出規則で置き直す。
    # 1 個でも取りこぼすとそこだけ S3 を見にいく。
    assets.NUCLEUS_ASSET_ROOT_DIR = root
    assets.NVIDIA_NUCLEUS_DIR = f"{root}/NVIDIA"
    assets.ISAAC_NUCLEUS_DIR = f"{root}/Isaac"
    assets.ISAACLAB_NUCLEUS_DIR = f"{root}/Isaac/IsaacLab"
    return root


def localize(path: str) -> str:
    """アセットのパスを、ローカルのキャッシュのものへ読み替える。

    ⚠️ **使う直前に通すこと。** cfg のクラス属性へ焼き込まれた URL は
    ``use_local_assets()`` では直らない（このファイル冒頭の説明を参照）。

    Args:
        path: 読み替えたいパス。クラウドの URL でなければ何もしない。

    Returns:
        ローカルのパス。対象外ならそのまま返す。
    """
    if not _local_root or not _cloud_root:
        return path
    if not path.startswith(_cloud_root):
        return path
    return _local_root + path[len(_cloud_root) :]
