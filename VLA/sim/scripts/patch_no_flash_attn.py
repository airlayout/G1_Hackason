"""flash-attn が無い環境（nvcc 無し等）でも GR00T N1.6 が読めるよう、Isaac-GR00T の clone にパッチを当てる。

当てる対象は 2 か所（どちらも冪等。何度実行しても二重には当たらない）:
  1. gr00t/model/modules/eagle_backbone.py   flash-attn が無ければ SDPA にフォールバックし、assert を外す
  2. gr00t/model/modules/nvidia/Eagle-Block2A-2B-v2/{config.json,modeling_eagle3_vl.py}
                                              attention 実装を flash_attention_2 から sdpa に変える

Isaac-GR00T は 7d5a455 で確認した。別のコミットでは文字列が一致せず、assert で止まる（壊しはしない）。
使い方: python3 -I patch_no_flash_attn.py [--groot-dir ~/Isaac-GR00T]   （既定は環境変数 GROOT_DIR、無ければ ~/Isaac-GR00T）
"""
import argparse
import os
import pathlib
import re

# 既に当てた環境で二重に当たらないよう、最初に当てたときの目印文字列をそのまま使う。
MARK = "# [omen-patch] flash-attn 未導入なら SDPA にフォールバック"


def patch_eagle_backbone(path: pathlib.Path) -> None:
    s = path.read_text()
    if MARK in s:
        print(f"[patch] {path.name}: already applied")
        return
    anchor = "        # Add attention kwargs\n"
    # extra_kwargs_sdpa は未使用。動作確認したパッチと同一にするため残している。
    fallback = (
        f"        {MARK}\n"
        "        if use_flash_attention:\n"
        "            from transformers.utils import is_flash_attn_2_available\n"
        "            if not is_flash_attn_2_available():\n"
        "                print('[patch] flash-attn なし: attn_implementation=sdpa で読み込む')\n"
        "                use_flash_attention = False\n"
        "                extra_kwargs_sdpa = True\n"
        f"{anchor}"
    )
    assert s.count(anchor) == 1, f"{path}: 挿入位置が 1 か所に定まらない（コミットが違う？）"
    s = s.replace(anchor, fallback)
    old_assert = (
        "            assert use_flash_attention, (\n"
        '                "nvidia/Eagle-Block2A-2B-v2 requires flash attention by default"\n'
        "            )\n"
    )
    assert s.count(old_assert) == 1, f"{path}: 外す assert が見つからない（コミットが違う？）"
    path.write_text(s.replace(old_assert, ""))
    print(f"[patch] {path.name}: applied")


def patch_eagle_model(model_dir: pathlib.Path) -> None:
    cfg = model_dir / "config.json"
    s = cfg.read_text()
    s2 = s.replace('"_attn_implementation": "flash_attention_2"', '"_attn_implementation": "sdpa"', 1)
    cfg.write_text(s2)
    print(f"[patch] config.json: {'changed' if s != s2 else 'unchanged'}")

    mdl = model_dir / "modeling_eagle3_vl.py"
    m = mdl.read_text()
    m2 = m.replace(
        'config.vision_config._attn_implementation = "flash_attention_2"',
        'config.vision_config._attn_implementation = "sdpa"',
    )
    m2, n = re.subn(
        r'\n\s+assert \(\n\s+config\.text_config\._attn_implementation == "flash_attention_2"\n'
        r'\s+\), f"Qwen[23] must use flash_attention_2 but got \{config\.text_config\._attn_implementation\}"',
        "",
        m2,
    )
    mdl.write_text(m2)
    print(f"[patch] modeling_eagle3_vl.py: asserts removed={n}, {'changed' if m != m2 else 'unchanged'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "--groot-dir",
        type=pathlib.Path,
        default=pathlib.Path(os.environ.get("GROOT_DIR", pathlib.Path.home() / "Isaac-GR00T")),
        help="Isaac-GR00T の clone（既定: $GROOT_DIR または ~/Isaac-GR00T）",
    )
    args = ap.parse_args()
    modules = args.groot_dir.expanduser() / "gr00t/model/modules"
    for p in (modules / "eagle_backbone.py", modules / "nvidia/Eagle-Block2A-2B-v2"):
        if not p.exists():
            raise SystemExit(f"[error] {p} が無い。--groot-dir を確認（Isaac-GR00T の clone か？）")
    patch_eagle_backbone(modules / "eagle_backbone.py")
    patch_eagle_model(modules / "nvidia/Eagle-Block2A-2B-v2")


if __name__ == "__main__":
    main()
