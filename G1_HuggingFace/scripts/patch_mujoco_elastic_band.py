#!/usr/bin/env python
"""lerobot/unitree-g1-mujoco (HF Hub 上の trust_remote_code 環境) には、既知のバグがある:
ElasticBand.Advance() が骨盤の四元数がゼロノルムになる瞬間(band解除直後や reset 直後に
たまに起きる)に scipy.Rotation.from_quat() で例外を投げ、それを補足するのが
_subscribe_lowstate という物理演算(mujoco step)を回している非デーモンスレッドのため、
例外が出た瞬間そのスレッドが黙って死ぬ ―― 以後シミュレーションは完全に停止するが、
teleop ループやコントローラループは何事もなく動き続けるため気づきにくい。

このスクリプトは、キャッシュされた unitree_sdk2py_bridge.py にゼロノルムガードを
当てて安定化する。何度実行しても安全（既にパッチ済みなら何もしない）。
HF Hub 側でファイル内容が更新されると別ハッシュのblobになるため、その場合は
このスクリプトを再実行してパッチを当て直すこと。
"""
import sys
from pathlib import Path

MARKER = "Found zero norm quaternions"  # 例外メッセージの一部。パッチ済み判定に使う。
GUARD_MARKER = "np.linalg.norm(quat) < 1e-8"

OLD = '''        # --- Orientation PD control for torque ---
        quat = np.array([quat[1], quat[2], quat[3], quat[0]])  # reorder to [x,y,z,w] for scipy
        rot = scipy.spatial.transform.Rotation.from_quat(quat)
        rotvec = rot.as_rotvec()  # axis-angle error
        torque = -self.kp_ang * rotvec - self.kd_ang * ang_vel'''

NEW = '''        # --- Orientation PD control for torque ---
        quat = np.array([quat[1], quat[2], quat[3], quat[0]])  # reorder to [x,y,z,w] for scipy
        if np.linalg.norm(quat) < 1e-8:
            # A transient zero-norm quaternion can appear for a step or two right after
            # reset/(un)suspend; skip the orientation term instead of crashing the whole
            # sim-stepping thread (scipy.Rotation.from_quat raises on zero norm).
            torque = -self.kd_ang * ang_vel
        else:
            rot = scipy.spatial.transform.Rotation.from_quat(quat)
            rotvec = rot.as_rotvec()  # axis-angle error
            torque = -self.kp_ang * rotvec - self.kd_ang * ang_vel'''


def find_bridge_files() -> list[Path]:
    cache_root = Path.home() / ".cache" / "huggingface" / "hub"
    return sorted(cache_root.glob("models--lerobot--unitree-g1-mujoco/blobs/*"))


def looks_like_bridge_file(path: Path) -> bool:
    try:
        text = path.read_text(errors="ignore")
    except OSError:
        return False
    return "class ElasticBand" in text and "def Advance" in text


def main() -> int:
    candidates = [p for p in find_bridge_files() if looks_like_bridge_file(p)]
    if not candidates:
        print(
            "unitree_sdk2py_bridge.py が見つからない(まだ一度も lerobot-teleoperate 等で "
            "シミュレーションを起動していない可能性)。先に一度シミュレーションを起動して "
            "HF Hub からダウンロードさせてから再実行すること。",
            file=sys.stderr,
        )
        return 1

    changed = 0
    for path in candidates:
        text = path.read_text()
        if GUARD_MARKER in text:
            print(f"[skip] already patched: {path}")
            continue
        if OLD not in text:
            print(f"[warn] expected code block not found (upstream may have changed): {path}", file=sys.stderr)
            continue
        path.write_text(text.replace(OLD, NEW))
        print(f"[patched] {path}")
        changed += 1

    print(f"\n{changed} file(s) patched, {len(candidates)} candidate(s) checked.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
