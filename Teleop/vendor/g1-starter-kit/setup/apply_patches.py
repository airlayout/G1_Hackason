#!/usr/bin/env python3
"""xr_teleoperate に必要な修正を当てる（冪等・取り消し可）。

xr_teleoperate は Unitree の公式リポジトリなので、fork せず「手元で当てる」形にしている。
patch ファイルではなく文字列置換にしてあるのは、上流が更新されて行番号がずれても
壊れないようにするため。既に当たっていれば何もしない。

    python3 setup/apply_patches.py --list        # 何を当てるか見る
    python3 setup/apply_patches.py --dry-run     # 当てずに結果だけ確認
    python3 setup/apply_patches.py               # 当てる
    python3 setup/apply_patches.py --revert      # 元に戻す
    python3 setup/apply_patches.py --only safe-startup   # 一部だけ当てる

当てる内容（すべて実機で確認済み）:

  safe-startup    起動直後に腕がゼロ姿勢へ飛ぶ速度を 2 rad/s に落とす。
                  ⚠️ これが無いと起動した瞬間に両腕が高速で振られる。
                  上流の既定は 30.0 rad/s（set_arm_velocity_limit の既定引数）。

  speed-restore   追従開始（r）の時点で速度を 30.0 rad/s へ戻す。
                  ⚠️ safe-startup と必ず対で当てること。上流から speed_gradual_max()
                  が削除され、初期化後に速度を戻す処理がどこにも無くなったため、
                  これが無いとテレオペ中もずっと 2 rad/s に制限されたままになる。
                  q で終了する場合は追従前なら 2 rad/s のまま帰還する（安全側）。

  head-reference  腕の制御基準を head_yaw → head_position にする。
                  既定では「頭の向き」で腕の前後左右が回るため、操作中に
                  よそを向くと左右がズレる。位置基準にすると向きに依存しない。

  record-camera   頭部カメラが無い環境で記録を開始すると、画像を保存しようとして
                  TypeError で teleop ごと落ちる。カメラ画像の中身も確認するようにする。
                  ⚠️ これが無いと「s を押した瞬間に落ちる」。

  motion-switcher-retry
                  モード判定 RPC のタイムアウトを 1.0 → 5.0 秒に伸ばし、リトライを入れる。
                  MotionSwitcherClient の RPC は「[ClientStub] send request error」で
                  散発的に失敗する。素の Enter_Debug_Mode は 1 回失敗しただけで例外を
                  握りつぶして (None, None) を返すため、実際には解除できる状況でも
                  「Enter debug mode: Failed」と誤表示され、原因の切り分けを妨げる。
"""

import argparse
import sys
from pathlib import Path

TELEOP = Path.home() / "xr_teleoperate" / "teleop"

# 各エントリは末尾に marker（適用済みを判断する固有の目印）を持つ。
#   scoped: (クラスのアンカー, 適用前, 適用後, marker)  … そのクラス内の 1 件だけ
#   plain : (適用前, 適用後, marker)                    … ファイル全体で全件
# marker を置換ごとに分けているのは、複数置換のうち一部だけ適用された状態を
# 「適用済み」と誤判定しないため。
PATCHES = {
    "safe-startup": {
        "desc": "起動時の腕の速度を 2 rad/s に落とす（安全）",
        "file": "robot_control/robot_arm.py",
        # __init__ 内の 1 箇所だけを狙う。set_arm_velocity_limit() の定義側は触らない。
        # 上流 845b25a で `self.arm_velocity_limit = 20.0` は
        # `self.set_arm_velocity_limit()`（既定 30.0）に置き換わった。
        "scoped": [
            (anchor,
             "        self.all_motor_q = None\n        self.set_arm_velocity_limit()",
             "        self.all_motor_q = None\n"
             "        # PATCH(safe-startup): 起動直後にゼロ姿勢へ飛ぶ速度。上流既定は 30.0。\n"
             "        # 追従開始（r）時に teleop_hand_and_arm.py 側で 30.0 へ戻す\n"
             "        # （speed-restore パッチ）。対で当てること。\n"
             "        self.set_arm_velocity_limit(2.0)",
             "self.set_arm_velocity_limit(2.0)")
            for anchor in ("class G1_29_ArmController",
                           "class G1_23_ArmController",
                           "class G1_29_Arm_Internal_Dex1_Controller")
        ],
    },
    "speed-restore": {
        "desc": "追従開始時に速度を 30 rad/s へ戻す（safe-startup と対）",
        "file": "teleop_hand_and_arm.py",
        # 挿入型なので before に「置換後には存在しなくなる範囲」を含める。
        # アンカー行だけにすると置換後もそれが残り、apply_plain が未適用と
        # 誤判定して二重挿入される。
        "plain": [
            ('        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")\n'
             "\n"
             "        head_img = None",
             '        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")\n'
             "        # PATCH(speed-restore): safe-startup で 2.0 rad/s に絞った上限を、\n"
             "        # 追従が始まるここで上流既定の 30.0 rad/s へ戻す。上流から\n"
             "        # speed_gradual_max() が削除されたため、これが無いとテレオペ中も\n"
             "        # 2.0 rad/s のままになる。q で抜けた場合(START=False)は戻さない。\n"
             "        if START:\n"
             "            arm_ctrl.set_arm_velocity_limit(30.0)\n"
             "\n"
             "        head_img = None",
             "arm_ctrl.set_arm_velocity_limit(30.0)"),
        ],
    },
    "head-reference": {
        "desc": "腕の制御基準を head_yaw → head_position（よそを向いてもズレない）",
        "file": "teleop_hand_and_arm.py",
        "plain": [
            ('arm_reference_mode="head_yaw"',
             '# PATCH(head-reference): 元は "head_yaw"。頭の向きで腕の前後左右が回ってしまい、\n'
             '                                     # 操作中によそを向くと左右がズレるため位置基準にする。\n'
             '                                     arm_reference_mode="head_position"',
             'arm_reference_mode="head_position"'),
        ],
    },
    "motion-switcher-retry": {
        "desc": "モード判定 RPC のタイムアウトを伸ばしてリトライする",
        "file": "utils/motion_switcher.py",
        "plain": [
            ("        self.msc.SetTimeout(1.0)",
             "        # PATCH(motion-switcher-retry): 元は 1.0。CheckMode の RPC が散発的に\n"
             "        # 失敗するため、余裕を持たせる。\n"
             "        self.msc.SetTimeout(5.0)",
             "self.msc.SetTimeout(5.0)"),
            ("""    def Enter_Debug_Mode(self):
        try:
            status, result = self.msc.CheckMode()
            while result['name']:
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)
            return status, result
        except Exception as e:
            return None, None""",
             """    def _check_mode_retry(self, attempts=6):
        # PATCH(motion-switcher-retry): CheckMode は「send request error」で散発的に
        # 失敗する。1 回で諦めず数回試す。判定できなければ (None, None)。
        last = (None, None)
        for _ in range(attempts):
            try:
                status, result = self.msc.CheckMode()
                if status == 0 and result is not None:
                    return status, result
                last = (status, result)
            except Exception:
                pass
            time.sleep(0.6)
        return last

    def Enter_Debug_Mode(self):
        # PATCH(motion-switcher-retry): 元は CheckMode を素で呼び、1 回でも例外が出ると
        # (None, None) を返していた。呼び元は status != 0 を「Failed」と表示するため、
        # 解除できる状況でも失敗に見えていた。リトライと打ち切り時刻を入れる。
        try:
            status, result = self._check_mode_retry()
            if result is None:
                return None, None
            deadline = time.time() + 20.0
            while result.get('name'):
                self.msc.ReleaseMode()
                time.sleep(1)
                status, result = self._check_mode_retry()
                if result is None:
                    return None, None
                if time.time() > deadline:
                    break
            return status, result
        except Exception as e:
            return None, None""",
             "_check_mode_retry"),
        ],
    },
    "record-camera": {
        "desc": "カメラが無い環境でも記録できるようにする（記録開始で落ちるのを防ぐ）",
        "file": "teleop_hand_and_arm.py",
        "plain": [
            ("if head_img is not None:",
             "if head_img is not None and head_img.bgr is not None:",
             "head_img.bgr is not None"),
            ("if left_wrist_img is not None:",
             "if left_wrist_img is not None and left_wrist_img.bgr is not None:",
             "left_wrist_img.bgr is not None"),
            ("if right_wrist_img is not None:",
             "if right_wrist_img is not None and right_wrist_img.bgr is not None:",
             "right_wrist_img.bgr is not None"),
        ],
    },
}


def load(path):
    if not path.exists():
        sys.exit(f"エラー: {path} がありません。\n"
                 "  xr_teleoperate が導入されていないようです。"
                 "setup/install_env.sh を先に実行してください。")
    return path.read_text(encoding="utf-8")


def apply_scoped(text, anchor, before, after, revert, marker):
    """anchor で始まるクラスブロック内の最初の 1 件だけを置換する。

    見つからなかったときは marker の有無で「既に適用済み」か「対象なし」を分ける。
    """
    src, dst = (after, before) if revert else (before, after)
    start = text.find(anchor)
    if start < 0:
        return text, "anchor-missing"
    end = text.find("\nclass ", start + 1)
    end = len(text) if end < 0 else end
    block = text[start:end]
    hit = block.find(src)
    if hit < 0:
        # 適用方向で marker があれば適用済み、戻す方向で marker が無ければ戻し済み
        done = (marker in block) if not revert else (marker not in block)
        return text, "already" if done else "not-found"
    abs_hit = start + hit
    return text[:abs_hit] + dst + text[abs_hit + len(src):], "applied"


def apply_plain(text, before, after, revert, marker):
    """ファイル全体で全件置換する。"""
    src, dst = (after, before) if revert else (before, after)
    if src not in text:
        done = (marker in text) if not revert else (marker not in text)
        return text, "already" if done else "not-found"
    return text.replace(src, dst), f"applied x{text.count(src)}"


def run(names, dry_run, revert):
    changed_files = {}
    results = []

    for name in names:
        spec = PATCHES[name]
        path = TELEOP / spec["file"]
        text = changed_files.get(path, load(path))

        for anchor, before, after, marker in spec.get("scoped", []):
            text, status = apply_scoped(text, anchor, before, after, revert, marker)
            results.append((name, f"{spec['file']} [{anchor.split()[-1]}]", status))
        for before, after, marker in spec.get("plain", []):
            text, status = apply_plain(text, before, after, revert, marker)
            label = before.split("(")[0][:44]
            results.append((name, f"{spec['file']} [{label}]", status))

        changed_files[path] = text

    verb = "戻す" if revert else "当てる"
    print(f"\n{'（dry-run）' if dry_run else ''}パッチを{verb}:\n")
    ng = 0
    for name, where, status in results:
        mark = {"applied": "✅", "already": "・ ", "not-found": "⚠️ ", "anchor-missing": "❌"}.get(
            status.split()[0] if status.startswith("applied") else status, "❓")
        if status.startswith("applied"):
            mark = "✅"
        note = {"already": "既に適用済み（何もしません）",
                "not-found": "対象が見つかりません（上流が変わった可能性）",
                "anchor-missing": "クラスが見つかりません（別バージョン？）"}.get(status, status)
        if status in ("not-found", "anchor-missing"):
            ng += 1
        print(f"  {mark} {name:<21} {where:<52} {note}")

    if dry_run:
        print("\n  --dry-run なのでファイルは変更していません。")
        return 1 if ng else 0

    # 一部だけ当たった中途半端な状態を残さない。全部通ったときだけ書き込む。
    if ng:
        print(f"\n  ⚠️ {ng} 件が当たらなかったため、**何も変更していません**。")
        print("     xr_teleoperate のバージョンが想定と違う可能性があります。")
        print("     手元の版に合わせて setup/apply_patches.py の対象文字列を見直してください。")
        return 1

    for path, text in changed_files.items():
        backup = path.with_suffix(path.suffix + ".orig")
        if not revert and not backup.exists():
            backup.write_text(load(path), encoding="utf-8")
            print(f"\n  バックアップ: {backup}")
        # 同一ディレクトリの一時ファイルに書いてから置き換える（途中で切れても壊れない）
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(path)
        print(f"  更新: {path}")

    print("\n  完了。")
    return 0


def main():
    p = argparse.ArgumentParser(description="xr_teleoperate に必要な修正を当てる")
    p.add_argument("--list", action="store_true", help="当てる内容の一覧を出す")
    p.add_argument("--dry-run", action="store_true", help="当てずに結果だけ見る")
    p.add_argument("--revert", action="store_true", help="元に戻す")
    p.add_argument("--only", action="append", choices=list(PATCHES),
                   help="指定したものだけ当てる（複数指定可）")
    args = p.parse_args()

    if args.list:
        print("\n当てられるパッチ:\n")
        for name, spec in PATCHES.items():
            print(f"  {name:<21} {spec['desc']}")
            print(f"  {'':<21} 対象: {spec['file']}")
        print(f"\n対象ディレクトリ: {TELEOP}")
        print("詳しい理由は setup/apply_patches.py の冒頭コメントに書いてあります。")
        return

    names = args.only or list(PATCHES)
    # safe-startup と speed-restore は対で当てないと、テレオペ中も 2 rad/s に
    # 制限されたままになる（あるいは起動時だけ速いまま）。片方指定は補完する。
    if ("safe-startup" in names) != ("speed-restore" in names):
        missing = "speed-restore" if "safe-startup" in names else "safe-startup"
        print(f"  注意: {missing} は対で当てる必要があるため自動的に追加します。")
        names = names + [missing]
    sys.exit(run(names, args.dry_run, args.revert))


if __name__ == "__main__":
    main()
