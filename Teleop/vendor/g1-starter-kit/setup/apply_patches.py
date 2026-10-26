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

  state-watchdog  rt/lowstate が 0.2 秒途切れたら rt/lowcmd の送信を止め、
                  プロセスを強制終了する（fail-closed）。
                  ⚠️ これが無いと、通信が切れている間も 250Hz で送信が続き、
                  復帰した瞬間に「途絶前に凍結した目標」へ全 29 関節が
                  飽和トルクで跳ねる。2026-09-30 に有線 LAN の抜線→再接続で
                  実際に発生した（脚 kp=300 / 腕 kp=80）。腕の速度制限も
                  比較対象の実測値が凍結するため無効化される。
                  再接続での自動復帰はさせない（人間が状態を確認して再起動する）。

  record-loco-cmd [G1_Hackason 改変] 手なし（--ee 無し）でも、--motion のコントローラ操作では
                  機体へ送った歩きの指令（前後・横・旋回）を記録の body に残す。
                  上流は dex1／brainco のコントローラ版の分岐にしか記録の行が無く、
                  手なしは else に落ちて空になる。機体へ送る Move(...) は手の種類に
                  関係なく毎周期送られるので、「機体は歩くのに記録は空」になっていた。
                  ⚠️ これが無いと歩きを含むデモを録っても、行動に歩きが入らない。

  stick-pause     [G1_Hackason 改変] 両スティックを 0.5 秒押し続けると一時停止／再開する。
                  一時停止中は歩きの指令を 0 にし、腕を止めた瞬間の実測の角度で固める
                  （脱力しない・腕を勝手に下ろさない）。再開直後の 2 秒は腕を 2 rad/s に絞る。
                  上流は両スティック押し込みで loco_wrapper.Damp() を呼ぶが、LocoClientWrapper に
                  Damp は無く（Enter_Damp_Mode のみ）AttributeError で teleop ごと落ち、後始末の
                  ctrl_dual_arm_go_home で腕を勝手に下ろしていた。
                  ⚠️ record-loco-cmd の loco_cmd_sent を使うので対で当てる（自動で補完する）。
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
    # [G1_Hackason 改変] 上流 45ead40 には無い。手なしの G1 で歩きを含むデモを録るため。
    "record-loco-cmd": {
        "desc": "手なしでも --motion の歩きの指令を記録に残す",
        "file": "teleop_hand_and_arm.py",
        # 1) 機体へ送る歩きの指令を変数 loco_cmd_sent に取る（記録と stick-pause が同じ値を使う）。
        # 2) 記録の分岐の最後（どの手にも当たらない else）で、その値を記録する。
        "plain": [
            ("                loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * 0.3,\n"
             "                                  -tele_data.left_ctrl_thumbstickValue[0] * 0.3,\n"
             "                                  -tele_data.right_ctrl_thumbstickValue[0]* 0.3)\n",
             "                # PATCH(record-loco-cmd): 送る指令を変数に取り、記録にも同じ値を使う。\n"
             "                loco_cmd_sent = [-tele_data.left_ctrl_thumbstickValue[1] * 0.3,\n"
             "                                 -tele_data.left_ctrl_thumbstickValue[0] * 0.3,\n"
             "                                 -tele_data.right_ctrl_thumbstickValue[0]* 0.3]\n"
             "                loco_wrapper.Move(*loco_cmd_sent)\n",
             "loco_wrapper.Move(*loco_cmd_sent)"),
            ("                else:\n"
             "                    left_ee_state = []\n"
             "                    right_ee_state = []\n"
             "                    left_hand_action = []\n"
             "                    right_hand_action = []\n"
             "                    current_body_state = []\n"
             "                    current_body_action = []\n",
             "                else:\n"
             "                    left_ee_state = []\n"
             "                    right_ee_state = []\n"
             "                    left_hand_action = []\n"
             "                    right_hand_action = []\n"
             "                    # PATCH(record-loco-cmd): 手なしでも --motion のコントローラ操作では、\n"
             "                    # 機体へ実際に送った歩きの指令（前後・横・旋回）を記録に残す。上流は\n"
             "                    # dex1／brainco のコントローラ版にしか無く、手なしは空になっていた。\n"
             "                    if args.input_mode == \"controller\" and args.motion:\n"
             "                        current_body_state = arm_ctrl.get_current_motor_q().tolist()\n"
             "                        current_body_action = list(loco_cmd_sent)\n"
             "                    else:\n"
             "                        current_body_state = []\n"
             "                        current_body_action = []\n",
             "PATCH(record-loco-cmd): 手なしでも"),
        ],
    },
    # [G1_Hackason 改変] 上流 45ead40 には無い。record-loco-cmd の loco_cmd_sent を使うので対で当てる。
    "stick-pause": {
        "desc": "両スティック 0.5 秒長押しで一時停止／再開（脱力も腕の帰還もしない）",
        "file": "teleop_hand_and_arm.py",
        "plain": [
            # 1) 長押しを数える状態を主な周期の前に用意する。
            #    挿入型なので、置換後に残らないよう元のコメント行も書き換える。
            ("        # main loop. robot start to follow VR user's motion\n"
             "        while not STOP:\n",
             "        # PATCH(stick-pause): 両スティック長押しの一時停止の状態（high level control で使う）\n"
             "        STICK_PAUSE_HOLD_S = 0.5        # この秒数押し続けたら切り替える\n"
             "        STICK_RESUME_SLOW_S = 2.0       # 再開直後に腕の速さを絞る秒数\n"
             "        stick_paused = False\n"
             "        stick_pause_since = None        # 両スティックを押し始めた時刻\n"
             "        stick_pause_armed = True        # 離すまで次の切り替えをしない\n"
             "        stick_pause_q = None            # 止めた瞬間の腕の実測の角度\n"
             "        stick_pause_tauff = None\n"
             "        stick_resume_slow_until = None\n"
             "        # main loop. robot start to follow VR user's motion（stick-pause の状態は直前で用意）\n"
             "        while not STOP:\n",
             "stick_pause_armed = True        # 離すまで"),
            # 2) 両スティック押し込みの Damp() を、長押しで一時停止／再開の切り替えに替える。
            ("                # command robot to enter damping mode. soft emergency stop function\n"
             "                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:\n"
             "                    loco_wrapper.Damp()\n",
             "                # PATCH(stick-pause): 元は両スティック押し込みで loco_wrapper.Damp() を呼んで\n"
             "                # いたが、LocoClientWrapper に Damp は無く（Enter_Damp_Mode のみ）、\n"
             "                # AttributeError で teleop ごと落ちて後始末で腕を勝手に下ろしていた。\n"
             "                # 脱力もさせない方針なので一時停止に替える: 0.5 秒押し続けると、歩きを 0 に\n"
             "                # して腕を今の実測の角度で固める。もう一度 0.5 秒押すと再開する。\n"
             "                stick_now = time.time()\n"
             "                if not (tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick):\n"
             "                    stick_pause_since = None\n"
             "                    stick_pause_armed = True\n"
             "                elif stick_pause_since is None:\n"
             "                    stick_pause_since = stick_now\n"
             "                elif stick_pause_armed and stick_now - stick_pause_since >= STICK_PAUSE_HOLD_S:\n"
             "                    stick_pause_armed = False\n"
             "                    stick_paused = not stick_paused\n"
             "                    if stick_paused:\n"
             "                        stick_pause_q = arm_ctrl.get_current_dual_arm_q().copy()\n"
             "                        stick_pause_tauff = None\n"
             "                        logger_mp.warning(\"⏸  一時停止: 歩きを止め、腕を今の角度で固めました（両スティック 0.5 秒で再開）\")\n"
             "                    else:\n"
             "                        # 操作者の手が止めた位置から離れていても腕が飛ばないよう、しばらく絞る\n"
             "                        arm_ctrl.set_arm_velocity_limit(2.0)\n"
             "                        stick_resume_slow_until = stick_now + STICK_RESUME_SLOW_S\n"
             "                        logger_mp.warning(\"▶  再開: 2 秒間は腕を 2 rad/s に絞って追従します\")\n"
             "                if stick_resume_slow_until is not None and stick_now >= stick_resume_slow_until:\n"
             "                    arm_ctrl.set_arm_velocity_limit(30.0)\n"
             "                    stick_resume_slow_until = None\n",
             "stick_paused = not stick_paused"),
            # 3) 一時停止中は歩きの指令を 0 にして送る（record-loco-cmd の記録にも 0 が残る）。
            ("                                 -tele_data.right_ctrl_thumbstickValue[0]* 0.3]\n"
             "                loco_wrapper.Move(*loco_cmd_sent)\n",
             "                                 -tele_data.right_ctrl_thumbstickValue[0]* 0.3]\n"
             "                if stick_paused:\n"
             "                    loco_cmd_sent = [0.0, 0.0, 0.0]  # PATCH(stick-pause): 一時停止中は歩かない\n"
             "                loco_wrapper.Move(*loco_cmd_sent)\n",
             "loco_cmd_sent = [0.0, 0.0, 0.0]"),
            # 4) 一時停止中は IK の解を使わず、止めた瞬間の実測の角度を送り続ける（力を足さない）。
            ("            logger_mp.debug(f\"ik:\\t{round(time_ik_end - time_ik_start, 6)}\")\n"
             "            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n",
             "            logger_mp.debug(f\"ik:\\t{round(time_ik_end - time_ik_start, 6)}\")\n"
             "            # PATCH(stick-pause): 一時停止中は止めた瞬間の実測の角度を送り続ける。重力の\n"
             "            # 補償は止めた周期の値を使い回す。記録の行動（sol_q）もこの値になる。\n"
             "            if stick_paused:\n"
             "                if stick_pause_tauff is None:\n"
             "                    stick_pause_tauff = sol_tauff\n"
             "                sol_q, sol_tauff = stick_pause_q, stick_pause_tauff\n"
             "            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)\n",
             "sol_q, sol_tauff = stick_pause_q, stick_pause_tauff"),
        ],
    },
    "state-watchdog": {
        "desc": "lowstate が 0.2 秒途切れたら送信を停止して強制終了（fail-closed）",
        "file": "robot_control/robot_arm.py",
        # 挿入型なので before には「適用後には存在しなくなる範囲」を含めてある
        # （アンカー行だけにすると適用後も残り、未適用と誤判定して二重挿入される）。
        "plain": [
            # (1) 強制終了に必要な標準モジュール。素の robot_arm.py には無い。
            ("import numpy as np\nimport threading\nimport time\nfrom enum import IntEnum",
             "import numpy as np\nimport os\nimport signal\nimport sys\n"
             "import threading\nimport time\nfrom enum import IntEnum",
             "import signal"),
            # (2) 途絶を検出したときの処理をモジュール直下に置く。
            ('kTopicLowState = "rt/lowstate"\n\nG1_29_Num_Motors = 35',
             '''kTopicLowState = "rt/lowstate"

# PATCH(state-watchdog): rt/lowstate が途切れた状態で rt/lowcmd を送り続けると、
# 通信が復帰した瞬間に「途絶前に凍結した目標」へ全関節が飽和トルクで跳ねる。
#
# 実機で発生（2026-09-30）: teleop 稼働中に有線 LAN が抜けた。teleop は
# 鮮度を見ずに 250Hz で送信を続け、再接続と同時に脚 kp=300 / 腕 kp=80 のまま
# 全 29 関節が一斉に最大トルクで動いて大きな音を出した。
# 腕の clip_arm_q_target() による速度制限も、比較対象 get_current_dual_arm_q()
# が途絶時点で凍結した実測値になるため「偏差ゼロ」と誤判定して無効化される。
#
# そのため「送信を止める」だけでは不十分（再接続で再開してしまう）。
# 検出したらプロセスごと落とし、人間が状態を確認してから再起動させる。
STATE_WATCHDOG_TIMEOUT_S = 0.2


def _state_watchdog_fail_closed(age_s):
    """rt/lowstate 途絶を検出したら、送信を止めてプロセスごと強制終了する。

    再接続による自動復帰はさせない（意図的な fail-closed）。
    """
    message = ("[state-watchdog] rt/lowstate が %.3f 秒途切れました "
               "(しきい値 %.3f 秒)。rt/lowcmd の送信を停止し強制終了します。"
               "再接続しても自動復帰しません。ロボットの状態を確認してから "
               "起動し直してください。" % (age_s, STATE_WATCHDOG_TIMEOUT_S))
    try:
        logger_mp.error(message)
    except BaseException:
        pass
    try:
        print(message, file=sys.stderr, flush=True)
    except BaseException:
        pass
    # 子プロセスを残すと 8012 などを掴んだままになり、次回の起動を妨げる。
    try:
        import multiprocessing
        for child in multiprocessing.active_children():
            try:
                child.kill()
            except BaseException:
                pass
    except BaseException:
        pass
    try:
        me = os.getpid()
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open("/proc/" + entry + "/stat", encoding="utf-8") as handle:
                    ppid = int(handle.read().rsplit(")", 1)[1].split()[1])
            except BaseException:
                continue
            if ppid == me:
                try:
                    os.kill(int(entry), signal.SIGKILL)
                except BaseException:
                    pass
    except BaseException:
        pass
    os._exit(1)


G1_29_Num_Motors = 35''',
             "_state_watchdog_fail_closed"),
        ],
        # (3) 受信時刻の記録 / (4) 送信直前の鮮度判定。G1 の 3 クラスだけに当てる
        #     （safe-startup と同じ範囲。H1/H2/R1 は本キットの対象外）。
        "scoped": [
            (anchor,
             "                self.lowstate_sub_ready = True\n"
             "            time.sleep(0.002)",
             "                self.lowstate_sub_ready = True\n"
             "                # PATCH(state-watchdog): 受信時刻。送信側の鮮度判定に使う。\n"
             "                self.lowstate_last_time = time.time()\n"
             "            time.sleep(0.002)",
             "self.lowstate_last_time = time.time()")
            for anchor in ("class G1_29_ArmController",
                           "class G1_29_Arm_Internal_Dex1_Controller",
                           "class G1_23_ArmController")
        ] + [
            (anchor,
             "            self.msg.crc = self.crc.Crc(self.msg)\n"
             "            self.lowcmd_publisher.Write(self.msg)",
             "            # PATCH(state-watchdog): lowstate 途絶中は送信しない。\n"
             "            # 届かない指令を送り続けると再接続時に跳ねるため、\n"
             "            # ここで打ち切って強制終了する（fail-closed）。\n"
             "            _wd_last = getattr(self, \"lowstate_last_time\", None)\n"
             "            if _wd_last is not None:\n"
             "                _wd_age = time.time() - _wd_last\n"
             "                if _wd_age > STATE_WATCHDOG_TIMEOUT_S:\n"
             "                    _state_watchdog_fail_closed(_wd_age)\n"
             "\n"
             "            self.msg.crc = self.crc.Crc(self.msg)  # PATCH(state-watchdog)\n"
             "            self.lowcmd_publisher.Write(self.msg)",
             "_state_watchdog_fail_closed(_wd_age)")
            for anchor in ("class G1_29_ArmController",
                           "class G1_29_Arm_Internal_Dex1_Controller",
                           "class G1_23_ArmController")
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
            # [G1_Hackason 改変] 複数行の置換元でも表示が崩れないよう、最初の行だけを見出しにする。
            label = before.strip().splitlines()[0].split("(")[0][:44]
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
    # [G1_Hackason 改変] stick-pause は record-loco-cmd が作る loco_cmd_sent の上に当たる。
    # 当てるときは record-loco-cmd を先に、外すときは stick-pause を先にする。
    if "stick-pause" in names and "record-loco-cmd" not in names and not args.revert:
        print("  注意: stick-pause は record-loco-cmd の上に当たるため自動的に追加します。")
        names = names + ["record-loco-cmd"]
    if "record-loco-cmd" in names and "stick-pause" not in names and args.revert:
        print("  注意: record-loco-cmd を外す前に stick-pause を外すため自動的に追加します。")
        names = names + ["stick-pause"]
    names = [n for n in PATCHES if n in names]      # 当てる順は PATCHES の並び
    if args.revert:
        names = list(reversed(names))               # 外す順は逆
    sys.exit(run(names, args.dry_run, args.revert))


if __name__ == "__main__":
    main()
