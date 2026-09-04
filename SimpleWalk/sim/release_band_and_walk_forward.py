#!/usr/bin/env python
"""MuJoCo環境はデフォルトで elastic band(骨盤をz=1m付近に吊るす仮想バンド、
config.yaml の ENABLE_ELASTIC_BAND: True)が有効なため、地面に足がつかず
「浮いている」ように見える。lerobot 側の UnitreeG1 実装にはこれを解除する経路が
無いため、ここでは MuJoCo の内部オブジェクトに直接アクセスして
elastic_band.enable = False にし、実際に地面へ降りて前進コマンドで水平移動するかを
pelvis の qpos(x,y,z) で確認する。

このスクリプトはシミュレーション専用。実機ではそのまま動かない。
  - [シミュレーション専用]: get_inner_sim(), pelvis_pos(), elastic_band の解除、
    UnitreeG1Config(is_simulation=True) の指定。いずれも MuJoCo 内部の状態
    （robot.sim_env や mj_data.qpos）に依存しており、実機では robot.sim_env が
    None になるため使えない。elastic_band 自体、実機には存在しない
    （実機では人間が支える／自立スタンドを使うのが対応する安全確保の代わり）。
  - [実機でも共通]: robot.connect()/disconnect() と、run() の中の
    robot.send_action({"remote.ly": ...}) によるコマンド送信ループ。この部分は
    UnitreeG1 の通信レイヤーの話であり、is_simulation=False + robot_ip=<G1のIP>
    にすれば同じコードで実機にもコマンドを送れる（歩行結果の確認方法だけは、
    実機では pelvis_pos() の代わりに目視や実機側のセンサーで行う必要がある）。
"""
import os
import sys
import time

import numpy as np

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot.robots.unitree_g1.g1_utils import default_remote_input


def get_inner_sim(robot):
    """[シミュレーション専用] MuJoCo の生データ（mj_model/mj_data）と elastic_band を持つ、
    実体の DefaultEnv を取得する。

    UnitreeG1.sim_env は gym ラッパーで、実体の DefaultEnv はさらに1段ネストした
    .simulator.sim_env にある。この階層は lerobot 側の unitree_g1.py の disconnect() 実装
    （image_publish_process を止める箇所）から辿って特定したもので、公開 API ではない。
    HF Hub 側の実装が変われば壊れうる。
    """
    se = robot.sim_env
    sim = getattr(se, "simulator", None)
    inner = getattr(sim, "sim_env", None) if sim is not None else None
    return inner


def pelvis_pos(inner):
    """[シミュレーション専用] pelvis（骨盤）のワールド座標 (x, y, z) を取得する。歩行の有無を
    目視ではなく座標の変化そのもので確認するための値。実機にはこの絶対位置を直接読む
    手段が無いため、実機では代わりに目視やロボット側のセンサー・カメラで確認する。

    骨盤は mjcf 上で type="free" の floating_base_joint を持ち、これがモデル内で最初の
    自由度のため qpos[0:3] がそのまま pelvis の (x, y, z) になる（qpos[3:7] はクォータニオン）。
    """
    return np.array(inner.mj_data.qpos[0:3], dtype=float)


def run(robot, action_overrides, duration_s, inner, control_hz=50):
    """action_overrides で指定した remote.* コマンド（例: {"remote.ly": 0.5}）を duration_s
    秒間送り続け、区間終了時点の pelvis 位置を返す。

    [実機でも共通] robot.send_action() を 50Hz で呼び続けるループ自体は、シミュレーション
    かどうかによらない UnitreeG1 の通信レイヤーの話。robot.send_action() は
    controller_input を書き換えるだけで、実際に関節へ反映するのはバックグラウンドの
    _controller_thread（GrootLocomotionController, 50Hz）なので、コントローラ側の反映と
    （シミュレーションなら）mujoco の物理ステップが追いつくだけの時間を確保している。
    [シミュレーション専用] 戻り値の pelvis_pos(inner) だけはシミュレーション専用。実機で
    このループだけ流用する場合は、戻り値をそのまま使わず削るか、実機側の確認手段に
    差し替えること。
    """
    action = default_remote_input()
    action.update(action_overrides)
    dt = 1.0 / control_hz
    for _ in range(int(duration_s * control_hz)):
        t0 = time.perf_counter()
        robot.send_action(dict(action))
        elapsed = time.perf_counter() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)
    return pelvis_pos(inner)


def main():
    """MuJoCo上のG1に接続し、elastic band(仮想吊りバンド)を解除したうえで前進コマンドを
    5秒間送り、実際に地面の上を移動するかを pelvis 座標の変化で検証する。
    """
    # [シミュレーション専用のconfig] is_simulation=True で MuJoCo 環境（HF Hub の
    # lerobot/unitree-g1-mujoco、trust_remote_code 経由で自動ダウンロード）に接続する。
    # 実機で動かす場合はここを is_simulation=False, robot_ip=<G1のIP> に変更する
    # （実機はスペック上 MuJoCo を使わないので sim_env は常に None になる）。
    # GrootLocomotionController は歩行方策(ONNX)を nepyope/GR00T-WholeBodyControl_g1 から
    # 自動取得し、接続後はバックグラウンドスレッドが 50Hz で立位バランス〜歩行の制御を
    # 回し続ける。この部分（コントローラの指定・接続そのもの）は実機でも共通。
    cfg = UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController")
    robot = UnitreeG1(cfg)
    print("Connecting...", flush=True)  # [実機でも共通]
    robot.connect()
    time.sleep(1.0)  # reset() 直後の姿勢が安定するまでの猶予

    # [シミュレーション専用] ここから disconnect() までの elastic_band 関連の一連の処理
    # （get_inner_sim / pelvis_pos / elastic_band.enable の操作）は、実機では
    # robot.sim_env が None のためすべて意味を持たない。実機では inner は必ず None になり
    # 下の if で即 abort する（実機用に流用するなら、この abort より下のブロックは丸ごと
    # 削除して、後述の「実機でも共通」の run() 呼び出しだけを残せばよい）。
    inner = get_inner_sim(robot)
    print("inner sim_env type:", type(inner), flush=True)
    if inner is None or not hasattr(inner, "elastic_band"):
        # get_inner_sim() が辿っている内部階層は非公開 API なので、環境更新で構造が
        # 変わった場合はここで早期に気づけるようにしている。
        print("elastic_band not found at expected path; aborting release test.", flush=True)
        robot.disconnect()
        os._exit(1)

    # elastic_band は骨盤を point=[0,0,1] 付近へ kp_pos=10000 という強いPD力で
    # 吊り上げ続ける仮想サスペンション（本家 unitree_mujoco 由来、config.yaml の
    # ENABLE_ELASTIC_BAND: True でデフォルト有効）。lerobot 側の UnitreeG1 実装には
    # これを解除する経路が無いため、ここでは inner オブジェクトへ直接書き込んで無効化する。
    # 実機にはそもそも elastic_band という概念が無く、代わりに人間が支える／自立スタンドを
    # 使うのが安全確保の対応にあたる。
    print(
        f"elastic_band before: enable={inner.elastic_band.enable} length={inner.elastic_band.length} "
        f"point={inner.elastic_band.point}",
        flush=True,
    )
    print("pelvis pos while suspended:", pelvis_pos(inner), flush=True)

    inner.elastic_band.enable = False
    # 解除直後は骨盤のクォータニオンが一瞬ゼロノルムになることがあり、対策前の
    # ElasticBand.Advance() ではここで例外を投げて物理演算スレッドが黙って死んでいた
    # （SimpleWalk/sim/patch_mujoco_elastic_band.py で修正済み。未パッチだとこの settle 中に
    # 「pelvis pos after release+settle」の値がスポーン位置のまま一切変わらなくなる）。
    print("elastic_band released (enable=False). Letting it settle for 2s (idle)...", flush=True)
    pos_settled = run(robot, {}, duration_s=2.0, inner=inner)
    print("pelvis pos after release+settle (idle):", pos_settled, flush=True)

    # [実機でも共通] remote.ly は GrootLocomotionController 内で cmd[0](前進速度 vx)に
    # 直結する（lerobot/robots/unitree_g1/controllers/gr00t_locomotion.py 参照）。
    # send_action({"remote.ly": 0.5}) というコマンド自体は実機でも同じ意味を持つ。
    # ただし戻り値の pos_after_fwd（pelvis_pos）はシミュレーション専用の確認手段。
    print("Sending forward command (remote.ly=0.5) for 60s...", flush=True)
    pos_after_fwd = run(robot, {"remote.ly": 0.5}, duration_s=60.0, inner=inner)
    print("pelvis pos after forward cmd:", pos_after_fwd, flush=True)

    disp = pos_after_fwd - pos_settled
    dist_xy = float(np.linalg.norm(disp[:2]))
    print(f"\n=== SUMMARY ===", flush=True)
    print(f"suspended pelvis z (band有効時)   : {pelvis_pos(inner)[2]:.3f} (直後の再取得。参考値)", flush=True)
    print(f"settled  pelvis pos (band解除, 静止): {pos_settled}", flush=True)
    print(f"final    pelvis pos (前進60秒後)     : {pos_after_fwd}", flush=True)
    print(f"displacement xyz = {disp}", flush=True)
    print(f"horizontal distance = {dist_xy:.4f} m over 60s", flush=True)
    # 0.05m は静止時のノイズ（バランス制御による微小な重心移動）を上回るよう
    # 実測値（静止時は数cm未満、歩行成功時は5秒で1.8m前後）から経験的に決めた閾値。
    print("MOVED_FORWARD" if dist_xy > 0.05 else "DID_NOT_MOVE_SIGNIFICANTLY", flush=True)

    # [実機でも共通] disconnect() はシミュレーション/実機どちらでも同じ後片付けの入り口。
    print("Disconnecting...", flush=True)
    try:
        robot.disconnect()
    except Exception as e:  # noqa: BLE001
        print(f"disconnect() error (ignored): {e}", flush=True)
    print("DONE", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    # disconnect() 内の subscribe_thread が非デーモンかつ join タイムアウト後も生き残った
    # 場合、通常の終了処理を待つとプロセスが終了せず残り続けることがあるため、
    # 後片付けが済んだここで確実に終了させる。
    os._exit(0)


if __name__ == "__main__":
    main()
