"""段 2-2〜2-4 のドアの場面ファイル（DoorMan の data/tasks/<名前>/scenario_cfg/isaacsim.py）を作る。

元の door/scenario_cfg/isaacsim.py（ラッチあり・クローザーは DoorMan の既定 2.5〜4.5 N·m）を写し、
DoorSpawnerCfg の引数だけを差し替える。タスク名（＝ディレクトリ名）は env.config.task.name で選ぶ。

| タスク名 | 段 | 中身 |
|---|---|---|
| door（元のまま） | 2-2 | ラッチあり・クローザーは既定（どの等級よりも弱い） |
| door_en1154_g1〜g5 | 2-3 | ラッチあり・クローザーを EN 1154 の等級ごとに強く（閉じる力の最低値〜次の等級の最低値）。閉じた位置の押し付けは既定のまま |
| door_en1154_upto_g1〜g5 | 2-3 の学習用 | ラッチあり・クローザーの上限を DoorMan の既定の下限（2.5 N·m）〜等級 k の上限で台ごとにばらつかせる（易しい扉を混ぜる。合否は door_en1154_g<k> で見る） |
| door_en1154_g4g5 | 2-3 の学習用 | ラッチあり・クローザーの上限を等級 4 の下限〜等級 5 の上限（26〜54 N·m）でばらつかせる（今の方策がぎりぎり開ける等級 4 だけを混ぜる。2026-10-09 の実験 E3） |
| door_en1154_g4g5_82 | 2-3 の学習用 | ラッチあり・台の 8 割が等級 4（26〜37 N·m）・2 割が等級 5（37〜54 N·m）。台 i は i % 5 == 4 なら等級 5（2026-10-09 の案 2） |
| door_room4_wide | 2-4（旧） | UIS Room 4 を含む広い範囲（重さ・レバー・ラッチ・向き）。等級 1〜4・PD のクローザー |
| door_en1154_g1〜g4_shaped | 2-4 の評価用 | door_en1154_g<k> とクローザーの形だけが違う（実物の形、door_closer.py・0037） |
| door_en1154_g1〜g4_left | 2-4 の評価用 | door_en1154_g<k> と向きだけが違う（把手が機体の左＝door_open_lr=+1。実物の UIS Room 4 の向き）。鏡写し（0038）の確かめ |
| door_en1154_g13_shaped | 2-4a の学習用 | 等級 1〜3 を台ごとに 1/3 ずつ・実物の形のクローザー。寸法・重さ・レバー・向きは door_en1154_g<k> のまま（ばらつきは 1 つずつ足す） |
| door_en1154_g13_shaped_left | 2-4a の学習用 | door_en1154_g13_shaped と向きだけが違う（把手が機体の左＝実物の UIS Room 4 の向き。10-14 に写真で確かめた）。++env.config.mirror_left_doors=True と組で使う |
| door_room4_g13 | 2-4 の学習用 | door_room4_wide の範囲で、等級 1〜3 を台ごとに 1/3 ずつ・実物の形のクローザー・レバーの力の下限 0.5 N·m |

使い方:
    python make_door_scenarios.py <VIRAL>/gr00t/rl/data/tasks
"""

import sys
from pathlib import Path

# EN 1154 の閉じる力の最低値 [N·m]（等級 1〜6）。uniononline.co.uk・safelincs で確認（2026-10-06）
EN1154_MIN_CLOSING_MOMENT = {1: 9.0, 2: 13.0, 3: 18.0, 4: 26.0, 5: 37.0, 6: 54.0}

ORIGINAL_LATCH_LINE = "    build_latch=True,\n"
MULTI_SPAWNER_LINES = "multi_spawner_cfg = sim_utils.MultiAssetSpawnerCfg(\n    assets_cfg=[door_spawner_cfg] * 4096,\n"

# 閉じた位置での押し付け [N·m]。DoorMan の既定の扉と同じ範囲（目標 -10° × ばねを上限 2.5〜4.5 で切った値）。
# 上限（等級）をそのまま閉じた位置でも掛けると、押し付けがラッチの斜面を押してレバーが勝手に回る（2026-10-07）。
# そこで閉じた位置は既定と同じにし、開けるほど強くして等級の上限で頭打ちにする（door.py の hinge_closed_preload_range）
CLOSED_PRELOAD_LINE = "    hinge_closed_preload_range=(2.5, 4.5),  # 閉じた位置での押し付け [N·m]（DoorMan の既定と同じ）\n"

HEADER = """# G1_Hackason（{stage}）: G1_Hackason/Door/tools/make_door_scenarios.py が door/scenario_cfg/isaacsim.py から生成した。
# 手で直さず、生成し直すこと。{note}
"""


# DoorMan の既定のクローザーの上限の下限 [N·m]（door.py の hinge_drive_max_force_range の既定 (2.5, 4.5)）
DOORMAN_DEFAULT_MIN_FORCE = 2.5


def grade_args(grade: int) -> str:
    low = EN1154_MIN_CLOSING_MOMENT[grade]
    high = EN1154_MIN_CLOSING_MOMENT[grade + 1]
    return (
        "    build_latch=True,\n"
        f"    hinge_drive_max_force_range=({low}, {high}),  # EN 1154 等級 {grade} [N·m]\n"
        + CLOSED_PRELOAD_LINE
    )


ROOM4_WIDE_ARGS = """    build_latch=True,
    # UIS Room 4: 幅 0.94 m・レバー左・蝶番右・押して開ける・バックセット 64 mm・レバー 約 142 mm・ラッチの出 11.5 mm
    door_width=(0.8, 1.1),
    door_weight=(15.0, 100.0),
    hinge_drive_max_force_range=(9.0, 37.0),  # EN 1154 等級 1〜4（等級 5 の最低値まで）[N·m]
    hinge_closed_preload_range=(2.5, 4.5),  # 閉じた位置での押し付け [N·m]（DoorMan の既定と同じ）
    handle_drive_max_force_range=(0.2, 4.0),  # レバーを回す力 [N·m]（上限は G1 が手のひらで押し下げられる力の見込み）
    handle_length_range=(0.10, 0.16),
    handle_cross_section=["round", "square"],
    handle_max_angle_deg_range=(30.0, 60.0),
    latch_release_angle_deg_range=(30.0, 60.0),  # レバーの上限を超える分は上限で外れる
    latch_throw_range=(0.008, 0.015),
"""

# EN 1154 の効率の最小（閉鎖 ÷ 開く向き、等級 1〜5）。開く向きの上限 0〜60° は 閉鎖の最大 ÷ この値（26・36・47・62・83 N·m）
EN1154_MIN_EFFICIENCY = {1: 0.50, 2: 0.50, 3: 0.55, 4: 0.60, 5: 0.65}
# 効率の幅。実物は最小より良い（カム式は高い）ので、最小から 0.15 上までばらつかせる（等級 1・2 で 50〜65%）
EFFICIENCY_SPAN = 0.15


def shaped_closer_args(grade: int) -> str:
    """実物の形のクローザー（door_closer.py）を等級 grade の効率で使う行。0〜4° の閉鎖モーメントは hinge_drive_max_force_range"""
    low = EN1154_MIN_EFFICIENCY[grade]
    return (
        "    closer_shaped=True,  # 閉じ際が最大・90° で 1/3〜1/2・開ける向きは効率ぶん重い（env が毎物理ステップ力を掛ける）\n"
        f"    closer_efficiency_range=({low}, {low + EFFICIENCY_SPAN:.2f}),  # EN 1154 等級 {grade} の最小〜+{EFFICIENCY_SPAN}\n"
    )


ROOM4_REPLACEMENTS = {
    # レバーの高さ 0.85〜1.10 m・扉の端からの距離（バックセット）30〜100 mm
    "    door_handle_tblr=(0.95, 0.85, 0.08, 0.15),\n": "    door_handle_tblr=(1.10, 0.85, 0.03, 0.10),\n",
    # レバーの左右を両方（この扉はレバー左）
    '    door_open_lr=["right"],\n': '    door_open_lr=["left", "right"],\n',
}


def write(tasks_dir: Path, name: str, body: str) -> None:
    out = tasks_dir / name / "scenario_cfg"
    out.mkdir(parents=True, exist_ok=True)
    (out / "isaacsim.py").write_text(body, encoding="utf-8")
    print(f"[make_door_scenarios] {out / 'isaacsim.py'}")


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    tasks_dir = Path(sys.argv[1])
    src = (tasks_dir / "door" / "scenario_cfg" / "isaacsim.py").read_text(encoding="utf-8")
    if src.count(ORIGINAL_LATCH_LINE) != 1:
        raise ValueError("元の場面ファイルに build_latch=True の行が 1 つでない")

    for grade in range(1, 6):
        body = HEADER.format(stage="段 2-3", note=f"EN 1154 等級 {grade}。") + src.replace(
            ORIGINAL_LATCH_LINE, grade_args(grade)
        )
        write(tasks_dir, f"door_en1154_g{grade}", body)

    # 段 2-3 の学習用: 等級 k の扉だけで学習すると誰も扉を開けられず、扉を開ける押しがかえって減った
    # （10-08、等級 1 を 1 回学習して「レバーを下げて扉 2° 以上」が 27 → 4/128）。既定の強さから等級 k の上限までを混ぜる
    for grade in range(1, 6):
        high = EN1154_MIN_CLOSING_MOMENT[grade + 1]
        args = (
            "    build_latch=True,\n"
            f"    hinge_drive_max_force_range=({DOORMAN_DEFAULT_MIN_FORCE}, {high}),  # DoorMan の既定〜EN 1154 等級 {grade} の上限 [N·m]\n"
            + CLOSED_PRELOAD_LINE
        )
        body = HEADER.format(stage="段 2-3 の学習用", note=f"既定〜EN 1154 等級 {grade} を混ぜる。") + src.replace(ORIGINAL_LATCH_LINE, args)
        write(tasks_dir, f"door_en1154_upto_g{grade}", body)

    # 段 2-3 の実験 E3（2026-10-09）: 等級 5 だけだと扉が 1 度も開かず、成功から学べない（見込み B）。
    # 今の方策がぎりぎり開ける等級 4 を混ぜる。易しすぎる扉（既定〜等級 1）を混ぜると押しを弱めた（10-08）ので、等級 4 から上だけ
    args = (
        "    build_latch=True,\n"
        f"    hinge_drive_max_force_range=({EN1154_MIN_CLOSING_MOMENT[4]}, {EN1154_MIN_CLOSING_MOMENT[6]}),  # EN 1154 等級 4〜5 [N·m]\n"
        + CLOSED_PRELOAD_LINE
    )
    body = HEADER.format(stage="段 2-3 の学習用", note="EN 1154 等級 4〜5 を混ぜる（実験 E3）。") + src.replace(ORIGINAL_LATCH_LINE, args)
    write(tasks_dir, "door_en1154_g4g5", body)

    # 段 2-3 の案 2（2026-10-09）: E3 の一様な 26〜54 N·m は等級 5 が 6 割で、学習の中の成功が少なすぎて等級 4 まで失った。
    # 等級 4 の扉を 8 割・等級 5 の扉を 2 割にして、学習の中の成功を 5 割より上に保つ（見込み 0.8×0.81＋0.2×0.165 ≈ 0.68）。
    # Isaac Lab 3.0 は random_choice を無視するが、DoorMan（0006 の移植）が spawn_paths を env_i に明示するので台 i に
    # assets_cfg[i] が当たる。4:1 で並べると i % 5 == 4 の台が等級 5（くじではなく決まった割合）
    if src.count(MULTI_SPAWNER_LINES) != 1:
        raise ValueError("元の場面ファイルに assets_cfg=[door_spawner_cfg] * 4096 の行が 1 つでない")
    g5_low, g5_high = EN1154_MIN_CLOSING_MOMENT[5], EN1154_MIN_CLOSING_MOMENT[6]
    mixed_spawner_lines = (
        "# 等級 5 の扉（クローザーの上限だけが等級 4 の扉と違う）\n"
        f"door_spawner_cfg_g5 = door_spawner_cfg.replace(hinge_drive_max_force_range=({g5_low}, {g5_high}))  # EN 1154 等級 5 [N·m]\n"
        "\n"
        "multi_spawner_cfg = sim_utils.MultiAssetSpawnerCfg(\n"
        "    # 台 i に i 番目の扉（DoorMan の isaacsim.py が spawn_paths を env_i に明示する）。4:1 で並べて等級 5 を 2 割\n"
        "    assets_cfg=([door_spawner_cfg] * 4 + [door_spawner_cfg_g5]) * 820,\n"
    )
    body = (
        HEADER.format(stage="段 2-3 の学習用", note="台の 8 割が EN 1154 等級 4・2 割が等級 5（案 2）。")
        + src.replace(ORIGINAL_LATCH_LINE, grade_args(4)).replace(MULTI_SPAWNER_LINES, mixed_spawner_lines)
    )
    write(tasks_dir, "door_en1154_g4g5_82", body)

    body = src.replace(ORIGINAL_LATCH_LINE, ROOM4_WIDE_ARGS)
    for old, new in ROOM4_REPLACEMENTS.items():
        if body.count(old) != 1:
            raise ValueError(f"置き換え先が 1 つでない: {old!r}")
        body = body.replace(old, new)
    write(
        tasks_dir,
        "door_room4_wide",
        HEADER.format(stage="段 2-4", note="開いたまま止まる角度は exp の door_hold_open_angle_deg。") + body,
    )

    # 段 2-4 の評価用（2026-10-13）: door_en1154_g<k> とクローザーの形だけが違う扉。反復 34,950 の等級ごとの評価と並べて、
    # 形を実物に寄せると通り抜けの崖（26〜29 N·m）が動くかを見る（学習の前に）
    for grade in range(1, 5):
        body = HEADER.format(stage="段 2-4 の評価用", note=f"EN 1154 等級 {grade}・実物の形のクローザー。") + src.replace(
            ORIGINAL_LATCH_LINE, grade_args(grade) + shaped_closer_args(grade)
        )
        write(tasks_dir, f"door_en1154_g{grade}_shaped", body)

    # 段 2-4 の評価用（2026-10-13）: 把手が左の扉（実物の UIS Room 4 の向き）。door_en1154_g<k> と向きだけが違う。
    # 今の方策は 0/70 だったので、観測と行動の鏡写し（0038、++env.config.mirror_left_doors=True）で開けられるかを確かめる
    right_line = '    door_open_lr=["right"],\n'
    if src.count(right_line) != 1:
        raise ValueError('元の場面ファイルに door_open_lr=["right"] の行が 1 つでない')
    for grade in range(1, 5):
        body = HEADER.format(stage="段 2-4 の評価用", note=f"EN 1154 等級 {grade}・把手が左（door_open_lr=+1）。") + src.replace(
            ORIGINAL_LATCH_LINE, grade_args(grade)
        ).replace(right_line, '    door_open_lr=["left"],  # 把手が機体の左（実物の UIS Room 4 の向き）\n')
        write(tasks_dir, f"door_en1154_g{grade}_left", body)

    # 段 2-4a の学習用（2026-10-13）: door_room4_g13 は一度に 5 つ変えて今の方策が 0/127 だったので、まず段 2-4 の決まり
    # （実物の形のクローザー・等級 1〜3）だけを入れる。寸法・重さ・レバー・向きは段 2-3 の扉のまま。台 i は等級 i % 3 + 1
    shaped_spawner_lines = "".join(
        f"door_spawner_cfg_g{g} = door_spawner_cfg.replace(\n"
        f"    hinge_drive_max_force_range=({EN1154_MIN_CLOSING_MOMENT[g]}, {EN1154_MIN_CLOSING_MOMENT[g + 1]}),  # EN 1154 等級 {g} [N·m]\n"
        f"    closer_efficiency_range=({EN1154_MIN_EFFICIENCY[g]}, {EN1154_MIN_EFFICIENCY[g] + EFFICIENCY_SPAN:.2f}),\n"
        ")\n"
        for g in (2, 3)
    ) + (
        "\n"
        "multi_spawner_cfg = sim_utils.MultiAssetSpawnerCfg(\n"
        "    # 台 i に i 番目の扉（DoorMan の isaacsim.py が spawn_paths を env_i に明示する）。等級 1・2・3 を順に並べて 1/3 ずつ\n"
        "    assets_cfg=[door_spawner_cfg, door_spawner_cfg_g2, door_spawner_cfg_g3] * 1366,\n"
    )
    body = HEADER.format(stage="段 2-4a の学習用", note="EN 1154 等級 1〜3（台 i は i % 3 + 1）・実物の形のクローザー。") + src.replace(
        ORIGINAL_LATCH_LINE, grade_args(1) + shaped_closer_args(1)
    ).replace(MULTI_SPAWNER_LINES, shaped_spawner_lines)
    write(tasks_dir, "door_en1154_g13_shaped", body)

    # 段 2-4a の学習用・実物の向き（2026-10-14 のユーザー判断「向きを直す」）: UIS Room 4 は廊下（押す側）から見てレバーが左
    # （IMG_0946）＝door_open_lr=+1。今の方策は −1 しか学んでいないので、観測と行動の鏡写し（0038）と組で使う
    left_body = body.replace(right_line, '    door_open_lr=["left"],  # 把手が機体の左（実物の UIS Room 4 の向き）\n')
    left_body = left_body.replace("EN 1154 等級 1〜3（台 i は i % 3 + 1）・実物の形のクローザー。", "EN 1154 等級 1〜3（台 i は i % 3 + 1）・実物の形のクローザー・把手が左（door_open_lr=+1）。", 1)
    write(tasks_dir, "door_en1154_g13_shaped_left", left_body)

    # 段 2-4 の学習用（2026-10-13 のユーザー判断）: UIS Room 4 を含む広い範囲（door_room4_wide）で、クローザーは等級 1〜3 を
    # 実物の形で。台 i は i % 3 + 1 の等級（1/3 ずつ。等級ごとに効率の最小が違うので扉を 3 つ並べる）。等級 4 は評価だけ
    room4_line = "    hinge_drive_max_force_range=(9.0, 37.0),  # EN 1154 等級 1〜4（等級 5 の最低値まで）[N·m]\n"
    handle_line = "    handle_drive_max_force_range=(0.2, 4.0),  # レバーを回す力 [N·m]（上限は G1 が手のひらで押し下げられる力の見込み）\n"
    if ROOM4_WIDE_ARGS.count(room4_line) != 1 or ROOM4_WIDE_ARGS.count(handle_line) != 1:
        raise ValueError("ROOM4_WIDE_ARGS のクローザーかレバーの行が 1 つでない")
    g1_low, g1_high = EN1154_MIN_CLOSING_MOMENT[1], EN1154_MIN_CLOSING_MOMENT[2]
    g13_args = ROOM4_WIDE_ARGS.replace(
        room4_line,
        f"    hinge_drive_max_force_range=({g1_low}, {g1_high}),  # EN 1154 等級 1 [N·m]（等級 2・3 は下の door_spawner_cfg_g2・_g3）\n"
        + shaped_closer_args(1),
    ).replace(
        handle_line,
        # 元の DoorMan はレバーに 0.26 N·m を下げる向きに掛け続ける（door_open_homie.py の DOOR_HANDLE_TENSION）。
        # それより弱いレバーは機体が触れる前に自分で下がりラッチが外れる（コードからの推測）ので、下限を 0.5 にする
        "    handle_drive_max_force_range=(0.5, 4.0),  # レバーを回す力 [N·m]（下限はレバーに掛け続ける 0.26 N·m より上）\n",
    )
    g13_spawner_lines = "".join(
        f"door_spawner_cfg_g{g} = door_spawner_cfg.replace(\n"
        f"    hinge_drive_max_force_range=({EN1154_MIN_CLOSING_MOMENT[g]}, {EN1154_MIN_CLOSING_MOMENT[g + 1]}),  # EN 1154 等級 {g} [N·m]\n"
        f"    closer_efficiency_range=({EN1154_MIN_EFFICIENCY[g]}, {EN1154_MIN_EFFICIENCY[g] + EFFICIENCY_SPAN:.2f}),\n"
        ")\n"
        for g in (2, 3)
    ) + (
        "\n"
        "multi_spawner_cfg = sim_utils.MultiAssetSpawnerCfg(\n"
        "    # 台 i に i 番目の扉（DoorMan の isaacsim.py が spawn_paths を env_i に明示する）。等級 1・2・3 を順に並べて 1/3 ずつ\n"
        "    assets_cfg=[door_spawner_cfg, door_spawner_cfg_g2, door_spawner_cfg_g3] * 1366,\n"
    )
    body = src.replace(ORIGINAL_LATCH_LINE, g13_args).replace(MULTI_SPAWNER_LINES, g13_spawner_lines)
    for old, new in ROOM4_REPLACEMENTS.items():
        if body.count(old) != 1:
            raise ValueError(f"置き換え先が 1 つでない: {old!r}")
        body = body.replace(old, new)
    write(
        tasks_dir,
        "door_room4_g13",
        HEADER.format(
            stage="段 2-4 の学習用",
            note="EN 1154 等級 1〜3（台 i は i % 3 + 1）・実物の形のクローザー。開いたまま止まる角度は exp の door_hold_open_angle_deg。",
        )
        + body,
    )


if __name__ == "__main__":
    main()
