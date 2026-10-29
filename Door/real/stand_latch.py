"""止まるはずの場面では、脚を立つ HOMIE に固定する掛け金（G1_Hackason の G2、2026-10-29）。

学習（VIRAL6、torch）と実機（Door/real、numpy・PC2 の Python 3.8）に、1 字も違わない同じファイルを置く。
  学習: gr00t/rl/envs/base_task/stand_latch.py（パッチ 0082）
  実機: G1_Hackason/Door/real/stand_latch.py（こちらが元。試験 tests/test_stand_latch.py が 2 つが同じことを確かめる）
numpy の bool 配列と torch の bool の tensor のどちらを渡しても動くよう、両方にある書き方（| & ~ < >=）だけを使う。
Python の bool をそのまま渡さない（~True は -2 になる）。torch は import しない（PC2 には無い）。

実機は段階（近づく・構える…）を知らないので、実機でも測れる 2 つの値だけで「止める場面」を決める:
  胴と把手の水平距離（ライダーの扉の位置から）と、扉の開いた角度（ライダー）。
入る: 距離が enter_m 未満、かつ扉が door_deg 未満（構える・つかむ・レバーの間）。
抜ける: 扉が door_deg 以上（押し開けが始まった）、または距離が exit_m より遠い（入る線より遠い所で抜けて、境目で揺れない）。
once_opened: その回で扉が一度 door_deg 以上に開いたら、もう入らない。押し開けの途中で扉が 10° 未満に戻ったときに
  止めない（10-29 に段 C・4,096 台の記録 256 回へ当てはめた: 段階 1〜3 の歩の 99.2% を止めるのはどちらも同じで、
  押し開けの段階で止まる台が 6 → 0）。
止めている間、呼ぶ側は歩く指令（前後・横・旋回）を 0 にする。HOMIE は指令の大きさが 0.5 以下なら立つ方策になる。
"""


def update_stand_latch(latched, opened, dist_xy, door_deg, enter_m=0.3, exit_m=0.5, door_deg_max=10.0,
                       once_opened=True):
    """掛け金を 1 周期進める。新しい (latched, opened) を返す。入力は書き換えない。

    latched・opened: [...] の bool（前の周期の値。回の始めは False）
    dist_xy: [...] 胴と把手の水平距離 [m]、door_deg: [...] 扉の開いた角度 [度]（向きは問わない大きさ）
    """
    opened = opened | (door_deg >= door_deg_max)
    enter = (dist_xy < enter_m) & (door_deg < door_deg_max)
    if once_opened:
        enter = enter & ~opened
    leave = (door_deg >= door_deg_max) | (dist_xy > exit_m)
    return (latched | enter) & ~leave, opened
