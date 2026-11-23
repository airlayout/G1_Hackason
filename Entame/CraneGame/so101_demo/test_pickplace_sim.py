import struct
import time
import unittest

import sim_web

try:
    import pickplace_sim as ps
    HAVE = True
except Exception:          # mujoco が無い環境では飛ばす
    HAVE = False

AIM_X = 0.27 - 0.025         # SNACK_X + AIM_DX


def fallback_calibration():
    """実機の校正ファイルに左右されないよう、代用の範囲を明示して使う。"""
    return {j: dict(ps.FALLBACK_RANGE) for j in ps.ARM_JOINTS}


@unittest.skipUnless(HAVE, "mujoco が無い")
class PickPlaceTests(unittest.TestCase):
    def new(self, dx=0.0, dy=0.0):
        sim = ps.PickPlaceSim(fallback_calibration())
        if dx or dy:                  # 操作の狙いのずれ＝お菓子の位置のずれとして試す
            x, y, z = sim.snack_start
            sim.snack_start = (x + dx, y + dy, z)
            sim.reset()
        return sim

    def run_for(self, sim, seconds, **want):
        if want:
            sim.command(dict(op="set", **want))
        for _ in range(round(seconds / ps.TICK)):
            sim.advance()

    def grab_and_lift(self, sim, aim_height=0.010):
        self.run_for(sim, 1.5, opening=1.0, x=AIM_X, height=0.07)
        self.run_for(sim, 1.5, height=aim_height)
        self.run_for(sim, 1.0)
        self.run_for(sim, 1.5, opening=0.0)
        self.run_for(sim, 1.5, height=0.06)

    def test_scripted_pick_carry_and_drop_into_hole(self):
        sim = self.new()
        self.grab_and_lift(sim)
        self.assertTrue(sim.task["lifted"])
        self.run_for(sim, 2.0, pan=ps.PLACE_PAN)
        self.run_for(sim, 1.5, height=0.010)
        self.assertFalse(sim.task["got"], "掴んだままでは景品ゲットにしない")
        self.run_for(sim, 1.5, opening=1.0)
        self.run_for(sim, 1.5)
        self.assertTrue(sim.task["got"])
        self.assertFalse(sim.task["fallen"])
        self.assertLess(sim.state()["snack"]["pos"][2], ps.reach_map.Z_DESK - 0.1, "穴の下の床まで落ちている")

    def test_drop_into_hole_from_operator_aim_errors(self):
        for dpan in (-4.0, 4.0):
            sim = self.new()
            self.grab_and_lift(sim)
            self.run_for(sim, 2.0, pan=ps.PLACE_PAN + dpan)
            self.run_for(sim, 1.5, opening=1.0)
            self.run_for(sim, 1.5)
            self.assertTrue(sim.task["got"], dpan)

    def test_hole_is_in_the_desk_and_within_reach(self):
        sim = self.new()
        d = sim.describe()
        self.assertEqual(len(d["desk"]), 4)
        hx, hy = d["hole"]["center"][:2]
        for piece in d["desk"]:       # 穴の中心は、どの天板の箱にも入っていない
            (cx, cy, _), (ax, ay, _) = piece["center"], piece["half"]
            self.assertFalse(abs(hx - cx) < ax and abs(hy - cy) < ay)
        sim.command({"op": "set", "x": ps.SNACK_X + ps.AIM_DX, "height": 0.02, "pan": ps.PLACE_PAN})
        self.run_for(sim, 3.0)
        tip = sim.state()["tip"]
        self.assertLess(max(abs(tip[0] - hx), abs(tip[1] - hy)), ps.HOLE_HALF, "穴の上へ先端が届く")
        self.assertFalse(sim.clamped)

    def test_snack_pushed_off_desk_is_fallen_not_got(self):
        sim = ps.PickPlaceSim(fallback_calibration())
        adr = sim.model.jnt_qposadr[sim.model.body_jntadr[sim.snack_id]]
        sim.data.qpos[adr:adr + 3] = [0.9, 0.0, ps.reach_map.Z_DESK + 0.1]        # 机の外
        for _ in range(60):
            sim.advance()
        self.assertTrue(sim.task["fallen"])
        self.assertFalse(sim.task["got"])

    def test_grasp_holds_for_operator_aim_errors(self):
        for dx, dy in ((0.0, 0.0), (0.01, 0.005), (-0.01, -0.005), (0.0, 0.008), (0.0, -0.01)):
            for height in (0.006, 0.014):
                sim = self.new(dx, dy)
                self.grab_and_lift(sim, height)
                self.run_for(sim, 3.0)
                self.assertGreater(sim.state()["task"]["lift"], 0.02, (dx, dy, height))

    # --- アシスト（自動でつかむ・離す）
    def assisted(self, dx=0.0, dy=0.0):
        sim = self.new(dx, dy)
        sim.command({"op": "assist", "on": True})
        return sim

    def approach(self, sim):
        self.run_for(sim, 1.5, opening=1.0, x=AIM_X, height=0.07)
        self.run_for(sim, 1.5, height=0.03)               # つかめる距離まで下げただけ（床すれすれまで下げない）

    def test_assist_grabs_with_rough_aim_carries_and_drops_into_hole(self):
        for dx, dy in ((0.0, 0.0), (-0.015, 0.0), (0.012, 0.01), (0.0, -0.015)):
            sim = self.assisted(dx, dy)
            self.approach(sim)
            self.assertTrue(sim.state()["grabbable"], (dx, dy))
            self.run_for(sim, 0.5, opening=0.0)
            self.assertTrue(sim.state()["held"], (dx, dy))
            self.run_for(sim, 1.5, height=0.06)
            self.assertGreater(sim.state()["task"]["lift"], 0.02, (dx, dy))
            self.run_for(sim, 2.5, pan=ps.PLACE_PAN)           # 手は握ったまま。穴の上で止まれば、勝手に開いて落ちる
            self.run_for(sim, 2.0)
            self.assertTrue(sim.task["got"], (dx, dy))
            self.assertFalse(sim.state()["held"])

    def test_assist_releases_when_opened_and_is_off_by_default(self):
        sim = self.assisted()
        self.approach(sim)
        self.run_for(sim, 0.5, opening=0.0)
        self.assertTrue(sim.state()["held"])
        self.run_for(sim, 0.5, opening=1.0)
        self.assertFalse(sim.state()["held"])
        plain = self.new()
        self.assertFalse(plain.state()["assist"])
        self.approach(plain)
        self.run_for(plain, 0.5, opening=0.0)
        self.assertFalse(plain.state()["held"])

    def test_assist_off_lets_go_and_validates(self):
        sim = self.assisted()
        self.approach(sim)
        self.run_for(sim, 0.5, opening=0.0)
        self.assertTrue(sim.state()["held"])
        sim.command({"op": "assist", "on": False})
        self.assertFalse(sim.state()["held"])
        with self.assertRaises(ValueError):
            sim.command({"op": "assist", "on": "yes"})

    def test_assist_does_not_regrab_while_falling(self):
        sim = self.assisted()
        self.approach(sim)
        self.run_for(sim, 0.5, opening=0.0)
        self.run_for(sim, 1.0, height=0.06)
        self.run_for(sim, 2.5, pan=ps.PLACE_PAN)
        for _ in range(round(1.5 / ps.TICK)):
            sim.command({"op": "set", "opening": 0.0})        # 手は握りっぱなし
            sim.advance()
        self.assertTrue(sim.task["got"])

    # --- アシストの強さ（吸い寄せない・アームの位置を寄せる）と、穴の位置
    def test_hole_is_on_the_right_of_the_arm(self):
        sim = self.new()
        self.assertLess(sim.hole[1], -0.05, "穴はアームから見て右（-y）")
        self.assertLess(ps.PLACE_PAN, 0)

    def test_grab_does_not_pull_the_snack_to_the_gripper(self):
        sim = self.assisted()
        self.approach(sim)
        before = sim.state()["snack"]["pos"]
        self.run_for(sim, 0.5, opening=0.0)
        self.assertTrue(sim.state()["held"])
        after = sim.state()["snack"]["pos"]
        self.assertLess(max(abs(a - b) for a, b in zip(after, before)), 0.008, "つかんでも、お菓子を寄せない（その場で持つ）")

    def test_aim_assist_pulls_arm_toward_the_grasp_point_only_when_on(self):
        ideal = self.new(-0.015, 0.0).snack_start[0] + ps.AIM_DX
        on, off = self.assisted(-0.015, 0.0), self.new(-0.015, 0.0)
        for sim in (on, off):
            self.run_for(sim, 2.0, opening=1.0, x=AIM_X, height=0.05)
        self.assertAlmostEqual(off.goal["x"], AIM_X, places=3)
        self.assertLess(abs(on.goal["x"] - ideal), abs(off.goal["x"] - ideal) - 0.004, "狙いの位置へ寄る")
        self.assertAlmostEqual(on.want["x"], AIM_X, places=6, msg="人の目標そのものは書き換えない")

    def test_aim_assist_does_nothing_far_from_the_snack(self):
        sim = self.assisted()
        self.run_for(sim, 2.0, opening=1.0, pan=-40.0)          # お菓子から水平に 9 cm 以上離れた向き
        self.assertAlmostEqual(sim.goal["pan"], -40.0, places=1)

    def test_aim_assist_pulls_held_snack_toward_the_hole(self):
        sim = self.assisted()
        self.approach(sim)
        self.run_for(sim, 0.5, opening=0.0)
        self.run_for(sim, 1.5, height=0.06)
        self.assertTrue(sim.state()["held"])
        self.run_for(sim, 0.8, pan=ps.PLACE_PAN + 14.0)
        self.assertLess(sim.goal["pan"], ps.PLACE_PAN + 14.0 - 1.0, "穴の側へ寄る")

    # --- スタート・ストップ・リスタート
    def test_before_start_nothing_moves_and_commands_are_ignored_but_validated(self):
        sim = ps.PickPlaceSim(fallback_calibration(), running=False)
        before = sim.state()
        self.assertFalse(before["running"])
        sim.command({"op": "set", "pan": 20.0})
        sim.command({"op": "move", "pan": 1.0})
        for _ in range(30):
            sim.advance()
        after = sim.state()
        self.assertEqual((after["t"], after["goal"]), (before["t"], before["goal"]))
        self.assertFalse(after["task"]["started"])
        with self.assertRaises(ValueError):
            sim.command({"op": "set", "pan": "a"})
        sim.command({"op": "start"})
        self.assertTrue(sim.state()["running"])
        self.run_for(sim, 1.0, pan=20.0)
        self.assertGreater(sim.state()["goal"]["pan"], 5.0)

    def test_stop_freezes_time_and_timer_and_start_resumes(self):
        sim = self.new()
        sim.command({"op": "start"})
        self.run_for(sim, 1.0, pan=20.0)
        sim.command({"op": "stop"})
        frozen = sim.state()
        self.run_for(sim, 1.0, pan=-20.0)                  # 止まっている間の指令は反映しない
        still = sim.state()
        self.assertFalse(still["running"])
        self.assertEqual((still["t"], still["goal"], still["task"]["elapsed"]), (frozen["t"], frozen["goal"], frozen["task"]["elapsed"]))
        sim.command({"op": "start"})
        self.run_for(sim, 0.5)
        self.assertGreater(sim.state()["task"]["elapsed"], frozen["task"]["elapsed"])
        self.assertLess(sim.state()["task"]["elapsed"], frozen["task"]["elapsed"] + 1.0, "止めていた間は数えない")

    def test_restart_resets_and_runs_and_start_after_finish_starts_over(self):
        sim = self.new()
        self.grab_and_lift(sim)
        sim.command({"op": "stop"})
        sim.command({"op": "restart"})
        state = sim.state()
        self.assertTrue(state["running"])
        self.assertFalse(state["task"]["lifted"])
        self.assertAlmostEqual(state["snack"]["pos"][0], sim.snack_start[0], places=2)
        self.assertTrue(state["task"]["started"])
        sim.task["fallen"] = True                          # 課題が終わっているとき、スタートは最初から
        sim.command({"op": "start"})
        self.assertFalse(sim.task["fallen"])
        self.assertTrue(sim.state()["running"])

    def test_grip_does_not_crush_the_snack(self):
        sim = self.new()
        self.run_for(sim, 1.5, opening=1.0, x=AIM_X, height=0.07)
        self.run_for(sim, 1.5, height=0.010)
        self.run_for(sim, 2.5, opening=0.0)
        depth = max((-c.dist for c in sim.data.contact[:sim.data.ncon]
                     if sim.snack_id in (sim.model.geom_bodyid[c.geom1], sim.model.geom_bodyid[c.geom2])
                     and 0 not in (sim.model.geom_bodyid[c.geom1], sim.model.geom_bodyid[c.geom2])), default=0.0)
        self.assertLess(depth, 0.006, "ジョーがお菓子へ 6 mm 以上めり込んでいる")

    def test_snack_stays_still_until_touched(self):
        sim = self.new()
        start = sim.state()["snack"]["pos"]
        self.run_for(sim, 2.0, x=0.20, pan=-20.0)
        self.assertEqual([round(v, 3) for v in sim.state()["snack"]["pos"]], [round(v, 3) for v in start])
        self.assertFalse(sim.task["lifted"])

    def test_reach_limit_is_reported_and_not_overshot(self):
        sim = self.new()
        sim.command({"op": "set", "x": 0.18, "height": 0.20})      # 台座に最も近い位置では、先端は高さ 15 cm 台までしか届かない（実測）
        seen = False
        for _ in range(round(4.0 / ps.TICK)):
            sim.advance()
            seen = seen or sim.state()["clamped"]
        self.assertTrue(seen, "届かない所へ動かしている間に、届かない印が出る")
        self.assertLess(sim.goal["height"], 0.20)
        for _ in range(round(1.0 / ps.TICK)):             # 動かさなければ、印は消える
            sim.advance()
        self.assertFalse(sim.state()["clamped"])

    def test_restart_while_holding_keeps_the_snack_on_the_desk(self):
        """つかんだままリスタートしても、お菓子は机の上に戻って動かない（つかむ間だけ切る衝突を戻す。戻さないと机をすり抜けて落ち続け、画面から消える）。"""
        sim = self.new()
        sim.command({"op": "start"})
        sim.assist = True
        self.grab_and_lift(sim)
        self.assertTrue(sim.held, "前提: つかんでいる")
        sim.command({"op": "restart"})
        self.run_for(sim, 3.0)
        pos = sim.state()["snack"]["pos"]
        self.assertAlmostEqual(pos[2], sim.snack_start[2], delta=0.005, msg="お菓子が机の上に載っている")
        self.assertFalse(sim.task["fallen"])

    def test_tip_can_rise_over_the_snack_near_the_base(self):
        """台座に近くても、先端を机から 9 cm 以上上げられる（肩を後ろへ引ける）。"""
        sim = self.new()
        sim.command({"op": "start"})
        self.run_for(sim, 2.5, x=0.22, height=0.09)
        tip_height = sim.state()["tip"][2] - ps.reach_map.Z_DESK
        self.assertGreater(tip_height, 0.08)
        self.assertFalse(sim.state()["clamped"])

    def test_low_sweep_toward_the_snack_does_not_kick_it(self):
        """人の指令が低いまま横から近づいても、近づく間は先端がお菓子の上を通り、蹴飛ばさない。"""
        def swept(assist):
            sim = self.new()
            sim.command({"op": "start"})
            sim.assist = assist
            start = list(sim.state()["snack"]["pos"])
            self.run_for(sim, 2.0, x=0.30, height=0.0, pan=-25.0, opening=1.0)
            moved = 0.0
            sim.command({"op": "set", "pan": 25.0})            # お菓子の奥（x=0.30）を、低い指令のまま横へ通り抜ける
            for _ in range(round(3.0 / ps.TICK)):
                sim.advance()
                pos = sim.state()["snack"]["pos"]
                moved = max(moved, ((pos[0] - start[0]) ** 2 + (pos[1] - start[1]) ** 2) ** 0.5)
            return moved
        self.assertGreater(swept(False), 0.003, "前提: アシスト無しでは、低く通り抜けるとお菓子に当たる")
        self.assertLess(swept(True), 0.003, "アシストが先端を上げて、お菓子に当たらずに通り抜ける")

    def test_commands_are_validated(self):
        sim = self.new()
        for bad in ({"op": "bogus"}, {"op": "set", "x": "abc"}, {"op": "set", "x": float("nan")}, {}):
            with self.assertRaises((ValueError, KeyError, TypeError), msg=bad):
                sim.command(bad)
        sim.command({"op": "set", "x": 9.0})                 # 範囲外は弾かず、範囲の端へ丸める
        self.assertLessEqual(sim.want["x"], ps.reach_map.X_MAX)
        sim.command({"op": "move", "reach": 5})
        self.assertEqual(sim.rates["reach"], 1.0)

    def test_reset_restores_snack_and_task(self):
        sim = self.new()
        self.grab_and_lift(sim)
        sim.command({"op": "reset"})
        state = sim.state()
        self.assertFalse(state["task"]["lifted"])
        self.assertEqual(state["task"]["elapsed"], 0.0)
        self.assertAlmostEqual(state["snack"]["pos"][0], ps.SNACK_X, places=2)

    def test_move_stops_when_commands_stop(self):
        sim = self.new()
        sim.command({"op": "move", "reach": 1})
        self.run_for(sim, 0.2)
        moving = sim.goal["x"]
        self.run_for(sim, 1.0)
        self.assertAlmostEqual(sim.goal["x"], moving, delta=ps.REACH_SPEED * ps.MOVE_TIMEOUT)

    def test_state_matches_the_model_description(self):
        sim = self.new()
        self.assertEqual(len(sim.state()["geoms"]), len(sim.describe()["parts"]))
        self.assertTrue(all(len(g) == 12 for g in sim.state()["geoms"]))

    def test_mesh_bytes_layout(self):
        sim = self.new()
        for part in sim.describe()["parts"]:
            body = sim.mesh_bytes(part["mesh"])
            verts, faces = struct.unpack_from("<II", body)
            self.assertEqual(len(body), 8 + verts * 12 + faces * 12)
        with self.assertRaises(ValueError):
            sim.mesh_bytes(sim.model.nmesh)

    def test_tick_matches_the_service(self):
        self.assertAlmostEqual(sim_web.TICK, ps.TICK)


@unittest.skipUnless(HAVE, "mujoco が無い")
class SimServiceTests(unittest.TestCase):
    def setUp(self):
        self.service = sim_web.SimService(lambda: ps.PickPlaceSim(fallback_calibration()))

    def tearDown(self):
        self.service.close()

    def test_other_paths_are_not_taken(self):
        self.assertIsNone(self.service.get("/state"))
        self.assertIsNone(self.service.post("/command", b"{}"))

    def test_world_is_not_built_before_first_use(self):
        self.assertIsNone(self.service.sim)

    def test_bad_requests_are_400(self):
        for body in (b"not json", b"[]", b'{"op": "bogus"}', b'{"op": "set", "x": "a"}'):
            self.assertEqual(self.service.post("/sim/command", body)[0], 400, body)
        self.assertEqual(self.service.get("/sim/mesh/9999")[0], 400)
        self.assertEqual(self.service.get("/sim/mesh/abc")[0], 400)

    def test_time_runs_only_while_watched(self):
        self.assertEqual(self.service.get("/sim/state")[0], 200)
        time.sleep(0.4)
        running = self.service.get("/sim/state")[1]["t"]
        self.assertGreater(running, 0.1)
        self.service.seen = time.monotonic() - sim_web.IDLE_S - 1      # 見ている人がいなくなった
        time.sleep(0.5)
        frozen = self.service.sim.t
        time.sleep(0.4)
        self.assertEqual(self.service.sim.t, frozen)


if __name__ == "__main__":
    unittest.main()
