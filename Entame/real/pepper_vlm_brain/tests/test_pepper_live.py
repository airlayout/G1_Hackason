from types import SimpleNamespace

from run_vlm_pepper_live import (ReactionState, grounded_speech,
                                  map_direction_to_head_pose, should_react)


def result(direction="CENTER", target="PERSON", fact="holding phone",
           fact_ja="スマートフォンを持っている", speech="スマホですね。"):
    decision = SimpleNamespace(
        direction=SimpleNamespace(value=direction),
        target=SimpleNamespace(value=target),
        salient_fact=fact,
        salient_fact_ja=fact_ja,
        speech=speech,
    )
    return SimpleNamespace(decision=decision)


def test_new_person_reacts_and_exact_repeat_is_suppressed():
    state = ReactionState()
    look, speak, *_ = should_react(result("LEFT"), state, 100.0, 4.0, 0.08, 0.0, 1.0)
    assert look and speak
    state.last_target_present = True
    state.last_direction = "LEFT"
    state.last_salient_fact = "holding phone"
    state.last_speech = "スマホですね。"
    state.last_reaction_time = 100.0
    state.last_head_yaw = 0.15
    state.last_look_time = 100.0
    look, speak, *_ = should_react(result("CENTER"), state, 110.0, 4.0, 0.08, 0.15, 1.0)
    assert not look and not speak


def test_direction_and_fact_changes_bypass_speech_cooldown():
    state = ReactionState(True, "CENTER", "standing", "立っていますね。", 100.0, 0.0)
    look, speak, reasons, *_ = should_react(
        result("LEFT", fact="holding phone"), state, 101.0, 4.0, 0.08, 0.0, 1.0)
    assert look and speak
    assert "direction_changed" in reasons and "fact_changed" in reasons


def test_absence_never_reacts_and_reappearance_is_new_event():
    state = ReactionState(True, "LEFT", "holding phone", "スマホですね。", 100.0, 0.3)
    assert should_react(result(target="NONE"), state, 101.0, 4.0, 0.08, 0.3, 1.0)[:2] == (False, False)
    look, speak, reasons, *_ = should_react(result("RIGHT"), state, 102.0, 4.0, 0.08, 0.3, 1.0)
    assert look and speak and "new_person" in reasons


def test_repeated_left_tracks_current_yaw_and_center_holds():
    assert map_direction_to_head_pose("LEFT", 0.0, 0.12) == (0.15, 0.12, 0.15)
    assert map_direction_to_head_pose("LEFT", 0.15, 0.12) == (0.30, 0.12, 0.15)
    assert map_direction_to_head_pose("CENTER", 0.30, 0.12) == (0.30, 0.12, 0.0)
    assert map_direction_to_head_pose("RIGHT", -0.10, 0.12) == (-0.25, 0.12, -0.15)


def test_yaw_is_clamped_and_generic_speech_is_grounded():
    assert map_direction_to_head_pose("LEFT", 0.75, 0.1)[0] == 0.80
    assert map_direction_to_head_pose("RIGHT", -0.75, 0.1)[0] == -0.80
    text, grounded, rejected = grounded_speech(result(speech="おはようございます。" ).decision)
    assert text == "スマートフォンを持っていますね。"
    assert grounded and rejected
