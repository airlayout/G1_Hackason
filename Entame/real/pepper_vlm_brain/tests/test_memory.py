from brain.memory import Memory
from brain.schemas import Decision, Action, Observation


def decision(action="LOOK", direction="RIGHT", target="PERSON"):
    return Decision(action=action, direction=direction, target=target, distance="UNKNOWN", confidence=0.8)


def test_initial_context_is_independent_copy():
    memory = Memory()
    context = memory.context()
    assert context["last_seen_person_direction"] == "UNKNOWN"
    context["last_seen_person_direction"] = "LEFT"
    assert memory.context()["last_seen_person_direction"] == "UNKNOWN"


def test_seen_then_disappeared_retains_last_direction():
    memory = Memory()
    memory.update(decision("FOUND", "CENTER"), observation=observed(direction="CENTER"))
    memory.update(decision("LOOK", "RIGHT"), observation=observed())
    assert memory.person_visible_last_frame
    memory.update(decision("SEARCH", "RIGHT"), observation=observed(False))
    assert memory.last_action == Action.SEARCH
    assert memory.last_seen_person_direction == "RIGHT"
    assert not memory.person_visible_last_frame
    assert memory.frames_since_person_seen == 1


def test_search_does_not_refresh_person_sighting():
    memory = Memory()
    memory.update(decision(), observation=observed())
    for _ in range(4):
        memory.update(decision("SEARCH"), observation=observed(False))
    assert memory.frames_since_person_seen == 4
    assert not memory.person_visible_last_frame
    assert memory.last_seen_person_direction == "RIGHT"


def test_reappearance_resets_gap_and_direction():
    memory = Memory()
    memory.update(decision(), observation=observed())
    memory.update(decision("SEARCH"), observation=observed(False))
    memory.update(decision("LOOK", "LEFT"), observation=observed(direction="LEFT"))
    assert memory.person_visible_last_frame
    assert memory.frames_since_person_seen == 0
    assert memory.last_seen_person_direction == "LEFT"


def test_failed_perception_does_not_invent_disappearance():
    memory = Memory()
    memory.update(decision(), observation=observed())
    memory.update(decision("WAIT", "UNKNOWN", "NONE"), perception_valid=False)
    assert memory.last_action == Action.WAIT
    assert memory.person_visible_last_frame
    assert memory.frames_since_person_seen == 0
    assert memory.last_seen_person_direction == "RIGHT"


def test_unknown_new_direction_preserves_known_direction():
    memory = Memory()
    memory.update(decision(), observation=observed())
    memory.update(decision(direction="UNKNOWN"), observation=observed(direction="UNKNOWN"))
    assert memory.last_seen_person_direction == "RIGHT"


def test_nonperson_target_not_seen():
    memory = Memory()
    memory.update(decision("FOUND", "CENTER", "PLUSHIE"), observation=observed(False))
    assert not memory.person_visible_last_frame
    assert memory.last_seen_person_direction == "UNKNOWN"


def test_wait_with_visible_person_target_does_not_erase_sighting():
    memory = Memory()
    memory.update(decision("WAIT", "CENTER", "PERSON"), observation=observed(direction="CENTER"))
    assert memory.person_visible_last_frame
    assert memory.last_seen_person_direction == "CENTER"


def observed(visible=True, direction="RIGHT"):
    return Observation(person_visible=visible, person_count=int(visible),
                       person_direction=direction if visible else "UNKNOWN",
                       person_distance="UNKNOWN", blocking_obstacle="NONE",
                       person_transition="UNKNOWN")
