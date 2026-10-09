from conftest import frame_of, make_person, pose

from pepperapp.events import EventConfig, EventDetector
from pepperapp.observations import DetectedObject


def _kinds(events):
    return [e.kind for e in events if e.kind != "person_visible"]


def test_person_appears_after_hold_and_leaves_after_hold():
    detector = EventDetector(EventConfig(appear_hold_s=0.5, leave_hold_s=1.5))
    person = frame_of(make_person(pose()))
    log = [(round(i * 0.1, 1), _kinds(detector.update(person, {}, i * 0.1))) for i in range(10)]
    appeared = [t for t, kinds in log if "person_appeared" in kinds]
    assert appeared == [0.5]
    left = [round(t, 1) for t in (1.0 + i * 0.1 for i in range(25))
            if "person_left" in _kinds(detector.update(frame_of(), {}, t))]
    assert left == [2.5]


def test_person_visible_every_frame_with_target_on_the_face():
    detector = EventDetector(EventConfig(appear_hold_s=0.0))
    events = detector.update(frame_of(make_person(pose())), {}, 0.0)
    visible = [e for e in events if e.kind == "person_visible"]
    assert len(visible) == 1
    assert visible[0].target.x == 250 / 640 and visible[0].target.y == 100 / 480


def test_short_blink_does_not_create_events():
    detector = EventDetector(EventConfig(appear_hold_s=0.5))
    detector.update(frame_of(make_person(pose())), {}, 0.0)
    assert _kinds(detector.update(frame_of(), {}, 0.2)) == []
    assert _kinds(detector.update(frame_of(make_person(pose())), {}, 0.4)) == []


def test_gesture_event_only_when_it_starts():
    detector = EventDetector()
    person = frame_of(make_person(pose(), track_id=3))
    first = detector.update(person, {3: frozenset({"wave"})}, 0.0)
    again = detector.update(person, {3: frozenset({"wave"})}, 0.1)
    stopped = detector.update(person, {3: frozenset()}, 0.2)
    restarted = detector.update(person, {3: frozenset({"wave"})}, 0.3)
    assert [e.track_id for e in first if e.kind == "wave"] == [3]
    assert all(e.kind != "wave" for e in again + stopped)
    assert [e.kind for e in restarted if e.kind == "wave"] == ["wave"]


def test_seen_label_rises_once():
    detector = EventDetector(EventConfig(seen_hold_s=0.5, unseen_hold_s=2.0))
    chair = DetectedObject("chair", (0, 240, 64, 480), 0.8)
    kinds = [k for i in range(10) for k in _kinds(detector.update(frame_of(objects=[chair]), {}, i * 0.1))]
    assert kinds == ["seen:chair"]


def test_seen_event_targets_the_largest_object():
    detector = EventDetector(EventConfig(seen_hold_s=0.0))
    small = DetectedObject("chair", (0, 0, 10, 10), 0.9)
    large = DetectedObject("chair", (320, 240, 640, 480), 0.6)
    event = next(e for e in detector.update(frame_of(objects=[small, large]), {}, 0.0)
                 if e.kind == "seen:chair")
    assert (event.target.x, event.target.y) == (0.75, 0.75)


def test_person_near_has_hysteresis():
    detector = EventDetector(EventConfig(appear_hold_s=0.0, near_height_ratio=0.6))
    def at(height, t):
        return _kinds(detector.update(frame_of(make_person(pose(), box=(0, 0, 100, height))), {}, t))
    assert "person_near" in at(300, 0.0)
    assert "person_near" not in at(300, 0.1)
    assert "person_near" not in at(260, 0.2)   # 0.54: still inside the hysteresis band
    at(200, 0.3)                                # 0.42: reset
    assert "person_near" in at(300, 0.4)


def test_box_iou():
    from pepperapp.observations import box_iou
    assert box_iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert box_iou((0, 0, 10, 10), (5, 0, 15, 10)) == 50 / 150
    assert box_iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
