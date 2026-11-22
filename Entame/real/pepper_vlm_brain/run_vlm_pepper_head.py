"""One-image Qwen3-VL -> validated decision -> one safe Pepper head pose."""

import argparse
from pathlib import Path

from PIL import Image

from adapters.pepper.motion import PepperMotionAdapter, Ros2HeadTransport
from brain.decision import Brain
from brain.vlm import QwenVLM
from config import Config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--send", action="store_true")
    args = parser.parse_args()

    print(f"[VLM INPUT] {args.image}")
    engine = QwenVLM(Config(max_new_tokens=192))
    with Image.open(args.image) as image:
        result = Brain(engine).decide(image)
    print(f"[VLM RAW] {result.raw_response}")
    print(f"[DECISION] parsed={result.decision.model_dump_json(exclude_none=True)}")
    if result.fallback:
        print(f"[DECISION] error={result.error}")
        return 2
    action = result.decision.action.value
    target = result.decision.target.value
    direction = result.decision.direction.value
    observation = result.decision.observation
    salient_fact = result.decision.salient_fact
    speech = result.decision.speech
    print(f"[DECISION] target={target} direction={direction}")
    print(f"[OBSERVATION] {observation}")
    print(f"[SALIENT FACT] {salient_fact}")
    print(f"[SPEECH] {speech}")
    if (target != "PERSON" or direction not in {"LEFT", "CENTER", "RIGHT"}
            or not observation or not salient_fact or not speech):
        print("[PEPPER] person/direction/visual-reaction gate rejected decision")
        return 3
    if action not in {"LOOK", "FOUND", "SURPRISE"}:
        print(f"[PEPPER] mapped_action=NONE source_action={action}")
        return 4
    print(f"[PEPPER] mapped_action=LOOK source_action={action}")
    transport = Ros2HeadTransport(dry_run=not args.send)
    adapter = PepperMotionAdapter(transport)
    print(f"[PEPPER LOOK] direction={direction}")
    adapter.look_at(direction=direction.lower())
    transport.speak(speech)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
