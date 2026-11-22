"""Pepper 本体（NAOqi 2.5）の Wi-Fi を ALConnectionManager でつなぐ。

パスワードはこの端末で入力する（画面に出さず、どこにも保存しない）。
つながると connman がその Wi-Fi を覚え、次の起動からは自動でつなぐ。

    .venv/bin/python tools/connect_pepper_wifi.py --list
    .venv/bin/python tools/connect_pepper_wifi.py --ssid Physical_AI_5G
"""
import argparse
import getpass
import sys
import time

import qi

DEFAULT_IP = "192.168.123.99"  # wired link from OMEN
CONNECTED = ("ready", "online")
MAX_ANSWERS = 2  # connman asks again if an answer was rejected; do not loop forever


def as_dict(pairs) -> dict:
    return {key: value for key, value in pairs}


def wifi_services(manager) -> list[dict]:
    manager.scan()
    return [d for d in (as_dict(s) for s in manager.services()) if d.get("Type") == "wifi"]


def show(services: list[dict]) -> None:
    for d in sorted(services, key=lambda d: -int(d.get("Strength", 0))):
        print(f"{d.get('Name', '(hidden)')!r:28} state={d.get('State'):<13} "
              f"strength={d.get('Strength'):>3} saved={d.get('Favorite')} ipv4={d.get('IPv4', '')}")


def service_state(manager, service_id: str) -> dict:
    """The service may vanish from connman's list for a moment after a failed attempt."""
    try:
        return as_dict(manager.service(service_id))
    except RuntimeError:
        return {"State": "missing"}


def is_request_for(value, service_id: str) -> bool:
    if not isinstance(value, (list, tuple)):
        return False
    try:
        return as_dict(value).get("ServiceId") == service_id
    except (TypeError, ValueError):
        return False


def last_event(memory, key: str):
    """(timestamp, value) of the last time `key` was raised; (None, None) if never."""
    try:
        value, seconds, microseconds = memory.getTimestamp(key)
    except RuntimeError:
        return None, None
    return (seconds, microseconds), value


def connect(session, manager, service_id: str, passphrase: str | None, timeout_s: float) -> dict:
    """passphrase=None is a probe: when Pepper asks for the password, cancel instead of answering.

    ALMemory.subscriber() fails with "Invalid signature" between qi 3.1.5 and NAOqi 2.5, so the
    events are read by polling ALMemory. New events are told apart by their timestamp: the same
    request repeated has exactly the same value.
    """
    memory = session.service("ALMemory")
    keys = ("NetworkServiceInputRequired", "NetworkServiceStateChanged", "NetworkConnectStatus")
    seen = {key: last_event(memory, key)[0] for key in keys}
    seen_status = seen["NetworkConnectStatus"]
    asked, answers = False, 0
    future = manager.connect(service_id, _async=True)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for key in keys:
            stamp, value = last_event(memory, key)
            if stamp is None or stamp == seen[key]:
                continue
            seen[key] = stamp
            print(f"  event {key}: {value}", flush=True)
            # Answer only real requests for this network: the key is also reset to None.
            if key == "NetworkServiceInputRequired" and is_request_for(value, service_id):
                asked = True
                answers += 1
                if passphrase is None:
                    manager.disconnect(service_id)
                elif answers <= MAX_ANSWERS:
                    manager.setServiceInput([["ServiceId", service_id], ["Passphrase", passphrase]])
        if future.isFinished() and future.hasError():
            print(f"  connect() failed: {future.error()}", flush=True)
            break
        status = last_event(memory, "NetworkConnectStatus")
        if status[0] != seen_status and status[1] and status[1][1] is False:
            break  # connman gave up (wrong password, radio, ...): the reason is printed above
        state = service_state(manager, service_id).get("State")
        if state in CONNECTED or state == "failure" or (passphrase is None and asked):
            break
        time.sleep(0.2)
    return {**service_state(manager, service_id), "input_requested": asked}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--port", type=int, default=9559)
    parser.add_argument("--ssid", help="つなぐ Wi-Fi の名前。省略すると一覧だけ出す")
    parser.add_argument("--list", action="store_true", help="一覧だけ出す（何も変えない）")
    parser.add_argument("--probe", action="store_true",
                        help="パスワードを渡さず、Pepper がパスワードを求めてくるかだけ試す")
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    session = qi.Session()
    session.connect(f"tcp://{args.ip}:{args.port}")
    manager = session.service("ALConnectionManager")
    services = wifi_services(manager)
    if args.list or not args.ssid:
        show(services)
        return 0
    match = [d for d in services if d.get("Name") == args.ssid]
    if not match:
        print(f"{args.ssid!r} が見えません。--list で名前を確かめてください", file=sys.stderr)
        return 1
    passphrase = None
    if not args.probe:
        passphrase = getpass.getpass(f"{args.ssid} のパスワード（表示されません）: ")
        if not passphrase:
            print("パスワードが空なのでやめました", file=sys.stderr)
            return 1
    result = connect(session, manager, match[0]["ServiceId"], passphrase, args.timeout)
    print(f"state={result.get('State')} error={result.get('Error')!r} "
          f"input_requested={result['input_requested']} ipv4={result.get('IPv4', '')}")
    if args.probe:
        return 0 if result["input_requested"] else 1
    return 0 if result.get("State") in CONNECTED else 1


if __name__ == "__main__":
    sys.exit(main())
