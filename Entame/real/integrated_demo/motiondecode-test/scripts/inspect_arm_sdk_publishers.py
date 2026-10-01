#!/usr/bin/env python3
"""Receive-only ownership probe for rt/arm_sdk, rt/armsdk and Arm Action.

Despite the historical filename, this inspects the complete arm-control DDS
topology. It creates DataReaders and built-in discovery readers only.
"""
import argparse
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import time

from common import ROOT, write_json
from robot_transport import (
    ARM_ACTION_STATE_TOPIC, ARM_SDK_INPUT_TOPIC, ARM_SDK_TOPIC, ReadOnlyState,
    decode_arm_action_state, state_summary, validate_state,
)

LOWSTATE_TOPIC = "rt/lowstate"
OBSERVED_TOPICS = (ARM_SDK_TOPIC, ARM_SDK_INPUT_TOPIC, ARM_ACTION_STATE_TOPIC)
PROCESS_PATTERN = re.compile(
    r"cyclonedds|arm_sdk|armsdk|lowcmd|motion_switcher|run_real_reaction|play_g1_arms",
    re.IGNORECASE,
)


def _json_value(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [_json_value(x) for x in value]
    if isinstance(value, dict):
        return {str(k): _json_value(v) for k, v in value.items()}
    return repr(value)


def qos_detail(qos):
    policies, properties = [], {}
    for policy in qos:
        fields = {k: _json_value(v) for k, v in vars(policy).items()}
        policies.append({"kind": type(policy).__qualname__, "fields": fields})
        if "key" in fields and "value" in fields:
            properties[str(fields["key"])] = str(fields["value"])
    return {"policies": policies, "properties": properties, "repr": repr(qos)}


def liveliness_detail(qos):
    for policy in qos:
        if "Liveliness" in type(policy).__qualname__:
            lease = getattr(policy, "lease_duration", None)
            return {
                "kind": type(policy).__qualname__,
                "lease_duration_ns": int(lease) if lease is not None else None,
                "lease_is_effectively_infinite": lease == 9223372036854775807,
            }
    return {"kind": "UNKNOWN", "lease_duration_ns": None,
            "lease_is_effectively_infinite": False}


def interface_ipv4(interface):
    try:
        result = subprocess.run(
            ["ip", "-j", "address", "show", "dev", interface], check=True,
            capture_output=True, text=True, timeout=3,
        )
        return sorted(
            address["local"] for dev in json.loads(result.stdout)
            for address in dev.get("addr_info", [])
            if address.get("family") == "inet" and address.get("local")
        )
    except (OSError, subprocess.SubprocessError, ValueError, KeyError):
        return []


def process_info(pid):
    base = Path("/proc") / str(pid)
    try:
        return {
            "pid": pid,
            "command_line": base.joinpath("cmdline").read_bytes().replace(
                b"\0", b" "
            ).decode(errors="replace").strip(),
            "executable": os.readlink(base / "exe"),
            "working_directory": os.readlink(base / "cwd"),
        }
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return None


def ancestor_pids():
    result = {os.getpid()}
    pid = os.getppid()
    while pid > 1 and pid not in result:
        result.add(pid)
        try:
            pid = int((Path("/proc") / str(pid) / "stat").read_text().split()[3])
        except (OSError, ValueError, IndexError):
            break
    return result


def local_process_scan(interface_ips):
    excluded = ancestor_pids()
    candidates = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded:
            continue
        info = process_info(int(entry.name))
        # An SSH command may contain transferred source text mentioning SDK
        # symbols while owning no local DDS participant or arm command writer.
        if info and Path(info['executable']).name=='ssh':
            continue
        if info and PROCESS_PATTERN.search(info["command_line"]):
            info["reason"] = "command line contains a DDS or arm-control term"
            candidates.append(info)
    sockets = []
    try:
        output = subprocess.run(
            ["ss", "-H", "-uapn"], check=True, capture_output=True, text=True,
            timeout=3,
        ).stdout
        for line in output.splitlines():
            pid_match = re.search(r"pid=(\d+)", line)
            if pid_match and int(pid_match.group(1)) in excluded:
                continue
            if any(ip in line for ip in interface_ips) or re.search(r":74\d\d\b", line):
                sockets.append(line.strip())
        available = True
    except (OSError, subprocess.SubprocessError):
        available = False
    return {
        "local_hostname": socket.gethostname(),
        "interface_ipv4": interface_ips,
        "diagnostic_pid": os.getpid(),
        "candidate_processes_excluding_diagnostic": sorted(candidates, key=lambda x: x["pid"]),
        "udp_sockets_on_interface_or_dds_ports_excluding_diagnostic": sockets,
        "ss_available": available,
    }


def participant_locators(participant):
    properties = qos_detail(participant.qos)["properties"] if participant else {}
    locators = [x.strip() for x in properties.get("__NetworkAddresses", "").split(",")
                if x.strip()]
    return properties, locators


def activity_delta(start, end, topic, handle=None):
    if handle is None:
        return end[topic]["count"] - start[topic]["count"]
    return (end[topic]["by_handle"].get(handle, 0) -
            start[topic]["by_handle"].get(handle, 0))


def endpoint_report(reader, endpoint, pubs, subs, participants, start_activity,
                    end_activity, duration, kind):
    base = reader._endpoint_record(endpoint, pubs, subs, participants)
    participant = participants.get(str(endpoint.participant_key))
    properties, locators = participant_locators(participant)
    handle = str(int(endpoint.sample_info.instance_handle))
    count = (activity_delta(start_activity, end_activity, endpoint.topic_name, handle)
             if kind == "publication" and endpoint.topic_name in OBSERVED_TOPICS else None)
    base.update({
        "endpoint_kind": kind, "source_locator": locators or None,
        "application_name": properties.get("__ProcessName"), "sample_count": count,
        "sample_rate_hz": count / duration if count is not None and duration else None,
        "endpoint_liveliness": liveliness_detail(endpoint.qos),
        "participant_liveliness": liveliness_detail(participant.qos) if participant else None,
    })
    return base


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--state-json", type=Path,
                        help="optional unique output for the LowState captured by this observation")
    args = parser.parse_args()
    if not 15 <= args.seconds <= 120:
        parser.error("seconds must be 15..120 (30 recommended)")
    if args.json is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.json = ROOT / "output" / f"arm_control_ownership_{stamp}.json"

    # The receive-only identity probe discovers the robot Arm PID; it cannot
    # require that result before discovery.  Bind the reader's local identity
    # to this diagnostic process, then resolve the strict remote candidate from
    # the built-in DDS inventory below.
    reader = ReadOnlyState(args.network_interface, expected_arm_pid=os.getpid())
    try:
        time.sleep(1)  # discovery settling, outside the measured window
        start_activity = reader.activity_snapshot()
        with reader.lock:
            start_action_timestamp = (reader.action_messages[-1]["source_timestamp_ns"]
                                      if reader.action_messages else -1)
        start = time.monotonic()
        deadline = start + args.seconds
        timeline, presence, fresh_errors = [], {}, []
        last_state = None

        while True:
            now = time.monotonic()
            pubs, subs, participants = reader._discovery_inventory()
            elapsed = min(now - start, args.seconds)
            topic_pubs = {topic:[x for x in pubs if x.topic_name == topic]
                          for topic in (*OBSERVED_TOPICS, LOWSTATE_TOPIC)}
            topic_subs = {topic:[x for x in subs if x.topic_name == topic]
                          for topic in OBSERVED_TOPICS}
            for endpoint in [x for values in topic_pubs.values() for x in values]:
                key = str(endpoint.key)
                record = presence.setdefault(key, {
                    "topic": endpoint.topic_name, "first_seen_s": elapsed,
                    "last_seen_s": elapsed, "polls_present": 0,
                })
                record["last_seen_s"] = elapsed
                record["polls_present"] += 1
            try:
                last_state = reader.get(max_age=.5)
                fresh, tick = True, int(last_state.tick)
            except RuntimeError as exc:
                fresh, tick = False, None
                fresh_errors.append({"at_s":round(elapsed, 3), "error":str(exc)})
            activity = reader.activity_snapshot()
            timeline.append({
                "at_s": round(elapsed, 3), "lowstate_fresh": fresh,
                "lowstate_tick": tick,
                "publication_counts": {k:len(v) for k,v in topic_pubs.items()},
                "subscription_counts": {k:len(v) for k,v in topic_subs.items()},
                "sample_counts": {topic:activity_delta(start_activity, activity, topic)
                                  for topic in OBSERVED_TOPICS},
            })
            if now >= deadline:
                break
            time.sleep(min(.25, max(0, deadline - time.monotonic())))

        duration = time.monotonic() - start
        end_activity = reader.activity_snapshot()
        pubs, subs, participants = reader._discovery_inventory()
        publications = {
            topic:[endpoint_report(reader, x, pubs, subs, participants, start_activity,
                                   end_activity, duration, "publication")
                   for x in pubs if x.topic_name == topic]
            for topic in (*OBSERVED_TOPICS, LOWSTATE_TOPIC)
        }
        subscriptions = {
            topic:[endpoint_report(reader, x, pubs, subs, participants, start_activity,
                                   end_activity, duration, "subscription")
                   for x in subs if x.topic_name == topic]
            for topic in OBSERVED_TOPICS
        }
        for records in publications.values():
            for endpoint in records:
                seen = presence.get(endpoint["endpoint_guid"])
                endpoint["discovery"] = None if seen is None else {
                    "first_seen_s": round(seen["first_seen_s"], 3),
                    "last_seen_s": round(seen["last_seen_s"], 3),
                    "persistence_s": round(seen["last_seen_s"]-seen["first_seen_s"], 3),
                    "polls_present": seen["polls_present"], "polls_total": len(timeline),
                    "present_at_end": True,
                }

        with reader.lock:
            action_messages = [dict(x) for x in reader.action_messages
                               if x["source_timestamp_ns"] > start_action_timestamp]
        decoded_actions = [decode_arm_action_state(x["data"]) for x in action_messages]
        if not decoded_actions:
            action_status = "UNKNOWN"
        elif any(x["status"] == "ACTIVE" for x in decoded_actions):
            action_status = "ACTIVE"
        elif all(x["status"] == "IDLE" for x in decoded_actions):
            action_status = "IDLE"
        else:
            action_status = "UNKNOWN"

        state_error = stationary_error = None
        try:
            if last_state is None: raise RuntimeError("No fresh LowState")
            validate_state(last_state, initial=True)
        except (RuntimeError, ValueError) as exc:
            state_error = str(exc)
        try:
            reader.require_stationary()
        except RuntimeError as exc:
            stationary_error = str(exc)
        low_counts = [x["publication_counts"][LOWSTATE_TOPIC] for x in timeline]
        lowstate_pass = (bool(low_counts) and min(low_counts) == max(low_counts) == 1
                         and not fresh_errors and state_error is None)
        stationary_pass = stationary_error is None

        ownership = reader.ownership_snapshot()
        os_scan = local_process_scan(interface_ipv4(args.network_interface))
        if os_scan["candidate_processes_excluding_diagnostic"]:
            ownership["passed"] = False
            ownership["reasons"].append("Ubuntu arm/DDS controller candidate process exists")
        ownership["arm_action"]["status_during_observation"] = action_status
        if action_status != "IDLE":
            ownership["passed"] = False
            ownership["reasons"].append("Arm Action was not IDLE throughout observation")

        expected = [x for x in publications[ARM_SDK_TOPIC]
                    if x["classification"] == "EXPECTED ROBOT INTERNAL PARTICIPANT"]
        unknown_arm = [x for x in publications[ARM_SDK_TOPIC]
                       if x["classification"] != "EXPECTED ROBOT INTERNAL PARTICIPANT"]
        robot_samples = sum(x["sample_count"] or 0 for x in expected)
        armsdk_unknown = [x for x in publications[ARM_SDK_INPUT_TOPIC]
                          if x["classification"] != "EXPECTED ROBOT INTERNAL PARTICIPANT"]
        remote_armsdk_subs = [x for x in subscriptions[ARM_SDK_INPUT_TOPIC]
                              if x["classification"] != "THIS DIAGNOSTIC/CONTROLLER PROCESS"]

        report = {
            "schema": "motiondecode-test.arm-control-ownership.v2",
            "checked_local_time": time.strftime("%Y-%m-%d %H:%M:%S %z"),
            "network_interface": args.network_interface, "dds_domain": 0,
            "receive_only": True, "command_publishers_created": 0,
            "observation_requested_s": args.seconds,
            "observation_actual_s": round(duration, 3),
            "lowstate": {
                "status": "PASS" if lowstate_pass else "FAIL",
                "stationary": "PASS" if stationary_pass else "FAIL",
                "state_validation_error": state_error, "stationary_error": stationary_error,
                "freshness_errors": fresh_errors,
                "latest_state": state_summary(last_state) if last_state else None,
                "publishers": publications[LOWSTATE_TOPIC],
            },
            "topics": {
                ARM_SDK_TOPIC: {
                    "meaning": "Motion-mode user arm command blend input (underscore)",
                    "publishers": publications[ARM_SDK_TOPIC],
                    "total_sample_count": activity_delta(start_activity,end_activity,ARM_SDK_TOPIC),
                    "total_sample_rate_hz": activity_delta(start_activity,end_activity,ARM_SDK_TOPIC)/duration,
                    "expected_robot_internal_count": len(expected),
                    "expected_robot_internal_sample_count": robot_samples,
                    "external_writer_count": len(unknown_arm),
                },
                ARM_SDK_INPUT_TOPIC: {
                    "meaning": "Arm Action service input (no underscore)",
                    "publishers": publications[ARM_SDK_INPUT_TOPIC],
                    "subscribers": subscriptions[ARM_SDK_INPUT_TOPIC],
                    "remote_subscribers_excluding_observer": remote_armsdk_subs,
                    "total_sample_count": activity_delta(start_activity,end_activity,ARM_SDK_INPUT_TOPIC),
                    "total_sample_rate_hz": activity_delta(start_activity,end_activity,ARM_SDK_INPUT_TOPIC)/duration,
                    "unknown_writer_count": len(armsdk_unknown),
                },
                ARM_ACTION_STATE_TOPIC: {
                    "idl_type": "std_msgs::msg::dds_::String_",
                    "publishers": publications[ARM_ACTION_STATE_TOPIC],
                    "sample_count": len(action_messages),
                    "sample_rate_hz": len(action_messages)/duration,
                    "distinct_raw_payloads": sorted({x["data"] for x in action_messages}),
                    "decoded_distinct_payloads": [decode_arm_action_state(x)
                                                  for x in sorted({m["data"] for m in action_messages})],
                    "status_during_observation": action_status,
                },
            },
            "ownership": ownership, "local_os_scan": os_scan, "timeline": timeline,
            "ownership_preflight": "PASS" if (ownership["passed"] and lowstate_pass and stationary_pass)
                                            else "BLOCKED",
            "real_robot_test": "NOT EVALUATED; RUN COMBINED OFFLINE PREFLIGHT",
            "limitations": [
                "Locator/IP values are self-advertised DDS participant properties.",
                "Per-writer samples use publication_handle matched to the built-in endpoint instance handle.",
                "No command publisher, MotionSwitcher client, arm RPC, or robot motion was created.",
            ],
        }
        write_json(args.json, report)
        if args.state_json:
            captured = dict(report["lowstate"]["latest_state"] or {})
            captured.update({
                "checked_local_time": report["checked_local_time"],
                "interface": args.network_interface,
                "source_ownership_report": str(args.json.resolve()),
                "receive_only": True,
                "command_publishers_created": 0,
            })
            write_json(args.state_json, captured)

        print("\n=== ARM CONTROL OWNERSHIP ===\n")
        print("Robot internal arm service: " + ("EXPECTED" if len(expected)==1 else "UNKNOWN"))
        print("Robot internal rt/arm_sdk traffic: " +
              ("ACTIVE" if robot_samples else ("IDLE" if expected else "UNKNOWN")))
        print(f"External rt/arm_sdk writers: {len(unknown_arm)}")
        print(f"External rt/armsdk writers: {len(armsdk_unknown)}")
        print(f"rt/armsdk remote subscribers: {len(remote_armsdk_subs)}")
        print(f"Arm Action: {action_status} ({len(action_messages)} samples, "
              f"{len(action_messages)/duration:.3f} Hz)")
        print("G1 stationary: " + ("PASS" if stationary_pass else "FAIL"))
        print("Ownership preflight: " + ("PASS" if ownership["passed"] else "BLOCKED"))
        print("Next gate: " + ("READY FOR OFFLINE POSE CHECK"
                               if report["ownership_preflight"] == "PASS" else "BLOCKED"))
        if report["ownership_preflight"] == "BLOCKED":
            for reason in ownership["reasons"]: print(f"  - {reason}")
            if not lowstate_pass: print("  - LowState health failed")
            if not stationary_pass: print("  - G1 stationary check failed")
        print(f"Report: {args.json}")
        if args.state_json: print(f"Captured LowState: {args.state_json}")
        return 0 if report["ownership_preflight"] == "PASS" else 2
    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
