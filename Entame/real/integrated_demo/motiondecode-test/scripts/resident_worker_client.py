#!/usr/bin/env python3
"""Client/stdio bridge for the resident Unix-socket worker."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from resident_protocol import unix_request


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, default=Path("/tmp/motiondecode-reaction.sock"))
    parser.add_argument("--stdio", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.)
    parser.add_argument("reaction", nargs="?")
    args = parser.parse_args()
    if args.stdio:
        for line in sys.stdin:
            try:
                request = json.loads(line)
                response = unix_request(args.socket, request, args.timeout)
            except BaseException as exc:
                response = {"accepted": False, "state": "FAULT", "reason": str(exc)}
            print(json.dumps(response, sort_keys=True), flush=True)
        return 0
    request = {"operation": "execute", "reaction": args.reaction} if args.reaction else {
        "operation": "status"}
    print(json.dumps(unix_request(args.socket, request, args.timeout), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
