"""Safe command-line probe: dry-run is the default and opens no connection."""

import argparse
import json

from adapters.pepper.motion import DryRunTransport, HttpBridgeTransport, PepperMotionAdapter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", help="MOVE/TURN/LOOK/STOP JSON object")
    parser.add_argument("--bridge-url", help="Pepper Android bridge base URL")
    parser.add_argument("--send", action="store_true", help="send to the real bridge (never implied)")
    args = parser.parse_args()
    if args.send and not args.bridge_url:
        parser.error("--send requires --bridge-url")
    transport = HttpBridgeTransport(args.bridge_url) if args.send else DryRunTransport()
    PepperMotionAdapter(transport).execute(json.loads(args.command))


if __name__ == "__main__":
    main()
