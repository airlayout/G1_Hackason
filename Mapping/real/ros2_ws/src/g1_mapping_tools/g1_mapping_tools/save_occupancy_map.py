"""Save the canonical map and propagate service failures as a nonzero exit code."""
from __future__ import annotations

import argparse
import sys

import rclpy
from std_srvs.srv import Trigger


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("save_occupancy_map_client")
    try:
        client = node.create_client(Trigger, "/g1_mapping/save_map")
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise RuntimeError("/g1_mapping/save_map is unavailable")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            raise RuntimeError("occupancy map save timed out")
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(result.message if result else "empty save response")
        print(f"[OK] {result.message}")
        return 0
    except Exception as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
