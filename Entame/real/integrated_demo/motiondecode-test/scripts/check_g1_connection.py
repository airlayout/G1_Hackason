#!/usr/bin/env python3
"""Receive-only probe: no motion RPC or command publisher is created."""
import argparse
from pathlib import Path
import time
from common import ROOT,write_json
from robot_transport import ReadOnlyState,state_summary

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--network-interface',required=True)
    p.add_argument('--seconds',type=float,default=3)
    p.add_argument('--json',type=Path,default=ROOT/'output/connection.json')
    args=p.parse_args()
    if not 1<=args.seconds<=10:p.error('seconds must be 1..10')
    reader=ReadOnlyState(args.network_interface)
    try:
        state=reader.wait(args.seconds);report=state_summary(state)
        report.update({'checked_local_time':time.strftime('%Y-%m-%d %H:%M:%S %z'),
                       'interface':args.network_interface,'received_samples':len(reader.history),
                       'discovered_writers':{k:len(v) for k,v in reader.publications().items()},
                       'observed_arm_sdk_packets':reader.arm_packets,'command_publishers_created':0,
                       'standing_balance_verified':False})
        write_json(args.json,report)
        for key,value in report.items():print(f'{key}: {value}')
    finally:reader.close()

if __name__=='__main__':main()
