"""読み取りのみ: lidar 点群 1〜数フレームを取得し、このPCへ保存する。"""
import base64
import json
import shlex
import subprocess
from pathlib import Path

REMOTE = r'''
import base64, json, os, time
os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.types as types
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Policy, Qos
from cyclonedds.internal import InvalidSample

@dataclass
class Time_(idl.IdlStruct, typename="builtin_interfaces::msg::dds_::Time_"):
    sec: types.int32
    nanosec: types.uint32
@dataclass
class Header_(idl.IdlStruct, typename="std_msgs::msg::dds_::Header_"):
    stamp: Time_
    frame_id: str
@dataclass
class PointField_(idl.IdlStruct, typename="sensor_msgs::msg::dds_::PointField_"):
    name: str
    offset: types.uint32
    datatype: types.uint8
    count: types.uint32
@dataclass
class PointCloud2_(idl.IdlStruct, typename="sensor_msgs::msg::dds_::PointCloud2_"):
    header: Header_
    height: types.uint32
    width: types.uint32
    fields: types.sequence[PointField_]
    is_bigendian: bool
    point_step: types.uint32
    row_step: types.uint32
    data: types.sequence[types.uint8]
    is_dense: bool

dp = DomainParticipant(0)
out = {"frames": []}
r = DataReader(dp, Topic(dp, "rt/utlidar/cloud_livox_mid360", PointCloud2_), Qos(Policy.History.KeepLast(1)))
end = time.time() + 8
last = None
while time.time() < end and len(out["frames"]) < 3:
    for s in r.take(N=1):
        if isinstance(s, InvalidSample): continue
        now = time.time()
        f = {"recv": now, "dt": None if last is None else round(now - last, 3), "frame_id": s.header.frame_id,
             "stamp": [s.header.stamp.sec, s.header.stamp.nanosec], "height": s.height, "width": s.width,
             "point_step": s.point_step, "row_step": s.row_step, "bigendian": s.is_bigendian, "dense": s.is_dense,
             "fields": [[x.name, x.offset, x.datatype, x.count] for x in s.fields], "bytes": len(s.data)}
        if len(out["frames"]) == 0:
            f["data_b64"] = base64.b64encode(bytes(s.data)).decode()
        out["frames"].append(f)
        last = now
    time.sleep(0.02)
print(json.dumps(out))
'''
r = subprocess.run(["ssh", "-o", "BatchMode=yes", "g1-ts", "cd ~/unitree_sdk2_python && python3 -u -c " + shlex.quote(REMOTE)],
                   capture_output=True, text=True, timeout=60)
line = [l for l in r.stdout.splitlines() if l.startswith("{")]
if not line:
    print("取得失敗", r.stderr[-800:])
else:
    d = json.loads(line[-1])
    d_dir = Path("/Users/koba/aicle/G1_Hackason/Console/docs/g1_snapshot")
    for f in d["frames"]:
        b = f.pop("data_b64", None)
        if b:
            (d_dir / "lidar_frame0.bin").write_bytes(base64.b64decode(b))
    (d_dir / "lidar_frames_meta.json").write_text(json.dumps(d, indent=1), encoding="utf-8")
    print(json.dumps(d, indent=1))
