import subprocess, shlex
src = r'''
import os, time
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsTopic, BuiltinTopicDcpsPublication
from cyclonedds.core import Qos, Policy
import os
dp = DomainParticipant(0)
r = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
seen = {}
for _ in range(8):
    time.sleep(1)
    for x in r.take(N=500):
        seen[x.topic_name] = x.type_name
for k in sorted(seen): print(k, "|", seen[k])
print("total", len(seen))
'''
r = subprocess.run(["ssh","-o","BatchMode=yes","g1-ts","cd ~/unitree_sdk2_python && python3 -u -c "+shlex.quote(src)],
    capture_output=True, text=True, timeout=90)
print(r.stdout); print(r.stderr[-700:] if r.returncode else "")
