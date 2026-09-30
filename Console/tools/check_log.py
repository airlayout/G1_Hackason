import collections
import json
from pathlib import Path

d = Path("/Users/koba/aicle/G1_Hackason/Console/docs/g1_logs")
f = sorted(d.glob("live_*.jsonl"))[-1]
c = collections.Counter()
n = 0
for line in f.read_text(encoding="utf-8").splitlines():
    r = json.loads(line)
    n += 1
    c[r.get("topic", "meta" if "meta" in r else "beat")] += 1
print(f.name, f.stat().st_size, "bytes,", n, "lines")
for k, v in c.most_common():
    print("%5d %s" % (v, k))
