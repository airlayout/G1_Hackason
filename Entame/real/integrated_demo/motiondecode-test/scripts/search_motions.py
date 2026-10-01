#!/usr/bin/env python3
"""Search taxonomy AND actual file inventory; never invent durations/FPS."""
import argparse
import csv
import json
import re
from pathlib import Path
from common import ROOT, write_json

REACTIONS = {
    'suspicious': ['1.7.6.2','1.7.6.6','1.7.7.3','1.10.2.4','1.10.4.3'],
    'found': ['1.8.2.1','1.7.6.2','1.10.1.4'],
    'mistaken': ['1.7.7.3','1.7.7.4','1.10.2.4'],
    'surprised': ['1.7.6.5','1.10.6.1'],
    'confused': ['1.10.2.4','1.10.2.5','1.7.7.4'],
    'look_around': ['1.10.4.2','1.7.6.6'],
}

def catalog():
    rows = list(csv.DictReader((ROOT/'data/metadata/index.csv').open()))
    inventory = json.loads((ROOT/'data/metadata/revision.json').read_text())
    groups = {}
    for f in inventory['siblings']:
        path = f['rfilename']
        if not path.endswith('.csv') or not path.startswith('samples/'):
            continue
        m = re.match(r'(\d+(?:\.\d+)*)\.',Path(path).parent.name)
        if m:
            groups.setdefault(m[1],[]).append(path)
    return rows, groups

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--query',default='')
    p.add_argument('--reaction',choices=REACTIONS)
    p.add_argument('--limit',type=int,default=10)
    p.add_argument('--per-category',type=int,default=2)
    p.add_argument('--json',type=Path)
    args=p.parse_args()
    rows, groups=catalog(); found=[]
    query=args.query.lower().replace('hesitation','hesitat').replace(' ','_')
    for row in rows:
        label=row['Label']
        if args.reaction and label not in REACTIONS[args.reaction]:continue
        if query and query not in str(row).lower().replace(' ','_'):continue
        paths=groups.get(label,[])
        category=row['Tertiary class name'] or row['Secondary class name']
        if not paths:
            print(f'{label} {category}: taxonomy only; no matching samples in pinned inventory')
        for path in paths[:args.per_category]:
            found.append({'name':Path(path).stem,'category':category,'label':label,
                          'path':path,'duration':None,'fps':None,
                          'description':category.replace('_',' '),
                          'available_takes':len(paths)})
    found=found[:max(0,args.limit)]
    for f in found:
        print(f"{f['name']} | {f['category']} | duration=unknown | FPS=unknown\n  {f['path']}")
    if args.json:write_json(args.json,found)

if __name__=='__main__':main()
