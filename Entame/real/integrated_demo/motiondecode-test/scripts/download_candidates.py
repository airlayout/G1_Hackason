#!/usr/bin/env python3
"""Fetch only the 10 pinned candidate CSVs; never a dataset snapshot download."""
import json
from pathlib import Path
import urllib.parse
import urllib.request
from common import ROOT,output_path,digest

def main():
    manifest=json.loads((ROOT/'data/metadata/downloads.json').read_text())
    if len(manifest)>10:raise ValueError('First-test download limit is 10 motions')
    for item in manifest:
        dest=output_path(ROOT/item['local'])
        if dest.exists() and digest(dest)==item['sha256']:
            print('verified',dest.name);continue
        url='https://huggingface.co/datasets/CMRobot/MotionDecode/resolve/'+item['revision']+'/'+urllib.parse.quote(item['path'])+'?download=true'
        with urllib.request.urlopen(url,timeout=45) as response:
            content=response.read(10_000_001)
        if len(content)>10_000_000:raise ValueError('Candidate exceeds 10 MB')
        import hashlib
        if hashlib.sha256(content).hexdigest()!=item['sha256']:raise ValueError('Download hash mismatch')
        dest.write_bytes(content);print('downloaded',dest.name)

if __name__=='__main__':main()
