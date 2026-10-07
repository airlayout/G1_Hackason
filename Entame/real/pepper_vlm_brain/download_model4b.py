"""Download pinned official 4B files; verify LFS sizes and SHA256 before use."""
import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import urllib.request

REPO = 'Qwen/Qwen3-VL-4B-Instruct'
REVISION = 'ebb281ec70b05090aa6165b016eac8ec08e71b17'
ROOT = Path(__file__).resolve().parent
CACHE = ROOT / '.cache' / 'huggingface' / 'hub' / 'models--Qwen--Qwen3-VL-4B-Instruct'


def main():
    with urllib.request.urlopen(f'https://huggingface.co/api/models/{REPO}/revision/{REVISION}?blobs=true',timeout=30) as r:
        manifest=json.load(r)
    assert manifest['sha']==REVISION
    snapshot=CACHE/'snapshots'/REVISION;snapshot.mkdir(parents=True,exist_ok=True)
    files=[f for f in manifest['siblings'] if f['rfilename'].endswith(('.json','.txt','.safetensors'))]
    def fetch(item):
        name=item['rfilename']
        if Path(name).name!=name: raise ValueError('Unexpected nested filename')
        target=snapshot/name
        if not target.exists() or target.stat().st_size!=item['size']:
            partial=target.with_suffix(target.suffix+'.partial')
            with (ROOT/'.cache'/f'{name}.download.log').open('w') as log:
                subprocess.run(['curl.exe','--fail','--location','--retry','3','--continue-at','-',
                    '--output',str(partial),f'https://huggingface.co/{REPO}/resolve/{REVISION}/{name}'],
                    stdout=log,stderr=log,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
            if partial.stat().st_size!=item['size']: raise ValueError(f'Size mismatch: {name}')
            partial.replace(target)
        with target.open('rb') as stream: digest=hashlib.file_digest(stream,'sha256').hexdigest()
        expected=item.get('lfs',{}).get('sha256')
        if expected and digest!=expected: raise ValueError(f'SHA mismatch: {name}')
        print(f'Verified {name}: {target.stat().st_size} bytes',flush=True)
        return {'filename':name,'size':target.stat().st_size,'sha256':digest,'expected_lfs_sha256':expected}
    with ThreadPoolExecutor(max_workers=2) as pool: verified=list(pool.map(fetch,files))
    (CACHE/'refs').mkdir(exist_ok=True);(CACHE/'refs'/'main').write_text(REVISION,encoding='ascii')
    (ROOT/'reports'/'phase32_model_download.json').write_text(json.dumps({'repository':REPO,'revision':REVISION,'files':verified,'status':'pass'},indent=2),encoding='utf-8')

if __name__=='__main__': main()
