import json,sys,importlib.metadata
from pathlib import Path
report={'python':sys.version,'executable':sys.executable,'stages':{}}
try:
 import torch
 import bitsandbytes as bnb
 report['stages']['import']='pass'
 report.update(torch=torch.__version__,cuda_runtime=torch.version.cuda,bitsandbytes=bnb.__version__,bitsandbytes_path=bnb.__file__,gpu=torch.cuda.get_device_name(0),compute_capability=torch.cuda.get_device_capability(0))
 from bitsandbytes.cextension import lib
 report['cuda_library']=str(getattr(getattr(lib,'_lib',None),'_name',None))
 layer=bnb.nn.Linear4bit(64,32,bias=False,compute_dtype=torch.bfloat16,compress_statistics=True,quant_type='nf4').to('cuda:0')
 with torch.inference_mode(): out=layer(torch.randn(2,64,device='cuda:0',dtype=torch.bfloat16))
 torch.cuda.synchronize()
 report['stages']['cuda_nf4_kernel']='pass' if out.is_cuda and torch.isfinite(out).all().item() else 'fail'
 report.update(quant_type=layer.weight.quant_state.quant_type,nested=layer.weight.quant_state.nested,output_device=str(out.device),output_shape=list(out.shape))
 report['status']='pass'
except Exception as exc:
 report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
Path('reports/phase32_bnb_probe.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
