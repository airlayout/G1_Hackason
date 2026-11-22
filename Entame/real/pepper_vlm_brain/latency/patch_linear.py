"""Exact patch geometry with unchanged weights, implemented by GEMM instead of Conv3d."""
from contextlib import contextmanager
from types import MethodType
import torch.nn.functional as F


def linear_patch_forward(module,hidden_states):
    conv=module.proj
    kernel=(module.temporal_patch_size,module.patch_size,module.patch_size)
    if (tuple(conv.kernel_size)!=kernel or tuple(conv.stride)!=kernel or tuple(conv.padding)!=(0,0,0)
            or tuple(conv.dilation)!=(1,1,1) or conv.groups!=1):
        raise ValueError('Only complete non-overlapping single-output patches are supported')
    patches=hidden_states.view(-1,module.in_channels,*kernel).flatten(1).to(dtype=conv.weight.dtype)
    return F.linear(patches,conv.weight.flatten(1),conv.bias)


@contextmanager
def patch_backend(model,enabled):
    if not enabled:
        yield; return
    module=model.model.visual.patch_embed; original=module.forward
    module.forward=MethodType(linear_patch_forward,module)
    try: yield
    finally: module.forward=original
