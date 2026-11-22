"""Process-local equivalent KV expansion for fused SDPA without native GQA support."""
from contextlib import contextmanager


@contextmanager
def expanded_kv(enabled):
    if not enabled:
        yield; return
    # Transformers already has the equivalent repeat_kv path. Select it temporarily.
    from transformers.integrations import sdpa_attention
    original=sdpa_attention.use_gqa_in_sdpa
    # A plain predicate must not retain the large CUDA key tensors in mock call history.
    sdpa_attention.use_gqa_in_sdpa=lambda attention_mask,key: False
    try: yield
    finally: sdpa_attention.use_gqa_in_sdpa=original
