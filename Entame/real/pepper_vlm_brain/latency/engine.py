"""Shared Qwen experimental engine with low-volume CUDA events and first-token timing."""
import hashlib
from contextlib import nullcontext
from time import perf_counter
import torch
from torch.nn.attention import sdpa_kernel, SDPBackend
from transformers.generation.streamers import BaseStreamer
from transformers import LogitsProcessor, LogitsProcessorList
from brain.split_vlm import SplitQwenVLM
from brain.images import prepare_image
from brain.split_prompt import planner_user
from config import Config
from .codecs import PLANNER_CLASS
from .patch_linear import patch_backend
from .attention import expanded_kv


class TokenClock(BaseStreamer):
    def __init__(self): self.prompt=True; self.first=None; self.count=0
    def put(self,value):
        if self.prompt: self.prompt=False; return
        if self.first is None: self.first=perf_counter()
        self.count += value.numel()
    def end(self): pass


class AllowedCodes(LogitsProcessor):
    """Finite-class argmax over tokenizer-verified codes; no labels or state rules."""
    def __init__(self,ids): self.ids=tuple(ids); self.last_scores=None
    def __call__(self,input_ids,scores):
        self.last_scores=scores[:,self.ids].detach()
        masked=torch.full_like(scores,float('-inf')); masked[:,self.ids]=scores[:,self.ids]
        return masked


class ForwardClock:
    def __init__(self,model):
        self.handles=[]; self.events={}; self.wall={}
        for name,module in [('prefill',model),('vision',model.model.visual),
                            ('patch_embed',model.model.visual.patch_embed),
                            ('vision_block0',model.model.visual.blocks[0])]:
            def before(module,args,name=name):
                if name in self.events: return
                pair=(torch.cuda.Event(enable_timing=True),torch.cuda.Event(enable_timing=True))
                self.events[name]=pair; self.wall[name]=[perf_counter(),None]; pair[0].record()
            def after(module,args,output,name=name):
                if name not in self.events or self.wall[name][1] is not None: return
                self.events[name][1].record(); self.wall[name][1]=perf_counter()
            self.handles += [module.register_forward_pre_hook(before),module.register_forward_hook(after)]
    def close(self):
        for handle in self.handles: handle.remove()
    def metrics(self):
        result={}
        for name,(start,end) in self.events.items():
            if self.wall[name][1] is not None:
                result[name+'_cuda_elapsed_s']=start.elapsed_time(end)/1000
                result[name+'_host_forward_s']=self.wall[name][1]-self.wall[name][0]
        return result


class LatencyQwen(SplitQwenVLM):
    def __init__(self,profile):
        super().__init__(Config(max_image_side=profile.edge,max_pixels=profile.edge**2))
        self.profile=profile

    def configure(self,profile):
        if self.model is not None and self.profile.dtype!=profile.dtype:
            raise ValueError('Use a separate engine process for dtype comparisons')
        self.profile=profile
        self.config=Config(max_image_side=profile.edge,max_pixels=profile.edge**2)

    def load(self):
        if self.model is not None: return
        super().load()
        if self.profile.dtype=='fp16':
            # Preserve explicitly FP32 rotary/numerical buffers; compare weight/activation dtype.
            fp32_buffers={name:value for name,value in self.model.named_buffers() if value.dtype==torch.float32}
            start=perf_counter(); self.model.to(dtype=torch.float16)
            for name,value in fp32_buffers.items():
                parent,_,key=name.rpartition('.')
                setattr(self.model.get_submodule(parent),key,value)
            torch.cuda.synchronize()
            self.last_metrics.update(dtype='torch.float16',dtype_conversion_s=perf_counter()-start)
            self.last_metrics['preserved_fp32_buffers']=list(fp32_buffers)
            self.last_metrics['model_load_s']+=self.last_metrics['dtype_conversion_s']
        self.last_metrics.update(model_instance_id=id(self.model),processor_instance_id=id(self.processor),
                                 model_load_count=self.model_load_count,
                                 model_load_peak_allocated_gib=torch.cuda.max_memory_allocated()/2**30,
                                 model_load_peak_reserved_gib=torch.cuda.max_memory_reserved()/2**30)
        self.code_tokens={c:self.processor.tokenizer.encode(c,add_special_tokens=False) for c in PLANNER_CLASS}
        if any(len(v)!=1 for v in self.code_tokens.values()): raise RuntimeError('Scene codes are not single tokens')

    def perceive(self,image):
        start=perf_counter(); image=prepare_image(image,self.config); elapsed=perf_counter()-start
        system,user=self.profile.prompt('perception')
        messages=[]
        if system: messages.append({'role':'system','content':[{'type':'text','text':system}]})
        messages.append({'role':'user','content':[{'type':'image','image':image},{'type':'text','text':user}]})
        raw,metrics=self._profile_generate(messages,'perception',(system or '')+user)
        metrics.update(image_preprocessing_s=elapsed,stage_wall_s=perf_counter()-start)
        if 'ttft_stage_s' in metrics: metrics['ttft_stage_s']+=elapsed
        return raw,metrics

    def plan(self,observation,snapshot):
        start=perf_counter(); system,_=self.profile.prompt('planner'); user=planner_user(observation,snapshot)
        messages=[{'role':'system','content':[{'type':'text','text':system}]},
                  {'role':'user','content':[{'type':'text','text':user}]}]
        raw,metrics=self._profile_generate(messages,'planner',system+user)
        metrics.update(image_preprocessing_s=0.0,stage_wall_s=perf_counter()-start)
        return raw,metrics

    def _profile_generate(self,messages,stage,prompt):
        self.pending_metrics={'stage':stage,'format':self.profile.stage_format(stage),'input_tokens':None,'output_tokens':None}
        self.load(); torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats(); start=perf_counter()
        self.processor.image_processor.size={'shortest_edge':self.config.min_pixels,'longest_edge':self.config.max_pixels}
        pstart=perf_counter()
        inputs=self.processor.apply_chat_template(messages,tokenize=True,add_generation_prompt=True,return_dict=True,return_tensors='pt')
        processor_s=perf_counter()-pstart
        grids=inputs.image_grid_thw.tolist() if 'image_grid_thw' in inputs else []
        if len(grids)!=(1 if stage=='perception' else 0): raise RuntimeError('Stage image invariant violated')
        transfer=perf_counter(); inputs=inputs.to('cuda:0'); torch.cuda.synchronize(); transfer_s=perf_counter()-transfer
        clock=TokenClock() if self.profile.instrument else None
        forward=ForwardClock(self.model) if self.profile.instrument else None
        context=(sdpa_kernel({'flash':SDPBackend.FLASH_ATTENTION,'efficient':SDPBackend.EFFICIENT_ATTENTION,
                             'efficient_expanded':SDPBackend.EFFICIENT_ATTENTION}[self.profile.attention])
                 if self.profile.attention!='default' else nullcontext())
        generation_start=perf_counter()
        self.pending_metrics.update(processor_s=processor_s,device_transfer_s=transfer_s,
                                    input_tokens=int(inputs.input_ids.shape[1]),image_count=len(grids),
                                    image_tokens=sum(int(g[0]*g[1]*g[2]//self.processor.image_processor.merge_size**2) for g in grids),
                                    model_instance_id=id(self.model),processor_instance_id=id(self.processor),
                                    model_load_count=self.model_load_count)
        alphabet=('ABCD' if stage=='perception' and self.profile.constrained else
                  ''.join(PLANNER_CLASS) if stage=='planner' and self.profile.planner_class else '')
        allowed=AllowedCodes([self.code_tokens[c][0] for c in alphabet]) if alphabet else None
        try:
            with context,expanded_kv(self.profile.attention=='efficient_expanded'),patch_backend(self.model,self.profile.linear_patch),torch.inference_mode():
                output=self.model.generate(**inputs,do_sample=False,temperature=None,top_p=None,top_k=None,
                                           max_new_tokens=self.profile.budget(stage),streamer=clock,
                                           logits_processor=LogitsProcessorList([allowed]) if allowed else None)
            torch.cuda.synchronize(); generation_end=perf_counter()
        finally:
            if forward: forward.close()
        generated=output[:,inputs.input_ids.shape[1]:]; count=int(generated.shape[1])
        decode_start=perf_counter()
        raw=self.processor.batch_decode(generated,skip_special_tokens=True,clean_up_tokenization_spaces=False)[0]
        text_decode_s=perf_counter()-decode_start
        merge=self.processor.image_processor.merge_size
        metrics={'stage':stage,'model':self.config.model_name,'revision':self.config.model_revision,
                 'dtype':self.profile.dtype,'attention':self.profile.attention,'format':self.profile.format,
                 'model_instance_id':id(self.model),'processor_instance_id':id(self.processor),'model_load_count':self.model_load_count,
                 'image_count':len(grids),'image_grids_thw':grids,'image_tokens':sum(int(g[0]*g[1]*g[2]//merge**2) for g in grids),
                 'prompt_sha256':hashlib.sha256(prompt.encode()).hexdigest(),'max_new_tokens':self.profile.budget(stage),
                 'input_tokens':int(inputs.input_ids.shape[1]),'output_tokens':count,'generated_token_ids':generated[0].tolist(),
                 'processor_s':processor_s,'device_transfer_s':transfer_s,'text_decoding_s':text_decode_s,
                 'generation_latency_s':generation_end-generation_start,'inference_latency_s':perf_counter()-start,
                 'peak_allocated_gib':torch.cuda.max_memory_allocated()/2**30,'peak_reserved_gib':torch.cuda.max_memory_reserved()/2**30,
                 'instrumented':self.profile.instrument}
        if clock and clock.first is not None:
            metrics.update(ttft_generation_s=clock.first-generation_start,ttft_stage_s=clock.first-start,
                           decode_total_s=generation_end-clock.first,decode_steps=max(count-1,0),
                           decode_ms_per_token=(generation_end-clock.first)*1000/(count-1) if count>1 else None,
                           streamer_token_count=clock.count)
        if forward: metrics.update(forward.metrics())
        if allowed: metrics.update(class_allowed_token_ids=list(allowed.ids),class_logits=allowed.last_scores[0].tolist(),
                                   class_logits_finite=bool(torch.isfinite(allowed.last_scores).all().item()))
        return raw,metrics

    def failure_metrics(self,stage):
        metrics=dict(self.pending_metrics) if getattr(self,'pending_metrics',{}).get('stage')==stage else {'stage':stage}
        try:
            metrics.update(peak_allocated_gib=torch.cuda.max_memory_allocated()/2**30,
                           peak_reserved_gib=torch.cuda.max_memory_reserved()/2**30)
        except Exception as exc: metrics['telemetry_error']=str(exc)
        return metrics
