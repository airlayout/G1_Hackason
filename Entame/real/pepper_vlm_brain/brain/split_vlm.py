"""Qwen shared model with separate image-only Perception and text-only Planner calls."""
import hashlib
from time import perf_counter
import torch
from PIL import ImageOps
from .vlm import QwenVLM
from .images import prepare_image
from .split_prompt import PERCEPTION_SYSTEM, PERCEPTION_USER, PLANNER_SYSTEM, planner_user


class SplitQwenVLM(QwenVLM):
    def load(self):
        if self.model is not None:
            return
        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats()
        super().load()
        self.model_load_count = getattr(self, 'model_load_count', 0) + 1
        self.last_metrics.update(model_load_peak_allocated_gib=torch.cuda.max_memory_allocated()/2**30,
                                 model_load_peak_reserved_gib=torch.cuda.max_memory_reserved()/2**30,
                                 model_footprint_gib=self.model.get_memory_footprint()/2**30)
        if self.config.quantize_4bit:
            import bitsandbytes as bnb
            layers=[m for m in self.model.modules() if isinstance(m,bnb.nn.Linear4bit)]
            if not layers or any(m.weight.quant_state is None or m.weight.quant_state.quant_type != 'nf4' for m in layers):
                raise RuntimeError('4B did not load verified NF4 layers')
            self.last_metrics.update(nf4_linear_layers=len(layers),
                                     double_quant_layers=sum(m.weight.quant_state.nested for m in layers),
                                     loaded_in_4bit=bool(getattr(self.model,'is_loaded_in_4bit',False)),
                                     bitsandbytes_version=bnb.__version__)

    def perceive(self, image):
        image = prepare_image(image, self.config)
        messages = [{'role': 'system', 'content': [{'type': 'text', 'text': PERCEPTION_SYSTEM}]},
                    {'role': 'user', 'content': [{'type': 'image', 'image': image},
                                                 {'type': 'text', 'text': PERCEPTION_USER}]}]
        return self._generate(messages, 'perception', PERCEPTION_SYSTEM + PERCEPTION_USER)

    def plan(self, observation, snapshot):
        user = planner_user(observation, snapshot)
        messages = [{'role': 'system', 'content': [{'type': 'text', 'text': PLANNER_SYSTEM}]},
                    {'role': 'user', 'content': [{'type': 'text', 'text': user}]}]
        return self._generate(messages, 'planner', PLANNER_SYSTEM + user)

    def _generate(self, messages, stage, prompt):
        self.load()
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        start = perf_counter()
        self.processor.image_processor.size = {'shortest_edge': self.config.min_pixels,
                                               'longest_edge': self.config.max_pixels}
        inputs = self.processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                                     return_dict=True, return_tensors='pt')
        grids = inputs.image_grid_thw.tolist() if 'image_grid_thw' in inputs else []
        if len(grids) != (1 if stage == 'perception' else 0):
            raise RuntimeError('Stage image count invariant violated')
        merge = self.processor.image_processor.merge_size
        metrics = {'stage': stage, 'model': self.config.model_name, 'revision': self.config.model_revision,
                   'quantize_4bit': self.config.quantize_4bit, 'image_count': len(grids),
                   'image_tokens': sum(int(g[0]*g[1]*g[2]//merge**2) for g in grids),
                   'image_grids_thw': grids, 'prompt_sha256': hashlib.sha256(prompt.encode()).hexdigest(),
                   'max_new_tokens': self.config.max_new_tokens}
        metrics.update(model_instance_id=id(self.model), processor_instance_id=id(self.processor),
                       model_load_count=self.model_load_count)
        inputs = inputs.to('cuda:0')
        torch.cuda.synchronize(); generation_start = perf_counter()
        with torch.inference_mode():
            output = self.model.generate(**inputs, do_sample=False, temperature=None, top_p=None, top_k=None,
                                         max_new_tokens=self.config.max_new_tokens)
        generated = output[:, inputs.input_ids.shape[1]:]
        torch.cuda.synchronize()
        metrics.update(generation_latency_s=perf_counter()-generation_start)
        raw = self.processor.batch_decode(generated, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]
        metrics.update(inference_latency_s=perf_counter()-start,
                       preprocessing_latency_s=generation_start-start, input_tokens=int(inputs.input_ids.shape[1]),
                       output_tokens=int(generated.shape[1]), peak_allocated_gib=torch.cuda.max_memory_allocated()/2**30,
                       peak_reserved_gib=torch.cuda.max_memory_reserved()/2**30)
        self.last_stage_metrics = metrics
        return raw, metrics
