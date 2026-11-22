"""Qwen3-VL inference on CUDA; no robot SDK or camera dependency."""
from time import perf_counter
import os
import hashlib
from config import Config, PROJECT_ROOT

# Set before importing Hugging Face: nested processor loads sometimes omit cache_dir.
os.environ["HF_HOME"] = str(PROJECT_ROOT / ".cache" / "huggingface")
os.environ["HF_HUB_CACHE"] = str(PROJECT_ROOT / ".cache" / "huggingface" / "hub")
import torch
from PIL import Image, ImageOps
from transformers import AutoProcessor, Qwen3VLForConditionalGeneration, BitsAndBytesConfig
from .prompt import system_prompt, user_prompt, observation_system_prompt, observation_user_prompt
from .images import prepare_image


class QwenVLM:
    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self.model = None
        self.processor = None
        self.last_metrics = {}

    def load(self):
        if self.model is not None:
            return
        start = perf_counter()
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable. Install the official CUDA wheel in this venv.")
        torch.cuda.set_per_process_memory_fraction(self.config.vram_fraction, 0)
        torch.set_num_threads(4)
        dtype = torch.bfloat16 if torch.cuda.is_bf16_supported() else torch.float16
        self.last_metrics = {"gpu": torch.cuda.get_device_name(0), "cuda_available": True,
                             "gpu_used": False, "dtype": str(dtype), "attention": "sdpa",
                             "model": self.config.model_name,
                             "revision": self.config.model_revision,
                             "quantize_4bit": self.config.quantize_4bit}
        common = dict(cache_dir=str(self.config.cache_dir),
                      revision=self.config.model_revision, trust_remote_code=False)
        self.processor = AutoProcessor.from_pretrained(
            self.config.model_name, min_pixels=self.config.min_pixels,
            max_pixels=self.config.max_pixels, **common)
        # AutoProcessor 4.57 does not forward these kwargs to every fast processor.
        # Explicitly set the actual resize budget, rather than trusting kwargs.
        self.processor.image_processor.size = {"shortest_edge": self.config.min_pixels,
                                              "longest_edge": self.config.max_pixels}
        kwargs = {}
        if self.config.quantize_4bit:
            kwargs["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_quant_type="nf4",
                bnb_4bit_use_double_quant=True, bnb_4bit_compute_dtype=dtype)
        self.model = Qwen3VLForConditionalGeneration.from_pretrained(
            self.config.model_name, dtype=dtype, device_map={"": "cuda:0"},
            attn_implementation="sdpa", low_cpu_mem_usage=True, **common, **kwargs).eval()
        if not all(p.device.type == "cuda" for p in self.model.parameters()):
            self.model = None
            raise RuntimeError("Model parameters were not fully placed on CUDA")
        torch.cuda.synchronize()
        self.last_metrics.update(model_load_s=perf_counter() - start, gpu_used=True,
                                 model_allocated_gib=torch.cuda.memory_allocated() / 2**30)

    def generate(self, image: Image.Image, memory_context=None, previous_image=None):
        self.load()
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        start = perf_counter()
        for key in ("inference_latency_s", "generation_latency_s", "preprocessing_latency_s",
                    "input_tokens", "output_tokens", "peak_allocated_gib", "peak_reserved_gib"):
            self.last_metrics.pop(key, None)
        self.last_metrics["source_image_size"] = list(ImageOps.exif_transpose(image).size)
        image = prepare_image(image, self.config)
        self.last_metrics["image_size"] = list(image.size)
        if previous_image is not None and not self.config.combined_output:
            raise ValueError("Temporal input requires combined Observation output")
        self.processor.image_processor.size = {"shortest_edge": self.config.min_pixels,
                                               "longest_edge": self.config.max_pixels}
        previous = prepare_image(previous_image, self.config) if previous_image is not None else None
        system = observation_system_prompt(previous is not None) if self.config.combined_output else system_prompt(self.config.prompt_profile)
        user = (observation_user_prompt(memory_context, previous is not None) if self.config.combined_output
                else user_prompt(memory_context, self.config.prompt_profile))
        content = []
        if previous is not None:
            content.extend([{"type": "text", "text": "IMAGE 1 = previous observation"},
                            {"type": "image", "image": previous},
                            {"type": "text", "text": "IMAGE 2 = current observation (CURRENT IMAGE)"}])
        else:
            content.append({"type": "text", "text": "IMAGE 1 = current observation (CURRENT IMAGE)"})
        content.extend([{"type": "image", "image": image}, {"type": "text", "text": user}])
        messages = [
            {"role": "system", "content": [{"type": "text", "text": system}]},
            {"role": "user", "content": content},
        ]
        if not self.config.combined_output:
            messages[1]["content"] = [{"type": "image", "image": image}, {"type": "text", "text": user}]
        inputs = self.processor.apply_chat_template(
            messages, tokenize=True, add_generation_prompt=True,
            return_dict=True, return_tensors="pt")
        grids = inputs.image_grid_thw.tolist()
        grid = grids[-1]
        patch = self.processor.image_processor.patch_size
        merge = self.processor.image_processor.merge_size
        self.last_metrics.update(image_grid_thw=grid,
                                 processor_image_size=[grid[2] * patch, grid[1] * patch],
                                 image_tokens=sum(int(g[0] * g[1] * g[2] // merge**2) for g in grids),
                                 image_tokens_per_image=[int(g[0] * g[1] * g[2] // merge**2) for g in grids],
                                 image_grids_thw=grids, image_count=len(grids),
                                 prepared_image_sizes=([list(previous.size)] if previous is not None else []) + [list(image.size)],
                                 combined_output=self.config.combined_output,
                                 prompt_characters=len(system + user), prompt_profile=self.config.prompt_profile,
                                 prompt_sha256=hashlib.sha256((system + user).encode()).hexdigest(),
                                 max_new_tokens=self.config.max_new_tokens)
        inputs = inputs.to("cuda:0")
        torch.cuda.synchronize()
        generation_start = perf_counter()
        self.last_metrics["preprocessing_latency_s"] = generation_start - start
        with torch.inference_mode():
            output = self.model.generate(**inputs, do_sample=False, temperature=None,
                                         top_p=None, top_k=None,
                                         max_new_tokens=self.config.max_new_tokens)
        generated = output[:, inputs.input_ids.shape[1]:]
        torch.cuda.synchronize()
        self.last_metrics["generation_latency_s"] = perf_counter() - generation_start
        raw = self.processor.batch_decode(generated, skip_special_tokens=True,
                                          clean_up_tokenization_spaces=False)[0]
        torch.cuda.synchronize()
        free, total = torch.cuda.mem_get_info()
        self.last_metrics.update(
            inference_latency_s=perf_counter() - start,
            input_tokens=int(inputs.input_ids.shape[1]), output_tokens=int(generated.shape[1]),
            peak_allocated_gib=torch.cuda.max_memory_allocated() / 2**30,
            peak_reserved_gib=torch.cuda.max_memory_reserved() / 2**30,
            vram_free_gib=free / 2**30, vram_total_gib=total / 2**30)
        return raw, dict(self.last_metrics)
