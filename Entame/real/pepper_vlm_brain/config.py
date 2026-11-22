"""Robot-independent model settings. No device SDKs or motor parameters."""
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
MODEL_2B = "Qwen/Qwen3-VL-2B-Instruct"
MODEL_4B = "Qwen/Qwen3-VL-4B-Instruct"
REVISION_2B = "89644892e4d85e24eaac8bacfd4f463576704203"
REVISION_4B = "ebb281ec70b05090aa6165b016eac8ec08e71b17"


@dataclass(frozen=True)
class Config:
    model_name: str = MODEL_2B
    revision: str | None = None
    quantize_4bit: bool = False
    cache_dir: Path = PROJECT_ROOT / ".cache" / "huggingface" / "hub"
    max_image_side: int = 448
    min_pixels: int = 64 * 64
    max_pixels: int = 448 * 448
    max_new_tokens: int = 96
    prompt_profile: str = "legacy"
    vram_fraction: float = 0.80
    combined_output: bool = False

    def __post_init__(self):
        if self.prompt_profile not in {"legacy", "compact"}:
            raise ValueError("prompt_profile must be legacy or compact")
        if not 64 <= self.max_image_side <= 512 or not 4096 <= self.max_pixels <= 512 * 512:
            raise ValueError("Image budget must be between 4096 and 262144 pixels, edge <=512")
        if not 0 < self.min_pixels <= self.max_pixels:
            raise ValueError("Invalid min/max pixel budget")
        if not 16 <= self.max_new_tokens <= 256:
            raise ValueError("Generation budget must be 16..256")
        if not 0 < self.vram_fraction <= 0.80:
            raise ValueError("v0.1 VRAM fraction must be <= 0.80")
        if self.model_name == MODEL_4B and not self.quantize_4bit:
            raise ValueError("4B requires explicit NF4 quantization on this 8GB profile")

    @property
    def model_revision(self):
        return self.revision or ({MODEL_2B: REVISION_2B, MODEL_4B: REVISION_4B}.get(self.model_name, "main"))
