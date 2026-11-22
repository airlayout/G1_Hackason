"""Read-only Python/CUDA inspection; works even before dependencies exist."""
import importlib
import importlib.metadata
import json
import platform
import sys


def inspect():
    result = {"python": sys.version, "executable": sys.executable,
              "platform": platform.platform()}
    for name in ("pip", "torch", "torchvision", "transformers", "accelerate",
                 "bitsandbytes", "pydantic", "Pillow", "opencv-python"):
        try:
            result[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result[name] = "not installed"
    try:
        torch = importlib.import_module("torch")
        result["cuda_available"] = torch.cuda.is_available()
        result["torch_cuda_runtime"] = torch.version.cuda
        if result["cuda_available"]:
            result["gpu"] = torch.cuda.get_device_name(0)
            result["vram_gib"] = torch.cuda.get_device_properties(0).total_memory / 2**30
            free, total = torch.cuda.mem_get_info()
            result["vram_free_gib"] = free / 2**30
            result["bf16_supported"] = torch.cuda.is_bf16_supported()
    except Exception as exc:
        result["torch_error"] = str(exc)
    return result


if __name__ == "__main__":
    print(json.dumps(inspect(), indent=2))
