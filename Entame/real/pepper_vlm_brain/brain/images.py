"""Shared, deterministic PIL input preparation for every image source."""
from PIL import Image, ImageOps
from config import Config


def prepare_image(image: Image.Image, config: Config) -> Image.Image:
    prepared = ImageOps.exif_transpose(image).convert("RGB")
    prepared.thumbnail((config.max_image_side, config.max_image_side))
    return prepared
