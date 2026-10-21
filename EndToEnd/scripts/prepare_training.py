"""監査済みepisodeを学習cacheへ変換し、episode単位の固定splitを作る。"""

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
from dataclasses import asdict

import h5py
import json
from pathlib import Path

import torch

from g1_e2e.models import ModelConfig
from g1_e2e.training.cache import prepare


def prepare_one(source, cache, config):
    torch.set_num_threads(1)
    if cache.exists():
        with h5py.File(cache) as file:
            metadata = json.loads(file.attrs["metadata_json"])
            if not file.attrs.get("complete") or metadata["source"] != str(source.resolve()):
                raise ValueError("既存cacheの出所または完了状態が一致しません")
            if metadata["source_bytes"] != source.stat().st_size:
                raise ValueError("cache生成後に生データのサイズが変わりました")
            if json.dumps(metadata["model_config"], sort_keys=True) != json.dumps(
                asdict(config), sort_keys=True
            ):
                raise ValueError("cacheと現在のモデル構成が異なります")
            return {
                "source": str(source.resolve()),
                "cache": str(cache.resolve()),
                "rows": int(file.attrs["rows"]),
            }
    return prepare(source, cache, config)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--model-config", type=Path, default=root / "configs/model/recurrent_perceiver.yaml")
    args = parser.parse_args()
    torch.set_num_threads(1)
    config = ModelConfig.load(args.model_config)
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "model_config": str(args.model_config.resolve()),
        "splits": {"train": [], "validation": [], "test": []},
        "rejected": [],
    }
    jobs = []
    for directory in args.directories:
        for index, source in enumerate(sorted(directory.glob("env_*/dataset.h5"))):
            split = "test" if index % 8 == 7 else "validation" if index % 8 == 6 else "train"
            cache = args.output / f"{directory.name}-{source.parent.name}.h5"
            jobs.append((source, cache, split))
    with ProcessPoolExecutor(
        max_workers=args.workers, mp_context=multiprocessing.get_context("spawn")
    ) as pool:
        futures = {
            pool.submit(prepare_one, source, cache, config): (source, split) for source, cache, split in jobs
        }
        for future in as_completed(futures):
            source, split = futures[future]
            try:
                report = future.result()
            except ValueError as error:
                manifest["rejected"].append({"source": str(source.resolve()), "error": str(error)})
                print(f"[REJECTED] {source}: {error}", flush=True)
                continue
            manifest["splits"][split].append(report)
            for values in manifest["splits"].values():
                values.sort(key=lambda value: value["cache"])
            (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"[PREPARED] {split}: {json.dumps(report)}", flush=True)
    if any(not values for values in manifest["splits"].values()):
        raise ValueError("train・validation・testに採用episodeが必要です")
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
