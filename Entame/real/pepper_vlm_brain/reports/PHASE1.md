# ROBOT VLM BRAIN PHASE 1

検証日: 2026-10-01 JST

```text
Environment: PASS
GPU: RTX 3060 Ti
CUDA available: PASS
Model: Qwen3-VL-2B-Instruct
Model load: PASS
Image inference: PASS
JSON parse: PASS
Validated Action: PASS
Fallback handling: PASS
```

証拠:

- `environment-before.json`, `environment-after.json`: 新venvと導入前後version。
- `torch-wheel-sha256.txt`: 公式PyTorch CUDA 12.8 Windows wheelのSHA256照合。
- `model-files.json`, `model-sha256.txt`: 固定revisionの公式LFS hashとの照合。
- `phase1-image.json`: 実Qwen3-VL-2BのCUDA画像推論。fallback=false、error=null、
  GPU上の全model parametersをコードで検査。raw JSONとparsed Decision一致。
- `phase1-repeat.json`: 同じモデル・同じ画像の2回推論、同じDecisionを確認。
- `fallback-missing-image.json`: 無効な画像パスでWAIT、exit code 1。
- `tests.json`: 40 pytest tests成功。正常JSON、fence、前後の説明、未知enum、
  欠損、完全parse不能、重複key、未知field（speed等）、NaN/Inf、不正confidence、
  推論例外、camera失敗時のresource releaseを検証。
- `phase2-camera.json`: index 0でDSHOW/MSMFの両方が失敗しWAIT、exit code 1。

結果: LOOK RIGHT / PERSON / MID / 0.95。モデルload 5.532秒、推論8.926秒。
peak allocated 4.134GiB / reserved 4.160GiB。モデル本体3.964GiB。
物理VRAMは8GiB、PyTorch割り当て上限80%。Windowsのdisplay等の使用は
allocated/reservedの値には含みません。`vram_free_gib` はdriver経由の別指標です。

次: Phase 2の実Webcam確認。カメラデバイスがないため現在未達。
Memoryとmemory tests（Phase 3）、4B NF4実験（Phase 4）は順序を守って未着手。
Driver/System CUDA/G1 repositoriesは変更していません。
