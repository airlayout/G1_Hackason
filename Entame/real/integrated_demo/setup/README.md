# Offline reconstruction (no robot connection)

`rebuild_runtime.py` builds a new bundle from the integrated source. It never invokes SSH, DDS participants, LocoClient, workers, or motion. Do this on an isolated Linux aarch64 CPython 3.8 build host, not by connecting to G1. Windows can generate and verify the source-only bundle.

## Recorded requirements versus reconstruction pins

- Source: motiondecode-test `78bacb68caa620797042703444a8286223663128`, including scripts/config/data/external and required historical safety artifacts.
- Original startup explicitly requires `/usr/bin/python3`, MuJoCo **3.1.6**, import location `/tmp/motiondecode-hold-deps/mujoco`; source records identify PC2 Python 3.8/aarch64.
- Unitree Python SDK: **vendored snapshot**, not a new SDK clone or latest version. CRC aarch64 binary and BSD notice are included.
- CycloneDDS Python binding **0.10.2**; native C library **0.10.2**, rebuilt for aarch64 from commit `9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd` ([upstream](https://github.com/eclipse-cyclonedds/cyclonedds/tree/9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd)). The checked-in `.so` is x86_64 and must not be deployed to PC2.
- `runtime-requirements.txt` pins all selected runtime Python dependencies (MuJoCo/numpy/DDS, SDK import dependencies, MuJoCo's absl/etils/glfw/OpenGL dependencies). These are a reconstruction recipe for Python 3.8, **not an archived pip freeze**. Desktop's MuJoCo 3.13.0 lock is not the real runtime lock.
- Build-host prerequisites: git, C compiler, cmake, Python 3.8 with pip, Linux aarch64 compatible libc/libstdc++ and OpenGL loader if needed by the wheel. No native library or credential is fetched from G1. Python package licenses remain in installed dist-info.
- Build-tool versions are pinned separately in `build-requirements.txt`; build isolation uses a generated venv under `build/`, never committed or deployed. Compiler/libc versions remain platform-dependent, so this is not a bit-reproducible binary toolchain. CPython 3.8 aarch64 wheels for MuJoCo 3.1.6 and NumPy 1.24.4 were actually retrieved during this audit.

## Build and verify

From `integrated_demo`:

```bash
# Local source-only audit: works on Windows too, does not install packages/network.
python setup/rebuild_runtime.py --output /new/empty/path/source-bundle
python setup/rebuild_runtime.py --output /new/empty/path/source-bundle --verify

# On a separate Linux aarch64 Python 3.8 build host only:
python3.8 setup/rebuild_runtime.py \
  --output /new/empty/path/runtime-bundle --install-deps
python3.8 setup/rebuild_runtime.py --output /new/empty/path/runtime-bundle --verify
```

Output is `code/`, `deps/`, their `MANIFEST.sha256`, `BUNDLE_MANIFEST.sha256`, `REBUILD_INFO.json`. Existing output is never overwritten. C source/build cache under `build/` is not a deployment component and excluded from the outer manifest. `code/external/cyclonedds` in the generated bundle is replaced by the aarch64 build; the repository source is untouched. SHA manifests use the existing restore script's relative `sha256sum -c` format. Verification rejects changed bytes and paths escaping the bundle.

`--install-deps` performs package installation, C compilation and **import-only** checks, not DDS initialization. It has not been executed on this Windows PC. A source-only bundle is marked `SOURCE_ONLY_NOT_RUNNABLE`; do not deploy it as a completed runtime.

The CycloneDDS Python binding is forced to build from its 0.10.2 source distribution against the pinned C library, rather than selecting a wheel with an independently bundled C library. Its [upstream build requirements](https://github.com/eclipse-cyclonedds/cyclonedds-python/blob/0.10.2/pyproject.toml) are satisfied by the pinned build tools. Resident startup sets `CYCLONEDDS_HOME` to the deployed code's native prefix, so the binding resolves after relocation to `/tmp`.

## Runtime directories (later authorized operator work)

After import-only validation on the target architecture, place the generated bundle under `g1-bottle-reaction/.runtime/motiondecode-known-good/` on the operator PC if using the existing restore interface. For a separately authorized deployment, `code/` goes to `/tmp/motiondecode-current`, `deps/` to `/tmp/motiondecode-hold-deps`. The canonical resident startup now resolves Python SDK and native CycloneDDS library inside the deployed code snapshot. Start/stop/ownership gates are unchanged. No deployment or robot connection has been performed in this task.

**Original runtime bundle not archived.** Its exact selected code subset, patches, full transitive versions and hashes cannot be recovered from fixed Git commits. This recipe reconstructs documented runtime requirements and integrated source, but byte identity and measured real-robot equivalence are not asserted. Fresh platform import/ABI checks and attended safety validation remain an operator responsibility; absence of the old generated bundle itself is not a source-integration push blocker.

## YOLO

`python setup/fetch_yolo.py` downloads bytes only from the recorded official v8.3.0 release, never imports torch or runs inference. Model: `yolo11n.pt`; destination: `g1-bottle-reaction/.runtime/models/yolo11n.pt`; **5,613,764 bytes**, SHA256 **0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1**. Source evidence: `g1-bottle-reaction/docs/G1_PERSON_YOLO.md`. Both size and hash must match; otherwise no file is written. Fetch and hash verification succeeded in this audit. Weight remains ignored and outside Git. Review [Ultralytics licensing](https://www.ultralytics.com/license) for usage/distribution; this task does not change project licensing.
