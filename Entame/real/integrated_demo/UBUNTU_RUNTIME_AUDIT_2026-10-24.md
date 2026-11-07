# Ubuntu G1 runtime audit (2026-10-24)

This note records a read-only audit of the Ubuntu development environment used
through 2026-09-30. It does not change the archived runtime, safety gates,
motion data, or launch behavior.

## Result

The newest complete paired source found on Ubuntu is:

| Component | Ubuntu source | Commit |
|---|---|---|
| Reaction / Patrol / integrated supervisor | `/home/ubuntu/dev/g1-bottle-integration-20260926` | `506d6cbd849502139be223ec7def74b3d75feaaa` |
| MotionDecode resident / safety runtime | `/home/ubuntu/dev/motiondecode-test` | `78bacb68caa620797042703444a8286223663128` |

Both worktrees were clean for tracked files. The MotionDecode checkout contained
untracked `output/` telemetry, previews, and logs; these remain local and are not
source dependencies. No stash existed in either repository.

`SNAPSHOT_MANIFEST.json` already names exactly these two commits. A source/tree
review found no later Ubuntu source edit from 2026-09-30. The integrated snapshot
therefore remains the correct public source snapshot; it was not replaced with
`main` or with a newer unvalidated feature branch.

The commonly named checkout `/home/ubuntu/dev/g1-bottle-reaction` is not the
latest runtime: it remains at `42bf90edbe99584bb57f821e7e0f6281197768c0`
on `recovery/integrated-demo-20260926`. Do not use that checkout as evidence that
the newer recovery worktree is absent.

## Runtime path and interfaces

The canonical preserved integration is
`g1-bottle-reaction/scripts/run_integrated_demo.py` with Patrol, Reaction, and
the resident MotionDecode protocol. Patrol exposes a local Unix control socket;
Reaction pauses Patrol, waits for stationarity, sends a named reaction request
to the resident MotionDecode socket over a persistent SSH stdio bridge, then
requires a safe resident postflight before Patrol may resume.

The canonical supervisor explicitly leaves Wander off. The separate
`scripts/start-integrated-demo.sh` still selects the legacy `--with-wander`
route and a robot-side `/home/unitree/g1-bottle-reaction-wander` path. It is
preserved for history but is not the canonical validated Patrol integration.

Required development metadata is present: `g1-bottle-reaction/pyproject.toml`,
`motiondecode-test/requirements.lock.txt`, the MotionDecode resident/client
scripts, Patrol relays, configuration YAML/JSON, and the integrated launch
scripts. Robot-side aarch64 native dependencies and the original generated
runtime bundle were not reconstructed during this audit.

## GitHub comparison

### A. Ubuntu-only findings before this audit

- Five commits reachable from the local
  `stabilization/reaction-runtime-20260926` and
  `stabilization/vision-runtime-20260926` branches were not reachable from any
  remote branch. They are alternate 2026-09-26 stabilization work, not the
  2026-09-29 runtime pair. They were archived without merging as
  `archive/ubuntu-reaction-runtime-20260926` and
  `archive/ubuntu-vision-runtime-20260926` in `IharaTakumi/g1-bottle-reaction`.
- Local MotionDecode `output/` artifacts and the two 2026-09-29 demo logs were
  not committed. They remain excluded because they are generated telemetry/logs,
  not required source.

### B. GitHub-only or divergent work

- `g1-bottle-reaction` has divergent 2026-09-27/28 branches for resident binding,
  control leases, correlated stops, locomotion ownership, Wander ownership, and
  stationary calibration/evidence.
- `motiondecode-test` has divergent
  `fix/atomic-resident-session-20260927`.
- These branches were not part of the preserved SHA pair and were not folded
  into the runtime merely because they are newer. They require separate review
  and physical validation.

### C. Matching work

- The source commits above match the source IDs in the snapshot merged by
  `airlayout/G1_Hackason` PR #34.
- Current `Dev/Entame` still contains the same runtime snapshot. Later changes
  under the integrated demo are documentation only.

### D. Old, duplicate, or non-source material

- `/home/ubuntu/dev/g1-bottle-reaction` at `42bf90e` is an older checkout.
- Python caches, venvs, sockets, PID files, SSH ControlMaster files, model caches,
  raw telemetry, videos, previews, and large generated outputs are not promoted.
- Four WAV files with unresolved redistribution rights remain excluded from this
  public integrated snapshot, as recorded by the existing license review.

### E. Items requiring a future decision

- Whether to port any divergent control-lease/ownership/stationarity branches
  onto the preserved runtime must be decided independently and revalidated on G1.
- The exact raw log for the reported 2026-09-30
  `Arms are not stationary` FAULT was not found. The MotionDecode code path that
  emits it is preserved, but this audit does not infer missing telemetry.
- A read-only SSH attempt to `unitree@10.42.0.76` timed out, so the current G1
  filesystem could not be hash-compared. No service or file on G1 was changed.

## Retained test evidence

All checks used detached audit worktrees and did not run robot control:

- Python compile of the G1 and MotionDecode source/entrypoint trees: pass.
- Focused G1 Patrol/Reaction/MotionDecode interface tests: **89 passed**.
- Focused MotionDecode resident/safety/runtime tests: **75 passed**.
- Integrated supervisor with `--no-locomotion --dry-run`, loopback route, and an
  existing local YOLO model: pass; output included `NO G1 COMMAND SENT` and
  `WANDER=OFF`.
- Focused secret and sensitive-filename scan: no credential, private key, token,
  `.env` payload, or SSH key file found. Matches were code identifiers/test text.
- Diff whitespace check for the source changes: pass.

The retained `logs-20260929/g1-3lap-reaction.log` records a MotionDecode FAULT
caused by a stale successful ownership check, after the arms-stationary marker;
it also records a later SSH camera exit. This is failure evidence, not a claim
of a clean final physical run. The separately reported 2026-09-30
`Arms are not stationary` event has no retained raw log in the audited paths.

## Source preservation branches and review

- `IharaTakumi/g1-bottle-reaction:preserve/ubuntu-runtime-20260930` points to
  `506d6cbd849502139be223ec7def74b3d75feaaa` and is proposed by
  [IharaTakumi/g1-bottle-reaction PR #1](https://github.com/IharaTakumi/g1-bottle-reaction/pull/1).
- `IharaTakumi/motiondecode-test:preserve/ubuntu-runtime-20260930` points to
  `78bacb68caa620797042703444a8286223663128` and is proposed by
  [IharaTakumi/motiondecode-test PR #1](https://github.com/IharaTakumi/motiondecode-test/pull/1).
- Both are proposed to their respective `main` branches by review-only PRs. No
  branch was force-pushed or auto-merged.

## Recommended development base

Until the source PRs are reviewed, use the two `preserve/ubuntu-runtime-20260930`
branches as the source-of-truth pair and this `Dev/Entame` snapshot as the public
portable archive. After review, the source repositories' merged reviewed heads
should become authoritative; do not select an older `main` merely because it is
the default branch.

For the next physical session, first fetch the two exact source commits, restore
only locally licensed audio/model files, rebuild the documented aarch64 runtime
outside the robot, and run the existing read-only readiness checks. Resolve the
missing stationarity/ownership evidence before starting the resident. Only then,
with the normal site and operator approvals, use the canonical
`run_integrated_demo.py` path documented in `README.md`; do not substitute the
legacy Wander launcher.
