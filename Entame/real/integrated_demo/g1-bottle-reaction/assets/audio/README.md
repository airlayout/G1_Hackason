# Historical reaction audio (not redistributed)

The 2026-09-29 validated G1 development version used these audio assets:

- `reactions/person/detected.wav`
- `reactions/banana/detected.wav`
- `reactions/plushie/detected.wav`
- `reactions/plushie/plushie_affectionate.wav`

They are excluded from this public repository because public redistribution rights have not been confirmed. Exact paths are ignored to prevent accidental re-addition. Original audio may remain in local, untracked storage.

This is a **2026-09-29 validated development snapshot / archival integration**, not a complete runnable distribution. Source code, Reaction mapping, MotionDecode, Patrol and their implementation records are preserved without these WAVs. Existing audio preflight/tests can fail when the files are absent; runtime logic has not been changed to hide that limitation.

To reproduce the real-robot demo, place appropriately licensed PCM WAV files at each path above. The historical files were mono, 44,100 Hz, 16-bit PCM. Replacement audio does not guarantee identical historical sound. Do not commit the excluded files or bypass the ignore rules without verifying redistribution rights.
