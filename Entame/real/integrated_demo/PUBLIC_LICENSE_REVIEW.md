# Public distribution review

Review scope: integrated snapshots and bundled third-party material, not merely the original repository's visibility.

| Material | Evidence / notice | Result |
|---|---|---|
| Original g1-bottle-reaction project code | pyproject declares MIT; user authorizes this integration. No standalone LICENSE in the fixed SHA | User-authorized source; no third-party license invented |
| Original motiondecode-test project scripts/config/tests | User explicitly authorizes inclusion despite original private repository | Private visibility is not a blocker; no blanket third-party rights inferred |
| Unitree SDK, example, model/meshes, CRC libraries | `external/UNITREE_SDK_LICENSE`, `external/g1_model/LICENSE`, provenance in motiondecode README | BSD-3-Clause notices retained; redistribution condition clear |
| CycloneDDS headers/native library | `external/CYCLONEDDS_LICENSE`; native source pinned in setup | EPL-2.0/EDL-1.0 notice retained, upstream source/version link supplied; generated build preserves notices in source tree |
| Viewer `external/csv_to_animation.py` | `external/VIEWER_LICENSE`, Viewer README | MIT attribution to Beyond Capture retained |
| Chingmu motion CSV 10 files and derived selected_arms.csv | `external/MotionDecode_LICENSE.md`, metadata/downloads.json pinned revision/hash, original README | Permitted non-commercial prototyping/research; attribution retained. No commercial/re-sale/competing dataset hosting permission asserted. This limited demo is documented as non-commercial prototyping; broader distribution/use needs written permission under the supplied terms |
| MotionDecode README/Log and metadata | Original Chingmu provenance and retained terms | Preserve attribution/terms alongside motion data |
| YOLO | Official Ultralytics v8.3.0 release and recorded matching hash; weight not committed | AGPL-3.0/Enterprise terms apply to usage; setup refers to official license. No Ultralytics source or weight vendored in this commit |
| GVHMR/GMR/SMPL-related tools | Original project integration glue and links, no GVHMR/GMR clones or SMPL models copied | Optional generation dependency, not bundled demo third-party models |

## Specifically unresolved files

The following WAVs lack an identified creator/model-specific license or public redistribution statement in the fixed snapshot:

- `g1-bottle-reaction/assets/audio/reactions/person/detected.wav` — historical hash-named audio; available text does not identify voice/model or rights.
- `g1-bottle-reaction/assets/audio/reactions/banana/detected.wav` — source docs call it user-provided `banana_surprised.wav`, but do not record public redistribution rights.
- `g1-bottle-reaction/assets/audio/reactions/plushie/detected.wav`
- `g1-bottle-reaction/assets/audio/reactions/plushie/plushie_affectionate.wav` — these two files have the same source Git blob; docs describe user-provided audio without a rights statement.

No copyright infringement is asserted. The files are retained locally to preserve the validated demo, but public redistribution permission cannot be concluded from the current repository evidence. User confirmation of authorship/redistribution rights or a source license is required before publishing these specific assets. A blanket AivisSpeech Engine license would not prove the license of an unidentified voice model or these WAVs.

No additional third-party file with an identified conflicting redistribution condition was found. This review does not relabel all private-origin code or all optional dependencies as blockers.
