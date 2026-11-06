# UAV-X submission preparation

This is a technical draft, not a submitted or fully qualified entry. The team
identity, registration/qualification, official advanced scenarios/log format,
resource/framework acceptance and final submission route remain unconfirmed.

## Core deliverables

- Source and installation/launch instructions in the repository README.
- Exactly 30 UAVs including spares, with explicit charging assumptions.
- Scenario, trajectory, packet, image and landing evidence with independent
  consistency verification; see [validation](UAVX_VALIDATION.md).
- The report/video builder in `scripts/build_uavx_submission.py`, which rejects
  source/evidence errors and retains failed constraint outcomes in the report.
- Current GUI evidence: seed 42 completed at 1970 s with 10/10 PoIs, all
  30 landed and all 14 checks passing. Its five core files match the headless
  matrix case byte for byte; the saved final display was visually inspected.
- [Known gaps](UAVX_FAILURES.md) and the [11-item register](UAVX_ISSUE_REGISTER.md).

The final selected core suite passed 486 tests. All 19 matrix cases finished
and passed evidence consistency verification with unchanged runtime sources:
17 passed constraints and two retained strict radio/permanent-loss failures.
The [portable final summary](verification/uavx/fleet30-core-final.json) includes
every case, configuration, verifier result and source/artifact hashes.

## Verified local artifacts

Location: `submission-artifacts/fleet30-core-20261106`.

| File | Verified content |
|---|---|
| `UAVX-technical-proposal-draft.pdf` | Eight A4 pages; 77,481 bytes; includes all 19 cases with 17 PASS and 2 FAIL labels |
| `demo.mp4` | 74.4 s; 1280 x 720; 15 fps; 780,455 bytes; kinematic replay of the link-outage case |
| `validation-results.json` | All 19 runs verified; constraint failures retained |
| `trajectory.png` | Recorded mission trajectory figure |
| `evidence.zip` | 692 files; 22,097,843 bytes (21.1 MiB); CRC integrity check passed |
| `qa/` | Rendered report pages and visual inspection artifacts |

The primary report/demo run is `link-outage-seed42`, which completes at
1970 s. All eight rendered report pages were reviewed together, with full-size
checks of dense pages 4 and 7. Extracted PDF text confirms all 19 labels and the
17/2 result split. A video frame during the active fault at simulation time
910 s was inspected. The replay identifies its kinematic and synthetic-camera
scope; it is not physical flight evidence.

The evidence archive contains 520,062,396 bytes of uncompressed evidence,
including the full matrix, saved source snapshot/run plan, test results, and
GUI verification with `viewer/final.png`. It compresses these to about 21.1 MiB.
The final committed core tree is archived separately as `source.zip` after the
commit, keeping source provenance tied to the reviewed revision.

The older local `submission-artifacts/fleet30-core-20261105` package is interim
and superseded. The current report is still a technical draft because team and
organizer information remains unconfirmed; artifact QA does not establish
competition submission or framework acceptance.

## Storage scope

City/Unreal work is paused. Its local files are preserved, excluded from the
core PR and package, and not required by the core launcher. Reuse the existing
Python environment and dependencies. Do not package virtual environments,
downloaded city assets, caches, or unrelated DroneOS/Garuda files.

Earlier proposal/performance claims are retained in the
[original package record](history/SUBMISSION_PACKAGE_PRE_FLEET30.md).
No organizer email or form submission has been sent.
