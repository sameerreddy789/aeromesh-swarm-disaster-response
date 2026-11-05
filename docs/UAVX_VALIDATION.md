# UAV-X validation - 30-aircraft core

The qualification target is **30 UAVs including spares**, with 30 charging pads
in nominal cases and 15 in the resource-stress case. Mission checks retain the
1000 x 1000 m task area, 2700 s deadline, 1200 s sortie, 100 m radio range,
5 m/s speed, 100 m height, 20 m separation and 10 s reporting deadline.
The planner uses a 22 m separation margin. Increasing the fleet or waiving a
failed aircraft's landing is not a repair.

## Final core results

The frozen source completed all **19 declared development cases: 17 passed
and 2 failed strict constraints**. All 12 nominal seeds, three link-outage
cases, the 15-pad resource case and 10% packet-loss case passed. The complete
radio-outage and permanent-aircraft-loss cases remain failures. The overall
development matrix therefore remains failed; no universal compliance or
organizer qualification is claimed.

All 19 cases finished and passed evidence consistency verification. Runtime
source hashes were unchanged from start to finish. The portable
[final matrix summary](verification/uavx/fleet30-core-final.json) contains every
scenario, outcome, verifier result, source inventory and artifact hash. Full
logs remain in `evidence/fleet30-core-final-01`; the exact runtime/test snapshot
and run plan remain in `evidence/fleet30-core-final-01-source`.

The final selected core suite passed **486 tests in 64.89 s**. Dependency
checking (`pip check`) passed. City/Unreal, SITL and the legacy dashboard are
excluded as described below.

| Case | Surveys/images | Landed | Completion | Downtime | Strict result |
|---|---|---|---|---|---|
| `nominal-seed42` | 10/10 | 30/30 | 1970 s | 0 s | Pass |
| `nominal-seed7` | 10/10 | 30/30 | 2414.5 s | 0 s | Pass |
| `nominal-seed100` | 10/10 | 30/30 | 2096 s | 0 s | Pass |
| `nominal-seed101` | 10/10 | 30/30 | 2111 s | 0 s | Pass |
| `nominal-seed102` | 10/10 | 30/30 | 2027 s | 0 s | Pass |
| `nominal-seed103` | 10/10 | 30/30 | 2072.5 s | 0 s | Pass |
| `nominal-seed104` | 10/10 | 30/30 | 2418 s | 0 s | Pass |
| `nominal-seed105` | 10/10 | 30/30 | 2102 s | 0 s | Pass |
| `nominal-seed106` | 10/10 | 30/30 | 1822.5 s | 0 s | Pass |
| `nominal-seed107` | 10/10 | 30/30 | 2492 s | 0 s | Pass |
| `nominal-seed108` | 10/10 | 30/30 | 2469.5 s | 0 s | Pass |
| `nominal-seed109` | 10/10 | 30/30 | 2374 s | 0 s | Pass |
| `link-outage-seed42` | 10/10 | 30/30 | 1970 s | 0 s | Pass |
| `radio-outage-seed42` | 10/10 | 30/30 | 1966 s | 30 s | Fail: connectivity |
| `permanent-failure-seed42` | 10/10 | 29/30 | Not achieved | 0 s | Fail: all-UAV landing/deadline |
| `charging-pads15-seed42` | 10/10 | 30/30 | 2245.5 s | 0 s | Pass |
| `early-link-seed42` | 10/10 | 30/30 | 1970 s | 0 s | Pass |
| `late-link-seed42` | 10/10 | 30/30 | 1970 s | 0 s | Pass |
| `packet-loss10pct-seed42` | 10/10 | 30/30 | 1970 s | 0 s | Pass |

All cases collected and delivered images for all 10 PoIs. The radio-outage
case completed survey/reporting and home landing at 1966 s, but its 30 s of
endpoint downtime fails continuous connectivity. In the permanent-loss case,
all 29 survivors landed at 1973.5 s; strict completion remains null and the run
continues to the 2700 s cutoff because one failed UAV cannot land. Survivor
completion does not replace strict all-UAV completion.

Key runtime SHA-256 values are shown below; the linked summary retains the
complete inventory.

| File | SHA-256 |
|---|---|
| `simulation/resilient_uavx.py` | `c7edda669645edcb82d1e6d5df7445a79d0ec40e5a9c5475ff3915afc4baf7a5` |
| `simulation/uavx.py` | `faf695035c6ab20c3b9035c8abdbba50d251b4b6951efcadfa696f877b9b63ae` |
| `verify_submission.py` | `c6820fe003262df4c041bad86f9270a19f3f641c24f099a7de0273a378c196c7` |

## Current live GUI verification

The same frozen source completed `evidence/fleet30-core-live-final-01` at
1970 s: 10/10 surveys and received images, all 30 UAVs landed, and all 14
checks passing with no verifier errors. Its five core evidence files
(`metrics.json`, `events.json`, `packets.json`, `connectivity.json`, and
`trajectory.jsonl`) are byte-identical to final-matrix nominal seed 42.

The GUI's saved `viewer/final.png` was visually inspected. Desktop capture
could not initialize in this session, so the display check used that saved
surface and telemetry; this is not a claim of a new independent desktop capture.
The [GUI guide](UAVX_GUI.md) preserves earlier baseline measurements separately.
The GUI case shares the final matrix source; its result does not override the
two strict fault failures.

## Report, replay and archive verification

The local `submission-artifacts/fleet30-core-20261106` artifacts were generated
from all 19 final runs, with the passing link-outage case as the primary demo.
The eight-page A4 report was rendered and visually checked; extracted text
contains all 19 cases with 17 PASS and 2 FAIL labels. The 74.4 s, 1280 x 720,
15 fps replay was checked during the active fault at simulation time 910 s.

`validation-results.json` retains all 19 verifier outcomes. The 692-file
`evidence.zip` passed CRC checking and compresses 520,062,396 bytes of evidence
to 22,097,843 bytes. It includes the source snapshot/run plan and the verified
GUI evidence/final surface. See [package contents](SUBMISSION_PACKAGE.md).
These artifacts preserve the strict failures and do not establish organizer
qualification or physical flight.

## Preserved attempts

Every failed attempt remains in local evidence. Results belong to the source
snapshot that produced them; later changes do not inherit an earlier pass.

| Local evidence set | Scope | Outcome |
|---|---|---|
| `fleet30-withdrawal-formal-01` | Nominal 42, 100, 109 | Seeds 42/109 incomplete; seed 100 passed |
| `fleet30-withdrawal-formal-02` | Nominal 42, 100, 109 | 10/10, 7/10, 5/10 tasks respectively |
| `fleet30-withdrawal-formal-03` | Nominal 42, 100, 109 | All passed at 2046, 2100, 2370 s; superseded by canonical-order repair |
| `fleet30-canonical-matrix-01` | Full 19 cases | 16 passed; seed 108 completed 9/10; strict radio/permanent-loss cases failed |
| `fleet30-admission-seed108-01` / `-02` | Seed 108 admission experiments | Both failed; 9/10 tasks |
| `fleet30-admission-seed108-03` | Seed 108 with revised idle energy policy | Passed; 10/10 at 2469.5 s |
| `fleet30-admission-matrix-02` | Full 19 cases | 14 passed; nominal 7/105/107 regressed; strict radio/permanent-loss cases failed |
| `fleet30-readiness-subset-01` | Nominal 7, 105, 107, 108 | First three passed; seed 108 failed |
| `fleet30-readiness-subset-02` | Same four nominal seeds | All passed; superseded by cache and charging/hold corrections |
| `fleet30-charge-safety-subset-01` | Nominal 7, 105, 107, 108 | All passed; superseded by fractional-timestep timestamp normalization |

The canonical complete-radio-outage case surveyed all 10 PoIs and landed all
30 UAVs at 1999.5 s, while retaining its 30 s connectivity failure. The
permanent-loss case surveyed all 10 PoIs and landed all 29 survivors at
2024.5 s; strict all-UAV landing and mission completion failed. These outcomes
explain the fault model and are not universal compliance claims.

## Reproduce the core

Use the existing environment to avoid another installation. Every run needs a
fresh output directory so old evidence is preserved.

```powershell
.venv\Scripts\python.exe run_simulation.py --scenario-file scenarios/uavx/nominal.json --output evidence/new-nominal
.venv\Scripts\python.exe verify_submission.py evidence/new-nominal
.venv\Scripts\python.exe scripts/benchmark_uavx.py --workers 3 --output evidence/new-matrix
```

The default matrix has 12 nominal seeds (42, 7, 100-109), link outages at 180,
900 and 1600 s, a 30 s complete radio outage, permanent aircraft loss, 15
charging pads, and 10% packet loss. It retains constraint failures and crashes.
Exit 0 means all implemented checks passed; exit 2 means completed, consistent
cases failed constraints; exit 1 means process, evidence or source-stability
failure. None establishes submission readiness.

The selected core test command is:

```powershell
.venv\Scripts\python.exe -m pytest tests -q --ignore-glob=tests/test_unreal_*.py --ignore=tests/test_aeromesh_bridge.py --ignore=tests/test_city_streaming.py --ignore=tests/test_valdrada_degenerate_far.py --ignore=tests/test_integration_sitl.py --ignore=tests/test_dashboard.py
```

Paused city/Unreal tests and opt-in SITL are excluded. Legacy dashboard tests
are excluded because Flask is absent in this environment. This command does
not qualify those components or connect to hardware.

## Evidence boundary

Core motion is kinematic. Panda3D renders synthetic scene images; actual PNG
bytes pass through a bounded per-node radio model to the GCS. Verifiers check
artifact completeness, byte delivery, timestamps, capacity ledgers, geometric
constraints and landing consistency. They do not establish calibrated RF,
airframe dynamics, battery behavior or survivor-recognition accuracy.

Unreal/city work is paused for storage and excluded from the core results.
Team identity/qualification, official advanced scenarios and log format,
resource/framework acceptance, root licensing and submission route remain
unconfirmed. Earlier records are retained in the
[pre-30-fleet validation archive](history/UAVX_VALIDATION_PRE_FLEET30.md).
