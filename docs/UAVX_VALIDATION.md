# UAV-X validation — 5 November 2026

The two nominal development runs passed all implemented sample constraints.
The outage and permanent-failure runs did not. These results qualify an abstract
kinematic proof of concept under the [declared assumptions](UAVX_REQUIREMENTS.md),
not hidden organizer scenarios, RF hardware, onboard perception or physical flight.

## Environment and scope

- Windows 11, Python 3.10.11; isolated `.venv`.
- NumPy 1.26.4, NetworkX 3.4.2, pytest 8.4.2 (`requirements-uavx.txt`).
- `pip check` passed. No aircraft, PX4 SITL or ProjectAirSim client was connected.
- Separate clone: `AeroMesh-UAVX`, branch `codex/uavx-submission`, based on
  `0ba914d79dc8223866c3333956c31251f5c5f661`.
- The original downloaded AeroMesh checkout and DroneOS/Garuda checkout were
  preserved. A separate read-only DroneOS check passed 119 relevant software
  tests; those results are not swarm or physical-flight qualification.

## Reproduce the regression suite

```powershell
.venv\Scripts\python.exe -m pip check
.venv\Scripts\python.exe -m pytest tests/test_mission_safety.py tests/test_uavx_profile.py tests/test_packet_evidence.py tests/test_evidence_metrics.py tests/test_submission_verifier.py tests/test_comm.py tests/test_route_recovery.py tests/test_interference.py tests/test_simulation.py -q --disable-warnings
```

Final result: **167 passed in 10.68 s**, including 29 verifier regressions and three GCS-partition regressions.
Coverage includes unreachable traffic, duplicate logical packet IDs, edge-count
hop limits, one-hop-per-tick forwarding, stale landing observations, crossing
paths, future tasks, priority changes, removed relay slots, returning replacements,
radio-loss dispatch, speed vetoes and incomplete/fabricated evidence rejection.
The optional hardware/SITL test remains disabled unless explicitly opted in.

## Full mission runs

All runs use 30 UAVs, 0.5 s steps, a 100 m 3D radio cutoff and 2% independent
per-attempt packet loss. Other limits follow the supplied sample slide. The
disturbance is injected at t=900 s. Output folders must be new.

```powershell
.venv\Scripts\python.exe run_simulation.py --profile uavx --seed 42 --output evidence/partition-fix-nominal-42
.venv\Scripts\python.exe run_simulation.py --profile uavx --seed 7 --output evidence/partition-fix-nominal-7
.venv\Scripts\python.exe run_simulation.py --profile uavx --seed 42 --disturbance outage --output evidence/partition-fix-outage-42
.venv\Scripts\python.exe run_simulation.py --profile uavx --seed 42 --disturbance failure --output evidence/partition-fix-failure-42
.venv\Scripts\python.exe verify_submission.py evidence/partition-fix-nominal-42
.venv\Scripts\python.exe verify_submission.py evidence/partition-fix-nominal-7
.venv\Scripts\python.exe verify_submission.py evidence/partition-fix-outage-42
.venv\Scripts\python.exe verify_submission.py evidence/partition-fix-failure-42
```

The runner exits 0 when evidence is written; add `--require-pass` to return 2 on
failed sample checks. The verifier returns 0 only for a passing, consistent run
and 2 for failed constraints. A failed fault benchmark is an expected recorded
result, not a successful competition submission.

| Run | PoIs surveyed / received | Completion (s) | PDR (%) | Connectivity (%) | Failed checks |
|---|---:|---:|---:|---:|---|
| nominal-42 | 10 / 10 | 1906.0 | 99.9800 | 100.0000 | None |
| nominal-7 | 10 / 10 | 2214.5 | 99.9679 | 100.0000 | None |
| outage-42 | 10 / 10 | 1906.0 | 99.7439 | 98.4248 | continuous_connectivity |
| failure-42 | 10 / 10 | Incomplete at 2700 s | 99.8637 | 99.8345 | continuous_connectivity, all_uavs_landed, mission_deadline |

| Run | Minimum separation (m) | Maximum altitude (m) | Longest sortie (s) | Downtime (s) | Post-fault service recovery (s) |
|---|---:|---:|---:|---:|---:|
| nominal-42 | 20.000537 | 97.352871 | 1013.0 | 0 | N/A |
| nominal-7 | 20.000669 | 90.000000 | 1012.5 | 0 | N/A |
| outage-42 | 20.000537 | 97.352871 | 1013.0 | 30.000000 | 8.0 |
| failure-42 | 20.000537 | 97.352871 | 1013.5 | 3.151142 | 8.0 |

Every run had zero battery exhaustions, collision pairs, separation violations,
geofence violations and reporting-deadline misses. Maximum reporting delay was
7.0 s in each. Maximum speed was 5 m/s within floating-point error (less than
2e-13 m/s above 5); the check uses a 1e-8 tolerance, not relaxed flight limits.

## How the measurements should be read

- **Connectivity** is the fraction of active mission time during which all
  operational airborne aircraft have a GCS path. Range-boundary crossings within
  linear motion intervals are evaluated, including the final disarm interval.
  Grounded-only time does not inflate availability.
- **Post-fault service recovery** requires receipt of fresh post-fault packets
  from every member of the affected surviving source set. It excludes the
  disabled aircraft. Thus 8 s service recovery can coexist with 30 s of strict
  all-aircraft connectivity loss in the outage case. It does not mean the whole
  radio network was restored after 8 s.
- **PDR** counts all offered logical packets, including attempts with no route.
  The snapshot can contain queued, nonterminal packets. Finding a route is not
  delivery, and route-recovery counters are separate from fault service recovery.
- **Completion** requires all ten surveyed PoIs to have actual GCS receipts and
  every aircraft to have a measured settled/disarmed landing at its own pad.
  A permanently failed vehicle freezes in this fault model; it remains an
  obstacle and cannot be reclassified as landed. That explains the failure run's
  null completion time despite successful reporting and return of the survivors.
- **Safety** is evaluated for simultaneous linear point-vehicle segments. The
  result is not a physical controller guarantee, and the small clearance margin
  does not establish robustness to tracking error, wind or finite vehicle size.

## Evidence and integrity

Each complete local run contains `metrics.json`, `events.json`, `packets.json`,
`connectivity.json`, `trajectory.jsonl`, and `manifest.json`. The manifest records
the runtime versions, source SHA-256 values captured before execution, and hashes
of all five output files. Hashes describe exact local file bytes; a checkout with
different line endings can have different source hashes without a logic change.
The verifier requires matching runtime source bytes; use the source snapshot in
the local draft package when verifying these exact precomputed runs.

[benchmark-summary.json](verification/uavx/benchmark-summary.json) retains metrics,
manifests and final verification results for review. It is deliberately a compact
summary, not a complete log folder; regenerate the runs or use the local draft
package for the full data. Full logs, old failed development runs, the virtual
environment and generated PDF/video are ignored by Git.

The verifier checks the required artifact inventory and hashes, schema, packet
outcomes, PoI receipts and consistency between summary claims and supporting
measurements. This detects missing/inconsistent evidence; it is not a digital
signature, independent reconstruction of every simulation step, or protection
against coordinated edits to both artifacts and their hashes.

## Report and demonstration

The optional authoring environment uses `requirements-artifacts.txt` and ffmpeg:

```powershell
python scripts/build_uavx_submission.py --runs evidence/partition-fix-nominal-42 evidence/partition-fix-nominal-7 evidence/partition-fix-outage-42 evidence/partition-fix-failure-42 --output submission-artifacts/uavx-2026-11-05
```

The preserved baseline report is an eight-page draft following the supplied template concepts.
Its numerical results also match the partition-fix reruns; source hashes are retained
separately for each version.
All eight rendered pages were visually checked. The H.264 demo is 1280 x 720,
15 fps, 954 frames and 63.6 s; a complete ffmpeg decode reported no errors.
The demo replays nominal seed-42 recorded positions at approximately 30x speed;
range links are reconstructed from positions. It contains no camera imagery or
physical flight. Team identity, eligibility/qualification, official Stage 2
scenario/log compliance, video hosting and final submission review remain open.
No entry was emailed or submitted through a form.
