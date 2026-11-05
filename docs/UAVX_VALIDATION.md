# UAV-X validation — 5 November 2026

Candidate update: 345 targeted tests passed in the clean environment. The latest
movement/landing candidate is running its full matrix in
`evidence/resilience-release-matrix-20261105`. Until that completes, the full
matrix numbers below describe the previous source snapshot, not this candidate.

The current release completed all **19 declared development cases**: **15 pass
and four fail constraints**. Every case has zero verifier evidence errors, and
the benchmark recorded unchanged source hashes from start to finish. The matrix
is not an overall pass. The separate native GUI nominal mission passes all 14
implemented checks. Results concern kinematic motion, rendered scene-camera
images and bounded radio service under the [declared assumptions](UAVX_RESILIENCE.md).
They do not establish hidden organizer-scenario compliance or physical flight.

## Environment and regression scope

Validation used Windows 11 and Python 3.10.11. A fresh environment at
`evidence/repro-environment` was installed using only `requirements-repro.txt`:
NumPy 1.26.4, NetworkX 3.4.2, pytest 8.4.2, pygame 2.6.1, Panda3D 1.10.16 and
Pillow 12.3.0. The isolated targeted suite passed **339 tests in 32.55 s**;
raw output is `evidence/repro-tests.txt`. This is the listed targeted suite,
not a claim that every historical or hardware/SITL test was run.

To reproduce in another fresh environment:

```powershell
py -3.10 -m venv evidence/repro-clean
evidence\repro-clean\Scripts\python.exe -m pip install -r requirements-repro.txt
evidence\repro-clean\Scripts\python.exe -m pytest tests/test_resilient_uavx.py tests/test_resilient_verifier.py tests/test_uavx_scenarios.py tests/test_camera_sensor.py tests/test_radio_service.py tests/test_uavx_gui.py tests/test_mission_safety.py tests/test_uavx_profile.py tests/test_packet_evidence.py tests/test_evidence_metrics.py tests/test_submission_verifier.py tests/test_comm.py tests/test_route_recovery.py tests/test_interference.py tests/test_simulation.py -q --disable-warnings
```

The suite covers scenario validation, bounded service/queue overflow,
source/packet/image provenance, fragment reassembly, fault scheduling, recapture
timing, charging-slot admission, mission safety and original-profile regressions.
GUI unit tests use the SDL dummy driver; the full GUI run below opened the native
AeroMesh window separately. No aircraft, PX4 SITL or ProjectAirSim client supplies
these results.

## Complete development matrix

Full local matrix evidence is saved at:

`evidence/resilience-matrix-20261105/summary.json`

Each case has its saved input under `scenarios/`, process log and verification
result under its named case directory, and full artifacts under `<case>/run/`.
All 19 cases finished and verified. Summary fields are `source_stable=true`,
`source_changed=[]`, `infrastructure_ok=true` and `development_matrix_passed=false`.

The default scenario declares 60 UAVs, 60 charging pads, 720 s full recharge,
25 m pad spacing, 44 m protected relay slots, 128 x 128 rendered PNGs,
131072 bytes/s shared half-duplex service per node and 1048576 queued bytes per
node. It uses a 0.5 s step, 100 m 3D radio cutoff and 2% independent per-attempt
loss. The resource case reduces both fleet and pads to 30; the higher-loss case
changes per-attempt loss to 10%. Sample flight, range, separation, reporting and
mission limits remain unchanged.

| Case | PoIs / images received | Completion (s) | PDR (%) | Connectivity (%) | Maximum received-image delay (s) | Result |
|---|---:|---:|---:|---:|---:|---|
| nominal-seed42 | 10/10 | 2005.0 | 99.9960 | 100.0000 | 6.5 | Pass |
| nominal-seed7 | 10/10 | 2278.5 | 99.9859 | 100.0000 | 9.0 | Pass |
| nominal-seed100 | 10/10 | 2462.5 | 99.9983 | 100.0000 | 7.0 | Pass |
| nominal-seed101 | 10/10 | 2161.0 | 99.9961 | 100.0000 | 9.0 | Pass |
| nominal-seed102 | 10/10 | 1987.5 | 99.9812 | 100.0000 | 5.5 | Pass |
| nominal-seed103 | 10/10 | 2019.0 | 99.9868 | 100.0000 | 7.5 | Pass |
| nominal-seed104 | 10/10 | 1839.0 | 99.9736 | 100.0000 | 8.5 | Pass |
| nominal-seed105 | 10/10 | 2372.0 | 99.9805 | 100.0000 | 8.0 | Pass |
| nominal-seed106 | 10/10 | 1904.5 | 100.0000 | 100.0000 | 7.5 | Pass |
| nominal-seed107 | 10/10 | 2336.0 | 99.9909 | 100.0000 | 7.5 | Pass |
| nominal-seed108 | 10/10 | 2387.5 | 100.0000 | 100.0000 | 6.5 | Pass |
| nominal-seed109 | 10/10 | 2597.0 | 99.9713 | 100.0000 | 8.0 | Pass |
| link-outage-seed42, t=900 s | 10/10 | 2005.0 | 99.9960 | 100.0000 | 6.5 | Pass |
| radio-outage-seed42, t=900 s | 10/10 | 2007.0 | 99.9364 | 98.5041 | 6.5 | Connectivity fails |
| permanent-failure-seed42, t=900 s | 10/10 | Incomplete at 2700 | 99.9919 | 100.0000 | 6.5 | All-UAV landing and deadline fail |
| resource30-seed42 | 7/10 | Incomplete at 2700 | 96.8466 | 85.5392 | 6.0 | Survey/report completion, deadline, connectivity, reporting and imagery checks fail |
| early-link-seed42, t=180 s | 10/10 | 2005.0 | 99.9960 | 100.0000 | 7.0 | Pass |
| late-link-seed42, t=1600 s | 10/10 | 1979.5 | 99.7888 | 98.2609 | 23.5 | Connectivity and reporting deadline fail |
| packet-loss10pct-seed42 | 10/10 | 2005.0 | 99.9960 | 100.0000 | 7.5 | Pass |

The delay column includes received images only. In resource30, three PoIs have
no detection/report, so a 6.0 s maximum among seven received images does not
establish reporting compliance. A numerical completion time records survey/image
delivery and landing; it does not override failed communication/reporting checks.

Across all 19 cases, battery exhaustions, collision pairs, separation violations,
geofence violations and queue-overflow drops are zero. Aggregate minimum
separation is **22.000023716 m**, maximum altitude **99.965243520 m**, and longest
sortie **1123.0 s**. Maximum measured speed is 5 m/s within floating-point error
(at most 2.17e-13 m/s above 5); the existing 1e-8 tolerance is unchanged.
Zero queue overflow in this workload does not establish performance under
arbitrary congestion; targeted tests exercise that behavior separately.

## Interpret the negative cases

| Case | Downtime (s) | Fresh affected-source service after injection (s) | Additional evidence |
|---|---:|---:|---|
| Link outage at 180 s | 0 | 6.5 | The selected link is disabled for 30 s; alternate service maintains strict connectivity. |
| Link outage at 900 s | 0 | 6.0 | The selected link is disabled for 30 s; all 14 checks pass. |
| Radio outage at 900 s | 30.0 | 6.5 | UAV 36's sole radio is unavailable until 930 s. Other-source service does not restore that endpoint. |
| Permanent failure at 900 s | 0 | 6.5 | The failed UAV stays an obstacle. The 59 operational survivors finish survey and return by 1992.5 s; strict all-UAV completion is absent. |
| Link outage at 1600 s | 34.399303 | 37.5 | PoI 9 detection at 1615.0 s completes image receipt at 1638.5 s, missing the 10 s deadline. |

Connectivity includes operational airborne UAVs. A permanently failed UAV is
excluded from that network denominator but remains in the strict landing test.
The radio-disabled aircraft is still operational and airborne, so its outage
is included. Grounded-only intervals do not inflate mission availability.
Service recovery requires fresh post-fault receipts from every recorded affected
surviving source. It does not assert that all radios recovered.

The late-link case demonstrates that the moving network is not protected at
every transition. The resource30 case demonstrates dependence on the declared
larger resources. Both remain failures in the [failure register](UAVX_FAILURES.md).

## Reproduce missions and verification

Use a new output path for each invocation:

```powershell
.venv\Scripts\python.exe scripts/benchmark_uavx.py --output evidence/new-resilience-matrix --workers 2
.venv\Scripts\python.exe run_simulation.py --scenario-file scenarios/uavx/nominal.json --gui --output evidence/new-resilience-gui --auto-close --require-pass
.venv\Scripts\python.exe verify_submission.py evidence/new-resilience-gui
```

The matrix defaults to all 19 cases. `--quick` runs only four named cases and
must not be presented as the complete matrix. `--seeds` changes the nominal seed
list; saved input files identify precisely what was tested. The benchmark returns
2 for consistent constraint failures, 1 for process/evidence/source errors and
0 only when every declared case passes. The current four failures prevent exit 0.

Separately verified native GUI evidence is `evidence/resilience-gui-qualified`.
It finished at **2005.0 simulated seconds**, surveyed and received all 10 PoI
images, and passed all 14 checks, including all-UAV landing. Its verifier reports
`evidence_errors=[]`, `failed_constraints=[]`, `sample_run_passed=true` and
`submission_ready=false`. Saved `viewer/` images show the live AeroMesh state;
the displayed camera image is a completed GCS receipt, not an onboard bypass.

## Evidence inventory and integrity

Schema 2 records the original five artifacts (`metrics.json`, `events.json`,
`packets.json`, `connectivity.json`, `trajectory.jsonl`) plus `scenario.json`,
`radio-service.json`, `sensing/observations.json`, capture PNGs and received PNGs.
The manifest hashes these files and records actual runtime source bytes.
Capture metadata includes source UAV, acquisition time, pose/intrinsics, scene
fingerprint and image hash. Complete image receipt is reconstructed from delivered
fragments; a first fragment alone cannot establish reporting success.

The verifier checks source/artifact inventories, packet provenance, assembled
image bytes, per-node capacity and supporting survey/receipt/landing times.
It detects missing or inconsistent evidence. It is not a digital signature,
independent reconstruction of all dynamics, or protection against coordinated
edits to artifacts and hashes. Exact source bytes, including line endings, must
match; use each delivery's source snapshot for its saved runs.

## Preserved historical baselines and submission boundary

The earlier 30-UAV, footprint-only release remains in
[gui-benchmark-summary.json](verification/uavx/gui-benchmark-summary.json), and
the partition-recovery baseline in
[benchmark-summary.json](verification/uavx/benchmark-summary.json). Its 172-test
result, 1906.0 s nominal seed-42 result, earlier eight-page proposal and trajectory
demo describe that older version. They are not the current 60-UAV image-delivery
release's evidence. Prior source snapshots and delivery updates remain intact.

Current report/video packaging must identify these 19 outcomes and sources.
Team/qualification information, official scenarios/log schema, allowable
resources, simulator acceptance, failed-aircraft scoring, license and final
submission review remain open. Physical drones, a particular 3D engine,
decentralized planning and learned survivor recognition are not being added as
unsupported mandatory requirements. No entry has been emailed or submitted.
