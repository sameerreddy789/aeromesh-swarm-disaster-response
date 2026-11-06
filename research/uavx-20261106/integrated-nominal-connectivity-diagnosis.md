# Full nominal physics run: connectivity failure diagnosis

The completed seed-42 nominal run surveyed/reported all ten PoIs and grounded/disarmed all 30 aircraft at **2514 s**, but it is a **strict mission failure**: 14 of 15 checks pass and `continuous_connectivity` fails with **19.8 s** of sampled downtime. The independent verifier reports valid evidence and a failed mission. This diagnosis does not change the result, source, radio range, fleet size or safety requirements.

Full evidence remains in the local-only repository directory `evidence/research-physics-nominal-20261106-01`, excluded from Git. The delivered [summary](integrated-nominal-summary.json), [verification](integrated-nominal-verification.json), [evidence index](EVIDENCE.md) and [compact diagnosis JSON](integrated-nominal-connectivity-diagnosis.json) retain the result and provenance. The diagnosis JSON records 19 measured bridge distances, sample times, disconnected sets, input-file hashes and limitations. No physics was rerun and no production file was edited for this diagnosis.

## What is measured

- Mission safety/connectivity is sampled at **20 Hz** in `physics-safety.json`.
- Full body telemetry is **1 Hz**, normally at half-second timestamps, in `physics-telemetry.jsonl`.
- `trajectory.jsonl` is sampled **every 2 seconds**, approximately 0.5 Hz, rather than 2 Hz.
- Bullet/controller stepping is 120 Hz, but this run does not retain all body positions or connectivity sets at that rate.

The five windows contain 396 failing safety samples. Each contributes 0.05 s, totaling 19.8 s. Window endpoints below are the first/last failing samples, so subtracting the endpoints alone omits one sample interval. Exact continuous crossing times lie between samples. Downtime measures time with any required active source disconnected; it is not the sum of disconnected-aircraft seconds.

The availability graph includes GCS and **all operational UAV radios, including grounded aircraft**, then intersects reachability with the armed/operational set captured at the beginning of each 0.5 s mission interval. A grounded intermediate radio can preserve an airborne aircraft's path. This denominator is implemented at [physics_uavx.py:187](../../simulation/physics_uavx.py#L187) and [physics_uavx.py:219](../../simulation/physics_uavx.py#L219), using [uavx.py:320](../../simulation/uavx.py#L320).

## Five outage windows

| Window | Failing samples / accounted duration | Disconnected component at available position samples | Planner phase |
|---|---|---|---|
| W1: 278.80–283.15 s | 88 / **4.4 s** | UAVs **1, 8**; 20 active | Formation pivot from PoI 0 to PoI 6, committed at 242.5 s; three retired aircraft also returning |
| W2: 1235.55–1239.30 s | 76 / **3.8 s** | **0, 1, 6, 7, 8, 12, 13, 14, 18, 19, 20, 21, 24, 25, 26, 27** | PoI 8 to PoI 3 pivot, committed at 1171.5 s; active count grows from 16 to 20 after replacement launches |
| W3: 1309.00–1317.55 s | 172 / **8.6 s** | **0, 1, 6, 14, 21, 27** | Same PoI 3 deployment and replacement transit; active count grows from 20 to 25 |
| W4: 2261.00–2263.65 s | 54 / **2.7 s** | **All 21 active aircraft** | `mission_return`, begun at 2247.5 s after the last PoI survey; nearest relay climbs |
| W5: 2324.80–2325.05 s | 6 / **0.3 s** | 16 disconnected according to safety counts; **exact identities not retained inside this window** | Mission return with staggered descents/landings |

The next good samples are 283.20, 1239.35, 1317.60, 2263.70 and 2325.10 s. Component identities above are independently reconstructed from actual recorded positions at available times, not claimed for every 20 Hz sample. W2's 14 initially grounded radios remain connected to GCS while the original 16 active aircraft lose their common bridge.

## Measured bridge margins

Range margin is `100 m - measured 3D separation`. Positive means inside the declared spherical range; negative means beyond it. These are geometry measurements, not empirical RF measurements.

| Window | Bridge evidence | Alternative or recovery evidence |
|---|---|---|
| W1 | 8–28: **99.365 m at 278.5 s → 101.008 m at 279.5 s**; margin +0.635 → -1.008 m | Reconnects through 1–7, **99.078 m at 283.5 s** |
| W2 | 27–grounded 5: **99.9515 m at 1235.5 s → 100.9185 m at 1236.0 s**; only about 4.85 cm of prior margin | 19–grounded 29 remains **100.314 m at 1238.5 s**; recovery through 24–29 is **99.704 m at 1239.5 s** |
| W3 | 27–8: **98.458 m at 1308.5 s → 101.040 m at 1309.5 s** | Alternate 27–7 is still **100.156 m at 1317.5 s**, then **98.442 m at 1318.0 s** |
| W4 | Sole root bridge GCS–24: **98.583 m at 2260.5 s → 101.590 m at 2262.0 s** | Nearest alternate grounded 8–24 is **112.055 m at 2262.0 s**; GCS–24 returns to **99.591 m at 2264.0 s** |
| W5, brackets only | Old 3–20: **99.572 m at 2324.5 s → 100.731 m at 2325.5 s** | New 9–26: **101.043 m → 99.344 m** over those same bracketing samples |

W4 is a clear **three-dimensional range loss during a climb**, rather than merely horizontal relay spacing. UAV 24 has z=88.715 m and vertical velocity +3.984 m/s at 2260.5 s, then z=91.614 m and vertical velocity +1.903 m/s at 2261.5 s. Its common root link crosses the 100 m range while the other 20 active aircraft depend on it. The 90 m return-height request is not an exact physical-height constraint.

For W5, no 1 Hz telemetry or every-2-s trajectory record occurs inside 2324.80–2325.05 s. The brackets are consistent with an old bridge breaking before its replacement forms, potentially isolating `{2,3,4,5,9,10,11,15,16,17,21,22,23,27,28,29}`. That set and exact within-window bridge are **hypotheses**, not directly observed identities. Linear interpolation must not be presented as a measurement.

## Code-supported cause candidates

1. **Connectivity admission applies to a kinematic reference, not to the actual lagged trajectory.** [physics_uavx.py:100](../../simulation/physics_uavx.py#L100) computes a reference using an isolated kinematic copy. That planner checks linearly interpolated candidate displacement and retained edges at [uavx.py:377](../../simulation/uavx.py#L377) and [resilient_uavx.py:662](../../simulation/resilient_uavx.py#L662). The reference is produced once per 0.5 s outer interval at [physics_uavx.py:186](../../simulation/physics_uavx.py#L186), then tracked through PI, attitude/rate control and motor lag. The measured centimetre-to-metre margins are small compared with the stopping/turning excursions found by the isolated probes.
2. **The actual 20 Hz command filter has no upper-distance connectivity constraint.** [physics_uavx.py:133](../../simulation/physics_uavx.py#L133) adjusts commands for minimum separation, landing approach, geofence and speed. It does not preserve a measured connected backbone or constrain an outward-moving critical edge. Connectivity at [physics_uavx.py:219](../../simulation/physics_uavx.py#L219) is observed after movement. Thus a successful reference admission is not a guarantee that the force-driven plant preserves that link.
3. **Return altitude and path changes need overlap.** [uavx.py:400](../../simulation/uavx.py#L400) requests a 90 m return approach. [resilient_uavx.py:359](../../simulation/resilient_uavx.py#L359) starts a group withdrawal, but measured W4 shows that the nearest relay can lose its only root link while climbing. W5's brackets suggest a second path-transition gap during later landing. A connected endpoint formation is insufficient if intermediate climbs/descents break its critical edge.
4. **Shared path loss produces multiple replacement launches.** Events dispatch replacements for slots 0–2 at 1236 s and slot 3 at 1239 s, then slots 10–12 at 1309 s, slot 13 at 1311.5 s and slot 14 at 1317.5 s. The reason is `gcs_path_lost`. [uavx.py:291](../../simulation/uavx.py#L291) reacts to path loss per slot. These launches are observed consequences; the logs do not establish whether they prolong or shorten each outage. A common cut should be distinguished from several independently failed radios in a future policy.

The kinematic reference contains an emergency-return bypass at [uavx.py:445](../../simulation/uavx.py#L445). Representative returning aircraft near W1 have at least about 790 s of remaining-budget slack above that threshold; at W4/W5 the minimum representative slack is about 436 s. W2/W3 have no returning aircraft at the inspected midpoint samples. These observations do not support blaming the outages on a necessary low-energy exception. They are sampled checks, not a log of every admission decision.

Actual requested/filtered commands, controller integral state and chosen critical-edge sets were not recorded at these times. Therefore the precise contribution of reference ordering, filter alteration, tracking lag and overshoot cannot be isolated retrospectively. The **range failures are confirmed**; the exact counterfactual software repair is still to be tested.

## A separate evidence-timing defect

The `links` list in a trajectory row can lag that row's positions. [uavx.py:572](../../simulation/uavx.py#L572) calls `_network()` before `_move()`, while [uavx.py:636](../../simulation/uavx.py#L636) writes the resulting positions alongside the previously computed network graph. At 1236 s the stored graph still includes 5–27 although same-row separation is 100.9185 m. At 1318 s it omits restored 7–27; at 2264 s it omits restored GCS–24.

This does not invalidate the independent 20 Hz safety failure. For this diagnosis, geometry is reconstructed from timestamped positions. A future logger should either stamp graph acquisition time explicitly or write a post-move graph separately; it must preserve the actual packet-service clock rather than making packet routing appear to have updated at a higher rate.

## Prioritized proposed repair and falsifiable test

**First, make the failing mechanism observable.** At each safety interval record disconnected IDs, component membership, selected root/backbone edges, 3D distance and outward radial rate, requested and filtered commands, and the reason for any constraint override. Preserve acquisition times and distinguish geometric reachability from delivered service. A bounded append-only stream would also avoid retaining all diagnostics until completion.

**Then preserve a measured backbone with tracking margin through every transition.** Maintain a root-connected subset whose critical edges stay below `100 m - margin`, where margin accounts for measured outward velocity, held-command/observation delay, conservative braking and tracking error. Use a reachable-envelope or identified tracking bound; a speed/acceleration request cap is not a proven braking bound. Apply connectivity, minimum separation, geofence and actuator feasibility together, with an explicit infeasible outcome. Stage a root relay's climb/descent or hold it until another measured path overlaps with positive margin. Keep the old relay path until the new one is actually established. No universal margin value is established by this one run.

**Finally, refine shared-cut recovery.** Detect that many missing sources share one failed bridge before dispatching replacements per endpoint. Bound any debounce or recovery delay and keep every outage in the score. This policy change must not relabel a disconnected aircraft or erase its requirement.

The first regression should be this unchanged nominal scenario as a retained failing baseline. A candidate should then repeat all five transition phases with the same 30 aircraft, 100 m range, task schedule, sensing, radio load and reserve constraints. Require zero connectivity downtime at 20 Hz, a finer independent observer or a justified between-sample bound, all existing separation/speed/height/energy/landing checks, and unchanged image deadline semantics. Add controlled outward-speed, climb/descent and old/new-path overlap variations; repeat at a finer physics timestep to distinguish a robust repair from a discretization coincidence.

Small phase fixtures should use declared complete initial conditions, including controller/actuator state when exact replay is claimed. The current sparse logs do not contain enough state for a bit-exact mid-mission replay. Until a candidate passes these tests, retain the nominal result as **evidence-valid, mission-failed** and describe the repair as proposed.
