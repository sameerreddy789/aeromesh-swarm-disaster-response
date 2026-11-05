# UAV-X issue register - 6 November 2026

**Fleet decision: 30 UAVs.** Future challenge qualification, the city prototype,
resource tables and demonstration must use that same total, including spares.
Earlier 60- and 72-UAV results are historical diagnostics. They cannot satisfy
the 30-UAV acceptance gate. Charging pads are finite and must be explicitly
reported; fleet size cannot be increased to turn a failed case into a pass.

**Current scope: core reliability.** City/Unreal work is paused because of
storage constraints. Existing city work and evidence are preserved and excluded
from the core PR. City flight-backend and physics qualification remain incomplete.

This register retains the original eleven items from
[the baseline audit](history/UAVX_FAILURES_GUI_BASELINE.md). They combine code
defects, modeling limits and qualification dependencies. “Resolved” requires
the stated evidence; a written plan is not a fix.

| # | Original issue | Current status | Evidence and remaining acceptance boundary |
|---|---|---|---|
| 1 | Scenario robustness | Final source completed 19 cases with valid, source-stable evidence: 17 passed, including all 12 nominal seeds; strict radio/permanent-loss cases failed. Earlier failed candidates remain preserved. | The [final matrix](UAVX_VALIDATION.md) covers nominal seeds, declared faults, energy/landing and resource limits. The two strict failures remain open under their declared semantics. Hidden scenarios remain unqualified until provided and tested. |
| 2 | Fleet and staging assumptions | Resolved fleet choice: 30 including spares. The final matrix uses 30 nominal pads and tests 15 shared pads. Organizer resource acceptance and physical staging remain open. | The same 30-aircraft configuration in the core launcher, scenario, evidence and report, with measured pad/resource usage and no undeclared spares. The city bridge must meet the same total when that work resumes. |
| 3 | Missing physical dynamics | Core motion remains kinematic. Preserved Unreal Chaos experiments provide partial mechanics evidence; city/Unreal work is paused for storage and full physics qualification remains open. | Complete measured city missions with collision volumes, bounded force response, landing and obstacle tests. World-frame actuation is not a calibrated four-motor model; wind/aerodynamics need separate implementation/validation. This work is outside the core PR. |
| 4 | Simplified radio | Bounded service, queues, retries and image-byte accounting implemented. Obstacle propagation/interference remain open. | Reproducible modeled RF tests including building obstruction and competing links; parameter assumptions and independent radio-calibration limits stated. |
| 5 | Centralized state access | Architecture limitation remains explicit. Centralized coordination is permitted by the supplied template with justification. | Document the authority boundary and failure policy. If distributed operation is claimed, demonstrate stale/lost-state behavior without a global-state oracle. |
| 6 | Synthetic perception | Core rendered-camera capture and bounded PNG-to-GCS transport exist. City native-camera work is preserved but paused and excluded from the core PR. Proximity triggers remain synthetic. | Actual camera frames, timestamps, intrinsics/provenance and GCS receipt for each qualified backend; fixture detection quality with missed/false detections. Clean city-native recapture/delivery remains unqualified. No claim of general survivor recognition from proximity. |
| 7 | Uncalibrated energy | Finite endurance/recharge accounting, shared deployment/return budgets and bounded headroom preference are implemented. Non-finite state rejects admission. Charge and hold safety corrections pass focused regressions and the final source matrix retains no battery-exhaustion failures. Physical energy values remain assumptions. | Explicit physical power/battery parameterization and energy-sensitive trajectories, reserve/charging tests, and a distinction between simulated estimates and measured airframe calibration. |
| 8 | Permanent-failure recovery | The final permanent-loss case completes 10/10 PoIs and lands all 29 survivors at 1973.5 s. Strict all-UAV landing/deadline fail; an immobile/destroyed vehicle cannot satisfy them. | Test recoverable control/radio faults and spare replacement, keep survivor and all-UAV metrics separate, and record unrecoverable physical loss honestly. No deletion or relabeling of a failed aircraft to obtain a pass. |
| 9 | Flight backend | Core uses explicit kinematic state. The existing Unreal bridge is preserved; city work is paused for storage, excluded from the core PR, and not fully qualified for missions or restart/expiry behavior. | Planner commands drive forces; actual telemetry drives planning. Verify 30 identities, no pose animation, command expiry, initialization, landing and reproducible launch before claiming city-backend qualification. |
| 10 | Evidence limits | All 19 final cases passed consistency verification with unchanged source hashes; 486 core tests passed. GUI seed 42 matches the headless core files byte for byte. Ordered-owner/handoff and motion comparisons pass across tested Python hash seeds. City evidence remains partial and outside the core PR. | Exact source snapshots, output checksums, measured trajectories, independent geometric/mechanical cross-checks and explicit gaps. Hashes alone never prove physical realism; a small determinism fixture does not qualify a mission matrix. |
| 11 | Broken nested repository reference | Repaired by removing the unused unmapped gitlink while preserving local contents. | Git index contains no unmapped dependency; maintained source imports and selected core tests pass. Fresh-machine installation is a separate reproduction check. Original pointer recorded in [source layout](LEGACY_SOURCE_LAYOUT.md). |

## Current observations

The [validation record](UAVX_VALIDATION.md) and
[portable final summary](verification/uavx/fleet30-core-final.json) record all
19 cases, exact sources, evidence hashes, reproduction commands and exclusions.
The final source passes all 12 nominal seeds, three link-outage cases, the
15-pad resource case and 10% packet loss. Sole-radio outage retains a 30 s
connectivity failure; permanent loss retains strict landing/deadline failures.
The selected core suite passed 486 tests and dependency checking passed.

The planner searches complete role assignments under deployment, retiring-
aircraft wait, energy and deadline bounds. Squared travel cost reduces crossed
assignments; a bounded headroom preference favors a viable cohort. Task
admission tries feasible announced alternatives at the same priority while
preserving unfinished equal-priority work. Connected withdrawal coordinates
return; observed replacement handoffs can finish during announcement gaps.

Idle waiting permits a 30 s minimum grace only above the hard return reserve.
Completed formations use a copied charge forecast before waiting for a larger
task. The forecast grants no real charge, arrival, ownership or clock credit;
actual admission still requires actual readiness. Useful follow-up headroom is
required on the capable subset for already-announced work, not every current
owner and not unannounced future tasks.

A final independent review found failed charging UAVs could be revived and
consume queue service, charge completion could be one step later than the
forecast, and a low-energy non-owner replacement could be overlooked during a
hold. The corrections pass 89 focused tests and all four affected nominal
seeds. Actual charge service and forecast
now share operational/landed/disarmed eligibility and timestep arithmetic; the
hold reserve covers airborne replacements as well as current owners. The
final source, including fractional-timestep timestamp correction, completed
the 19-case matrix and the 486-test selected core suite.

A subprocess counterexample showed that matching-result set order affected
later greedy replacement identities and motion. Commit now orders owner and
handoff maps by slot. The [determinism regression](../tests/test_fleet30_determinism.py)
compares owners, dispatch identities, handoffs and all 30 positions after 20 s
across Python hash seeds 0, 1 and 2. See the
[formal-02 diagnosis](UAVX_FLEET30_FORMAL02_DIAGNOSIS.md) for the earlier movement
and idle-handoff failure evidence. Small regressions do not replace full missions.

## Paused city observations (historical)

The first city integration run is retained under
`evidence/unreal-city-run-20261105T144115Z`. Three assigned aircraft climbed under
Chaos force control. A Windows reader lock stopped the planner's status-file
replacement; command expiry then braked the aircraft to stationary holds. This
is a failed mission integration attempt, not a successful completed mission.
The snapshot writer was corrected with bounded retries and regression tests.

A separate unforced physics body fell onto the imported source building at the
district origin. Its measured lower bound was 23.566696606 m versus the exported
roof height 23.566695690 m, with zero final velocity. This qualifies that contact
fixture only, not every city surface or all flight paths.

These preserved experiments do not establish complete city physics or backend
qualification. City/Unreal work remains paused for storage and excluded from
the core PR.

## External information still unavailable

Team identity/registration/qualification, organizer-issued advanced scenarios,
required final log schema, resource/framework acceptance and current submission
route have not been supplied. Root-project licensing remains unresolved.
These are tracked dependencies, not invented software requirements or assumed
approvals. No organizer communication or submission has been sent.
