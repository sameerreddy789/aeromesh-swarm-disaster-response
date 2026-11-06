# Changes and errors found - 6 November 2026

Original repair baseline: `0ba914d79dc8223866c3333956c31251f5c5f661`.
Working branch: `codex/uavx-submission`. Original DroneOS and downloaded AeroMesh
checkouts were preserved; implementation lives in a separate AeroMesh-UAVX clone.

## Separate rigid-body backend - 6 November 2026

Added an asset-free, fixed-30-UAV Bullet backend with four motor forces,
attitude/inertia, motor lag, gravity, drag/wind, collision contact and power-cut
freefall. The existing mission algorithms consume measured body telemetry;
an isolated planning copy supplies velocity references only. The native 3D
observer has procedural quadcopters, eagle/close views and zoom/pan controls.
City/Unreal files remain paused and excluded from this addition.

Physics-specific integration repairs include slower-speed cohort budgets,
braking-aware velocity constraints, contact-based settled landing, cumulative
collision reporting and 120 Hz speed extrema. Testing caught an endurance
accounting gap on the interval ending in disarm; consumption is now measured
per 20 Hz control interval. Stale/disarmed raw motor overrides cannot reactivate
through a later normal command. The viewer retains body nodes until closed.

The [physics guide](UAVX_PHYSICS.md) describes launch commands, evidence and
model limits. The original kinematic runtime files and packaged matrix remain
unchanged. Physics results require their own verifier and cannot inherit the
previous kinematic qualification. The model is uncalibrated; electrical battery,
propeller aerodynamics, city collision geometry and physical flight remain open.

## Admission and readiness correction - 6 November 2026

The canonical matrix completed all 19 cases with valid, source-stable evidence:
16 passed, seed 108 completed only 9/10 tasks, complete radio outage failed
connectivity for 30 s, and permanent loss failed strict all-UAV return/deadline
while all 29 survivors completed. Those results remain in
`evidence/fleet30-canonical-matrix-01`.

The repair addresses several distinct scheduling failures:

- An infeasible nearest new task could block feasible announced work of equal
  priority. Admission tries alternatives at that priority; it still preserves
  unfinished equal-priority work and excludes future/lower-priority shortcuts.
- A healthy connected cohort was forced to land when a larger announced task
  lacked enough ready aircraft. It can continue its existing work above the
  common reserve instead of wasting a landing/recharge cycle.
- Blind holding could consume the energy needed for later tasks. The idle
  policy applies the hard return reserve immediately and the additional 40 s
  survey/report allowance plus 180 s readiness headroom after a 30 s grace.
  This grace does not inspect future task announcements.
- A completed cohort now forecasts whether already-charging aircraft can make
  the next task feasible before its own useful energy expires. The forecast
  copies flight records and the planning clock, considers the finite service
  queue, and does not assume a returning aircraft's arrival or alter real state.
- Follow-up headroom is required on enough capable aircraft for the largest
  already-announced follow-up task. Requiring it on every owner of a larger
  current formation wrongly rejected feasible small follow-up work.
- Cached holding is invalidated by new announced work, operational-aircraft
  changes, projected readiness expiry and formation changes. Actual measured
  charge readiness can admit work earlier; a forecast never grants readiness.

The first two seed-108 attempts failed; the third passed at 2469.5 s. The
subsequent full `fleet30-admission-matrix-02` passed 14/19, exposing regressions
in nominal seeds 7, 105 and 107. `fleet30-readiness-subset-01` then passed those
three but failed seed 108. The revised `fleet30-readiness-subset-02` passed all
four at 2414.5, 2102, 2492 and 2469.5 s, respectively. All attempts remain
preserved. These results precede the later cache and charging/hold review;
the final source results are recorded below.

Final review corrected three additional defects:

- Failed charging aircraft could receive service and be revived. Actual service
  and forecasting now share operational, landed and disarmed eligibility; an
  ineligible aircraft cannot consume shared charging service or receive energy.
- Floating-point accumulation could complete one timestep later than its
  forecast. Forecasting repeats the actual charge increment, with work bounded
  by the remaining mission horizon even when recharge is extremely slow.
- A finite low-energy non-owner replacement could be ignored by a formation
  hold. The reserve guard covers every operational non-returning airborne UAV,
  including replacements and unassigned aircraft.

The corrected source passed 89 focused tests and all four affected nominal
seeds in `fleet30-charge-safety-subset-01`, at the same completion times as
`fleet30-readiness-subset-02`. A final fractional-timestep review found event
timestamps could drift from the rounded model clock and delay charging by
one step. Movement observations, landing and charge timestamps now use the
same rounded clock boundary. The new regression and 77 focused tests pass.
The final source completed all 19 cases in `fleet30-core-final-01` with
unchanged runtime hashes and valid evidence: 17 passed; sole-radio outage
failed connectivity for 30 s, and permanent loss failed strict all-UAV landing
and deadline. All 12 nominal seeds, three link-outage cases, the 15-pad case
and 10% packet-loss case passed. The final selected core suite passed 486 tests
in 64.89 s and dependency checking passed. The GUI seed-42 run completed at
1970 s with all 14 checks passing and five core files identical to the headless
case. The [portable final summary](verification/uavx/fleet30-core-final.json)
retains all outcomes and source/artifact hashes.

## Final core artifacts - 6 November 2026

Generated `submission-artifacts/fleet30-core-20261106` from all 19 final runs,
using the passing 900 s link-outage case for the primary replay. The report is
eight A4 pages; its rendered pages were reviewed and extracted text confirms
17 PASS and 2 FAIL labels. The 74.4 s, 1280 x 720, 15 fps video was inspected
during the active fault. The 692-file evidence archive passed CRC verification
and occupies 22,097,843 bytes, including matrix, source snapshot and GUI evidence.
The final source tree is archived from its commit as `source.zip`. The earlier
`fleet30-core-20261105` artifact set remains preserved as interim/superseded.

## Earlier 30-UAV matching and handoff repair

The declared fleet is 30 UAVs including spares. Earlier 60- and 72-UAV results
are historical diagnostics and cannot qualify this configuration. Formal-03
completed 10/10 PoIs for nominal seeds 42, 100 and 109 at 2046 s, 2100 s and
2370 s, respectively, with zero downtime and all 30 UAVs landed in every run.
Those exact-source results are preserved but superseded by the canonical-order
repair described below. Its completed 19-case canonical matrix passed 16
cases; the later admission/readiness correction is described above. The earlier formal-02 candidate completed 10/10, 7/10 and 5/10 PoIs
for the same seeds and remains preserved as failed matrix evidence.

- Replaced incomplete greedy role admission with complete injective matching
  within candidate energy, retiring-aircraft wait and mission-deadline bounds.
  An inexpensive but infeasible assignment no longer hides a feasible complete
  alternative; ownership and flight clocks change only at the appropriate
  commit or measured-motion stage.
- Uncrossed the selected cohort's assignments using squared travel cost within
  the same deployment bounds. This addresses the observed forward/backward
  oscillation behind settled relays; bounded-motion fixtures check arrival as
  well as separation and continuous connectivity.
- Added a shared energy-headroom preference capped at 180 s, followed by the
  existing added-flight-time objective. This favors a capable cohort when its
  alternative has little common surplus while limiting preference for unused
  charge once the shared margin is sufficient. It does not calibrate energy
  assumptions or establish full-mission completion.
- Added coordinated connected withdrawal when the formation cannot admit work
  or must return. Deployment, work, common retreat and reserve obligations are
  checked before commitment; actual motion and landing evidence determine the
  outcome.
- Completed already dispatched, observed relay handoffs during the bounded
  wait for announced tasks. A replacement must be operational, armed, assigned
  to the expected slot, connected and within the measured arrival bound.
  Maintenance does not select future tasks or grant predicted arrival credit.
- Rejected non-finite operational positions, remaining energy and sortie clocks
  before budget calculation. A non-finite distance cannot establish handoff
  arrival. Added negative coverage for invalid replacement state and unknown
  retiring-aircraft energy.
- Canonicalized owner and handoff maps by slot at formation commit. The matching
  result was a set: different `PYTHONHASHSEED` values produced the same owner
  mapping in different insertion orders, so later greedy handoff allocation
  chose different replacement UAVs and changed their subsequent motion. A
  subprocess regression now compares ordered owners, dispatch order/identities,
  ordered handoffs and all 30 positions after 20 s across hash seeds 0, 1 and 2.
  The [determinism regression](../tests/test_fleet30_determinism.py)
  is preserved separately from formal mission evidence.

That earlier candidate passed 35 focused tests and 449 selected core tests. The latter
excludes paused city/Unreal tests, SITL, and legacy dashboard tests requiring
unavailable Flask. Current outcomes belong in the validation record. The [issue register](UAVX_ISSUE_REGISTER.md) preserves all 11
original issues and their outstanding acceptance evidence. The
[formal-02 diagnosis](UAVX_FLEET30_FORMAL02_DIAGNOSIS.md) records the failures that
motivated the movement and idle-handoff changes.

City/Unreal work is paused because of storage constraints. Existing city code,
tests and evidence are preserved and excluded from the core PR. City flight
backend and physics status remain only partially qualified; this core repair
does not change that status.

## Earlier scenario/camera resilience update (historical profile)

- Added validated JSON scenarios, exact realized task inputs, explicit link/radio/
  aircraft fault schedules and a complete development matrix retaining failures.
- Added protected 44 m relay slots, measured biconnected GCS paths, nearest-task
  selection within priority, 22 m planning separation and configurable charging.
- Fixed a late link-failure regression caused by replacement relays spreading
  apart during a priority change. Ordinary movement now retains existing
  primary GCS reachability throughout the interval and evaluates backup paths at
  the proposed endpoint, allowing backup links to hand over to new neighbors;
  the emergency-return override and independently measured checks remain intact.
  The original failed run is preserved with its source snapshot.
- Fixed a landing approach that remained latched after avoidance displaced a
  descending aircraft from its pad; it now climbs and realigns before descent.
  This addresses the seed-100 battery-exhaustion regression without relaxing
  separation, connectivity, charge or landing checks.
- Replaced repeated trial-graph construction with an equivalent Tarjan traversal;
  independent NetworkX comparisons verify the reachability/protection score.
- Added finite shared TX/RX service, exact serialized byte accounting, queue
  limits, priority/aging and capacity consumption for retransmissions.
- Added Panda3D CPU scene-camera PNG capture, fragmentation, hash-checked GCS
  reassembly and received-only GUI imagery. A failed sensor carrier can be
  replaced through a separately timestamped recapture without resetting deadlines.
- Fixed double-charging one pad during a timestep, lost-camera-carrier deadlock,
  occupied survey-altitude targets and an event-argument naming collision.
- Strengthened evidence verification against premature completion, fabricated
  radio-service ledgers, truncated images, corrupted captures and invalid paths.
- Reduced unnecessary routing computation to the GCS destination for this profile;
  actual packet routing, service limits and constraints remain unchanged.
- That earlier example declared 60 UAVs/charging pads. It is a historical
  diagnostic profile, superseded by the 30-UAV fleet decision. Its results,
  together with earlier 72-UAV results, are not current qualification.

## Earlier implementation

- Added `Start-AeroMesh.cmd`, isolated GUI dependencies and a live desktop view
  wired directly to the updated UAV-X engine. The observer displays current UAV
  positions, actual topology edges and packet/landing state; pause and speed
  controls affect pacing only. New runs preserve previous evidence. Five GUI
  regressions and a full visible mission verify this path; see [GUI notes](UAVX_GUI.md).
- Restored simulation-only CLI; new UAV-X profile is the default, old benchmarks
  remain explicit `--profile legacy`. Hardware-oriented old CLI preserved separately.
- Added sample constraints, energy admission, per-sortie accounting, own-pad
  settled/disarmed landing checks, and continuous swept separation checks.
- Added seeded dynamic tasks, prioritized re-tasking, range-limited relay chain,
  role dispatch/arrival distinction, reserve returns and non-instant recharge.
- Added immutable detection/send/receive evidence, packet provenance, offered
  traffic including no-route losses, one-hop-per-tick forwarding, duplicate-ID
  handling, failed-node drops and edge-count hop limits.
- Removed hard-coded connectivity, 2.1 s recovery, and unconditional robustness
  success from the legacy reporter. Unknown results are null; terminal packet
  recovery requires actual delivered traffic.
- Removed the simulator's undefined `dt` vision call and synthetic detector
  construction timing presented as inference. Added explicit synthetic sensor scope.
- Added isolated dependencies, adversarial tests and current documentation.
- The old optional SITL test is explicitly opt-in; normal validation never connects
  to or arms an aircraft. AeroMesh's live GUI uses the kinematic engine.

## Errors uncovered / corrected during implementation

- Original CLI did not accept the documented scenario flags; simulation crashed
  with NameError; report/verification scripts claimed success without execution.
- Original telemetry was offered only when connected, excluding disconnections
  from PDR; packet attempts without routes did not enter its denominator.
- Original routing counted nodes instead of edges against max_hops and could
  process a packet through multiple queues in one tick based on iteration order.
- Old completion could latch across new tasks; an empty/failed fleet could appear
  landed; relay dispatch was incorrectly described as completed recovery.
- Sample limits differ from legacy 2 km maps, 10 m/s cruise and 15 m separation.
- New full-scale runs exposed crowding during return and reserve exhaustion;
  these failures remain in local evidence. Return-altitude routing, horizontal
  escape choices for vertical conflicts, stable relay identities, and conservative
  reserve checks were added without suppressing measured failures.
- Landing approach could oscillate between descent and climb after a lateral
  avoidance step. Latching the approach keeps subsequent motion directed home.
- Shortening the relay chain could promote a replacement into a deleted slot,
  causing a KeyError and an invalid slot-speed ratio. Removed-slot replacements
  now return; a separate speed veto rejects every over-limit motion candidate.
- Radio outages now dispatch a relay replacement without waiting for low battery.
  Fault-injection events no longer supply a duplicate event-type argument.
- Landing events now use the supporting observation timestamp. Source hashes are
  captured before execution so edits made later cannot relabel a completed run.
- Final review found that an empty manifest and fabricated constraint booleans
  could pass the verifier. Complete artifact inventories and consistency checks
  now prevent that false success; hashes are not a claim of tamper-proof evidence.
- A connected group of UAVs could lose its GCS path while retaining local
  neighbors, bypassing the old recovery trigger. Recovery now checks the actual
  GCS-connected component and logs whether dispatch was caused by lost GCS
  connectivity or low energy. Three regressions and all four full reruns passed
  their expected outcomes; the original strict fault failures remain.

## Known limits

Read [requirements](UAVX_REQUIREMENTS.md) for model assumptions. Complete
formation matching addresses feasibility within the declared budget model;
task selection and local movement remain heuristic. Deadlock, incomplete survey
and communication downtime can still occur. A passing small fixture does not
prove a 45-minute mission or hidden-scenario robustness. Qualification of the
current source completed the final 19-case matrix with 17 passes and two
strict fault failures; the selected core suite passed 486 tests. These declared
development results do not qualify hidden scenarios. Earlier nominal passes
belong to their saved source versions. The
full-run result table is in
[validation](UAVX_VALIDATION.md); no universal compliance claim is made.

AeroMesh's existing simulator adapter acknowledges commands without execution.
The current profile uses its own explicit kinematic state, not that acknowledgement
as landing proof. The cached Garuda Gazebo script's unconditional PASS and the
Garuda recorder's 600 s limit were identified but not modified in their repos.
No root LICENSE exists in the original AeroMesh tree despite historical MIT text;
repository licensing should be confirmed by its owner before external reuse.
