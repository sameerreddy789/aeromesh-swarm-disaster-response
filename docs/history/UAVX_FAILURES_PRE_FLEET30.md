# Historical record — superseded by the fixed30-aircraft work

This preserves earlier claims and failures; it is not current acceptance evidence.

# Current failures and open gaps — 5 November 2026

The last complete matrix, before the latest movement and landing fixes, passed
**15 of 19 declared development cases**. All 19 produced internally consistent evidence with no verifier errors
and unchanged recorded sources. Four cases still fail constraints; the full
matrix does not pass. The current candidate has 345 passing targeted tests; its
full 19-case rerun is still in progress. Exact commands, all case results and local evidence paths
are in [validation](../UAVX_VALIDATION.md).

The release adds bounded radio service, rendered scene-camera imagery delivered
through the mesh, explicit fault scenarios and denser relay placement. These
replace several limitations of the earlier footprint-only proof of concept.
See [implementation and assumptions](../UAVX_RESILIENCE.md) for their actual scope.

## Observed failures in the current matrix

| Case | Failed checks | Measured consequence |
|---|---|---|
| `radio-outage-seed42` | `continuous_connectivity` | Disabling UAV 36's sole radio for 30 s produces 30.0 s downtime and 98.5041% availability. All 10 images arrive within 10 s of detection and the mission finishes at 2007.0 s. Affected surviving-source service is confirmed after 6.5 s; the disabled endpoint remains disconnected until its radio returns. |
| `permanent-failure-seed42` | `all_uavs_landed`, `mission_deadline` | UAV 36 remains permanently immobile. The 59 operational survivors maintain 100% measured connectivity, deliver all 10 images and finish returning by 1992.5 s. Strict completion remains null at the 2700 s cutoff because the failed UAV never lands. |
| `resource30-seed42` | `all_pois_surveyed_and_reported`, `mission_deadline`, `continuous_connectivity`, `reporting_deadline`, `imagery_received` | The protected planner with 30 UAVs and 30 charging pads completes only 7/10 PoIs/images by 2700 s. Downtime is 377.138294 s and availability is 85.5392%. All operational UAVs land, but missing surveys/reports prevent completion. |
| `late-link-seed42` | `continuous_connectivity`, `reporting_deadline` | The 30 s link outage at t=1600 s produces 34.399303 s total downtime and 98.2609% availability. PoI 9 is detected at 1615.0 s and fully received at 1638.5 s: a 23.5 s delay. All 10 images ultimately arrive and the fleet lands by 1979.5 s; that does not erase the missed deadline. |

The link-outage cases at t=180 s and t=900 s pass all 14 implemented checks. The
late-link failure shows that the moving and replacing relay network is not
continuously protected in every phase. The next engineering target is preserving
an alternate path during that transition without exceeding range, separation,
energy or fleet constraints. Settled-topology redundancy alone is insufficient.

For the 30-UAV case, already received images have a maximum delay of 6.0 s and
no detected image is recorded as late. The reporting check still fails because
three PoIs lack detection/report evidence. Missing reports are not successful
low-latency service.

Across all 19 cases, measured battery exhaustions, collision pairs, separation
violations and geofence violations are zero. Minimum separation is 22.000024 m;
the longest sortie is 1123.0 s. These safety results apply only to the tested
kinematic model and do not turn the four failed missions into passes.

## Remaining model and qualification limits

1. **Resource feasibility is unresolved.** Passing examples use 60 UAVs,
   60 charging pads, 25 m pad spacing and a declared staging area. The explicit
   30-UAV case fails. Organizer acceptance of these resources and geometry is
   not established; team size is not a UAV allowance.
2. **Fault robustness is incomplete.** The matrix covers 12 nominal seeds,
   three link-outage timings, one radio outage, one permanent failure, one
   resource reduction and one higher-loss case. It does not establish arbitrary
   multi-fault, terrain or unseen-scenario robustness. The late-link failure is
   an observed planning limitation, not merely an untested possibility.
3. **Motion remains kinematic.** Vehicles follow linear point-state segments;
   thrust, acceleration, wind, finite airframe volume and tracking error are not
   modeled. The larger separation margin is not a physical-controller guarantee.
   No PX4/ROS/ProjectAirSim flight backend or hardware result is claimed.
4. **Radio service is bounded but links remain idealized.** Per-node shared
   TX/RX bandwidth, queue limits, retries and serialized image bytes are accounted
   for. Obstacle RF propagation and interference between disjoint links are not
   calibrated. Grounded/charging radios remain usable, with external charging
   power assumed. These are explicit simulation assumptions.
5. **Sensing is rendered simulation imagery.** Panda3D TinyDisplay supplies
   images from generated disaster scenery at the supplied kinematic camera pose.
   Source, timestamps, intrinsics, bytes and GCS reassembly are traced. A physical
   camera, learned survivor recognition and perception accuracy are unqualified.
6. **Energy and charging remain assumptions.** Endurance is a conservative time
   budget, and full recharge takes 720 s in the examples. Finite charging slots
   are enforced; this does not establish an airframe battery/charging design.
7. **Coordination is centralized.** The planner uses global simulation state.
   A distributed protocol with stale/lost planning state is not implemented. The
   supplied report template permits centralized coordination with justification;
   decentralization is not being invented as a mandatory rule.
8. **Fault meanings must stay explicit.** Relays cannot communicate with a
   completely disabled sole radio. A frozen aircraft cannot land. Separate
   survivor metrics never replace strict all-UAV checks. Organizer scoring of
   these events remains unconfirmed.
9. **Evidence verification has limits.** It checks hashes, image reconstruction,
   radio-service bounds and consistency with survey/receipt/landing timestamps.
   It is not independent dynamics reconstruction or protection against
   coordinated edits to logs and hashes.
10. **Legacy metadata remains unresolved.** The root tracks
    `aeromesh-swarm-disaster-response` as a Git submodule pointer (`7cf5a89`)
    without a `.gitmodules` mapping. Its nested contents are not needed by this
    profile and are not claimed bundled by the source snapshot.

## Submission and ownership dependencies

- Team name/registration, members, institution, eligibility and Stage 2
  qualification have not been supplied.
- Organizer scenarios, communication assumptions and the standardized log schema
  were not supplied. Schema 2 is a project evidence format.
- Acceptance of this custom kinematic/Panda3D framework and its resources as an
  equivalent simulation setup has not been established.
- Final report/video review, team contributions and an evaluator-accessible video
  link remain pending. Generated documents remain drafts.
- The supplied PDF specifies email submission while the original message contains
  a Forms URL. The currently applicable route requires confirmation.
- Historical MIT references lack a root LICENSE in the original repository; its
  owner must determine the applicable license.
- The branch has been pushed, but the connected GitHub credential has not been
  able to create the PR. Prepared PR text does not establish a created PR.
- No email, organizer message or competition submission has been sent.

## Historical results and corrected issues

Earlier 30-UAV results, including the 172-test live-GUI release, remain in
[gui-benchmark-summary.json](../verification/uavx/gui-benchmark-summary.json) and the
previous delivery update. The partition-recovery baseline remains in
[benchmark-summary.json](../verification/uavx/benchmark-summary.json). Their
footprint-only sensing, unbounded parallel radio service and older source hashes
must not be presented as the current release's validation.

Current fixes include bounded service/queues, immutable serialized packet sizes,
complete image receipt instead of first-fragment success, strict scenario input,
explicit missed-fault checks, failed-carrier recapture with the original deadline
retained, and charging-slot accounting within each timestep. Earlier CLI/crash,
packet-provenance, landing and false-success corrections remain in the
[changelog](../UAVX_CHANGELOG.md). They do not remove the four current failures.
