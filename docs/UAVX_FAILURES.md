# Current failures and open gaps — 5 November 2026

The two final nominal runs pass all implemented sample checks. The following
failures and limitations remain; successful PoI reporting does not erase them.
Exact measurements and reproduction commands are in [validation](UAVX_VALIDATION.md).

## Observed failures in the final fault runs

| Case | Failed checks | Evidence / consequence |
|---|---|---|
| 30 s radio outage, seed 42 | continuous_connectivity | 30.0 s downtime; 98.4248% availability. Fresh delivery from the surviving source group recovers in 8.0 s, which does not restore every UAV's connectivity. |
| Permanent UAV failure, seed 42 | continuous_connectivity, all_uavs_landed, mission_deadline | 3.151142 s downtime; the failed UAV remains stranded. All 10 PoIs are surveyed/reported, but the mission has no valid completion at the 2700 s cutoff. |

All four final runs have zero measured battery exhaustions, collision pairs,
separation/geofence violations and reporting-deadline misses. Those statements
apply only to the tested abstract model. Earlier development failures are
retained locally and described in the [changelog](UAVX_CHANGELOG.md).

## Unresolved implementation and qualification gaps

1. **Limited scenario coverage:** two nominal seeds and two single-fault cases
   are insufficient to establish hidden-scenario robustness. The greedy planner
   is not complete; resource shortages or other geometries can stall it.
2. **Fleet and staging assumptions:** the benchmark uses 30 aircraft, 25 m home
   pad spacing, a declared staging geofence and assumed recharge capacity. An
   18-aircraft development run did not complete coverage. Organizer acceptance
   of these resources and geometry is not established.
3. **No physical flight dynamics:** point vehicles follow linear kinematic
   segments. Acceleration, wind, terrain, finite vehicle volume and tracking
   error are absent. Minimum separation is only about 20.0005 m, leaving no
   demonstrated physical-control margin beyond the required 20 m.
4. **Simplified radio:** independent loss and a hard distance cutoff do not
   capture interference propagation, obstacles, bandwidth contention or a
   practical radio stack. Parallel packet service is unbounded.
5. **Centralized state access:** planner and routing use global simulation
   information. A distributed protocol operating with stale/lost state has not
   been implemented or qualified in this profile.
6. **Synthetic perception:** task coordinates are announced to the planner and
   geometric proximity produces detections. Survivor-recognition accuracy,
   real camera processing and sensor uncertainty are unmeasured.
7. **Uncalibrated energy:** constant endurance and a 720 s full-recharge
   assumption do not establish an airframe battery/charging design.
8. **Permanent-failure recovery:** a failed aircraft freezes and remains a
   collision obstacle. The simulation cannot recover or land that vehicle.
9. **Backend integration:** the legacy AeroMesh adapter acknowledges commands
   without actual execution. Garuda's current single-client scene reload is
   unsuitable for independent clients controlling a swarm. No multi-UAV
   PX4/ROS/ProjectAirSim integration or hardware-in-loop result is claimed.
10. **Evidence limits:** hashes and cross-checks detect missing/inconsistent
    artifacts; they are not independent dynamics verification or protection
    against coordinated edits to both logs and hashes. Trajectory export is
    sampled every 2 s; runtime safety uses the finer 0.5 s movement intervals.
11. **Legacy checkout issue:** the root tracks `aeromesh-swarm-disaster-response`
    as a Git submodule pointer (`7cf5a89`) without a `.gitmodules` mapping. The
    required nested source cannot be resolved from that metadata alone. The
    UAV-X profile does not use it; the draft package records its pointer without
    claiming to bundle the nested repository's contents.

## Submission gaps

- Team name/registration ID, members, institution, eligibility and qualification
  status have not been supplied; the report leaves them explicitly incomplete.
- Organizer-issued Stage 2 scenarios and a required log schema were not supplied.
  The current JSON format is a project format, not claimed organizer compliance.
- The report and trajectory demo are drafts. Final review and an evaluator-
  accessible video link remain outstanding. No external submission was sent.
- The supplied PDF specifies email submission while the message contains a Forms
  URL. The submission route requires resolution through official instructions.
- Historical MIT references are unsupported by a root LICENSE in the original
  AeroMesh tree; its owner needs to resolve the licensing terms.
- PR delivery is blocked by the connected GitHub credential: the branch was
  pushed to `sameerreddy789:codex/uavx-submission`, but GitHub rejected
  `CreatePullRequest` for missing permissions (the REST route also returned 404).
  No PR was created. The prepared PR description is included in the local draft
  package; creation needs an account/credential with the required PR access.

## Errors already corrected

The implementation fixes the undefined `dt` crash and CLI mismatch, fabricated
legacy metric/verification success, omitted no-route traffic in PDR, multiple-hop
forwarding within one tick, incorrect hop limits, premature recovery counts,
latched mission completion, false empty/failed-fleet landing, orphaned/returning
relay replacements, deleted-slot promotion/KeyError, excessive-speed candidates,
landing oscillation, premature landing timestamps and fault-event argument crash.
The verifier now rejects incomplete artifact manifests and contradictory claims.
These corrected errors are not listed as outstanding failures.
