# IIT Bombay UAV-X: implementation and submission plan

Prepared 5 November 2026 from the user-supplied challenge PDF, report template
and sample-scenario slide. This is a work plan, not organizer approval or a
claim that the remaining capabilities have been implemented.

## First establish the competition route

The supplied schedule closed Stage 1 on 27 September. Stage 2 runs from
3 October to 2 December 2026 and is for qualified teams. Team registration and
qualification have not been established from the available materials. Confirm
the team's status and obtain the organizer's scenario files and log format.
Engineering work can continue while this is resolved; a working demo alone
does not establish eligibility to submit for Stage 2.

The supplied challenge PDF p. 8 specifies `pushpak_gc2026@aero.iitb.ac.in` for
submission and explicitly says no submission links are used. The Forms URL in
the original message conflicts with that instruction. Confirm the currently
applicable route through the organizer. No email has been sent by the agent.

## What we should build next

Mission completion (25%), communications (25%), autonomous roles (20%) and
fault recovery (15%) together account for 85% of the published score. Prioritize
reproducible performance in these areas while retaining the safety constraints.

| Priority | Concrete work | Acceptance evidence |
|---|---|---|
| 1 | Detect loss of a GCS path, including disconnected groups whose UAVs still have local neighbors | A partition with internal links triggers recovery; a healthy GCS path does not; no crash when the GCS graph node is absent |
| 2 | Define faults through scenario input: fault kind, target, injection time and duration; distinguish failed link, unavailable radio and total aircraft failure | Replay the same scenario/seed deterministically; preserve original strict checks and record each injected fault |
| 3 | Protect critical relay paths before failures where resources permit; use two independent relay routes and energy-aware replacement | Remove either relay in a protected test topology; surviving sources retain a path and actual report delivery remains within 10 s |
| 4 | Bound radio service and prioritize emergency reports; model bandwidth/queues using organizer assumptions when supplied | Congested traffic cannot all move simultaneously without a service budget; queue time and drops remain in latency/PDR; detections keep original timestamps |
| 5 | Test resource feasibility and safety margins across seeds, fleet sizes and failures during survey, handoff and return | Record every failure; no selected-seed-only summary; retain 20 m separation, 100 m range, 5 m/s speed, 100 m altitude, 1200 s sortie and 2700 s mission checks |
| 6 | Qualify the required simulator/framework and observation path | Real measured motion/sensing through one multi-UAV scene; configuration and logs reproduce the reported results; no pose-setting or acknowledgement presented as physics flight |
| 7 | Regenerate the report, fault demonstration and submission archive from final evidence | Independent clean setup reproduces results; video shows PoIs, links, relay changes, disturbance and recovery; artifact hashes match the submitted sources |

## Fix the fault interpretation before promising success

In the current outage case, the affected aircraft's only radio is disabled for
30 seconds. More relay UAVs can help its peers, but cannot communicate with that
disabled endpoint. Strict connectivity for that aircraft must fail while it is
airborne with no working radio. Restoring it requires radio recovery or an
explicitly modeled independent communication channel; neither should be invented
just to improve a score.

A permanently failed aircraft is immobile in the current model. It cannot land,
so the original all-UAV return and completion checks must remain failed. Add
separately labeled surviving-fleet measurements to explain mission continuity;
do not replace or reinterpret the original checks. Ask the organizer how failed
aircraft and transient communication loss are scored in advanced scenarios.

The current chain places relays roughly 85 m apart. Removing an intermediate
relay can leave a 170 m gap, beyond the 100 m range. A spare dispatched only after
the break cannot guarantee continuous service. Redundancy has to exist before
the failure and satisfy 3D range, separation, endurance and fleet constraints.
Test protected routes first; do not claim that all routes are protected merely
because an extra relay is present.

## Simulator decision

The challenge PDF p. 5 and report template section 9 permit an open-source
simulation framework such as PX4/ArduPilot SITL, Gazebo, AirSim, Webots, ns-3,
or equivalent. A specific 3D engine and physical drones are not required by
the supplied simulation-stage instructions. Acceptance of the current custom
Python kinematic model as an equivalent is not established.

Keep the current fast model for algorithm regression. Obtain the organizer's
required benchmark/framework assumptions before committing to a large migration.
If a robotics simulator is required, first qualify two UAVs in a single shared
scene, then measure performance at the required fleet size. The installed
ProjectAirSim environment is a candidate, but the existing Garuda single-UAV
client is not a proven swarm adapter. Webots is another explicitly listed option
for the Windows host; no Webots installation was found in the checked locations.
Select one backend on evidence, not several parallel integrations.

The report template explicitly permits centralized, decentralized or hybrid
coordination with justification. A distributed rewrite is not automatically a
submission requirement. The mission requires collecting imagery/situational
data at designated PoIs; it does not explicitly require a trained survivor-
recognition model. Qualify a traceable sensing/reporting path before adding
machine-learning perception features.

Do not make new voice, LLM control or cinematic-camera features part of the
competition-critical path. The required data collection and resilient reporting
must remain reproducible with deterministic control and clear sensor provenance.

## Development acceptance matrix

Start with the two known nominal seeds, 7 and 42. Add ten declared holdout seeds
and targeted failures during deployment, survey, replacement and return. Record
the full matrix, including low-resource cases and failures; compare each change
against the preserved baseline. These are development tests, not claimed hidden
jury scenarios. Use organizer scenarios unchanged when available.

Essential fault tests include: an intermediate relay lost from a protected pair
of routes; a source's sole radio disabled despite redundant routes; a partition
that retains local neighbors; and an immobile aircraft remaining an obstacle
while the survivors finish and return. A passing small graph test is not enough
to establish full-map performance.

Fleet count, home-pad/staging geometry, recharge duration/capacity, sensor model,
traffic rates and fault semantics must be explicit scenario inputs or documented
assumptions. Validate the required fleet rather than assuming 30 will be accepted.

## Proposed schedule if Stage 2 participation is confirmed

| Dates | Deliverable |
|---|---|
| 5-8 November | Organizer clarification; explicit fault/scenario definitions; partition-recovery fix |
| 9-15 November | Protected relay routes, bounded radio queues, resource/seed matrix |
| 16-22 November | Required simulator and organizer scenario/log integration; full regression |
| 23-27 November | Reproducible final benchmarks; fix remaining demonstrated failures; freeze candidate |
| 28 November-1 December | Final report, video link, installation check and submission review |
| By 2 December | Stage 2 submission through the organizer-confirmed route |

These are planning targets, not an estimate that all remaining work is already
complete or guaranteed to finish. Change scope based on organizer requirements
and measured progress. Retain an earlier reproducible release throughout.

## Team and delivery work

- Team/organizer: confirm participation status, scenario/log files, allowable fleet
  and charging resources, fault scoring and submission route; supply final team
  identification/contributions before the report is finalized.
- Engineering: implement and measure the resilience improvements, qualify the
  required simulator, publish accurate installation instructions and evidence.
- Repository owner: resolve the missing root license and legacy submodule
  mapping. Neither should be silently invented by an agent.
- GitHub access: the branch is pushed and a PR description is prepared, but the
  connected credential lacks PR-creation access. Enable the needed access or
  create the PR through an authorized GitHub session; this does not block local
  simulation work.

The original verified delivery remains in `UAVX-Submission-2026-11-05`. Keep it
as the baseline, and issue a separately named package for a newly validated
release so its report, code and hashes remain aligned.
