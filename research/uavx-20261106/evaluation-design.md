# Evaluation plan that can survive review

Research proposal. This is not a declaration that the following suite has already passed. It complements the existing evaluated 19-case **kinematic** matrix (17 passes, two retained strict failures; all evidence consistent) and the separately verified **compact Bullet** run.

## Claims, baselines and independent measurements

For each claimed improvement, write one falsifiable sentence before implementing it. Example: “Under the same 30-UAV fleet, scenario seeds, byte rates and image sizes, selective chunk retry increases the fraction of required images delivered within 10 seconds without increasing safety violations.” A higher packet delivery ratio alone does not establish that claim.

Keep a minimal baseline for each layer:

| Layer | Baseline | Candidate | Measurements that decide |
|---|---|---|---|
| Target service | Feasible nearest task / greedy assignment | Existing budgeted matching and bounded headroom preference | Complete required tasks, elapsed time, sorties, returns, reserve, missed deadlines |
| Unknown-target search, if required | Partitioned lawnmower sweep with identical sensing | Frontier or informative planning | Area observed, detection delay, false/missed detections, revisits, bytes and energy |
| Relay placement | Minimum feasible unprotected chain | Protected spacing / battery-aware handoff | Root connectivity, disjoint paths, service downtime, resource use, mission completion |
| Routing | Shortest-hop or existing expected-attempt cost | Link-age / throughput-aware route | Image deadline delivery, per-class age, control overhead, queue/drop counts |
| Image transport | Current full-image 2-second retries | Selective chunks with modeled feedback | Useful bytes, total transmitted bytes, complete images, first-detection deadline |
| Physical tracking | Existing declared velocity/attitude cascade | Identified drag / bounded disturbance compensation | Tracking error, actual speed/tilt, actuator saturation, reserve cost |
| Safety | Current sequential velocity projections | Acceleration-aware or sampled-data robust filter | Minimum measured separation, intersample bound, infeasibility, intervention cost |
| Viewer | Existing observational overview | Role/route/fault/receipt panels with provenance | Observer task accuracy and interpretation time; unchanged mission artifacts |

Do not compare a candidate with privileged world state to a baseline restricted to delayed messages. Do not give one method better cameras, extra drones or different radio budgets. Separate representation improvements from policy improvements; R11 provides a useful example of why this matters.

## Three types of scenario set

1. **Deterministic mechanism fixtures:** one changed variable with an analytic or directly checkable expected outcome. Examples: free fall, motor lag, no route, duplicate image chunk, time-expired command. These diagnose implementation/model errors.
2. **Adversarial design cases:** far-corner target, late emergence, two adjacent relays lost, return corridor conflict, queue saturation, failed charging vehicle. These deliberately probe boundaries. Report every case; their success fraction is not a population probability.
3. **Randomized performance sample:** a frozen generator with a stated distribution over positions, emergence times, faults, loss and wind. Pick seeds before evaluation, retain unsuccessful runs and compare methods on identical seeds. Only this defined sampling process supports an appropriately qualified statistical success estimate.

The existing 19-case matrix contains heterogeneous handpicked cases and regression seeds. Its 17/19 result is a coverage statement, not an IID estimate of success in all disasters. The compact physics fixture is one integrated example and must not replace full-map qualification.

## Confidence without exaggerated certainty

For n independent representative runs that all succeed, an exact one-sided 95% lower confidence bound for the success probability is `0.05**(1/n)`. Thus 30/30 would support a lower bound of roughly 90.5%, 60/60 roughly 95.1%, and 100/100 roughly 97.0%, subject to the sampling assumptions. To obtain a 99% lower bound from zero failures would require at least 299 such runs. This calculation is an illustration based on binomial intervals [R04], **not a claim that these experiments have been performed**.

Use a Wilson or exact binomial interval for ordinary success counts; state whether it is one- or two-sided and the confidence level. Correlated seeds, adaptive scenario selection, repeated tuning on the evaluation set and heterogeneous distributions invalidate a casual binomial interpretation.

For paired algorithm comparisons, publish per-seed differences, median, tails and failures. A single mean can hide an occasional catastrophic deadline or safety failure. Failed or unfinished missions do not receive an invented favorable completion time; report censored/not-completed outcomes separately. A paired bootstrap or an exact paired success test may be useful when its assumptions and sample size fit the experiment. Do not choose the statistical method after seeing which one gives a favorable result.

## Mandatory metric definitions

- **Mission completion:** all required PoIs surveyed and fully reported, and strict required return/landing conditions satisfied by the deadline.
- **PoI report delay:** complete validated receipt minus first detection time. A later recapture must not reset the original deadline.
- **Image service:** required images complete / required images; also show captures, recaptures, missing chunks and pending images at cutoff.
- **Packet PDR:** name the numerator and denominator. Current router counts unique delivered logical packet IDs / all unique offered IDs, including pending. It is not per-hop success.
- **Latency:** only delivered packets have a finite measured receipt delay. State sample count and drops/pending alongside median/p95/p99/max.
- **Age:** current time minus the acquisition/generation timestamp of the freshest valid sample actually received, including outage intervals. An old sample arriving late must not reset age to zero or replace a newer sample. Current packet delay is not a substitute for this proposed metric.
- **Recovery:** fault onset, detection, decision, command receipt, first restored useful service and all-required-source restored service are different timestamps.
- **Connectivity:** explain geometric path availability versus actual useful data delivery and the definition of active/failed nodes. Never remove a failed requirement from the denominator silently.
- **Safety:** report observation frequency, full-rate extrema where available, contact envelope and any validated between-sample bounds. Body contact is not the 20 m mission separation threshold.
- **Energy/resources:** declared 1200-second endurance, sorties, charging-pad occupancy/queue, reserves and grounded spares. Watts/Wh predictions require separate calibrated parameters.
- **Computation:** simulated time, wall time, real-time factor, CPU/RAM, planner tail latency and rendering rate are distinct. Requested 12× playback is not achieved throughput.

## Failure taxonomy and what counts as recovery

| Disturbance | Possible response within current resources | What cannot be claimed |
|---|---|---|
| One link disappears, alternate path exists | Reroute and preserve service | Zero delivery delay solely from graph redundancy |
| Relay temporarily loses radio | Detect, preserve remaining network, use local cached behavior if implemented | Continuous connectivity for the isolated UAV |
| Whole aircraft loses power permanently | Keep failure physical; avoid falling/grounded body; replace mission role; recover survivors | Controlled landing of the destroyed UAV or a strict all-30 return pass |
| Packet burst loss / congestion | Bounded retry, pacing, prioritization | Delivery guarantee over an unavailable physical path |
| Position estimate becomes stale | Reduce authority and execute a declared locally feasible policy | Continued globally informed avoidance with no observation path |
| Late impossible task | Reject/defer with a specific resource/deadline witness; preserve safe return | Universal infeasibility from one planner failure |
| Too few charged aircraft | Wait if a feasible forecast exists; service another admissible task; return | Invented spares, free charging or instantaneous replenishment |

An optimistic lower bound can prove some cases infeasible under named assumptions. Failure of a heuristic planner cannot prove that no valid schedule exists. A feasible task rejected by the planner is a planner defect; a physically impossible strict condition must remain visibly impossible.

## Reproducibility package

Use the ACM artifact qualities as a practical checklist [R12], not a badge claim:

1. One current launch/verify guide that names the intended backend and exact scenario.
2. Source commit plus runtime hashes; preserve dirty-source snapshots when applicable.
3. Dependency versions, Python/platform details, seeds, parameters and coordinate frames.
4. Input scenario, raw observation/packet/event evidence, independent verification result and scripts to derive plots.
5. A short known-good smoke run, a full mission run and named expected-failure cases.
6. Expected artifact names and acceptance criteria; clean-environment reproduction by another person is a separate milestone.
7. Asset/code licenses and provenance. Public GitHub access does not imply a license to redistribute source or datasets. Keep third-party page/PDF research caches local and excluded from the notes PR.

Do not publish historical performance tables as current results when their source run and configuration cannot be found. Preserve them with a historical/unqualified status and link to the current evidence.

## Research-session experiment queue

The exact status and resulting paths are recorded in `WORK_LOG.md`; only completed, source-stable results should appear in the final synthesis.

- Full nominal Bullet mission on unchanged source, maximum 2700 simulated seconds.
- Isolated timestep/convergence and controller disturbance probes, with frozen source hash and explicit analytic references.
- Isolated router clock/load/loss/retry sweep, clearly separated from whole-mission qualification.
- Actual received-image integrity/resolution audit and research-only visual wireframe.
- Additional integrated physics cases selected after the nominal result and controller probes reveal the most informative remaining boundary.

This is a staged validation program inspired by NIST capability-specific tests [R03] and NASA model-credibility principles [R07]. It is not NIST, NASA, aviation or airframe certification.
