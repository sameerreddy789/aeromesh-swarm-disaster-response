# Swarm, network, and resilience research

Research began 2026-11-06 at 02:28:39 UTC, with final synthesis after the requested research window. This is a research document, not an implementation or qualification report. It compares primary literature and official protocol documentation with the frozen AeroMesh-UAVX implementation. The companion [source ledger](swarm-network-sources.json) distinguishes full-text reading, selected sections, institutional abstracts, and project documentation. The observed clock jumped from around 03:00 to 05:28 UTC; elapsed wall timestamps must not be described as three hours of continuous computation. Authorized isolated radio scripts were added under this research directory; production code and original evidence were preserved.

## Findings that already affect decisions

1. The present planner is a centralized, budget-constrained matching system. Replacing its name with “CBBA,” “decentralized,” or “AODV” would overstate the implementation. The valuable next experiment is to restrict each controller to timestamped local observations and messages, then measure what fails.
2. A connected graph is only one condition for useful communication. Root biconnectivity helps with a single intermediate-node failure, but says nothing by itself about congestion, stale routes, complete image delivery, or a target whose sole radio has completely failed.
3. The fixed fleet of 30 can complete some declared scenarios. A protected far-corner formation consumes 27 vehicles, leaving little temporal capacity for recharge and replacement. Continuous far-corner station keeping for the full 45 minutes is impossible under the current energy assumptions; a short far-corner visit is not.
4. Measured image completion is already stronger than packet-offer accounting. Preserve the complete reassembly, capture identity, length and SHA-256 checks. Add freshness and deadline distributions with explicitly different denominators.
5. Current velocity projections have useful safety intent but no formal continuous-time certificate. ORCA and CBF papers do not automatically certify sampled commands tracked by force-driven aircraft.

## Reviewed implementation and evidence

Source inspection on 2026-11-06 found the following hashes unchanged from the supplied final core/physics artifacts:

| File | SHA-256 |
|---|---|
| `comm/mesh_router.py` | `31ff5888dc11779edab0c1185b0eac2abe4e9d5510432d840567074340983b87` |
| `simulation/resilient_uavx.py` | `c7edda669645edcb82d1e6d5df7445a79d0ec40e5a9c5475ff3915afc4baf7a5` |
| `simulation/uavx.py` | `faf695035c6ab20c3b9035c8abdbba50d251b4b6951efcadfa696f877b9b63ae` |
| `simulation/uavx_scenario.py` | `1f3af1d445eb8b9948f24213765ab4f9ddc3a424539830d4a67554aaf49ec7fd` |
| `simulation/physics_uavx.py` | `0b8085967e602f8a6ca5f37a7c35bcf9ebb22d1ccdc96f07fc6186bf7f675343` |

The nominal seed-42 record in [the final core summary](baseline-summaries/kinematic-core.json) reports all ten PoIs, all 30 aircraft landed, completion at 1970 s, 100% measured connectivity availability, maximum sortie 939.5 s, and no reporting deadline misses. The same artifact explicitly says the full development matrix did not pass and is not submission-ready. These are kinematic results from prior work. The parent separately completed an unchanged-source full nominal **physics** run: ten PoIs and all 30 landed at 2514 s, but only **14/15 checks** passed because continuous connectivity failed for **19.8 s**. See [summary](integrated-nominal-summary.json) and [outage diagnosis](integrated-nominal-connectivity-diagnosis.md). The kinematic pass does not override that physical-motion failure.

The [physics summary](baseline-summaries/compact-physics.json) reports a separate compact fixture completed at 623.5 s with 30 vehicles total, ten launched, peak eight active, and 20 grounded spares. That is not evidence that 27 simultaneous far-corner aircraft have been qualified in Bullet. Neither result qualifies physical aircraft or organizer scenarios.

## Current algorithm compared with alternatives

| Concern | Current implementation | Source-supported alternative | Transfer limit |
|---|---|---|---|
| Assignment | `_budgeted_assignment` enumerates feasible common deployment bounds and uses NetworkX maximum-weight matching, followed by spatial uncrossing | CBBA greedy bundles and neighbor conflict resolution; CBBA-PR selective suffix resets [SW01–SW03] | Global return obligations and relay coupling may violate diminishing marginal gains; communication rounds must consume real service |
| Dynamic PoIs | Highest priority first; nearest announced equal-priority alternative; current equal-priority task retained | Partial replanning trades convergence time and solution quality [SW03] | No future task coordinates may enter the online policy; planning cannot manufacture late-arrival slack |
| Search coverage | Synthetic proximity trigger and rendered camera observation around known/announced PoIs | Lloyd/Voronoi density-weighted spatial coverage [SW05] | Sensor placement is different from searching every cell or verifying survivor detection |
| Relay topology | 44 m slots; reachability and GCS-rooted biconnected membership in movement preferences | Spectral connectivity plus vulnerability-aware topology control [SW06] | Positive lambda_2 is not two vertex-disjoint paths, bandwidth, or arbitrary-fault resilience |
| Routing | Global graph rebuilt from all positions; global weighted shortest paths and rerouting | OLSRv2, AODV, or Linux batman-adv as actual message-driven baselines [SW08–SW12] | No RREQ/RREP, HELLO/TC, OGM or per-node stale topology state is currently simulated |
| Radio service | Shared per-node half-duplex TX/RX; finite queues; serialized JSON; traffic priority and aging | Measured link cost, pacing, bounded retries, MTU-aware payloads [SW07, SW14–SW15] | This is not calibrated interference, CSMA/CA, ACK airtime or an RF propagation model |
| Collision avoidance | Pairwise sequential velocity projections at 20 Hz, 22 m planning separation, bounded reference speed | ORCA, bounded-acceleration CBF/QP, later robust/sampled-data formulations [SW16–SW18] | Model, observations, input feasibility, delay, tracking error and failed-agent responsibility matter |
| Endurance | 1200 s armed-time capacity; proportional recharge with 720 s empty-to-full assumption | Joint schedule of service, return, recharge and constrained availability [SW19] | Energy estimates and charger service require calibration; fixed fleet remains 30 |

### Task allocation and coverage

The [MIT CBBA project](https://acl.mit.edu/projects/consensus-based-bundle-algorithm) describes local bundle construction and consensus, asynchronous conflict handling, coupled-constraint variants, and relay tasks. The [original authors’ institutional abstract](https://pure.kaist.ac.kr/en/publications/consensus-based-decentralized-auctions-for-robust-task-allocation/) states convergence and worst-case performance subject to score assumptions. The original 2009 full paper has not yet been read here.

The [CBBA-PR full text](https://arxiv.org/html/1806.04836), section 2.3, makes the assumptions more useful for this project: diminishing marginal gains are needed, and convergence costs communication rounds proportional to tasks and network diameter. Its 50% bound refers to the applicable submodular sequential-greedy formulation, not every constrained objective. Dropping a lost bundle task requires dropping the later suffix because those bids were conditioned on that task. A relay cohort with a shared retreat deadline is a coupled scheduling problem; the [CMU iTax abstract](https://publications.ri.cmu.edu/a-comprehensive-taxonomy-for-multi-robot-task-allocation) provides the relevant classification.

A fair experiment would retain the current matching planner as a centralized baseline, compare simple feasible greedy assignment and a genuinely message-driven CBBA/CBBA-PR variant, and evaluate identical announcements, radio service, energy budgets and failures. Scores should include complete mission feasibility and deadline success, not only total travel or auction value. A decentralized claim requires each agent’s local inputs and state to be independently inspectable.

[Coverage-control research](https://arxiv.org/html/math/0212212v1) optimizes a continuous spatial sensing objective and converges to centroidal Voronoi critical configurations under its assumptions. It is useful for exploring an unknown search field, but is not a direct replacement for service of discrete announced PoIs. A future search benchmark needs coverage footprints, revisit/unobserved-area metrics, and true/false/missed detection labels. Rendered imagery by itself does not establish recognition.

### Connectivity and relay placement

For an undirected graph with suitable nonnegative edge weights, positive second Laplacian eigenvalue indicates connectedness. The [quadrotor field study](https://arxiv.org/pdf/1909.10378) explicitly warns that connectivity-only control can retain vulnerable high-centrality nodes and adds a separate robustness term. It also describes the communication needed for distributed spectral estimates and one-/two-hop neighborhood information. The graph is not available for free.

Root biconnectivity is already a meaningful stronger measure for this mission: a UAV in a biconnected component containing GCS has two internally node-disjoint paths to GCS. This protects the surviving endpoint from deletion of one intermediate vertex in that graph. It does not protect against loss of the endpoint itself, GCS, a common physical interference zone, or enough adjacent failures. Geometric redundancy should be paired with measured delivery after failure and explicit fault scope.

A centralized spectral score may be a diagnostic overlay. It should not replace observed root reachability, articulation/vertex-cut checks, or packet results. A distributed spectral controller is a separate implementation requiring bounded estimate age, communication cost and adequate low-level tracking.

## Fixed-fleet feasibility: conditional bounds, not blanket failure claims

These calculations are derived from code and stated assumptions; they are not new flight measurements. The fixed limits remain 30 UAVs including spares, 100 m radio range, 20 m minimum separation, 5 m/s, 100 m altitude, 2700 s mission, 1200 s flight budget and 10 s reporting deadline.

### Far-corner geometry

`ScenarioTask` permits ground PoIs at x=75..1075 and y=-500..500. The current slot builder places aircraft at z=40 m. For the far corner:

- Horizontal GCS-to-PoI distance is sqrt(1075² + 500²) = **1185.591 m**.
- GCS-to-survey-point distance is sqrt(1075² + 500² + 40²) = **1186.265 m**.
- Any ideal 100 m-link path requires at least ceil(1186.265/100) = **12 edges**. With GCS at one end and the sensing aircraft at the other, this means at least 12 UAVs including the sensing endpoint, under that ideal geometry. It is a connectivity lower bound, not a protected formation plan.
- Current `_build_slots` uses ceil(1186.265/44) = **27 UAV slots**. All slots are at z=40 while XY is interpolated, so adjacent interior spacing is **43.911 m**, second-neighbor spacing **87.822 m**, and GCS-to-second-slot distance **96.502 m**. The three-dimensional GCS link must not be approximated as merely 88 m.
- Deleting two adjacent interior aircraft can leave **131.732 m** between surviving neighbors, exceeding 100 m. The single-node-redundant chain is not two-adjacent-node-loss tolerant.

A formation can reconfigure after loss if surviving vehicles have travel, battery, radio and deadline slack. Reconfiguration does not erase any interval of lost connectivity. Increasing fleet count or adding radios is outside the user’s selected resources and is not proposed as a repair.

### Energy and time capacity

Under ideal repeated cycles with 1200 s flight and 720 s recharge, the long-run airborne duty factor is **0.625**. Thirty vehicles can therefore supply at most **18.75 average airborne vehicles** in persistent steady operation, before travel overhead, charger limits and reserves.

The finite 45-minute mission begins fully charged, so using 18.75 as its universal cap would be wrong. An optimistic finite-horizon bound is possible. Let A be aggregate airborne seconds, N=30, T=2700, D=1200, and recharge rate r=D/720=5/3 units of flight-time budget per grounded second. Even with unlimited chargers and useful recharge during every grounded second:

A <= N D + r (N T - A), hence **A <= 64,125 aircraft-seconds**, or **23.75 average airborne aircraft**.

This is an upper bound, not an achievable schedule. Flight transitions, initial saturated batteries, individual sortie limits, charging capacity, useful relay positions, landing and final reserve can only reduce it. Continuously keeping 27 aircraft airborne for all 2700 s would require 72,900 aircraft-seconds and therefore violates even this optimistic bound. A temporary 27-aircraft deployment or a shorter/sparser mission does not violate it. The successful 1970 s nominal record is compatible with these statements.

### Late task announcements

A necessary lower-bound test uses the actual eligible aircraft and the allowed sensing volume, not a fictitious departure from GCS for every task. For a vehicle at p and an allowed sensing region F, outbound travel is at least dist(p,F)/5. If the task requires a remaining dwell w and return home after service, a necessary individual time budget is:

dist(p,F)/5 + w + inf(q in F) dist(q,home)/5 + required landing time.

Using independent minima is optimistic and thus useful for proving impossibility when even that lower bound exceeds remaining time. Radio delivery, route restoration, relay deployment, collision detours and charger waits add constraints. The current reporting clock starts at first detection; a late announcement is a mission-completion issue before it becomes a detection-to-report issue.

A task released after t=2670 with a required fresh 30 s dwell cannot finish by 2700 even with an aircraft already at the sensing location and zero return time. The scenario schema permits spawn times through 2700, so syntactically valid scenarios need not be feasible. Record “scenario infeasible under the declared constraints” separately from planner failure. Do not silently relax a deadline, omit an aircraft, backdate detection, or count survivor-only completion as a strict pass.

## Routing, service capacity and freshness

### What ETX does and does not mean

The [original ETX paper](https://pdos.csail.mit.edu/papers/grid:mobicom03/paper.pdf), sections 3.1–3.2, defines per-link ETX as **1/(d_f d_r)**, incorporating data delivery and reverse acknowledgment delivery, then sums ETX along a path. Its implementation uses one-second probes and a ten-second observation window. It explicitly does not solve mobility or congestion, and acknowledges measurement-responsiveness tradeoffs. Its 29-node stationary 802.11b results are evidence for that setup.

AeroMesh’s range fixture returns a common p and **1/p**, without independently measured ACK loss, MAC retransmission cost or asymmetric links. This is internally meaningful as the expected attempts of its abstract Bernoulli service model; calling it measured bidirectional ETX would be inaccurate. At 5 m/s, ten seconds of old geometry can correspond to 50 m of individual motion or up to 100 m of relative motion. That numerical comparison is a worst-case bound, not an argument for blindly selecting a faster beacon interval.

### Protocol comparisons

- [OLSR RFC 3626](https://www.rfc-editor.org/rfc/rfc3626) is an Experimental proactive protocol with MPR flooding and base hop-count paths.
- [OLSRv2 RFC 7181](https://www.rfc-editor.org/rfc/rfc7181), sections 4.5–4.6, supports additive link metrics and separate flooding/routing MPRs. Periodic TC updates remain necessary.
- [AODV RFC 3561](https://www.rfc-editor.org/rfc/rfc3561) is Experimental and uses on-demand discovery and sequence-number freshness. Its illustrative defaults give a 2 s HELLO-loss threshold, 2.8 s NET_TRAVERSAL_TIME and 5.6 s PATH_DISCOVERY_TIME. These are parameter examples, not measured recovery promises.
- [BATMAN V documentation](https://www.open-mesh.org/projects/batman-adv/wiki/BATMAN_V) separates local ELP discovery from OGMv2 propagation and uses throughput estimates that depend on driver/interface support. Some features on that page are explicitly marked upcoming. [batman-adv](https://www.open-mesh.org/projects/batman-adv/wiki/Doc-overview) is an actual Linux Layer 2 implementation, not another name for Dijkstra.

No protocol is automatically the winner for 30 UAVs. Compare stable relay topology, churn, loss bursts, route repair delay and control overhead under equal offered traffic and link observations. The central graph oracle should remain a labeled upper-information baseline.

### Deadline and load feasibility

Each forwarded serialized byte consumes receive service and transmit service at an intermediate node. With a single 128 KiB/s TX/RX calendar, a relay serving a one-way through-flow has an optimistic steady-state useful wire-rate ceiling of **64 KiB/s**, before retries, other traffic and protocol overhead. GCS receipt service is shared across all sources. Disjoint links in this simulator can operate simultaneously; that is not evidence that physically nearby links do not interfere.

The current service loop allows at most one hop per packet per 0.5 s mission tick. A current far-corner path exploiting second-neighbor links can have 14 hops, creating at least roughly 7 s of hop-stage delay before congestion/retry effects. This statement must be confirmed against actual route/event timestamps before use as a benchmark result; it is a model-derived warning about a ten-second deadline.

[UDP usage guidance](https://www.rfc-editor.org/rfc/rfc8085) supports congestion-aware pacing, bounded traffic and explicit application reliability. [Fragmentation guidance](https://www.rfc-editor.org/rfc/rfc8900) supports avoiding fragile IP fragmentation. A 512-byte raw image chunk expands under base64 and JSON; the serialized payload is not the same thing as Ethernet/IP/UDP/MAC wire length. Full-frame retransmission every two seconds can offer duplicate chunks faster than a bottleneck drains. Selective retry and deadline-aware admission are candidate experiments, not implemented features.

[AoI](https://arxiv.org/pdf/2007.08564) asks how old the newest received observation is. That differs from mean delay of delivered packets: a receiver can have few quick deliveries and long blind intervals. For replaceable telemetry, coalescing obsolete updates may improve age and reduce queues. Required evidence images need separate reliable completion rules and cannot be discarded merely to improve freshness averages.

## Exact metric denominators

The following are current semantics, verified by reading the router and resilient receiver:

| Metric | Numerator / event | Denominator / clock | Interpretation |
|---|---|---|---|
| Logical-packet PDR | Unique packet IDs with terminal delivery | All unique offered packet IDs, including pending and offers dropped for unavailable source/no route | `get_network_stats`; repeated same ID is idempotent |
| Terminal-outcome delivery fraction | Delivered | Delivered + terminal dropped | Not the router’s main PDR; excludes pending and must not replace it |
| Delivered-packet mean latency | Sum(receipt - frozen offer time) for GCS packets | Number of GCS-delivered packets | Success-conditioned; missing/dropped packets have no finite observed delivery time |
| Successful hops | `total_hops` increments after each successful link attempt | No fraction currently exposed | Includes forwarding of packets that may later fail end-to-end |
| Completed radio attempts | Fully serialized attempts, success or loss | Attempts begun/whole attempts as explicitly chosen | Partial aborted attempts and their consumed bytes are separate |
| Attempt success fraction | Successful hops | Completed radio attempts in the bounded-service model | A diagnostic derivation, not logical-packet PDR; source=destination/direct-call exceptions require care |
| Complete image receipt | Capture reassembled from all unique chunk indices; length and SHA-256 verified | Captures offered or required PoIs, explicitly choose one | Recaptures and retry chunks must not silently change denominator |
| PoI reporting deadline success | Required PoIs whose complete image arrives within 10 s of first detection | All required PoIs, with undetected/unreceived separately shown | Current strict check also requires every PoI detected |
| Information age | Now - timestamp of newest useful received observation | Time distribution per source, including disconnected intervals | Proposed metric; not currently equivalent to packet delay |
| Recovery latency | New post-fault useful receipt from each required affected source | Fault onset to last required receipt | Distinct from router `avg_recovery_latency`, which is E2E delay of rerouted packets |

Relevant code: `comm/mesh_router.py` lines 224, 326, 364, 419, 503 and 589; `simulation/resilient_uavx.py` lines 896–933 and 966–976; `simulation/uavx.py` lines 585–600. The resilient receiver permits duplicate packets but only one byte sequence per capture/chunk identity. Its image resend loop creates new logical packet IDs for every chunk every two seconds until receipt. Therefore chunk retransmissions increase offered-packet counts while image completion de-duplicates them.

The older `simulation/metrics.py` computes GCS received/(received+dropped), while `simulation/challenge_metrics.py` uses router delivered/offered. That denominator difference should be visible when comparing historical outputs. It is a reporting consistency issue, not evidence that the verified resilient artifacts used the old denominator.

Recommended additions: per-source/per-class PDR, offered and pending counts at cutoff, deadline-success fraction, image completion/size/retry distributions, p50/p95/p99/max latency with the successful count, time-weighted AoI and outage tail, serialized useful/retry/control bytes, queue delay/overflow, and fault-correlated first/final restored service. Zero offered traffic should be “not observed,” not interpreted as success.

## Inter-vehicle safety: preserve assumptions

The [ORCA author page](https://gamma-web.iacs.umd.edu/ORCA/) describes reciprocal responsibility split and LP velocity selection. The linked original paper currently returns 404, so only page-supported claims are used. The [official RVO2 page](https://gamma-web.iacs.umd.edu/RVO2/) states Apache 2.0 release on 2016-05-04 and distinguishes 2D RVO2 from RVO2-3D, whose described implementation lacks static obstacles. Its historical version list is not proof of the latest repository release.

The [heterogeneous CBF paper](https://arxiv.org/html/1609.00651), sections II–IV, uses control-affine dynamics and then bounded-acceleration 2D double integrators. Its safe set combines separation and approaching velocity: braking distance depends on v_close²/[2(a_i+a_j)]. It assumes the controller can satisfy the barrier and input constraints from an initially safe state. The conclusion explicitly acknowledges infeasible crowded configurations. Conservative estimates of a neighbor’s available braking capability are needed.

For AeroMesh this motivates, rather than proves, an acceleration-aware safety layer. Evaluate failed/grounded agents as noncooperative obstacles, observed closing speeds, stale-neighbor uncertainty, actuator saturation, solver infeasibility and command tracking. Sequentially projecting individual velocity constraints and then norm-clipping can alter prior constraints; results must remain checked from measured physics. Sampled observations cannot alone rule out intersample excursions. Plant/sampling sources and calibration are developed in [the physics research](physics-control-research.md).

## Proposed experiments and priorities

| Priority | Experiment | What would falsify the proposal | Cost/dependency |
|---|---|---|---|
| First | Standardize logical-packet/image/deadline metric names and denominators | Same trace gives contradictory totals; pending/undetected traffic disappears | Low; trace analysis only |
| First | Scenario admission report with optimistic travel, dwell, return and fleet-time bounds | A rejected case has a valid feasible schedule under the same assumptions | Low/medium; no policy relaxation |
| First | Per-node local state and timestamped message replay; inject asymmetric delay/loss | Planner still reads hidden global truth during outage | Medium; architecture boundary needed |
| First | Radio load/deadline sweep by image size, simultaneous sources, route length and burst loss | Better PDR hides worse image deadline fraction or stale telemetry | Medium; current service model sufficient for first comparison |
| Next | Selective chunk retransmission and bounded telemetry coalescing | Retry/ack load exceeds saved bytes; required evidence disappears | Medium; receiver feedback and protocol timing |
| Next | Feasible greedy vs current matching vs limited partial replanning | Complexity adds no mission/deadline benefit on held-out cases | Medium; identical inputs and frozen benchmarks |
| Next | Robust topology diagnostics: root cuts, predicted time-to-link-break, battery at articulation/relay roles | Diagnostics improve while measured service worsens | Medium; explicitly charge signaling |
| Later | Actual MANET implementation/emulation comparison | Claimed protocol is only global graph recomputation | Higher; existing resources/platform constraints |
| Later | Certified or robust sampled-data multi-agent safety filter | Feasibility/saturation/intersample violations not represented | Higher; calibrated dynamics and state-error bounds |

Every comparison keeps fleet=30, all-aircraft landing, separation and range requirements fixed. Report impossible strict outcomes and graceful recovery separately. Mission continuity after total vehicle loss is useful evidence, but cannot become a strict all-aircraft-return pass.

## Retrieval limitations and next wave

The agent-reach CLI and mcporter are unavailable; no installation was attempted. Jina Reader and read-only GitHub tools are available. Google was temporarily blocked; DuckDuckGo and arXiv search were used for discovery. Several experimental arXiv HTML pages redirected to an SVG and were excluded, then replaced by PDF text. The original ORCA paper link returned 404. The source ledger marks abstract-only sources and reading depth; missing material is not filled with search snippets.

Next wave: deepen persistent-monitoring assumptions, routing timer/reliability details, distributed stale-state failure cases, benchmark counterexamples, and implementation-priority estimates. No original evidence package, production code, dependencies or city assets are changed.

## Wave 2: evidence-driven counterexamples and implementation details

### Actual nominal image traffic is small and repeatedly offered

A read-only analysis of the existing `evidence/fleet30-core-live-final-01/packets.json` trace (same source hashes as above) found:

| Traffic | Offered logical packets | Delivered | Dropped | Mean delivered delay | p95 delivered delay | Maximum |
|---|---:|---:|---:|---:|---:|---:|
| Synthetic telemetry | 28,622 | 28,592 | 30 | 2.155 s | 4.5 s | 8.0 s |
| Image chunks | 103 | 103 | 0 | 4.903 s | 6.5 s | 6.5 s |

The p95 above uses the sorted observation at index floor(0.95*n), capped at n-1. The raw trace remains the authority; this was not a rerun. Only 33 unique capture/chunk identities generated the 103 image offers: 17 identities were offered three times, ten four times, and six twice. Ten received images totaled **14,357 raw bytes**, while offered image JSON envelopes totaled **98,370 bytes**. That is about 6.85 offered serialized bytes per unique raw image byte, before multi-hop forwarding and link retries. Complete image delays were 2.5–6.5 s.

This record establishes working transfer of those small procedural 128×128 images, not a throughput qualification for richer camera imagery. It also shows why packet PDR=100% for image chunks can coexist with wasteful duplicate traffic: duplicate logical packets can all arrive after an image has already completed.

The source retry loop stops after reading shared `task.report_s`. No acknowledgment packet returns over the mesh. A future selective-retry design must charge ACK/NACK bytes, routing and delay; it cannot retain an instantaneous cross-network receipt oracle. Current telemetry is only `{"kind":"telemetry"}` with requested padding, so the delivered packets do not convey the pose/velocity/energy/role state shown by the truth-driven viewer.

### Static far-corner graph check

A standalone mathematical graph calculation, not a simulator mission, instantiated exactly the current 27 slot positions and 100 m 3D cutoff. The shortest GCS path from the outer endpoint has **14 edges**, and root-to-endpoint vertex connectivity is **2**. With the current per-packet one-hop-per-0.5-second-call rule, the hop-stage lower bound is **7.0 s**.

This confirms the model-derived graph count. It does not establish a real-radio latency floor; nor does it include actual deployed transients, extra nearby vehicles, queueing, retries or updated paths. In particular, reducing the mission step also reduces `hop_delay_s` in the current constructor, so a timestep change can change network performance independently of physical realism.

### Necessary radio-load constraints

For each node v, let incident flow f contribute offered serialized bytes at rate b_f and an expected attempt multiplier e_f on an incident link. A necessary stationary service condition in the current radio model is:

sum(incident b_f e_f) < B_v,

with receive and transmit work both included. The inequality is necessary for sustained stable load, not sufficient for a deadline. It ignores finite bursts and correlations. For a one-way through-flow on a single relay with equal incoming/outgoing service, this reduces to roughly 2b < B. A GCS bottleneck instead aggregates every receiving flow. A real interference conflict graph can impose additional constraints between links that do not share a node; the current disjoint-link concurrency does not model these.

The [guaranteed-service specification](https://www.rfc-editor.org/rfc/rfc2212) [SW28] makes explicit why a delay guarantee requires a traffic envelope, admitted service and bounded deviations. It does not imply that AeroMesh implements IntServ or a reserved path. Similarly, [DiffServ service classes](https://www.rfc-editor.org/rfc/rfc4594) [SW26] and [802.11 mapping](https://www.rfc-editor.org/rfc/rfc8325) [SW27] distinguish traffic labeling from the conditioners, queues and MAC treatment that deliver it. Merely naming a packet “control” does not grant actual radio priority.

A useful offered-load benchmark therefore needs raw image size, base64/JSON/protocol expansion, image arrival bursts, application retry policy, link retry policy, competing telemetry/control, shared endpoint service, GCS bottlenecks and end-to-end receipt. A payload limit inferred from nominal average throughput alone is unsafe.

### Failure detection is an inference process

[SWIM](https://www.cs.cornell.edu/projects/Quicksilver/public_pdfs/SWIM.pdf) [SW25], sections 3.1 and 4.2, separates local failure detection, disseminated membership knowledge, and a suspicion stage that can be refuted. A delayed or lost response is not equivalent to confirmed destruction. This is a design principle, not a recommendation to transplant a PC-cluster protocol into the aircraft.

An evidence trace should separate:

1. Fault truth injected by the scenario.
2. Last valid observation and last receipt at each observer.
3. Local suspicion threshold crossing.
4. Local declaration/role change and the evidence used.
5. Other agents learning of that declaration.
6. Replacement dispatch, arrival, path availability and useful delivery.
7. False-positive suspicion cleared or old membership information rejected.

For radio silence, an operational decision may need a conservative timeout while the cause remains unknown. That timeout must be evaluated against false alarms, queue delay, message loss, CPU stalls and asymmetric links. A replacement should not be assigned the same occupied space on the assumption that the silent vehicle has vanished. Total radio loss may still permit local onboard safety under a known policy; it cannot satisfy continuous connectivity of that aircraft during the outage.

The current scenario injector gives the central state exact fault kind/target/time. Root is auditing that authority boundary separately. A benchmark that feeds injected truth directly to the planner measures response after perfect detection, not detection capability.

### ORCA, AVO, CBF and MPC: the implementation choice changes the claim

| Method/source | Formal or implemented model | Critical assumption for this project | Appropriate role |
|---|---|---|---|
| ORCA/RVO2-3D [SW16, SW22] | Reciprocal velocity-space half-planes/planes, maximum speed | Neighbor state known; reciprocal responsibility; adopted velocity semantics | Kinematic reference baseline with measured downstream tracking |
| AVO [SW21] | 2D disc robots, proportional acceleration toward chosen velocity | Perfect inferred obstacle pose/velocity and specified acceleration response | Acceleration-aware comparison; not a generic 6DOF proof |
| Heterogeneous CBF [SW18] | Bounded-acceleration double integrators | Initially safe state, feasible constraints, conservative braking estimates | Candidate safety filter with explicit feasibility and actuation validation |
| Sampled-data CBF [SW20] | Zero-order-hold nonlinear system with bounded uncertainty/known delay | Entire hold interval and backup-safe set conditions | Source for robust interval margins; not solved merely by fast sampling |
| Luis et al. DMPC [SW23] | Identified linear trajectory-tracking dynamics; shared future trajectories | Soft constraint relaxation; input/state relation; experiments run centrally | Transition-performance baseline with architecture labels |
| Chen et al. IMPC-DR [SW24] | Double integrators, terminal equilibrium and recursively feasible plans | Initial feasibility; target geometry; sufficient communication radius | Stronger proof-based comparison after checking exact assumptions |

The [pinned RVO2-3D implementation](https://github.com/snape/RVO2-3D/blob/f940bfba8bac9532ecae948f11639085e8cbf277/src/Agent.cc) computes a half-responsibility plane at line 434, calls its LP solvers at lines 438–442, and then at lines 471–473 assigns `velocity_ = newVelocity_` and advances `position_ += velocity_*timeStep_`. Those are kinematic simulator updates. Feeding a selected velocity to Bullet changes the downstream model; it does not transfer the library's instantaneous update semantics.

The [AVO paper](https://gamma-web.iacs.umd.edu/AVO/AVO.pdf), pp. 1–2, explicitly warns that clamping a velocity after selection to respect acceleration can invalidate collision avoidance. Its own remedy assumes a particular proportional-acceleration response and perfect sensed geometry. That is a closer question to ask of AeroMesh's controller than whether an algorithm carries a “collision-free” label.

The [Luis et al. paper](https://arxiv.org/pdf/1909.05150), pp. 2–6, identifies tracking dynamics from experimental step responses, selects 5 Hz replanning for that platform, shares future trajectories, and permits negative collision slack. Its physical platform lacked inter-agent communication, so the agent optimizations were parallelized on one offboard computer. It explicitly reports that larger relaxations at high density can lead to collisions. Its results are valuable; they are not an unconditional distributed hard-safety guarantee.

The [Chen et al. paper](https://arxiv.org/pdf/2202.06071v4), **p. 8, Theorem 2**, requires target separation greater than r'_min+2 epsilon and no three-or-more collinear horizontal target projections for its no-stable-deadlock result. Current relay targets deliberately violate the collinearity condition. This means the theorem cannot be cited unchanged; it does not prove that the current chain must deadlock. **P. 9, Theorem 3 and Corollary 1**, require initially feasible plans and use a terminal equilibrium to preserve feasibility. **P. 9, Theorem 4**, gives required local communication radius:

R_comm = 2 v_max K h + r'_min + 2 epsilon.

Thus with v_max=5 and R_comm<=100, a direct-neighbor interpretation requires 10(Kh)+r'_min+2 epsilon<=100. Choosing a long MPC horizon while assuming only 100 m fresh direct communication can contradict the cited proof. Multi-hop communication adds route/queue delay and is not automatically equivalent to that local communication assumption. These theorem statements were checked in the original rendered PDF pages 8 and 9.

The current `_safe_commands` uses positions and requested velocities, performs three sequential pairwise projections within 34 m, then applies geofence component clamps and a 4 m/s norm limit. It does not use observed closing velocity or actuator state inside the projection. Later operations can change earlier pair constraints. The physics model's 120 Hz extrema do not include full pairwise separation; that mission quantity is sampled at 20 Hz. Treat this as a heuristic whose measured outcomes and model limitations remain explicit.

### Fixed-resource feasibility categories for benchmark interpretation

| Category | Example | Correct interpretation |
|---|---|---|
| Statically feasible, dynamically unqualified | 27-slot far-corner graph has links and 20 m clearance | Graph feasibility only; deployment, return, energy and tracking still needed |
| Conditionally feasible with available slack | Temporary link loss while alternate path and capacity remain | Measure outage, repair delay and complete reporting; a dispatch event is not recovery |
| Algorithm can waste feasible resources | Equal-priority cross-map ping-pong, unnecessary withdrawal, full-image retries | Compare feasible baselines and ablations under identical announcements |
| Energy/time lower bound violated | 27 continuously airborne for all 45 min under 1200/720 assumptions | Explicit resource infeasibility; do not add aircraft to label it passed |
| Announcement infeasible | Fresh 30 s dwell required after t>2670 | Valid input schema can still be impossible; return safely and state incompletion |
| Total-loss strict objective impossible | Destroyed aircraft must still return; sole radio entirely unavailable but connectivity demanded | Preserve failed strict constraints; survivor continuity is separate |
| Uncertain without measurement | Urban shadowing, asymmetric links, correlated interference, physical braking | State missing model/data; do not infer a pass or universal impossibility |

### Concrete benchmark and ablation cases

These are proposed experiments, except the isolated radio study explicitly documented below. Keep fleet=30 and original strict limits in every mission-level comparison.

| ID | Fixture/change | Held fixed | Primary falsification criterion |
|---|---|---|---|
| N1 | Logical chains of 1/5/10/14 hops; router step 0.5/0.25/0.1 s | Byte rate, payload, loss seed, queue limit | A “network improvement” is merely reduced per-hop tick delay |
| N2 | 512 B, 32 KiB, 256 KiB encoded image payload; one image vs synchronized burst | Routes and service | PDR stays high while full-image ten-second success collapses |
| N3 | Full-image 2 s retry vs one offer; one missing logical chunk | Same intrinsic link retries | Duplicate offered bytes worsen completion or queue tails |
| N4 | Selective ACK/NACK retry, including reverse feedback loss/delay | Image identities and hard deadline | Savings vanish when ACK traffic and latency are charged |
| N5 | Telemetry FIFO vs latest-only replacement | Per-source generation and explicit observation timestamps | Lower mean packet delay hides higher time-weighted AoI or missing sources |
| N6 | One-hop downlink degradation only | Forward links and offered traffic | Symmetric link oracle misses asymmetric loss/detection behavior |
| N7 | Bursty correlated outage vs IID loss at same mean | Mean loss and mission instance | IID success is wrongly generalized to deadline tails under bursts |
| N8 | Control load flood and high-priority imagery | Total byte service and safety requirement | Priority rule starves another required class or falsely promises all deadlines |
| A1 | Greedy feasible/current matching/partial replan | Task arrivals, priorities, energy, radio truth access | Better distance/auction score accompanies worse strict mission success |
| A2 | Three equal-priority PoIs alternating across map; then higher-priority arrival | No future coordinates | Churn, repeated role swaps or starvation explained by policy |
| A3 | Charged near aircraft vs tired far relay with shared retreat obligation | 30 identities, same home pads | Local return estimate strands a relay that must support outer aircraft |
| A4 | Arrival at t=2660,2670,2690 near/far ready aircraft | 30 s dwell and actual sensing footprint | Infeasible timing is misreported as planner defect or counted passed |
| S1 | Head-on vehicles at same distance but different measured closing speed | Max speed and acceleration limits | Position-only admission accepts a state outside braking-safe set |
| S2 | Geofence corner with three simultaneous pair constraints | Current projection order and norm cap | Final command no longer satisfies an earlier pair constraint |
| S3 | Radio-silent but physically occupied relay location | Collision model and 20 m limit | Replacement assumes failed aircraft disappeared |
| S4 | Collinear chain targets, then slight noncollinear control case | Fleet, spacing and radio range | A deadlock proof is cited outside target-geometry assumptions |
| S5 | State age / command hold / jitter sweep | Physical integrator and sensor truth | More frequent logging hides intersample safety violations |
| F1 | Same injected fault with immediate oracle detection vs delayed local suspicion | Physical fault time | Recovery claim conflates perfect fault truth with observed detection |
| F2 | Return-to-service stale membership message after newer failure/rejoin | Message identities and source times | Old information revives authority or overwrites current role state |

For every case, preserve seeds, exact scenario, source hash, runtime configuration, offered traffic, packet/capture identifiers and all failed outcomes. Use matched instances for comparison. Any deterministic fixture counterexample disproves a universal guarantee, but a finite set of successful fixtures does not establish one.

## Completed isolated radio studies

Two authorized research-only studies used the unchanged production `MeshRouter`. Their scripts, raw results and before/after hashes are saved separately. Neither is an integrated flight, physical RF qualification or reliability campaign.

| Study | Cases and measured work | Main evidence | Interpretation limit |
|---|---|---|---|
| [Payload, timestep and retry sensitivity](radio-sensitivity-01/README.md) | 240 cases in 97.05 s; one seed 42 in IID cases; 24 scripted missing-chunk cases included | At 14 hops/dt 0.5 a 32 KiB frame takes 8 s; 256 KiB misses 10 s even without loss. At dt 0.1 the 256 KiB frame takes 14.8 s. Whole-image retries greatly inflate bytes and queue drops. | No ACK traffic; application retries stop from a shared receiver oracle. Heterogeneous success counts are not reliability estimates. |
| [Transported ACK/NACK and XOR parity](radio-feedback-01/README.md) | 80 deterministic cases in 4.063 s | At 14 hops/dt 0.5, one erased chunk: full-image retry completes in 9 s, one NACK repair in 21 s, XOR parity in 7.5 s. The respective source ACK times are 16/28/14.5 s. | Different metadata envelope from first study; loss profiles are deterministic. One NACK round and one XOR parity chunk are explicitly limited protocol sketches. |

Do not combine these into “320 reliability runs.” The first study crosses heterogeneous load/timestep/path/loss settings with one seed; the second contains deterministic protocol counterexamples. Repeating a seed across policies that generate different traffic does not produce event-keyed paired channel realizations. Comparing empirical reliability would require repeated independent instances, common exogenous channel traces or event-keyed randomness, clearly stated policy-dependent exposure and confidence intervals.

The follow-up envelope uses 64,748 serialized data bytes rather than the first study's 66,540, straddling the 65,536-byte service available in one 0.5 s interval. Its no-loss 32 KiB image therefore completes at 7 s rather than 8 s on 14 hops. This is a packetization/scheduling threshold, not evidence that adding ACKs made the network faster. All policy comparisons within the second study use the same data envelope, with explicitly charged parity/control additions.

At 14 hops/dt 0.5, one-erasure full-image retry offers 520,393 bytes, NACK repair 66,264 bytes and XOR parity 66,035 bytes, including actual reverse control offers. Selective repair saves bytes but its receiver→source→receiver repair loop is too slow for 10 s. XOR avoids that round trip but fails two erasures. Under an all-link 3 s burst, whole-image retry completes at 11.5 s and misses the deadline; the other tested policies remain incomplete at 30 s. A 5-hop/dt 0.1 middle-link burst delivers the image at 2.1 s while losing its ACK, separating receiver success from source knowledge.

The next network priority is to **decouple or calibrate the service clock and admit actual serialized image workloads against remaining deadlines**. A hard image deadline needs a bound on remaining path service, admitted competing traffic, feedback/coding policy and outage assumptions. No finite service reservation can make a completely unavailable radio deliver data. Adaptive quality or compression must also preserve the required detection/identification value; smaller byte count alone is not mission success.

## Hidden failures, common causes and observer knowledge

A physical fault, a local suspicion and restoration of useful service have different timestamps. A useful trace should retain at least `fault_truth_s`, `last_message_observed_s`, `suspicion_s(observer)`, `failure_declared_s(observer)`, `replacement_command_received_s`, `replacement_ready_s`, `first_fresh_route_receipt_s` and `first_valid_image_receipt_s`. Only the evaluator may use fault truth to score detection. A controller must act on messages and local measurements that it actually received.

For a healthy-but-partitioned relay and a failed-radio relay producing the same received-message history, another controller cannot infer which physical cause occurred from that history alone. Silence can justify a bounded suspicion/fallback policy; it does not prove that the airframe disappeared, that a replacement can occupy its volume, or that landing authority remains available. Total radio failure, total aircraft loss and delayed telemetry need separate cases and strict metrics. A destroyed UAV still fails the original return requirement; survivor continuity is a different outcome.

| Proposed case | Hidden truth and observer view | Required measurements and failure condition |
|---|---|---|
| H1: delayed healthy relay | Identical last received sequence as a failed radio for an explicit interval | False-suspicion duration, authority withdrawal and safe recovery; do not score perfect hidden fault knowledge |
| H2: asymmetric link | Images travel upstream but ACK/control cannot return | GCS receipt and source knowledge separately; retain stale command age and loss of reverse authority |
| H3: common obstruction | Two geometrically separate routes share a building shadow or interference region | Report correlated outages, route spatial exposure and complete-image deadline failures; graph vertex connectivity alone is insufficient |
| H4: physically occupied silent relay | Radio stops but body continues under last/expired control | Minimum separation and replacement admission; preserve the obstacle/body after communication failure |
| H5: membership rejoin | Delayed pre-failure status arrives after a newer role/failure epoch | Epoch/sequence/source-time rejection; stale status must not revive authority or duplicate role ownership |
| H6: cut versus congestion | Same image/heartbeat delay caused by partition or queue backlog | Per-class service and queue age, useful deadline delivery, false reroute/redispatch rate |
| H7: burst duration versus retry cadence | Same outage interval with varying packet stage/attempt timing | All attempt timestamps and service bytes; four retries are not a fixed amount of outage tolerance |

The current synthetic telemetry payload does not carry pose, velocity, energy or role; a future observer-restricted baseline must define these payloads, timestamps and byte costs before calling its controllers decentralized. Latest-only state queues may lower age of information, but deleting old telemetry must not delete irreplaceable image chunks, task announcements or authority changes. Compare time-weighted per-source AoI, missing-source intervals and command freshness, rather than only a delivered-packet mean.

Redundant routes protect only failures outside their shared components and failure domains. With a single radio per UAV, paths share each endpoint's radio; separate graph paths may share the same RF obstruction. Coding/interleaving can spread some losses but cannot create capacity during a complete cut. A common scene manifest for visual geometry, Bullet collisions and RF obstruction is a useful future modeling boundary: current decorative parcel visuals do not establish corresponding integrated collision or RF geometry.

## Cross-review and completion record

The physics agent cross-reviewed the finite-horizon energy arithmetic, far-corner geometry, ORCA/AVO/CBF/DMPC transfer limits and collinear-target theorem caveat without substantive correction. This network review independently checked the physics report's frame/units, exact actuator state versus body-impulse error, gyroscopic cancellation distinction, 20 Hz pairwise sampling versus 120 Hz engine extrema and empirical-validation limits; no substantive correction was found. These are review findings, not additional physical qualification.

The source ledger has **28 entries**: 12 primary papers inspected through selected full-text sections, 2 institutional abstracts/metadata records, 8 official RFCs inspected through selected sections, 5 official project/documentation sources and 1 pinned official implementation source read without building. Exact per-entry reading levels and locators remain authoritative. The SWIM publication date remains explicitly unresolved; no proxy HTTP timestamp was substituted. Agent Reach's optional update check was unavailable because its CLI is absent; retrieval used its documented Jina/GitHub routes without installation. Integrated crosswind and injected-outage missions were not run for this research. The completed nominal physics result is linked above, including its connectivity failure.


