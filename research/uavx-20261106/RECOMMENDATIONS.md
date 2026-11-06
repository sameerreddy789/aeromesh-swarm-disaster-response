# AeroMesh UAV-X: research findings and improvement plan

6 November 2026. Reference implementation: `af544e505af51848f5e3a013a52b300f0cd4bfb9`. **30 simulated UAVs total, including spares. City downloads remain paused.**

The best next version is a clearly explained, communication-aware physics demonstration: aircraft move through forces, the network carries the information the controller actually uses, the ground station shows only received state, and every reported success has a traceable receipt or physical measurement. Keep the working lightweight simulator and improve these boundaries before investing in a large city or replacing the stack.

This is a research deliverable, not a claim that the proposed changes are already implemented or that AeroMesh meets all IIT Bombay qualification requirements. Runtime code was preserved during the research. Research scripts, notes, figures and an explicitly mocked interface were added separately.

## What was researched

Three specialist agents and the coordinating review covered flight dynamics/control, swarm/network planning, and visual explanation/demo design, with cross-review of the conclusions. The [source catalog](SOURCE_CATALOG.md) contains **91 source records across four ledgers**, including overlapping references and sources read at different depths. They are not 91 independent experiments or 91 papers read cover to cover. Official requirements, primary research, technical documentation, inspected code, observed experiments and design recommendations are distinguished.

The detailed deliverables are:

- [Physics, control, sensors, energy and simulator research](physics-control-research.md), with 32 source records and 40 isolated mechanics trials.
- [Swarm algorithms, radio, recovery and feasibility](swarm-network-research.md), with 28 source records.
- [Visual explanation, storyboard and demo research](visual-demo-research.md), with 16 source records and a checked offline concept.
- [Architecture, information access and sensing](architecture-observability-research.md).
- [Requirement-to-implementation/evidence matrix](requirements-and-gaps.md).
- [Evaluation and reproducibility plan](evaluation-design.md).
- [240-case radio sensitivity study](radio-sensitivity-01/README.md) and [80-case transported-feedback study](radio-feedback-01/README.md).
- [Actual received-image audit](received-image-quality.json) and [image contact sheet](received-image-quality.png).

These cover equations and frames, motor mixing/lag, drag/wind/contact, safety and control, estimation, endurance/charging, allocation and coverage, connectivity and protocols, queues and payloads, fault semantics, simulation fidelity, perception, metrics, uncertainty, reproducibility, visual design, presentation and implementation priorities. They do not exhaust all literature or replace missing organizer information.

## Start with the published score and hard constraints

The supplied problem PDF lists six weighted categories, corroborated by the live official page [R01,R02]:

| Category | Weight | What the demonstration should make visible |
|---|---:|---|
| Mission completion | 25% | Required PoIs, validated reports and return outcome |
| Communication resilience | 25% | Actual delivered traffic, outages, receipt age and deadline |
| Relay/role management | 20% | Why a relay is needed, who owns it, when a handoff completes |
| Fault recovery/reconfiguration | 15% | Disturbance, observable detection, decision and restored service |
| Safety | 10% | Measured separation/speed/height/landing and their limits |
| Innovation/technical merit | 5% | A specific justified improvement with a fair baseline |

Communication, roles and recovery total **60%**. That supports concentrating engineering and screen space on those behaviors. It does not establish what an individual judge personally prefers. Safety limits remain constraints even though safety has a smaller rubric weight.

The sample slide specifies a 45-minute mission, 20-minute maximum flight, 100 m communication range, 1000×1000 m arena, 100 m maximum height, 5 m/s maximum speed, 20 m minimum separation, ten emerging PoIs, reporting within ten seconds of first detection, and return to the start area [R06]. The team has chosen 30 aircraft including spares; nominal 30 pads and 720 s recharge are declared modeling choices requiring organizer acceptance. The current 30 s survey dwell is also a project choice, not a number specified in that slide.

The template explicitly allows centralized, decentralized or hybrid coordination with justification [R05]. Do not describe centralized control itself as disallowed. The real issue is whether the controller uses information or command paths that are unavailable under its stated communication model.

## What is already established

The earlier kinematic matrix evaluated 19 cases with consistent, source-stable evidence: **17 passed**, including 12 nominal seeds; sole-radio outage and permanent loss retained their strict failures. The selected core suite passed 486 tests.

The separate compact Bullet run completed ten PoIs at **623.5 simulated seconds**, with all 30 aircraft grounded/disarmed and all 15 checks passing. Ten aircraft launched during the run, eight were active at peak, and twenty remained grounded spares. Minimum sampled separation was **21.956 m**, actual engine maximum speed **3.989 m/s**, and maximum sampled height **92.646 m**. The earlier selected regression suite passed 534 tests; the final focused engine/adapter/verifier suite passed 55 tests. These are existing results, not new counts added by the research.

The physical backend uses 120 Hz Bullet rigid bodies, explicit inertia, four motor forces and reaction torques, motor lag, gravity, declared drag/wind and contacts. The observer does not move the aircraft: the paired 60-second native/headless comparison produced identical mission artifacts. These are useful foundations.

The new unchanged-source **full nominal Bullet run finished at 2514.0 simulated seconds (41.9 minutes)**. It completed ten PoIs and landed all 30 aircraft, with all 30 launching at some point and 28 active at the sampled peak. It passed **14/15 checks** and failed continuous connectivity: **19.8 seconds of recorded downtime** occurred without injected faults. Minimum sampled separation was 20.368 m, maximum engine speed 3.996 m/s, maximum sampled height 99.007 m and maximum sortie 876.2 s. All ten images arrived within their reporting deadlines. The independent verifier accepted the evidence and correctly returned `mission_passed=false`. See [the full summary](integrated-nominal-summary.json), [verification](integrated-nominal-verification.json) and [measured timeline](integrated-nominal-timeline.png).

This full run establishes that the current model can survey/report and return the 30-aircraft fleet within the mission clock on this scenario. It does **not** pass the entire strict profile. Repairing the physical-motion connectivity loss is now the first concrete engineering task. The kinematic nominal result and compact physics pass cannot erase this new failure.

The [outage diagnosis](integrated-nominal-connectivity-diagnosis.md) identifies five 20 Hz windows: 4.4, 3.8, 8.6, 2.7 and 0.3 seconds. The first three follow formation pivots; the final two occur during mission return. Position-based graph reconstruction confirms bridge links stretching beyond 100 m. During one return climb, the sole GCS–UAV 24 link reached 101.590 m and all 21 active aircraft lost their GCS path. This was a 3D range failure measured independently of packet delivery; the nominal scenario still has its declared 2% attempt-loss setting and no injected fault events. The final 0.3-second outage falls between saved full-position samples, so its exact disconnected identities are not proven.

The proposed repair is to retain a verified replacement path before moving a critical relay, predict **3D** link margin using measured relative motion and achievable braking, and include climbs, descents and landing transitions in that decision. Collision, connectivity and resource constraints must be checked together; holding one aircraft can still break a link if its neighbor continues. Initially test the recorded pivot/return geometries in bounded fixtures, then repeat the full nominal run with unchanged limits and all 30 vehicles. Add disconnected identities and critical-edge distances to the 20 Hz evidence so short outages are diagnosable. This is a concrete engineering proposal, not an implemented fix or a formal safety guarantee.

The diagnosis also found a logging inconsistency: a trajectory row's stored links come from the network update before movement, while its positions are recorded after movement. Those links can disagree with the same row's positions. Reconstruct geometry from position samples for this audit; a later producer fix should timestamp graph and position snapshots consistently.

Important limits remain: uncalibrated vehicle parameters, spherical collision envelopes, ideal world-down cameras, declared-time energy, simplified radio and global true-state control. Full fault/resource and hidden-scenario physics qualification remains open. Integrated crosswind and additional fault runs were not performed during this research.

## What the new experiments and code review found

| Finding | Observed evidence | Practical consequence |
|---|---|---|
| Full nominal physical motion loses connectivity | 10/10 PoIs and all 30 land at 2514 s, but 19.8 s downtime causes a 14/15 result; evidence verifier accepts the failure | Fix and regression-test the connectivity margin during actual motion before declaring the full physics mission ready |
| Ground-station state is not yet received telemetry | Telemetry payload is only `kind`; shared state is refreshed directly from Bullet | Add serialized sample values/timestamps and a separate receive-only store before claiming a live GCS state view |
| Commands and fault knowledge bypass the modeled radio | Direct `physics.command()` calls; controller shares injected failure/graph truth | Keep the current architecture as an explicit ideal-information baseline; test local authority and transported intent |
| Image retry completion has ideal feedback | Sender stops from shared `task.report_s`, without a returning ACK packet | Model reverse feedback and distinguish GCS receipt from sender knowledge |
| Radio service depends strongly on the outer timestep | At 14 hops, tiny image completion was 7.0 / 3.5 / 1.4 s for 0.5 / 0.25 / 0.1 s steps | Separate/calibrate service timing before claiming a physical-network latency improvement |
| Payload size can dominate mission success | In the isolated sensitivity sweep, 256 KiB missed ten seconds on all tested 5/10/14-hop paths, even at zero loss | Define useful image quality/size, serialize actual bytes and admit traffic against its deadline |
| High packet PDR can hide zero usable imagery | A 32 KiB image with one missing chunk had 63/64 delivered chunks and no valid complete image | Put complete-image deadline success on the screen, alongside packet counts |
| Selective retransmission is not always faster | With one erasure at 14 hops/0.5 s: whole-frame retry 9 s, tested one-NACK repair 21 s, one XOR parity chunk 7.5 s | Choose feedback/redundancy by delay and loss envelope; XOR failed with two erasures and is not a general solution |
| Correlated loss defeats simple remedies | In one all-link 2–5 s burst, whole-frame retry completed at 11.5 s; other tested policies were incomplete at 30 s | Test burst/common-mode disturbances separately from IID loss |
| Numerical contact/impulse error is measurable | At 120 Hz, a 3 m drop penetrated the sphere's ground plane by about 20.2 mm before settling; refinement reduced the error | State contact tolerance and test timestep convergence; a rigid-body engine is not automatically a calibrated impact model |
| Exact motor state is not exact motion integration | Exponential lag state matched its analytic result, while applied body impulse remained timestep-dependent | Verify actuator state and body trajectory separately |
| Camera detail is limited | All ten compact images hash-match; at the measured first-camera height, ideal flat-ground sampling distance is about 0.362 m/pixel | The images prove rendering/transport, not reliable survivor recognition |
| Visual and physical scenes differ | Camera parcels are in an independent TinyDisplay scene; integrated Bullet has ground and drone spheres; RF has no parcel geometry | Introduce a small shared environment description before adding a city |

The first radio study used one fixed RNG seed per stochastic condition; the second used deterministic counterexamples. **240 sensitivity cases plus 80 feedback cases are not 320 independent mission reliability trials.** Neither study includes flight, background telemetry load, real RF interference or discovery protocols. The feedback experiment uses a different metadata envelope from the first study; its 8-to-7 s no-loss difference crosses a byte-service threshold and must not be credited to a new protocol. Full details and raw measurements are preserved.

The 40 mechanics trials were 32 convergence cases and eight rotational cases. Frame/motor torque signs and nonprincipal-axis gyroscopic behavior were checked against equations. Free-fall position error and torque-free energy drift decreased with smaller timesteps. These verify properties of the declared model; no airframe, propeller, battery or real wind was identified.

## Ranked changes, with concrete completion criteria

Effort labels are relative engineering estimates, not schedules or promised gains. Existing resources remain fixed.

| Order | Change | Why it comes here | Dependencies / effort | Acceptance test |
|---|---|---|---|---|
| 1 | Repair full-map connectivity and extend physics qualification | The completed nominal run has 19.8 s connectivity loss despite successful reports/return | Trace diagnosis and joint motion/connectivity margins; medium-high | Zero nominal downtime without sacrificing separation, energy, reports or 30-aircraft return; then named recoverable fault/resource cases, retaining all failures |
| 2 | Explicit truth/onboard/GCS information boundaries | Direct global-state access can make resilience look stronger than it is | Versioned telemetry/intent contracts; medium-high | Uplink/downlink independently cut; GCS age grows; no hidden current pose; expired commands cause declared local behavior |
| 3 | Correct transport timing and image deadline model | A clock artifact or tiny payload can dominate the score | Agreed radio semantics and packetization; medium | Invariant physical service under planner-rate changes; measured bytes/queues; image size/loss/route sweep with useful-quality criterion |
| 4 | Transported receipt feedback and bounded retry | Shared completion knowledge hides reverse-path cost | Item 3; medium | Lost ACK, lost tail chunk, reorder, burst loss and queue pressure; receiver completion and sender acknowledgment logged separately |
| 5 | Acceleration-aware safety and finer evidence | Velocity projection does not prove braking under lag/stale state | Conservative achievable braking/estimation bounds; high | Head-on/crossing/return encounters, noncooperative failed bodies, saturation and infeasibility; measured intersample safety or explicit counterexample |
| 6 | One small consistent scene model | Buildings must have stated physical/visibility/RF meaning | Shared frames/units/scene IDs; medium | Known block appears at same pose in RGB/observer/collision/LOS tests; visual-only objects explicitly tagged |
| 7 | Preserve resource feasibility and explain rejection | 30 vehicles cannot cover every arbitrary permanent-loss/late-task demand | Existing reserve/admission logic; low-medium | Optimistic lower bounds plus actual schedule checks; no extra drones or free energy; distinguish impossible case from planner failure |
| 8 | Improve the live explanation | Judges need to see why the system acts and what reached the center | Items 2–4 for genuine received-state view; medium | Trace-derived route/role/fault timeline, complete image delay, 30 total count, stale state and strict failures; observer non-interference |
| 9 | Resolve discovery mode and measure sensor utility | Announced targets and unknown-area search are different problems | Organizer semantics or explicitly separate test modes; medium-high | Hidden target coordinates cannot influence planner; coverage/detection/false alarms/localization measured from sensor observations |
| 10 | Calibrate energy and vehicle details when data exists | Additional equations with guessed values do not establish realism | Chosen vehicle/propeller/pack data; high | Unit-consistent thrust/torque/lag/inertia/power fits and held-out validation; retain declared 1200 s scenario cap |
| 11 | Consolidate reproducible handoff | Legacy setup/performance prose can send reviewers to the wrong backend | Current launch/evidence inventory; low-medium | One canonical guide, pinned environment, smoke/full/expected-failure runs, another person's clean setup, license inventory |

The two strict total-loss failures cannot be “fixed” by renaming a destroyed aircraft as landed or excluding an isolated UAV from required connectivity. Survivor mission continuity is useful and should be shown, but it is a separate outcome. No backup radio or extra aircraft is assumed.

## Which algorithms are worth using

| Method family | Sensible use here | Condition before adopting |
|---|---|---|
| Current budgeted matching | Reproducible centralized assignment baseline with energy/return constraints | Publish exact objective and compare to feasible greedy on paired scenarios |
| CBBA / asynchronous variants | Distributed task negotiation if message-limited autonomy is needed | Implement conflict resolution, timestamp/epoch rules, actual messages and delayed/lost-state tests; current matching is not CBBA |
| Graph connectivity / root cuts / disjoint paths | Diagnose vulnerable relays and guide handoff | Count actual service restoration and energy cost, not graph score alone |
| Shortest-hop / expected-attempt / throughput routes | Baselines for the declared network model | Equal observed inputs and charged control traffic; current central Dijkstra is not a deployed MANET protocol |
| OLSR/AODV/BATMAN variants | Later protocol/emulation comparison | Need discovery/convergence/overhead tests on the same workload; no universally best protocol was established |
| Lawn-mower / partitioned coverage | First unknown-target-search baseline in known open areas | Define optical footprint, overlap, misses, relay support and energy feasibility |
| Frontier / RACER / informative planning | Unknown-map exploration or uncertainty-driven revisits | Include estimation/map exchange and all UAV-X constraints; mapping-paper results do not transfer directly |
| ORCA | A reference for reciprocal velocity avoidance | Account for lag, acceleration, failed/noncooperative agents, obstacles and stale neighbors; raw velocity replacement is not force-driven tracking |
| CBF / robust sampled-data filter | Explicit safety constraints with modeled dynamics | Feasible safe initial state, input bounds, sampling/uncertainty and solver failure handled; current sequential projection is not this proof |
| MPC | Joint trajectory/resource constraints when computation is justified | Validate horizon, communication range, feasible initialization and soft-constraint violations; some published deadlock proofs exclude collinear goals |
| Learning/RL/LLM planning | Later bounded policy experiment or explanation aid | Beat a simple baseline without privileged information; model output must not bypass safety or masquerade as a recorded causal decision |

The present relay goals are deliberately collinear. One inspected distributed-MPC deadlock theorem requires that no three target projections be collinear; its guarantee therefore cannot simply be cited for this geometry. A different paper's “distributed MPC” experiment ran on one offboard computer. Algorithm decomposition and decentralized deployment are distinct claims. The specialist notes record the exact source assumptions [SW23,SW24].

## How the simulation should develop

**Tier 1 — retain and qualify the current lightweight system.** Keep 30 Bullet bodies, the existing motor controller, fixed timesteps, procedural geometry and separate observer. Measure full missions and disturbance limits. Increase fidelity only where an experiment shows that it can change a planning/control decision.

**Tier 2 — make sensing, communication and authority coherent.** Add a shared small scene description, locally available state, sensor/command delay, actual serialized telemetry, received-state display and independent event clocks. Use configurable simple obstacles for collisions, visibility and radio obstruction; do not equate those three effects automatically. This tier is likely to improve the challenge demonstration more than a large asset pack.

**Tier 3 — identify physical parameters and optional autopilot behavior.** Use data for the chosen airframe/propeller/pack; validate motor lag, drag, inertia, control response and energy. A later PX4/ArduPilot/SITL adapter is a separate backend and qualification program. Simulator API acknowledgment is not measured completion.

**Tier 4 — consider Unreal/city presentation after storage and purpose are clear.** A district inside the mission area can support meaningful obstacle and visibility tests. An entire million-building dataset is unnecessary for a 1 km² mission unless a specific evaluation requires it. A city renderer should consume the same physical state and scene metadata, with explicit coordinate conversion, without supplying fictional success.

The local host is an Acer Nitro ANV15-41, Ryzen 5 6600H, 12 logical processors and about 16 GB RAM; the existing native viewer used an RTX 3050. Only about 2.1 GiB RAM was free at one research check, so additional integrated simulations were not launched concurrently. These are host observations, not minimum requirements or clean throughput measurements.

Storage should favor sparse required images and bounded previews. Calculated raw RGB for thirty 128×128 cameras at 5 fps over 45 minutes is about 19.9 GB; thirty 640×480 cameras at 10 fps would be about 746.5 GB before metadata/depth. Those are hypothetical uncompressed workloads, not current capture rates. Keep required original evidence, use content references/compression where appropriate, and do not record all cameras at high rate merely for appearance.

Current event/packet/service/safety collections are largely retained in memory and written at completion; trajectory and physical telemetry stream separately. A later bounded append-only event log with explicit incomplete-run status would improve crash recovery and memory use. Preserve byte accounting even if repeated payloads are stored once for evidence. This is a proposed engineering improvement, not an implemented logger change.

## A demonstration that explains the work

Open the [interactive offline concept](demo-wireframe.html). It is **mock data and a topology schematic**, not a live simulator or a feasible flight geometry. Its purpose is to make the proposed information hierarchy concrete. The [actual-image sheet](received-image-quality.png) is separate measured rendering evidence.

Recommended desktop layout:

1. A stable eagle view with thirty identities, simple role shapes/colors, selected route and optional full topology. Grounded spares remain counted.
2. A received-image panel with capture time, complete receipt time, fragments, byte count, hash status and first-detection deadline. Partial imagery never appears “complete.”
3. Selected-vehicle details with state source and age. Stale/unavailable state stays visibly unknown.
4. A short causal timeline: observed symptom → detector decision → role assignment → verified handoff → useful service restored.
5. A compact safety/resource strip that says which measurements are sampled and which are unavailable in received-state mode.

Use color and shape together; avoid a screen full of crossing links or thirty dense telemetry panels. Close-up quadcopter views can show motor-driven roll/tilt and contact, then return to the overview. Do not smooth away an outage by animating a stale received-state marker as though its current pose were known. The viewer remains observational.

### Provisional four-minute edited video

The exact allowed video length has not been established from organizer instructions. This is a production proposal, not an official time limit.

| Time | What to show | What to say / substantiate |
|---|---|---|
| 0:00–0:25 | Mission area, 30 total, backend/scenario identity | Define the disaster task, resources and declared physics/radio limits |
| 0:25–1:00 | Takeoff, relay deployment, brief physical close-up | Commands produce forces; measured state drives this backend; explain one role decision |
| 1:00–1:40 | PoI observation and actual complete GCS image | Follow one report along its recorded route and show the ten-second calculation |
| 1:40–2:35 | A tested recoverable fault and timeline | Separate injected truth from observed detection, assignment, physical handoff and restored receipt |
| 2:35–3:15 | Energy reserve / replacement / return decision | Explain why an aircraft returns and how coverage/connectivity are maintained |
| 3:15–3:45 | Completion and controlled landing | Show actual final outcome, all 30 count and retained failures if any |
| 3:45–4:00 | Evidence/baseline summary | State what passed, what is still unqualified and how to reproduce |

Edited time compression must be labeled. Do not splice different runs into one apparent successful mission. A fault segment must come from a run that actually exercised that fault; the nominal compact pass does not supply fault-recovery footage.

### Live jury execution

Start with one current launcher and a brief run/scenario identity display. Load the provided scenario, keep the default overview stable and reveal future tasks only when the scenario policy allows. Show the receipt and event evidence as it arrives. Retain any failure trace. A prerecorded example may explain the intended behavior after a live failure, but must be clearly labeled as a different recorded run.

For rehearsal, ask teammates to identify the disconnected/stale vehicle, the delivered report's route, the reason for replacement and whether an image is actually complete. Record correctness and interpretation time. No external user study has been conducted; the research UI's interaction/layout checks are not evidence that judges will prefer it.

## External questions that research cannot answer

- Whether this team is registered/qualified for Stage2 and the official advanced scenario/log format.
- Whether PoI coordinates are announced or must be discovered, and what image/sensing utility is required.
- Acceptance of 30 aircraft, pad/staging geometry, recharge assumptions and the selected simulator.
- The official submission route: the public page includes a Forms link but its rules/FAQ say email-only.
- Root project licensing and any additional terms for code/assets used in a final submission.

The live page also says “seven” criteria while enumerating six weights totaling 100%; no seventh criterion has been invented. No organizer contact, submission, hardware arming, city download or additional fleet resource was performed during this research.

## Immediate implementation sequence

First repair the 19.8 s connectivity loss in the frozen full-map physics result, using the recorded outage phases and actual motion margins. Then implement timestamped telemetry/intent and receiver-state boundaries; correct/calibrate transport timing and feedback; add safety/scene consistency tests; integrate the demo panels from real logs. Re-run the same baseline and adversarial scenarios after each change. Leave electrical calibration, learned perception and large city rendering for clearly defined later milestones.

The result to aim for is a jury member being able to follow one chain of evidence: **a task emerges, a feasible team is assigned, physical aircraft observe it, bytes traverse a disrupted network, the complete report reaches the center on time, and the required aircraft return safely.** Every link in that explanation should be visible and testable.
