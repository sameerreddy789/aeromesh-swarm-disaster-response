# Visualization, evidence and demonstration research

Status: visual research finalized 6 November 2026; recommendations only, with simulator implementation unchanged. Source ledger: [visual-demo-sources.json](visual-demo-sources.json). References V01–V16 below identify 16 inspected publications, official documentation or published competition rules. Source-cache integrity and the offline concept have been checked. A small [offline mock interface](demo-wireframe.html) and a separate [actual received-image audit](received-image-quality.png) are available. The final full nominal physics result is 14/15 checks, with 19.8 s connectivity failure; see [the measured timeline](integrated-nominal-timeline.png) and [diagnosis](integrated-nominal-connectivity-diagnosis.md).

The useful next visual investment is a clear account of what the swarm is doing and what the ground station has received. The current procedural city-free view already proves that physical quadcopter bodies can be observed in an eagle view and close-up. More scenery would not resolve ambiguous status, incomplete payload provenance or unclear role changes. The city remains deferred because storage is constrained.

## Authority and scope

The supplied UAV-X problem statement governs. Its page 5 gives these weights: mission completion 25%, communication resilience 25%, autonomous relay/role management 20%, fault recovery/reconfiguration 15%, safety/collision avoidance 10%, innovation/technical merit 5%. It lists completion time/rate, priority-weighted mission score, delivery ratio, latency, availability, downtime, recovery time, reallocation, collision count and minimum separation. It also forbids exhausted charge and geofence departures. The report template makes communication central and asks for a demonstration link. See the document provenance and corroborating official page in [R01, R02 and R05](SOURCE_CATALOG.md); full supplied-document extracts remain local.

Those are requirements. The layout, storytelling sequence and proposed usability thresholds in this note are design recommendations. DARPA SubT and SUAS rules are useful published examples of evidence practices, not extra IIT Bombay rules. No assertion about unpublished jury preferences is made. The supplied material does not establish a mandatory video length in the inspected passages; any duration proposed here is provisional. Stage 2 organizer scenarios and standardized logs must take precedence when supplied.

## What the existing viewer actually establishes

The inspected code is `visualization/physics_viewer.py`; SHA-256 at this audit is `6101537bd19dbdc74c71047ce47816272bf93435b61dad02c52a2126130fe566`.

| Existing feature or evidence | What it establishes | Important boundary |
| --- | --- | --- |
| Procedural quadcopters attached to Bullet bodies | Bodies, arms, motors, props and landing legs follow simulated rigid-body motion | Decorative rotor animation is not measured RPM; geometry is not an airframe qualification |
| Eagle, orbit, pan, zoom, selected-drone close-up, full/work-area fit | Overview and detail can be inspected without external city assets | Close-up does not explain mission outcome by itself |
| Separate distance-scaled markers | Drone bodies retain physical scale while overview markers remain visible | Marker scale is symbolic, not a larger collision envelope |
| Real router graph edges | The displayed links come from the model's graph | Full topology is cluttered; graph reachability is not complete message receipt |
| Separate mission/physics clocks and requested/achieved playback rate | Slower rendering does not masquerade as faster physical progression | Wall heartbeat freshness and GCS acquisition/receipt age still need distinct treatment |
| Explicit uncalibrated-physics label | The display acknowledges model limits | It is not full aerodynamics, a flight-qualified estimator or physical BVLOS validation |
| Native/headless comparison of the 60 s observer fixture | All 12 manifest artifacts were byte-identical for the paired runs | A limited non-interference check, not proof across every future UI change |

The paired evidence is in `evidence/physics-viewer-native-headless-comparison.json`, `evidence/physics-viewer-final-native-01` and `evidence/physics-viewer-paired-headless-01`. Camera-operation checks preserved all 30 physics observations. The independent sensing PNG also stayed unchanged. The native 60 s run took 81.672 s wall time versus 28.75 s headless; a render-only test was about 29.1 FPS while a captured native mission view was about 10 FPS under concurrent load. These are observed configurations, not a general frame-rate guarantee.

The separately inspected full **compact development** physics run completed at 623.5 s with all 15 local verifier checks passing, 10 PoIs completed, 10 distinct UAVs launched, peak 8 active and 20 never-launched grounded spares. It is team-authored and not organizer-issued. The report explicitly retains uncalibrated dynamics, a spherical collision envelope, simplified endurance, sampled safety and an unqualified full physics matrix. The later full nominal physics run completed ten PoIs and landed all 30 at 2514 s, with 28 sampled active at peak, but failed continuous connectivity for 19.8 s. Its 14/15 result must remain visible when presenting that run. The older kinematic matrix and its two strict failures remain separate evidence.

### Confirmed display and evidence gaps

1. **Truth and received knowledge are mixed conceptually.** The viewer reads `sim.state`, `sim.physics.observe` and the live router graph. It is a simulator observer, not the knowledge available at the ground station after communication delay/loss. Label it accordingly.
2. **Telemetry does not yet carry vehicle state.** `simulation/resilient_uavx.py` sends `dict(kind='telemetry')` with a nominal 256-byte payload. The compact packet trace confirms this. A received-state view cannot reconstruct a legitimate current vehicle pose from that payload. Looking up the current truth position when a marker packet arrives would manufacture communication performance. A versioned state payload and receive-only store are prerequisites.
3. **AIRBORNE currently means armed.** `_update_hud` counts `f.armed`. An armed vehicle can still be on the ground or completing touchdown. Use explicit arming, measured airborne, landed and disarmed concepts, with declared thresholds/hysteresis.
4. **GCS LINK is instantaneous truth connectivity.** Its numerator counts armed vehicles with `connected_to_gcs`. That is neither a freshness measure nor a delivered-image count. The denominator is also armed vehicles.
5. **Role/failure distinctions rely heavily on color in the overview.** Selected detail includes role text, but overview redundancy should improve. A separate fault/status dimension is needed because a relay can also be returning or stale.
6. **Network explanation is not selective.** Drawing all available edges does not answer which path delivered this image, which relay is critical, or whether an observed backup path is still valid.
7. **Energy is displayed in seconds.** Preserve that honest model unit. Do not relabel it calibrated state-of-charge, watt-hours, voltage or a physical battery prediction.
8. **Image delivery evidence exists but is not the main visual story.** Capture and receipt records include acquisition time, receipt time, byte length, SHA-256, capture ID and chunks. Those should be inspectable as a complete chain.

## What the research supports

### Overview, then exceptions and detail

Shneiderman's task taxonomy [V03] motivates overview, zoom, filter, relate, history and details on demand. It is a starting point rather than a universal layout prescription. The human-swarm survey [V01] explains why individual supervision can grow with robot count and why autonomous coordination enables useful task-level interaction. Together these support a stable eagle view with a selected route, a small fleet-state summary and one selected UAV detail pane. QGroundControl [V04] provides a concrete operational precedent for map/video switching and focused instruments; it does not prove that its layout is optimal for AeroMesh.

Nunnally et al. [V02] tested 25 participants controlling 30 virtual robots. Timely aggregate information performed comparably to full individual updates on a simple foraging task; very restricted updates performed worse. This is unusually relevant to our fixed fleet size, but it must not be oversold: nonsignificance is not equivalence, the environment was simplified, the authors report a possible ceiling effect, and the task did not require diagnosing an individual aerial relay. Recommended interpretation: summarize ordinary fleet state while making exceptions and selected individuals easy to inspect. Never hide an exhausted or disconnected vehicle behind an average.

### Freshness is part of the information

ROS 2's clock design [V05] distinguishes simulation time from wall/steady time and explicitly addresses pause and backward jumps. Its QoS documentation [V06] separates expected publication interval, lifespan and publisher liveliness. Foxglove's 3D documentation [V08] distinguishes message timestamp from receive/log time and requires transforms valid for the object's timestamp. These concepts directly expose a dashboard trap: a continuously moving icon can imply knowledge the communication system never delivered.

The proposed display needs four separate quantities:

| Quantity | Definition | Correct use |
| --- | --- | --- |
| Simulation observation age | current simulation time minus acquisition time, within the same run/epoch | How old the last known position or image is in the mission |
| Transport delay | receipt time minus acquisition time, using an explicit clock domain | How long the evidence took to reach the GCS |
| Wall heartbeat age | current monotonic wall time minus last renderer/stream update | Whether the live display process has stopped updating |
| Playback factor | simulation elapsed divided by wall elapsed, with pauses identified | How the recording/view is being presented |

A paused simulation should not accumulate fictitious simulation age, but a disconnected live viewer can accumulate wall staleness. A replay seek must create/reset an epoch so that an image from the future is not shown as current and negative ages are not silently clamped away. Do not display a received object's transform using current truth geometry. If timestamps cannot be aligned, report unknown age rather than a precise-looking value.

The age-of-information survey [V16] formalizes recipient age as `current_time - generation_time_of_freshest_received_update`. A low delivery latency does not guarantee fresh state: with deterministic 0.1 s delay, a stream sampled every 0.1 s has steady-state mean age 0.15 s, while a stream sampled every 10 s has mean age 5.1 s. Both can truthfully report the same 0.1 s delivered-packet latency. This derived example assumes constant delay, no drops and steady state; it is not a measured AeroMesh result. An out-of-order older arrival must not replace a fresher observation just because it arrived last.

### Explain the decision that actually happened

NISTIR 8312 [V14] separates meaningfulness, explanation accuracy and knowledge limits. A correct outcome can have a false explanation, and a faithful explanation can describe a poor decision. Therefore, explain role changes using actual event reasons and recorded constraints. For example, `return_started` already carries `formation_energy_reassignment`; `formation_committed` records deployment, withdrawal and required-energy budgets. A human-readable label can clarify those fields without inventing unlogged causal alternatives.

If the system does not record why a particular candidate was rejected, the UI should say that detailed candidate reasoning was not recorded. It should not infer that UAV A was selected because it was optimal, nearest, safest or most reliable merely because the resulting trace looks plausible. A future explanation record should contain the selected task/role, applicable priority, feasibility tests, resource reserve, candidate exclusions and the planner version.

### Show the report arriving, not only the scout arriving

The SubT rules [V12] score valid reports at a command post, require the modeled RF communication path in the virtual competition, and use logs for replay/adjudication. SUAS 2021 [V13] explicitly separates mission demonstration, design and readiness evidence and asks teams to discuss full-mission results and flaws. These are strong examples of why an attractive animation cannot replace evidence; their exact numbers are not UAV-X requirements.

For UAV-X, show acquisition → capture → fragments offered → delivered/reassembled image → hash-verified GCS receipt. The first delivered fragment is not complete image delivery. A route line is not a packet trace. A file existing in `sensing/captures` is not a file received at the base. The source image, received image, capture ID, byte count and digest should agree before the UI marks a report complete. Keep the original detection/report deadline; retrying must not silently create a fresh deadline.

The compact run gives an inspectable example: PoI 0 was detected at 23.5 s and received at 25.0 s, with 1.5 s recorded delay, 1,665 bytes and 4 chunks. This is a specific procedural 128×128 PNG, not evidence that a large disaster photograph or video stream can meet the same delay. Use this example only with its run ID and exact scope; the later wireframe uses mock data and must remain distinct.

### Actual received-image quality audit

The read-only [audit JSON](received-image-quality.json) verifies all ten compact-run received files against their capture files and recorded SHA-256 values. All match; the total PNG content is 14,710 bytes. The [image sheet](received-image-quality.png) shows native-size PoIs 0, 3, 6 and 9 and a nearest-neighbour 4× enlargement of PoI 0. The sheet is derived from actual received files; it adds no image detail. Observation records are matched by PoI identity, not their chronological list position.

At PoI 0, the recorded ideal downward camera is at z = 40.125610 m, with 60° vertical field of view, 128 pixels and the flat ground plane at z = 0. The pinhole calculation `footprint = 2 × height × tan(FOV / 2)` gives 46.333064 m; dividing by 128 gives 0.361977 m/pixel. A flat 0.5 m feature would span about 1.381 pixels. This is a geometric illustration, not an empirical detector threshold; elevated objects have different projection scales and occlusions. The nominal calculation at exactly 40 m is approximately 46.19 m and 0.361 m/pixel.

The inspected frames show the renderer's roads, walls, roof slabs, debris, tarpaulin and damaged-vehicle geometry. They are low-detail procedural imagery with flat colors. They substantiate capture and transport of a simulated view, not real-world search or survivor recognition. The ideal stabilized world-down camera does not establish motion blur, noise, exposure, rolling shutter or physical-gimbal performance. Future image-resolution/codec choices should be evaluated against useful task detail and their actual serialized network cost, not only a more attractive enlarged preview.

The router's payload budget is converted into a serialized envelope plus padding; it is not automatically UDP/IP/MAC physical wire cost. A new state payload should charge the actual modeled serialization and explicitly name any additional transport overhead assumed.

### Credible graphics and safety evidence

PX4 Flight Review [V09] interprets controller behavior by mode and compares relevant estimated values against setpoints. For AeroMesh the comparable diagnostic is commanded reference versus actual simulated motion, with thrust, saturation and contact state when recorded. It would be inaccurate to label direct simulator truth an estimated GPS/IMU state. A short physical close-up can show roll/pitch, motion and thrust, but it must retain uncalibrated-model context.

NIST response-robot methods [V10] emphasize measured capabilities across communication, energy, sensing, mobility and safety. NIST's simulation guidance [V11] explicitly preserves the need for real-system testing outside simulation. That supports a small set of clear, measurable fixtures while city assets remain deferred. The realistic-looking aircraft should not imply physical-airframe qualification, collision-free continuous time, RF field validation or visual recognition of real survivors.

WCAG 2.2 [V07] supports redundant encoding and adequate contrast: roles can use shape/letter, health can use outline/status text, and links can use line patterns. Its relevant values are a useful design baseline (normally 4.5:1 for text and 3:1 for meaningful non-text marks), not a claim that the native viewer is formally WCAG compliant. Verify the exported video, because compression and downscaling can erase thin links and small text.

## Proposed information architecture

The fixed fleet is **30 total, including reserves**. Show the number explicitly. Two useful summaries should not be collapsed into one:

- Mutually exclusive lifecycle states: ready on ground, launching, airborne, returning/landing, charging, failed/unavailable, unknown/stale where the view only has received knowledge.
- Assigned mission roles: scout, relay, standby, or unassigned; role and lifecycle can coexist.

Each UAV must appear exactly once in a lifecycle partition for a given view. If “returning/landing” is included in an airborne count, label that overlap; do not sum overlapping categories to 30. Ground contact, launch-height reference, vertical speed, landing confirmation and disarm acknowledgement should determine measured state. During ambiguous transitions show “launching” or “landing”, not a guessed final state.

| View region | Default content | Available on selection | Provenance required |
| --- | --- | --- | --- |
| Header | run ID, scenario provenance, truth/received mode, pause/live/replay, mission/physics time | source hash and fidelity limits | run manifest and selected clock |
| Eagle map | geofence, announced tasks, GCS, all fleet markers, selected route | graph, selected trail, safety envelope | measured truth or explicitly received snapshots, never mixed invisibly |
| Fleet strip | 30 total and lifecycle counts; exception count | vehicle list sorted by actionable state | measured/received lifecycle state and freshness |
| Selected report | original observation, source UAV, capture/receive time, bytes/chunks, integrity and deadline | actual received image and packet chain | receiver completion record and saved received file |
| Selected UAV | role, mode, remaining endurance seconds, next action | measured velocity, thrust, command/observation age | physical telemetry plus planner events |
| Event timeline | task appearance, selection/preemption, relay handoff, fault detection, receipt, landing | raw event and relevant trace interval | typed event records with run/sequence IDs |
| Safety band | sampled min separation, speed/altitude limits, geofence and energy exceptions | sample interval and engine contact extrema | measured safety artifacts; unknown if unavailable |

The selected route should initially show the path associated with the selected message/receipt, with direction and a timestamp. A separate “current model route” can be shown when needed. A backup path needs evidence of independence at its observation time. Do not extrapolate a five-second protection sample into an unqualified continuous guarantee. Full topology is a diagnostic toggle and should remain accessible for relay planning questions.

## Priorities, cost and acceptance checks

These are rough engineering estimates for one developer familiar with the code, not delivery promises. They assume the existing renderer and logs; none require a city download. Producer changes need a new source snapshot and fresh qualification, so they should be grouped deliberately after this research.

| Priority | Proposal | Dependency / estimated effort | Acceptance check |
| --- | --- | --- | --- |
| P0 | Explicit observer-truth label; correct armed/airborne/landed labels | Read existing measured state; roughly 0.5–1 day including regression | Armed-on-ground and landing transitions are not falsely called airborne/landed; all 30 partition correctly |
| P0 | Actual versioned telemetry payload and receive-only state model | Router payload/byte accounting, snapshot fields, sequence/epoch semantics; 2–4 days | Drop/delay/reorder packets and confirm received view never moves with undelivered truth; actual bytes are charged |
| P0 | Receipt inspector with acquisition/receipt age, complete chunks/hash and original deadline | Receiver events + received-file validation; 1–2 days | Corrupt/missing/late fragment never yields green complete; no capture-file shortcut |
| P1 | Selected current route vs historical delivery path, optional full graph | Per-packet hops and timestamped topology; 1–2 days | Path identity remains tied to the selected evidence and is not relabeled after rerouting |
| P1 | Actual-event explanation and timeline | Event schema and stable reason vocabulary; 1–2 days for existing reasons | A replayed event renders the same reason; absent reason is explicitly absent |
| P1 | Role/status shapes, readable text and semantic zoom | Existing screen markers; 0.5–1.5 days | Greyscale and final-resolution video preserve selected source, relay, failed/stale and grounded states |
| P1 | Read-only selected physics diagnostics | Targets/actual state with units and sampling metadata; 1–2 days | Plot state and target from the same clock/epoch; UI controls do not change evidence |
| P2 | Small overview/detail inset and camera presets | Observer only; 1–2 days | Horizon/frame remains understandable; returning to overview preserves orientation; render overhead measured |
| P2 | Recording index, small rehearsal video and evidence package links | Final qualified run and storage budget; 0.5–1 day | Video run ID/seed/source hash matches bundled evidence and playback factor is visible |

Freshness cutoffs should be configured and justified by actual telemetry rate and mission needs. This research does not declare an arbitrary 2 s or 5 s threshold an organizer rule. At every threshold boundary, test late arrivals, repeated timestamps, packet duplication, sequence wrap/reset, fault recovery, pause and backward replay.

## Provisional demonstration storyboard

An edited video explains a selected run; the live jury demo must execute the unseen scenario supplied at the finale. They need different pacing and cannot substitute for one another. The following approximately four-minute explanatory cut is a recommendation pending the actual video-duration instructions.

| Segment | Evidence shown | Main supported criterion | Claim boundary |
| --- | --- | --- | --- |
| 0:00–0:20 | Fleet 30 including spares; map, geofence, known tasks, run ID and uncalibrated physics | Scope / reproducibility | Do not call a known team fixture an unseen organizer scenario |
| 0:20–0:55 | Task selection and measured scout/relay deployment; briefly select a real body close-up | Mission / role management | Physical appearance is illustration; logged motion and events are evidence |
| 0:55–1:40 | One image from detection through actual fragments and complete GCS receipt, delay and hash | Communication / mission | Show received file; distinguish graph path from delivered packets |
| 1:40–2:30 | A separately qualified fault/reconfiguration case, detection delay, replacement and resumed reporting | Resilience / recovery | If only nominal/compact physics is qualified, use a clearly separate qualified kinematic clip or omit a physics recovery claim |
| 2:30–3:10 | Emerging priority task and recorded preemption/reallocation reason; reserve/charging state | Role management / priority handling | No invented optimality claim; show the actual event reason |
| 3:10–3:40 | Return, measured touchdown and disarm; min separation and limits with sample scope | Safety / mission | A landed flag alone is insufficient; retained sample limits stay visible |
| 3:40–4:00 | Outcome table, scope/failed cases and where to reproduce the run | Reproducibility / technical merit | Local passing checks are not an official challenge score or deployment certification |

The live demonstration should begin from a preflight checklist that is visible but short: source/run identity, scenario provenance, fleet count and fidelity mode. Load only the supplied scenario; preserve hidden future locations from observer knowledge until scenario policy allows revelation. Keep a default map/receipt view stable while the mission runs. Use close-up only briefly and on demand. If execution fails, preserve the failing trace and label the failure. A recorded qualified run is a clearly labeled explanatory fallback, never a substitute passed off as live success.

Published precedent: the UAV-X supplied document expressly calls for an unseen finale execution; SUAS [V13] separates readiness presentation from mission demonstration; SubT [V12] uses receiver reports and run logs for validation. None of these sources establishes that cinematic camera motion earns marks. The recommended time allocation follows UAV-X's published emphasis on communication, role management and recovery, rather than an assumed jury taste.

## How to evaluate the display itself

A small formative team study can find obvious comprehension problems, but it is not a statistical population study. Compare the current graph-heavy view with the proposed selected-route/receipt view using counterbalanced scenario order and recorded clips from equivalent runs. Do not give one condition easier faults or more exposure. Record task completion time, correctness and confidence separately.

Useful tasks include: identify which vehicle has stale GCS telemetry; name the source and relay chain of the selected report; tell whether the full image reached the ground station; distinguish a returned/disarmed vehicle from a failed one; explain the logged reason for a relay handoff; and state which safety fact is sampled versus continuously checked by the engine. An incorrect high-confidence answer is an especially useful failure signal.

Suggested formative targets (team-set, not published standards): no false interpretation of a partial image as complete; no miscount of 30 total; correct stale/current distinction in every injected stale case; and faster route identification without worse fault detection. Choose actual timing targets after a baseline, rather than inventing a universal five-second limit. NASA-TLX [V15] can supplement objective tasks with subjective workload if the manual is followed; a TLX score is not mission success or situation-awareness accuracy.

Re-run the existing observer non-interference comparison after integration. Test camera moves, toggles and playback controls against unchanged mission artifacts. If a visual observer can pause or throttle simulation, that effect must remain explicit and measurable. Capture final-resolution screenshots and the encoded video to inspect contrast, text, thin lines, overlapping labels and frame-rate behavior. No external participants have been contacted and no human-subject evaluation is claimed.

### Reviewable offline concept

[demo-wireframe.html](demo-wireframe.html) is a small dependency-free concept with three illustrated snapshots, truth/received modes, selected UAV state, complete/partial image status, selected route, optional topology, zoom, event timeline and safety scope. Every mission value and thumbnail in this interface is explicitly mock data. It links separately to the actual received-image audit. It does not read the simulator or control aircraft.

The [concept checks](demo-wireframe-checks.json) record 30 markers, no external runtime dependencies, hidden future timeline events, a partial image that remains unverified, local timeout detection distinguished from received events, working graph/zoom/fit controls, actual pointer selection of UAV 17, and 390 px layout without horizontal overflow. The map is explicitly schematic rather than metric; the 21 grounded markers are a roster rather than pad locations, and no metric scale or geofence claim is made. The mock safety limit is 20 m, matching the supplied sample. Desktop and mobile screenshots were visually inspected; the selected topology button's contrast was repaired. The mobile map is necessarily small, so the jury-demonstration target remains a desktop display. These checks validate the illustration's interactions, not the proposed simulator integration or comprehensive accessibility.

Before submission, consolidate the judge-facing launch/evidence path around the current source and exact environment. Existing legacy documents with different branches, 20-UAV assumptions or untraced performance tables have provenance gaps; they should not silently become the authority for the 30-UAV physics demonstration. This observation is a documentation recommendation, not a claim that those historical results were fabricated.

## Deliberate deferrals and counterexamples

- A million-building city, heavy engine migration and more simultaneous camera streams remain deferred. They consume storage/render budget and do not repair receiver-state provenance.
- Average fleet health can hide one depleted relay, and centroid-based views can hide a disconnected subgroup. Aggregate success in [V02] does not justify suppressing exceptions.
- A predicted icon may help comprehension but can overstate knowledge. Show last received position by default; any prediction needs a separate style, horizon and validated uncertainty model.
- More labels and alarms can overwhelm the selected task. Use a small exception list and stable priorities, with raw diagnostics accessible on demand.
- Smooth interpolation can conceal packet loss or a fault. Never interpolate operational received state without identifying it as a rendering estimate and preserving age.
- A short successful video is selection-biased. The evidence appendix should show the complete scenario matrix, retained strict failures and run conditions.
- Physics, sensing and communication each have their own fidelity limits. A realistic quadcopter model does not validate a radio, a camera detector or an electrical battery.

## References used in this pass

Full authors, dates, retrieved UTC, inspected sections, supported claims, confidence and access limitations are recorded in [visual-demo-sources.json](visual-demo-sources.json).

- V01: [Kolling et al., human-swarm interaction survey](https://eprints.whiterose.ac.uk/id/eprint/90476/8/Human%20interaction%20with%20robot%20swarms.pdf).
- V02: [Nunnally et al., bandwidth/localization user study](https://faculty.cs.byu.edu/~mike/mikeg/papers/06377723.pdf).
- V03: [Shneiderman, visualization task taxonomy](https://www.cs.umd.edu/~ben/papers/Shneiderman1996eyes.pdf).
- V04: [QGroundControl Fly View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html).
- V05: [ROS 2 Clock and Time](https://design.ros2.org/articles/clock_and_time.html).
- V06: [ROS 2 QoS documentation source](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).
- V07: [WCAG 2.2](https://www.w3.org/TR/WCAG22/).
- V08: [Foxglove 3D documentation](https://docs.foxglove.dev/docs/visualization/panels/3d).
- V09: [PX4 Flight Review](https://docs.px4.io/main/en/log/flight_review).
- V10: [NIST response-robot test methods](https://www.nist.gov/el/intelligent-systems-division-73500/standard-test-methods-response-robots).
- V11: [NIST aerial simulation guidance](https://www.nist.gov/el/intelligent-systems-division-73500/standard-test-methods-response-robots/aerial-drone-tests-8).
- V12: [DARPA SubT final rules, May 2021](https://www.darpa.mil/sites/default/files/SubT_Challenge_Finals_Rules.pdf).
- V13: [SUAS 2021 rules](https://suas-competition.org/s/auvsi_suas-2021-rules.pdf).
- V14: [NIST Four Principles of Explainable AI](https://nvlpubs.nist.gov/nistpubs/ir/2021/NIST.IR.8312.pdf).
- V15: [NASA Task Load Index](https://www.nasa.gov/human-systems-integration-division/nasa-task-load-index-tlx/).
- V16: [Yates et al., Age of Information survey](https://arxiv.org/abs/2007.08564).
