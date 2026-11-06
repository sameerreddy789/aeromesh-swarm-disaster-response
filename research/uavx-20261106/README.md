# AeroMesh UAV-X research

**Start with [the findings and ranked improvement plan](RECOMMENDATIONS.md).** It explains what to change, which algorithms fit the mission, how to build the simulation and how to demonstrate it against the published rubric.

This folder contains 91 source records across four ledgers, 40 isolated mechanics trials, a 240-case radio sensitivity study, an 80-case feedback study, an actual received-image audit and a checked interactive demo concept. References overlap and reading depth varies; the experiments are not hundreds of independent mission reliability trials.

These are recommendations and research fixtures, not newly implemented production features or claims of IIT Bombay qualification. The previously verified physics work is separately saved in `submission-artifacts/physics-20261106` at the repository root. Runtime code remained unchanged during this research.

**New full nominal result: 14/15 checks.** All ten PoIs were reported and all 30 aircraft landed at 2514 s, but 19.8 s of lost connectivity fails the strict profile. The evidence itself passes verification. See [the result](integrated-nominal-summary.json) and [timeline](integrated-nominal-timeline.png). This is the first concrete fix in the recommendations.

The supplied problem statement remains the governing task. Official sources, scholarly/technical evidence and design inference will be identified separately. Current code and measured logs establish what AeroMesh actually does.

## Main deliverables

| Read or open | Purpose |
|---|---|
| [Recommendations](RECOMMENDATIONS.md) | Priorities, acceptance tests, simulation tiers and demo storyboard |
| [Requirements and gaps](requirements-and-gaps.md) | Requirement → code → evidence → remaining work, including the original eleven issues |
| [Physics and controls](physics-control-research.md) | Equations, simulator comparison, calibration and numerical probes |
| [Swarm and network](swarm-network-research.md) | Allocation, relay geometry, protocols, recovery and resource feasibility |
| [Architecture and sensing](architecture-observability-research.md) | True state, received state, commands, sensor discovery and data provenance |
| [Evaluation design](evaluation-design.md) | Baselines, adversarial cases, statistics and reproducibility |
| [Visual/demo research](visual-demo-research.md) | Information design, truthful displays and jury explanation |
| [Interactive concept](demo-wireframe.html) | Offline **mock-data topology schematic**, not a live simulator |
| [Actual image sheet](received-image-quality.png) | Received procedural camera images, separate from the mocked concept |
| [Radio sensitivity](radio-sensitivity-01/README.md) / [feedback](radio-feedback-01/README.md) | Raw results, scripts, figures and exact limitations |
| [Full nominal physics summary](integrated-nominal-summary.json) / [verification](integrated-nominal-verification.json) | New full-map result: 14/15; 19.8 s connectivity failure retained |
| [Nominal outage diagnosis](integrated-nominal-connectivity-diagnosis.md) | Five outage windows, measured range failures, repair proposal and tests |
| [Evidence and reproduction](EVIDENCE.md) | Saved summary provenance, local raw traces and commands |
| [Source catalog](SOURCE_CATALOG.md) | Clickable sources, reading scope and full claim ledgers |
| [Research audit](research-audit.json) | Local links, source IDs, JSON, experiment counts and unchanged hashes |

Requested work started 6 November 2026 at 02:28:39 UTC. The work log records observed timestamps and a clock discontinuity; elapsed wall time is not presented as uninterrupted active research or compute time. No large city/data download, physical flight, organizer communication or submission was performed.

## Coverage inventory

1. Challenge authority, exact metrics, submission/rubric uncertainty, feasibility and resource assumptions.
2. Translational/rotational dynamics, quaternion frames, inertia, motor mixing, saturation, lag and numerical integration.
3. Aerodynamic drag, rotor thrust/torque, ground effect, downwash, wind/turbulence and contacts.
4. Controllers: cascaded PID, geometric control, MPC, control barrier functions, braking and disturbance rejection.
5. State estimation, sensor models, latency, timing, stale observations and flight authority.
6. Energy/power models, recharge, return reserve, spare allocation and fleet sizing.
7. Mission coverage, emerging PoIs, priority/preemption, allocation/auction/matching, planning under deadlines.
8. Collision avoidance, geofences, obstacle/terrain handling, safe altitude and takeoff/landing corridors.
9. Connectivity graphs, relay placement, node-disjoint paths, articulation, distributed state and reconfiguration.
10. RF propagation, interference, MANET/FANET routing, bandwidth, queueing, retransmission and MAC realism.
11. Image payloads, compression, reliable reassembly, priority/QoS, deadline and age of information.
12. Recoverable versus total faults, loss detection, fault injection, recovery metrics and degraded-mode honesty.
13. Simulator selection, PX4/SITL/HIL boundaries, multivehicle performance, deterministic replay and storage.
14. Verification: analytic checks, timestep convergence, Monte Carlo seeds, ablations, baselines, stress/counterexamples.
15. Visual perception, search footprints, false/missed detections and sensor provenance.
16. Visualization: fleet roles, physical scale, eagle/close views, task/route graphs, trail/age/error displays.
17. Demo story, scenario selection, failure reveal, evidence overlays, narration, video pacing and live fallback.
18. Reproducibility, installation, source/data licensing, dependency pinning and delivery readiness.
19. Prioritized improvement proposals, dependencies, expected gain, engineering cost and acceptance tests.
20. Evidence limits, uncertainty, publication bias, inappropriate algorithm complexity and decisions to defer.
