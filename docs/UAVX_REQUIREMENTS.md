# UAV-X requirements and assumptions

Source authority: user-supplied `UAV-X__Resilient_BVLOS_Swarm_Challenge_1_compressed.pdf`
(pp. 3-5), `sample scenario for GC 1.pptx` (slide 1), and
`Report Template  - Grand challenge 1.pdf` (report structure). These describe
requirements; they do not authorize external submissions. No organizer-issued
Stage 2 scenario files or standardized log schema were supplied.

| Supplied requirement | Implementation / evidence |
|---|---|
| 1000 m x 1000 m operational area | PoIs constrained to x=75..1075, y=-500..500 |
| 45-minute mission; return all UAVs to start | 2700 s hard stop; completion requires all surveys received and every operational vehicle settled/disarmed at its own home pad; failures cannot count as landed |
| Maximum flight time 20 minutes | Per-sortie 1200 s hard budget, measured accumulated armed time, explicit exhaustion failure |
| Maximum radio range 100 m | 3D Euclidean link cutoff in MeshRouter, refreshed each tick |
| Height maximum 100 m; speed maximum 5 m/s | Proposed segments checked before motion, observed maxima reported |
| Minimum vehicle distance 20 m | Simultaneous swept-segment closest approach, including ground vehicles; no launch exemption |
| Detection-to-center report maximum 10 s | Immutable first detection timestamp; scenario-camera reporting requires complete, hash-verified image reassembly at the GCS; missing/late delivery fails |
| 10 PoIs with random position/time | Fixed-seed sampling of positions and spawn times; future tasks prevent early mission completion |
| Dynamic relay assignment and reconfiguration | Radial relay chain, stable slot identities, energy-based replacement, measured arrival/connectivity before handoff where reserve allows |
| Emerging high priority regions | Higher-priority spawned task can preempt current task; prior unfinished task stays pending |
| Failures and communication outages | Validated scenario schedules inject link outages, complete radio outages and permanent vehicle failures; actual resolved targets/times are recorded |
| Measured performance and evidence | Packet offers/drops/deliveries, elapsed communication downtime, trajectories, landing events, checks with explicit failures |

## Current scenario extension

The required mission limits above remain unchanged. The default live launcher
now selects the scenario-driven profile, whose example declares 60 UAVs, 44 m
protected relay slots, 22 m planning separation, finite 128 KiB/s shared per-node
TX/RX service, bounded queues and Panda3D rendered camera imagery. Realized tasks,
charging resources and faults are saved in each scenario. Details and remaining
qualification limits are in [resilience notes](UAVX_RESILIENCE.md). These resources
are team assumptions, not organizer approvals.
The validated scenario fleet range is 1..72, subject to staging geometry and pad
constraints. The earlier profile's smaller layout is described separately below.

## Earlier footprint-profile assumptions (retained for reproduction)

- 30 vehicles by default, configurable 1..36 for this launch layout. Team size
  is not UAV count. The maximum map distance needs roughly 14 hops with 85 m
  planned spacing, so five vehicles cannot span every distant PoI. Thirty provides
  a second cohort for relay replacement/recharge; an 18-vehicle development run
  was insufficient for full coverage. Fleet size is an explicit resource assumption.
- GCS=(0,0,0), outside the disaster square. Individual home pads use 25 m
  spacing. The flight geofence x=-75..1075, y=-500..500 includes staging and
  approach. The supplied slide does not specify that geofence polygon or pad
  layout; these coordinates are explicit test assumptions.
- Centralized full-state simulation planner, not a distributed consensus
  deployment. Spawned tasks are announced to the planner; a synthetic camera
  footprint records a detection only within 5 m horizontally at 30..70 m height.
  Survey dwell is 30 s. No survivor-recognition accuracy or real imagery is claimed.
- First task spawns at t=0; remaining random spawn times are in 60..1800 s. The
  last generated task has priority 5; others priority 1. Parameters and seed are
  retained in evidence. This is a development benchmark, not hidden jury data.
- Kinematic point vehicles; no thrust, acceleration, wind, terrain, or vehicle
  volume dynamics. 20 m separation applies to centers. Return altitude is 90 m;
  relay nominal altitude 40 m, replacement approach 65 m.
- Radio within range has 2% independent loss per attempt by default. Routing
  is global shortest path; every queued packet gets at most one hop per 0.5 s
  tick. Parallel service is unbounded: bandwidth contention and RF propagation
  are not qualified. Packet retry/queue time remains in measured latency.
- Battery is a conservative constant endurance budget; recharge from empty
  takes 720 s. This is not a calibrated electrochemical/airframe model. Reserve
  checks include travel, survey/relay dwell, landing and the overall deadline.
- A permanent failed aircraft freezes in this kinematic fault model. It remains
  a separation obstacle and fails the all-vehicles-landed check. Reporting that
  outcome honestly is required; it is not evidence of fault-tolerant physical landing.
- Connectivity-preserving motion is best effort. Safety/return reserve can take
  precedence; downtime is recorded and continuous-connectivity check fails.

## Garuda inputs

Design references were inspected in `sameerreddy789/Garuda_OS`: measured landing
in `garuda/flight/pas_client.py`, target-plus-home energy feasibility in
`garuda/digital_twin/energy_model.py`, paired observations in
`garuda/perception/visual_worker.py`, and evidence timing in
`garuda/flight/shot_recorder.py`. No Garuda source or simulator assets were copied.
The new safety helpers include dwell/deadline terms absent from the original
single-UAV energy helper. Time is simulation time throughout.

The Garuda single-client scene reload is unsuitable for one client per swarm
vehicle. The old city under D:/Aatonovaz/Garuda is not used. Simulated scene-camera
capture and network delivery are implemented in AeroMesh. Physical backend
integration and learned perception remain unqualified. The supplied material
does not establish a 45-minute wall-clock video as a submission requirement.

## Submission boundary

The supplied PDF specifies email submission, unlike the conflicting Forms URL.
No submission was sent. The prepared report and demo remain a technical draft.
Team identity, qualification, organizer log conversion, official scenario
validation and final submission review remain outstanding.
