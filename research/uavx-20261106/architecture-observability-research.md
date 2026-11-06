# Architecture, information access and sensing — research notes

Status: research and proposed changes only. Production reference is commit `af544e505af51848f5e3a013a52b300f0cd4bfb9`. The completed unchanged-source full nominal physics run passed 14/15 checks, failing continuous connectivity for 19.8 s despite completing ten PoIs and landing all 30 at 2514 s. See [the result](integrated-nominal-summary.json) and [diagnosis](integrated-nominal-connectivity-diagnosis.md). Fleet remains 30 including spares. No additional radio, sensor, charging hardware or aircraft is assumed.

## The most consequential architecture gap

The mission uses a centralized shared Python state. Centralized coordination is allowed by the supplied template [R05]; it is not by itself a rules violation. The qualification issue is **what information and command delivery this central controller is permitted to have during a network failure**.

Inspection of `simulation/physics_uavx.py` shows:

- `_sync_measured()` obtains every Bullet pose and velocity directly, then updates every UAV's shared mission state and `observed_at_s` on each control interval.
- `_safe_commands()` computes pairwise avoidance from those simultaneous true positions, including distant and radio-isolated aircraft. This is a useful ideal-observation safety baseline, not a demonstrated onboard perception or received-telemetry controller.
- `_move()` directly invokes `physics.command()` for each aircraft. These commands do not pass through the mesh. Command expiry in the mechanics layer therefore does not, by itself, exercise mesh-related command loss in the integrated mission.
- `_reference()` uses isolated copies so that its kinematic movement cannot create physical motion or landing credit. This existing separation is a strength and should be retained.

Inspection of `simulation/resilient_uavx.py` shows:

- `_network()` rebuilds routing from the shared current state and explicitly masked fault graph.
- Telemetry packets use the payload `{'kind': 'telemetry'}` with a nominal 256-byte size. They demonstrate delivery service, but they do not contain the aircraft's serialized position/attitude/battery at the time of transmission.
- `_update_faults()` marks failed vehicles directly and `_radio_exclusions()` reads fault records. That is acceptable in the referee/fault-injection layer; using the same injected truth to trigger controller recovery does not establish realistic fault detection.
- The current viewer's `GCS LINK` is simulator graph connectivity. Its `AIRBORNE` count is `armed`, which is not identical to measured liftoff.
- Image retransmission stops when the shared `task.report_s` becomes non-null. There is no separate GCS acknowledgment packet returning to the sender, so the feedback path is idealized too.

**Recommendation:** present the existing view as a simulator/referee view. Before claiming communication-limited autonomy, create explicit information boundaries and tests. A delivered telemetry packet must not be joined to the aircraft's *current* true state to manufacture a received-state display.

## Proposed information boundaries

| Layer | Allowed information | Output / authority | Acceptance evidence |
|---|---|---|---|
| Referee and physical world | Full body state, true map, PoI spawn truth, actual injected faults | Advance physics; inject physical/channel disturbances; independent scoring | Truth logs and source/scenario hashes |
| Sensor adapter | True body state only to synthesize declared sensors | Timestamped observations with delay/noise/dropout; no mission score | Sensor configuration and observation traces |
| Onboard controller | Its own sensor estimates, cached mission intent, explicitly received neighbor messages | Bounded local motor/velocity control and declared lost-link policy | State/command age, reasons, actuator response |
| Network transport | Serialized packets and modeled channel/resources | Delay/drop/forward bytes; no hidden telemetry lookup | Send/receive/expiry records and byte accounting |
| Mission coordinator / GCS | Received telemetry, received reports, announced tasks if allowed | Timestamped mission intents sent through the declared transport | Received-state log, command receipt/acknowledgment |
| Viewer | Either referee truth or received GCS state, explicitly selected | Observation only | Label, timestamp, age and provenance on each display |

These are logical boundaries and need not initially become 30 operating-system processes or a ROS migration. A small deterministic message bus and per-vehicle state store can expose the failure semantics first. Process distribution is a later deployment concern.

ROS 2's official QoS documentation distinguishes expected publication deadline, message lifespan and liveliness lease [R09]. Reuse those semantics even if AeroMesh remains plain Python. A delivery guarantee cannot compensate for a physically absent route.

## State and command contract proposal

Each telemetry sample should include a schema version, vehicle ID, monotonically increasing sequence, acquisition time, publication time, frame/unit declaration, position/velocity, attitude, armed/contact status, energy estimate, current role/intent, estimate validity and covariance or bounded error when available. Reception time belongs to the receiving log, not to the transmitted sample.

The sender's actual serialized bytes must consume the declared radio budget. Keep images, telemetry and control traffic distinct so that a large image cannot invisibly block a safety heartbeat. Do not count an acknowledgment as completion of physical movement.

Each mission command should contain an ID, issue time, expiry, intended receiver, intent/parameters and applicable mission epoch. The onboard adapter should reject expired, duplicate, out-of-order or wrong-epoch commands according to a documented policy, and report acknowledgment separately from measured completion. The controller must log why it holds, returns or lands. Those policies must respect the existing resource limits; adding a second radio is outside the user's chosen scope.

Clock assumptions need to be explicit. One simulation clock can drive all components for deterministic tests. Inject bounded clock skew in robustness cases before claiming that age checks survive separate real computers.

For a later external MAVLink adapter, use the protocol's established signing and per-stream replay rules [R15] where applicable instead of inventing a new authentication format. This is separate from control-command expiry and from delivery reliability. The present PNG SHA-256 checks establish byte consistency, not sender authentication; no protocol migration is required just to improve the local simulator.

## Small falsification tests worth doing before adding more algorithms

1. **Cut only uplink telemetry.** Vehicle truth changes; GCS state must stop refreshing and age must increase. The truth view continues updating. A GCS command calculated from a newer unavailable pose is a failed test.
2. **Cut only downlink commands.** Old intent expires; the physical vehicle must follow its configured local behavior. The GCS must show unacknowledged intent, not completed action.
3. **Delay a stale command behind a fresh one.** An older command arriving later must not undo a newer mission epoch.
4. **Drop the fault announcement.** A radio fault is injected in the world but not revealed to controller code. Measure detection from missed heartbeats/observations, then decision, reconfiguration and restored service separately.
5. **Flood with imagery.** Charge actual payload bytes and verify stated telemetry/control deadlines or count misses. Merely retaining a topological path is insufficient.
6. **Disconnect a whole aircraft.** Preserve strict connectivity failure. Any eventual local return is a separate degraded-mode outcome and cannot retroactively make the outage disappear.
7. **Remove one neighbor observation.** Local collision avoidance must degrade according to its declared sensor/communication assumptions; it must not silently consult global truth.
8. **Move a task that has not emerged.** The policy's action should remain unchanged unless the official scenario explicitly announces future tasks. This is a metamorphic test for future-information leakage.
9. **Replay the same packet log into the GCS view.** The received-state display should reproduce without the physics engine and without access to future truth.
10. **Pause/zoom/change camera.** Mission artifacts should remain identical, extending the existing native/headless observational viewer test.

These tests can be implemented as research fixtures before any runtime migration. Their result should be reported as a pass/fail under named assumptions, not proof of arbitrary distributed-system resilience.

## Known task service versus autonomous discovery

The supplied sample says ten PoIs appear at random positions and times [R06]. It does not explicitly tell us whether coordinates are announced to the fleet at emergence, or whether the fleet must discover them through sensor observations. The main challenge describes surveying disaster zones and relaying sensed data. This unresolved distinction materially changes the search algorithm.

Current code receives `SurveyTask` objects with ground-truth positions/spawn times and chooses tasks after emergence. `_sense()` recognizes a visit using true horizontal distance at most 5 m and height 30–70 m; it then renders an image from the procedural parcel. This demonstrates physically located observation and image transport. It does not demonstrate visual discovery, survivor recognition, segmentation, false-alarm control or comprehensive area coverage.

Two explicit benchmark modes are preferable:

- **Announced target service:** the environment releases coordinates at spawn time. The planner solves time/energy/network-constrained service. Keep this mode as the current baseline and label it clearly.
- **Unknown target search:** only the world knows target locations; the planner gets sensor detections/coverage beliefs. Start with a deterministic lawnmower/partitioned sweep and a documented sensor model. Add frontiers or informative planning only when obstacle/unknown-map assumptions require them.

Do not add YOLO merely because the system contains drones. If the challenge requires real visual recognition, define target classes, resolution/range, dataset provenance, train/test separation and false/missed detection metrics first. Synthetic parcel screenshots alone are not a detector benchmark.

The existing idle/charge policy explicitly avoids using future task times or locations: `_plan_formation()` filters tasks by `spawn_s <= now`, derives its grace period from current geometry, and `_forecast_charge_admission()` uses announced work. Preserve those protections. The proposed metamorphic test checks that contract; it is not a finding that the present planner already exploits future coordinates.

RACER [R10] provides a substantive decentralized exploration reference: pairwise region allocation, workload balancing and hierarchical frontier/viewpoint planning, with explicit onboard execution and communication assumptions. Its objective is mapping unknown space; it does not by itself enforce AeroMesh's all-active-to-GCS connectivity, 100 m links, 20 m separation, 20-minute sortie limit or 10-second report deadline. Borrow a component only after those constraints are reintroduced and measured.

RACER's implementation section is particularly useful: map chunks have bookkeeping so lost information can be retransmitted, relative estimation is initialized using common visual features, and onboard cameras/IMU feed local estimation. Its physical demonstrations use two or three UAVs at 1–1.5 m/s in much smaller spaces; they do not qualify a 30-UAV kilometre-scale mission. Its authors reduce speed to preserve depth-image quality. These are concrete integration lessons, not reasons to copy the full stack.

A recent GP–Kalman informative-planning paper [R11] is valuable mainly for its careful separation of footprint observations, local fused beliefs and centralized reference beliefs. It explicitly warns that low posterior uncertainty is self-reported and may reflect overconfidence; independent ground-truth visit counts are needed. The paper's two-UAV field run is a feasibility demonstration without a controlled field baseline. For AeroMesh, first add a simple coverage/observation ledger and false/missed-detection tests; a learned planner should then beat a paired-seed coverage baseline under the same sensing model and radio constraints.

## Sensor geometry and affordable fidelity

`camera_sensor.py` uses an ideal pinhole downward camera with 60-degree vertical field of view, 128×128 pixels by default and no optical noise. The physics camera uses measured position but perfect world-down stabilization, independent of body attitude. This is an explicit model assumption.

For a flat plane and a nadir pinhole camera, ground footprint height is `H = 2 h tan(vertical_FOV/2)` and width is `W = image_aspect_ratio * H`, derived from the pinhole model [R13]. Ground sample spacing is `H / image_height` (equivalently `W / image_width` for these square pixels). At 40 m height and 60 degrees over 128×128 pixels, H and W are about 46.19 m and spacing is about 0.361 m/pixel. A 0.5 m feature spans only about 1.39 pixels across one dimension before blur or occlusion. **This is a geometric illustration, not a required person-detection threshold or an empirical detector result.** The 5 m visit gate in `_sense()` is not the full optical footprint.

VisDrone [R14] is a useful reference for the variation an aerial perception evaluation must cover: view, object size/density, lighting, weather and occlusion. It is not automatically a disaster-survivor dataset. If perception becomes part of the required mission, separate training and evaluation by scene/sequence, preserve negative frames and report false alarms, missed detections, localization error and detection-to-report latency by operating condition. Do not download or train on a large dataset until the target classes and resource budget are defined.

Recommended stages fit the storage constraint:

1. Declare and show camera footprint, acquisition time, ideal stabilization and actual received-image age.
2. Add camera/body extrinsics, attitude coupling or an explicit bounded gimbal model; test projection with known landmarks.
3. Add configurable delay/dropout, then noise/blur/exposure/occlusion only when evaluating a sensor-dependent policy.
4. Keep rendering fidelity independent of radio, control and task truth. Procedural obstacle blocks suffice for many line-of-sight, visibility and avoidance tests.
5. Defer photorealistic city assets and trained perception until their decision value and available storage are demonstrated.

## Validation discipline

NASA-STD-7009B [R07] distinguishes model verification from validation against referents and emphasizes intended use, assumptions, parameter pedigree, uncertainty and sensitivity. Use those principles as a research framework; AeroMesh is not being certified under NASA standards.

A controller can be correctly implemented and still operate on unrealistic information. A byte-identical replay can be reproducible and still model the wrong task. A photorealistic image can be attractive and still hide a synthetic detection gate. Every demo claim should name the relevant layer and its evidence.

The ACM artifact-review framework [R12] separates functional/documented/complete/exercisable artifacts from independent reproduction of results. Apply that distinction to the handoff: source archives and a successful local rerun are strong ingredients, but another developer's clean-environment run remains a separate gate. No ACM badge or external certification is claimed.

Two legacy documentation findings need reconciliation before jury handoff: `docs/REPRODUCIBILITY.md` names an older branch/commit and Python 3.12 environment, while the current verified physics environment is the pinned Windows Python 3.10 setup. `docs/REALTIME_PERFORMANCE.md` includes 1–20 UAV timing tables with a placeholder/example host and no raw artifact link in the inspected section. Treat those measurements as unqualified for the current 30-UAV backend until their provenance is established. Preserve historical content but make one current judge setup/evidence guide authoritative.

## Current recommendation order

1. Repair the measured nominal connectivity failure, then extend full physics qualification while preserving all failures.
2. Make information access, sensing mode and display provenance explicit.
3. Add timestamped telemetry/command contracts and isolated delayed/lost-message fixtures.
4. Validate an appropriate onboard lost-link policy with current resources.
5. Compare simple coverage/allocation/routing baselines before adding sophisticated methods.
6. Improve visual explanations around received evidence, route changes and grounded spare usage.

The source ledger and other topic notes provide complementary radio, physics, safety and demo details. This file records proposed work; it does not claim the proposed boundaries already exist.
