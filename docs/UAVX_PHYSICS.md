# AeroMesh rigid-body physics

The separate Bullet backend runs **30 simulated UAVs including spares** with
the existing mission, relay, radio and camera pipeline. It uses the installed
Panda3D dependency and small procedural geometry. No city download, Unreal
installation or physical aircraft connection is needed.

## Run it

Double-click `Run-AeroMesh-Physics.cmd` in the repository. For the compact
development mission, use PowerShell from the AeroMesh-UAVX folder:

```powershell
.venv\Scripts\python.exe run_physics.py --scenario-file scenarios/uavx/physics-compact.json --gui --output evidence/physics-demo-new
```

Omit `--gui` for a headless run. The default scenario is
`scenarios/uavx/nominal.json`. Use a new output folder on every run; existing
evidence is never overwritten. `--max-time 60` runs a short, deliberately
incomplete flight test. `--wind 3 0 0` applies a steady world-frame wind in m/s.
The compact fixture has ten emerging PoIs in a smaller part of the same
declared geofence. It is a development fixture, not an organizer-issued or
hidden challenge scenario.

The eagle observer supports wheel zoom, right-drag pan, middle-drag orbit,
Home full-map fit, A active-work fit, Tab drone selection, F close view, Space
pause, +/- requested playback speed, L links and Esc stop. Actual throughput
depends on CPU load; requested playback speed never changes simulated thrust,
gravity or time steps. The window stays open when a run ends. Detailed drone
components are visible when zoomed in; markers identify them at map scale.

## What determines flight

`simulation/quadrotor_physics.py` supplies a headless six-degree-of-freedom
Bullet world in SI units, with world Z pointing up. Each 1.5 kg body has an
explicit inertia tensor, four X-layout motors at a 0.28 m arm radius, thrust
saturation, first-order motor lag, reaction torque, gravity, wind-relative
linear/quadratic drag, angular drag and contact response. The initial pose is
set once at spawn. Subsequent motion comes from forces and torques.

The engine advances at 120 Hz. A local velocity/attitude controller mixes
four motor forces. Command expiry requests a zero-velocity hold through
feedback; power loss cuts all motor thrust and leaves gravity/contact active.
A stopped command therefore does not teleport, freeze or instantly brake a
drone. Velocity, tilt and rate limits apply to controller requests. External
forces and failed-aircraft freefall can exceed them and must remain visible
in measurements.

`simulation/physics_uavx.py` obtains motion references from a fully isolated
copy of the original motion-admission planner. Only reference displacement
and takeoff/approach intent are used. The copy cannot provide actual position,
landing, charge, energy, clock or mission credit. Bullet telemetry updates
the mission state. A 20 Hz local velocity constraint layer provides separation
and geofence braking margins; it is not a formal continuous-time guarantee.

`simulation/physics_planning.py` preserves the cohort matching policy but
uses 4 m/s travel estimates and an eight-second deployment allowance. Shared
withdrawal estimates also use the slower speed and extra settling allowance.
Avoidance can extend travel beyond these estimates. A complete physics matrix
is required before claiming challenge robustness for this backend.

Landing requires measured contact with the ground, position at the UAV's own
pad, speed at most 0.15 m/s, motor disarm, and at least two seconds of settled
20 Hz observations. A permanently failed body remains a failed aircraft even
after it hits the ground; that does not satisfy controlled all-UAV landing.
Armed endurance is charged per control interval, including the final powered
interval before disarm.

## Evidence and checks

```powershell
.venv\Scripts\python.exe -m pytest tests/test_quadrotor_physics.py tests/test_physics_uavx.py tests/test_physics_verifier.py -q
.venv\Scripts\python.exe verify_physics.py evidence/physics-demo-new --source-root .
```

Physics runs have their own `aeromesh-bullet-1` schema. Do not pass them to
`verify_submission.py`, whose geometric assumptions belong to kinematic
runs. Output includes:

- `metrics.json`: mission checks, engine parameters, 120 Hz maximum speed,
  contact history, and explicit calibration limits.
- `physics-telemetry.jsonl`: approximately 1 Hz full-fleet measured position,
  velocity, attitude, angular velocity, rotor forces, acceleration, contact,
  command freshness and failure state, plus a terminal sample.
- `physics-safety.json`: 20 Hz separation, speed and range-connectivity
  aggregates. These sampled records are not continuous-time proofs.
- Existing packet/image receipts, task events, scenario and output/source
  hashes. The viewer's screenshots/status are separate observational artifacts.

The verifier distinguishes evidence consistency from mission success. A
faithfully recorded incomplete mission or strict failure is valid evidence
and still does not pass the mission. Integrity hashes cannot establish that
the model accurately represents a real aircraft.

## Verified compact result — 6 November 2026

The source-stable compact fixture completed at **623.5 simulated seconds** with
all ten PoIs reported, all thirty UAVs grounded/disarmed, and all **15 checks**
passing. Ten UAVs launched, with eight active at peak; twenty stayed grounded
spares. Minimum sampled separation was 21.956 m, maximum 120 Hz measured speed
3.989 m/s, and maximum sampled altitude 92.646 m. There were no non-ground
collisions or sampled radio outages. The physics verifier accepted the evidence.

The selected regression run passed **534 tests**. After verifier hardening, the final focused suite passed **55 tests** (15 engine, 9 integration, 31 verifier). In the paired 60-second
native/headless viewer check, all twelve manifest artifacts were byte-identical.
The native run used the RTX 3050 and actual 3D window; its mission cutoff is
intentionally incomplete. Camera operations did not change physical bodies.
See [the compact source/metric summary](verification/uavx/physics-compact-final.json).

Full nominal/fault/resource physics qualification remains pending. An earlier
nominal exploration was interrupted near 871.5 s while code was being revised;
its partial logs are not a completed or source-stable mission result.

## Model boundaries

The collision envelope is a **0.35 m sphere** around the centre of mass.
Visual arms, propellers, legs and sensors are illustrative geometry; there
are no spinning blade collision volumes. The engine can add static boxes
for collision fixtures. This viewer's ground/grid is not a city reconstruction.
There is no terrain following, ground effect, propeller wake, downwash,
battery voltage sag, motor identification, sensor noise, or PX4 qualification.

Battery accounting remains the declared 1200-second endurance/recharge model,
not an electrical power calculation. Camera images use the existing synthetic
disaster parcels with an ideal world-down stabilized camera at measured body
position. They do not demonstrate survivor recognition or body-mounted
camera attitude. Radio service remains the declared range/queue/loss model;
building attenuation is not implemented here.

The original kinematic matrix and submission package remain separately
preserved. Their 17/19 result and 486-test evidence do not automatically
qualify this new backend. City/Unreal work remains paused for storage.
