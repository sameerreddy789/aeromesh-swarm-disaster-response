# Scenario-driven resilience and image delivery

The updated AeroMesh runner combines configurable faults, denser relay paths,
bounded network service and a scene-camera-to-GCS data path. Use
`--scenario-file scenarios/uavx/nominal.json`; `Start-AeroMesh.cmd` selects it by
default. The earlier footprint-only profile remains available without this flag.

## What the implementation now does

- Strict JSON scenarios declare realized tasks, resources, camera resolution,
  packet loss, and an ordered schedule of link outages, complete radio outages
  and permanent aircraft failures. Unknown keys, malformed values and ambiguous
  overlapping explicit faults are rejected. Dynamic targets retain both the
  requested injection time and the actual resolved UAV/link and time.
- Protected relay placement uses 44 m nominal slots. A settled single missing
  intermediate relay can leave an 88 m gap, within the 100 m range. The actual
  graph is sampled for two internally node-disjoint paths to the GCS. Merely
  enabling protection is not treated as proof that every moving UAV is protected.
- Motion admission also protects redundant GCS paths for aircraft assigned to
  mission work during ordinary
  deployment, replacement and task changes. Primary reachability uses only
  edges present throughout the movement interval; backup links may hand over
  to new neighbors and are evaluated at the proposed endpoint. Reachability
  takes precedence, then biconnectivity; the emergency-return reserve can
  override this guard. Actual downtime and path protection are still measured
  independently. Two-path protection at every intermediate instant and arbitrary
  failure survivability are not guaranteed.
  Returning aircraft retain the primary-path requirement but may release their
  backup path, so redundant-path admission does not consume their return reserve.
- Priority remains dominant. When the current task finishes, equally urgent
  announced tasks are selected by proximity to the previous task, reducing
  unnecessary full-map travel. Future task positions are excluded from selection.
  If the preferred new task lacks a complete feasible assignment, the planner
  tries other announced tasks at the same priority. It does not use a lower
  priority or future task to bypass that decision, or preempt unfinished work
  with an equal-priority alternative.
- Before committing a formation, the planner finds a complete one-to-one role
  assignment under common deployment, work, withdrawal and energy budgets.
  It tries feasible budget bounds instead of rejecting a task because one
  cheapest matching was too slow. Selected cohorts favor up to 180 seconds of
  additional shared energy headroom, then added flight cost; squared travel
  costs reduce crossed role assignments. These are planning estimates, and
  measured movement still decides whether deployment succeeds.
- During a short gap in announced work, an already dispatched replacement can
  complete its handoff only after observed arrival and GCS connectivity. While
  a larger task waits for chargers, a complete connected cohort can continue
  its existing assignment above the shared reserve. Idle announcement gaps
  allow a minimum 30 s grace above the hard return reserve. After that grace,
  waiting also preserves another 40 s survey/report budget and 180 s readiness
  headroom. The hard reserve applies immediately, even during grace.
- A completed cohort waits for charging resources only when a copied forecast
  predicts useful task admission. It uses measured charging levels and the
  finite-pad queue; it never assumes returning-aircraft arrival. A separate
  copied clock and copied flight records leave real time, energy, ownership,
  positions and RNG unchanged. The forecast cannot launch aircraft or grant
  arrival/charge credit. Actual task admission still requires actual readiness.
  Already-announced follow-up work determines the capable subset needing extra
  headroom. This is a planning heuristic, not proof that the follow-up will
  complete. New announced work, operational-fleet changes, readiness expiry
  or formation changes invalidate cached waiting decisions.
  The hold reserve also covers operational non-returning airborne replacements
  and unassigned UAVs. An incomplete, disconnected or depleted formation
  withdraws. Planning and handoffs never reset sortie clocks.
- Every node, including the GCS, has a shared half-duplex TX/RX service limit.
  The example uses 131072 bytes/s and a 1048576-byte queue. Actual serialized
  frame sizes, including base64 image bytes, consume capacity; retries consume
  capacity too. Partial transmissions continue across ticks without accumulating
  idle credits. Control and emergency traffic receive priority, with aging for
  ordinary traffic. Queue overflow remains a recorded packet drop.
- Panda3D TinyDisplay renders a downward RGB image of a procedural disaster
  scene at the observing UAV's measured kinematic pose. The camera records source,
  acquisition time, configured intrinsics, scene fingerprint and PNG hash. The
  actual PNG is fragmented into packets; complete, hash-verified GCS reassembly
  establishes reporting. The GUI shows only received images, never an onboard
  frame that bypassed the network. **V** switches camera/details views.
- If a sensor carrier fails, a replacement can recapture the site from another
  safe altitude. Both capture attempts and the original detection deadline are
  preserved. A late recapture is still late; the deadline is not restarted.
- Planning uses 22 m separation where the declared pad spacing permits it,
  while the original 20 m measured check remains unchanged. Failed aircraft stay
  as obstacles. Charging-pad admission is frozen per interval so a released pad
  cannot supply two complete charging increments in one timestep. Actual service
  and its forecast share eligibility: a UAV must be operational, landed,
  disarmed and have a finite, nonnegative energy value and a started charging
  interval. Failed aircraft cannot receive charge or be revived at completion.
  Forecast timing repeats the actual floating-point increments within the
  remaining mission horizon. Pads model shared service, not detailed physical
  docking or occupancy.
- A landing approach displaced more than 5 m from its home pad below 30 m is re-acquired
  from overhead before descent resumes. This avoids trapping a descending UAV
  among neighboring grounded vehicles after a collision-avoidance maneuver.
  Detours above that band continue around airborne relays instead of restarting
  a blocked descent. Measured landing, speed, separation and battery checks apply.
- Mission aircraft obstructing a returning UAV's home-pad column temporarily
  move sideways to clear it, then resume their original target/slot. Yielding
  uses the normal speed cap and the same swept separation, geofence, radio and
  energy checks. It is measured movement, not a position reset or landing waiver.

## Resources and model scope

The example scenarios explicitly use **30 UAVs and 30 charging pads**. The
resource-stress case retains 30 UAVs and reduces charging capacity to 15 pads.
Earlier 60/72-aircraft results are historical and cannot qualify this fleet.
Fleet size remains a resource cost, not an organizer-approved allowance.
Full recharge remains the declared 720 s assumption.
Operational grounded/charging UAV radios remain available as network nodes;
the shared service budget applies to them too. This availability and their
external charging power are declared simulation assumptions.

Motion remains kinematic, and camera imagery comes from an ideal pinhole camera
over generated scenery. This demonstrates simulated image collection and data
delivery, not learned survivor recognition, a physical camera or qualified flight
dynamics. Per-node service contention is modeled; obstacle RF propagation and
interference between disjoint radio links are not calibrated. Centralized planning
is retained, as permitted by the supplied report template with justification.

## Read the fault results correctly

A link outage can be routed around when an alternative path exists. A complete
radio outage disables every link incident to that UAV; redundancy elsewhere
cannot communicate with that endpoint until its radio returns. A permanent
aircraft failure freezes that aircraft. Its missing landing must remain failed.

`survivor_metrics` explains whether the operational fleet completed the survey
and returned. It never substitutes for the strict all-UAV landing/deadline checks.
`recovered_s` requires fresh post-fault packets from every member of the recorded
affected source group, excluding the disabled endpoint. It is a service receipt
metric, not proof of restoration of every radio.

The two added checks require every scheduled fault to have been exercised and
every PoI image to have reached the GCS. An unexercised late fault or a footprint
run cannot silently pass those checks.

## Evidence and reproduction

Schema 2 retains the original five core artifacts plus `scenario.json`,
`radio-service.json`, `sensing/observations.json`, original capture PNGs and
received PNGs. All are included in the run manifest. Retried captures have
distinct attempt numbers and files. Runtime source hashes are captured before
execution; an old run should be verified with its saved source snapshot.

The verifier reconstructs complete image receipt from delivered fragments,
checks source/timestamp/image consistency, compares radio service with capacity
and delivered byte/hop requirements, and checks completion against supporting
survey, receipt and landing timestamps. It remains a consistency check, not
independent physics reconstruction or cryptographic authority.

```powershell
.venv\Scripts\python.exe -m pip install -r requirements-repro.txt
.venv\Scripts\python.exe run_simulation.py --scenario-file scenarios/uavx/nominal.json --gui --output evidence/new-live
.venv\Scripts\python.exe scripts/benchmark_uavx.py --output evidence/new-matrix --workers 2
```

The matrix preserves every planned case, crash and constraint failure. It saves
the exact case inputs before execution and rejects changes to relevant source
files during the run. These are declared development scenarios, not unseen jury
tests. The source snapshot and full evidence accompany the delivery update.

## Organizer and ownership dependencies

Team identity/qualification, official Stage 2 scenarios and log specifications,
allowable resources, custom-framework acceptance and failed-aircraft scoring
cannot be supplied by implementation. The root license also requires an owner
decision. Report/video finalization and submission must use those answers. No
competition submission or message to an organizer has been sent.
