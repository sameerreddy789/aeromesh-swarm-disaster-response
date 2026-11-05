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
- Motion admission also preserves existing redundant GCS paths during ordinary
  deployment, replacement and task changes. A conservative graph retains only
  edges present at both ends of the movement interval. Reachability takes
  precedence, then biconnectivity; the existing emergency-return reserve can
  override this guard. Actual downtime and path protection are still measured
  independently, and arbitrary failures are not guaranteed survivable.
- Priority remains dominant. When the current task finishes, equally urgent
  announced tasks are selected by proximity to the previous task, reducing
  unnecessary full-map travel. Future task positions are excluded from selection.
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
  cannot supply two complete charging increments in one timestep.
- A landing approach displaced more than 5 m from its home pad below 30 m is re-acquired
  from overhead before descent resumes. This avoids trapping a descending UAV
  among neighboring grounded vehicles after a collision-avoidance maneuver.
  Detours above that band continue around airborne relays instead of restarting
  a blocked descent. Measured landing, speed, separation and battery checks apply.

## Resources and model scope

The example scenarios explicitly use **60 UAVs and 60 charging pads**, versus
30 UAVs in the earlier unprotected benchmark. Fleet size is a resource cost, not
an organizer-approved allowance. A 30-UAV resource case is included in the matrix
to expose its effect. Full recharge remains the declared 720 s assumption.
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
