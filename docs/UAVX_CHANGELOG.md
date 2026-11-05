# Changes and errors found — 2026-11-05

Base: `0ba914d79dc8223866c3333956c31251f5c5f661`.
Working branch: `codex/uavx-submission`. Original DroneOS and downloaded AeroMesh
checkouts were preserved; implementation lives in a separate AeroMesh-UAVX clone.

## Implemented

- Restored simulation-only CLI; new UAV-X profile is the default, old benchmarks
  remain explicit `--profile legacy`. Hardware-oriented old CLI preserved separately.
- Added sample constraints, energy admission, per-sortie accounting, own-pad
  settled/disarmed landing checks, and continuous swept separation checks.
- Added seeded dynamic tasks, prioritized re-tasking, range-limited relay chain,
  role dispatch/arrival distinction, reserve returns and non-instant recharge.
- Added immutable detection/send/receive evidence, packet provenance, offered
  traffic including no-route losses, one-hop-per-tick forwarding, duplicate-ID
  handling, failed-node drops and edge-count hop limits.
- Removed hard-coded connectivity, 2.1 s recovery, and unconditional robustness
  success from the legacy reporter. Unknown results are null; terminal packet
  recovery requires actual delivered traffic.
- Removed the simulator's undefined `dt` vision call and synthetic detector
  construction timing presented as inference. Added explicit synthetic sensor scope.
- Added isolated dependencies, adversarial tests and current documentation.
- The old optional SITL test is explicitly opt-in; normal validation never connects
  to or arms an aircraft. No physical simulator was launched for this change.

## Errors uncovered / corrected during implementation

- Original CLI did not accept the documented scenario flags; simulation crashed
  with NameError; report/verification scripts claimed success without execution.
- Original telemetry was offered only when connected, excluding disconnections
  from PDR; packet attempts without routes did not enter its denominator.
- Original routing counted nodes instead of edges against max_hops and could
  process a packet through multiple queues in one tick based on iteration order.
- Old completion could latch across new tasks; an empty/failed fleet could appear
  landed; relay dispatch was incorrectly described as completed recovery.
- Sample limits differ from legacy 2 km maps, 10 m/s cruise and 15 m separation.
- New full-scale runs exposed crowding during return and reserve exhaustion;
  these failures remain in local evidence. Return-altitude routing, horizontal
  escape choices for vertical conflicts, stable relay identities, and conservative
  reserve checks were added without suppressing measured failures.
- Landing approach could oscillate between descent and climb after a lateral
  avoidance step. Latching the approach keeps subsequent motion directed home.
- Shortening the relay chain could promote a replacement into a deleted slot,
  causing a KeyError and an invalid slot-speed ratio. Removed-slot replacements
  now return; a separate speed veto rejects every over-limit motion candidate.
- Radio outages now dispatch a relay replacement without waiting for low battery.
  Fault-injection events no longer supply a duplicate event-type argument.
- Landing events now use the supporting observation timestamp. Source hashes are
  captured before execution so edits made later cannot relabel a completed run.
- Final review found that an empty manifest and fabricated constraint booleans
  could pass the verifier. Complete artifact inventories and consistency checks
  now prevent that false success; hashes are not a claim of tamper-proof evidence.
- A connected group of UAVs could lose its GCS path while retaining local
  neighbors, bypassing the old recovery trigger. Recovery now checks the actual
  GCS-connected component and logs whether dispatch was caused by lost GCS
  connectivity or low energy. Three regressions and all four full reruns passed
  their expected outcomes; the original strict fault failures remain.

## Known limits

Read [requirements](UAVX_REQUIREMENTS.md) for model assumptions. The greedy
planner is not complete: deadlock, incomplete survey and communication downtime
can still occur. A passing small fixture does not prove a 45-minute mission or
hidden-scenario robustness. The full-run result table is in
[validation](UAVX_VALIDATION.md); no universal compliance claim is made.

AeroMesh's existing simulator adapter acknowledges commands without execution.
The current profile uses its own explicit kinematic state, not that acknowledgement
as landing proof. The cached Garuda Gazebo script's unconditional PASS and the
Garuda recorder's 600 s limit were identified but not modified in their repos.
No root LICENSE exists in the original AeroMesh tree despite historical MIT text;
repository licensing should be confirmed by its owner before external reuse.
