# UAV-X failures and remaining work

The agreed fleet is **30 UAVs including spares**. The final source completed
all 19 declared development cases with valid, source-stable evidence:
**17 passed and 2 failed strict constraints**. All 12 nominal seeds passed.
The complete-radio-outage and permanent-loss failures remain visible in
[validation](UAVX_VALIDATION.md) and the [eleven-item register](UAVX_ISSUE_REGISTER.md).

## Core defects addressed

- Complete role matching checks common deployment, work, withdrawal and
  energy budgets for selected and retiring aircraft. A slow cheapest matching
  no longer hides a feasible complete alternative.
- Squared travel costs reduce crossed assignments that caused recurring
  forward/backward motion behind settled relays.
- Cohort selection favors up to 180 s of shared energy headroom before
  travel cost, avoiding immediate withdrawal of a nearly depleted formation.
- Ready relay handoffs finish during gaps in announced tasks. Invalid
  position or energy values cannot establish arrival or admission feasibility.
- Canonical slot ordering removes the demonstrated dependency of replacement
  identities and subsequent motion on Python's process hash seed.
- Connected group withdrawal and temporary clearance of landing-pad columns
  permit measured return without resetting energy or moving poses.
- An infeasible preferred new task no longer blocks a feasible announced task
  at the same priority. Unfinished work retains equal-priority authority.
- Healthy existing work can continue while a larger task waits for resources.
  A 30 s minimum idle grace protects short announcement gaps, while the hard
  return reserve still applies immediately. Holding a completed formation uses
  a copied charge-readiness forecast and useful follow-up cohort headroom.

The canonical matrix passed 16/19 cases. A later admission candidate fixed
seed 108 but regressed seeds 7, 105 and 107, passing 14/19. Revised readiness
planning then passed all four affected seeds. The final full matrix passed
17/19, with only the declared strict radio/permanent-loss failures. Every
attempt remains preserved. The final selected core suite passed 486 tests;
dependency checking passed.

The final review also found and corrected charging progression that revived
a permanently failed queued aircraft, optimistic floating-point charge timing,
and a hold guard that overlooked a finite low-energy replacement. Operational,
landed and disarmed aircraft now share the same charge eligibility rule in the
actual service queue and forecast. The forecast repeats actual timestep
arithmetic, bounded by the remaining mission horizon. Holding checks every
operational non-returning airborne aircraft, including replacements. These
corrections passed 89 focused tests and the four affected nominal seeds. A
subsequent timestamp correction aligns movement, landing and charge events
with the model clock for fractional timesteps. That source passed the final
486-test core suite and completed the final 19-case matrix with unchanged
source hashes and valid evidence.

## Strict fault failures

A complete outage of a UAV's sole radio disconnects that endpoint for the
declared outage even if its peers retain alternate paths. The final case
records 30 s downtime and fails continuous connectivity. All 10 PoIs were
reported and all 30 UAVs landed at 1966 s.

A permanently failed aircraft cannot land. The final case completed all
10 surveys and landed 29 survivors at 1973.5 s, but retains strict all-UAV
landing and mission-deadline failures. Strict completion is null at the
2700 s cutoff. Survivor measurements do not replace strict checks.
An exemption or independent backup radio cannot be invented to improve a score.

## Remaining acceptance work

- Preserve the completed 19-case matrix and selected core suite with their
  exact source snapshot. Re-run qualification after further runtime changes.
- Preserve the checked eight-page report, 74.4 s replay and full evidence
  archive from the final sources; [package contents](SUBMISSION_PACKAGE.md)
  record their QA and size.
- Keep centralized state, kinematic motion, synthetic perception and
  uncalibrated radio/energy parameters explicit.
- Obtain team/qualification details, official scenarios/log definitions,
  resource/framework acceptance, submission route and owner licensing before
  organizer submission.

The unmapped nested Git reference is removed while its local contents remain.
City work is paused at the user's request, excluded from the core PR, and not
needed to run this simulation. Earlier failures remain in the
[historical record](history/UAVX_FAILURES_PRE_FLEET30.md).
