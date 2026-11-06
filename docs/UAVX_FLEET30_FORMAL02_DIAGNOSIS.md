# Nominal seed 109: formation liveness diagnosis

Inspected local evidence: `events.json`, `trajectory.jsonl`, `metrics.json`
and `manifest.json` in
`evidence/fleet30-withdrawal-formal-02/nominal-seed109/run`. These large files
remain local and ignored; the linked regression fixtures are part of the PR.
Both simulator source hashes matched that manifest when the diagnosis began.
Subsequent source repairs do not change these recorded results.

The run completed 5 of 10 tasks, landed every aircraft, and recorded zero
communication downtime, battery exhaustion, or separation violations.
The failures below concern mission progress in the kinematic simulator.

## Crossed assignments stall otherwise safe movement

The priority-task formation committed at 1285.5 s with a 159.92 s deployment
budget. Detection occurred only at 1911.5 s, 626 s after commitment.
The trajectory holds scout 28 near `(706.9, -54.4, 40)` from 1450 through
1900 s, short of its `(740.7, -54.4, 40)` target. UAV 12, assigned slot 3,
remains behind settled slot 0; UAV 0, assigned slot 15, remains behind slot 12.
Replacement dispatches beginning at 1831.5 s eventually unblock the chain.

Reconstructing the 1700 s trajectory in memory reproduces a one-second cycle:
UAV 12 moves `x=21.174 -> 23.674 -> 21.174`; UAV 0 moves
`540.896 -> 543.396 -> 540.896`; scout 28 moves
`706.866 -> 709.366 -> 706.866`. The first legal movement candidate alternates
between forward progress and the fixed negative-x escape. Safety checks
correctly retain separation and connectivity, but do not establish liveness.

A four-aircraft fixture isolates this problem: a task at `(170, -12.5, 0)`
creates four slots at altitude 40 m. Three aircraft start at slots 0, 1, 2;
the fourth starts at `(20, -12.5, 40)` and owns slot 3. After 60 s the crossed
assignment still has 150 m of target error, with 22.088 m minimum separation
and zero downtime. Reassigning the same cohort in projected chain order
reaches all slots within 60 s, with 23.710 m minimum separation and zero
downtime. Any reassignment must revalidate energy, deployment, retirement,
and deadline budgets before commitment.

## Idle grace skips measured handoff completion

Task 3 finishes at 604 s. No new task is announced until 797.5 s, so the
formation is intentionally held. Replacement UAVs 25, 6, 19, 7, and 1 reach
their 65 m handoff goals within 3 m at 718, 720, 720, 724, and 728 s,
respectively. No handoff completes before the withdrawal at 797.5 s.

The no-available-task branch returns before the base planner's handoff
maintenance. Incumbents therefore continue consuming energy while ready
replacements hover. Reconstructing the 796 s sample and advancing three
motion steps reproduces the admission rejection: retiring UAV 4 has 402.5 s
remaining, but needs 81.703 s deployment wait + 184.192 s own return + 180 s
reserve. The 445.895 s obligation correctly exceeds available energy.

Idle holding must still maintain measured handoffs and energy state without
selecting unannounced tasks. Calling the entire base planner is insufficient:
its empty-task branch orders a return instead of preserving the grace hold.

[Regression fixtures](../tests/test_formation_liveness.py) retain a 30-aircraft
fleet, constrain the eligible cohort with grounded charging aircraft, verify
actual bounded movement, and check ready handoff completion with two different
unannounced task locations. They use footprint mode and do not establish
camera, airframe, or physical-flight qualification.
