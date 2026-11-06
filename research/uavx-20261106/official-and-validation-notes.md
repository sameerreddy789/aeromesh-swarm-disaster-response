# Official criteria and validation research — working notes

## Authority and confirmed facts

The supplied problem statement remains governing. Its page5 lists six criteria totaling100%: mission25, communication25, role management20, recovery15, safety10, innovation5. The live official page corroborates those values, although it calls them “seven” while listing six. No seventh category is specified. [R01,R02]

The supplied template explicitly permits centralized, decentralized or hybrid coordination if justified. Distributed algorithms may improve resilience, but claiming they are mandatory would be incorrect. Its section5 identifies communication as the heart of this challenge. [R05]

The official page still has a Forms link above rules and FAQ requiring email-only submission. Keep this unresolved; research does not authorize sending submissions or asking organizers. Stage2 qualification, official advanced scenarios and standardized log schema remain unavailable. Registration closed is not evidence of this team's qualification status. [R02]

## Implication for project priorities (design inference)

Communication, role management and recovery account for60% of published weight. A visually impressive city is useful only if it helps the viewer understand those decisions or exposes meaningful RF/obstacle constraints. A defensible default demonstration is a simple disaster scene with clear motion, selected end-to-end routes, delivered imagery and a fault/recovery timeline. This is an inference from the rubric, not a promise about individual judges' preferences.

Mission25% makes reliable complete runs and deadline behavior the first engineering gate. Safety10% does not mean accepting unsafe behavior for points; the problem also contains hard constraints. Novelty5% argues for a small, explained and ablated improvement over a working baseline instead of stacking unverified algorithms.

## Evaluation design

NIST's response-robot work separates capabilities and uses quantitative repeated tests. Apply that method as inspiration: one-drone mechanics tests, controlled pair encounters, radio transport tests, then integrated mission benchmarks. This is not a claim of NIST/ASTM certification. [R03]

For random scenario seeds, publish the seed-generating procedure before evaluating and use paired seeds for comparisons. Report success fraction with an appropriate binomial interval; an all-success set does not establish a100% general success rate. NIST documents Wilson and exact intervals. Do not apply an IID confidence interval to the mixed19-case handpicked fault/resource matrix as if it were a random sample of all disasters. [R04]

Separate these result layers:

1. Mechanics correctness for the declared equations.
2. Controller stability and tracking in the declared disturbance envelope.
3. Network/path and payload service behavior.
4. Mission constraint satisfaction and recovery on a frozen scenario suite.
5. Sim-to-real calibration and physical qualification, which are absent.

A pass at one layer must not silently substitute for another. A1Hz recording does not establish continuous separation; a20Hz safety ledger is stronger sampled evidence but still needs a bounded between-sample argument or finer independent checks.

## Current empirical baseline

Compact physics fixture:30 total,10 distinct UAVs launched,peak8active,20groundedspares;10PoIs;15checks passed at623.5s. Minimum20Hz separation21.956m;120Hz speed maximum3.989m/s;zero sampled communication downtime. This is a useful integrated smoke scenario, not proof that distant or hidden missions pass.

The unchanged-source full nominal physics run in `evidence/research-physics-nominal-20261106-01` completed at 2514 s. Ten PoIs were reported and all 30 aircraft landed; all 30 launched during the run and peak sampled active was 28. It passed **14/15 checks**, failing continuous connectivity with **19.8 s** of downtime across five windows. The independent verifier accepted the evidence and returned `mission_passed=false`. Minimum sampled separation was 20.368 m, maximum engine speed 3.996 m/s and maximum sampled height 99.007 m. These observations do not qualify fault/resource or hidden scenarios. See [the summary](integrated-nominal-summary.json) and [outage diagnosis](integrated-nominal-connectivity-diagnosis.md).
