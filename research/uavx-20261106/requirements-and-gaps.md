# Requirement, implementation and evidence matrix

Reference runtime: `af544e505af51848f5e3a013a52b300f0cd4bfb9`. This matrix distinguishes **official requirements**, **team/model choices**, **inspected implementation**, **observed results**, and **unqualified work**. It does not assign an unofficial competition score.

The governing supplied PDF/template/slide are R01, R05 and R06. The public page R02 corroborates the rubric but has submission-route contradictions. References Pxx, SWxx and Vxx in the topic ledgers provide supporting research rather than new challenge rules.

**Final research update:** the unchanged-source full nominal physics run completed ten PoIs and landed all 30 aircraft at 2514 s, but passed only **14/15 checks** because continuous connectivity failed for **19.8 s**. All 30 launched over the run; peak sampled active was 28. The independent verifier accepted the evidence while correctly rejecting mission qualification. See [summary](integrated-nominal-summary.json), [verification](integrated-nominal-verification.json) and [outage diagnosis](integrated-nominal-connectivity-diagnosis.md).

| Topic | Governing requirement or declared choice | Current implementation and evidence | Remaining gap / next acceptance test |
|---|---|---|---|
| Fleet | Team choice: exactly 30 including spares; sample does not prescribe this number | Current configurations enforce 30. Full nominal physics launched all 30 over the run, peak 28 sampled active; compact launched ten, peak eight | Keep fleet count, spare count and charging resources visible; do not imply all 30 flew simultaneously |
| Mission arena | Sample: 1000×1000 m, operational center shown 75 m outside arena | Full nominal physics completed the declared arena/corridor mission at 2514 s but failed connectivity; compact fixture uses a smaller portion | Repair full-map connectivity; operational-center/pad extent is a declared assumption |
| Ten emerging PoIs | Sample: ten random locations and emergence times | Seeded/explicit scenario tasks; planner filters availability by emergence, and idle/charge policy avoids future task times/positions | Official announced-coordinate versus sensor-discovery semantics unresolved; preserve separate modes and a future-information test |
| Mission clock | Sample: finish and return by 45 minutes | Full nominal completion/return at 2514 s is within the 2700 s cutoff, without a full-profile pass | Repeat after connectivity repair; fault and resource physics matrix remains unqualified |
| Flight endurance | Sample: maximum 20-minute flight | Armed sortie time and remaining endurance tracked at 20 Hz in physics; admission/return reserve logic exists | Current 1200 s model is not electrical energy; add calibrated power/pack model only when parameters exist |
| Charging | Team/model choice: nominal 30 pads, 720 s full recharge; resource case uses 15 pads | Shared eligibility/admission/forecast tests and kinematic resource case | Organizer acceptance unknown; do not call 720 s charging a measured LiPo property |
| Speed | Sample: ≤5 m/s | Physics commands limited to 4 m/s; engine observes actual 120 Hz maximum. Compact maximum 3.989 m/s | Wind/failure/contact can exceed commands; report actual extrema and qualify disturbance envelope |
| Height | Sample: ≤100 m | Flat-world z-up coordinates and measured height checks; compact maximum 92.646 m | Terrain-relative altitude, obstacle height and sensor errors unqualified |
| Separation | Sample: ≥20 m | Sequential velocity projections; 20 Hz pairwise measurement. Compact minimum 21.956 m | Actual-velocity/braking-aware safety, joint feasibility and intersample checking; no formal guarantee currently |
| Landing | Sample: all aircraft land at start area | Contact + own-pad proximity + low speed + disarm + two-second settling; all 30 in compact and full nominal fixtures | Fault/resource return unqualified; failed body on ground is not a controlled landing; touchdown/terrain model calibration absent |
| Radio range | Sample: maximum 100 m | Declared range graph and sampled connectivity; bounded shared TX/RX service and queues | Obstacle attenuation, interference, asymmetric links, protocol discovery and clock calibration absent |
| Continuous connectivity | Challenge emphasizes resilient GCS communication | Full nominal physics fails: five recorded outage windows total 19.8 s during formation pivots and return | Protect measured 3D link margin through motion and handoff; sole-radio total outage remains a separate strict failure |
| Role/relay management | Published rubric: 20% | Budgeted assignment, reserve-aware handoff and connected withdrawal, with events | Baseline/ablation under identical information/resources; distinguish assignment decision from arrived/verified handoff |
| Fault recovery | Published rubric: 15% | Explicit link/radio/vehicle faults, replacement and survivor metrics; physical power cut falls | Controller knows injected truth/shared state; measure detection, decision, receipt and service restoration through actual modeled observations |
| Report deadline | Sample: ≤10 s from first detection to center | Complete PNG reassembly, length/SHA-256 check; recapture retains original detection deadline | Large-image/load/tick sensitivity: isolated 256 KiB cases miss at five or more hops. Qualify useful image size and quality, not only packet PDR |
| Image transport | Main problem/template asks sensed data/imagery | Actual TinyDisplay RGB at measured Bullet position, chunked through the router | Shared `report_s` stops retries without reverse ACK. Research feedback comparison shows bandwidth/latency tradeoffs |
| Discovery/perception | Mission surveying described; exact detector requirement unresolved | Proximity/altitude visit gate triggers a procedural image; no learned survivor detector | If autonomous discovery is required, implement sensor-only detections/coverage beliefs and false/missed detection evaluation |
| Camera fidelity | Declared ideal sensor model | 60° vertical FOV, default128×128, world-down stabilization, no noise/distortion | Body/gimbal extrinsics, latency, blur and occlusion as needed. Current resolution cannot establish useful survivor-recognition performance |
| Physical dynamics | Team request: realistic simulated quadcopters | Bullet 120 Hz, explicit mass/inertia, four forces/torques, lag, gravity/drag/wind, contact; 40 isolated research trials | Parameters uncalibrated; finite-step contact penetration and impulse error measured; no downwash/ground-effect/electrical validation |
| Common environment | Proposed simulation consistency requirement | Camera parcels are visual geometry in a separate scene; mission physics has ground + UAV spheres; RF has no parcel map | Shared small scene manifest with explicit visual/collision/visibility/RF roles before adding tall city assets |
| Centralized authority | Template permits centralized, decentralized or hybrid if justified | Shared true poses/vehicle state and direct command calls; graph reconstructed centrally | Separate referee truth, onboard observations, received GCS state and delivered intent before claiming communication-limited autonomy |
| Actual telemetry | Needed for defensible ground-station display/control | Current telemetry packets contain only `kind`; arrival service is modeled | Serialize time-stamped state and charge its bytes; maintain receive-only store; stale/reordered sample tests |
| Evidence | Template requests reproducible source, metrics and demo | Source/output hashes, independent verifiers, preserved failures, native/headless observer agreement | Full physics matrix, crash-readable incremental packet/event records, clean-machine reproduction and canonical current guide |
| Demonstration | Stage3 unseen mission plus presentation | Current 3D viewer physically observes the run; research wireframe is separate mocked UI | Integrate route/receipt/role/fault explanation without influencing flight; show provenance and actual complete image delay |
| City/Unreal | User has paused city work for storage | Earlier assets/prototype preserved; not needed for current Bullet backend | Defer large downloads; first make a small procedural scene semantically consistent |
| Submission/admin | Official stages and deliverables apply | Local source/evidence package and draft PR exist | Team registration/qualification, Stage2 scenario/log schema, resource acceptance and conflicting submission route remain unresolved |
| Licensing | Challenge requires original/noninfringing work | Source provenance notes and citations can be delivered; raw research cache ignored | Root project license unresolved; inspect component/asset/data rights before redistribution |

## How the original eleven issues map to this research

The eleven entries in `docs/UAVX_ISSUE_REGISTER.md` remain the original register, not a claim that eleven binary defects can all be erased. This research adds evidence and implementation choices:

1. **Scenario robustness:** extend the frozen physics suite and separate adversarial coverage from random-sample statistics.
2. **Fleet/staging:** preserve the fixed 30-resource ledger, explicit pads and conditional feasibility bounds.
3. **Physical dynamics:** retain force-driven Bullet and qualify numerical/contact/control assumptions.
4. **Radio:** address packet-clock coupling, useful payload deadlines, reverse feedback and correlated outages.
5. **Centralized state:** define and test information/command boundaries.
6. **Synthetic perception:** retain honest labels; qualify discovery and image utility separately.
7. **Energy:** distinguish endurance scheduling from calibrated power/voltage/charging.
8. **Permanent recovery:** keep irreversible total loss a strict failure; evaluate survivor continuity separately.
9. **Flight backend:** complete full physics integration checks; the paused city bridge is a separate backend.
10. **Evidence limits:** source hashes, raw traces and checks are necessary but do not establish physical fidelity or hidden-scenario success.
11. **Repository reference:** the unmapped gitlink repair remains completed; current documentation and clean install are separate handoff concerns.

The master recommendations rank these changes. A research note, diagram or passing isolated probe is not itself a production fix.
