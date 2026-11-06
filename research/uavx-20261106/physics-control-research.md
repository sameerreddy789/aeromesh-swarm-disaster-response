# Physics, vehicle control, and simulator fidelity research

Specialist literature synthesis completed 2026-11-06 at approximately 05:30 UTC; research began at 02:28:39 UTC. This document records literature and source-code findings against the frozen AeroMesh-UAVX physics branch. Forty isolated numerical trials completed. The later full nominal mission finished at 2514 s with ten PoIs and all 30 aircraft landed, but only 14/15 checks passed: continuous connectivity failed for 19.8 s. See [the final summary](integrated-nominal-summary.json) and [outage diagnosis](integrated-nominal-connectivity-diagnosis.md). Integrated crosswind and additional injected-outage cases remain unrun. A host-clock discontinuity was observed during the session; the wall-clock research window must not be represented as three hours of active computation. No production changes, installs, or large asset downloads are part of this research, and simulator evidence does not qualify an aircraft.

## Evidence scope

The inspected repository HEAD at 02:33 UTC was `af544e505af51848f5e3a013a52b300f0cd4bfb9`. The physics engine is `simulation/quadrotor_physics.py`, SHA-256 `b81c3003f0a3845c17f3779d5396f422f37204ad56a9296138490a44dcc61074`. The verifier is `verify_physics.py`, SHA-256 `b037e63bd3ea451a75778e7ef85ea286aff4b6140cb89149c70c5b5a2e6636d9`. These hashes identify the reviewed implementation; later source changes require fresh qualification.

The pre-research qualification log is `qualification/physics-tests-final-observed.txt`: 55 tests passed in 13.13 s. It is explicitly a transcription of an earlier observed run, not a research-phase rerun. The compact mission under `evidence/physics-compact-final-01` simulated 623.5 s and passed its extended verifier. Those results concern one uncalibrated software model and a compact scenario. They do not establish accuracy for a 1000 m mission, urban aerodynamic wakes, onboard estimation, or physical hardware.

## Current model, before any literature proposals

The engine uses headless Panda3D 1.10.16/Bullet with SI units, world Z-up, body +Z thrust, and wxyz quaternions. Each vehicle has a spherical collision envelope, mass 1.5 kg, diagonal inertia (0.025, 0.025, 0.045) kg m², arm length 0.28 m, and four X-layout thrust actuators. Each actuator is bounded to 8 N and has first-order *thrust* lag with time constant 0.035 s. The integrator is stepped explicitly at 120 Hz. Ground and box contacts are resolved by Bullet. Positions and velocities are not written after initial spawning.

The controller maps a bounded velocity reference through PI velocity feedback, gravity compensation, a tilt cone, a desired orientation, a sine-like rotation error, rate feedback, and a saturated mixer. Its 4 m/s speed, 3 m/s² acceleration, and 0.45 rad tilt settings constrain requests, not the physical state after gusts or contact. A stale command requests zero velocity through feedback; it does not freeze the pose. Permanent failure cuts thrust immediately and leaves the rigid body to fall.

Wind is a deterministic velocity vector. Drag is isotropic linear plus quadratic drag relative to that wind. Observations are ideal Bullet truth. There is no IMU/GNSS/barometer noise, timestamp latency, estimator, rotor-speed state, blade-element model, ground effect, downwash, turbulence, propeller gyroscopic inertia, structural compliance, electrical battery model, or physical charging model. The 1200 s endurance/recharge rules remain explicit mission assumptions. The camera is ideal world-down stabilized at measured center of mass; it is not a mounted sensor/rolling-shutter/gimbal simulation.

## Research questions

1. Which dynamics and controller equations are consistent with the present implementation, and which proven-controller assumptions are absent?
2. Which low-cost additions most improve relevant accuracy after measurement and parameter identification?
3. Which simulator features supply real dynamics versus rendering, interfaces, or optional model plugins?
4. Which validation fixtures and timestep/latency studies distinguish numerical consistency from experimental validity?
5. Which fidelity tiers fit a CPU, an RTX 3050-class GPU, and constrained storage? All unmeasured throughput and storage figures will be labeled estimates.

## Source provenance

The structured evidence inventory is `physics-control-sources.json`. Sources are primary papers, official documentation, and pinned official repository code. Search snippets and failed retrievals do not support substantive claims. Article dates come from publication/version records, not proxy retrieval headers.

The final inventory contains 32 primary sources, including Lee et al. arXiv:1003.2005v4, Faessler et al. arXiv:1712.02402v3, and the current GymPyBulletDrones official repository, which redirects to `learnsyslab/gym-pybullet-drones`. Each entry states the inspected text/code and transfer limits. Exact initial retrieval timestamps were not retained for four late additions; those entries state the retrieval date and final verification time instead of inventing precision.

## First-wave findings: equations mapped to code

Citation IDs below resolve to the URL, exact version/commit, claim, and limitations in the JSON inventory. Equations in this report use AeroMesh's **Z-up and body +Z thrust** convention; copying a NED paper's signs unchanged would reverse gravity/thrust.

### Rigid-body and actuator equations

Let `p` and `v` be world position and velocity, `R` map body coordinates to world coordinates, `omega` be body angular velocity, `J` be the body inertia matrix, and `e3=(0,0,1)`. The implemented free-flight force/moment structure is

\[
\dot p=v,\qquad
m\dot v=-mg e_3+R e_3\sum_{i=1}^{4}f_i+F_D+F_{contact},
\]

\[
\dot R=R[\omega]_\times,\qquad
J\dot\omega=\sum_i r_i\times(f_i e_3)
+e_3\kappa\sum_i s_i f_i-\omega\times J\omega-c_\omega\omega+\tau_{contact}.
\]

`f_i` is in N, `r_i` in m, `J` in kg m², and the reaction-torque/thrust ratio `kappa` in m. The drag coefficient `c_omega` has units N m s (radians are dimensionless). Bullet supplies contact impulses rather than the smooth contact-force terms displayed here. These equations follow the standard rigid-body structure [P01], with additional chosen drag and contact models.

For air-relative velocity `u=v-w`, the current drag is

\[
F_D=-(c_1+c_2\|u\|)u,
\]

where `c1` is N s/m and `c2` is N s²/m². This is isotropic and dissipative relative to the air: `F_D dot u <= 0`. Wind can still do positive work in the world frame. This is **not** the same model as body-axis rotor drag `-m R D R^T(v-w)`, where `D` has units 1/s. Faessler et al. derive their flatness result for constant body-axis linear rotor drag with no wind, stiff propellers, and no thrust dependence [P02]. RotorS instead contains rotor-speed-dependent drag perpendicular to the rotor axis and rolling moments [P04]. Each is a distinct approximation, not a selectable universal accuracy level.

The current actuator state is thrust, with

\[
\dot f_i=(f_{i,cmd}-f_i)/\tau_f,
\quad
f_{i,k+1}=f_{i,cmd}+(f_{i,k}-f_{i,cmd})e^{-\Delta t/\tau_f}.
\]

The second expression is the exact update for a held command. The implementation uses `-expm1(-dt/tau)` for numerical stability. At 120 Hz and `tau=0.035 s`, there are 4.2 physics intervals per time constant. That does not itself establish adequacy of the coupled controller/contact discretization. A speed-state alternative is `dot Omega=(Omega_cmd-Omega)/tau_Omega`, `f=k_f Omega²`, `Q=s k_m Omega²`; when Omega is in rad/s, `k_f` has units N s² and `k_m` N m s². A first-order speed lag is not equivalent to the same first-order thrust lag during large steps. RPM coefficients require the `(2*pi/60)²` conversion. Current code cannot identify rotor angular speed or electrical/mechanical power from thrust alone without added assumptions [P03, P04, P21, P23].

With the present parameters, steady upright hover requires `mg/4=3.67875 N` per rotor. The static total thrust/weight ratio is `32/14.715=2.175` (calculated, not measured). At collective 8 N per rotor the current allocation leaves no differential torque authority. A maximum-thrust vertical acceleration of `32/1.5-9.81=11.52 m/s²` is a no-drag upright model bound, not the 3 m/s² requested-acceleration limit. Flight and safety logic must use available state-dependent authority rather than treating command limits as hard physical-state bounds.

### What the current controller does and does not inherit

The velocity integrator is `I_v <- I_v + (v_ref-v) dt`. Therefore `I_v` has units **m**, `Kp` has units 1/s, and `Ki` has units 1/s² when the output is acceleration. The configuration name `velocity_integral_limit_mps` is dimensionally misleading for the integrated variable; this is a documentation/naming finding, not a production edit or a demonstrated mission failure.

The implemented attitude error is

\[
e_{impl}=\tfrac12(R^T R_d-R_d^T R)^\vee,
\qquad \omega_d=K_R e_{impl},
\]

followed by rate clipping and `tau=J K_omega(omega_d-omega)+omega cross J omega`. Lee et al. use the opposite error sign with negative feedback, transformed desired angular rates, and desired-angular-acceleration feed-forward [P01]. Their initial-error conditions and ideal-input assumptions do not carry over to this rate-clamped cascade with motor lag, nonnegative rotor thrusts, collective-preserving differential saturation, and velocity PI.

For a 180-degree relative rotation, this skew-symmetric error vanishes. Exact inversion is therefore a useful mathematical counterexample to any claim of unconditional recovery. It does not imply the tested upright mission encounters that state. Bounded thrust also makes sufficiently large downward velocity or late braking physically unrecoverable regardless of the control law. Retain an explicit operating envelope and test saturation recovery, finite settling time, and disturbance rejection within that envelope.

PX4 demonstrates the engineering details a mature cascade adds: derivative filtering, integrator limits, allocation-saturation feedback, and precise normalized command interfaces [P08, P10]. Its gains cannot be copied into the SI torque controller. Also, the PX4 parameter prefix `MPC_` denotes multicopter position control; it is not evidence that a model-predictive optimizer is running.

### MPC and barrier guarantees

PAMPC optimizes a reduced nonlinear position/quaternion/velocity plant driven by mass-normalized thrust and body-rate commands, couples a mounted camera's projection to motion, and enforces input bounds [P20]. Its 2 s horizon at 0.1 s transcription intervals is different from its 100 Hz control update. It reported a 3.53 ms average solve time on its own Snapdragon platform; that is not a measured 30-vehicle Windows result. A future AeroMesh MPC should include identified actuator lag or a verified inner-loop tracking envelope, enforce actual allocation constraints, and define a feasible fallback for late/failed solves. Perception terms in this paper are optimized costs; they are not an unconditional visibility constraint.

For a control-affine plant `dot x=f(x)+g(x)u`, a continuous-time CBF inequality has the form

\[
L_f h(x)+L_g h(x)u+\alpha(h(x))\ge0.
\]

The inequality only supports an invariance claim under its hypotheses, including a valid initial set, feasible bounded inputs, and a suitable plant/model. A held digital command must satisfy safety between updates. Singletary et al. explicitly formulate reachable-set conditions over a complete hold interval and use a backup-controller invariant set, with additional assumptions for uncertainty and known delay [P17]. Current 20 Hz velocity projections do not implement that construction. The 120 Hz extrema/contact evidence is stronger observation than a 1 Hz log, but it is still discrete numerical evidence.

A basic braking requirement illustrates why geometry alone is insufficient: with closing speed `v_c`, response delay `t_d`, and a verified lower bound `a_rel>0` on available relative braking acceleration, the one-dimensional stopping contribution is `v_c*t_d + v_c²/(2*a_rel)`. This is a model calculation, not a sufficient 3D quadrotor safety certificate. Tilt buildup, motor lag, wind, failed vehicles, and constraints shared across neighbors can all reduce `a_rel`. The swarm report covers pairwise multi-agent formulations and QP feasibility in more detail.

With bounded relative-position uncertainty `epsilon_p` and a conservative tracking/reachable-tube allowance `epsilon_track`, an illustrative one-dimensional margin is `d_margin >= v_c*t_d + v_c²/(2*a_rel_min) + epsilon_p + epsilon_track`. The lower braking bound must hold throughout the maneuver and account for simultaneous constraints; a maximum requested acceleration is not such a lower bound. Delay includes observation age, transport, computation and held-command time as applicable, without double-counting terms already included in a reachable tube. If the lower bound is unavailable or zero after failure, this expression supplies no finite guarantee. Measured fixture stopping distances support envelope identification, not substitution of an observed best case for a robust bound.

## Aerodynamics, ground contact, and atmosphere

### Prioritize identifiable effects

NASA's ideal actuator-disk relation gives `T=2*rho*A*v_i²` in hover and induced power `P_i=T*v_i=T^(3/2)/sqrt(2*rho*A)` [P19]. For separated rotors, sum each rotor's contribution using its own disk area and thrust. This lower-order estimate excludes profile power, motor/ESC loss, swirl/tip losses, interference, and unsteady inflow. It gives dimensionally useful scaling, not an aircraft power specification.

The GymPyBulletDrones ground-effect addition is `Delta f_i=k_G f_i (r_P/(4*h_i))²` with experimentally fitted coefficients and minimum-height guards [P03, P21]. Here `h_i` is rotor-plane height above the local surface, not generic world altitude. The unclipped formula diverges as height approaches zero and does not represent table edges, sloping roofs, vegetation, all attitudes, or rapid unsteady landing. The paper's fitted downwash force is a vertical Gaussian footprint with an inverse-square vertical-separation factor; its authors explicitly report missing pitch/lateral response. Copying Crazyflie coefficients to the present 1.5 kg vehicle is unjustified.

Neural Lander provides a stronger example of data-driven residual modeling with Lipschitz-constrained networks and bounded-error stability analysis [P22]. Its bounded approximation-error assumption still requires evidence for the actual operating domain. Spectral normalization bounds network sensitivity; it does not prove the real aerodynamic residual remains close to the network on an unseen roof, payload, or propeller. A low-cost next tier is an identified body-axis drag model plus thrust-map/lag tests before any high-dimensional learned residual or CFD-derived table.

### Wind is not one interchangeable feature

The present constant wind vector is a reproducible disturbance fixture. Gazebo's documented `WindEffects` is a uniform-world filtered/noisy wind with approximate link forces and optional spatial scaling [P12]. JSBSim contains military-specification turbulence filters whose scales depend on height and speed [P18]. Its inspected code uses `tau=L/V`, includes imperial-unit assumptions and minimum-height clipping, and disables this branch at exactly zero relative speed. A direct transplant would thus mishandle calm-air hovering turbulence. Urban recirculation, gust fronts, wakes behind buildings, and correlated disturbances across a 30-vehicle formation require separate spatial assumptions and measurements.

A useful **proposed stochastic test fixture**, not a claim of Dryden turbulence, is a stationary colored process

\[
z_{k+1}=e^{-\Delta t/\tau}z_k+\sigma\sqrt{1-e^{-2\Delta t/\tau}}\,\xi_k,
\qquad \xi_k\sim\mathcal N(0,1).
\]

This preserves variance `sigma²` when the timestep changes. Label its correlation time and amplitude explicitly. Compare separate common gusts, spatially correlated gusts, and independent local residuals; assigning independent white noise to every drone does not model a coherent gust. Store seeds and evaluate the same continuous disturbance realization when comparing timesteps.

### Contact interpretation

Panda3D/Bullet resolves rigid-body dynamics and contacts [P05-P07]. The present sphere envelope is useful for conservative body separation, ground rest, and failed-body impact detection. It omits feet, flexible arms, propeller strikes, breakage, battery ejection, and terrain compliance. A sphere landing at COM height 0.35 m is a property of this proxy geometry, not the aircraft's measured landing-gear height.

The configured CCD sweeps a 0.28 m sphere while the discrete collision sphere has radius 0.35 m. Official docs say CCD uses the swept sphere instead of the full shape [P07]. This warrants thin-obstacle/high-speed and glancing-contact fixtures; enabling CCD is not a universal no-tunneling proof. Impact impulse and penetration/settling metrics should be compared across timesteps and solver settings. A zero-restitution contact can dissipate energy intentionally; do not use post-impact energy conservation as the acceptance criterion.

## Sensors and state estimation

For an aligned IMU at the center of mass, a basic measurement model is

\[
\tilde\omega=\omega+b_g+n_g,\qquad
\tilde a=R^T(\dot v-g_W)+b_a+n_a,\quad g_W=-g e_3.
\]

An accelerometer measures specific force, so an ideal resting upright IMU reads approximately +g along its body vertical while an ideal free-falling IMU reads zero. Off-center mounting adds `dot omega cross r + omega cross (omega cross r)` in body coordinates; sensor orientation/extrinsics add another rotation. Current `PhysicsObservation.acceleration` is world finite-difference velocity acceleration, not an IMU output.

Kalibr distinguishes continuous noise density from discrete sample deviation [P11]. Under its stated ideal filtering assumption, white-noise deviation scales as `sigma/sqrt(dt)` and a bias random-walk increment as `sigma_b*sqrt(dt)`. A constant per-sample deviation while changing the sensor rate silently changes the noise spectrum. Static Allan-deviation calibration does not cover temperature drift, vibration, clipping, misalignment, or all motion-dependent errors.

PX4 EKF2 provides concrete delay handling: sensor FIFO buffers, a delayed fusion horizon, and propagation to the present using buffered IMU samples [P09]. A proposed sensor layer should carry acquisition, delivery, and estimator-use timestamps separately, with configurable rate, delay, jitter, dropout, saturation, and bias. Controller inputs must be estimates from those measurements when claiming estimator robustness; retain hidden truth only for scoring. GNSS multipath and image occlusion are structured failures, not always zero-mean Gaussian noise. The ideal world-down camera also removes real attitude/perception coupling demonstrated in PAMPC [P20].

## Energy and charging

Use dimensional identities before importing a battery model:

\[
E=\int V(t)I(t)\,dt,\qquad
1\,\mathrm{Ah}=3600\,\mathrm C,\qquad
1\,\mathrm{Wh}=3600\,\mathrm J.
\]

With constant voltage, `E[J]=3600*V[V]*Q[Ah]`; with changing voltage, integrate. ProjectAirSim's official battery example incorrectly equates Ah directly with joules [P14]. Its own text explicitly says the sensor targets mission planning rather than battery modeling. Its simplified torque-times-speed estimate has correct power dimensions when summed per rotor: `P_mech=sum_i Q_i*Omega_i` in W; electrical power additionally needs efficiency and auxiliary loads.

Zeng et al.'s popular rotary-wing curve separates profile, induced, and parasite power [P16]:

\[
P(V)=P_0(1+3V^2/U_{tip}^2)
+P_i\left(\sqrt{1+V^4/(4v_0^4)}-V^2/(2v_0^2)\right)^{1/2}
+\tfrac12 d_0\rho s A V^3.
\]

All terms are watts. Their study explicitly neglects extra acceleration energy and assumes constant-altitude mission motion. It supports the distinction between minimum-power endurance speed and minimum-`P/V` range speed. It does not justify one universal cruise speed for takeoff, braking, wind, relay hovering, and landing. Use air-relative speed for aerodynamic modeling and world-relative speed for progress toward home.

A proposed identified electrical tier can use state of charge `z`, terminal voltage `V_t`, and one polarization state `V_p`:

\[
\dot z=-I/Q_C,\qquad
\dot V_p=-V_p/(R_1C_1)+I/C_1,\qquad
V_t=OCV(z,T)-I R_0-V_p.
\]

Here `Q_C` is capacity in coulombs; alternatively use `3600*Q_Ah` in the denominator. This is a generic Thevenin candidate, not a claim that the exact equation is the NASA implementation. NASA ProgPy provides an inspectable, more detailed circuit/temperature model and an electrochemical alternative [P24]. Parameters, valid current/temperature/SOC ranges, and pack series/parallel structure must come from the selected battery. A fixed 1200 s budget remains valid as an openly declared scheduling constraint, but should not be labeled voltage/thermal prediction.

For charging, distinguish setup/contact time, any preconditioning, constant-current duration, constant-voltage taper, temperature limits, and completion criteria [P25]. Treat pad availability and power limits as queue/resource constraints. The TI note's numerical cell values are examples, not settings for an unspecified pack. A battery-swap fixture is a separate operational assumption and should not be represented as physically charging an exhausted pack instantly.

## Deeper source and implementation checks

### Force frames, inertia, and allocation

Panda3D 1.10.16 passes `applyForce(force, position)` directly to Bullet [P26]. Bullet forms `relative_position cross force` in its force accumulator. AeroMesh rotates both the body thrust axis and arm offsets into world coordinates before applying them; `R*r cross R*F = R*(r cross F)` gives the correct world moment. It then maps world angular velocity back to body coordinates before applying diagonal-body-inertia feedback. The isotropic angular drag is applied in world coordinates and is frame-consistent. No sign mismatch has been found by this source inspection.

For the motor order `(+,+),(-,+),(-,-),(+,-)` and `a=arm_length/sqrt(2)`, the allocation is

\[
\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}
=\begin{bmatrix}1&1&1&1\\a&a&-a&-a\\-a&a&a&-a\\\kappa&-\kappa&\kappa&-\kappa\end{bmatrix}
\begin{bmatrix}f_0\\f_1\\f_2\\f_3\end{bmatrix}.
\]

The current principal inertias are positive and satisfy the triangle inequalities, a necessary sanity check. They are not the inertia of a uniform sphere of collision radius 0.35 m: collision geometry and mass distribution are intentionally separate approximations. Panda3D warns that setting mass, adding/removing collision shapes, or scaling the physical body after `setInertia` can overwrite the explicit inertia [P26]. The inspected constructor sets shape and mass first, and the viewer attaches visual children. Future model replacements need to preserve this ordering and avoid scaling the physical parent.

### Four distinct clocks

1. The mission planner produces a reference once per outer simulator interval.
2. The safety projection and sampled geometric link availability/separation/geofence accounting run every 0.05 s (20 Hz).
3. The velocity PI, attitude/rate feedback, thrust lag, forces, and Bullet step run every 1/120 s.
4. Full per-body telemetry is written at 1 Hz; the safety ledger is 20 Hz and engine extrema/contacts are accumulated at 120 Hz.

This is not a single 120 Hz mission-safety loop. In particular, pairwise separation is calculated in `PhysicsUAVXSimulator._move` at 20 Hz. The engine's 120 Hz extrema track speed, angular speed, tilt, minimum COM height, and contacts; they do not independently calculate the mission's 20 m pairwise separation threshold. A physical body-contact test is not equivalent to a 20 m separation check.

Actual packet forwarding/service runs on the outer mission tick (0.5 s in the nominal scenario). A 20 Hz range/connectivity sample does not mean packets or acknowledgments are delivered at 20 Hz; the swarm/network report analyzes that separate service clock.

The projection considers pairs within 34 m, makes three sequential passes using `relative_command_velocity dot direction >= -0.7*(distance-22)`, then applies geofence component clamps and a final 4 m/s norm limit. It uses observed positions but **does not use observed velocity, tilt, or rotor state to compute braking feasibility**. Later projections/clamps can alter a constraint satisfied earlier. This is an inspectable heuristic with successful scenario evidence, not a CBF-QP implementation. The held 20 Hz reference is then tracked by the lagged 120 Hz flight cascade. A 0.5 s stale-command timeout is also distinct from either update period.

As a conditional sampling calculation, if each actual vehicle speed were continuously bounded by 4 m/s, relative distance could change by at most 0.4 m over a 50 ms interval, or by 0.2 m from the nearest endpoint. The code's request limit does not establish that premise under arbitrary wind/contact. Therefore do not use this calculation to certify the existing runs. A finer research observer or a validated swept-distance bound is needed to quantify between-sample separation.

### Camera geometry is internally explicit but idealized

The inherited camera uses a 60-degree vertical FOV and aspect-correct horizontal FOV, near/far clips 0.1/1000 m, zero distortion, and an ideal pinhole model. Its pixels use a top-left origin after an explicit RAM-row flip. The optical transform has world rotation `diag(1,-1,-1)` and translation at the measured COM. Hence right in the image is +world-X, down is -world-Y, and forward is -world-Z. For a flat z=0 surface, the center ray lands directly beneath the COM and the vertical footprint is `2*h*tan(30 degrees)`.

For a 128-pixel-tall image, nominal ground sampling distance is `2*h*tan(30 degrees)/128` m/pixel on that plane: approximately 0.180 m/pixel at h=20 m and 0.541 m/pixel at h=60 m (calculated ideal geometry). Buildings change range and occlusion; attitude has no effect in this stabilized camera. The decorative front camera in the viewer does not define this sensor's extrinsics. Image-byte integrity and metadata reconstruction establish traceability of this rendering pipeline, not physical camera calibration, exposure, gimbal dynamics, or perception accuracy.

The rendered scene is also separate from the integrated physical/RF world. `camera_sensor.py:88` explicitly selects CPU TinyDisplay, creates an independent `disaster-scene` root at line 104, and builds visual roads, terrain and parcels from line 169. `physics_uavx.py:45` constructs `QuadrotorWorld(homes, config, wind)` and makes no `add_box` calls. The world constructor creates a ground plane and drone spheres; its optional static-box API is exercised separately, not populated from these parcels in the mission. `comm/mesh_router.py:138-152` evaluates links from positions and its RF-channel model without camera-parcel geometry. A visible wall therefore is not evidence of a corresponding collision or radio obstruction. Current approximately 40 m mission flight above approximately 5 m parcels may not reveal that mismatch.

A proposed small shared scene manifest should identify each object's coordinate frame, transform, emergence/lifetime, visual geometry, collision proxy, optical occlusion role and RF material/obstruction role. These roles need not use identical meshes, but their differences must be explicit and testable. Start with a single box/ray/impact fixture before city assets. Camera tests should include rectangular aspect ratios, center and oblique rays, pixel-center conventions, known landmark reprojection, occlusion, and body rotation with either fixed extrinsics or an explicitly stabilized gimbal. These tests are proposed, not completed here.

### Learned-model claims require their exact assumptions

The full Neural Lander analysis assumes bounded reference position/velocity/acceleration, a bound on one-step control change relative to tracking error, and bounded residual-model error over compact state/input sets. Its Theorem V.2 converges to an **error ball** whose radius depends on residual error; nonzero model error is not exact zero-error tracking. The paper motivates its timescale approximation with attitude updates above 100 Hz and motor-speed control above 5 kHz versus roughly 10 Hz position control. The current 120 Hz simulated thrust-control layer is not that same architecture. Its table-edge experiment collected an additional dataset and trained another model with x-y inputs, so it is not evidence of an unchanged ground model automatically generalizing to arbitrary terrain [P22].

NASA's turbulence report supplies a useful validation pattern: compare spectra, autocorrelation, means and RMS values across multiple long seeded sequences, not only a visually convincing gust trace [P28]. The report studies an F18 model at fixed-wing speeds. Its spectral methods transfer as methods; its aircraft-specific parameters do not.

## Completed isolated numerical probes

`physics_convergence_probe.py` is an isolated, opt-in research script. Its default invocation prints a plan and does not import or execute the physics engine. `--execute` requires a new output path, validates the frozen engine SHA-256, runs without a renderer, stores its source/platform/dependency provenance, and checks the engine hash again afterward. The authorized 32-trial sweep ran from 02:53:37 to 02:54:19 UTC on 2026-11-06; `physics-convergence-20261106-01.json` records `source_stable=true`. The probe SHA-256 was `17bff4e94d390a20a1b0a1770631ecc0a51bca64761a211867b9b945d09d13c7`. The nominal mission ran concurrently, so these wall timings are **contended execution durations, not clean throughput benchmarks**.

| Probe | Controlled variables | Measurements and falsifiable expectation |
|---|---|---|
| Free fall | No drag/thrust; same 0.5 s; 60/120/240/480 Hz | Compare `v_z=-gt` and `z=z0-gt²/2`. Velocity should match to floating-point tolerance; position error should diminish with timestep refinement. Record deviations rather than retuning the physics. |
| Thrust step | Held 5 N/rotor command for 0.2 s | Compare every final rotor state against the exact exponential; identifies motor-lag update/unit mistakes separately from the body integrator. |
| Rotor frame signs | Initial yaw 0 and 90 degrees, one differential step | Body-frame angular response should follow the same allocation signs and inertia-scaled magnitude. Initial attitude placement is declared; no pose corrections occur during flight. |
| Hover and lateral brake | Identical 20 Hz command times at each physics rate | Compare altitude drift, velocity traces, stopping distance, max tilt, settling, and actuator saturation. Fine-step agreement does not establish real-aircraft accuracy. |
| Stale command | Refresh to t=3.95 s then stop; TTL 0.5 s | Record first stale observation, first step receiving stale controller input, and stopping distance. Distinguish flag timestamp, application timestamp, and 20 Hz observation quantization. |
| Constant crosswind | Same 3 m/s vector at each rate | Compare steady velocity/drift and command saturation. This is a disturbance test of chosen drag constants, not a wind-calibration result. |
| Ground drop | Same sphere/mass/height/drag; multiple rates | Compare first contact, penetration, rest height, speed, and impulse statistics. Pointwise impact peaks need not converge smoothly; compare event-integrated behavior. |

The completed rates are 60, 120, 240 and 480 Hz. The following values come from the saved artifact rather than a rerun. Positive vertical errors mean above the exact continuous solution.

| Measurement | 60 Hz | 120 Hz (current default) | 240 Hz | 480 Hz |
|---|---:|---:|---:|---:|
| Freefall position error after 0.5 s, mm | -40.874 | -20.439 | -10.217 | -5.109 |
| Thrust-step vertical-velocity error after 0.2 s, m/s | +0.10199 | +0.05318 | +0.02714 | +0.01371 |
| Thrust-step vertical-position error after 0.2 s, mm | +19.983 | +10.099 | +5.065 | +2.538 |
| Hover final COM height after 8 s from 20 m, m | 19.96304 | 19.95802 | 19.95533 | 19.95403 |
| Maximum forward displacement after explicit 2 m/s command changes to zero at 4 s, m | 1.07691 | 1.09343 | 1.10172 | 1.10586 |
| Maximum forward displacement after 4 s in expired-command fixture, m | 4.70750 | 4.69713 | 4.69191 | 4.68924 |
| Drop penetration below radius-0.35 m ground-rest height, mm | 50.8 | 20.2 | 4.93 | approximately 0 |

The freefall velocity error remains within approximately 6.4e-6 m/s; its position error halves with timestep refinement, consistent with first-order position integration. The exact rotor exponential agrees to at most 1.8e-15 N, but that does **not** make the coupled body trajectory exact. For total commanded thrust `F=20 N`, zero initial thrust and no drag,

\[
v_z(T)=(F/m-g)T-(F/m)\tau(1-e^{-T/\tau}),
\]

\[
z(T)=z_0+\tfrac12(F/m-g)T^2-(F/m)\tau[T-\tau(1-e^{-T/\tau})].
\]

At `T=0.2 s`, `tau=0.035 s`, these are 0.239539302686 m/s and 19.993412791073 m. The implementation updates thrust to its endpoint and applies that value throughout the whole body step. This right-endpoint force approximation and Bullet's position integration explain why an exact actuator-state check coexists with a material impulse/trajectory error. A later research candidate is an interval-average thrust comparison, isolated from body integration; no production fix was made. Coarser hover happens to dip less in this startup fixture, so a visually closer hover height does not establish greater numerical accuracy.

The expired-command fixture last refreshes at 3.95 s and has a 0.5 s TTL. First stale controller inputs occur at 4.466667, 4.458333, 4.454167 and 4.452083 s respectively. The strict comparison and next physics tick account for the difference from 4.45 s. The reported travel starts at 4 s and includes the remaining freshness interval. At 8 s its velocity has a small reverse transient, about -0.126 m/s. This fixture establishes observed motion under its initial state and chosen model; 4.697 m is **not** a universal stopping-distance bound. The 3 m/s crosswind fixture produces approximately 1.422 m drift and 0.1383 m/s residual speed at 8 s across these rates. It is not yet an asymptotic wind-rejection result.

The unpowered drop starts at COM 3 m and uses default drag. All rates settle near the radius-0.35 m rest height with negligible final velocity, despite differing peak penetration and first-contact times. Contact-rest success therefore does not establish impact fidelity. One-step differential-rotor probes at initial yaw 0 and 90 degrees match the body-frame allocation signs and inertia-scaled angular response to about 1e-8 or better. Those are frame/units checks within one model, not a calibrated torque test.

### Nonprincipal-axis rotation and gyroscopic coupling

`physics_rotation_probe.py` ran eight trials from 02:56:47 to 02:56:52 UTC. The complete result is `physics-rotation-20261106-01.json`, with probe SHA-256 `7703b46c50d7609dab5eb3c3a155bf587e28c3256b9948bae95d8770a44c229c` and an unchanged engine hash. An angular-velocity write supplies the declared **initial condition only**: body `omega=(1,2,3)` rad/s, the configured inertia, motors off, zero drag, COM 20 m. No pose correction occurs during the 0.5 s free-flight trials.

The installed `get_bullet_version()` returns 284, matching Bullet 2.84. That tag's rigid-body source defaults to implicit-body gyroscopic force handling [P29]. The expected continuous torque-free initial derivative is `-J^-1(omega cross J omega)=(-4.8,2.4,0)` rad/s². Measured first-step derivatives converge from approximately `(-4.888,2.204,0)` at 60 Hz to `(-4.812,2.376,0)` at 480 Hz.

| Rotation measurement over 0.5 s | 60 Hz | 120 Hz | 240 Hz | 480 Hz |
|---|---:|---:|---:|---:|
| Torque-free maximum absolute relative energy error | 1.1045% | 0.5594% | 0.2808% | 0.1405% |
| Torque-free maximum world angular-momentum error, kg m²/s | 0.001449 | 0.0007304 | 0.0003664 | 0.0001835 |
| Ideal gyroscopic-cancellation maximum absolute relative energy error | 2.1573% | 1.1051% | 0.5594% | 0.2817% |

The second case injects world torque `R*(omega cross J omega)` directly each step. It isolates the algebraic cancellation from rotor allocation, saturation and motor lag; it is **not a validation of the actual motor/controller path**. The exact continuous model would retain constant body angular velocity, while the measured final x/y components converge toward (1,2). World angular momentum is not conserved under this applied external torque, so its nonzero change is not an error criterion in that case. A pure yaw or other principal-axis trial has `omega cross J omega=0` and cannot diagnose a cross-term bug. The nonprincipal-axis fixture supplies evidence missing from such a test.

## Further numerical studies proposed

Further proposed studies, not yet implemented or run, are a pairwise 120 Hz distance observer around approach/landing crossings; 10/20/40 Hz outer safety timing at a fixed fine physics step; 20-200 ms sensor/input delay with bounded jitter and loss bursts; independent mass/inertia/thrust/lag perturbations with correlated motor parameters; and failure during maximum bank or near a neighbor. These must log measured state limits and constraints, not just command feasibility. Hold the disturbance realization and reference timing constant across paired runs.

For smooth non-contact traces, report differences between h, h/2, and h/4 and an observed convergence order only when ratios are stable and above floating-point noise. For contacts, compare landing event time, cumulative impulse/rest state, and penetration envelope rather than forcing Richardson extrapolation across discontinuities. For stochastic tests, compare distributions and spectral statistics with uncertainty intervals. Separate parameter sensitivity from timestep sensitivity, so changing dt does not silently change controller, noise density, or fault timing.

## Verification, validation, and parameter identification

NASA-STD-7009B distinguishes code/solution verification from conceptual/empirical validation and ties both to intended use [P27]. The current tests and compact mission primarily support implementation/solution consistency for the declared model. There is no measured airframe dataset here from which to claim empirical aerodynamic, battery, sensor, or landing validation. This is a methodological comparison, not a claim of NASA compliance or a new procedural approval requirement.

A later identification campaign should fit mass and geometry independently, static thrust/torque maps versus measured rotor speed and supply voltage, upward/downward transient actuator response, principal inertia and center of mass, body/rotor drag across axes and speeds, and sensor bias/noise/extrinsics/time offsets. Fit on one dataset and validate on held-out trajectories, loads, and environmental conditions. Report parameter covariance or bounds and residual structure, not only a best-fit curve.

Hover alone cannot distinguish thrust scale from mass, nor identify lateral drag. A closed-loop step can confound motor lag with sensor/transport delay; rotor-speed/current observations or independently measured delays help identify them. An accurate fitted drag constant can absorb a biased attitude/velocity estimate and then fail on a different maneuver. More parameters are not automatically more fidelity when the measurements do not identify them. The priority is to measure the effects that change reserve, stopping, landing, and perception outcomes.

Uncertainty should be separated into numerical error, parameter uncertainty, model-form discrepancy, and stochastic variability. The compact pass cannot establish that a longer route with more saturation/traffic, unknown wind, delayed estimates, repeated charging, or hardware sensor/actuator errors remains within the same domain. The final full nominal run under `evidence/research-physics-nominal-20261106-01` completed at 2514 s with ten PoIs and all 30 aircraft landed, but failed continuous connectivity for 19.8 s (14/15 checks). The independent verifier accepted the evidence while retaining mission failure. See [the measured diagnosis](integrated-nominal-connectivity-diagnosis.md). Integrated crosswind and recoverable-outage cases were not launched because of limited free RAM during the active run; they remain unqualified.

### Propeller maps and identifiable power quantities

The UIUC Propeller Database supplies measured thrust and torque for named propellers and test conditions [P32]. Its definitions use rotor rate `n` in **revolutions per second**, diameter `D` in m, axial advance speed `V`, and air density `rho`:

\[
J_{adv}=V/(nD),\quad
C_T=T/(\rho n^2D^4),\quad
C_Q=Q/(\rho n^2D^5),\quad
C_P=P/(\rho n^3D^5)=2\pi C_Q.
\]

The last identity follows from shaft power `P=Q*Omega` and `Omega=2*pi*n`. For a constant coefficient at a declared operating point, `k_f=C_T*rho*D^4/(2*pi)^2` and `k_m=C_Q*rho*D^5/(2*pi)^2` when the state is rad/s. For RPM, divide by 60 before using `n`. This avoids mixing RPM, rad/s and revolutions/s coefficients. `J_adv` is advance ratio here, not the inertia matrix `J` used above.

These coefficients vary with propeller geometry, Reynolds number and advance conditions. Static tests identify hover-region thrust/torque; they do not determine oblique inflow, vehicle interference or dynamic descent response. The database's static Reynolds number uses the 75%-radius blade chord and local rotational velocity `0.75*Omega*R_prop`. A convenient propeller name or visually similar diameter does not justify importing an unrelated coefficient.

The site's propulsive efficiency is `eta=T*V/P`. At static hover `V=0`, that definition is zero even for a useful hover propeller. Hover figure of merit instead compares ideal induced power with measured shaft power. Electrical efficiency additionally depends on motor, ESC, voltage, current and auxiliary loads. Keep stand measurements of thrust, shaft torque, rotor speed and electrical input synchronized so a fit does not silently absorb one loss mechanism into another. No UIUC dataset archive or airframe parameters were imported during this research.

## Simulator selection and resource tradeoffs

A simulator name does not determine a fidelity level. Select the equations, numerical scheme, sensor interface, collision behavior and intended validation domain separately. A renderer can be replaced without replacing vehicle dynamics, provided coordinate frames, timestamps and authority remain explicit.

| Backend or reference | What the inspected source supplies | Material limitation for this task | Appropriate next use |
|---|---|---|---|
| Current Panda3D/Bullet model | Force-driven 6-DoF bodies, lagged bounded thrust, declared drag, rigid contacts, sparse ideal cameras | Unidentified vehicle parameters; truth feedback; simple sphere contact and wind; documented timestep error | Preserve the working mission baseline and improve its measurements and stated domain |
| GymPyBulletDrones [P03, P21] | Bullet with multirotor force models and optional fitted ground effect/drag/downwash | Crazyflie parameters and limited downwash response; features depend on chosen physics mode | Compare model equations and small identification fixtures; no automatic parameter transfer |
| RotorPy [P30] | Transparent Newton–Euler model, rotor-speed lag, body/rotor drag, blade-flapping moments, wind and sensor models | Requires measured coefficients; collision termination is not an impact solver; current feature-camera projection is not a city renderer | Reference for low-cost aerodynamic/sensor models and offline controller identification |
| Flightmare [P23] | Decoupled Unity rendering and configurable C++ quadrotor dynamics, including Euler/RK4 choices | Published dynamics-step and rendering rates use different workloads; not this contact/network/planner stack | Reference for a clean rendering/physics boundary and paired integrator studies |
| IAMAI ProjectAirSim Fast Physics [P13, P14] | Rigid-body/rotor approximation, drag surfaces, Unreal collision detection and simple response; mission battery options | Inspected upstream source is not the exact local binary; images and a battery sensor do not establish calibrated physics/charging | Optional later renderer/backend comparison only after the paused city/storage constraints are resolved |
| Crazyflow [P31] | Batched differentiable JAX dynamics and measured Crazyflie abstractions; MuJoCo/MJX geometry/render/contact queries | Kinematic scene bodies have no contact response in the JAX plant; Python/OS/GPU requirements differ from this environment | Large independent-world identification/RL experiments when those become a measured need |

RotorPy's current vehicle documentation explicitly ignores some frame lift and drag-imbalance moments and assumes wind length scales larger than the vehicle. Its richer equations therefore remain reduced models. Similarly, Crazyflow's official integration documentation states that new scene geometry has **no effect on drone dynamics**: drones are mocap bodies, and geometry synchronizes for `contacts()` or `render()`. Do not use the presence of MuJoCo or a collision flag as evidence of landing-impact dynamics.

Crazyflow's published README benchmark uses a Ryzen 9 7950X and RTX 4090, first-principles dynamics, and one drone per independent world. At 64 worlds it reports 3.3 million CPU steps/s versus 1.2 million GPU steps/s; at 1024, 9.2 versus 18.7 million; at 262144, 12.6 versus 914 million [P31]. This shows why GPU batching can become worthwhile at scale; it does not predict 30 coupled aircraft, their planner/radio work, contacts, or an RTX 3050. The documented GPU extra targets Linux x86-64/CUDA 12 and the README requires Python 3.11+, whereas the current runtime is Windows/Python 3.10. No replacement dependency was installed.

### CPU, GPU and storage budgets to measure

The previously observed short 30-body hover cost of approximately 0.252 wall seconds per simulated second measures one headless hover fixture. The integrated nominal mission has routing, matching, safety projections, evidence writing and occasional rendering; its elapsed time must be measured independently. The present convergence probe ran concurrently with that mission and supplies no clean throughput comparison. Future timing reports should separate startup/JIT compilation, warm steady stepping, planner p50/p95/p99 latency, rendering, image encoding, disk writing, CPU/RAM and GPU/VRAM usage. A requested playback multiplier is not an achieved real-time factor.

Keep one renderer and bounded render targets. Increasing physics frequency mainly costs CPU in the current engine; sending its unchanged workload to an RTX 3050 is not automatic. GPU rasterization, readback, PNG encoding and Python planner work have different bottlenecks. Record the actual GPU variant/VRAM and current free RAM before an experiment; do not treat shared graphics memory as dedicated VRAM or use another paper's hardware speed as a local estimate.

For uncompressed RGB8, storage throughput is `N*W*H*3*fps` bytes/s. These are **calculated scenarios, not measured or current capture rates**:

| Hypothetical continuous capture | Raw RGB throughput | Raw RGB over 2700 s |
|---|---:|---:|
| 30 cameras, 128×128, 5 fps | 7.373 MB/s | 19.907 GB |
| 30 cameras, 640×480, 10 fps | 276.480 MB/s | 746.496 GB |

Here MB and GB are decimal. Float32 depth adds four bytes per pixel, so RGB plus depth is 7/3 times the RGB-only estimate before metadata or duplicate buffers. Compression ratios depend on image content and codec; no fixed saving is assumed. The current mission captures sparse task imagery and should not be described as these continuous streams.

The practical order is to retain complete numerical/event evidence and required original images, bound preview rate and image queues, render only views with decision value, and use procedural geometry for visibility/contact fixtures. A low-detail preview can coexist with the original required capture. Cropping, resizing or compression used by perception or transport changes the actual workload and must be included in its evaluation. Large city assets and high-rate all-camera recording are deferred because their storage/compute value has not been demonstrated for this mission.

