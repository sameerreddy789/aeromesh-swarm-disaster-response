"""Opt-in headless research probes; never edits production or opens a renderer.

Default invocation prints the planned experiments. Coordinate CPU use before
passing --execute. These measurements test numerical consistency of an
uncalibrated model; they are not hardware qualification or mission certificates.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
from datetime import datetime, timezone


ROOT = Path(__file__).resolve().parents[2]
ENGINE = ROOT / "simulation" / "quadrotor_physics.py"
EXPECTED_ENGINE_SHA256 = "b81c3003f0a3845c17f3779d5396f422f37204ad56a9296138490a44dcc61074"
CASES = ("freefall", "hover", "velocity_brake", "expired_command", "motor_lag",
         "constant_wind", "touchdown", "frame_signs")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def planned() -> dict:
    return {
        "scope": "Numerical consistency and falsification probes; research only",
        "production_edits": False,
        "renderer": False,
        "default_rates_hz": [60, 120, 240, 480],
        "cases": {
            "freefall": "0.5 s, no drag or thrust; compare vz and z with analytic gravity",
            "hover": "8 s upright hover, command at 20 Hz; compare trace and steady drift",
            "velocity_brake": "1 s hover, 3 s at 2 m/s, 4 s zero velocity; measure overshoot",
            "expired_command": "4 s at 4 m/s, then no refresh for 4 s; measure delay and stopping",
            "motor_lag": "0.2 s held 5 N-per-rotor override; compare exact exponential thrust",
            "constant_wind": "8 s hover in 3 m/s crosswind; compare drift and steady error",
            "touchdown": "Unpowered drop from COM 3 m onto plane; compare impulse/settling",
            "frame_signs": "One raw-thrust step at initial yaw 0 and 90 deg; compare body torque signs",
        },
        "interpretation": [
            "Compare identical commands at exact physical times, independent of integrator rate.",
            "Report errors and nonconvergence; do not retune gains or tolerances after seeing results.",
            "Contact impulses may not converge pointwise; compare event time, total response and rest state.",
            "No noisy sensors, estimated state, downwash, urban wind, battery circuit, or physical data.",
        ],
    }


def run_case(name: str, rate: int) -> dict:
    import numpy as np
    from simulation.quadrotor_physics import PhysicsConfig, QuadrotorWorld

    cfg_args = {"fixed_dt": 1.0 / rate}
    if name in ("freefall", "motor_lag", "frame_signs"):
        cfg_args.update(linear_drag_n_per_mps=0.0,
                        quadratic_drag_n_per_mps2=0.0, angular_drag_nms=0.0)
    cfg = PhysicsConfig(**cfg_args)
    if name == "frame_signs":
        from panda3d.core import Quat, Vec3
        rows = []
        for yaw_deg in (0.0, 90.0):
            world = QuadrotorWorld({0: (0.0, 0.0, 20.0)}, cfg)
            try:
                # Initial condition only, before any step or applied command.
                q0 = Quat()
                q0.setFromAxisAngle(yaw_deg, Vec3(0, 0, 1))
                world.bodies[0].setQuat(q0)
                world.command(0, armed=True)
                world.set_motor_override(0, (5.0, 3.0, 3.0, 3.0))
                world.step(cfg.fixed_dt)
                obs = world.observe(0)
                R = world._rotations(np.array([obs.quaternion_wxyz]))[0]
                omega_body = R.T @ np.array(obs.angular_velocity_world)
                f = np.array(obs.rotor_thrusts_n)
                arm = cfg.arm_length_m / math.sqrt(2.0)
                predicted_tau_body = np.array([
                    arm * (f[0] + f[1] - f[2] - f[3]),
                    arm * (-f[0] + f[1] + f[2] - f[3]),
                    cfg.yaw_torque_per_thrust_m * (f[0] - f[1] + f[2] - f[3]),
                ])
                rows.append({"initial_yaw_deg": yaw_deg,
                             "measured_omega_body_rps": omega_body.tolist(),
                             "predicted_first_step_omega_body_rps":
                                 (cfg.fixed_dt * predicted_tau_body /
                                  np.array(cfg.inertia_kg_m2)).tolist(),
                             "actual_inertia_kg_m2": list(world.bodies[0].node().getInertia()),
                             "initial_condition_only_pose_write": True})
            finally:
                world.close()
        return {"case": name, "rate_hz": rate, "config": asdict(cfg), "rows": rows}

    duration = 0.5 if name == "freefall" else 0.2 if name == "motor_lag" else 8.0
    if name == "touchdown":
        duration = 4.0
    start_z = 3.0 if name == "touchdown" else 20.0
    wind = (3.0, 0.0, 0.0) if name == "constant_wind" else (0.0, 0.0, 0.0)
    world = QuadrotorWorld({0: (0.0, 0.0, start_z)}, cfg, wind)
    trace, first_contact, brake_x = [], None, None
    max_x_after_brake, first_stale, first_stale_input = None, None, None
    start_wall = time.perf_counter()
    try:
        if name == "motor_lag":
            world.command(0, armed=True)
            world.set_motor_override(0, (5.0, 5.0, 5.0, 5.0))
        command_stride = rate // 20
        for k in range(round(duration * rate)):
            t = k / rate
            if k % command_stride == 0 and name not in ("freefall", "touchdown", "motor_lag"):
                velocity = (0.0, 0.0, 0.0)
                if name == "velocity_brake" and 1.0 <= t < 4.0:
                    velocity = (2.0, 0.0, 0.0)
                if name == "expired_command":
                    velocity = (4.0, 0.0, 0.0)
                if name != "expired_command" or t < 4.0:
                    world.command(0, velocity=velocity, armed=True)
            if k == 4 * rate and name in ("velocity_brake", "expired_command"):
                brake_x = world.observe(0).position[0]
                max_x_after_brake = brake_x
            before_step = world.observe(0)
            if name == "expired_command" and before_step.command_stale and first_stale_input is None:
                first_stale_input = before_step.time_s
            world.step(cfg.fixed_dt)
            obs = world.observe(0)
            if name == "expired_command" and obs.command_stale and first_stale is None:
                first_stale = obs.time_s
            if max_x_after_brake is not None:
                max_x_after_brake = max(max_x_after_brake, obs.position[0])
            if obs.ground_contact and first_contact is None:
                first_contact = obs.time_s
            # Common 20 Hz trace for cross-rate comparisons, plus a final sample.
            if (k + 1) % command_stride == 0 or k + 1 == round(duration * rate):
                trace.append({"time_s": obs.time_s, "position_m": list(obs.position),
                              "velocity_mps": list(obs.velocity),
                              "quaternion_wxyz": list(obs.quaternion_wxyz),
                              "thrusts_n": list(obs.rotor_thrusts_n),
                              "ground_contact": obs.ground_contact,
                              "command_stale": obs.command_stale})
        obs = world.observe(0)
        metrics = {"duration_s": duration, "final_position_m": list(obs.position),
                   "final_velocity_mps": list(obs.velocity),
                   "first_contact_s": first_contact,
                   "model_stats": world.stats(),
                   "wall_s": time.perf_counter() - start_wall}
        if name == "freefall":
            metrics["analytic_velocity_z_mps"] = -cfg.gravity_mps2 * duration
            metrics["analytic_position_z_m"] = start_z - 0.5 * cfg.gravity_mps2 * duration**2
            metrics["velocity_z_error_mps"] = obs.velocity[2] - metrics["analytic_velocity_z_mps"]
            metrics["position_z_error_m"] = obs.position[2] - metrics["analytic_position_z_m"]
        if name == "motor_lag":
            expected = 5.0 * (1.0 - math.exp(-duration / cfg.motor_tau_s))
            metrics["analytic_final_rotor_thrust_n"] = expected
            metrics["max_thrust_error_n"] = max(abs(f - expected) for f in obs.rotor_thrusts_n)
        if brake_x is not None:
            after = [row for row in trace if row["time_s"] >= 4.0]
            metrics["distance_after_4s_m"] = max_x_after_brake - brake_x
            metrics["distance_after_4s_from_20hz_samples_m"] = max(row["position_m"][0] for row in after) - brake_x
            metrics["brake_reference_time_s"] = 4.0
            if name == "expired_command":
                metrics["last_refreshed_command_time_s"] = 3.95
                metrics["first_physics_observed_stale_time_s"] = first_stale
                metrics["first_stale_controller_input_time_s"] = first_stale_input
                metrics["first_observed_stale_time_s"] = next(
                    (row["time_s"] for row in trace if row["command_stale"]), None)
        return {"case": name, "rate_hz": rate, "config": asdict(cfg),
                "metrics": metrics, "trace_20hz": trace}
    finally:
        world.close()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--execute", action="store_true")
    p.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    p.add_argument("--rates", nargs="+", type=int, default=[60, 120, 240, 480])
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    if not args.execute:
        print(json.dumps(planned(), indent=2))
        return
    if not args.output:
        p.error("--execute requires a new --output path")
    if args.output.exists():
        p.error("output already exists; use a new research result path")
    if any(rate < 60 or rate > 960 or rate % 20 for rate in args.rates):
        p.error("rates must be multiples of 20 from 60 to 960 Hz")
    before = digest(ENGINE)
    if before != EXPECTED_ENGINE_SHA256:
        p.error("production engine differs from the frozen research snapshot")
    sys.path.insert(0, str(ROOT))
    from importlib.metadata import version
    payload = {"started_utc": datetime.now(timezone.utc).isoformat(),
               "python": platform.python_version(), "platform": platform.platform(),
               "panda3d": version("panda3d"), "engine_sha256": before,
               "probe_sha256": digest(Path(__file__)),
               "wall_timing_scope": "Contended host: nominal mission running concurrently; not a clean throughput benchmark",
               "plan": planned(), "results": []}
    for name in args.cases:
        for rate in args.rates:
            payload["results"].append(run_case(name, rate))
            print(f"Completed {name} at {rate} Hz", flush=True)
    payload["source_stable"] = digest(ENGINE) == before
    payload["completed_utc"] = datetime.now(timezone.utc).isoformat()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
