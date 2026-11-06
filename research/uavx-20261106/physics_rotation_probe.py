"""Opt-in numerical rotation research; initial-state injection only, no renderer."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
ENGINE = ROOT / "simulation/quadrotor_physics.py"
EXPECTED_SHA = "b81c3003f0a3845c17f3779d5396f422f37204ad56a9296138490a44dcc61074"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def trial(rate, compensate):
    import numpy as np
    from panda3d.core import Vec3
    from simulation.quadrotor_physics import PhysicsConfig, QuadrotorWorld

    config = PhysicsConfig(fixed_dt=1.0 / rate, linear_drag_n_per_mps=0,
                           quadratic_drag_n_per_mps2=0, angular_drag_nms=0)
    world = QuadrotorWorld({0: (0.0, 0.0, 20.0)}, config)
    J = np.array(config.inertia_kg_m2)
    omega0 = np.array([1.0, 2.0, 3.0])
    expected_dot = -np.cross(omega0, J * omega0) / J
    H0 = J * omega0
    E0 = 0.5 * np.dot(omega0, J * omega0)
    rows = []
    try:
        # Initialization before the first physics step, never a flight correction.
        world.bodies[0].node().setAngularVelocity(Vec3(*map(float, omega0)))
        for k in range(round(0.5 * rate)):
            observation = world.observe(0)
            rotation = world._rotations(np.array([observation.quaternion_wxyz]))[0]
            omega = rotation.T @ np.array(observation.angular_velocity_world)
            if compensate:
                # Inject the ideal continuous gyroscopic feed-forward torque to
                # isolate Bullet rotation integration from motor saturation/lag.
                torque = rotation @ np.cross(omega, J * omega)
                world.bodies[0].node().applyTorque(Vec3(*map(float, torque)))
            world.step(config.fixed_dt)
            observation = world.observe(0)
            rotation = world._rotations(np.array([observation.quaternion_wxyz]))[0]
            omega = rotation.T @ np.array(observation.angular_velocity_world)
            H = rotation @ (J * omega)
            energy = 0.5 * np.dot(omega, J * omega)
            rows.append({"time_s": observation.time_s, "omega_body_rps": omega.tolist(),
                         "world_momentum_kg_m2_rps": H.tolist(),
                         "rotation_energy_j": float(energy),
                         "momentum_error_norm": float(np.linalg.norm(H - H0)),
                         "relative_energy_error": float((energy - E0) / E0)})
        return {"rate_hz": rate, "case": "ideal_gyro_compensation" if compensate else "torque_free",
                "config": asdict(config), "initial_omega_body_rps": omega0.tolist(),
                "initial_condition_only_velocity_write": True,
                "external_torque_injection": bool(compensate),
                "expected_initial_body_accel_rps2": ([0, 0, 0] if compensate else expected_dot.tolist()),
                "measured_first_step_body_accel_rps2":
                    ((np.array(rows[0]["omega_body_rps"]) - omega0) / config.fixed_dt).tolist(),
                "max_relative_energy_error_abs": max(abs(r["relative_energy_error"]) for r in rows),
                "max_momentum_error_norm": max(r["momentum_error_norm"] for r in rows),
                "final": rows[-1], "trace": rows}
    finally:
        world.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.execute:
        print("Plan: 0.5 s torque-free nonprincipal-axis rotation and ideal gyro compensation at 60/120/240/480 Hz.")
        return
    if not args.output or args.output.exists():
        parser.error("--execute requires a new --output path")
    before = sha(ENGINE)
    if before != EXPECTED_SHA:
        parser.error("engine differs from frozen research source")
    sys.path.insert(0, str(ROOT))
    from panda3d.bullet import get_bullet_version
    output = {"started_utc": datetime.now(timezone.utc).isoformat(),
              "scope": "Numerical gyroscopic terms only; not actual controller or airframe validation",
              "wall_timing_scope": "Contended host; no throughput claim",
              "engine_sha256": before, "probe_sha256": sha(Path(__file__)),
              "bullet_version_integer": get_bullet_version(),
              "results": [trial(rate, compensate) for compensate in (False, True)
                          for rate in (60, 120, 240, 480)]}
    output["source_stable"] = sha(ENGINE) == before
    output["completed_utc"] = datetime.now(timezone.utc).isoformat()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as f:
        json.dump(output, f, indent=2, allow_nan=False)
        f.write("\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
