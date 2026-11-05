"""Reproducible, simulator-only entry point. Hardware CLI is run_hardware.py."""
import argparse
import json
import random
from pathlib import Path
import numpy as np
from simulation.scenario_builder import ScenarioBuilder
from simulation.challenge_metrics import ChallengeMetricsReporter

BUILDERS = {
    1: ScenarioBuilder.scenario_1_baseline_survey,
    2: ScenarioBuilder.scenario_2_deep_canyon_los_blockage,
    3: ScenarioBuilder.scenario_3_dynamic_emergency_and_node_failure,
    4: ScenarioBuilder.scenario_4_continuous_swapping_endurance,
}


def run(scenario, seed, max_time, output):
    from core.models import Vector3D
    random.seed(seed)
    np.random.seed(seed)
    sim = BUILDERS[scenario]()
    output.mkdir(parents=True, exist_ok=False)
    next_sample = 0.0
    injected_emergency = injected_failure = False
    with (output / "trajectory.jsonl").open("w") as stream:
        while sim.mission_state.time < max_time:
            t = sim.mission_state.time
            if scenario == 3:
                if t >= 100 and not injected_emergency:
                    sim.event_manager.inject_high_priority_poi(Vector3D(800, 800, 0), priority=8)
                    injected_emergency = True
                if t >= 180 and not injected_failure:
                    sim.health_monitor.handle_unexpected_node_failure(1)
                    injected_failure = True
                    sim.next_routing_update = t
            sim.step()
            if sim.mission_state.time >= next_sample:
                state = sim.mission_state
                stream.write(json.dumps({
                    "time_s": state.time, "coverage_percent": state.calculate_coverage(),
                    "uavs": [{"id": u.id, "position": [u.position.x, u.position.y, u.position.z],
                              "battery": u.battery_level, "role": u.role.value,
                              "connected": u.connected_to_gcs} for u in state.uavs.values()],
                    "pois": [{"id": p.id, "position": [p.position.x, p.position.y], "surveyed": p.surveyed} for p in state.pois.values()],
                    "links": list(sim.mesh_router.network_graph.edges()),
                }) + "\n")
                next_sample += 1.0
            if sim.mission_state.mission_complete and (scenario != 3 or injected_failure):
                break
    report = ChallengeMetricsReporter(sim.mission_state).generate_full_report()
    report["run"] = {"scenario": scenario, "seed": seed, "time_limit_s": max_time,
                     "configuration": sim.config.to_dict(), "sample_scenario_compliant": False}
    (output / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    (output / "events.json").write_text(json.dumps(sim.mission_state.events, indent=2))
    (output / "connectivity.json").write_text(json.dumps(sim.connectivity_samples))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-pass", action="store_true", help="Exit 2 when a UAV-X constraint check fails")
    parser.add_argument("--profile", choices=["uavx", "legacy"], default="uavx")
    parser.add_argument("--fleet-size", type=int, default=30)
    parser.add_argument("--packet-loss", type=float, default=0.02)
    parser.add_argument("--disturbance", choices=["none", "outage", "failure"], default="none")
    parser.add_argument("--scenario", type=int, choices=BUILDERS, default=1)
    parser.add_argument("--benchmark-all", action="store_true")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-time", type=float, default=None)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.max_time = args.max_time if args.max_time is not None else (2700 if args.profile == "uavx" else 600)
    if not np.isfinite(args.max_time) or args.max_time <= 0:
        parser.error("--max-time must be positive and finite")
    if args.profile == "uavx":
        if args.benchmark_all:
            parser.error("--benchmark-all applies to --profile legacy")
        from simulation.uavx import UAVXSimulator
        try:
            sim = UAVXSimulator(seed=args.seed, fleet_size=args.fleet_size,
                                packet_loss=args.packet_loss, disturbance=args.disturbance)
            report = sim.run(args.output, args.max_time)
        except (ValueError, FileExistsError) as exc:
            parser.error(str(exc))
        print(json.dumps(report, indent=2, allow_nan=False))
        if args.require_pass and not report["sample_constraints_passed"]:
            raise SystemExit(2)
        return
    for scenario in BUILDERS if args.benchmark_all else [args.scenario]:
        report = run(scenario, args.seed, args.max_time, args.output / f"scenario-{scenario}")
        print(json.dumps({"scenario": scenario, "mission": report["mission"], "communication": report["communication"]}))


if __name__ == "__main__":
    main()
