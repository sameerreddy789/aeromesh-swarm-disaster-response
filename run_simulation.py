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
    parser.add_argument("--gui", action="store_true", help="Open the live AeroMesh UAV-X desktop simulation")
    parser.add_argument("--playback-speed", type=float, default=12., help="GUI wall-clock pacing; does not change flight limits")
    parser.add_argument("--repeat", action="store_true", help="Repeat GUI missions in separate numbered evidence folders")
    parser.add_argument("--auto-close", action="store_true", help="Close GUI after this run, useful for verification")
    parser.add_argument("--profile", choices=["uavx", "legacy"], default="uavx")
    parser.add_argument("--fleet-size", type=int, default=None,
                        help="Default 30 UAVs including spares; overrides are diagnostic runs")
    parser.add_argument("--packet-loss", type=float, default=None)
    parser.add_argument("--scenario-file", type=Path, help="Validated UAV-X JSON scenario with resources, sensing and faults")
    parser.add_argument("--disturbance", choices=["none", "outage", "failure"], default="none")
    parser.add_argument("--scenario", type=int, choices=BUILDERS, default=1)
    parser.add_argument("--benchmark-all", action="store_true")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--max-time", type=float, default=None)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.scenario_file and args.profile != 'uavx':
        parser.error('--scenario-file requires --profile uavx')
    if args.scenario_file and args.disturbance != 'none':
        parser.error('Configure faults inside --scenario-file; do not combine it with --disturbance')
    if (args.gui and args.profile != 'uavx') or ((args.repeat or args.auto_close) and not args.gui):
        parser.error('--gui/--repeat/--auto-close require the UAV-X GUI profile')
    if args.repeat and args.auto_close:
        parser.error('--repeat and --auto-close are mutually exclusive')
    if not np.isfinite(args.playback_speed) or not .25 <= args.playback_speed <= 100:
        parser.error('--playback-speed must be finite and between 0.25 and 100')
    args.max_time = args.max_time if args.max_time is not None else (2700 if args.profile == "uavx" else 600)
    if not np.isfinite(args.max_time) or args.max_time <= 0:
        parser.error("--max-time must be positive and finite")
    if args.profile == "uavx":
        if args.benchmark_all:
            parser.error("--benchmark-all applies to --profile legacy")
        from simulation.uavx import UAVXSimulator
        scenario_config = None
        if args.scenario_file:
            from dataclasses import replace
            from simulation.uavx_scenario import load_scenario
            from simulation.resilient_uavx import ResilientUAVXSimulator
            try:
                scenario_config = load_scenario(args.scenario_file)
                overrides = {name: getattr(args,name) for name in ('seed','fleet_size','packet_loss')
                             if getattr(args,name) is not None}
                scenario_config = replace(scenario_config, **overrides)
            except (ValueError, OSError) as exc:
                parser.error(str(exc))
        def make_simulator():
            if scenario_config is not None:
                return ResilientUAVXSimulator(scenario_config)
            return UAVXSimulator(seed=args.seed if args.seed is not None else 42,
                                 fleet_size=args.fleet_size if args.fleet_size is not None else 30,
                                 packet_loss=args.packet_loss if args.packet_loss is not None else .02,
                                 disturbance=args.disturbance)
        if args.gui:
            try:
                from visualization.uavx_gui import UAVXVisualizer
            except ImportError as exc:
                parser.error(f'GUI dependency unavailable: {exc}. Install requirements-gui.txt')
            viewer=UAVXVisualizer(speed=args.playback_speed)
            iteration=1
            try:
                while True:
                    folder=args.output if iteration==1 else args.output.with_name(f'{args.output.name}-{iteration:03d}')
                    sim=make_simulator()
                    report=sim.run(folder,args.max_time,observer=viewer)
                    print(json.dumps({'output':str(folder),'completion_time_s':report['completion_time_s'],
                                      'checks':report['checks']},indent=2),flush=True)
                    if not viewer.wait_at_end(repeat=args.repeat,auto_close=args.auto_close):
                        break
                    iteration+=1
            except (ValueError, FileExistsError, RuntimeError) as exc:
                parser.error(str(exc))
            finally:
                viewer.close()
            if args.require_pass and not report['sample_constraints_passed']:
                raise SystemExit(2)
            return
        try:
            sim = make_simulator()
            report = sim.run(args.output, args.max_time)
        except (ValueError, FileExistsError, RuntimeError) as exc:
            parser.error(str(exc))
        print(json.dumps(report, indent=2, allow_nan=False))
        if args.require_pass and not report["sample_constraints_passed"]:
            raise SystemExit(2)
        return
    for scenario in BUILDERS if args.benchmark_all else [args.scenario]:
        report = run(scenario, args.seed if args.seed is not None else 42, args.max_time, args.output / f"scenario-{scenario}")
        print(json.dumps({"scenario": scenario, "mission": report["mission"], "communication": report["communication"]}))


if __name__ == "__main__":
    main()
