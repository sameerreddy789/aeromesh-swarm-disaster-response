"""Run the fixed 30-UAV Bullet mission with optional native 3D eagle view."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario-file', type=Path, default=Path(__file__).parent/'scenarios/uavx/nominal.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-time', type=float, default=2700.)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--auto-close', action='store_true')
    parser.add_argument('--playback-speed', type=float, default=12.)
    parser.add_argument('--wind', type=float, nargs=3, default=(0.,0.,0.), metavar=('X','Y','Z'))
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args()
    if args.auto_close and not args.gui:
        parser.error('--auto-close requires --gui')
    from simulation.physics_uavx import PhysicsUAVXSimulator
    from simulation.uavx_scenario import load_scenario
    sim = PhysicsUAVXSimulator(load_scenario(args.scenario_file), wind=args.wind)
    viewer = None
    try:
        if args.gui:
            from visualization.physics_viewer import PhysicsVisualizer
            viewer = PhysicsVisualizer(speed=args.playback_speed)
        report = sim.run(args.output, max_time=args.max_time, observer=viewer)
        print(json.dumps(dict(output=str(args.output.resolve()), elapsed_time_s=report['elapsed_time_s'],
                             completion_time_s=report['completion_time_s'], pois_completed=report['pois_completed'],
                             checks=report['checks'], physics=report['physics']), indent=2), flush=True)
        if viewer:
            viewer.wait_at_end(auto_close=args.auto_close)
        if args.require_pass and not report['sample_constraints_passed']:
            raise SystemExit(2)
    finally:
        if viewer: viewer.close()
        sim.close()


if __name__ == '__main__':
    main()
