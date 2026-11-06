"""Run every declared UAV-X development case and preserve failures in the summary.

Examples (from the repository root):
  .venv\\Scripts\\python.exe scripts/benchmark_uavx.py --output evidence/matrix-01
  .venv\\Scripts\\python.exe scripts/benchmark_uavx.py --output evidence/quick-01 --quick
  .venv\\Scripts\\python.exe scripts/benchmark_uavx.py --output evidence/seeds-01 --seeds 42 7 100,101

Exit 0 means every declared development case passed its implemented checks;
2 means one or more consistent cases failed constraints; 1 means setup, process,
evidence, interruption or source-stability failure. None implies submission readiness.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulation.uavx_scenario import FaultSpec, load_scenario
from verify_submission import SOURCES_V2, verify


DEFAULT_SEEDS = (42, 7, *range(100, 110))
COMPLETE_STATUSES = frozenset({'passed', 'constraints_failed'})
METRIC_FIELDS = (
    'evidence_scope', 'schema_version', 'seed', 'fleet_size', 'elapsed_time_s',
    'completion_time_s', 'pois_completed', 'pois_total', 'priority_weighted_score',
    'connectivity_availability_percent', 'communication_downtime_s',
    'minimum_separation_m', 'maximum_speed_mps', 'maximum_altitude_m',
    'maximum_sortie_s', 'battery_exhaustions', 'collision_count',
    'separation_violation_steps', 'geofence_violation_steps',
    'reporting_deadline_misses', 'relay_assignments', 'relay_handoffs_completed',
    'checks', 'sample_constraints_passed', 'survivor_metrics', 'faults', 'sensing',
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    """Include the engine inventory, validator, matrix definition and dependency pins."""
    names = SOURCES_V2 | {
        'verify_submission.py', 'scripts/benchmark_uavx.py',
        'verification/__init__.py', 'verification/uavx_extended.py',
        'scenarios/uavx/nominal.json', 'requirements-uavx.txt',
        'requirements-sensing.txt',
    }
    return {name: digest(ROOT / name) for name in sorted(names)}


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.' + uuid4().hex + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def parse_seeds(values):
    """Accept space- or comma-separated integers, preserving declared order."""
    if values is None:
        return DEFAULT_SEEDS
    try:
        seeds = tuple(int(part) for value in values for part in value.split(','))
    except ValueError as exc:
        raise ValueError('--seeds must contain integers separated by spaces or commas') from exc
    if not seeds or len(set(seeds)) != len(seeds):
        raise ValueError('--seeds must be nonempty and must not contain duplicate seeds')
    return seeds


def build_matrix(base, seeds=DEFAULT_SEEDS, quick=False):
    """Return named configs; no case is removed merely because it is expected to fail."""
    if base.fleet_size != 30:
        raise ValueError('The qualification matrix requires exactly 30 UAVs, including spares')
    def nominal(seed):
        return replace(base, seed=seed, faults=())

    def fault_case(kind, at=900.):
        selector = 'middle_active_link' if kind == 'link_outage' else 'middle_active_relay'
        duration = None if kind == 'vehicle_failure' else 30.
        return replace(nominal(42), faults=(FaultSpec(kind, at, selector, duration),))

    if quick:
        return [
            ('nominal-seed42', nominal(42)), ('nominal-seed7', nominal(7)),
            ('link-outage-seed42', fault_case('link_outage')),
            ('permanent-failure-seed42', fault_case('vehicle_failure')),
        ]
    # Resource pressure changes charging capacity, never the frozen total fleet.
    limited_pads = max(1, (base.charging_pads or base.fleet_size) // 2)
    return [
        *[(f'nominal-seed{seed}', nominal(seed)) for seed in seeds],
        ('link-outage-seed42', fault_case('link_outage')),
        ('radio-outage-seed42', fault_case('radio_outage')),
        ('permanent-failure-seed42', fault_case('vehicle_failure')),
        (f'charging-pads{limited_pads}-seed42', replace(nominal(42), charging_pads=limited_pads)),
        ('early-link-seed42', fault_case('link_outage', 180.)),
        ('late-link-seed42', fault_case('link_outage', 1600.)),
        ('packet-loss10pct-seed42', replace(nominal(42), packet_loss=.10)),
    ]


def compact_metrics(metrics):
    compact = {key: metrics[key] for key in METRIC_FIELDS if key in metrics}
    network = metrics.get('packet_statistics', {})
    compact['packet_statistics'] = {key: value for key, value in network.items() if key != 'radio_service'}
    if 'radio_service' in network:
        # Per-node byte maps stay in the complete run's metrics.json.
        compact['radio_service'] = {key: value for key, value in network['radio_service'].items()
                                    if key not in ('node_service_bytes', 'node_tx_bytes', 'node_rx_bytes',
                                                   'queue_bytes', 'peak_queue_bytes')}
    return compact


def evaluate_case(output, row, process_registry, lock, stop_event, publish):
    case_start = time.monotonic()
    case_dir = output / row['case']
    run_dir = case_dir / 'run'
    scenario_path = output / row['scenario_file']
    log_path = case_dir / 'console.txt'
    result = {'started_at_utc': utc_now(), 'status': 'running'}
    if stop_event.is_set():
        return {'status': 'cancelled', 'error': 'Cancelled before subprocess launch', 'wall_time_s': 0.}
    publish(row['case'], result)
    process = None
    try:
        if digest(scenario_path) != row['scenario_sha256']:
            raise ValueError('Scenario input changed after matrix preparation')
        command = [sys.executable, str(ROOT / 'run_simulation.py'), '--profile', 'uavx',
                   '--scenario-file', str(scenario_path), '--output', str(run_dir)]
        result['command'] = command
        publish(row['case'], {'command': command})
        with log_path.open('wb') as log:
            process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            with lock:
                process_registry[row['case']] = process
            if stop_event.is_set():
                process.terminate()
            result['exit_code'] = process.wait()
        with lock:
            process_registry.pop(row['case'], None)
        if stop_event.is_set():
            result.update(status='cancelled', error='Interrupted by user; partial evidence preserved')
            return result
        if result['exit_code'] != 0:
            result.update(status='crashed', error=f'Simulation subprocess exited {result["exit_code"]}; inspect console.txt')
        metrics_path = run_dir / 'metrics.json'
        if metrics_path.exists():
            metrics = json.loads(metrics_path.read_text(encoding='utf-8'))
            if not isinstance(metrics, dict):
                raise ValueError('metrics.json is not a JSON object')
            result['metrics'] = compact_metrics(metrics)
            result['metrics_sha256'] = digest(metrics_path)
        else:
            result.update(status='crashed', error='Simulation did not produce metrics.json; inspect console.txt')
        verification = verify(run_dir)
        manifest_path = run_dir / 'manifest.json'
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
            result['manifest_sha256'] = digest(manifest_path)
            recorded_sources = manifest.get('source_sha256', {})
            for name, expected in row['expected_runtime_source_sha256'].items():
                if recorded_sources.get(name) != expected:
                    verification['evidence_errors'].append(f'Run source differs from matrix start: {name}')
        if digest(scenario_path) != row['scenario_sha256']:
            verification['evidence_errors'].append('Matrix scenario input changed during execution')
        if (run_dir / 'scenario.json').exists():
            requested = load_scenario(scenario_path).to_dict()
            measured = load_scenario(run_dir / 'scenario.json').to_dict()
            # The runtime expands seeded task generation into an explicit task list.
            if requested['tasks'] is None:
                requested.pop('tasks')
                measured.pop('tasks')
            if requested != measured:
                verification['evidence_errors'].append('Run scenario does not match the declared matrix input')
        if verification['evidence_errors']:
            verification['sample_run_passed'] = False
        result['verification'] = verification
        atomic_json(case_dir / 'verification.json', verification)
        if result.get('status') != 'crashed':
            result['status'] = ('evidence_error' if verification['evidence_errors'] else
                                'passed' if verification['sample_run_passed'] else 'constraints_failed')
        return result
    except Exception as exc:
        result.update(status='crashed', error=f'{type(exc).__name__}: {exc}')
        return result
    finally:
        if process is not None:
            with lock:
                process_registry.pop(row['case'], None)
        result['finished_at_utc'] = utc_now()
        result['wall_time_s'] = round(time.monotonic() - case_start, 3)


def run_matrix(output, cases, workers):
    """Persist all planned cases before launching; atomically update on every outcome."""
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'scenarios').mkdir()
    initial_hashes = source_hashes()
    rows = []
    for name, scenario in cases:
        (output / name).mkdir()
        scenario_file = Path('scenarios') / (name + '.json')
        atomic_json(output / scenario_file, scenario.to_dict())
        rows.append({'case': name, 'status': 'queued', 'scenario_file': scenario_file.as_posix(),
                     'scenario_sha256': digest(output / scenario_file),
                     'run_directory': f'{name}/run', 'console_log': f'{name}/console.txt',
                     'verification_file': f'{name}/verification.json'})
    summary = {
        'schema_version': 1, 'started_at_utc': utc_now(), 'finished_at_utc': None,
        'scope': 'Declared development benchmarks; all attempts retained. Not hidden organizer scenarios or submission qualification.',
        'python_executable': sys.executable, 'workers': workers,
        'source_sha256_start': initial_hashes, 'source_sha256_end': None,
        'source_changed': None, 'source_stable': None, 'submission_ready': False,
        'all_cases_finished': False, 'all_cases_verified': False, 'development_matrix_passed': False,
        'cases': rows,
    }
    summary_path = output / 'summary.json'
    lock = threading.RLock()
    registry, stop_event = {}, threading.Event()

    def publish(name, updates):
        with lock:
            next(row for row in rows if row['case'] == name).update(updates)
            atomic_json(summary_path, summary)

    atomic_json(summary_path, summary)
    print(f'Running {len(cases)} declared cases with {workers} worker(s). Evidence: {output}', flush=True)
    interrupted = False
    futures = {}
    pool = ThreadPoolExecutor(max_workers=workers)
    try:
        futures = {pool.submit(evaluate_case, output,
                               dict(row, expected_runtime_source_sha256={name: initial_hashes[name]
                                                                        for name in SOURCES_V2 if name in initial_hashes}),
                               registry, lock, stop_event, publish): row['case']
                   for row in rows}
        for number, future in enumerate(as_completed(futures), 1):
            name = futures[future]
            try:
                result = future.result()
            except Exception as exc:
                result = {'status': 'crashed', 'error': f'Worker failed: {type(exc).__name__}: {exc}'}
            publish(name, result)
            failed = result.get('verification', {}).get('failed_constraints', [])
            detail = ', '.join(failed) if failed else result.get('error', '')
            print(f'[{number}/{len(cases)}] {name}: {result["status"]}; '
                  f'{result.get("wall_time_s", 0):.1f} wall seconds'
                  + (f'; {detail}' if detail else ''), flush=True)
    except KeyboardInterrupt:
        interrupted = True
        stop_event.set()
        print('Interrupted: stopping only benchmark-owned subprocesses and preserving all partial evidence.', flush=True)
        with lock:
            processes = list(registry.values())
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
    finally:
        pool.shutdown(wait=True, cancel_futures=interrupted)
        for future, name in futures.items():
            if future.cancelled():
                publish(name, {'status': 'cancelled', 'error': 'Cancelled before worker start'})
            elif future.done():
                try:
                    publish(name, future.result())
                except Exception as exc:
                    publish(name, {'status': 'crashed', 'error': f'Worker failed: {type(exc).__name__}: {exc}'})
        try:
            final_hashes = source_hashes()
            changed = sorted(name for name in set(initial_hashes) | set(final_hashes)
                             if initial_hashes.get(name) != final_hashes.get(name))
            summary.update(source_sha256_end=final_hashes, source_changed=changed, source_stable=not changed)
        except OSError as exc:
            summary.update(source_stable=False, source_changed=['Source inventory unavailable'],
                           source_inventory_error=str(exc))
        summary['finished_at_utc'] = utc_now()
        summary['all_cases_finished'] = all(row['status'] not in ('queued', 'running') for row in rows)
        summary['all_cases_verified'] = all(row['status'] in COMPLETE_STATUSES for row in rows)
        summary['infrastructure_ok'] = (not interrupted and summary['all_cases_verified'] and summary['source_stable'])
        summary['development_matrix_passed'] = (summary['infrastructure_ok'] and
                                                all(row['status'] == 'passed' for row in rows))
        summary['case_counts'] = {status: sum(row['status'] == status for row in rows)
                                  for status in sorted({row['status'] for row in rows})}
        atomic_json(summary_path, summary)
    print(f'Complete matrix summary: {summary_path}', flush=True)
    print('Case outcomes: ' + ', '.join(f'{key}={value}' for key, value in summary['case_counts'].items()), flush=True)
    if not summary['infrastructure_ok']:
        print('Evidence is not qualified: inspect case console/verification files and source_changed in summary.json.', flush=True)
        return 1
    if not summary['development_matrix_passed']:
        print('Constraint failures are preserved. Review failed_constraints before making a challenge-compliance claim.', flush=True)
        return 2
    print('All declared development cases passed. Organizer validation and submission readiness remain separate.', flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True, help='New evidence directory; existing paths are rejected')
    parser.add_argument('--workers', type=int, choices=range(1, 5), default=2, help='Concurrent subprocesses (default: 2)')
    parser.add_argument('--quick', action='store_true', help='Only nominal 42/7, link outage 42 and permanent failure 42')
    parser.add_argument('--seeds', nargs='+', help='Nominal seeds, space/comma separated (default: 42 7 100 through 109)')
    args = parser.parse_args(argv)
    if args.quick and args.seeds is not None:
        parser.error('--quick has fixed seeds 42 and 7; omit --seeds or run the full matrix')
    try:
        seeds = parse_seeds(args.seeds)
        base = load_scenario(ROOT / 'scenarios/uavx/nominal.json')
        cases = build_matrix(base, seeds, args.quick)
        return run_matrix(args.output, cases, args.workers)
    except (OSError, ValueError) as exc:
        print(f'Benchmark setup failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
