"""Check UAV-X artifact integrity and internal consistency.

This is not an independent replay of full dynamics or tamperproof authority:
hashes and observations supplied by the same producer cannot establish either.
Submission readiness and physical flight qualification remain separate.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

ARTIFACTS = {'metrics.json', 'events.json', 'packets.json', 'connectivity.json', 'trajectory.jsonl'}
SOURCES = {'simulation/uavx.py', 'core/mission_safety.py', 'core/models.py',
           'core/config.py', 'comm/mesh_router.py', 'comm/channel.py',
           'comm/channel_legacy.py', 'run_simulation.py'}
SOURCES_V2 = SOURCES | {'simulation/resilient_uavx.py', 'simulation/uavx_scenario.py', 'simulation/camera_sensor.py'}
CHECKS_V2_EXTRA = {'scheduled_faults_exercised', 'imagery_received'}
REQUIREMENTS = dict(area_width_m=1000., area_height_m=1000., mission_duration_s=2700.,
                    max_flight_s=1200., radio_range_m=100., max_altitude_m=100.,
                    max_speed_mps=5., min_separation_m=20., report_deadline_s=10., poi_count=10)
CHECKS = {'all_pois_surveyed_and_reported', 'sample_has_ten_pois', 'all_uavs_landed',
          'mission_deadline', 'endurance', 'speed', 'altitude', 'separation', 'geofence',
          'no_battery_exhaustion', 'reporting_deadline', 'continuous_connectivity'}


def number(value, label, minimum=0., maximum=math.inf):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'Invalid measurement: {label}')
    return value


def integer(value, label, minimum=0, maximum=math.inf):
    number(value, label, minimum, maximum)
    if type(value) is not int:
        raise ValueError(f'Invalid integer: {label}')
    return value


def vector(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'Invalid vector: {label}')
    return [number(v, label, -math.inf) for v in value]


def close(a, b):
    return math.isclose(a, b, rel_tol=1e-8, abs_tol=1e-7)


def _verify(folder, errors):
    manifest = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
    version = manifest['schema_version']
    if type(version) is not int or version not in (1, 2):
        raise ValueError('Unsupported manifest schema')
    artifact_names, source_names = ARTIFACTS, SOURCES
    scenario = sensing = None
    if version == 2:
        from simulation.uavx_scenario import load_scenario
        scenario = load_scenario(folder/'scenario.json')
        sensing = json.loads((folder/'sensing/observations.json').read_text(encoding='utf-8'))
        artifact_names = ARTIFACTS | {'scenario.json','radio-service.json','sensing/observations.json'}
        source_names = SOURCES_V2
        if not isinstance(sensing['observations'], list):
            raise ValueError('Invalid camera observation inventory')
        seen_images = set()
        for observation in sensing['observations']:
            pid = integer(observation['poi_id'], 'camera PoI identity')
            attempt = integer(observation['attempt'],'capture attempt')
            if (pid,attempt) in seen_images:
                raise ValueError('Duplicate camera capture attempt')
            seen_images.add((pid,attempt))
            suffix = '' if attempt==0 else f'-attempt-{attempt}'
            for key, expected in [('capture_file',f'sensing/captures/poi-{pid}{suffix}.png'),
                                  ('received_file',f'sensing/received/poi-{pid}{suffix}.png')]:
                name = observation[key]
                if key == 'received_file' and name is None:
                    continue
                if name != expected:
                    raise ValueError('Invalid camera artifact path')
                artifact_names.add(name)
    root = Path(__file__).resolve().parent
    for key, names, base in [('output_sha256', artifact_names, folder), ('source_sha256', source_names, root)]:
        hashes = manifest[key]
        if not isinstance(hashes, dict) or set(hashes) != names:
            raise ValueError(f'Incomplete or invalid {key} inventory')
        for name in sorted(names):
            digest = hashes[name]
            if not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None:
                raise ValueError(f'Invalid SHA-256: {name}')
            if hashlib.sha256((base / name).read_bytes()).hexdigest() != digest:
                errors.append(f'Hash mismatch: {name}')
    for key in ('python', 'numpy', 'networkx'):
        if not isinstance(manifest[key], str) or not manifest[key]:
            raise ValueError(f'Missing runtime version: {key}')
    data = {name: json.loads((folder / name).read_text(encoding='utf-8'))
            for name in ARTIFACTS - {'trajectory.jsonl'}}
    m, events, packets, samples = (data[n] for n in ('metrics.json', 'events.json', 'packets.json', 'connectivity.json'))
    frames = [json.loads(line) for line in (folder / 'trajectory.jsonl').read_text(encoding='utf-8').splitlines()]
    if type(m['schema_version']) is not int or m['schema_version'] != version:
        raise ValueError('Unsupported metrics schema')
    if m['requirements'] != REQUIREMENTS:
        raise ValueError('Requirements do not match the supplied sample constraints')
    if not isinstance(m['evidence_scope'], str) or not m['evidence_scope']:
        raise ValueError('Missing evidence scope')
    checks = m['checks']
    check_names = CHECKS | CHECKS_V2_EXTRA if version == 2 else CHECKS
    if not isinstance(checks, dict) or set(checks) != check_names or any(type(v) is not bool for v in checks.values()):
        raise ValueError('Missing/invalid constraint checks')
    if type(m['sample_constraints_passed']) is not bool:
        raise ValueError('Invalid summary check')
    if not all(isinstance(v, list) for v in (events, packets, samples, m['tasks'])):
        raise ValueError('Evidence collections must be arrays')
    fleet = integer(m['fleet_size'], 'fleet_size', 1, 72)
    integer(m['seed'], 'seed', -math.inf)
    elapsed = number(m['elapsed_time_s'], 'elapsed_time_s', maximum=2700.)
    dt = number(m['dt_s'], 'dt_s', minimum=1e-12, maximum=.5)
    number(m['packet_loss_per_attempt'], 'packet_loss_per_attempt', maximum=1.)
    completion = m['completion_time_s']
    if completion is not None:
        number(completion, 'completion_time_s', maximum=elapsed)
    for name in ('maximum_speed_mps', 'maximum_altitude_m', 'maximum_sortie_s',
                 'communication_downtime_s', 'priority_weighted_score'):
        number(m[name], name)
    for name in ('battery_exhaustions', 'collision_count', 'separation_violation_steps',
                 'geofence_violation_steps', 'relay_assignments', 'relay_handoffs_completed',
                 'pois_completed', 'pois_total'):
        integer(m[name], name)
    if m['minimum_separation_m'] is not None:
        number(m['minimum_separation_m'], 'minimum_separation_m')

    def agree(label, actual, expected):
        same = close(actual, expected) if type(actual) in (int, float) and type(expected) in (int, float) else actual == expected
        if not same:
            errors.append(f'Contradictory {label}')

    if version == 2:
        agree('scenario configuration', m['scenario'], scenario.to_dict())
        agree('scenario fleet', fleet, scenario.fleet_size)
        agree('scenario seed', m['seed'], scenario.seed)
        agree('scenario packet loss', m['packet_loss_per_attempt'], scenario.packet_loss)

    event_counts = Counter()
    takeoffs, landings, failed = {}, {}, set()
    takeoff_times = {i: [] for i in range(fleet)}
    detections, surveys = {}, {}
    for e in events:
        if not isinstance(e['type'], str):
            raise ValueError('Invalid event type')
        stamp = number(e['time_s'], 'event time', maximum=elapsed + 1e-7)
        event_counts[e['type']] += 1
        if 'uav_id' in e:
            integer(e['uav_id'], 'event uav_id', maximum=fleet - 1)
        if e['type'] in ('takeoff_started', 'landing_confirmed', 'battery_exhausted', 'disturbance_injected'):
            uid = integer(e['uav_id'], 'event uav_id', maximum=fleet - 1)
        if e['type'] == 'takeoff_started':
            takeoffs[uid] = max(stamp, takeoffs.get(uid, -1))
            takeoff_times[uid].append(stamp)
        elif e['type'] == 'landing_confirmed':
            pos, vel = vector(e['position'], 'landing position'), vector(e['velocity'], 'landing velocity')
            columns = 5 if fleet <= 20 else 6
            spacing = 25.
            if version == 2:
                columns, spacing = scenario.staging_columns, scenario.pad_spacing_m
            home = ((uid % columns - (columns - 1) / 2) * spacing,
                    (uid // columns - (math.ceil(fleet / columns) - 1) / 2) * spacing, 0.)
            observed = number(e['observed_at_s'], 'landing observation', maximum=elapsed + 1e-7)
            number(e['sortie_s'], 'landing sortie', maximum=m['maximum_sortie_s'] + 1e-7)
            if (not close(stamp, observed) or e['disarm_acknowledged'] is not True
                    or math.dist(pos, home) > .25 + 1e-7 or math.hypot(*vel) > .15 + 1e-7):
                errors.append(f'Invalid landing evidence: UAV {uid}')
            landings[uid] = max(stamp, landings.get(uid, -1))
        elif e['type'] == 'battery_exhausted' or (e['type'] == 'disturbance_injected' and e['disturbance'] == 'failure'):
            failed.add(uid)
        elif e['type'] == 'scenario_fault_injected' and e['kind'] == 'vehicle_failure':
            failed.add(integer(e['target'], 'failed UAV', maximum=fleet-1))
        elif e['type'] == 'poi_detected':
            detections.setdefault(e['poi_id'], []).append(e)
        elif e['type'] == 'poi_surveyed':
            surveys.setdefault(e['poi_id'], []).append(e)
    agree('battery exhaustion count', m['battery_exhaustions'], event_counts['battery_exhausted'])
    agree('relay assignment count', m['relay_assignments'], event_counts['role_assigned'])
    agree('relay handoff count', m['relay_handoffs_completed'], event_counts['relay_handoff_completed'])

    offers, terminals, receipts = {}, {}, {}
    for e in packets:
        kind = e['type']
        if kind not in ('offered', 'delivered', 'dropped'):
            raise ValueError('Unknown packet event')
        pid = integer(e['packet_id'], 'packet_id')
        integer(e['source_id'], 'packet source', maximum=fleet - 1)
        if e['dest_id'] != -1:
            raise ValueError('UAV-X packet is not addressed to the GCS')
        created = number(e['creation_time_s'], 'packet creation time', maximum=elapsed + 1e-7)
        stamp = number(e['time_s'], 'packet event time', minimum=created, maximum=elapsed + 1e-7)
        if e['payload'] is not None and not isinstance(e['payload'], dict):
            raise ValueError('Invalid packet payload')
        if kind == 'offered':
            if pid in offers:
                errors.append('Duplicate logical packet offer')
            offers[pid] = e
            agree('packet offer time', stamp, created)
            continue
        if pid in terminals:
            errors.append('Duplicate terminal packet event')
        terminals[pid] = e
        offer = offers.get(pid)
        if offer is None:
            errors.append('Terminal packet lacks preceding offer')
        elif any(e[key] != offer[key] for key in ('source_id', 'dest_id', 'payload', 'creation_time_s')):
            errors.append(f'Packet provenance changed: {pid}')
        if kind == 'delivered':
            agree('packet latency', number(e['latency_s'], 'packet latency'), stamp - created)
            integer(e['hops'], 'delivered hops', 1, 32)
            integer(e['reroute_count'], 'reroute_count')
            if 'poi_id' in (e['payload'] or {}):
                receipts.setdefault(e['payload']['poi_id'], []).append(e)
    delivered = [e for e in terminals.values() if e['type'] == 'delivered']
    image_count = 0
    if version == 2:
        from verification.uavx_extended import verify_images, verify_radio, verify_faults
        receipts, image_count = verify_images(folder, m, sensing, delivered, receipts, agree, events)
        verify_radio(folder, m, packets, agree)
        verify_faults(m, events, delivered, agree)
    rerouted = [e for e in delivered if e['reroute_count'] > 0]
    stats = m['packet_statistics']
    for key, expected in [('packets_sent', len(offers)), ('packets_delivered', len(delivered)),
                          ('packets_dropped', len(terminals) - len(delivered)), ('successful_recoveries', len(rerouted))]:
        agree(key, integer(stats[key], key), expected)
    for key in ('reroute_attempts', 'reroutes_found', 'ttl_drops', 'num_nodes', 'num_edges'):
        integer(stats[key], key)
    if not len(rerouted) <= stats['reroutes_found'] <= stats['reroute_attempts'] or stats['ttl_drops'] > stats['packets_dropped']:
        errors.append('Inconsistent routing counters')
    for key, expected in [('pdr', len(delivered) / max(1, len(offers))),
                          ('avg_latency', sum(e['latency_s'] for e in delivered) / max(1, len(delivered))),
                          ('avg_hops', sum(e['hops'] for e in delivered) / max(1, len(delivered))),
                          ('avg_recovery_latency', sum(e['latency_s'] for e in rerouted) / max(1, len(rerouted)))]:
        agree(key, number(stats[key], key), expected)

    tasks, completed, late = m['tasks'], [], []
    if not tasks:
        raise ValueError('No PoI scenario evidence')
    ids = [integer(p['id'], 'PoI id') for p in tasks]
    if len(ids) != len(set(ids)) or not (set(receipts) | set(detections) | set(surveys)) <= set(ids):
        raise ValueError('Duplicate or unknown PoI identity')
    for p in tasks:
        pid = p['id']
        if version == 2:
            planned = next((task for task in scenario.tasks if task.id == pid), None)
            if planned is None:
                raise ValueError('PoI missing from scenario')
            agree('scenario task', {key:p[key] for key in ('id','position','spawn_s','priority')}, planned.to_dict())
        x, y, z = vector(p['position'], 'PoI position')
        if not (75 <= x <= 1075 and -500 <= y <= 500 and z == 0):
            raise ValueError('Invalid PoI scenario geometry')
        spawned = number(p['spawn_s'], 'PoI spawn', maximum=2700.)
        number(p['priority'], 'PoI priority', minimum=1e-12)
        if type(p['surveyed']) is not bool:
            raise ValueError('Invalid surveyed flag')
        detection, reported = p['detection_s'], p['report_s']
        if detection is not None:
            number(detection, 'PoI detection', minimum=spawned, maximum=elapsed + 1e-7)
            integer(p['detected_by'], 'detecting UAV', maximum=fleet - 1)
            matching = detections.get(pid, [])
            if len(matching) != 1 or not close(matching[0]['time_s'], detection) or matching[0]['uav_id'] != p['detected_by'] or not close(matching[0]['observation_time_s'], detection):
                errors.append(f'PoI {pid} lacks matching detection evidence')
        elif p['detected_by'] is not None or p['surveyed'] or reported is not None or detections.get(pid):
            errors.append(f'PoI {pid} completion lacks detection')
        if reported is not None:
            number(reported, 'PoI report', minimum=detection if detection is not None else 0, maximum=elapsed + 1e-7)
        found = receipts.get(pid, [])
        first = min((e['time_s'] for e in found), default=None)
        agree(f'PoI {pid} first GCS receipt', reported, first)
        if any(e['payload'].get('detection_time_s') != detection for e in found):
            errors.append(f'PoI {pid} detection timestamp changed in transit')
        if p['surveyed'] and not any(detection is not None and e['time_s'] >= detection for e in surveys.get(pid, [])):
            errors.append(f'PoI {pid} lacks survey evidence')
        if p['surveyed'] and reported is not None:
            completed.append(p)
        if detection is not None and (reported is None or reported - detection > 10 + 1e-8):
            late.append(pid)
    agree('PoI total', m['pois_total'], len(tasks))
    agree('PoI completed count', m['pois_completed'], len(completed))
    agree('priority weighted score', m['priority_weighted_score'], 100 * sum(p['priority'] for p in completed) / sum(p['priority'] for p in tasks))
    agree('reporting deadline misses', m['reporting_deadline_misses'], late)

    active_time = downtime = previous = 0.
    for s in samples:
        duration = number(s['dt_s'], 'connectivity dt', minimum=1e-12, maximum=.5)
        stamp = number(s['time_s'], 'connectivity time', maximum=elapsed + 1e-7)
        active = integer(s['active'], 'active count', maximum=fleet)
        connected = integer(s['connected'], 'connected count', maximum=active)
        down = number(s['downtime_s'], 'interval downtime', maximum=duration + 1e-8)
        agree('connectivity timestep', duration, dt)
        agree('connectivity timeline', stamp, previous + duration)
        if (connected == active and down > 1e-8) or (connected < active and down == 0):
            errors.append('Connectivity count disagrees with interval downtime')
        active_time += duration if active else 0
        downtime += down
        previous = stamp
    agree('connectivity elapsed time', previous, elapsed)
    agree('communication downtime', m['communication_downtime_s'], downtime)
    availability = 100 * (active_time - downtime) / active_time if active_time else None
    if m['connectivity_availability_percent'] is not None:
        number(m['connectivity_availability_percent'], 'connectivity availability', maximum=100 + 1e-7)
    agree('connectivity availability', m['connectivity_availability_percent'], availability)

    previous = 0.
    prior_positions = {}
    samples_by_time = {s['time_s']: s for s in samples}
    for frame in frames:
        stamp = number(frame['time_s'], 'trajectory time', minimum=previous, maximum=elapsed + 1e-7)
        if stamp - previous > 2 + dt + 1e-7:
            errors.append('Missing trajectory interval')
        uavs = frame['uavs']
        if len(uavs) != fleet or {u['id'] for u in uavs} != set(range(fleet)):
            raise ValueError('Incomplete trajectory fleet')
        frame_active = sum(u['armed'] is True and u['role'] != 'failed' for u in uavs)
        if stamp not in samples_by_time or samples_by_time[stamp]['active'] < frame_active:
            errors.append('Trajectory activity contradicts connectivity sample')
        for u in uavs:
            uid = integer(u['id'], 'trajectory UAV', maximum=fleet - 1)
            pos = vector(u['position'], 'trajectory position')
            number(u['battery'], 'trajectory battery', maximum=1.)
            if type(u['armed']) is not bool or type(u['landed']) is not bool or (u['armed'] and u['landed']):
                raise ValueError('Invalid trajectory flight flags')
            if u['armed'] and not any(t <= stamp for t in takeoff_times[uid]):
                errors.append(f'Airborne UAV {uid} lacks takeoff event')
            if pos[2] > m['maximum_altitude_m'] + 1e-7:
                errors.append('Recorded position exceeds reported maximum altitude')
            if uid in prior_positions and stamp > previous:
                if math.dist(pos, prior_positions[uid]) / (stamp - previous) > m['maximum_speed_mps'] + 1e-7:
                    errors.append('Recorded movement exceeds reported maximum speed')
            prior_positions[uid] = pos
            if not (-75 <= pos[0] <= 1075 and -500 <= pos[1] <= 500 and 0 <= pos[2] <= 100) and m['geofence_violation_steps'] == 0:
                errors.append('Recorded position contradicts zero geofence violations')
            if u['role'] == 'failed':
                failed.add(uid)
        if fleet > 1:
            observed_minimum = min(math.dist(a['position'], b['position'])
                                   for i, a in enumerate(uavs) for b in uavs[i + 1:])
            if m['minimum_separation_m'] is None or m['minimum_separation_m'] > observed_minimum + 1e-7:
                errors.append('Reported minimum separation lacks sampled support')
        previous = stamp
    if elapsed > 0 and (not frames or elapsed - previous >= 2 + dt + 1e-7):
        errors.append('Missing final trajectory interval')
    all_landed = not failed and all(uid not in takeoffs or landings.get(uid, -1) >= takeoffs[uid] for uid in range(fleet))
    derived = dict(all_pois_surveyed_and_reported=len(completed) == len(tasks), sample_has_ten_pois=len(tasks) == 10,
                   all_uavs_landed=all_landed, mission_deadline=completion is not None and completion <= 2700,
                   endurance=m['maximum_sortie_s'] <= 1200, speed=m['maximum_speed_mps'] <= 5 + 1e-8,
                   altitude=m['maximum_altitude_m'] <= 100, separation=m['separation_violation_steps'] == 0,
                   geofence=m['geofence_violation_steps'] == 0, no_battery_exhaustion=m['battery_exhaustions'] == 0,
                   continuous_connectivity=downtime < 1e-8,
                   reporting_deadline=not late and all(p['detection_s'] is not None for p in tasks))
    if version == 2:
        derived['imagery_received'] = image_count == len(tasks)
        derived['scheduled_faults_exercised'] = len(m['faults']) == len(scenario.faults)
        survivors = set(range(fleet)) - failed
        survivor_metrics = m['survivor_metrics']
        agree('survivor fleet count', survivor_metrics['operational_uavs'], len(survivors))
        agree('failed fleet count', survivor_metrics['failed_uavs'], len(failed))
        survivor_landed = bool(survivors) and all(uid not in takeoffs or landings.get(uid,-1) >= takeoffs[uid] for uid in survivors)
        agree('survivor landing', survivor_metrics['all_survivors_landed'], survivor_landed)
        survivor_time = survivor_metrics['completion_time_s']
        if survivor_time is not None:
            number(survivor_time, 'survivor completion', maximum=elapsed)
            if not survivor_landed or len(completed) != len(tasks):
                errors.append('Survivor completion lacks mission and landing evidence')
            required_times=[p['report_s'] for p in completed]
            required_times += [e['time_s'] for values in surveys.values() for e in values]
            required_times += [stamp for uid,stamp in landings.items() if uid in survivors]
            if required_times and survivor_time+1e-7 < max(required_times):
                errors.append('Survivor completion precedes supporting evidence')
    if completion is not None and not (all_landed and len(completed) == len(tasks)):
        errors.append('Completion time lacks completed mission and landing evidence')
    if completion is not None:
        required_times=[p['report_s'] for p in completed]
        required_times += [e['time_s'] for values in surveys.values() for e in values]
        required_times += list(landings.values())
        if required_times and completion+1e-7 < max(required_times):
            errors.append('Completion time precedes supporting evidence')
    if m['minimum_separation_m'] is not None and m['minimum_separation_m'] < 20 - 1e-7 and derived['separation']:
        errors.append('Minimum separation contradicts zero violations')
    for name in check_names:
        agree(f'{name} check', checks[name], derived[name])
    agree('summary check', m['sample_constraints_passed'], all(checks.values()))
    return [name for name, value in checks.items() if value is not True]


def verify(folder):
    errors, failed = [], []
    try:
        failed = _verify(Path(folder), errors)
    except (OSError, ValueError, KeyError, TypeError, AttributeError, IndexError) as exc:
        errors.append(f'Missing or invalid evidence: {exc}')
    return dict(evidence_errors=errors, failed_constraints=failed,
                sample_run_passed=not errors and not failed, submission_ready=False,
                note='Integrity and internal consistency only; not independent full dynamics proof or tamperproof authority. '
                     'Source hashes must match this checkout. Team details, report/video, official logs and organizer validation remain separate.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    result = verify(args.folder)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['sample_run_passed'] else 2)


if __name__ == '__main__':
    main()
