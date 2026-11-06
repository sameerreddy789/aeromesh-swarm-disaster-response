"""Audit Bullet mission evidence without applying the kinematic verifier.

Hashes and recorded-state consistency are checked independently of mission
success. The 1 Hz body records and 20 Hz safety ledger cannot prove continuous
physics or physical-airframe calibration; 120 Hz extrema are engine aggregates.
"""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import re


SCHEMA = 'aeromesh-bullet-1'
REQUIRED_OUTPUTS = {
    'metrics.json', 'events.json', 'physics-telemetry.jsonl', 'physics-safety.json',
    'scenario.json', 'connectivity.json', 'trajectory.jsonl', 'packets.json',
    'radio-service.json', 'sensing/observations.json',
}
REQUIRED_SOURCES = {
    'simulation/quadrotor_physics.py', 'simulation/physics_uavx.py',
    'simulation/physics_planning.py', 'simulation/resilient_uavx.py',
    'simulation/uavx.py', 'simulation/uavx_scenario.py', 'simulation/camera_sensor.py',
    'core/mission_safety.py', 'core/models.py', 'core/config.py',
    'comm/mesh_router.py', 'comm/channel.py', 'comm/channel_legacy.py',
    'run_physics.py', 'run_simulation.py',
}
CHECK_NAMES = {
    'all_pois_surveyed_and_reported', 'sample_has_ten_pois', 'all_uavs_landed',
    'mission_deadline', 'endurance', 'speed', 'altitude', 'separation', 'geofence',
    'no_battery_exhaustion', 'continuous_connectivity', 'reporting_deadline',
    'scheduled_faults_exercised', 'imagery_received', 'no_body_collision',
}


def _number(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(f'{name} is outside its permitted range')
    return float(value)


def _integer(value, name, minimum=0, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'{name} must be an integer')
    _number(value, name, minimum, maximum)
    return value


def _boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f'{name} must be a boolean')
    return value


def _vector(value, name, size=3):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f'{name} requires {size} finite components')
    return tuple(_number(v, name) for v in value)


def _unique_object(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError(f'duplicate JSON key {key!r}')
        result[key] = value
    return result


def _decode(text):
    def invalid_constant(value):
        raise ValueError(f'non-finite JSON value {value}')
    return json.loads(text, parse_constant=invalid_constant, object_pairs_hook=_unique_object)


def _read(path):
    return _decode(path.read_text(encoding='utf-8'))


def _contained(root, name):
    if not isinstance(name, str) or not name or '\\' in name or ':' in name:
        raise ValueError('manifest paths must be relative POSIX paths')
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts or '.' in relative.parts:
        raise ValueError(f'unsafe manifest path {name!r}')
    resolved = (root / relative).resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f'manifest path escapes its root: {name!r}')
    return resolved


def _hashes(root, values, required, label, errors):
    if not isinstance(values, dict):
        raise ValueError(f'{label} hash inventory must be an object')
    for name in sorted(required - values.keys()):
        errors.append(f'{label} inventory missing {name}')
    for name, expected in values.items():
        path = _contained(root, name)
        if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-fA-F]{64}', expected):
            raise ValueError(f'invalid SHA256 for {label} {name}')
        if not path.is_file():
            errors.append(f'{label} file missing: {name}')
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected.lower():
            errors.append(f'{label} SHA256 mismatch: {name}')


def _verify_receipts(folder, m, manifest, events, errors):
    """Rebuild GCS receipts from delivered bytes, independently of geometry."""
    sensing = _read(folder / 'sensing/observations.json')
    mode = m['scenario']['camera_mode']
    if sensing['mode'] != mode or m['sensing']['mode'] != mode:
        errors.append('camera mode differs between scenario, observations and metrics')
    observations = sensing['observations']
    if not isinstance(observations, list):
        raise ValueError('camera observations must be a list')
    if m['sensing']['captures'] != len(observations):
        errors.append('capture count differs from observation inventory')
    tasks = {p['id']: p for p in m['tasks']}
    captures, images = {}, {}
    scene = sensing.get('scene')
    if scene is not None:
        canonical_scene = dict(scene)
        digest = canonical_scene.pop('scene_sha256')
        actual = hashlib.sha256(json.dumps(canonical_scene, sort_keys=True,
                                           separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if actual != digest:
            errors.append('camera scene fingerprint differs from its metadata')
    if mode == 'footprint':
        if observations or scene is not None:
            errors.append('footprint mode claims rendered camera observations')
    elif mode != 'panda3d':
        raise ValueError('unknown physics camera mode')
    elif observations and scene is None:
        errors.append('rendered captures lack scene provenance')
    for observation in observations:
        identity = observation['capture_id']
        if not isinstance(identity, str) or not identity or identity in captures:
            raise ValueError('capture identities must be nonempty and unique')
        pid = observation['poi_id']
        if pid not in tasks:
            raise ValueError('capture references an unknown PoI')
        uid = _integer(observation['uav_id'], 'capture UAV identity', 0, 29)
        task = tasks[pid]
        acquired = _number(observation['acquired_at_s'], 'capture time', 0., m['elapsed_time_s'] + 1e-7)
        if task['detection_s'] is None or acquired < task['detection_s'] - 1e-7:
            errors.append('capture lacks a preceding PoI detection')
        if observation['detection_time_s'] != task['detection_s']:
            errors.append('capture original detection time differs from its task')
        attempt = _integer(observation['attempt'], 'capture attempt')
        if attempt == 0 and (acquired != task['detection_s'] or uid != task['detected_by']):
            errors.append('initial capture source/time differs from the detection')
        name = observation['capture_file']
        if name not in manifest['output_sha256']:
            errors.append('capture file is absent from the output hash inventory')
        content = _contained(folder, name).read_bytes()
        length = _integer(observation['byte_length'], 'capture byte length', 1, 4_000_000)
        if (len(content) != length or hashlib.sha256(content).hexdigest() != observation['sha256']
                or not content.startswith(b'\x89PNG\r\n\x1a\n')):
            errors.append('captured PNG bytes differ from their length/fingerprint metadata')
        if observation['chunk_count'] != math.ceil(length / 512):
            errors.append('capture fragment inventory differs from its byte length')
        captures[identity], images[identity] = observation, content
    packets = _read(folder / 'packets.json')
    if not isinstance(packets, list):
        raise ValueError('packet evidence must be a list')
    delivered, packet_ids = [], set()
    for packet in packets:
        if packet['type'] != 'delivered':
            continue
        identity = _integer(packet['packet_id'], 'delivered packet identity')
        if identity in packet_ids:
            raise ValueError('packet has multiple terminal delivery records')
        packet_ids.add(identity)
        when = _number(packet['time_s'], 'packet receipt time', 0., m['elapsed_time_s'] + 1e-7)
        _number(packet['creation_time_s'], 'packet creation time', 0., when + 1e-7)
        _integer(packet['source_id'], 'packet source', 0, 29)
        if packet['dest_id'] != -1:
            raise ValueError('mission delivery is not addressed to GCS')
        delivered.append(packet)
    delivered.sort(key=lambda p: (p['time_s'], p['packet_id']))
    chunks, completed_captures, first_receipts = {}, {}, {}
    for packet in delivered:
        payload = packet.get('payload') or {}
        if payload.get('kind') == 'poi_report' and mode == 'footprint':
            pid = payload['poi_id']
            if pid not in tasks or payload['detection_time_s'] != tasks[pid]['detection_s']:
                errors.append('delivered PoI report does not match a detected task')
            else:
                first_receipts.setdefault(pid, packet)
        elif payload.get('kind') == 'poi_image_chunk':
            identity = payload.get('capture_id')
            if mode != 'panda3d' or identity not in captures:
                raise ValueError('delivered image chunk lacks its capture provenance')
            observation = captures[identity]
            if (payload['poi_id'] != observation['poi_id']
                    or packet['source_id'] != observation['uav_id']
                    or payload['observation_time_s'] != observation['acquired_at_s']
                    or payload['detection_time_s'] != observation['detection_time_s']
                    or payload['image_sha256'] != observation['sha256']
                    or payload['chunk_count'] != observation['chunk_count']):
                errors.append('delivered image chunk provenance differs from its capture')
            index = _integer(payload['chunk_index'], 'image chunk index', 0, observation['chunk_count'] - 1)
            content = base64.b64decode(payload['data_b64'], validate=True)
            if content != images[identity][index * 512:(index + 1) * 512]:
                errors.append('delivered image fragment differs from captured bytes')
            inventory = chunks.setdefault(identity, {})
            if index in inventory and inventory[index] != content:
                errors.append('conflicting duplicate image fragment')
            inventory[index] = content
            if identity not in completed_captures and len(inventory) == observation['chunk_count']:
                rebuilt = b''.join(inventory[i] for i in range(observation['chunk_count']))
                if rebuilt != images[identity] or hashlib.sha256(rebuilt).hexdigest() != observation['sha256']:
                    errors.append('complete delivered image does not match its captured fingerprint')
                completed_captures[identity] = packet
                first_receipts.setdefault(observation['poi_id'], packet)
    for identity, observation in captures.items():
        received = completed_captures.get(identity)
        when = received['time_s'] if received is not None else None
        if observation['received_at_s'] != when:
            errors.append('image received_at_s differs from complete delivered chunk evidence')
        name = observation['received_file']
        if when is None:
            if name is not None:
                errors.append('incomplete image has a claimed GCS file')
        elif name is None:
            errors.append('complete image receipt lacks its GCS file')
        else:
            if name not in manifest['output_sha256']:
                errors.append('GCS image is absent from output hash inventory')
            if _contained(folder, name).read_bytes() != images[identity]:
                errors.append('GCS image bytes differ from complete delivered/captured bytes')
    receipts = {}
    for event in events:
        if event['type'] == 'poi_received':
            pid = event['poi_id']
            if pid in receipts or pid not in tasks:
                raise ValueError('duplicate or unknown PoI receipt event')
            receipts[pid] = event
    for pid, task in tasks.items():
        packet, event = first_receipts.get(pid), receipts.get(pid)
        when = packet['time_s'] if packet is not None else None
        if task['report_s'] != when:
            errors.append('task report time lacks matching first GCS delivery evidence')
        if packet is None:
            if event is not None:
                errors.append('PoI receipt event lacks a complete delivered payload')
        elif event is None:
            errors.append('delivered PoI report lacks its unique receipt event')
        else:
            if (event['time_s'] != when or event['packet_id'] != packet['packet_id']
                    or event['detection_time_s'] != task['detection_s']):
                errors.append('PoI receipt event differs from its delivered payload/task')
            if mode == 'panda3d':
                capture = captures[packet['payload']['capture_id']]
                if (event.get('capture_id') != capture['capture_id']
                        or event.get('image_sha256') != capture['sha256']
                        or event.get('image_bytes') != capture['byte_length']
                        or event.get('chunks') != capture['chunk_count']):
                    errors.append('image receipt event differs from completed image provenance')
        detections = [e for e in events if e['type'] == 'poi_detected' and e['poi_id'] == pid]
        surveys = [e for e in events if e['type'] == 'poi_surveyed' and e['poi_id'] == pid]
        if task['detection_s'] is None:
            if detections:
                errors.append('undetected task has a detection event')
        elif (len(detections) != 1 or detections[0]['time_s'] != task['detection_s']
              or detections[0]['uav_id'] != task['detected_by']):
            errors.append('task detection differs from its unique event')
        if bool(surveys) != task['surveyed'] or len(surveys) > 1:
            errors.append('task survey completion differs from its unique event')
    received_images = len(first_receipts) if mode == 'panda3d' else 0
    received_bytes = sum(len(images[p['payload']['capture_id']]) for p in first_receipts.values()) if mode == 'panda3d' else 0
    if m['sensing']['received_images'] != received_images or m['sensing']['received_bytes'] != received_bytes:
        errors.append('image receipt totals differ from complete delivered images')
    return received_images


def _audit(folder, source_root, errors):
    manifest, m = _read(folder / 'manifest.json'), _read(folder / 'metrics.json')
    if manifest.get('schema_version') != SCHEMA or m.get('schema_version') != SCHEMA:
        raise ValueError(f'physics evidence requires schema {SCHEMA}')
    if manifest.get('source_stable') is False or m.get('source_stable') is False:
        errors.append('run declares that its source changed during execution')
    _hashes(folder, manifest['output_sha256'], REQUIRED_OUTPUTS, 'output', errors)
    _hashes(source_root, manifest['source_sha256'], REQUIRED_SOURCES, 'source', errors)
    if _integer(m['fleet_size'], 'fleet size') != 30:
        raise ValueError('physics mission evidence must contain exactly 30 UAVs')
    scenario = _read(folder / 'scenario.json')
    if scenario != m['scenario'] or scenario.get('fleet_size') != 30:
        errors.append('scenario differs from metrics or fixed 30-UAV fleet')
    physics, elapsed = m['physics'], _number(m['elapsed_time_s'], 'elapsed time', 0., 2700.)
    config, stats = physics['parameters']['config'], physics['stats']
    fixed = _number(config['fixed_dt'], 'fixed physics interval', 1e-9, .02)
    control = _number(physics['control_dt_s'], 'control interval', fixed, 1.)
    if not math.isclose(physics['mission_safety_sampling_hz'], 1. / control, rel_tol=1e-9):
        errors.append('safety sampling frequency contradicts control interval')
    if not math.isclose(control / fixed, round(control / fixed), abs_tol=1e-7):
        errors.append('control interval is not an integer number of physics steps')
    if physics['parameters'].get('engine') != 'Panda3D Bullet':
        errors.append('physics engine identity is missing or incorrect')
    nonnegative_config = {'linear_drag_n_per_mps', 'quadratic_drag_n_per_mps2',
                          'angular_drag_nms', 'friction', 'restitution'}
    vector_config = {'inertia_kg_m2', 'velocity_kp', 'velocity_ki'}
    for name, value in config.items():
        if name in vector_config:
            values = _vector(value, name)
            if any(v < 0. or name != 'velocity_ki' and v <= 0. for v in values):
                raise ValueError(f'{name} has an invalid component')
        else:
            _number(value, name, 0. if name in nonnegative_config else 1e-300)
    if config['max_tilt_rad'] >= math.pi / 2. or config['restitution'] > 1.:
        raise ValueError('invalid tilt limit or restitution parameter')
    radius = _number(config['body_radius_m'], 'body radius', 1e-9)
    max_thrust = _number(config['max_motor_thrust_n'], 'motor thrust limit', 1e-9)
    timeout = _number(config['command_timeout_s'], 'command timeout', 1e-9)
    for name in ('mass_kg', 'motor_tau_s', 'gravity_mps2'):
        _number(config[name], name, 1e-9)
    for value in _vector(config['inertia_kg_m2'], 'inertia'):
        if value <= 0.:
            raise ValueError('inertia must be positive')
    _vector(physics['parameters']['wind_mps'], 'wind')
    steps = _integer(stats['fixed_steps'], 'fixed steps')
    if not math.isclose(steps * fixed, elapsed, abs_tol=1e-6) or not math.isclose(stats['time_s'], elapsed, abs_tol=1e-6):
        errors.append('engine clock or fixed-step count differs from mission duration')
    if stats['fleet_size'] != 30:
        errors.append('engine fleet size differs from 30')
    maximum_speed = _number(stats['max_speed_mps'], 'engine maximum speed', 0.)
    maximum_angular = _number(stats['max_angular_speed_rps'], 'engine maximum angular speed', 0.)
    maximum_tilt = _number(stats['max_tilt_rad'], 'engine maximum tilt', 0., math.pi + 1e-6)
    minimum_z = _number(stats['min_com_z_m'], 'engine minimum COM height')
    if not math.isclose(m['maximum_speed_mps'], maximum_speed, abs_tol=1e-7):
        errors.append('reported speed differs from all-substep engine maximum')
    contacts = physics['contacts']
    if contacts != stats['contacts']:
        errors.append('contact summaries disagree')
    pairs = contacts['pairs']
    if not isinstance(pairs, dict):
        raise ValueError('cumulative contact pairs must be an object')
    for key, value in pairs.items():
        names = key.split('|')
        if len(names) != 2 or len(set(names)) != 2 or not any(name.startswith('quadrotor-') for name in names):
            raise ValueError('invalid cumulative contact pair')
        for name in names:
            if name.startswith('quadrotor-'):
                _integer(int(name[len('quadrotor-'):]), 'contact UAV identity', 0, 29)
        _integer(value['begins'], 'contact entry count', 1)
        _number(value['seconds'], 'contact duration', fixed - 1e-9, elapsed + fixed + 1e-6)
        _number(value['max_impulse_ns'], 'contact impulse', 0.)
    for pair in contacts['current_pairs']:
        if not isinstance(pair, list) or len(pair) != 2 or '|'.join(sorted(pair)) not in pairs:
            errors.append('current contact lacks cumulative contact evidence')
    body_pairs = {key for key in pairs if 'ground' not in key.split('|')}
    if _integer(m['collision_count'], 'body collision count') != len(body_pairs):
        errors.append('collision count differs from cumulative non-ground body contacts')

    events = _read(folder / 'events.json')
    if not isinstance(events, list):
        raise ValueError('events must be a list')
    takeoffs, takeoff_times, landings, landing_times, failures, landing_events = {}, {}, {}, {}, {}, []
    injections = []
    for event in events:
        stamp = _number(event['time_s'], 'event time', 0., elapsed + 1e-6)
        kind = event['type']
        if kind == 'scenario_fault_injected':
            injections.append(event)
            if event['kind'] == 'vehicle_failure':
                uid = _integer(event['target'], 'fault UAV identity', 0, 29)
                failures[uid] = min(stamp, failures.get(uid, stamp))
        if kind in ('physics_power_cut', 'battery_exhausted', 'takeoff_started', 'landing_confirmed', 'recharge_completed'):
            uid = _integer(event['uav_id'], 'event UAV identity', 0, 29)
            if kind in ('physics_power_cut', 'battery_exhausted'):
                failures[uid] = min(stamp, failures.get(uid, stamp))
            elif kind == 'takeoff_started':
                takeoffs[uid] = max(stamp, takeoffs.get(uid, stamp))
                takeoff_times.setdefault(uid, []).append(stamp)
            elif kind == 'landing_confirmed':
                landing_events.append(event)
                landings[uid] = max(stamp, landings.get(uid, stamp))
                landing_times.setdefault(uid, []).append(stamp)
    for event in events:
        if event['type'] in ('takeoff_started', 'landing_confirmed', 'recharge_completed'):
            uid = event['uav_id']
            if uid in failures and event['time_s'] > failures[uid] + 1e-8:
                errors.append(f'failed UAV {uid} has later {event["type"]} credit')
    failed_stats = stats['failed_uav_ids']
    if not isinstance(failed_stats, list) or len(failed_stats) != len(set(failed_stats)):
        raise ValueError('invalid failed UAV inventory')
    if set(failed_stats) != set(failures):
        errors.append('engine failed identities differ from recorded fault/power-cut events')

    safety = _read(folder / 'physics-safety.json')
    if not isinstance(safety, list):
        raise ValueError('safety ledger must be a list')
    expected_count = round(elapsed / control)
    if len(safety) != expected_count:
        errors.append('safety ledger does not cover every control interval')
    minimum_separation, safety_max, downtime, active_time, violations = math.inf, 0., 0., 0., 0
    safety_times = {}
    for index, row in enumerate(safety, 1):
        stamp = _number(row['time_s'], 'safety time', 0., elapsed + 1e-6)
        if not math.isclose(stamp, index * control, abs_tol=1e-7):
            errors.append('safety ledger has an incorrect interval timestamp')
        separation = _number(row['minimum_separation_m'], 'safety separation', 0.)
        speed = _number(row['maximum_speed_mps'], 'safety speed', 0.)
        active = _integer(row['active'], 'active count', 0, 30)
        connected = _integer(row['connected'], 'connected count', 0, active)
        minimum_separation = min(minimum_separation, separation)
        safety_max = max(safety_max, speed)
        violations += int(separation < 20. - 1e-7)
        downtime += control if connected < active else 0.
        active_time += control if active else 0.
        safety_times[round(stamp, 6)] = row
    if safety_max > maximum_speed + 1e-6:
        errors.append('20 Hz safety speed exceeds the engine all-substep maximum')
    if safety and not math.isclose(m['minimum_separation_m'], minimum_separation, abs_tol=1e-6):
        errors.append('minimum separation differs from the 20 Hz safety ledger')
    if m['separation_violation_steps'] != violations:
        errors.append('separation violation count differs from safety ledger')
    if not math.isclose(m['communication_downtime_s'], downtime, abs_tol=1e-6):
        errors.append('communication downtime differs from control-interval ledger')
    expected_availability = 100. * (active_time - downtime) / active_time if active_time else None
    if expected_availability is None:
        if m['connectivity_availability_percent'] is not None:
            errors.append('availability reported without active flight intervals')
    elif not math.isclose(m['connectivity_availability_percent'], expected_availability, abs_tol=1e-5):
        errors.append('connectivity availability differs from control-interval ledger')

    frames, previous, observed_failures, sampled_geofence = [], 0., set(), 0
    with (folder / 'physics-telemetry.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            frame = _decode(line)
            stamp = _number(frame['time_s'], 'telemetry time', 0., elapsed + 1e-6)
            if stamp <= previous or stamp - previous > 1. + control + 1e-7:
                errors.append('telemetry timestamps are unordered or contain a missing 1 Hz interval')
            previous = stamp
            records = frame['uavs']
            if not isinstance(records, list) or len(records) != 30:
                raise ValueError('each telemetry frame requires exactly 30 UAVs')
            identities = [_integer(u['id'], 'telemetry identity', 0, 29) for u in records]
            if set(identities) != set(range(30)):
                raise ValueError('telemetry frame has duplicate or missing UAV identities')
            row = safety_times.get(round(stamp, 6))
            if row is None:
                errors.append('telemetry lacks a corresponding control-interval safety sample')
            active_count, positions = 0, []
            for u in records:
                uid = u['id']
                if not math.isclose(_number(u['time_s'], 'body observation time'), stamp, abs_tol=1e-7):
                    errors.append('body observation timestamp differs from its telemetry frame')
                pos = _vector(u['position'], 'position')
                vel = _vector(u['velocity'], 'velocity')
                angular = _vector(u['angular_velocity_world'], 'angular velocity')
                _vector(u['acceleration'], 'acceleration')
                q = _vector(u['quaternion_wxyz'], 'quaternion', 4)
                thrust = _vector(u['rotor_thrusts_n'], 'rotor thrusts', 4)
                for name in ('ground_contact', 'body_contact', 'failed', 'armed', 'command_stale'):
                    _boolean(u[name], name)
                if not math.isclose(math.hypot(*q), 1., abs_tol=1e-4):
                    errors.append(f'UAV {uid} quaternion is not normalized')
                if any(t < -1e-8 or t > max_thrust + 1e-8 for t in thrust):
                    errors.append(f'UAV {uid} exceeds rotor thrust bounds')
                if (not u['armed'] or u['failed']) and any(abs(t) > 1e-8 for t in thrust):
                    errors.append(f'UAV {uid} has thrust while disarmed or failed')
                if u['failed'] and u['armed']:
                    errors.append(f'failed UAV {uid} remains powered/armed')
                if u['armed']:
                    started = max((t for t in takeoff_times.get(uid, ()) if t <= stamp + 1e-7), default=-1.)
                    landed = max((t for t in landing_times.get(uid, ()) if t <= stamp + 1e-7), default=-1.)
                    if started < 0. or started < landed - 1e-7:
                        errors.append(f'armed UAV {uid} lacks a preceding motor-start event for its current sortie')
                if uid in failures and stamp >= failures[uid] + fixed - 1e-8:
                    observed_failures.add(uid)
                    if not u['failed'] or u['armed'] or any(abs(t) > 1e-8 for t in thrust):
                        errors.append(f'UAV {uid} lacks measured permanent motor cut after its failure')
                elif u['failed'] and (uid not in failures or stamp < failures[uid] - 1e-7):
                    errors.append(f'UAV {uid} failure has no supporting event')
                age = u['command_age_s']
                if age is None:
                    if not u['command_stale']:
                        errors.append('missing command timestamp is marked fresh')
                else:
                    age = _number(age, 'command age', 0., elapsed + 1e-6)
                    if u['command_stale'] != (age > timeout + 1e-12):
                        errors.append('command age and expiry flag disagree')
                speed = math.hypot(*vel)
                if speed > maximum_speed + 1e-6 or row is not None and speed > row['maximum_speed_mps'] + 1e-5:
                    errors.append('telemetry speed exceeds recorded safety/engine maximum')
                if math.hypot(*angular) > maximum_angular + 1e-6:
                    errors.append('telemetry angular rate exceeds engine maximum')
                tilt = math.acos(max(-1., min(1., 1. - 2. * (q[1] ** 2 + q[2] ** 2))))
                if tilt > maximum_tilt + 1e-4:
                    errors.append('telemetry tilt exceeds engine maximum')
                if pos[2] < minimum_z - 1e-6 or pos[2] > m['maximum_altitude_m'] + 1e-5:
                    errors.append('telemetry height exceeds reported extrema')
                if u['ground_contact'] and f'ground|quadrotor-{uid}' not in pairs:
                    errors.append('ground contact flag lacks cumulative contact evidence')
                if u['body_contact'] and not any(f'quadrotor-{uid}' in key.split('|') for key in body_pairs):
                    errors.append('body contact flag lacks cumulative collision evidence')
                sampled_geofence += int(not (-75. <= pos[0] <= 1075. and -500. <= pos[1] <= 500. and 0. <= pos[2] <= 100.))
                active_count += int(u['armed'] and not u['failed'])
                positions.append(pos)
            if row is not None:
                if active_count > row['active']:
                    errors.append('telemetry active fleet exceeds control-interval active count')
                sample_min = min(math.dist(a, b) for i, a in enumerate(positions) for b in positions[i+1:])
                if not math.isclose(sample_min, row['minimum_separation_m'], abs_tol=1e-5):
                    errors.append('telemetry positions disagree with simultaneous safety separation')
            frames.append(frame)
    if elapsed and (not frames or elapsed - previous > 1. + control + 1e-7):
        errors.append('missing final telemetry interval')
    if any(elapsed - stamp > 1. + control and uid not in observed_failures for uid, stamp in failures.items()):
        errors.append('fault lacks later telemetry confirming motor cut')
    if sampled_geofence > m['geofence_violation_steps']:
        errors.append('sampled geofence violations exceed reported 20 Hz total')

    columns = _integer(scenario['staging_columns'], 'staging columns', 1, 30)
    spacing = _number(scenario['pad_spacing_m'], 'pad spacing', 1e-9)
    rows = math.ceil(30 / columns)
    def home_for(uid):
        return ((uid % columns - (columns - 1) / 2.) * spacing,
                (uid // columns - (rows - 1) / 2.) * spacing, radius)

    def measured_home(u):
        home = home_for(u['id'])
        return (not u['armed'] and not u['failed'] and u['ground_contact']
                and math.dist(u['position'][:2], home[:2]) <= .75 + 1e-6
                and abs(u['position'][2] - radius) < .15 + 1e-6
                and math.hypot(*u['velocity']) <= .15 + 1e-6)

    for event in landing_events:
        uid, stamp = event['uav_id'], event['time_s']
        home = home_for(uid)
        pos, velocity = _vector(event['position'], 'landing position'), _vector(event['velocity'], 'landing velocity')
        if (event.get('disarm_acknowledged') is not True or event.get('ground_contact') is not True
                or event.get('evidence') != 'bullet_contact_and_telemetry'
                or math.dist(pos[:2], home[:2]) > .75 + 1e-6
                or abs(pos[2] - home[2]) >= .15 + 1e-6 or math.hypot(*velocity) > .15 + 1e-6):
            errors.append(f'UAV {uid} landing lacks disarmed ground-contact/home/speed evidence')
        observed = _number(event['observed_at_s'], 'landing observation time', 0., stamp + 1e-7)
        if stamp - observed > control + 1e-7:
            errors.append('landing observation is stale')
        if not any(t <= stamp + 1e-7 for t in takeoff_times.get(uid, ())):
            errors.append('landing has no preceding motor-start event')
        _number(event['sortie_s'], 'landing sortie time', 0., m['maximum_sortie_s'] + 1e-6)
        supported = False
        for frame in frames:
            if abs(frame['time_s'] - stamp) <= 1. + control:
                u = next(u for u in frame['uavs'] if u['id'] == uid)
                if measured_home(u):
                    supported = True
                    break
        if not supported:
            errors.append(f'UAV {uid} landing lacks nearby corroborating telemetry')

    tasks = m['tasks']
    if not isinstance(tasks, list) or not tasks or len({p['id'] for p in tasks}) != len(tasks):
        raise ValueError('mission tasks must have unique identities')
    completed, late = [], []
    for task in tasks:
        _boolean(task['surveyed'], 'survey completion')
        detection, receipt = task['detection_s'], task['report_s']
        if detection is not None:
            _number(detection, 'detection time', 0., elapsed + 1e-6)
        if receipt is not None:
            _number(receipt, 'report time', 0., elapsed + 1e-6)
            if detection is None or receipt < detection:
                errors.append('PoI report lacks a preceding detection')
        if task['surveyed'] and receipt is not None:
            completed.append(task)
        if detection is not None and (receipt is None or receipt - detection > 10. + 1e-8):
            late.append(task['id'])
    if m['pois_completed'] != len(completed) or m['pois_total'] != len(tasks):
        errors.append('PoI totals differ from task evidence')
    if sorted(m['reporting_deadline_misses']) != sorted(late):
        errors.append('reporting deadline misses differ from task timestamps')
    image_count = _verify_receipts(folder, m, manifest, events, errors)
    completion = m['completion_time_s']
    if completion is not None:
        _number(completion, 'strict completion time', 0., elapsed + 1e-6)
    all_landed = (not failures and bool(frames) and all(measured_home(u) for u in frames[-1]['uavs'])
                  and all(landings.get(uid, -1.) >= stamp for uid, stamp in takeoffs.items()))
    if completion is not None and (not all_landed or len(completed) != len(tasks)):
        errors.append('strict completion lacks all mission and landing evidence')
    if completion is not None and any(stamp > completion + 1e-7 for stamp in [*landings.values(), *(p['report_s'] for p in completed)]):
        errors.append('strict completion precedes its supporting events')
    checks = m['checks']
    if not isinstance(checks, dict) or not CHECK_NAMES <= checks.keys():
        raise ValueError('physics mission checks are incomplete')
    for name, value in checks.items():
        _boolean(value, f'check {name}')
    derived = {
        'all_pois_surveyed_and_reported': len(completed) == len(tasks),
        'sample_has_ten_pois': len(tasks) == 10, 'all_uavs_landed': all_landed,
        'mission_deadline': completion is not None and completion <= 2700.,
        'endurance': _number(m['maximum_sortie_s'], 'maximum sortie', 0.) <= 1200.,
        'speed': maximum_speed <= 5. + 1e-8,
        'altitude': _number(m['maximum_altitude_m'], 'maximum altitude', 0.) <= 100.,
        'separation': violations == 0,
        'geofence': _integer(m['geofence_violation_steps'], 'geofence violation count') == 0,
        'no_battery_exhaustion': _integer(m['battery_exhaustions'], 'battery exhaustion count') == 0,
        'continuous_connectivity': downtime < 1e-8,
        'reporting_deadline': not late and all(p['detection_s'] is not None for p in tasks),
        'scheduled_faults_exercised': len(injections) == len(scenario['faults']),
        'imagery_received': image_count == len(tasks),
        'no_body_collision': not body_pairs,
    }
    for name, value in derived.items():
        if checks[name] != value:
            errors.append(f'check {name} disagrees with recorded evidence')
    if m['sample_constraints_passed'] != all(checks.values()):
        errors.append('combined mission result disagrees with individual checks')
    return sorted(name for name, value in checks.items() if not value)


def verify(folder, source_root=None):
    """Return an audit result; this function never rewrites evidence files."""
    errors, failed = [], []
    try:
        failed = _audit(Path(folder).resolve(),
                        Path(source_root).resolve() if source_root is not None else Path(__file__).resolve().parent,
                        errors)
    except (OSError, ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        errors.append(f'{type(exc).__name__}: {exc}')
    errors = list(dict.fromkeys(errors))
    return {'schema_version': 'aeromesh-bullet-verification-1',
            'evidence_valid': not errors, 'evidence_errors': errors,
            'failed_constraints': failed, 'mission_passed': not errors and not failed,
            'submission_ready': False,
            'notes': ['Integrity and recorded-state consistency, not independent continuous dynamics proof.',
                      'Body telemetry is 1 Hz; mission safety is 20 Hz; engine extrema/contacts aggregate fixed substeps.',
                      'No kinematic qualification, calibrated airframe, physical battery or hardware qualification is inferred.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--source-root', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args()
    result = verify(args.folder, args.source_root)
    rendered = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(rendered + '\n', encoding='utf-8')
    print(rendered)
    raise SystemExit(1 if not result['evidence_valid'] else 2 if args.require_pass and not result['mission_passed'] else 0)


if __name__ == '__main__':
    main()
