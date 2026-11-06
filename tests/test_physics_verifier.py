"""Bullet evidence integrity and semantic checks, independent of mission success."""

import base64
import hashlib
import json
import random
import shutil
import struct
import zlib

import pytest

from simulation.physics_uavx import PhysicsUAVXSimulator
from simulation.uavx_scenario import FaultSpec, ScenarioConfig, ScenarioTask
from verify_physics import verify


@pytest.fixture(scope='module')
def evidence_runs(tmp_path_factory):
    root = tmp_path_factory.mktemp('bullet-verifier')
    folders = {}
    for name, faults in (('short', ()), ('failed', (FaultSpec('vehicle_failure', 0., 0),))):
        sim = PhysicsUAVXSimulator(ScenarioConfig(
            fleet_size=30, camera_mode='footprint', packet_loss=0.,
            tasks=(ScenarioTask(0, (75., 0., 0.)),), faults=faults))
        folder = root / name
        try:
            sim.run(folder, max_time=1.)
        finally:
            sim.close()
        folders[name] = folder
    return folders


def copied_run(tmp_path, evidence_runs, name='short'):
    return shutil.copytree(evidence_runs[name], tmp_path / 'run')


def read(folder, name):
    return json.loads((folder / name).read_text(encoding='utf-8'))


def rehash(folder, name):
    manifest = read(folder, 'manifest.json')
    manifest['output_sha256'][name] = hashlib.sha256((folder / name).read_bytes()).hexdigest()
    (folder / 'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')


def write(folder, name, value):
    (folder / name).write_text(json.dumps(value, allow_nan=False), encoding='utf-8')
    if name != 'manifest.json':
        rehash(folder, name)


def frames(folder):
    return [json.loads(line) for line in (folder / 'physics-telemetry.jsonl').read_text().splitlines()]


def write_frames(folder, values):
    path = folder / 'physics-telemetry.jsonl'
    path.write_text(''.join(json.dumps(value, allow_nan=False) + '\n' for value in values), encoding='utf-8')
    rehash(folder, path.name)


def rejected(folder, fragment):
    result = verify(folder)
    assert not result['evidence_valid'] and not result['mission_passed']
    assert not result['submission_ready']
    assert any(fragment in error for error in result['evidence_errors']), result


def with_image_receipt(folder):
    """Add a consistent byte-transport fixture; this does not qualify camera geometry."""
    def png_chunk(kind, content):
        return (struct.pack('>I', len(content)) + kind + content
                + struct.pack('>I', zlib.crc32(kind + content) & 0xffffffff))

    rng = random.Random(902)
    pixels = b''.join(b'\0' + rng.randbytes(32 * 3) for _ in range(32))
    image = (b'\x89PNG\r\n\x1a\n'
             + png_chunk(b'IHDR', struct.pack('>IIBBBBB', 32, 32, 8, 2, 0, 0, 0))
             + png_chunk(b'IDAT', zlib.compress(pixels)) + png_chunk(b'IEND', b''))
    digest = hashlib.sha256(image).hexdigest()
    count = (len(image) + 511) // 512
    assert count > 1
    capture_id = 'receipt-unit-fixture'
    metadata = dict(capture_id=capture_id, poi_id=0, uav_id=0, acquired_at_s=0.,
                    detection_time_s=0., attempt=0, byte_length=len(image), sha256=digest,
                    chunk_count=count, capture_file='sensing/captures/poi-0.png',
                    received_file='sensing/received/poi-0.png', received_at_s=.5)
    scene = {'test_fixture': 'image bytes and delivery only'}
    scene['scene_sha256'] = hashlib.sha256(json.dumps(scene, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    for name in (metadata['capture_file'], metadata['received_file']):
        path = folder / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(image)
        rehash(folder, name)
    write(folder, 'sensing/observations.json', dict(mode='panda3d', scene=scene, observations=[metadata]))

    packets = read(folder, 'packets.json')
    first_id = max((packet.get('packet_id', -1) for packet in packets), default=-1) + 1
    for index in range(count):
        payload = dict(kind='poi_image_chunk', poi_id=0, priority=1., detection_time_s=0.,
                       observation_time_s=0., capture_id=capture_id, image_sha256=digest,
                       chunk_count=count, chunk_index=index,
                       data_b64=base64.b64encode(image[index * 512:(index + 1) * 512]).decode())
        packets.append(dict(type='delivered', packet_id=first_id + index, source_id=0, dest_id=-1,
                            creation_time_s=0., time_s=.5, payload=payload, hops=1,
                            latency_s=.5, reroute_count=0))
    write(folder, 'packets.json', packets)
    events = read(folder, 'events.json')
    events.extend([
        dict(type='poi_detected', time_s=0., uav_id=0, poi_id=0),
        dict(type='poi_received', time_s=.5, poi_id=0, detection_time_s=0.,
             packet_id=first_id + count - 1, delay_s=.5, capture_id=capture_id,
             image_sha256=digest, image_bytes=len(image), chunks=count)])
    write(folder, 'events.json', events)
    metrics = read(folder, 'metrics.json')
    metrics['scenario']['camera_mode'] = 'panda3d'
    metrics['tasks'][0].update(detection_s=0., detected_by=0, report_s=.5)
    metrics['sensing'].update(mode='panda3d', captures=1, received_images=1, received_bytes=len(image))
    metrics['checks'].update(imagery_received=True, reporting_deadline=True)
    metrics['sample_constraints_passed'] = all(metrics['checks'].values())
    write(folder, 'scenario.json', metrics['scenario'])
    write(folder, 'metrics.json', metrics)
    return folder


@pytest.mark.parametrize('name', ['short', 'failed'])
def test_incomplete_physics_run_is_valid_evidence_without_mission_credit(evidence_runs, name):
    result = verify(evidence_runs[name])
    assert result['evidence_valid'], result
    assert result['evidence_errors'] == []
    assert not result['mission_passed'] and not result['submission_ready']
    assert 'all_pois_surveyed_and_reported' in result['failed_constraints']
    assert 'mission_deadline' in result['failed_constraints']
    assert any('not independent continuous dynamics proof' in note for note in result['notes'])


@pytest.mark.parametrize('mutation', ['missing', 'duplicate'])
def test_every_body_frame_requires_all_thirty_unique_ids(tmp_path, evidence_runs, mutation):
    folder = copied_run(tmp_path, evidence_runs)
    values = frames(folder)
    if mutation == 'missing':
        values[0]['uavs'].pop()
    else:
        values[0]['uavs'][-1]['id'] = values[0]['uavs'][0]['id']
    write_frames(folder, values)
    rejected(folder, '30 UAVs' if mutation == 'missing' else 'duplicate or missing')


def test_nonunit_orientation_is_rejected_after_rehash(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    values = frames(folder)
    values[0]['uavs'][0]['quaternion_wxyz'] = [2., 0., 0., 0.]
    write_frames(folder, values)
    rejected(folder, 'quaternion is not normalized')


def test_motor_thrust_cannot_exceed_recorded_physical_limit(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    values = frames(folder)
    limit = read(folder, 'metrics.json')['physics']['parameters']['config']['max_motor_thrust_n']
    values[0]['uavs'][0]['rotor_thrusts_n'][0] = limit + 1.
    write_frames(folder, values)
    rejected(folder, 'exceeds rotor thrust bounds')


def test_failed_aircraft_cannot_retain_motor_force(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs, 'failed')
    values = frames(folder)
    failed = next(u for u in values[-1]['uavs'] if u['id'] == 0)
    assert failed['failed'] and not failed['armed']
    failed['rotor_thrusts_n'][0] = 1.
    write_frames(folder, values)
    rejected(folder, 'thrust while disarmed or failed')


def test_engine_speed_extrema_must_agree_with_reported_speed(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    metrics = read(folder, 'metrics.json')
    metrics['physics']['stats']['max_speed_mps'] += 1.
    write(folder, 'metrics.json', metrics)
    rejected(folder, 'reported speed differs from all-substep engine maximum')


def test_engine_maximum_cannot_hide_a_faster_control_sample(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    metrics = read(folder, 'metrics.json')
    metrics['physics']['stats']['max_speed_mps'] = metrics['maximum_speed_mps'] = 0.
    write(folder, 'metrics.json', metrics)
    rejected(folder, 'safety speed exceeds the engine all-substep maximum')


def test_collision_count_must_include_exact_cumulative_body_contacts(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    metrics = read(folder, 'metrics.json')
    metrics['collision_count'] += 1
    write(folder, 'metrics.json', metrics)
    rejected(folder, 'collision count differs from cumulative')


def test_imagery_count_cannot_invent_a_missing_receipt(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    metrics = read(folder, 'metrics.json')
    assert metrics['sensing']['received_images'] == 0
    metrics['sensing']['received_images'] = 1
    metrics['checks']['imagery_received'] = True
    metrics['sample_constraints_passed'] = all(metrics['checks'].values())
    write(folder, 'metrics.json', metrics)
    result = verify(folder)
    assert not result['evidence_valid'], result
    assert result['evidence_errors']


def test_complete_png_delivery_is_valid_receipt_evidence(tmp_path, evidence_runs):
    folder = with_image_receipt(copied_run(tmp_path, evidence_runs))
    result = verify(folder)
    assert result['evidence_valid'], result
    assert not result['mission_passed']


@pytest.mark.parametrize('damage', ['missing_delivered_chunk', 'changed_received_bytes'])
def test_png_receipt_requires_all_delivered_bytes(tmp_path, evidence_runs, damage):
    folder = with_image_receipt(copied_run(tmp_path, evidence_runs))
    assert verify(folder)['evidence_valid']
    if damage == 'missing_delivered_chunk':
        packets = read(folder, 'packets.json')
        missing = next(index for index, packet in enumerate(packets)
                       if (packet.get('payload') or {}).get('kind') == 'poi_image_chunk')
        del packets[missing]
        write(folder, 'packets.json', packets)
        fragment = 'complete delivered chunk evidence'
    else:
        name = 'sensing/received/poi-0.png'
        path = folder / name
        path.write_bytes(path.read_bytes()[:-1] + b'\0')
        rehash(folder, name)
        fragment = 'GCS image bytes differ'
    rejected(folder, fragment)


@pytest.mark.parametrize('name', ['metrics.json', 'manifest.json'])
def test_explicit_source_instability_invalidates_evidence(tmp_path, evidence_runs, name):
    folder = copied_run(tmp_path, evidence_runs)
    document = read(folder, name)
    document['source_stable'] = False
    write(folder, name, document)
    result = verify(folder)
    assert not result['evidence_valid'], result
    assert result['evidence_errors']


@pytest.mark.parametrize('invalid_field', ['home', 'disarm', 'ground_contact'])
def test_forged_landing_needs_home_disarm_and_contact(tmp_path, evidence_runs, invalid_field):
    folder = copied_run(tmp_path, evidence_runs)
    metrics, events = read(folder, 'metrics.json'), read(folder, 'events.json')
    uid = next(e['uav_id'] for e in events if e['type'] == 'takeoff_started')
    scenario = metrics['scenario']
    columns = scenario['staging_columns']
    rows = (30 + columns - 1) // columns
    spacing = scenario['pad_spacing_m']
    radius = metrics['physics']['parameters']['config']['body_radius_m']
    position = [(uid % columns - (columns - 1) / 2.) * spacing,
                (uid // columns - (rows - 1) / 2.) * spacing, radius]
    event = dict(type='landing_confirmed', time_s=metrics['elapsed_time_s'], uav_id=uid,
                 position=position, velocity=[0., 0., 0.], sortie_s=0.,
                 observed_at_s=metrics['elapsed_time_s'], disarm_acknowledged=True,
                 ground_contact=True, evidence='bullet_contact_and_telemetry')
    if invalid_field == 'home':
        event['position'][0] += 2.
    elif invalid_field == 'disarm':
        event['disarm_acknowledged'] = False
    else:
        event['ground_contact'] = False
    events.append(event)
    write(folder, 'events.json', events)
    rejected(folder, 'landing lacks disarmed ground-contact/home/speed evidence')


def test_missing_control_interval_cannot_be_hidden_by_rehash(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    safety = read(folder, 'physics-safety.json')
    del safety[len(safety) // 2]
    write(folder, 'physics-safety.json', safety)
    rejected(folder, 'safety ledger does not cover every control interval')


def test_airborne_telemetry_cannot_be_relabelled_landed_by_removing_motor_events(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    assert verify(folder)['evidence_valid']
    assert any(u['armed'] for frame in frames(folder) for u in frame['uavs'])
    events = [event for event in read(folder, 'events.json') if event['type'] != 'takeoff_started']
    metrics = read(folder, 'metrics.json')
    metrics['checks']['all_uavs_landed'] = True
    metrics['sample_constraints_passed'] = all(metrics['checks'].values())
    write(folder, 'events.json', events)
    write(folder, 'metrics.json', metrics)
    result = verify(folder)
    assert not result['evidence_valid'], result
    assert result['evidence_errors']


@pytest.mark.parametrize('inventory,required', [
    ('source_sha256', 'simulation/quadrotor_physics.py'),
    ('output_sha256', 'physics-telemetry.jsonl')])
def test_manifest_cannot_omit_required_physics_evidence(tmp_path, evidence_runs, inventory, required):
    folder = copied_run(tmp_path, evidence_runs)
    manifest = read(folder, 'manifest.json')
    del manifest[inventory][required]
    write(folder, 'manifest.json', manifest)
    rejected(folder, 'inventory missing')


def test_source_digest_must_match_current_runtime(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    manifest = read(folder, 'manifest.json')
    manifest['source_sha256']['simulation/quadrotor_physics.py'] = '0' * 64
    write(folder, 'manifest.json', manifest)
    rejected(folder, 'source SHA256 mismatch')


def test_modified_output_requires_matching_digest(tmp_path, evidence_runs):
    folder = copied_run(tmp_path, evidence_runs)
    with (folder / 'events.json').open('a', encoding='utf-8') as stream:
        stream.write(' ')
    rejected(folder, 'output SHA256 mismatch')


@pytest.mark.parametrize('inventory', ['source_sha256', 'output_sha256'])
@pytest.mark.parametrize('unsafe_path', ['../outside.json', r'..\outside.json', 'C:/outside.json'])
def test_manifest_paths_cannot_escape_their_declared_root(tmp_path, evidence_runs, inventory, unsafe_path):
    folder = copied_run(tmp_path, evidence_runs)
    manifest = read(folder, 'manifest.json')
    manifest[inventory][unsafe_path] = '0' * 64
    write(folder, 'manifest.json', manifest)
    rejected(folder, 'path')
