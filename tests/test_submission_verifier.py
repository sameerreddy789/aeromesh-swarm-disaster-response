"""Evidence verification must reject omissions and contradictory producer claims."""
import hashlib
import json
import shutil

import pytest

from simulation.uavx import SurveyTask, UAVXSimulator
from verify_submission import CHECKS, verify


@pytest.fixture(scope='module')
def evidence_runs(tmp_path_factory):
    root = tmp_path_factory.mktemp('verifier-fixtures')
    short = root / 'short'
    UAVXSimulator(fleet_size=2, tasks=[SurveyTask(0, (175., 0., 0.), 0., 1.)], packet_loss=0).run(short, 3)
    complete = root / 'complete'
    sim = UAVXSimulator(fleet_size=5, tasks=[SurveyTask(0, (180., 0., 0.), 0., 1.)], packet_loss=0)
    result = sim.run(complete, 300)
    assert result['checks']['all_uavs_landed'] and result['completion_time_s'] is not None
    return {'short': short, 'complete': complete}


def copy_run(tmp_path, evidence_runs, name='short'):
    return shutil.copytree(evidence_runs[name], tmp_path / 'run')


def read(folder, name):
    return json.loads((folder / name).read_text(encoding='utf-8'))


def write(folder, name, data):
    """Rehash modified artifacts so the consistency check, not just hashes, is tested."""
    path = folder / name
    path.write_text(json.dumps(data), encoding='utf-8')
    if name != 'manifest.json':
        manifest = read(folder, 'manifest.json')
        manifest['output_sha256'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
        write(folder, 'manifest.json', manifest)


def rejected(folder, text=None):
    result = verify(folder)
    assert not result['sample_run_passed']
    assert result['evidence_errors']
    if text:
        assert any(text in error for error in result['evidence_errors']), result


def test_short_failed_run_is_valid_evidence(evidence_runs):
    result = verify(evidence_runs['short'])
    assert not result['evidence_errors']
    assert not result['sample_run_passed']
    assert 'sample_has_ten_pois' in result['failed_constraints']
    assert 'not independent full dynamics proof' in result['note']
    assert not result['submission_ready']


def test_empty_fabricated_success_is_rejected(tmp_path):
    (tmp_path / 'metrics.json').write_text(json.dumps(dict(
        packet_statistics=dict(packets_sent=0, packets_delivered=0, packets_dropped=0),
        tasks=[], checks=dict.fromkeys(CHECKS, True), sample_constraints_passed=True)))
    (tmp_path / 'packets.json').write_text('[]')
    (tmp_path / 'manifest.json').write_text('{"output_sha256": {}}')
    rejected(tmp_path)


@pytest.mark.parametrize('collection,key', [
    ('output_sha256', 'events.json'), ('output_sha256', 'trajectory.jsonl'),
    ('source_sha256', 'simulation/uavx.py'),
])
def test_manifest_requires_complete_inventory(tmp_path, evidence_runs, collection, key):
    folder = copy_run(tmp_path, evidence_runs)
    manifest = read(folder, 'manifest.json')
    del manifest[collection][key]
    write(folder, 'manifest.json', manifest)
    rejected(folder, 'inventory')


@pytest.mark.parametrize('digest', ['invalid', '0' * 64])
def test_source_hashes_are_valid_and_match_runtime(tmp_path, evidence_runs, digest):
    folder = copy_run(tmp_path, evidence_runs)
    manifest = read(folder, 'manifest.json')
    manifest['source_sha256']['simulation/uavx.py'] = digest
    write(folder, 'manifest.json', manifest)
    rejected(folder)


def test_missing_artifact_is_not_valid_evidence(tmp_path, evidence_runs):
    folder = copy_run(tmp_path, evidence_runs)
    (folder / 'events.json').unlink()
    rejected(folder)


@pytest.mark.parametrize('name', ['manifest.json', 'metrics.json'])
@pytest.mark.parametrize('version', [True, 999])
def test_schema_version_must_be_supported_integer(tmp_path, evidence_runs, name, version):
    folder = copy_run(tmp_path, evidence_runs)
    data = read(folder, name)
    data['schema_version'] = version
    write(folder, name, data)
    rejected(folder, 'schema')


@pytest.mark.parametrize('field,value', [
    ('maximum_speed_mps', 9.), ('maximum_speed_mps', 0.), ('maximum_altitude_m', 150.), ('maximum_sortie_s', 1400.),
    ('pois_total', 10), ('communication_downtime_s', 1.), ('completion_time_s', 0.),
    ('maximum_speed_mps', float('nan')),
])
def test_contradictory_measurements_cannot_preserve_passing_checks(tmp_path, evidence_runs, field, value):
    folder = copy_run(tmp_path, evidence_runs)
    metrics = read(folder, 'metrics.json')
    metrics[field] = value
    write(folder, 'metrics.json', metrics)
    rejected(folder)


def test_missing_measurement_is_rejected(tmp_path, evidence_runs):
    folder = copy_run(tmp_path, evidence_runs)
    metrics = read(folder, 'metrics.json')
    del metrics['maximum_sortie_s']
    write(folder, 'metrics.json', metrics)
    rejected(folder)


def test_landed_flag_needs_landing_evidence(tmp_path, evidence_runs):
    folder = copy_run(tmp_path, evidence_runs)
    metrics = read(folder, 'metrics.json')
    metrics['checks']['all_uavs_landed'] = True
    write(folder, 'metrics.json', metrics)
    rejected(folder, 'all_uavs_landed')


def test_packet_offer_and_terminal_provenance_must_agree(tmp_path, evidence_runs):
    folder = copy_run(tmp_path, evidence_runs)
    packets = read(folder, 'packets.json')
    delivered_id = next(e['packet_id'] for e in packets if e['type'] == 'delivered')
    offer = next(e for e in packets if e['type'] == 'offered' and e['packet_id'] == delivered_id)
    offer['payload'] = {'kind': 'changed_after_offer'}
    write(folder, 'packets.json', packets)
    rejected(folder, 'provenance')


def test_missing_connectivity_interval_is_rejected(tmp_path, evidence_runs):
    folder = copy_run(tmp_path, evidence_runs)
    samples = read(folder, 'connectivity.json')
    samples.pop(1)
    write(folder, 'connectivity.json', samples)
    rejected(folder, 'timeline')


def test_landing_after_last_sample_is_accepted(evidence_runs):
    folder = evidence_runs['complete']
    frames = [json.loads(line) for line in (folder / 'trajectory.jsonl').read_text().splitlines()]
    last_landing = max(e['time_s'] for e in read(folder, 'events.json') if e['type'] == 'landing_confirmed')
    assert last_landing > frames[-1]['time_s']
    assert not verify(folder)['evidence_errors']


@pytest.mark.parametrize('change', ['delete', 'wrong_home', 'future_observation'])
def test_completed_run_needs_valid_final_landing_event(tmp_path, evidence_runs, change):
    folder = copy_run(tmp_path, evidence_runs, 'complete')
    events = read(folder, 'events.json')
    landing = max((e for e in events if e['type'] == 'landing_confirmed'), key=lambda e: e['time_s'])
    if change == 'delete':
        events.remove(landing)
    elif change == 'wrong_home':
        landing['position'][0] += 10
    else:
        landing['observed_at_s'] += 1
    write(folder, 'events.json', events)
    rejected(folder)


def test_poi_report_time_needs_actual_first_receipt(tmp_path, evidence_runs):
    folder = copy_run(tmp_path, evidence_runs, 'complete')
    metrics = read(folder, 'metrics.json')
    metrics['tasks'][0]['report_s'] += .5
    write(folder, 'metrics.json', metrics)
    rejected(folder, 'first GCS receipt')
