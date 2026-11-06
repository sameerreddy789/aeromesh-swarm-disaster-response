"""Reject plausible-looking but unsupported imagery/radio/scenario claims."""
import json
import hashlib
import shutil

import pytest

from simulation.resilient_uavx import ResilientUAVXSimulator
from simulation.uavx_scenario import ScenarioConfig,ScenarioTask
from verify_submission import verify


@pytest.fixture(scope='module')
def source_run(tmp_path_factory):
    folder=tmp_path_factory.mktemp('v2-evidence')/'run'
    scenario=ScenarioConfig(fleet_size=12,packet_loss=0,tasks=(ScenarioTask(0,(175.,0.,0.)),))
    ResilientUAVXSimulator(scenario).run(folder,300)
    return folder


def mutate(folder,name,fn):
    path=folder/name
    data=json.loads(path.read_text())
    fn(data)
    path.write_text(json.dumps(data),encoding='utf-8')
    manifest=json.loads((folder/'manifest.json').read_text())
    manifest['output_sha256'][name]=hashlib.sha256(path.read_bytes()).hexdigest()
    (folder/'manifest.json').write_text(json.dumps(manifest),encoding='utf-8')


@pytest.mark.parametrize('case',['premature_receipt','missing_chunk','overservice','changed_image','invalid_path','false_sensing_count','invented_fault'])
def test_rejects_rehashed_v2_contradictions(tmp_path,source_run,case):
    folder=tmp_path/'run';shutil.copytree(source_run,folder)
    if case=='premature_receipt':
        mutate(folder,'sensing/observations.json',lambda d:d['observations'][0].update(received_at_s=0.))
    elif case=='missing_chunk':
        def remove(d):
            for event in d:
                if (event['type']=='delivered' and (event.get('payload') or {}).get('kind')=='poi_image_chunk'
                        and event['payload']['chunk_index']==0):
                    event['type']='dropped';event['reason']='test_missing'
        mutate(folder,'packets.json',remove)
    elif case=='overservice':
        mutate(folder,'radio-service.json',lambda d:d[1]['node_service_bytes'].update({'0':1e12}))
    elif case=='changed_image':
        path=folder/'sensing/received/poi-0.png';path.write_bytes(path.read_bytes()+b'corrupt')
        mutate(folder,'metrics.json',lambda d:None)
        manifest=json.loads((folder/'manifest.json').read_text())
        manifest['output_sha256']['sensing/received/poi-0.png']=hashlib.sha256(path.read_bytes()).hexdigest()
        (folder/'manifest.json').write_text(json.dumps(manifest))
    elif case=='invalid_path':
        mutate(folder,'sensing/observations.json',lambda d:d['observations'][0].update(capture_file='../outside.png'))
    elif case=='false_sensing_count':
        mutate(folder,'metrics.json',lambda d:d['sensing'].update(received_images=10))
    else:
        mutate(folder,'metrics.json',lambda d:d['faults'].append({'fault_id':0}))
    assert verify(folder)['evidence_errors']


@pytest.mark.parametrize('case', ['zero_usage_with_delivery', 'missing_service_node',
                                  'wrong_wire_size', 'fractional_attempt_count',
                                  'too_few_completed_attempts', 'invented_pending_queue',
                                  'noncanonical_node', 'no_gcs_receive_service'])
def test_radio_ledger_must_support_packet_delivery(tmp_path, source_run, case):
    folder = tmp_path/'run'
    shutil.copytree(source_run, folder)
    if case == 'zero_usage_with_delivery':
        mutate(folder, 'radio-service.json', lambda samples: [s.update(node_service_bytes={}) for s in samples])
        def clear_usage(metrics):
            radio = metrics['packet_statistics']['radio_service']
            for name in ('node_service_bytes','node_tx_bytes','node_rx_bytes'):
                radio[name] = {}
        mutate(folder, 'metrics.json', clear_usage)
    elif case == 'missing_service_node':
        def add_phantom_usage(metrics):
            radio = metrics['packet_statistics']['radio_service']
            radio['node_tx_bytes']['0'] = 1.
            radio['node_rx_bytes']['0'] = 1.
        mutate(folder, 'metrics.json', add_phantom_usage)
    elif case == 'wrong_wire_size':
        mutate(folder, 'packets.json', lambda packets: [p.update(wire_size_bytes=p['wire_size_bytes']-1) for p in packets])
    elif case == 'fractional_attempt_count':
        mutate(folder, 'metrics.json', lambda m: m['packet_statistics']['radio_service'].update(attempts_started=1.5))
    elif case == 'too_few_completed_attempts':
        mutate(folder, 'metrics.json', lambda m: m['packet_statistics']['radio_service'].update(
            attempts_started=0, attempts_completed=0, partial_attempts_aborted=0))
    elif case == 'invented_pending_queue':
        def extra_queue(metrics):
            radio = metrics['packet_statistics']['radio_service']
            radio['queue_bytes']['0'] = 1
            radio['peak_queue_bytes']['0'] = 1
        mutate(folder, 'metrics.json', extra_queue)
    elif case == 'noncanonical_node':
        mutate(folder, 'metrics.json', lambda m: m['packet_statistics']['radio_service']['queue_bytes'].update({'00':0}))
    else:
        # Move the whole GCS receiver ledger to another valid node, keeping all
        # aggregate and per-interval sums self-consistent. Delivered GCS packets
        # must still reject the resulting zero receiver usage at the GCS.
        def move_gcs_samples(samples):
            for sample in samples:
                nodes = sample['node_service_bytes']
                nodes['0'] = nodes.pop('-1', 0)
        def move_gcs_stats(metrics):
            radio = metrics['packet_statistics']['radio_service']
            radio['node_service_bytes']['0'] = radio['node_service_bytes'].pop('-1', 0)
            radio['node_rx_bytes']['0'] = radio['node_rx_bytes'].pop('-1', 0)
        mutate(folder, 'radio-service.json', move_gcs_samples)
        mutate(folder, 'metrics.json', move_gcs_stats)
    result = verify(folder)
    assert result['evidence_errors'], case
    assert not result['sample_run_passed']


def test_current_radio_evidence_is_consistent(source_run):
    assert not verify(source_run)['evidence_errors']
