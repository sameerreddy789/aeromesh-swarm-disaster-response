"""Scenario inputs cannot silently change resources, fault meaning or evidence."""
import json
from pathlib import Path

import pytest

from simulation.uavx_scenario import FaultSpec, ScenarioConfig, ScenarioTask, load_scenario


def test_roundtrip_keeps_explicit_assumptions_and_fault_order(tmp_path):
    scenario = ScenarioConfig(
        seed=7, fleet_size=60, charging_pads=4,
        tasks=[ScenarioTask(8, (175, -50, 0), 12, 5)],
        faults=[FaultSpec('link_outage', 15, (2, -1), 10),
                FaultSpec('radio_outage', 15, 3, 2),
                FaultSpec('vehicle_failure', 50, 'middle_active_relay')])
    path = tmp_path / 'scenario.json'
    path.write_text(json.dumps(scenario.to_dict(), allow_nan=False), encoding='utf-8')
    assert load_scenario(path) == scenario
    assert scenario.faults[0].target == (-1, 2)
    assert [fault.kind for fault in scenario.faults[:2]] == ['link_outage', 'radio_outage']
    assert scenario.to_dict()['tasks'][0]['position'] == [175., -50., 0.]
    assert scenario.to_dict()['faults'][2] == {
        'kind': 'vehicle_failure', 'time_s': 50., 'target': 'middle_active_relay'}


@pytest.mark.parametrize('change', [
    {'fleet_size': True}, {'fleet_size': 0}, {'fleet_size': 73}, {'fleet_size': 30.},
    {'seed': False}, {'seed': 7.2}, {'seed': '42'},
    {'schema_version': True}, {'schema_version': 2},
    {'packet_loss': True}, {'packet_loss': -1}, {'packet_loss': 1},
    {'packet_loss': float('nan')}, {'packet_loss': float('inf')},
    {'relay_protection': 1}, {'relay_protection': 'true'},
    {'radio_bytes_per_s': True}, {'radio_bytes_per_s': 0}, {'radio_bytes_per_s': float('inf')},
    {'radio_queue_bytes': False}, {'radio_queue_bytes': 0}, {'radio_queue_bytes': 1.5},
    {'recharge_s': True}, {'recharge_s': 0}, {'recharge_s': -1},
    {'charging_pads': True}, {'charging_pads': 0}, {'charging_pads': 31},
    {'staging_columns': 0}, {'staging_columns': 7}, {'staging_columns': True},
    {'pad_spacing_m': 19.9}, {'pad_spacing_m': 31}, {'pad_spacing_m': float('nan')},
    {'fleet_size': 72, 'staging_columns': 1},
    {'camera_mode': 'real'}, {'camera_mode': True},
    {'camera_width': True}, {'camera_width': 31}, {'camera_height': 1025},
    {'tasks': []}, {'tasks': 'seeded'}, {'faults': None}, {'faults': 'none'},
    {'mission_deadline': 9999}, {'ignore_failed_uavs': True},
])
def test_invalid_scenario_fields_are_rejected(change):
    with pytest.raises(ValueError):
        ScenarioConfig.from_dict(change)


@pytest.mark.parametrize('change', [
    {'id': True}, {'id': -1}, {'id': 1.5}, {'id': '0'},
    {'position': [75, 0]}, {'position': '75,0,0'},
    {'position': [74, 0, 0]}, {'position': [1076, 0, 0]},
    {'position': [75, -501, 0]}, {'position': [75, 501, 0]},
    {'position': [75, 0, 1]}, {'position': [75, 0, False]},
    {'position': [75, float('nan'), 0]},
    {'spawn_s': -1}, {'spawn_s': 2701}, {'spawn_s': True},
    {'priority': 0}, {'priority': False}, {'priority': float('inf')},
    {'surveyed': True}, {'report_s': 1},
])
def test_task_scenario_cannot_preload_success_or_invalid_geometry(change):
    data = {'id': 0, 'position': [75, 0, 0], **change}
    with pytest.raises(ValueError):
        ScenarioTask.from_dict(data)


@pytest.mark.parametrize('data', [
    {}, {'id': 0}, {'position': [75, 0, 0]}, None, [],
])
def test_missing_or_nonobject_tasks_rejected(data):
    with pytest.raises(ValueError):
        ScenarioTask.from_dict(data)


@pytest.mark.parametrize('change', [
    {'kind': 'propulsion_loss'}, {'kind': 'failure'}, {'kind': True},
    {'time_s': True}, {'time_s': -1}, {'time_s': 2700}, {'time_s': float('nan')},
    {'target': -1}, {'target': True}, {'target': 1.5}, {'target': [0, 1]},
    {'target': 'middle_active_link'}, {'target': 'any'},
    {'duration_s': 0}, {'duration_s': -1}, {'duration_s': True}, {'duration_s': float('inf')},
    {'ignore_in_metrics': True},
])
def test_invalid_radio_fault_rejected(change):
    data = {'kind': 'radio_outage', 'time_s': 10, 'target': 2, 'duration_s': 3, **change}
    with pytest.raises(ValueError):
        FaultSpec.from_dict(data)


@pytest.mark.parametrize('target', [[0, 0], [-2, 0], [True, 1], [0, 1.5], [0], [0, 1, 2], 0,
                                    'middle_active_relay', {'a': 0, 'b': 1}])
def test_invalid_link_fault_target_rejected(target):
    with pytest.raises(ValueError):
        FaultSpec('link_outage', 1, target, 3)


def test_fault_duration_is_required_only_for_temporary_faults():
    with pytest.raises(ValueError):
        FaultSpec('radio_outage', 10, 2)
    with pytest.raises(ValueError):
        FaultSpec('link_outage', 10, (0, 1))
    with pytest.raises(ValueError):
        FaultSpec('vehicle_failure', 10, 2, 3)
    assert FaultSpec('vehicle_failure', 10, 2).duration_s is None


def test_duplicate_task_id_and_out_of_fleet_fault_rejected():
    with pytest.raises(ValueError, match='unique'):
        ScenarioConfig(tasks=[ScenarioTask(1, (75, 0, 0)), ScenarioTask(1, (100, 0, 0))])
    for fault in (FaultSpec('radio_outage', 1, 30, 3), FaultSpec('link_outage', 1, (-1, 30), 3)):
        with pytest.raises(ValueError, match='outside the configured fleet'):
            ScenarioConfig(faults=[fault])


def test_fault_schedule_keeps_causality_and_unambiguous_durations():
    with pytest.raises(ValueError, match='ordered'):
        ScenarioConfig(faults=[FaultSpec('radio_outage', 20, 1, 3), FaultSpec('radio_outage', 10, 2, 3)])
    with pytest.raises(ValueError, match='duplicate'):
        ScenarioConfig(faults=[FaultSpec('radio_outage', 20, 1, 3)] * 2)
    with pytest.raises(ValueError, match='overlapping'):
        ScenarioConfig(faults=[FaultSpec('link_outage', 20, (1, 2), 10),
                               FaultSpec('link_outage', 21, (2, 1), 3)])
    with pytest.raises(ValueError, match='overlapping'):
        ScenarioConfig(faults=[FaultSpec('vehicle_failure', 20, 1), FaultSpec('radio_outage', 25, 1, 3)])
    config = ScenarioConfig(faults=[FaultSpec('radio_outage', 20, 1, 3), FaultSpec('radio_outage', 23, 1, 3)])
    assert len(config.faults) == 2
    # Repeated selectors can resolve different operational relays after failures.
    assert len(ScenarioConfig(faults=[FaultSpec('vehicle_failure', 20, 'middle_active_relay'),
                                     FaultSpec('vehicle_failure', 30, 'middle_active_relay')]).faults) == 2


@pytest.mark.parametrize('raw', [
    '{"seed": 42, "seed": 7}', '{"packet_loss": NaN}', '{"radio_bytes_per_s": Infinity}',
    '{"packet_loss": 1e999}', '{"faults": [{"kind":"radio_outage", "kind":"vehicle_failure"}]}',
    '[]', 'null', '{invalid}',
])
def test_json_does_not_accept_ambiguous_keys_or_nonfinite_numbers(tmp_path, raw):
    path = tmp_path / 'bad.json'
    path.write_text(raw, encoding='utf-8')
    with pytest.raises(ValueError):
        load_scenario(path)


def test_direct_constructors_are_validated_and_inputs_are_immutable():
    with pytest.raises(ValueError):
        ScenarioConfig(faults=[{'kind': 'radio_outage'}])
    with pytest.raises(ValueError):
        ScenarioConfig(tasks=[{'id': 0}])
    positions = [75., 0., 0.]
    tasks = [ScenarioTask(0, positions)]
    config = ScenarioConfig(tasks=tasks)
    positions[0] = 1000
    tasks.clear()
    assert config.tasks[0].position == (75., 0., 0.)
    assert config.radio_bytes_per_s > 0 and config.radio_queue_bytes > 0


def test_shipped_scenarios_declare_resources_and_roundtrip():
    root = Path(__file__).resolve().parents[1]
    paths = sorted((root / 'scenarios/uavx').glob('*.json'))
    assert {path.stem for path in paths} >= {'nominal', 'link-outage', 'radio-outage', 'permanent-failure'}
    for path in paths:
        scenario = load_scenario(path)
        assert scenario == ScenarioConfig.from_dict(scenario.to_dict())
        assert scenario.fleet_size == 60 and scenario.charging_pads == 60
        assert scenario.relay_protection and scenario.camera_mode == 'panda3d'
        assert scenario.radio_bytes_per_s == 131072


def test_utf8_bom_and_default_seeded_scenario_are_supported(tmp_path):
    path = tmp_path / 'defaults.json'
    path.write_text('{}', encoding='utf-8-sig')
    assert load_scenario(path) == ScenarioConfig()
    assert ScenarioConfig(fleet_size=72).fleet_size == 72
