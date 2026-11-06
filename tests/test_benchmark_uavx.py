"""Qualification resource invariants independent of individual mission outcomes."""
from pathlib import Path

import pytest

from scripts.benchmark_uavx import build_matrix
from simulation.uavx_scenario import ScenarioConfig, load_scenario


def test_complete_matrix_keeps_30_aircraft_and_stresses_charging_capacity():
    root=Path(__file__).resolve().parents[1]
    base=load_scenario(root/'scenarios/uavx/nominal.json')
    cases=dict(build_matrix(base))
    assert len(cases)==19
    assert all(scenario.fleet_size==30 for scenario in cases.values())
    assert cases['charging-pads15-seed42'].charging_pads==15
    assert cases['nominal-seed42'].charging_pads==30
    for name in ('radio-outage-seed42','permanent-failure-seed42',
                 'early-link-seed42','late-link-seed42'):
        assert cases[name].faults


@pytest.mark.parametrize('fleet_size',[29,60,72])
def test_qualification_matrix_rejects_a_changed_total_fleet(fleet_size):
    with pytest.raises(ValueError,match='exactly 30'):
        build_matrix(ScenarioConfig(fleet_size=fleet_size))


def test_quick_matrix_retains_fixed_fleet_and_declared_failures():
    cases=dict(build_matrix(ScenarioConfig(),quick=True))
    assert len(cases)==4
    assert all(scenario.fleet_size==30 for scenario in cases.values())
    assert cases['permanent-failure-seed42'].faults[0].kind=='vehicle_failure'


def test_reduced_charger_configuration_never_adds_pads_or_aircraft():
    base=ScenarioConfig(charging_pads=3)
    cases=dict(build_matrix(base,seeds=(42,)))
    assert cases['charging-pads1-seed42'].charging_pads==1
    assert all(scenario.fleet_size==base.fleet_size for scenario in cases.values())


def test_default_scenario_and_no_scenario_cli_use_30_aircraft():
    from simulation.uavx import UAVXSimulator
    assert ScenarioConfig().fleet_size==30
    assert len(UAVXSimulator().flights)==30
