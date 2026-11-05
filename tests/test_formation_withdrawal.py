"""Complete mission admission and energy-cohort withdrawal regressions."""
import math

import pytest

from core.models import UAVRole, UAVState, Vector3D
from simulation.resilient_uavx import ResilientUAVXSimulator
from simulation.uavx_scenario import ScenarioConfig, ScenarioTask


def cohort_fixture():
    tasks=(ScenarioTask(0,(1050.,350.,0.),0.),ScenarioTask(1,(600.,0.,0.),0.))
    sim=ResilientUAVXSimulator(ScenarioConfig(fleet_size=30,charging_pads=30,
                                           camera_mode='footprint',tasks=tasks))
    sim.state.time=1034.
    sim.active_id=0
    sim.slots=sim._build_slots(sim.tasks[0])
    assert len(sim.slots)==26
    for uid,goal in sim.slots.items():
        f,u=sim.flights[uid],sim.state.uavs[uid]
        f.armed=True;f.landed=False;f.slot=uid
        f.remaining_s=581. if uid<11 else 710. if uid<20 else 925.5
        f.sortie_s=1200.-f.remaining_s
        u.position=Vector3D(*goal);u.target_position=Vector3D(*goal)
        u.connected_to_gcs=True;u.role=UAVRole.RELAY
        sim.slot_owners[uid]=uid
    sim._network()
    return sim


def test_shrinking_chain_retains_fresh_outer_cohort_without_resetting_energy():
    sim=cohort_fixture()
    proposal=sim._budgeted_assignment(sim.tasks[1])
    assert proposal is not None
    assert len(proposal['owners'])==14
    assert len(set(proposal['owners'].values()))==14
    assert set(range(20,26))<=set(proposal['owners'].values())
    clocks={uid:(f.remaining_s,f.sortie_s) for uid,f in sim.flights.items()}
    sim._commit_formation(sim.tasks[1],proposal)
    assert clocks=={uid:(f.remaining_s,f.sortie_s) for uid,f in sim.flights.items()}
    assert all(not sim.flights[uid].returning for uid in range(20,26))
    assert sum(sim.flights[uid].returning for uid in range(11))>=8


def test_short_task_pivot_reuses_fully_capable_airborne_cohort():
    sim=cohort_fixture()
    for uid in range(26):
        sim.flights[uid].remaining_s=850.
        sim.flights[uid].sortie_s=350.
    task=sim.tasks[1]
    task.position=(1050.,340.,0.)
    proposal=sim._budgeted_assignment(task)
    assert proposal is not None
    assert len(proposal['owners'])==26
    assert set(proposal['owners'].values())==set(range(26))
    assert proposal['deployment_s']<5.


def test_incomplete_matching_does_not_launch_a_partial_far_formation():
    sim=cohort_fixture()
    for uid,f in sim.flights.items():
        if uid>=10:
            f.charge_started_s=1000.
            f.armed=False;f.landed=True;f.slot=None
            sim.state.uavs[uid].state=UAVState.CHARGING
    sim.slot_owners={slot:uid for slot,uid in sim.slot_owners.items() if uid<10}
    before=(dict(sim.slot_owners),{uid:f.slot for uid,f in sim.flights.items()})
    assert sim._budgeted_assignment(sim.tasks[0]) is None
    assert before==(dict(sim.slot_owners),{uid:f.slot for uid,f in sim.flights.items()})
    sim._plan()
    assert sim._withdrawal_reason=='formation_admission_failed'
    assert not sim.slot_owners and not sim.replacements
    assert all(f.returning for uid,f in sim.flights.items() if uid<10)


def test_inner_relay_budget_accounts_for_distant_aircraft_return():
    sim=cohort_fixture()
    common=sim._withdrawal_path_s()
    inner=sim.state.uavs[0].position.to_array()
    own=math.dist(inner,sim.flights[0].home)/5.
    assert common>own+150.


def test_deadline_and_energy_reject_assignment_without_changing_clock():
    sim=cohort_fixture();sim.state.time=2660.
    before={uid:(f.remaining_s,f.sortie_s) for uid,f in sim.flights.items()}
    assert sim._budgeted_assignment(sim.tasks[1]) is None
    assert before=={uid:(f.remaining_s,f.sortie_s) for uid,f in sim.flights.items()}


def test_withdrawal_preserves_continuous_service_until_measured_landing():
    sim=cohort_fixture()
    for task in sim.tasks: task.spawn_s=2400.
    for uid,f in sim.flights.items():
        if f.armed: f.remaining_s=550.;f.sortie_s=650.
    sim._start_connected_withdrawal('test_shared_return')
    for _ in range(800):
        sim.step()
        if all(f.landed and not f.armed for f in sim.flights.values()):
            break
    report=sim.report()
    assert report['checks']['all_uavs_landed']
    assert report['communication_downtime_s']==pytest.approx(0.,abs=1e-8)
    assert report['battery_exhaustions']==0
    assert report['collision_count']==0
    assert report['minimum_separation_m']>=22.-1e-8
    assert report['maximum_sortie_s']<=1200.
