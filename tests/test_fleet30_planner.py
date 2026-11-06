"""Regression tests for 30-aircraft mission allocation and bounded movement."""
import math
import networkx as nx
import numpy as np
import pytest
from core.models import Vector3D,UAVRole,UAVState
from simulation.resilient_uavx import ResilientUAVXSimulator
from simulation.uavx_scenario import ScenarioConfig,ScenarioTask


def motion_fixture(position=(0.,0.,40.),goal=(40.,0.,40.)):
    sim=ResilientUAVXSimulator(ScenarioConfig(fleet_size=30,charging_pads=30,camera_mode='footprint',tasks=(ScenarioTask(0,(900.,0.,0.),0.,1.),)))
    sim.slots={i:((i+1)*44.,0.,40.) for i in range(20)}
    sim.slot_owners={0:0};sim.flights[0].slot=0;sim.flights[0].armed=True;sim.flights[0].landed=False
    sim.state.uavs[0].position=Vector3D(*position);sim.state.uavs[0].target_position=Vector3D(*goal)
    sim.state.uavs[0].role=UAVRole.RELAY
    return sim


def test_near_relay_uses_normal_speed_to_close_gap_when_admission_allows():
    candidate=motion_fixture();candidate._move()
    assert candidate.state.uavs[0].position.x==pytest.approx(2.5)
    assert candidate.max_speed==pytest.approx(5.)
    assert candidate.flights[0].remaining_s==pytest.approx(1199.5)
    assert candidate.interval_downtime==0.
    assert candidate.separation_violations==0


def test_full_speed_does_not_bypass_swept_collision_admission():
    sim=motion_fixture(goal=(100.,0.,40.))
    sim.state.uavs[1].position=Vector3D(22.,0.,40.)
    sim.flights[1].armed=True;sim.flights[1].landed=False
    sim._move()
    assert sim.min_separation>=22.-1e-8
    assert sim.state.uavs[0].position.x<2.5
    assert sim.separation_violations==0 and not sim.collision_pairs
    assert sim.max_speed<=5.+1e-8


def test_full_speed_does_not_break_only_remaining_radio_path():
    sim=motion_fixture(position=(154.1,0.,40.),goal=(250.,0.,40.))
    starts=np.array([u.position.to_array() for u in sim.state.uavs.values()])
    before=sim._reachable_count(starts)
    sim._move()
    ends=np.array([u.position.to_array() for u in sim.state.uavs.values()])
    assert sim._reachable_count(ends,starts)>=before
    assert sim.interval_downtime==0.
    assert math.dist(sim.state.uavs[0].position.to_array(),sim.state.uavs[17].position.to_array())<=100.+1e-8
    assert sim.max_speed<=5.+1e-8


def handoff_fixture(slot=0,remaining=1000.,replacement_armed=True):
    tasks=(ScenarioTask(0,(400.,0.,0.),0.,1.),ScenarioTask(1,(350.,100.,0.),0.,1.))
    sim=ResilientUAVXSimulator(ScenarioConfig(fleet_size=30,charging_pads=30,camera_mode='footprint',tasks=tasks))
    sim.state.time=100.;sim.active_id=0
    sim.tasks[0].surveyed=True;sim.tasks[0].report_s=95.
    sim.slots=sim._build_slots(sim.tasks[0]);assert len(sim.slots)==10
    for i,goal in sim.slots.items():
        u,f=sim.state.uavs[i],sim.flights[i]
        u.position=Vector3D(*goal);u.target_position=Vector3D(*goal)
        f.armed=True;f.landed=False;f.remaining_s=1000.;f.slot=i
        sim.slot_owners[i]=i
    f=sim.flights[20];f.slot=slot;f.remaining_s=remaining;f.armed=replacement_armed;f.landed=not replacement_armed
    sim.replacements[slot]=20
    if replacement_armed:sim.state.uavs[20].position=Vector3D(20.,-20.,65.)
    sim.state.uavs[20].target_position=Vector3D(*sim.slots[slot][:2],65.)
    sim.router.network_graph=nx.complete_graph([-1,*range(30)])
    return sim


def test_valid_handoff_retargets_existing_flight_after_task_pivot():
    sim=handoff_fixture()
    sim._plan()
    assert sim.active_id==1 and len(sim.slots)==9
    assert sim.replacements[0]==20 and sim.flights[20].slot==0
    assert not sim.flights[20].returning
    assert tuple(sim.state.uavs[20].target_position.to_array())==(*sim.slots[0][:2],65.)
    assert any(e['type']=='relay_handoff_retargeted' and e['uav_id']==20 for e in sim.events)


def test_removed_handoff_slot_releases_replacement_and_returns_airborne_uav():
    sim=handoff_fixture(slot=9)
    sim._plan()
    assert 9 not in sim.slots and 9 not in sim.replacements
    assert sim.flights[20].slot is None and sim.flights[20].returning
    assert sim.state.uavs[20].target_position.to_array().tolist()==list(sim.flights[20].home)


def test_infeasible_unlaunched_replacement_is_released_without_energy_reset():
    sim=handoff_fixture(remaining=100.,replacement_armed=False)
    sim._plan()
    assert sim.flights[20].slot is None and sim.replacements.get(0)!=20
    assert sim.state.uavs[20].target_position is None
    assert sim.flights[20].remaining_s==100.


def failure_fixture():
    task=ScenarioTask(0,(150.,0.,0.),0.,1.)
    sim=ResilientUAVXSimulator(ScenarioConfig(fleet_size=30,charging_pads=30,camera_mode='footprint',tasks=(task,)))
    sim.state.time=10.;sim.active_id=0;sim.slots=sim._build_slots(sim.tasks[0])
    for slot,goal in sim.slots.items():
        uid=slot;u,f=sim.state.uavs[uid],sim.flights[uid]
        u.position=Vector3D(*goal);u.target_position=Vector3D(*goal);u.connected_to_gcs=True
        f.slot=slot;f.armed=True;f.landed=False;f.sortie_s=10.;sim.slot_owners[slot]=uid
    replacement=10;goal=(*sim.slots[1][:2],65.)
    sim.replacements[1]=replacement
    u,f=sim.state.uavs[replacement],sim.flights[replacement]
    u.position=Vector3D(*goal);u.target_position=Vector3D(*goal);u.connected_to_gcs=True
    f.slot=1;f.armed=True;f.landed=False;f.sortie_s=5.
    sim.router.network_graph=nx.complete_graph([-1,*range(30)])
    return sim


def assert_consistent(sim):
    for slot,uid in sim.slot_owners.items():
        assert sim.flights[uid].slot==slot
        assert sim.state.uavs[uid].is_operational()
        assert not sim.flights[uid].returning
    for slot,uid in sim.replacements.items():
        assert sim.flights[uid].slot==slot
        assert sim.state.uavs[uid].is_operational()
        assert not sim.flights[uid].returning


def test_failed_owner_promotes_live_replacement_without_resetting_energy():
    sim=failure_fixture();sim.state.uavs[1].state=UAVState.FAILED
    sim.flights[10].remaining_s=900.
    sim._plan()
    assert sim.slot_owners[1]==10 and 1 not in sim.replacements
    assert sim.flights[1].slot is None
    assert sim.flights[10].remaining_s==900.
    assert_consistent(sim)


def test_failed_replacement_cannot_remove_healthy_owner():
    sim=failure_fixture();sim.state.uavs[10].state=UAVState.FAILED
    sim._plan()
    assert sim.slot_owners[1]==1 and 1 not in sim.replacements
    assert sim.flights[10].slot is None
    assert_consistent(sim)


def test_simultaneous_owner_and_replacement_failure_reallocates_required_slot():
    sim=failure_fixture();sim.state.uavs[1].state=sim.state.uavs[10].state=UAVState.FAILED
    sim._plan()
    assert sim.slot_owners[1] not in (1,10)
    assert sim.flights[1].slot is sim.flights[10].slot is None
    assert_consistent(sim)


def test_returning_replacement_releases_reservation_without_stealing_owner():
    sim=failure_fixture();sim.flights[10].returning=True
    sim._plan()
    assert sim.slot_owners[1]==1
    assert 1 not in sim.replacements and sim.flights[10].slot is None
    assert sim.flights[10].returning
    assert_consistent(sim)


def starvation_fixture():
    task=ScenarioTask(0,(751.6994874229113,392.17956770484545,0.),0.,1.)
    sim=ResilientUAVXSimulator(ScenarioConfig(fleet_size=30,charging_pads=30,camera_mode='footprint',tasks=(task,)))
    sim.state.time=600.
    sim.active_id=0
    sim.slots=sim._build_slots(sim.tasks[0])
    assert len(sim.slots)==20
    # Seventeen existing relays retain their assigned mission positions.
    for uid in range(17):
        u,f=sim.state.uavs[uid],sim.flights[uid]
        u.position=Vector3D(*sim.slots[uid]);u.target_position=Vector3D(*sim.slots[uid])
        # Enough energy for the shared withdrawal obligation, but the outer
        # owners still enter optional-replacement territory. The test isolates
        # role ordering rather than requiring an unsafe formation admission.
        f.slot=uid;f.armed=True;f.landed=False;f.sortie_s=550.;f.remaining_s=650.
        sim.slot_owners[uid]=uid
    # Exactly three fresh reserve aircraft; the remaining ten are charging.
    for uid in range(20,30):
        f=sim.flights[uid];f.charge_started_s=590.;f.remaining_s=200.
        sim.state.uavs[uid].state=UAVState.CHARGING
    # The regression isolates allocation ordering, so every existing aircraft
    # has a GCS path and no recovery allocation is forced by a partition.
    sim.router.network_graph=nx.complete_graph([-1,*range(30)])
    return sim


def test_required_mission_slots_are_filled_before_optional_replacements():
    sim=starvation_fixture();sim._plan()
    assert set(sim.slot_owners)==set(range(20))
    assert not sim.replacements
    assert sim.state.uavs[sim.slot_owners[19]].role.value=='scout'
    assert len(sim.flights)==30


def test_missing_roles_do_not_override_shared_withdrawal_energy_requirement():
    sim=starvation_fixture()
    for uid in range(17):
        sim.flights[uid].remaining_s=400.
        sim.flights[uid].sortie_s=800.
    sim._plan()
    assert sim._withdrawal_reason=='formation_admission_failed'
    assert not sim.slot_owners
    assert all(sim.flights[uid].returning for uid in range(17))


def test_failure_during_landing_yield_retains_slot_until_planner_releases_it():
    sim=motion_fixture()
    home=sim.flights[1].home
    returning=sim.flights[1]
    returning.armed=True;returning.landed=False;returning.returning=True
    sim.state.uavs[1].position=Vector3D(*home[:2],70.)
    sim.state.uavs[1].target_position=Vector3D(*home)
    sim.state.uavs[0].position=Vector3D(*home[:2],40.)
    sim.state.uavs[0].target_position=Vector3D(*home[:2],40.)
    sim.flights[0].remaining_s=sim.dt
    sim._move()
    assert any(e['type']=='landing_corridor_yield' and e['uav_id']==0 for e in sim.events)
    assert not sim.state.uavs[0].is_operational()
    assert sim.flights[0].slot==0
    sim._plan()
    assert sim.flights[0].slot is None
    assert 0 not in sim.slot_owners.values()
