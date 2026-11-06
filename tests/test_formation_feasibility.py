"""Planner feasibility regressions; these fixtures do not execute a mission."""
import math

import pytest

from core.models import UAVRole, UAVState, Vector3D
from simulation.resilient_uavx import ResilientUAVXSimulator
from simulation.uavx_scenario import ScenarioConfig, ScenarioTask


def _two_slot_fixture():
    sim = ResilientUAVXSimulator(ScenarioConfig(
        fleet_size=30, charging_pads=30, camera_mode='footprint',
        tasks=(ScenarioTask(0, (75., 0., 0.), 0.),)))
    sim.active_id = 0
    sim.slots = sim._build_slots(sim.tasks[0])
    assert len(sim.slots) == 2
    for uid in range(2, 30):
        flight = sim.flights[uid]
        flight.charge_started_s = 0.
        flight.remaining_s = 200.
        flight.sortie_s = flight.max_sortie_s = 1000.
        sim.state.uavs[uid].state = UAVState.CHARGING
    return sim


def _clock_snapshot(sim):
    return {uid: (f.remaining_s, f.sortie_s, f.max_sortie_s, f.charge_started_s)
            for uid, f in sim.flights.items()}


def _position_airborne(sim, uid, position, remaining):
    flight, uav = sim.flights[uid], sim.state.uavs[uid]
    flight.armed, flight.landed = True, False
    flight.charge_started_s = None
    flight.remaining_s = remaining
    flight.sortie_s = flight.max_sortie_s = 1200. - remaining
    uav.position = Vector3D(*position)
    uav.state, uav.role = UAVState.RELAYING, UAVRole.RELAY


def _shorter_deployment_fixture():
    sim = _two_slot_fixture()
    sim.slot_owners = {0: 0}
    sim.flights[0].slot = 0
    _position_airborne(sim, 0, sim.slots[0], 1200.)
    sim.state.uavs[0].target_position = Vector3D(*sim.slots[0])
    # Use the measured topology, not an independently asserted connected flag.
    sim._network()
    return sim


def _assert_swapped_complete_assignment(sim, proposal):
    assert proposal is not None
    assert proposal['owners'] == {0: 1, 1: 0}
    # UAV1 takes off vertically before crossing to the near slot; UAV0 moves
    # 37.5 m to the far slot. This alternative's longest arrival is 26.028 s.
    expected = 8. + math.hypot(75., 50.) / 5.
    assert proposal['deployment_s'] == pytest.approx(expected)
    assert len(set(proposal['owners'].values())) == len(proposal['goals']) == 2
    assert not proposal['handoffs']


def test_complete_alternative_survives_minimum_sum_energy_rejection():
    sim = _shorter_deployment_fixture()
    common = 40. + sim._withdrawal_path_s(sim.slots.values()) + 180.
    flight = sim.flights[0]
    flight.remaining_s = common + 30.
    flight.sortie_s = flight.max_sortie_s = 1200. - flight.remaining_s
    clocks = _clock_snapshot(sim)
    owners = dict(sim.slot_owners)

    proposal = sim._budgeted_assignment(sim.tasks[0])

    assert _clock_snapshot(sim) == clocks
    assert sim.slot_owners == owners
    _assert_swapped_complete_assignment(sim, proposal)
    assert proposal['required_energy_s'] <= flight.remaining_s
    sim._commit_formation(sim.tasks[0], proposal)
    assert _clock_snapshot(sim) == clocks
    assert sim.flights[0].slot == 1 and sim.flights[1].slot == 0
    assert not sim.flights[0].returning
    assert all(sim.state.uavs[uid].target_position is None for uid in range(2, 30))


def test_complete_alternative_survives_minimum_sum_deadline_rejection():
    sim = _shorter_deployment_fixture()
    retreat = sim._withdrawal_path_s(sim.slots.values())
    # There is room for the 26.028 s alternative, but not the minimum-sum
    # assignment whose fresh aircraft needs 32.622 s to reach the far slot.
    sim.state.time = 2700. - 40. - retreat - 30. - 30.
    clocks = _clock_snapshot(sim)

    proposal = sim._budgeted_assignment(sim.tasks[0])

    _assert_swapped_complete_assignment(sim, proposal)
    assert _clock_snapshot(sim) == clocks
    assert sim.state.time + proposal['deployment_s'] + 40. + retreat + 30. <= 2700.


def _retiring_aircraft_fixture(wait_budget):
    sim = _two_slot_fixture()
    # This low-energy incumbent can cover a short deployment and return, but
    # cannot accept the full work/withdrawal obligation of another mission.
    # All mission candidates are fresh: enumerating their energy caps alone
    # would overlook the smaller, feasible bound set by the retiring aircraft.
    uid = 2
    _position_airborne(sim, uid, (*sim.flights[uid].home[:2], 40.),
                       36. + 180. + wait_budget)
    sim.flights[uid].slot = 0
    sim.slot_owners = {0: uid}
    sim.state.uavs[uid].target_position = Vector3D(*sim.slots[0])
    sim._network()
    return sim


def test_retirement_bound_admits_short_deployment_and_preserves_return_reserve():
    sim = _retiring_aircraft_fixture(wait_budget=40.)
    clocks = _clock_snapshot(sim)

    proposal = sim._budgeted_assignment(sim.tasks[0])

    assert proposal is not None
    assert set(proposal['owners'].values()) == {0, 1}
    assert _clock_snapshot(sim) == clocks
    assert proposal['deployment_s'] <= 40.
    assert sim.flights[2].remaining_s >= proposal['deployment_s'] + 36. + 180.
    assert all(sim.flights[uid].remaining_s >= proposal['required_energy_s']
               for uid in proposal['owners'].values())
    sim._commit_formation(sim.tasks[0], proposal)
    assert _clock_snapshot(sim) == clocks
    assert sim.flights[2].returning and sim.flights[2].slot is None
    assert sim.state.uavs[2].target_position.to_array().tolist() == list(sim.flights[2].home)


def test_insufficient_retiring_wait_budget_rejects_without_mutation():
    sim = _retiring_aircraft_fixture(wait_budget=25.)
    clocks, owners = _clock_snapshot(sim), dict(sim.slot_owners)

    assert sim._budgeted_assignment(sim.tasks[0]) is None

    assert _clock_snapshot(sim) == clocks
    assert sim.slot_owners == owners
    assert not sim.flights[2].returning
