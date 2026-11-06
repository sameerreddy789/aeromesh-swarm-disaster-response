"""Measured progress regressions from the 30-UAV nominal seed 109 audit."""

import pytest

from core.models import UAVRole, UAVState, Vector3D
from simulation.resilient_uavx import ResilientUAVXSimulator
from simulation.uavx_scenario import ScenarioConfig, ScenarioTask


def _fixture(tasks):
    sim = ResilientUAVXSimulator(ScenarioConfig(
        fleet_size=30, charging_pads=30, camera_mode='footprint', tasks=tasks))
    sim.active_id = 0
    sim.slots = sim._build_slots(sim.tasks[0])
    assert len(sim.slots) == 4
    # Keep the declared fleet and its real pad obstacles. Charging aircraft
    # cannot substitute for the four aircraft whose crossing is under test.
    for uid, flight in sim.flights.items():
        if uid >= 4:
            flight.charge_started_s = 0.
            flight.remaining_s = 200.
            flight.sortie_s = flight.max_sortie_s = 1000.
            sim.state.uavs[uid].state = UAVState.CHARGING
    return sim


def _airborne(sim, uid, position, slot, remaining=1200.):
    flight, uav = sim.flights[uid], sim.state.uavs[uid]
    flight.armed, flight.landed = True, False
    flight.charge_started_s = None
    flight.remaining_s = remaining
    flight.sortie_s = flight.max_sortie_s = 1200. - remaining
    flight.slot = slot
    uav.position = Vector3D(*position)
    uav.target_position = Vector3D(*sim.slots[slot])
    uav.state, uav.role = UAVState.RELAYING, UAVRole.RELAY


def _clocks(sim):
    return {uid: (f.remaining_s, f.sortie_s, f.max_sortie_s)
            for uid, f in sim.flights.items()}


def test_crossed_cohort_assignment_reaches_slots_with_bounded_safe_motion():
    sim = _fixture((ScenarioTask(0, (170., -12.5, 0.)),))
    positions = [sim.slots[0], sim.slots[1], sim.slots[2], (20., -12.5, 40.)]
    for uid, position in enumerate(positions):
        _airborne(sim, uid, position, uid)
        sim.slot_owners[uid] = uid
    sim._network()
    clocks = _clocks(sim)

    proposal = sim._budgeted_assignment(sim.tasks[0])

    assert proposal is not None
    assert set(proposal['owners'].values()) == set(range(4))
    assert _clocks(sim) == clocks
    sim._commit_formation(sim.tasks[0], proposal)
    assert _clocks(sim) == clocks
    goals = dict(sim.slots)
    downtime = 0.
    # Keeping UAV3 behind the three settled owners creates a permanent
    # x=20 <-> 22.5 m oscillation. A safe admission must produce progress,
    # not merely a complete matching or a collision-free stationary fleet.
    for _ in range(120):
        sim._move()
        sim.state.time += sim.dt
        downtime += sim.interval_downtime

    assert all(sim.state.uavs[uid].distance_to(Vector3D(*goals[slot])) <= .25
               for slot, uid in sim.slot_owners.items())
    assert sim.min_separation >= 22. - 1e-8
    assert sim.max_speed <= 5. + 1e-8
    assert downtime == pytest.approx(0., abs=1e-8)
    assert sim.exhaustions == 0
    assert all(sim.flights[uid].remaining_s == pytest.approx(1140.)
               for uid in range(4))
    assert not any(event['type'] == 'replacement_dispatched' for event in sim.events)


def _ready_idle_handoff_fixture(future_position=(75., -500., 0.)):
    tasks = (ScenarioTask(0, (170., -12.5, 0.)),
             ScenarioTask(1, future_position, spawn_s=900., priority=5.))
    sim = _fixture(tasks)
    sim.state.time = 100.
    sim.tasks[0].surveyed = True
    sim.tasks[0].dwell_s = 30.
    sim.tasks[0].report_s = 95.
    sim._idle_since_s = 95.
    for uid, goal in sim.slots.items():
        _airborne(sim, uid, goal, uid, remaining=500. if uid == 3 else 1000.)
        sim.slot_owners[uid] = uid
    replacement, incumbent, slot = 4, 3, 3
    handoff_goal = (*sim.slots[slot][:2], 65.)
    _airborne(sim, replacement, handoff_goal, slot, remaining=1000.)
    sim.state.uavs[replacement].target_position = Vector3D(*handoff_goal)
    sim.replacements[slot] = replacement
    sim._network()
    assert sim.state.uavs[replacement].connected_to_gcs
    assert sim.state.uavs[replacement].distance_to(Vector3D(*handoff_goal)) == 0.
    return sim


@pytest.mark.parametrize('future_position', [(75., -500., 0.), (1075., 500., 0.)])
def test_ready_handoff_completes_during_unannounced_task_grace(future_position):
    sim = _ready_idle_handoff_fixture(future_position)
    replacement, incumbent, slot = 4, 3, 3
    clocks = _clocks(sim)

    sim._plan()

    assert sim.slot_owners[slot] == replacement
    assert slot not in sim.replacements
    assert sim.flights[incumbent].returning
    assert not sim.flights[replacement].returning
    assert sim.flights[replacement].slot == slot
    assert _clocks(sim) == clocks
    assert sim.active_id == 0 and sim._withdrawal_reason is None
    assert not sim.shutdown
    assert any(event['type'] == 'relay_handoff_completed'
               and event['old_uav_id'] == incumbent
               and event['new_uav_id'] == replacement for event in sim.events)
    assert not any(event['type'] in ('formation_committed', 'task_selected')
                   for event in sim.events)


@pytest.mark.parametrize('blocked_by', [
    'unarmed', 'returning', 'wrong_slot', 'failed', 'disconnected',
    'in_transit', 'invalid_position',
])
def test_idle_handoff_keeps_incumbent_without_valid_arrival(blocked_by):
    sim = _ready_idle_handoff_fixture()
    replacement, incumbent, slot = 4, 3, 3
    flight, uav = sim.flights[replacement], sim.state.uavs[replacement]
    if blocked_by == 'unarmed':
        flight.armed, flight.landed = False, True
    elif blocked_by == 'returning':
        flight.returning = True
    elif blocked_by == 'wrong_slot':
        flight.slot = 2
    elif blocked_by == 'failed':
        uav.state = UAVState.FAILED
    elif blocked_by == 'disconnected':
        uav.connected_to_gcs = False
    elif blocked_by == 'in_transit':
        uav.position.x += 4.
    else:
        # NaN must never satisfy the observed-arrival gate through the fact
        # that a greater-than comparison against NaN evaluates to false.
        uav.position.x = float('nan')
    clocks = _clocks(sim)

    sim._plan()

    assert sim.slot_owners[slot] == incumbent
    assert not sim.flights[incumbent].returning
    assert _clocks(sim) == clocks
    assert not any(event['type'] == 'relay_handoff_completed' for event in sim.events)


def test_assignment_rejects_unknown_retiring_aircraft_energy():
    sim = _fixture((ScenarioTask(0, (170., -12.5, 0.)),))
    for uid, goal in sim.slots.items():
        _airborne(sim, uid, goal, uid)
        sim.slot_owners[uid] = uid
    retiring = 4
    _airborne(sim, retiring, (*sim.flights[retiring].home[:2], 40.), 0, remaining=1000.)
    sim.flights[retiring].slot = None
    sim.state.uavs[retiring].target_position = None
    sim.flights[retiring].remaining_s = float('nan')
    sim._network()
    owners, clocks = dict(sim.slot_owners), _clocks(sim)

    proposal = sim._budgeted_assignment(sim.tasks[0])

    # Four capable owners do not prove the excluded fifth aircraft can wait
    # for deployment and return. Its unknown budget must fail admission.
    assert proposal is None
    assert sim.slot_owners == owners and _clocks(sim) == clocks
    assert not sim.flights[retiring].returning
