"""Announced task selection must respect feasibility and priority together."""

import pytest
from dataclasses import replace

from core.models import UAVRole, UAVState, Vector3D
from simulation.uavx import SurveyTask
from simulation.resilient_uavx import ResilientUAVXSimulator
from simulation.uavx_scenario import ScenarioConfig, ScenarioTask


def fixture(near_spawn=0., far_priority=1.):
    sim = ResilientUAVXSimulator(ScenarioConfig(
        fleet_size=30, charging_pads=30, camera_mode='footprint', tasks=(
            ScenarioTask(0, (175., 0., 0.)),
            ScenarioTask(1, (170., 0., 0.), priority=far_priority),
            ScenarioTask(2, (75., 0., 0.), spawn_s=near_spawn))))
    sim.state.time = 100.
    sim.active_id = 0
    sim.tasks[0].surveyed = True
    sim.tasks[0].dwell_s = 30.
    sim.tasks[0].report_s = 90.
    for uid in range(2, 30):
        flight = sim.flights[uid]
        flight.charge_started_s = 0.
        flight.remaining_s = 200.
        flight.sortie_s = 1000.
        sim.state.uavs[uid].state = UAVState.CHARGING
    sim._network()
    return sim


def test_infeasible_nearest_task_does_not_starve_feasible_equal_priority_work():
    sim = fixture()
    assert sim._budgeted_assignment(sim.tasks[1]) is None
    assert sim._budgeted_assignment(sim.tasks[2]) is not None
    clocks = {uid: (f.remaining_s, f.sortie_s, f.charge_started_s)
              for uid, f in sim.flights.items()}

    sim._plan()

    assert sim.active_id == 2
    assert set(sim.slot_owners.values()) == {0, 1}
    assert len(sim.slot_owners) == len(sim.slots) == 2
    assert clocks == {uid: (f.remaining_s, f.sortie_s, f.charge_started_s)
                      for uid, f in sim.flights.items()}


@pytest.mark.parametrize('kwargs', [{'near_spawn': 101.}, {'far_priority': 5.}])
def test_admission_does_not_use_future_or_lower_priority_task(kwargs):
    sim = fixture(**kwargs)
    sim._plan()
    assert sim.active_id == 0
    assert not sim.slot_owners
    assert not any(f.armed for f in sim.flights.values())


def test_unfinished_current_task_is_not_preempted_by_equal_priority_alternative():
    sim = fixture()
    sim.active_id = 1
    sim._plan()
    assert sim.active_id == 1
    assert not sim.slot_owners


def active_fixture(remaining=1000.):
    sim = fixture(far_priority=5.)
    sim.active_id = 2
    sim.slots = sim._build_slots(sim.tasks[2])
    for slot, goal in sim.slots.items():
        sim.slot_owners[slot] = slot
        flight, uav = sim.flights[slot], sim.state.uavs[slot]
        flight.armed, flight.landed, flight.slot = True, False, slot
        flight.remaining_s, flight.sortie_s = remaining, 1200. - remaining
        uav.position = uav.target_position = Vector3D(*goal)
        uav.state, uav.role = UAVState.RELAYING, UAVRole.RELAY
    sim._network()
    return sim


def test_pending_larger_urgent_task_does_not_force_healthy_cohort_to_recharge():
    sim = active_fixture()
    for _ in range(40):
        sim._plan()
        assert sim.active_id == 2
        assert sim._withdrawal_reason is None
        assert set(sim.slot_owners.values()) == {0, 1}
        sim._move()
        sim.state.time += sim.dt
    assert all(sim.flights[uid].remaining_s == pytest.approx(980.) for uid in (0, 1))
    assert all(sim.flights[uid].sortie_s == pytest.approx(220.) for uid in (0, 1))
    assert not any(f.returning for f in sim.flights.values())
    assert sim.min_separation >= 22. - 1e-8


def test_pending_task_cannot_hold_cohort_below_shared_reserve():
    sim = active_fixture(remaining=300.)
    sim._plan()
    assert sim._withdrawal_reason == 'formation_admission_failed'
    assert not sim.slot_owners
    assert all(sim.flights[uid].returning for uid in (0, 1))


def test_unknown_airborne_energy_cannot_be_hidden_by_holding_other_owners():
    sim = active_fixture()
    flight = sim.flights[2]
    flight.armed, flight.landed, flight.charge_started_s = True, False, None
    flight.remaining_s = float('nan')
    sim.state.uavs[2].state = UAVState.RELAYING
    sim._plan()
    assert sim._withdrawal_reason == 'formation_admission_failed'
    assert not sim.slot_owners


@pytest.mark.parametrize('future_position', [(75., -500., 0.), (1075., 500., 0.)])
def test_idle_wait_preserves_next_task_headroom_without_using_future_geometry(future_position):
    sim = active_fixture(remaining=350.)
    sim.tasks[1].spawn_s = 500.
    sim.tasks[1].position = future_position
    sim.tasks[2].surveyed, sim.tasks[2].report_s = True, 99.
    sim.tasks[2].dwell_s = 30.
    sim._idle_since_s = 60.
    before = {uid: (f.remaining_s, f.sortie_s) for uid, f in sim.flights.items()}
    sim._plan()
    assert sim._withdrawal_reason == 'waiting_for_announced_tasks'
    assert all(sim.flights[uid].returning for uid in (0, 1))
    assert before == {uid: (f.remaining_s, f.sortie_s) for uid, f in sim.flights.items()}
    assert not any(e['type'] == 'task_selected' for e in sim.events)


@pytest.mark.parametrize('remaining,withdraw', [(350., False), (230., True)])
def test_short_idle_grace_never_overrides_hard_return_reserve(remaining, withdraw):
    sim = active_fixture(remaining=remaining)
    sim.tasks[1].spawn_s = 105.
    sim.tasks[2].surveyed, sim.tasks[2].report_s = True, 99.
    sim.tasks[2].dwell_s = 30.
    sim._idle_since_s = 99.
    sim._plan()
    assert (sim._withdrawal_reason is not None) is withdraw


@pytest.mark.parametrize('charge', [1190., 200.])
def test_completed_cohort_waits_only_for_feasible_measured_charge_progress(charge):
    sim = active_fixture(remaining=600.)
    sim.tasks[2].surveyed, sim.tasks[2].report_s = True, 99.
    sim.tasks[2].dwell_s = 30.
    for uid in (2, 3):
        sim.flights[uid].remaining_s = charge
    clocks = {uid: (f.remaining_s, f.sortie_s, f.charge_started_s)
              for uid, f in sim.flights.items()}
    before_time = sim.state.time
    sim._plan()
    assert clocks == {uid: (f.remaining_s, f.sortie_s, f.charge_started_s)
                      for uid, f in sim.flights.items()}
    assert sim.state.time == before_time
    if charge == 1190.:
        assert sim._withdrawal_reason is None
        assert sim._hold_ready_s == pytest.approx(106.5)
        assert sim.active_id == 2
    else:
        assert sim._withdrawal_reason == 'formation_admission_failed'


def test_charge_forecast_respects_one_shared_pad_without_completing_real_charge():
    sim = active_fixture(remaining=600.)
    sim.scenario = replace(sim.scenario, charging_pads=1)
    sim.tasks[2].surveyed, sim.tasks[2].report_s = True, 99.
    for uid in (2, 3):
        sim.flights[uid].remaining_s = 1190.
    sim._plan()
    assert sim._hold_ready_s == pytest.approx(113.)
    assert sim.state.time == 100.
    assert all(sim.flights[uid].remaining_s == 1190. for uid in (2, 3))
    assert all(sim.flights[uid].charge_started_s == 0. for uid in (2, 3))


@pytest.mark.parametrize('follow_position,can_wait', [((75., 0., 0.), True), ((170., 0., 0.), False)])
def test_charge_forecast_preserves_headroom_for_required_followup_cohort(follow_position, can_wait):
    sim = active_fixture(remaining=500.)
    sim.tasks[2].surveyed, sim.tasks[2].report_s = True, 99.
    sim.tasks.append(SurveyTask(3, follow_position, 0., 1.))
    for uid in (2, 3):
        sim.flights[uid].remaining_s = 1190.
    sim._plan()
    assert (sim._withdrawal_reason is None) is can_wait
    assert all(sim.flights[uid].remaining_s == 1190. for uid in (2, 3))
    assert sim.active_id == 2


@pytest.mark.parametrize('change', ['announcement', 'failed_charger'])
def test_positive_charge_forecast_is_invalidated_by_new_work_or_aircraft_loss(change):
    sim = active_fixture(remaining=600.)
    sim.tasks[2].surveyed, sim.tasks[2].report_s = True, 99.
    sim.tasks.append(SurveyTask(3, (170., 0., 0.), 102., 1.))
    for uid in (2, 3):
        sim.flights[uid].remaining_s = 1190.
    sim._plan()
    assert sim._hold_ready_s == 106.5
    if change == 'announcement':
        sim.state.time = 103.
        sim._plan()
        assert sum(e['type'] == 'formation_waiting_for_charge' for e in sim.events) == 2
    else:
        sim.state.uavs[2].state = UAVState.FAILED
        sim._plan()
        assert sim._withdrawal_reason == 'formation_admission_failed'


@pytest.mark.parametrize('pads', [1, 2, 30])
@pytest.mark.parametrize('energy', [1190., 1199., 1198.333333333])
def test_charge_forecast_matches_observed_assignment_readiness(pads, energy):
    sim = active_fixture(remaining=600.)
    sim.scenario = replace(sim.scenario, charging_pads=pads)
    sim.tasks[2].surveyed, sim.tasks[2].report_s = True, 99.
    for uid in (2, 3):
        sim.flights[uid].remaining_s = energy
    expected = sim._forecast_charge_admission(sim.tasks[1])
    assert expected is not None
    assert sim._budgeted_assignment(sim.tasks[1]) is None
    for _ in range(30):
        sim._move()
        sim.state.time += sim.dt
        if sim._budgeted_assignment(sim.tasks[1]) is not None:
            break
    assert sim.state.time == expected


def test_charge_forecast_bounds_extremely_slow_charger_at_mission_horizon():
    sim = active_fixture(remaining=600.)
    sim.recharge_s = 1e100
    assert sim._forecast_charge_admission(sim.tasks[1]) is None


@pytest.mark.parametrize('energy,sortie,returning', [
    (215., 985., False), (600., 985., False), (215., 985., True)])
def test_hold_checks_replacement_energy_and_sortie_without_overriding_its_return(energy, sortie, returning):
    sim = active_fixture(remaining=600.)
    flight, uav = sim.flights[2], sim.state.uavs[2]
    flight.armed, flight.landed, flight.charge_started_s, flight.slot = True, False, None, 0
    flight.remaining_s, flight.sortie_s, flight.returning = energy, sortie, returning
    sim.replacements[0] = 2
    uav.position = Vector3D(0., -50., 65.)
    uav.target_position = Vector3D(*sim.slots[0][:2], 65.)
    uav.state, uav.role = UAVState.RELAYING, UAVRole.RELAY
    sim._network()
    sim._plan()
    if returning:
        assert sim._withdrawal_reason is None
    else:
        assert sim._withdrawal_reason == 'formation_admission_failed'
        assert all(sim.flights[uid].returning for uid in (0, 1, 2))
    assert flight.remaining_s == energy and flight.sortie_s == sortie
