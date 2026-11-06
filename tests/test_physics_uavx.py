"""Mission adapter contracts: planner references never become measured evidence."""

from dataclasses import asdict, replace
import math
import pickle

import pytest

from core.models import UAVRole, UAVState, Vector3D
from simulation.physics_uavx import PhysicsUAVXSimulator
from simulation.quadrotor_physics import QuadrotorWorld
from simulation.uavx_scenario import FaultSpec, ScenarioConfig, ScenarioTask


def scenario(**changes):
    return ScenarioConfig(fleet_size=30, camera_mode='footprint', packet_loss=0.,
                          tasks=(ScenarioTask(0, (175., 0., 0.)),), **changes)


@pytest.fixture
def simulator_factory():
    simulators = []

    def make(*, airborne=None, faults=()):
        sim = PhysicsUAVXSimulator(scenario(faults=faults))
        simulators.append(sim)
        if airborne:
            # Different initial placement is a fixture; no body is teleported after spawning.
            initial = {uid: (*flight.home[:2], airborne.get(uid, flight.home[2]))
                       for uid, flight in sim.flights.items()}
            config = sim.physics.config
            sim.physics.close()
            sim.physics = QuadrotorWorld(initial, config=config)
            sim._sync_measured()
        return sim

    yield make
    for sim in reversed(simulators):
        sim.close()


def move_once(sim):
    sim._move()
    sim.state.time = round(sim.physics.time_s, 9)


def prepare_return(sim, uid=0, *, armed=False):
    flight, uav = sim.flights[uid], sim.state.uavs[uid]
    flight.armed, flight.landed, flight.returning = armed, False, True
    flight.landing_approach = True
    uav.target_position = Vector3D(*flight.home)
    uav.state, uav.role = UAVState.RETURNING, UAVRole.RELAY
    return flight


@pytest.mark.parametrize('fleet_size', [29, 31])
def test_adapter_rejects_changed_fleet_resources(fleet_size):
    with pytest.raises(ValueError, match='30'):
        PhysicsUAVXSimulator(ScenarioConfig(fleet_size=fleet_size, camera_mode='footprint'))


def test_adapter_spawns_exactly_thirty_bodies_at_physical_home_height(simulator_factory):
    sim = simulator_factory()
    assert set(sim.physics.bodies) == set(sim.flights) == set(range(30))
    assert sim.physics.stats()['fleet_size'] == 30
    for uid, flight in sim.flights.items():
        assert flight.home[2] == sim.physics.config.body_radius_m
        assert sim.physics.observe(uid).position == pytest.approx(flight.home, abs=1e-6)


def test_reference_copy_cannot_grant_motion_charge_or_landing_credit(simulator_factory):
    sim = simulator_factory()
    sim.state.uavs[0].target_position = Vector3D(*sim.flights[0].home[:2], 4.)
    sim.flights[1].charge_started_s = 0.
    sim.flights[1].remaining_s, sim.flights[1].sortie_s = 1199.9, 400.
    sim.state.uavs[1].state = UAVState.CHARGING
    returning = prepare_return(sim, 2)
    returning.settled_s = 1.9
    before_uavs = pickle.dumps(sim.state.uavs)
    before_tasks = pickle.dumps(sim.tasks)
    before_router = pickle.dumps(sim.router)
    before_physics = {uid: asdict(sim.physics.observe(uid)) for uid in sim.flights}
    clocks = {uid: (f.remaining_s, f.sortie_s, f.max_sortie_s, f.charge_started_s, f.observed_at_s)
              for uid, f in sim.flights.items()}

    reference = sim._reference()

    assert reference.shape == (30, 3) and reference[0, 2] > 0.
    assert sim.flights[0].armed and not sim.flights[0].landed  # Motor intention only.
    assert returning.returning and not returning.landed and returning.settled_s == 1.9
    assert clocks == {uid: (f.remaining_s, f.sortie_s, f.max_sortie_s, f.charge_started_s, f.observed_at_s)
                      for uid, f in sim.flights.items()}
    assert pickle.dumps(sim.state.uavs) == before_uavs
    assert pickle.dumps(sim.tasks) == before_tasks
    assert pickle.dumps(sim.router) == before_router
    assert before_physics == {uid: asdict(sim.physics.observe(uid)) for uid in sim.flights}
    assert sim.state.time == sim.physics.time_s == 0.
    assert [(event['type'], event['uav_id']) for event in sim.events] == [('takeoff_started', 0)]
    assert sim.events[0]['evidence'] == 'motor_command'


def test_landing_requires_measured_contact_and_two_seconds_settled(simulator_factory):
    sim = simulator_factory()
    flight = prepare_return(sim)
    flight.remaining_s, flight.sortie_s = 600., 600.
    for _ in range(3):
        move_once(sim)
        assert not flight.landed and flight.charge_started_s is None
        assert not any(event['type'] == 'landing_confirmed' for event in sim.events)
    for _ in range(2):
        if flight.landed:
            break
        move_once(sim)
    landing = next(event for event in sim.events if event['type'] == 'landing_confirmed')
    assert flight.landed and not flight.armed and not flight.returning
    assert landing['time_s'] >= 2.
    assert landing['time_s'] == pytest.approx(landing['observed_at_s'])
    assert landing['ground_contact'] and landing['disarm_acknowledged']
    assert landing['evidence'] == 'bullet_contact_and_telemetry'
    assert math.dist(landing['position'], flight.home) < .15
    assert math.hypot(*landing['velocity']) <= .15
    assert flight.remaining_s == 600.  # Newly landed charge begins on a later interval.


def test_kinematic_landing_prediction_cannot_replace_missing_contact(simulator_factory, monkeypatch):
    sim = simulator_factory()
    flight = prepare_return(sim)
    observe = sim.physics.observe

    def missing_contact(uid):
        observation = observe(uid)
        return replace(observation, ground_contact=False) if uid == 0 else observation

    monkeypatch.setattr(sim.physics, 'observe', missing_contact)
    for _ in range(5):
        move_once(sim)
    assert not flight.landed and flight.returning and flight.charge_started_s is None
    assert flight.settled_s == 0.
    assert not any(event['type'] in ('landing_confirmed', 'recharge_completed')
                   and event.get('uav_id') == 0 for event in sim.events)


def test_midinterval_disarm_preserves_the_armed_endurance_charge(simulator_factory):
    sim = simulator_factory()
    flight = prepare_return(sim, armed=True)
    flight.remaining_s, flight.sortie_s = 600., 600.
    move_once(sim)
    assert not flight.armed
    assert flight.remaining_s == pytest.approx(600. - sim.control_dt)
    assert flight.sortie_s == pytest.approx(600. + sim.control_dt)
    assert not flight.landed


def test_permanent_failure_cuts_power_but_body_keeps_falling(simulator_factory):
    sim = simulator_factory(airborne={0: 8.}, faults=(FaultSpec('vehicle_failure', 0., 0),))
    flight, uav = sim.flights[0], sim.state.uavs[0]
    flight.armed, flight.landed = True, False
    uav.state, uav.role = UAVState.RELAYING, UAVRole.RELAY
    before = sim.physics.observe(0)
    sim._network()
    move_once(sim)
    falling = sim.physics.observe(0)
    assert falling.failed and not falling.armed
    assert falling.position[2] < before.position[2] - .5
    assert falling.velocity[2] < -3.
    assert falling.rotor_thrusts_n == pytest.approx((0.,) * 4)
    assert uav.position.to_array() == pytest.approx(falling.position)
    assert not flight.landed
    for _ in range(6):
        move_once(sim)
    grounded = sim.physics.observe(0)
    assert grounded.ground_contact and grounded.failed
    assert grounded.position[2] == pytest.approx(flight.home[2], abs=.02)
    assert not flight.landed and flight.charge_started_s is None
    assert not uav.is_operational()
    assert not any(event['type'] in ('landing_confirmed', 'recharge_completed')
                   and event.get('uav_id') == 0 for event in sim.events)
    assert not sim.report()['checks']['all_uavs_landed']


def test_report_keeps_past_contacts_and_between_sample_speed_extrema(simulator_factory, monkeypatch):
    sim = simulator_factory()
    contacts = {'current_pairs': [], 'pairs': {
        'ground|quadrotor-0': {'begins': 1, 'seconds': 1., 'max_impulse_ns': .1},
        'quadrotor-0|test-wall': {'begins': 1, 'seconds': 1. / 120., 'max_impulse_ns': .2}}}
    stats = {**sim.physics.stats(), 'max_speed_mps': 5.25, 'contacts': contacts}
    monkeypatch.setattr(sim.physics, 'contacts_summary', lambda: contacts)
    monkeypatch.setattr(sim.physics, 'stats', lambda: stats)
    sim.max_speed = 4.
    report = sim.report()
    assert report['schema_version'] == 'aeromesh-bullet-1'
    assert report['collision_count'] == 1
    assert not report['checks']['no_body_collision']
    assert report['maximum_speed_mps'] == 5.25
    assert not report['checks']['speed'] and not report['sample_constraints_passed']
    assert report['physics']['contacts']['current_pairs'] == []
