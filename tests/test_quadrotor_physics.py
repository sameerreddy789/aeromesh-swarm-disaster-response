"""Independent force, motion, contact, and authority checks for the Bullet backend.

These qualify the declared rigid-body model, not an aircraft or a flight controller
on physical hardware. Expected ballistic motion and force/torque signs come from
Newtonian mechanics; contact and feedback checks use observed outcomes.
"""

from dataclasses import replace
import math

import pytest

from simulation.quadrotor_physics import PhysicsConfig, QuadrotorWorld


NO_DRAG = dict(linear_drag_n_per_mps=0., quadratic_drag_n_per_mps2=0., angular_drag_nms=0.)


@pytest.fixture
def world_factory():
    worlds = []

    def make(homes=None, *, wind=(0., 0., 0.), **config_changes):
        config = replace(PhysicsConfig(), **config_changes)
        if homes is None:
            homes = {0: (0., 0., config.body_radius_m)}
        world = QuadrotorWorld(homes, config=config, wind=wind)
        worlds.append(world)
        return world

    yield make
    for world in reversed(worlds):
        world.close()


def drive(world, duration, commands, *, sample_dt=.05):
    """Refresh commands while advancing actual physics; retain measured samples."""
    count = round(duration / sample_dt)
    assert count * sample_dt == pytest.approx(duration)
    samples = []
    for _ in range(count):
        for uid, velocity in commands.items():
            world.command(uid, velocity=velocity, armed=True, yaw_rad=0.)
        world.step(sample_dt)
        samples.append({uid: world.observe(uid) for uid in commands})
    return samples


def test_unpowered_airborne_body_follows_analytic_freefall(world_factory):
    world = world_factory({0: (0., 0., 10.)}, **NO_DRAG)
    duration = .25
    world.step(duration)
    observation = world.observe(0)

    # A finite-step integrator may accumulate O(g * dt * duration) position error.
    assert observation.position == pytest.approx((0., 0., 10. - .5 * 9.81 * duration**2), abs=.025)
    assert observation.velocity == pytest.approx((0., 0., -9.81 * duration), abs=.03)
    assert observation.acceleration == pytest.approx((0., 0., -9.81), abs=.04)
    assert not observation.ground_contact and not observation.body_contact
    assert observation.rotor_thrusts_n == pytest.approx((0.,) * 4)


def test_rotor_lag_is_bounded_and_thrust_changes_measured_acceleration(world_factory):
    world = world_factory({0: (0., 0., 10.)}, mass_kg=1.5, motor_tau_s=.2, **NO_DRAG)
    world.command(0, armed=True)
    world.set_motor_override(0, (8.,) * 4)
    world.step(.025)
    first = world.observe(0)
    world.step(.15)
    second = world.observe(0)

    assert all(0. < thrust < 2. for thrust in first.rotor_thrusts_n)
    assert all(a < b < 8. for a, b in zip(first.rotor_thrusts_n, second.rotor_thrusts_n))
    world.command(0, armed=True)
    world.step(.25)
    powered = world.observe(0)
    assert powered.acceleration[2] > 0.
    assert powered.acceleration[2] == pytest.approx(sum(powered.rotor_thrusts_n) / 1.5 - 9.81, abs=.15)

    world.set_motor_override(0, (0.,) * 4)
    world.step(.025)
    decreasing = world.observe(0)
    assert all(0. < b < a for a, b in zip(powered.rotor_thrusts_n, decreasing.rotor_thrusts_n))


@pytest.mark.parametrize('rotor,roll_sign,pitch_sign', [(0, 1., -1.), (2, -1., 1.)])
def test_asymmetric_rotor_rotates_about_the_physical_lever_arm(world_factory, rotor, roll_sign, pitch_sign):
    world = world_factory({0: (0., 0., 20.)}, motor_tau_s=.02, **NO_DRAG)
    thrusts = [0.] * 4
    thrusts[rotor] = 8.
    world.command(0, armed=True)
    world.set_motor_override(0, thrusts)
    world.step(.05)
    observation = world.observe(0)

    # Upward force at (+x,+y) produces +roll and -pitch; the opposite arm reverses both.
    assert roll_sign * observation.angular_velocity_world[0] > .05
    assert pitch_sign * observation.angular_velocity_world[1] > .05
    assert math.dist(observation.quaternion_wxyz, (1., 0., 0., 0.)) > .001


def test_ground_contact_settles_a_falling_body_without_position_reset(world_factory):
    world = world_factory({0: (0., 0., 2.)})
    observations = []
    for _ in range(60):
        world.step(.05)
        observations.append(world.observe(0))

    assert 1.9 < observations[0].position[2] < 2.
    assert sum(.5 < observation.position[2] < 1.8 for observation in observations) >= 4
    assert min(observation.position[2] for observation in observations) >= .32
    assert observations[-1].position[2] == pytest.approx(.35, abs=.015)
    assert abs(observations[-1].velocity[2]) < .03
    assert observations[-1].ground_contact
    assert max(o.position[2] for o in observations[-10:]) - min(o.position[2] for o in observations[-10:]) < .01


def test_velocity_feedback_takes_off_holds_and_brakes(world_factory):
    world = world_factory()
    ascent = drive(world, 2., {0: (0., 0., 1.5)})
    assert ascent[-1][0].position[2] > 1.5
    assert ascent[-1][0].velocity[2] > .5
    assert not ascent[-1][0].ground_contact

    hover = drive(world, 3., {0: (0., 0., 0.)})
    assert abs(hover[-1][0].velocity[2]) < .2
    assert abs(hover[-1][0].position[2] - hover[-11][0].position[2]) < .12

    travel = drive(world, 3., {0: (1.5, 0., 0.)})
    assert travel[-1][0].velocity[0] > 1.
    assert travel[-1][0].position[0] - hover[-1][0].position[0] > 2.
    braking = drive(world, 3., {0: (0., 0., 0.)})
    assert math.hypot(*braking[-1][0].velocity) < .25
    assert math.dist(braking[-1][0].position, braking[-11][0].position) < .15
    assert braking[-1][0].position[2] > 1.


def test_wind_applies_aerodynamic_force_and_calm_air_damps_drift(world_factory):
    world = world_factory({0: (0., 0., 10.)}, linear_drag_n_per_mps=1., quadratic_drag_n_per_mps2=0.)
    world.set_wind((4., 0., 0.))
    world.step(.5)
    windy = world.observe(0)
    assert windy.position[0] > .1
    assert windy.velocity[0] > .5

    world.set_wind((0., 0., 0.))
    world.step(.5)
    calm = world.observe(0)
    assert 0. < calm.velocity[0] < windy.velocity[0]
    assert calm.acceleration[0] < 0.
    assert not calm.ground_contact


def test_hover_feedback_resists_crosswind(world_factory):
    world = world_factory({0: (0., 0., 3.)}, wind=(3., -2., 0.))
    samples = drive(world, 4., {0: (0., 0., 0.)})
    final = samples[-1][0]
    assert math.hypot(*final.velocity) < .4
    assert math.hypot(*final.position[:2]) < 1.2
    assert 2. < final.position[2] < 4.
    assert not final.ground_contact and not final.body_contact


def test_power_cut_restores_freefall_and_cannot_be_rearmed(world_factory):
    world = world_factory({0: (0., 0., 10.)}, **NO_DRAG)
    before = drive(world, 2., {0: (0., 0., 0.)})[-1][0]
    world.fail(0)
    world.command(0, velocity=(0., 0., 3.), armed=True)
    world.set_motor_override(0, (8.,) * 4)
    duration = .25
    world.step(duration)
    failed = world.observe(0)

    assert failed.failed and not failed.armed
    assert failed.rotor_thrusts_n == pytest.approx((0.,) * 4, abs=1e-10)
    assert failed.velocity[2] == pytest.approx(before.velocity[2] - 9.81 * duration, abs=.03)
    assert failed.position[2] == pytest.approx(before.position[2] + before.velocity[2] * duration - .5 * 9.81 * duration**2, abs=.025)
    assert failed.position[2] < before.position[2]


def test_expired_command_brakes_with_feedback_instead_of_freezing_pose(world_factory):
    world = world_factory({0: (0., 0., 3.)})
    moving = drive(world, 1.5, {0: (1.5, 0., 0.)})[-1][0]
    assert moving.velocity[0] > .8
    world.step(.5)
    stale = world.observe(0)
    assert stale.command_stale and stale.command_age_s >= .5
    assert stale.position[0] > moving.position[0]
    assert stale.velocity[0] > .2

    world.step(3.)
    held = world.observe(0)
    assert held.command_stale and held.armed and not held.failed
    assert math.hypot(*held.velocity) < .25
    assert held.position[2] > 1.
    assert sum(held.rotor_thrusts_n) > 10.


@pytest.mark.parametrize('authority_loss', ['disarm', 'expiry'])
def test_fresh_command_does_not_restore_an_old_motor_override(world_factory, authority_loss):
    world = world_factory({0: (0., 0., 20.)})
    world.command(0, armed=True)
    world.set_motor_override(0, (8.,) * 4)
    world.step(.05)
    if authority_loss == 'disarm':
        world.command(0, armed=False)
        world.step(.05)
    else:
        world.step(.6)
        assert world.observe(0).command_stale

    world.command(0, velocity=(0., 0., 0.), armed=True)
    world.step(.25)
    observation = world.observe(0)
    assert observation.armed and not observation.failed and not observation.command_stale
    # A fresh hover/braking request cannot silently reinstate the old 32 N override.
    assert sum(observation.rotor_thrusts_n) < 25.


def test_static_box_blocks_motion_and_produces_measured_contact(world_factory):
    world = world_factory({0: (0., 0., 2.), 1: (0., 8., 2.)})
    world.add_box('wall', center=(2., 0., 3.), half_extents=(.15, 2., 3.))
    samples = drive(world, 4., {0: (2., 0., 0.), 1: (2., 0., 0.)})

    assert any(sample[0].body_contact for sample in samples)
    assert max(sample[0].position[0] for sample in samples) < 1.6
    assert samples[-1][1].position[0] > 4.
    assert all(math.isfinite(value) for sample in samples for value in (*sample[0].position, *sample[0].velocity))


def test_thirty_ids_are_finite_and_repeatable(world_factory):
    ids = [2 * index for index in range(30)]
    homes = {uid: ((index % 6) * 2., (index // 6) * 2., .35) for index, uid in enumerate(ids)}
    first, second = world_factory(homes), world_factory(homes)
    commands = {uid: (0., 0., .3 + .05 * (index % 3)) for index, uid in enumerate(ids)}
    drive(first, 1., commands)
    drive(second, 1., commands)

    for uid in ids:
        left, right = first.observe(uid), second.observe(uid)
        for field in ('position', 'velocity', 'quaternion_wxyz', 'angular_velocity_world', 'rotor_thrusts_n', 'acceleration'):
            values = getattr(left, field)
            assert all(math.isfinite(value) for value in values)
            assert values == pytest.approx(getattr(right, field), abs=1e-6)
        assert math.hypot(*left.quaternion_wxyz) == pytest.approx(1., abs=1e-5)
        assert left.position[2] >= .32
        assert all(0. <= thrust <= 8. for thrust in left.rotor_thrusts_n)


def test_step_rejects_fractional_model_intervals_without_advancing(world_factory):
    world = world_factory()
    before = world.observe(0)
    with pytest.raises(ValueError):
        world.step(PhysicsConfig().fixed_dt / 2.)
    assert world.observe(0) == before
