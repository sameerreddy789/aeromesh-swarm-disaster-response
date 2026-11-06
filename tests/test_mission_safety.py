"""Challenge safety checks exercise decision boundaries and missing evidence."""

from dataclasses import FrozenInstanceError
import math
import pytest

from core.mission_safety import (
    EnergyBudget, SampleConstraints, measured_landing_complete, swept_min_separation,
)


def assignment(**overrides):
    values = dict(travel_distance_m=100.0, survey_s=30.0, relay_wait_s=20.0,
                  home_distance_m=100.0, speed_mps=5.0,
                  elapsed_airborne_s=0.0, mission_time_s=0.0)
    values.update(overrides)
    return values


def landing(**overrides):
    values = dict(position=(10, 20, 0), velocity=(0, 0, 0), home_pad=(10, 20, 0),
                  observed_at_s=10.0, now_s=10.5, armed=False, settled_duration_s=2.0)
    values.update(overrides)
    return measured_landing_complete(**values)


def test_energy_includes_dwell_reserve_and_landing():
    # 40 travel + 30 survey + 20 relay + 60 reserve + 10 landing = 160 seconds.
    assert EnergyBudget(160).can_assign(**assignment())
    assert not EnergyBudget(159).can_assign(**assignment())
    assert not EnergyBudget(160).can_assign(**assignment(relay_wait_s=21))


def test_energy_hard_cap_and_deadline_apply_to_each_admission():
    budget = EnergyBudget(99999)
    assert budget.remaining_flight_s == 1200
    assert budget.can_assign(**assignment(elapsed_airborne_s=1040))
    assert not budget.can_assign(**assignment(elapsed_airborne_s=1041))
    assert budget.can_assign(**assignment(mission_time_s=2540))
    assert not budget.can_assign(**assignment(mission_time_s=2541))
    assert not budget.can_assign(**assignment(mission_time_s=2541, deadline_s=99999))
    assert not budget.can_assign(**assignment(speed_mps=5.01))


@pytest.mark.parametrize("invalid", [-1, math.nan, math.inf, -math.inf, True, "20"])
def test_energy_rejects_invalid_estimates(invalid):
    with pytest.raises(ValueError):
        EnergyBudget(invalid)


@pytest.mark.parametrize("field", list(assignment()) + ["deadline_s", "reserve_s", "landing_s"])
@pytest.mark.parametrize("invalid", [-1, math.nan, math.inf])
def test_energy_rejects_invalid_assignment_values(field, invalid):
    with pytest.raises(ValueError):
        EnergyBudget().can_assign(**assignment(**{field: invalid}))


def test_zero_speed_is_rejected_and_zero_budget_cannot_assign():
    with pytest.raises(ValueError):
        EnergyBudget().can_assign(**assignment(speed_mps=0))
    assert not EnergyBudget(0).can_assign(**assignment())


def test_landing_requires_own_pad_fresh_settled_disarmed_measurement():
    assert landing()
    assert landing(position=(11, 20, 0.25), velocity=(0.15, 0, 0), now_s=11)
    assert not landing(position=(0, 0, 0))
    assert not landing(position=(10, 20, 10))  # RTB hover is not landing.
    assert not landing(velocity=(0, 0, 0.151))
    assert not landing(now_s=11.01)
    assert not landing(observed_at_s=11)
    assert not landing(armed=True)
    assert not landing(armed=0)
    assert not landing(settled_duration_s=1.99)
    assert not landing(required_settled_duration_s=1)
    assert not landing(required_settled_duration_s=3)


@pytest.mark.parametrize("overrides", [
    {"position": (math.nan, 0, 0)}, {"velocity": (0, math.inf, 0)},
    {"home_pad": (0, 0)}, {"now_s": math.nan}, {"settled_duration_s": math.inf},
    {"required_settled_duration_s": math.nan}, {"observed_at_s": -1},
])
def test_invalid_landing_evidence_fails_closed(overrides):
    assert not landing(**overrides)


def test_swept_crossing_detects_collision_despite_safe_endpoints():
    assert swept_min_separation((-30, 0, 0), (30, 0, 0),
                                (0, -30, 0), (0, 30, 0)) == pytest.approx(0)


def test_swept_same_velocity_stationary_and_endpoint_minima():
    assert swept_min_separation((0, 0, 0), (10, 0, 0),
                                (0, 20, 0), (10, 20, 0)) == pytest.approx(20)
    assert swept_min_separation((0, 0, 0), (0, 0, 0),
                                (3, 4, 0), (3, 4, 0)) == pytest.approx(5)
    assert swept_min_separation((0, 0, 0), (10, 0, 0),
                                (30, 0, 0), (20, 0, 0)) == pytest.approx(10)
    assert swept_min_separation((0, 0, 0), (-10, 0, 0),
                                (5, 0, 0), (10, 0, 0)) == pytest.approx(5)


def test_swept_3d_clearance_and_invalid_coordinates():
    assert swept_min_separation((-30, 0, 0), (30, 0, 0),
                                (0, -30, 20), (0, 30, 20)) == pytest.approx(20)
    with pytest.raises(ValueError):
        swept_min_separation((math.nan, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))


def test_sample_constraints_are_frozen_and_match_document():
    constraints = SampleConstraints()
    assert (constraints.area_width_m, constraints.area_height_m) == (1000, 1000)
    assert constraints.mission_duration_s == 2700
    assert constraints.max_flight_s == 1200
    assert constraints.radio_range_m == constraints.max_altitude_m == 100
    assert constraints.min_separation_m == 20
    assert constraints.report_deadline_s == constraints.poi_count == 10
    assert constraints.max_speed_mps == 5
    with pytest.raises(FrozenInstanceError):
        constraints.max_speed_mps = 10


@pytest.mark.parametrize("overrides", [
    {"radio_range_m": math.inf}, {"max_flight_s": math.nan},
    {"min_separation_m": -1}, {"max_speed_mps": 0}, {"poi_count": 10.5},
    {"poi_count": True},
])
def test_sample_constraints_validate_values(overrides):
    with pytest.raises(ValueError):
        SampleConstraints(**overrides)
