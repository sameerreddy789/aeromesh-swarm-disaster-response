"""Pure safety predicates for the supplied UAV-X sample scenario.

These helpers do not command vehicles or certify an airframe. Time values are
seconds on one monotonic/simulation clock; coordinates are Cartesian metres.
The energy admission check follows Garuda's target-plus-return budget pattern,
with explicit survey, relay, landing and challenge deadline allowances.
"""

from dataclasses import dataclass, fields
import math
from numbers import Real
from typing import Sequence, Tuple


def _finite_nonnegative(name: str, value: float, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result) or result < 0 or (positive and result == 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return result


def _vector3(name: str, value: Sequence[float]) -> Tuple[float, float, float]:
    try:
        if len(value) != 3:
            raise ValueError(f"{name} must have three components")
        result = tuple(float(v) for v in value)
        if any(isinstance(v, bool) or not isinstance(v, Real) for v in value):
            raise ValueError(f"{name} must contain finite numbers")
        if not all(math.isfinite(v) for v in result):
            raise ValueError(f"{name} must contain finite numbers")
        return result
    except (TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must contain three finite numbers") from exc


@dataclass(frozen=True)
class SampleConstraints:
    """Defaults transcribed from the supplied one-slide mission constraints."""

    area_width_m: float = 1000.0
    area_height_m: float = 1000.0
    mission_duration_s: float = 2700.0
    max_flight_s: float = 1200.0
    radio_range_m: float = 100.0
    max_altitude_m: float = 100.0
    max_speed_mps: float = 5.0
    min_separation_m: float = 20.0
    report_deadline_s: float = 10.0
    poi_count: int = 10

    def __post_init__(self) -> None:
        for item in fields(self):
            _finite_nonnegative(item.name, getattr(self, item.name), positive=True)
        if isinstance(self.poi_count, bool) or not isinstance(self.poi_count, int):
            raise ValueError("poi_count must be a positive integer")


@dataclass(frozen=True)
class EnergyBudget:
    """Current remaining-flight estimate, capped by the 20-minute flight limit.

    ``remaining_flight_s`` is the current energy-derived estimate, not the
    original battery endurance. ``can_assign`` additionally subtracts elapsed
    airborne time from the challenge's per-flight hard cap.
    """

    remaining_flight_s: float = 1200.0

    def __post_init__(self) -> None:
        estimate = _finite_nonnegative("remaining_flight_s", self.remaining_flight_s)
        object.__setattr__(self, "remaining_flight_s", min(1200.0, estimate))

    def can_assign(
        self,
        travel_distance_m: float,
        survey_s: float,
        relay_wait_s: float,
        home_distance_m: float,
        speed_mps: float,
        elapsed_airborne_s: float,
        mission_time_s: float,
        deadline_s: float = 2700.0,
        reserve_s: float = 60.0,
        landing_s: float = 10.0,
    ) -> bool:
        """Reject invalid inputs; admit only work that can return with reserve.

        ``home_distance_m`` must describe the return route from the proposed
        task endpoint, including detours. Relay replacement time belongs in
        ``relay_wait_s``. No feasibility is inferred from straight-line distance.
        """
        values = {
            "travel_distance_m": travel_distance_m, "survey_s": survey_s,
            "relay_wait_s": relay_wait_s, "home_distance_m": home_distance_m,
            "elapsed_airborne_s": elapsed_airborne_s, "mission_time_s": mission_time_s,
            "deadline_s": deadline_s, "reserve_s": reserve_s, "landing_s": landing_s,
        }
        checked = {name: _finite_nonnegative(name, value) for name, value in values.items()}
        speed = _finite_nonnegative("speed_mps", speed_mps, positive=True)
        if speed > 5.0:
            return False
        required = (
            checked["travel_distance_m"] / speed
            + checked["home_distance_m"] / speed
            + checked["survey_s"] + checked["relay_wait_s"]
            + checked["reserve_s"] + checked["landing_s"]
        )
        available = min(self.remaining_flight_s, max(0.0, 1200.0 - checked["elapsed_airborne_s"]))
        return (
            math.isfinite(required)
            and required <= available
            and checked["mission_time_s"] + required <= min(2700.0, checked["deadline_s"])
        )


def measured_landing_complete(
    position: Sequence[float],
    velocity: Sequence[float],
    home_pad: Sequence[float],
    observed_at_s: float,
    now_s: float,
    armed: bool,
    settled_duration_s: float,
    required_settled_duration_s: float = 2.0,
) -> bool:
    """Require fresh, disarmed, settled telemetry at this UAV's own home pad.

    The caller must accumulate ``settled_duration_s`` only across consecutive
    fresh observations satisfying these pose/speed limits, resetting it on any
    violation or missing reading. An acknowledgement alone is insufficient.
    Invalid, future-dated or stale evidence returns False; nothing is mutated.
    """
    try:
        pos = _vector3("position", position)
        vel = _vector3("velocity", velocity)
        pad = _vector3("home_pad", home_pad)
        observed = _finite_nonnegative("observed_at_s", observed_at_s)
        now = _finite_nonnegative("now_s", now_s)
        settled = _finite_nonnegative("settled_duration_s", settled_duration_s)
        required = _finite_nonnegative("required_settled_duration_s", required_settled_duration_s)
    except (ValueError, TypeError, OverflowError):
        return False
    return (
        armed is False
        and required >= 2.0
        and settled >= required
        and 0.0 <= now - observed <= 1.0
        and math.hypot(pos[0] - pad[0], pos[1] - pad[1]) <= 1.0
        and abs(pos[2] - pad[2]) <= 0.25
        and math.hypot(*vel) <= 0.15
    )


def swept_min_separation(
    start_a: Sequence[float],
    end_a: Sequence[float],
    start_b: Sequence[float],
    end_b: Sequence[float],
) -> float:
    """Exact closest distance of two simultaneous linear segments.

    All four positions refer to the same start/end times. This measures the
    continuous interval, including both endpoints, so crossing paths cannot
    pass merely because their sampled endpoint distances are large.
    """
    a0, a1 = _vector3("start_a", start_a), _vector3("end_a", end_a)
    b0, b1 = _vector3("start_b", start_b), _vector3("end_b", end_b)
    relative = tuple(a - b for a, b in zip(a0, b0))
    change = tuple((end - start) - (other_end - other_start)
                   for start, end, other_start, other_end in zip(a0, a1, b0, b1))
    if not all(math.isfinite(v) for v in relative + change):
        raise ValueError("segment arithmetic exceeds finite coordinate range")
    # Scale avoids squaring very large finite coordinates during minimization.
    scale = max((abs(v) for v in relative + change), default=0.0)
    if scale == 0.0:
        return 0.0
    r = tuple(v / scale for v in relative)
    d = tuple(v / scale for v in change)
    denominator = sum(v * v for v in d)
    t = max(0.0, min(1.0, -sum(x * y for x, y in zip(r, d)) / denominator)) if denominator else 0.0
    distance = math.hypot(*(x + t * y for x, y in zip(r, d))) * scale
    if not math.isfinite(distance):
        raise ValueError("segment distance exceeds finite coordinate range")
    return distance
