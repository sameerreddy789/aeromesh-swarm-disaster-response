"""Strict, reproducible development scenarios for the UAV-X profile.

These inputs declare team assumptions, not organizer-approved resources or fault
scoring. A vehicle_failure is permanent; it never exempts a vehicle from the
original return/landing checks. Dynamic selectors must be resolved once by the
runner at injection and the resolved IDs recorded in the event evidence.
"""
from dataclasses import dataclass, fields
import json
import math
from numbers import Real
from pathlib import Path
from typing import Optional, Tuple, Union


FaultTarget = Union[int, Tuple[int, int], str]
FAULT_KINDS = frozenset({'link_outage', 'radio_outage', 'vehicle_failure'})


def _number(name, value, *, minimum=0., maximum=None, exclusive_maximum=False,
            positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f'{name} must be a finite number')
    try:
        value = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if (not math.isfinite(value) or value < minimum or (positive and value <= 0)
            or (maximum is not None and
                (value >= maximum if exclusive_maximum else value > maximum))):
        raise ValueError(f'{name} is outside its permitted range')
    return value


def _integer(name, value, *, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'{name} must be an integer')
    if ((minimum is not None and value < minimum) or
            (maximum is not None and value > maximum)):
        raise ValueError(f'{name} is outside its permitted range')
    return value


def _known_object(data, cls, name):
    if not isinstance(data, dict):
        raise ValueError(f'{name} must be a JSON object')
    unknown = set(data) - {item.name for item in fields(cls)}
    if unknown:
        raise ValueError(f'{name} has unknown fields: {sorted(unknown, key=str)}')
    return data


@dataclass(frozen=True)
class ScenarioTask:
    id: int
    position: Tuple[float, float, float]
    spawn_s: float = 0.
    priority: float = 1.

    def __post_init__(self):
        _integer('task.id', self.id, minimum=0)
        if not isinstance(self.position, (list, tuple)) or len(self.position) != 3:
            raise ValueError('task.position must contain exactly x, y, z')
        x, y, z = self.position
        position = (_number('task.position.x', x, minimum=75., maximum=1075.),
                    _number('task.position.y', y, minimum=-500., maximum=500.),
                    _number('task.position.z', z, minimum=0., maximum=0.))
        object.__setattr__(self, 'position', position)
        object.__setattr__(self, 'spawn_s', _number('task.spawn_s', self.spawn_s, maximum=2700.))
        object.__setattr__(self, 'priority', _number('task.priority', self.priority, positive=True))

    @classmethod
    def from_dict(cls, data):
        data = _known_object(data, cls, 'task')
        if not {'id', 'position'} <= data.keys():
            raise ValueError('task requires id and position')
        return cls(**data)

    def to_dict(self):
        return {'id': self.id, 'position': list(self.position),
                'spawn_s': self.spawn_s, 'priority': self.priority}


@dataclass(frozen=True)
class FaultSpec:
    kind: str
    time_s: float
    target: FaultTarget
    duration_s: Optional[float] = None

    def __post_init__(self):
        if not isinstance(self.kind, str) or self.kind not in FAULT_KINDS:
            raise ValueError(f'fault.kind must be one of {sorted(FAULT_KINDS)}')
        object.__setattr__(self, 'time_s', _number('fault.time_s', self.time_s,
                                                maximum=2700., exclusive_maximum=True))
        if self.kind == 'vehicle_failure':
            if self.duration_s is not None:
                raise ValueError('vehicle_failure is permanent and cannot have duration_s')
        else:
            object.__setattr__(self, 'duration_s',
                               _number('fault.duration_s', self.duration_s, positive=True))
        if self.kind == 'link_outage':
            if self.target == 'middle_active_link':
                return
            if not isinstance(self.target, (list, tuple)) or len(self.target) != 2:
                raise ValueError('link_outage target must be [node_a, node_b] or middle_active_link')
            pair = tuple(sorted(_integer('fault link node', node, minimum=-1)
                                for node in self.target))
            if pair[0] == pair[1]:
                raise ValueError('link_outage requires two distinct nodes')
            object.__setattr__(self, 'target', pair)
        elif self.target != 'middle_active_relay':
            _integer('fault target UAV', self.target, minimum=0)

    @classmethod
    def from_dict(cls, data):
        data = _known_object(data, cls, 'fault')
        if not {'kind', 'time_s', 'target'} <= data.keys():
            raise ValueError('fault requires kind, time_s and target')
        return cls(**data)

    def to_dict(self):
        result = {'kind': self.kind, 'time_s': self.time_s,
                  'target': list(self.target) if isinstance(self.target, tuple) else self.target}
        if self.duration_s is not None:
            result['duration_s'] = self.duration_s
        return result


@dataclass(frozen=True)
class ScenarioConfig:
    schema_version: int = 1
    seed: int = 42
    fleet_size: int = 30
    packet_loss: float = .02
    relay_protection: bool = True
    radio_bytes_per_s: float = 131072.
    radio_queue_bytes: int = 1048576
    recharge_s: float = 720.
    charging_pads: Optional[int] = None
    staging_columns: int = 6
    pad_spacing_m: float = 25.
    camera_mode: str = 'panda3d'
    camera_width: int = 128
    camera_height: int = 128
    tasks: Optional[Tuple[ScenarioTask, ...]] = None
    faults: Tuple[FaultSpec, ...] = ()

    def __post_init__(self):
        _integer('schema_version', self.schema_version, minimum=1, maximum=1)
        _integer('seed', self.seed)
        _integer('fleet_size', self.fleet_size, minimum=1, maximum=72)
        object.__setattr__(self, 'packet_loss',
                           _number('packet_loss', self.packet_loss, maximum=1., exclusive_maximum=True))
        if not isinstance(self.relay_protection, bool):
            raise ValueError('relay_protection must be a boolean')
        object.__setattr__(self, 'radio_bytes_per_s',
                           _number('radio_bytes_per_s', self.radio_bytes_per_s, positive=True))
        _integer('radio_queue_bytes', self.radio_queue_bytes, minimum=1)
        object.__setattr__(self, 'recharge_s', _number('recharge_s', self.recharge_s, positive=True))
        if self.charging_pads is not None:
            _integer('charging_pads', self.charging_pads, minimum=1, maximum=self.fleet_size)
        _integer('staging_columns', self.staging_columns, minimum=1, maximum=6)
        object.__setattr__(self, 'pad_spacing_m',
                           _number('pad_spacing_m', self.pad_spacing_m, minimum=20.))
        rows = math.ceil(self.fleet_size / self.staging_columns)
        if ((self.staging_columns - 1) * self.pad_spacing_m / 2 > 75. or
                (rows - 1) * self.pad_spacing_m / 2 > 500.):
            raise ValueError('staging layout exceeds x=[-75,75], y=[-500,500]')
        if not isinstance(self.camera_mode, str) or self.camera_mode not in ('panda3d', 'footprint'):
            raise ValueError('camera_mode must be panda3d or footprint')
        _integer('camera_width', self.camera_width, minimum=32, maximum=512)
        _integer('camera_height', self.camera_height, minimum=32, maximum=512)
        if self.tasks is not None:
            if not isinstance(self.tasks, (tuple, list)) or not self.tasks:
                raise ValueError('tasks must be a nonempty array, or null for seeded generation')
            if not all(isinstance(task, ScenarioTask) for task in self.tasks):
                raise ValueError('tasks must contain ScenarioTask values')
            object.__setattr__(self, 'tasks', tuple(self.tasks))
            if len({task.id for task in self.tasks}) != len(self.tasks):
                raise ValueError('task IDs must be unique')
        if not isinstance(self.faults, (tuple, list)) or not all(isinstance(fault, FaultSpec) for fault in self.faults):
            raise ValueError('faults must be an array of FaultSpec values')
        object.__setattr__(self, 'faults', tuple(self.faults))
        self._validate_fault_schedule()

    def _validate_fault_schedule(self):
        previous_time = -1.
        occupied = {}
        seen = set()
        for fault in self.faults:
            if fault.time_s < previous_time:
                raise ValueError('fault schedule must be ordered by time_s; equal times retain input order')
            previous_time = fault.time_s
            if fault in seen:
                raise ValueError('duplicate fault entries are not permitted')
            seen.add(fault)
            targets = fault.target if isinstance(fault.target, tuple) else (fault.target,)
            for node in targets:
                if isinstance(node, int) and node >= self.fleet_size:
                    raise ValueError(f'fault target {node} is outside the configured fleet')
            resource = ('link' if fault.kind == 'link_outage' else 'uav', fault.target)
            end = math.inf if fault.kind == 'vehicle_failure' else fault.time_s + fault.duration_s
            if not math.isfinite(end) and fault.kind != 'vehicle_failure':
                raise ValueError('fault end time must remain finite')
            # A permanent dynamic failure can select a different operational
            # relay later. Only resolved explicit nodes identify that resource.
            if isinstance(fault.target, str) and fault.kind == 'vehicle_failure':
                continue
            if resource in occupied and fault.time_s < occupied[resource]:
                raise ValueError('duplicate or overlapping faults on the same target are ambiguous')
            occupied[resource] = end

    @classmethod
    def from_dict(cls, data):
        data = dict(_known_object(data, cls, 'scenario'))
        if data.get('tasks') is not None:
            if not isinstance(data['tasks'], list):
                raise ValueError('tasks must be a JSON array')
            data['tasks'] = tuple(ScenarioTask.from_dict(item) for item in data['tasks'])
        if 'faults' in data:
            if not isinstance(data['faults'], list):
                raise ValueError('faults must be a JSON array')
            data['faults'] = tuple(FaultSpec.from_dict(item) for item in data['faults'])
        return cls(**data)

    def to_dict(self):
        return {
            'schema_version': self.schema_version,
            'seed': self.seed, 'fleet_size': self.fleet_size, 'packet_loss': self.packet_loss,
            'relay_protection': self.relay_protection,
            'radio_bytes_per_s': self.radio_bytes_per_s, 'radio_queue_bytes': self.radio_queue_bytes,
            'recharge_s': self.recharge_s, 'charging_pads': self.charging_pads,
            'staging_columns': self.staging_columns, 'pad_spacing_m': self.pad_spacing_m,
            'camera_mode': self.camera_mode, 'camera_width': self.camera_width,
            'camera_height': self.camera_height,
            'tasks': None if self.tasks is None else [task.to_dict() for task in self.tasks],
            'faults': [fault.to_dict() for fault in self.faults],
        }


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f'nonfinite JSON constant is not permitted: {value}')


def load_scenario(path):
    """Read UTF-8 JSON without duplicate keys, coercions or silently ignored keys."""
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8-sig'),
                          object_pairs_hook=_reject_duplicate_keys,
                          parse_constant=_reject_constant)
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise ValueError(f'invalid scenario JSON: {exc}') from exc
    return ScenarioConfig.from_dict(data)
