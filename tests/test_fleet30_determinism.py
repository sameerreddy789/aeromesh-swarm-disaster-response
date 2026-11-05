"""Subprocess regression for hash-dependent ownership and relay dispatch.

This manually positioned kinematic fixture is a determinism check, not mission
or physical-flight evidence.
"""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def fixture():
    sys.path.insert(0, str(ROOT))
    from core.models import UAVRole, UAVState, Vector3D
    from simulation.resilient_uavx import ResilientUAVXSimulator
    from simulation.uavx_scenario import ScenarioConfig, ScenarioTask

    sim = ResilientUAVXSimulator(ScenarioConfig(
        seed=109, fleet_size=30, charging_pads=30, camera_mode='footprint',
        tasks=(ScenarioTask(0, (400., 0., 0.)),)))
    sim._network()
    proposal = sim._budgeted_assignment(sim.tasks[0])
    assert proposal is not None and len(proposal['owners']) == 10
    sim._commit_formation(sim.tasks[0], proposal)
    owners = list(sim.slot_owners.items())
    for slot, uid in sim.slot_owners.items():
        flight, uav = sim.flights[uid], sim.state.uavs[uid]
        flight.armed, flight.landed = True, False
        flight.remaining_s, flight.sortie_s = 500., 700.
        uav.position = Vector3D(*sim.slots[slot])
        uav.state, uav.role = UAVState.RELAYING, UAVRole.RELAY
    sim.state.time = 100.
    sim._network()
    sim.events.clear()
    sim._plan()
    dispatches = [
        (event['slot'], event['old_uav_id'], event['new_uav_id'])
        for event in sim.events if event['type'] == 'replacement_dispatched'
    ]
    assert len(dispatches) == 10
    handoffs = list(sim.replacements.items())
    for _ in range(40):
        sim._move()
        sim.state.time += sim.dt
    return {
        'hash_seed': os.environ['PYTHONHASHSEED'],
        'source_sha256': {
            name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in ('simulation/uavx.py', 'simulation/resilient_uavx.py')
        },
        'sorted_owner_mapping': sorted(owners),
        'ordered_owners': owners,
        'handoff_dispatch_order': dispatches,
        'ordered_handoffs': handoffs,
        'positions_after_20_s': {
            uid: [round(float(value), 8) for value in uav.position.to_array()]
            for uid, uav in sim.state.uavs.items()
        },
    }



def test_hash_seed_does_not_change_owners_dispatches_or_measured_movement():
    runs = []
    for seed in ('0', '1', '2'):
        env = dict(os.environ, PYTHONHASHSEED=seed, PYTHONDONTWRITEBYTECODE='1')
        result = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve())],
                                cwd=ROOT, env=env, capture_output=True, text=True,
                                check=True, timeout=60)
        run = json.loads(result.stdout)
        run.pop('hash_seed')
        runs.append(run)
    assert runs[0] == runs[1] == runs[2]


if __name__ == '__main__':
    print(json.dumps(fixture()))
