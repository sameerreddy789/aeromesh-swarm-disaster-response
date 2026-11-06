"""Bullet flight adapter for the existing UAV-X mission planner.

The planner supplies references only. Bullet owns all actual positions,
velocities and contacts. The original kinematic evidence schema is not used
to qualify this backend. Distances/network availability are sampled at 20 Hz.
"""
from copy import copy, deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
from importlib.metadata import version

import numpy as np

from core.models import Vector3D, UAVRole, UAVState
from core.mission_safety import EnergyBudget
from simulation.resilient_uavx import ResilientUAVXSimulator
from simulation.uavx import UAVXSimulator
from simulation.physics_planning import PhysicsBudgetMixin
from simulation.quadrotor_physics import QuadrotorWorld, PhysicsConfig
from simulation.camera_sensor import DisasterCamera


class PhysicsCamera(DisasterCamera):
    EVIDENCE_SCOPE = ('Panda3D TinyDisplay RGB procedural disaster scene; '
                      'position from Bullet rigid-body telemetry; ideal world-down '
                      'stabilized camera, independent of airframe attitude; uncalibrated')


class PhysicsUAVXSimulator(PhysicsBudgetMixin, ResilientUAVXSimulator):
    schema_version = 'aeromesh-bullet-1'
    control_dt = .05
    command_speed_mps = 4.

    def __init__(self, scenario, physics_config=None, wind=(0., 0., 0.)):
        if scenario.fleet_size != 30:
            raise ValueError('The physics mission uses exactly 30 UAVs including spares')
        super().__init__(scenario)
        config = physics_config or PhysicsConfig()
        for uid, flight in self.flights.items():
            flight.home = (*flight.home[:2], config.body_radius_m)
            self.state.uavs[uid].position = Vector3D(*flight.home)
        self.physics = QuadrotorWorld({i:f.home for i,f in self.flights.items()}, config, wind)
        self.physics_samples = []
        self.safety_interventions = 0
        self._physics_failed = set()
        self._physics_stream = None
        self._next_physics_sample = 0.
        self._last_physics_sample = None
        self._physics_observer = None
        self._sync_measured()
        root = Path(__file__).resolve().parents[1]
        for name in ('simulation/physics_uavx.py', 'simulation/quadrotor_physics.py',
                     'simulation/physics_planning.py', 'run_physics.py'):
            self.source_sha256[name] = hashlib.sha256((root/name).read_bytes()).hexdigest()

    def _ensure_camera(self):
        if self._camera is None:
            self._camera = PhysicsCamera(self.tasks, seed=self.seed,
                width=self.scenario.camera_width, height=self.scenario.camera_height)
            self._scene_metadata = self._camera.get_scene_metadata()
        return self._camera

    def _prepare_output(self, output):
        super()._prepare_output(output)
        self._physics_stream = (output/'physics-telemetry.jsonl').open('w', encoding='utf-8')

    def run(self, output, max_time=2700, observer=None):
        # ResilientUAVXSimulator.run closes its backend immediately. A native
        # viewer still needs the body nodes after finish(), until the caller
        # explicitly closes this simulator.
        self._physics_observer = observer
        try:
            return UAVXSimulator.run(self, output, max_time, observer)
        finally:
            self._physics_observer = None

    def _withdrawal_path_s(self, positions=None):
        estimate = super()._withdrawal_path_s(positions)
        return estimate*5./self.command_speed_mps+12. if estimate else 0.

    def _eligible(self, uid, goal, dwell=30):
        flight, uav = self.flights[uid], self.state.uavs[uid]
        if not uav.is_operational() or flight.returning or flight.charge_started_s is not None:
            return False
        return EnergyBudget(flight.remaining_s).can_assign(
            math.dist(uav.position.to_array(),goal), dwell, 40., math.dist(goal,flight.home),
            self.command_speed_mps, flight.sortie_s, self.state.time, reserve_s=60., landing_s=25.)

    def _sync_measured(self):
        observations = {i:self.physics.observe(i) for i in self.flights}
        for uid, obs in observations.items():
            self.state.uavs[uid].position = Vector3D(*obs.position)
            self.state.uavs[uid].velocity = Vector3D(*obs.velocity)
            self.flights[uid].observed_at_s = obs.time_s
        return observations

    def _reference(self):
        """Run motion admission on an isolated copy; never credit its flight.

        Only its intended displacement and takeoff/approach intention leave
        this function. Its charge, position, landing and mission events do not.
        """
        ref = copy(self)
        ref.state = copy(self.state)
        ref.state.uavs = deepcopy(self.state.uavs)
        ref.flights = deepcopy(self.flights)
        ref.events, ref.samples = [], []
        ref.collision_pairs = set(self.collision_pairs)
        ref._landing_yields = dict(self._landing_yields)
        ref._charge_admission = set(self._charge_admission)
        ref.slot_owners = dict(self.slot_owners)
        ref.replacements = dict(self.replacements)
        ref._transit_speed = lambda uid: self.command_speed_mps
        ResilientUAVXSimulator._move(ref)
        velocity = np.zeros((len(self.flights), 3))
        for uid, uav in ref.state.uavs.items():
            flight = self.flights[uid]
            if ref.flights[uid].armed and not flight.armed and not flight.returning:
                flight.armed, flight.landed, flight.settled_s = True, False, 0.
                self.event('takeoff_started', uav_id=uid, evidence='motor_command')
            flight.landing_approach = ref.flights[uid].landing_approach
            velocity[uid] = (uav.position.to_array()-self.state.uavs[uid].position.to_array())/self.dt
        return velocity

    @staticmethod
    def _limit(vector, maximum):
        norm = np.linalg.norm(vector, axis=-1, keepdims=True)
        return vector*np.minimum(1., maximum/np.maximum(norm, 1e-12))

    def _safe_commands(self, reference, observations):
        """Braking margin via velocity constraints; never clip actual state.

        This local controller is not a mathematical continuous-time safety
        guarantee. Its results must be checked from measured telemetry.
        """
        positions = np.array([observations[i].position for i in self.flights])
        velocity = reference.copy()
        movable = np.array([f.armed and self.state.uavs[i].is_operational()
                            for i,f in self.flights.items()])
        velocity[~movable] = 0.
        for uid, flight in self.flights.items():
            if flight.returning and flight.landing_approach and movable[uid]:
                delta = np.array(flight.home)-positions[uid]
                if np.linalg.norm(delta[:2]) < 2. and positions[uid,2] < 3.:
                    velocity[uid,:2] = delta[:2]*.9
                    velocity[uid,2] = -min(.5, max(.15, positions[uid,2]-flight.home[2]))
        # Sequential projections account for both participants. Grounded
        # aircraft are fixed obstacles, never commanded to move aside.
        deltas = positions[:,None,:]-positions[None,:,:]
        distances = np.linalg.norm(deltas, axis=2)
        pairs = list(zip(*np.where(np.tril((distances<34.) & (distances>1e-9), -1))))
        for _ in range(3):
            for first, second in pairs:
                distance = distances[first,second]
                direction = deltas[first,second]/distance
                radial = np.dot(velocity[first]-velocity[second], direction)
                lower = -.7*(distance-22.)
                if radial < lower:
                    count = int(movable[first])+int(movable[second])
                    if count:
                        correction = (lower-radial)/count*direction
                        if movable[first]: velocity[first] += correction
                        if movable[second]: velocity[second] -= correction
        low, high = np.array([-74.,-499.,0.]), np.array([1074.,499.,99.])
        velocity = np.maximum(velocity, -.7*(positions-low))
        velocity = np.minimum(velocity, .7*(high-positions))
        velocity[~movable] = 0.
        velocity = self._limit(velocity, self.command_speed_mps)
        self.safety_interventions += int(np.max(np.abs(velocity-reference))>.01)
        return velocity

    def _move(self):
        observations = self._sync_measured()
        for uid, uav in self.state.uavs.items():
            if not uav.is_operational() and uid not in self._physics_failed:
                self.physics.fail(uid)
                self._physics_failed.add(uid)
                self.flights[uid].armed = False
                self.flights[uid].landed = False
                self.event('physics_power_cut', uav_id=uid, position=list(observations[uid].position))
        waiting = sorted((f.charge_started_s, i) for i,f in self.flights.items() if self._charge_eligible(i))
        self._charge_admission = {i for _,i in waiting[:self.scenario.charging_pads or len(self.flights)]}
        reference = self._reference()
        self.interval_active = {i for i,f in self.flights.items() if f.armed and self.state.uavs[i].is_operational()}
        self.interval_connected, self.interval_downtime = len(self.interval_active), 0.
        for _ in range(round(self.dt/self.control_dt)):
            commands = self._safe_commands(reference, observations)
            consuming = [i for i,f in self.flights.items() if f.armed and self.state.uavs[i].is_operational()]
            for uid, flight in self.flights.items():
                self.physics.command(uid, tuple(commands[uid]), armed=flight.armed)
            self.physics.step(self.control_dt)
            observations = self._sync_measured()
            for uid in consuming:
                flight, uav = self.flights[uid], self.state.uavs[uid]
                flight.sortie_s += self.control_dt
                flight.remaining_s = max(0.,flight.remaining_s-self.control_dt)
                flight.max_sortie_s = max(flight.max_sortie_s, flight.sortie_s)
                if flight.remaining_s <= 0.:
                    self.exhaustions += 1
                    uav.state, uav.role = UAVState.FAILED, UAVRole.FAILED
                    self.physics.fail(uid)
                    self._physics_failed.add(uid)
                    flight.armed, flight.landed = False, False
                    self.event('battery_exhausted', event_time_s=self.physics.time_s, uav_id=uid)
            positions = np.array([observations[i].position for i in self.flights])
            velocities = np.array([observations[i].velocity for i in self.flights])
            distances = np.linalg.norm(positions[:,None,:]-positions[None,:,:], axis=2)
            np.fill_diagonal(distances, np.inf)
            minimum = float(np.min(distances))
            speed = float(np.max(np.linalg.norm(velocities, axis=1)))
            self.min_separation = min(self.min_separation, minimum)
            self.max_speed = max(self.max_speed, speed)
            self.max_altitude = max(self.max_altitude, float(np.max(positions[:,2])))
            self.separation_violations += int(minimum < 20.-1e-7)
            self.geofence_violations += sum(not self._inside(p) for p in positions)
            connected = len(self.interval_active & self._reachable_nodes(positions))
            self.interval_connected = min(self.interval_connected, connected)
            self.interval_downtime += self.control_dt if connected<len(self.interval_active) else 0.
            sample = dict(time_s=round(self.physics.time_s,9), minimum_separation_m=minimum,
                          maximum_speed_mps=speed, connected=connected, active=len(self.interval_active))
            self.physics_samples.append(sample)
            for uid, flight in self.flights.items():
                obs = observations[uid]
                at_home = (math.dist(obs.position[:2], flight.home[:2]) <= .75
                           and abs(obs.position[2]-flight.home[2]) < .15
                           and np.linalg.norm(obs.velocity) <= .15 and obs.ground_contact)
                if flight.returning and at_home and self.state.uavs[uid].is_operational():
                    self.physics.command(uid, (0.,0.,0.), armed=False)
                    flight.armed = False
                    flight.settled_s += self.control_dt
                else:
                    flight.settled_s = 0.
            tick = getattr(self._physics_observer, 'physics_tick', None)
            if tick is not None:
                tick(self)
        end = round(self.state.time+self.dt,9)
        for uid, flight in self.flights.items():
            uav, obs = self.state.uavs[uid], observations[uid]
            if flight.returning and uav.is_operational():
                if flight.settled_s >= 2. and not flight.landed and not obs.armed:
                    flight.landed, flight.returning, flight.charge_started_s = True, False, end
                    uav.target_position = None
                    uav.state, uav.role = UAVState.CHARGING, UAVRole.CHARGING
                    self.event('landing_confirmed', event_time_s=end, uav_id=uid,
                        position=list(obs.position), velocity=list(obs.velocity), sortie_s=flight.sortie_s,
                        observed_at_s=obs.time_s, disarm_acknowledged=True, ground_contact=True,
                        evidence='bullet_contact_and_telemetry')
            if self._charge_eligible(uid) and self._charging_allowed(uid):
                flight.remaining_s = self._charge_after_step(flight.remaining_s)
                if flight.remaining_s >= 1200.:
                    flight.sortie_s, flight.charge_started_s = 0., None
                    uav.state, uav.role = UAVState.IDLE, UAVRole.STANDBY
                    self.event('recharge_completed', event_time_s=end, uav_id=uid)
            uav.battery_level = flight.remaining_s/1200.
        if self._physics_stream and end >= self._next_physics_sample:
            self._physics_stream.write(json.dumps(dict(time_s=end,
                uavs=[dict(id=i, **asdict(self.physics.observe(i))) for i in self.flights]), allow_nan=False)+'\n')
            self._next_physics_sample = end+1.
            self._last_physics_sample = end

    def report(self):
        report = super().report()
        report.update(schema_version=self.schema_version,
            evidence_scope='Bullet rigid-body flight with four motors; uncalibrated simulated vehicle; sampled mission safety',
            physics=dict(parameters=self.physics.parameters(), stats=self.physics.stats(),
                         engine_version=version('panda3d'),
                         contacts=self.physics.contacts_summary(), control_dt_s=self.control_dt,
                         mission_safety_sampling_hz=1/self.control_dt,
                         safety_interventions=self.safety_interventions,
                         energy_model='Declared 1200 s endurance and scenario recharge; not physical battery calibration',
                         camera_model='Ideal world-down stabilized RGB at measured COM position'),
            qualification='Development physics run; not the original kinematic qualification or hardware evidence')
        root = Path(__file__).resolve().parents[1]
        report['source_stable'] = all((root/name).exists() and hashlib.sha256((root/name).read_bytes()).hexdigest()==digest
                                    for name,digest in self.source_sha256.items())
        body_contacts = [pair for pair in self.physics.contacts_summary()['pairs'] if 'ground' not in pair.split('|')]
        report['collision_count'] = len(body_contacts)
        report['maximum_speed_mps'] = self.physics.stats()['max_speed_mps']
        report['checks']['speed'] = report['maximum_speed_mps'] <= 5.+1e-8
        report['checks']['no_body_collision'] = not body_contacts
        report['sample_constraints_passed'] = all(report['checks'].values())
        return report

    def _write_extra_evidence(self, output):
        if self._physics_stream:
            if self._last_physics_sample != self.state.time:
                self._physics_stream.write(json.dumps(dict(time_s=self.state.time,
                    uavs=[dict(id=i, **asdict(self.physics.observe(i))) for i in self.flights]), allow_nan=False)+'\n')
            self._physics_stream.close()
            self._physics_stream = None
        paths = super()._write_extra_evidence(output)
        (output/'physics-safety.json').write_text(json.dumps(self.physics_samples, separators=(',',':')), encoding='utf-8')
        return [*paths, 'physics-telemetry.jsonl', 'physics-safety.json']

    def close(self):
        if self._physics_stream:
            self._physics_stream.close()
        super().close()
        self.physics.close()
