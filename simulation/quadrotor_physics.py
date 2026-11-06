"""Headless force-driven quadrotors using Panda3D Bullet, in SI/Z-up units.

This is an uncalibrated rigid-body model, not airframe qualification. Four
massless motors act on an explicit inertia tensor and a spherical collision
envelope. There are no pose or velocity setters after initial body placement.
The velocity controller is local to this backend; it is not a radio model.
"""
from dataclasses import asdict, dataclass
import math
from typing import Dict, Optional, Tuple

import numpy as np
from panda3d.bullet import (BulletBoxShape, BulletPlaneShape, BulletRigidBodyNode,
                            BulletSphereShape, BulletWorld)
from panda3d.core import NodePath, Point3, Vec3


Vector = Tuple[float, float, float]


def _vector(value, name, size=3):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must contain {size} finite numbers') from exc
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result.copy()


@dataclass(frozen=True)
class PhysicsConfig:
    fixed_dt: float = 1. / 120.
    mass_kg: float = 1.5
    arm_length_m: float = .28
    body_radius_m: float = .35
    inertia_kg_m2: Vector = (.025, .025, .045)
    max_motor_thrust_n: float = 8.
    motor_tau_s: float = .035
    max_speed_mps: float = 4.
    max_accel_mps2: float = 3.
    max_tilt_rad: float = .45
    max_yaw_rate_rps: float = 1.5
    command_timeout_s: float = .5
    gravity_mps2: float = 9.81
    linear_drag_n_per_mps: float = .18
    quadratic_drag_n_per_mps2: float = .025
    angular_drag_nms: float = .02
    yaw_torque_per_thrust_m: float = .018
    velocity_kp: Vector = (2., 2., 2.6)
    velocity_ki: Vector = (.15, .15, .35)
    velocity_integral_limit_mps: float = 2.
    attitude_gain: float = 5.
    angular_rate_gain: float = 10.
    max_attitude_rate_rps: float = 4.
    friction: float = .65
    restitution: float = 0.

    def __post_init__(self):
        nonnegative = {'linear_drag_n_per_mps', 'quadratic_drag_n_per_mps2',
                       'angular_drag_nms', 'friction', 'restitution'}
        vectors = {'inertia_kg_m2', 'velocity_kp', 'velocity_ki'}
        for name, value in asdict(self).items():
            if name in vectors:
                vector = _vector(value, name)
                if np.any(vector < 0.) or (name != 'velocity_ki' and np.any(vector <= 0.)):
                    raise ValueError(f'{name} has an invalid component')
                object.__setattr__(self, name, tuple(map(float, vector)))
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value)
                  or (value < 0. if name in nonnegative else value <= 0.)):
                raise ValueError(f'{name} must be finite and {"nonnegative" if name in nonnegative else "positive"}')
        if self.fixed_dt > .02:
            raise ValueError('fixed_dt must be at most .02 seconds for this controller')
        if self.max_tilt_rad >= math.pi / 2.:
            raise ValueError('max_tilt_rad must be below pi/2')
        if self.restitution > 1.:
            raise ValueError('restitution must be at most 1')


@dataclass(frozen=True)
class PhysicsObservation:
    time_s: float
    position: Vector
    velocity: Vector
    quaternion_wxyz: Tuple[float, float, float, float]
    angular_velocity_world: Vector
    rotor_thrusts_n: Tuple[float, float, float, float]
    acceleration: Vector
    ground_contact: bool
    body_contact: bool
    command_age_s: Optional[float]
    failed: bool
    armed: bool
    command_stale: bool


class QuadrotorWorld:
    """Fixed-step rigid bodies, with optional attachment of ``root`` to a viewer.

    Keep root's transform and scale unchanged. ``bodies`` exposes NodePaths for
    observation/visual children, not teleportation. Motor order is X-layout:
    (+x,+y), (-x,+y), (-x,-y), (+x,-y); reaction signs are +,-,+,-.
    """

    def __init__(self, homes: Dict[int, Vector], config=None, wind=(0., 0., 0.)):
        self.config = config if config is not None else PhysicsConfig()
        if not isinstance(self.config, PhysicsConfig):
            raise TypeError('config must be a PhysicsConfig')
        if not isinstance(homes, dict) or not homes:
            raise ValueError('homes must be a nonempty dictionary of integer IDs and COM positions')
        if any(isinstance(uid, bool) or not isinstance(uid, int) for uid in homes):
            raise ValueError('aircraft IDs must be integers')
        positions = {uid: _vector(pos, f'home {uid}') for uid, pos in homes.items()}
        if any(p[2] < self.config.body_radius_m - 1e-8 for p in positions.values()):
            raise ValueError('initial COM must not place its sphere below the ground plane')
        self._wind = _vector(wind, 'wind')
        self.root = NodePath('quadrotor-physics')
        self.world = BulletWorld()
        self.world.setGravity(Vec3(0., 0., -self.config.gravity_mps2))
        self.bodies = {}
        self._static = {}
        ground = BulletRigidBodyNode('ground')
        ground.addShape(BulletPlaneShape(Vec3(0., 0., 1.), 0.))
        ground.setFriction(self.config.friction)
        self._static['ground'] = self.root.attachNewNode(ground)
        self.world.attachRigidBody(ground)
        self._ids = tuple(sorted(homes))
        self._index = {uid: index for index, uid in enumerate(self._ids)}
        self._node_ids = {}
        for uid in self._ids:
            body = BulletRigidBodyNode(f'quadrotor-{uid}')
            body.addShape(BulletSphereShape(self.config.body_radius_m))
            body.setMass(self.config.mass_kg)
            body.setInertia(Vec3(*self.config.inertia_kg_m2))
            body.setLinearDamping(0.)
            body.setAngularDamping(0.)
            body.setFriction(self.config.friction)
            body.setRestitution(self.config.restitution)
            body.setDeactivationEnabled(False)
            body.setCcdMotionThreshold(self.config.body_radius_m * .5)
            body.setCcdSweptSphereRadius(self.config.body_radius_m * .8)
            path = self.root.attachNewNode(body)
            path.setPos(*map(float, positions[uid]))  # Initial spawn only.
            self.world.attachRigidBody(body)
            self.bodies[uid] = path
            self._node_ids[body.getName()] = uid
        n = len(self._ids)
        self._steps = 0
        self.time_s = 0.
        self._closed = False
        self._armed = np.zeros(n, dtype=bool)
        self._failed = np.zeros(n, dtype=bool)
        self._command_times = np.full(n, np.nan)
        self._vel_commands = np.zeros((n, 3))
        self._yaw_commands = np.zeros(n)
        self._yaw_goals = np.zeros(n)
        self._integrals = np.zeros((n, 3))
        self._was_stale = np.zeros(n, dtype=bool)
        self._thrusts = np.zeros((n, 4))
        self._override = np.full((n, 4), np.nan)
        self._acceleration = np.zeros((n, 3))
        self._ground_contact = np.zeros(n, dtype=bool)
        self._body_contact = np.zeros(n, dtype=bool)
        self._contacts = set()
        self._contact_totals = {}
        self._max_speed = self._max_angular_speed = self._max_tilt = 0.
        self._min_com_z = min(float(p[2]) for p in positions.values())
        a = self.config.arm_length_m / math.sqrt(2.)
        self._motor_positions = np.array([(a, a, 0.), (-a, a, 0.),
                                          (-a, -a, 0.), (a, -a, 0.)])
        self._spins = np.array([1., -1., 1., -1.])
        mixer = np.vstack([np.ones(4), self._motor_positions[:, 1],
                           -self._motor_positions[:, 0],
                           self._spins * self.config.yaw_torque_per_thrust_m])
        self._mixer_inverse = np.linalg.inv(mixer)

    def _check_open(self):
        if self._closed:
            raise RuntimeError('physics world is closed')

    def _uid_index(self, uid):
        self._check_open()
        if isinstance(uid, bool) or uid not in self._index:
            raise KeyError(f'unknown aircraft {uid!r}')
        return self._index[uid]

    def command(self, uid, velocity=(0., 0., 0.), armed=True, yaw_rad=0.):
        i = self._uid_index(uid)
        value = _vector(velocity, 'velocity')
        if not isinstance(armed, bool):
            raise ValueError('armed must be a bool')
        if isinstance(yaw_rad, bool) or not isinstance(yaw_rad, (int, float)) or not math.isfinite(yaw_rad):
            raise ValueError('yaw_rad must be finite')
        scale = float(np.max(np.abs(value)))
        norm = scale * float(np.linalg.norm(value / scale)) if scale else 0.
        if norm > self.config.max_speed_mps:
            value = value / scale * (self.config.max_speed_mps / float(np.linalg.norm(value / scale)))
        if not self._armed[i] or not armed:
            self._integrals[i] = 0.
        self._armed[i] = armed and not self._failed[i]
        self._vel_commands[i] = value
        self._yaw_commands[i] = math.remainder(yaw_rad, 2. * math.pi)
        self._command_times[i] = self.time_s
        if not self._armed[i]:
            self._thrusts[i] = 0.
            self._override[i] = np.nan

    def set_wind(self, vector):
        self._check_open()
        self._wind = _vector(vector, 'wind')

    def fail(self, uid):
        i = self._uid_index(uid)
        self._failed[i] = True
        self._armed[i] = False
        self._thrusts[i] = 0.
        self._override[i] = np.nan
        self._integrals[i] = 0.

    def set_motor_override(self, uid, thrusts_or_none):
        i = self._uid_index(uid)
        if thrusts_or_none is None:
            self._override[i] = np.nan
            return
        values = _vector(thrusts_or_none, 'rotor thrusts', 4)
        if np.any(values < 0.) or np.any(values > self.config.max_motor_thrust_n):
            raise ValueError('rotor thrusts must be within the configured motor limits')
        self._override[i] = values

    def add_box(self, name, center, half_extents):
        self._check_open()
        if not isinstance(name, str) or not name or name in self._static or name in self._node_ids:
            raise ValueError('box name must be nonempty and unique')
        center = _vector(center, 'box center')
        extents = _vector(half_extents, 'box half extents')
        if np.any(extents <= 0.):
            raise ValueError('box half extents must be positive')
        body = BulletRigidBodyNode(name)
        body.addShape(BulletBoxShape(Vec3(*extents)))
        body.setFriction(self.config.friction)
        body.setRestitution(self.config.restitution)
        path = self.root.attachNewNode(body)
        path.setPos(*map(float, center))  # Initial static geometry placement only.
        self.world.attachRigidBody(body)
        self._static[name] = path
        return path

    @staticmethod
    def _rotations(quaternions):
        q = quaternions / np.linalg.norm(quaternions, axis=1)[:, None]
        w, x, y, z = q.T
        return np.array([[1.-2.*(y*y+z*z), 2.*(x*y-z*w), 2.*(x*z+y*w)],
                         [2.*(x*y+z*w), 1.-2.*(x*x+z*z), 2.*(y*z-x*w)],
                         [2.*(x*z-y*w), 2.*(y*z+x*w), 1.-2.*(x*x+y*y)]]).transpose(2, 0, 1)

    def _read_state(self):
        positions, velocities, quaternions, angular = [], [], [], []
        for uid in self._ids:
            path = self.bodies[uid]
            body = path.node()
            positions.append(tuple(path.getPos(self.root)))
            velocities.append(tuple(body.getLinearVelocity()))
            q = path.getQuat(self.root)
            quaternions.append((q.getR(), q.getI(), q.getJ(), q.getK()))
            angular.append(tuple(body.getAngularVelocity()))
        return tuple(np.asarray(a, dtype=float) for a in (positions, velocities, quaternions, angular))

    def _desired_thrusts(self, velocity, rotations, angular):
        c, dt = self.config, self.config.fixed_dt
        stale = np.isnan(self._command_times) | (self.time_s - self._command_times > c.command_timeout_s + 1e-12)
        self._integrals[stale & ~self._was_stale] = 0.
        # An expired raw actuation override must not reappear when a later
        # ordinary velocity command refreshes the command timestamp.
        self._override[stale] = np.nan
        self._was_stale = stale
        wanted = self._vel_commands.copy()
        wanted[stale] = 0.
        error = wanted - velocity
        self._integrals += error * dt
        self._integrals = np.clip(self._integrals, -c.velocity_integral_limit_mps,
                                  c.velocity_integral_limit_mps)
        self._integrals[~self._armed] = 0.
        accel = error * c.velocity_kp + self._integrals * c.velocity_ki
        magnitude = np.linalg.norm(accel, axis=1)
        accel *= np.minimum(1., c.max_accel_mps2 / np.maximum(magnitude, 1e-12))[:, None]
        force = accel * c.mass_kg
        force[:, 2] += c.mass_kg * c.gravity_mps2
        # Keep the commanded thrust direction in the upper, bounded tilt cone.
        force[:, 2] = np.maximum(force[:, 2], .05 * c.mass_kg * c.gravity_mps2)
        horizontal = np.linalg.norm(force[:, :2], axis=1)
        force[:, :2] *= np.minimum(1., force[:, 2] * math.tan(c.max_tilt_rad)
                                   / np.maximum(horizontal, 1e-12))[:, None]
        yaw_error = (self._yaw_commands - self._yaw_goals + math.pi) % (2. * math.pi) - math.pi
        self._yaw_goals += np.clip(yaw_error, -c.max_yaw_rate_rps * dt, c.max_yaw_rate_rps * dt)
        z_goal = force / np.linalg.norm(force, axis=1)[:, None]
        x_heading = np.column_stack([np.cos(self._yaw_goals), np.sin(self._yaw_goals), np.zeros(len(force))])
        y_goal = np.cross(z_goal, x_heading)
        y_goal /= np.linalg.norm(y_goal, axis=1)[:, None]
        x_goal = np.cross(y_goal, z_goal)
        desired = np.stack([x_goal, y_goal, z_goal], axis=2)
        relative = np.matmul(rotations.transpose(0, 2, 1), desired)
        attitude_error = .5 * np.column_stack([relative[:, 2, 1] - relative[:, 1, 2],
                                               relative[:, 0, 2] - relative[:, 2, 0],
                                               relative[:, 1, 0] - relative[:, 0, 1]])
        rate = attitude_error * c.attitude_gain
        xy = np.linalg.norm(rate[:, :2], axis=1)
        rate[:, :2] *= np.minimum(1., c.max_attitude_rate_rps / np.maximum(xy, 1e-12))[:, None]
        rate[:, 2] = np.clip(rate[:, 2], -c.max_yaw_rate_rps, c.max_yaw_rate_rps)
        omega = np.einsum('nji,nj->ni', rotations, angular)
        torque = np.asarray(c.inertia_kg_m2) * c.angular_rate_gain * (rate - omega)
        torque += np.cross(omega, omega * c.inertia_kg_m2)
        total = np.maximum(0., np.einsum('ni,ni->n', force, rotations[:, :, 2]))
        collective = np.clip(total / 4., 0., c.max_motor_thrust_n)
        deltas = np.column_stack([np.zeros(len(force)), torque]) @ self._mixer_inverse.T
        # Scale differential authority before saturation to retain collective.
        upper = np.where(deltas > 1e-12, (c.max_motor_thrust_n - collective[:, None])
                         / np.maximum(deltas, 1e-12), np.inf)
        lower = np.where(deltas < -1e-12, collective[:, None] / np.maximum(-deltas, 1e-12), np.inf)
        authority = np.minimum(1., np.minimum(upper.min(axis=1), lower.min(axis=1)))
        thrusts = collective[:, None] + deltas * authority[:, None]
        overridden = np.isfinite(self._override[:, 0]) & ~stale
        thrusts[overridden] = self._override[overridden]
        thrusts[~self._armed | self._failed] = 0.
        return np.clip(thrusts, 0., c.max_motor_thrust_n)

    def _update_contacts(self):
        self._ground_contact[:] = False
        self._body_contact[:] = False
        contacts = set()
        for manifold in self.world.getManifolds():
            points = [p for p in manifold.getManifoldPoints() if p.getDistance() <= 1e-6]
            if not points:
                continue
            names = (manifold.getNode0().getName(), manifold.getNode1().getName())
            if not any(name in self._node_ids for name in names):
                continue
            pair = tuple(sorted(names))
            contacts.add(pair)
            key = '|'.join(pair)
            total = self._contact_totals.setdefault(key, {'begins': 0, 'seconds': 0., 'max_impulse_ns': 0.})
            total['begins'] += int(pair not in self._contacts)
            total['seconds'] += self.config.fixed_dt
            total['max_impulse_ns'] = max(total['max_impulse_ns'], max(float(p.getAppliedImpulse()) for p in points))
            for first, second in (names, names[::-1]):
                if first in self._node_ids:
                    i = self._index[self._node_ids[first]]
                    if second == 'ground':
                        self._ground_contact[i] = True
                    else:
                        self._body_contact[i] = True
        self._contacts = contacts

    def step(self, duration_s):
        self._check_open()
        if (isinstance(duration_s, bool) or not isinstance(duration_s, (int, float))
                or not math.isfinite(duration_s) or duration_s < 0.):
            raise ValueError('duration_s must be finite and nonnegative')
        ratio = duration_s / self.config.fixed_dt
        if not math.isfinite(ratio):
            raise ValueError('duration_s is too large')
        count = round(ratio)
        if not math.isclose(duration_s, count * self.config.fixed_dt, rel_tol=1e-9, abs_tol=1e-12):
            raise ValueError('duration_s must be an integer multiple of fixed_dt')
        c = self.config
        alpha = -math.expm1(-c.fixed_dt / c.motor_tau_s)
        for _ in range(count):
            _, velocities, quaternions, angular = self._read_state()
            rotations = self._rotations(quaternions)
            desired = self._desired_thrusts(velocities, rotations, angular)
            self._thrusts += alpha * (desired - self._thrusts)
            self._thrusts[~self._armed | self._failed] = 0.
            airspeed = velocities - self._wind
            drag = -(c.linear_drag_n_per_mps + c.quadratic_drag_n_per_mps2
                     * np.linalg.norm(airspeed, axis=1))[:, None] * airspeed
            for i, uid in enumerate(self._ids):
                body = self.bodies[uid].node()
                axis = rotations[i, :, 2]
                offsets = self._motor_positions @ rotations[i].T
                for motor in range(4):
                    body.applyForce(Vec3(*map(float, axis * self._thrusts[i, motor])),
                                    Point3(*map(float, offsets[motor])))
                reaction = axis * float(self._thrusts[i] @ self._spins) * c.yaw_torque_per_thrust_m
                body.applyTorque(Vec3(*map(float, reaction - c.angular_drag_nms * angular[i])))
                body.applyCentralForce(Vec3(*map(float, drag[i])))
            # max_substeps=0 performs exactly this explicit fixed interval;
            # Bullet does not accumulate a second, hidden time remainder.
            self.world.doPhysics(c.fixed_dt, 0, c.fixed_dt)
            self._steps += 1
            self.time_s = self._steps * c.fixed_dt
            positions, updated, quaternions, angular = self._read_state()
            self._acceleration = (updated - velocities) / c.fixed_dt
            self._max_speed = max(self._max_speed, float(np.linalg.norm(updated, axis=1).max()))
            self._max_angular_speed = max(self._max_angular_speed, float(np.linalg.norm(angular, axis=1).max()))
            self._max_tilt = max(self._max_tilt, float(np.arccos(np.clip(self._rotations(quaternions)[:, 2, 2], -1., 1.)).max()))
            self._min_com_z = min(self._min_com_z, float(positions[:, 2].min()))
            self._update_contacts()

    def observe(self, uid):
        i = self._uid_index(uid)
        path, body = self.bodies[uid], self.bodies[uid].node()
        q = path.getQuat(self.root)
        age = None if math.isnan(self._command_times[i]) else max(0., self.time_s - float(self._command_times[i]))
        return PhysicsObservation(
            time_s=self.time_s, position=tuple(map(float, path.getPos(self.root))),
            velocity=tuple(map(float, body.getLinearVelocity())),
            quaternion_wxyz=(float(q.getR()), float(q.getI()), float(q.getJ()), float(q.getK())),
            angular_velocity_world=tuple(map(float, body.getAngularVelocity())),
            rotor_thrusts_n=tuple(map(float, self._thrusts[i])),
            acceleration=tuple(map(float, self._acceleration[i])),
            ground_contact=bool(self._ground_contact[i]), body_contact=bool(self._body_contact[i]),
            command_age_s=age, failed=bool(self._failed[i]), armed=bool(self._armed[i]),
            command_stale=age is None or age > self.config.command_timeout_s + 1e-12)

    def parameters(self):
        return {'engine': 'Panda3D Bullet', 'units': 'SI; world Z-up; quaternion wxyz',
                'config': asdict(self.config), 'wind_mps': self._wind.tolist(),
                'motor_positions_body_m': self._motor_positions.tolist(),
                'motor_reaction_signs': self._spins.tolist(),
                'collision_envelope': 'sphere; radius=body_radius_m; infinite ground at z=0',
                'command_limit_scope': 'velocity/tilt/rate request limits; external forces can exceed them',
                'scope': 'uncalibrated 6DOF rigid-body model; not physical-airframe qualification'}

    def contacts_summary(self):
        return {'current_pairs': [list(pair) for pair in sorted(self._contacts)],
                'pairs': {key: dict(value) for key, value in self._contact_totals.items()}}

    def stats(self):
        return {'time_s': self.time_s, 'fixed_steps': self._steps, 'fleet_size': len(self._ids),
                'max_speed_mps': self._max_speed, 'max_angular_speed_rps': self._max_angular_speed,
                'max_tilt_rad': self._max_tilt, 'min_com_z_m': self._min_com_z,
                'failed_uav_ids': [uid for uid, i in self._index.items() if self._failed[i]],
                'contacts': self.contacts_summary()}

    def close(self):
        if self._closed:
            return
        for path in (*self.bodies.values(), *self._static.values()):
            self.world.removeRigidBody(path.node())
        self.root.removeNode()
        self._closed = True
