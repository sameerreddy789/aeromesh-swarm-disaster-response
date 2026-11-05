"""UAV-X sample profile: deterministic kinematic mission with measured evidence.

This is an abstract PoC, not PX4/ProjectAirSim flight qualification. The profile
uses a centralized planner and a range-limited packet network. Scenario truth
spawns synthetic PoIs; observations occur only when a UAV reaches the sensor
footprint. Every reported result keeps that evidence scope explicit.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional
import json
import math
import random
import hashlib
import platform
import numpy as np
import networkx as nx

from core.models import MissionState, UAV, UAVRole, UAVState, Vector3D, Packet, PacketType
from core.mission_safety import SampleConstraints, EnergyBudget, measured_landing_complete
from comm.mesh_router import MeshRouter
from core.config import RFConfig


@dataclass
class FlightRecord:
    home: tuple
    armed: bool = False
    sortie_s: float = 0.0
    remaining_s: float = 1200.0
    returning: bool = False
    landing_approach: bool = False
    settled_s: float = 0.0
    landed: bool = True
    observed_at_s: float = 0.0
    max_sortie_s: float = 0.0
    slot: Optional[int] = None
    charge_started_s: Optional[float] = None


@dataclass
class SurveyTask:
    id: int
    position: tuple
    spawn_s: float
    priority: float
    detection_s: Optional[float] = None
    detected_by: Optional[int] = None
    report_s: Optional[float] = None
    surveyed: bool = False
    dwell_s: float = 0.0
    last_report_attempt_s: float = -100.0


class RangeChannel:
    """Declared ideal range link with independent per-attempt packet loss."""
    def __init__(self, loss):
        self.loss = loss

    def calculate_link_quality(self, a, b, interference_event=None):
        return dict(is_connected=True, pdr=1-self.loss, etx=1/(1-self.loss))


def pair_distances(starts, ends):
    """Continuous-time separation for simultaneous linear motion, vectorized."""
    relative = starts[:, None, :] - starts[None, :, :]
    motion = ends - starts
    change = motion[:, None, :] - motion[None, :, :]
    denom = np.sum(change * change, axis=2)
    t = np.clip(-np.sum(relative * change, axis=2) / np.maximum(denom, 1e-20), 0, 1)
    distances = np.linalg.norm(relative + change * t[:, :, None], axis=2)
    np.fill_diagonal(distances, np.inf)
    return distances


class UAVXSimulator:
    def __init__(self, seed=42, fleet_size=30, dt=0.5, packet_loss=0.02, tasks=None,
                 disturbance='none'):
        self.constraints = SampleConstraints()
        root=Path(__file__).resolve().parents[1]
        source_paths=['simulation/uavx.py','core/mission_safety.py','core/models.py',
                      'core/config.py','comm/mesh_router.py','comm/channel.py',
                      'comm/channel_legacy.py','run_simulation.py']
        self.source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in source_paths}
        if not isinstance(fleet_size, int) or not 1 <= fleet_size <= 36:
            raise ValueError('fleet_size must be between 1 and 36 for the declared home-pad layout')
        if not math.isfinite(dt) or not 0 < dt <= 0.5:
            raise ValueError('dt must be positive and at most 0.5 seconds')
        if not math.isfinite(packet_loss) or not 0 <= packet_loss < 1:
            raise ValueError('packet_loss must be in [0,1)')
        if disturbance not in ('none', 'outage', 'failure'):
            raise ValueError('unknown disturbance')
        self.dt, self.seed, self.packet_loss = dt, seed, packet_loss
        self.rng = random.Random(seed)
        self.state = MissionState()
        self.flights = {}
        for uid in range(fleet_size):
            columns = 5 if fleet_size <= 20 else 6
            rows = math.ceil(fleet_size/columns)
            home = ((uid % columns - (columns-1)/2) * 25.,
                    (uid // columns - (rows-1)/2) * 25., 0.)
            self.state.uavs[uid] = UAV(uid, position=Vector3D(*home), connected_to_gcs=False)
            self.flights[uid] = FlightRecord(home)
        self.router = MeshRouter(self.state, max_range_m=100., rng=random.Random(seed+1), hop_delay_s=dt)
        self.router.cfg_rf = RFConfig(max_hops=32)
        self.router.rf_channel = RangeChannel(packet_loss)
        self.tasks = tasks if tasks is not None else [SurveyTask(
            i, (self.rng.uniform(75, 1075), self.rng.uniform(-500, 500), 0.),
            0. if i == 0 else self.rng.uniform(60, 1800), 5. if i == 9 else 1.) for i in range(10)]
        self._validate_tasks()
        self.events, self.samples = [], []
        self.active_id = None
        self.slots = {}
        self.slot_owners = {}
        self.replacements = {}
        self.packet_cursor = 0
        self.next_telemetry_s = 0.
        self.min_separation = float('inf')
        self.max_speed = self.max_altitude = self.max_sortie = 0.
        self.geofence_violations = self.separation_violations = self.exhaustions = 0
        self.collision_pairs=set()
        self.disturbance = disturbance
        self.disturbance_record = None
        self.completion_s = None
        self.shutdown = False

    def _validate_tasks(self):
        if not self.tasks or len({p.id for p in self.tasks}) != len(self.tasks):
            raise ValueError('tasks need unique IDs and cannot be empty')
        for p in self.tasks:
            x,y,z = p.position
            if (not all(math.isfinite(v) for v in (x,y,z,p.spawn_s,p.priority))
                    or not (75 <= x <= 1075 and -500 <= y <= 500 and z == 0)
                    or not 0 <= p.spawn_s <= 2700 or p.priority <= 0):
                raise ValueError('invalid PoI geometry, spawn time or priority')

    def event(self, kind, event_time_s=None, **data):
        self.events.append(dict(type=kind, time_s=self.state.time if event_time_s is None else event_time_s, **data))

    def _eligible(self, uid, goal, dwell=30):
        f, u = self.flights[uid], self.state.uavs[uid]
        if not u.is_operational() or f.returning or f.charge_started_s is not None:
            return False
        distance = (u.position - Vector3D(*goal)).norm()
        home_distance = math.dist(goal, f.home)
        return EnergyBudget(f.remaining_s).can_assign(
            distance, dwell, 30., home_distance, 5., f.sortie_s, self.state.time,
            reserve_s=60., landing_s=15.)

    def _release_slot(self, uid):
        f=self.flights[uid]
        slot=f.slot
        if slot is None:
            return
        if self.replacements.get(slot)==uid:
            self.replacements.pop(slot)
        if self.slot_owners.get(slot)==uid:
            self.slot_owners.pop(slot)
            replacement=self.replacements.pop(slot,None)
            if replacement is not None:
                rf=self.flights[replacement]
                if slot in self.slots and self.state.uavs[replacement].is_operational() and not rf.returning:
                    self.slot_owners[slot]=replacement
                    self.event('replacement_promoted_in_transit',uav_id=replacement,slot=slot)
                else:
                    rf.slot=None
                    self._return(replacement, "replacement_slot_removed")
        f.slot=None

    def _return(self, uid, reason):
        f, u = self.flights[uid], self.state.uavs[uid]
        self._release_slot(uid)
        if f.returning or not u.is_operational():
            return
        if f.landed:
            u.target_position=None
            return
        f.returning = True
        f.landing_approach = False
        u.state = UAVState.RETURNING
        u.target_position = Vector3D(*f.home)
        self.event('return_started', uav_id=uid, reason=reason)

    def _plan(self):
        now = self.state.time
        for slot,uid in list(self.replacements.items()):
            if not self.state.uavs[uid].is_operational() or self.flights[uid].returning:
                self._release_slot(uid)
        for uid,f in self.flights.items():
            u=self.state.uavs[uid]
            reserve_return=(math.hypot(u.position.x-f.home[0],u.position.y-f.home[1])+180)/5+180
            if f.armed and not f.returning and f.remaining_s<=reserve_return:
                self._return(uid,'conservative_return_budget')
        remaining = [p for p in self.tasks if not (p.surveyed and p.report_s is not None)]
        longest_return = max(math.dist(tuple(u.position.to_array()), self.flights[i].home)/5
                             for i,u in self.state.uavs.items())
        if not remaining or now + longest_return + 120 >= 2700:
            self.shutdown = True
            for uid in self.flights:
                self._return(uid, 'mission_return')
            return
        available = [p for p in remaining if p.spawn_s <= now]
        if not available:
            for uid in self.flights:
                self._return(uid, "waiting_for_future_tasks")
            self.active_id=None
            self.slots.clear();self.slot_owners.clear();self.replacements.clear()
            return
        current = next((p for p in available if p.id == self.active_id), None)
        highest = max(available, key=lambda p: (p.priority, -p.spawn_s, -p.id))
        task = highest if current is None or highest.priority > current.priority else current
        if task.id != self.active_id:
            self.active_id = task.id
            count = math.ceil(math.hypot(*task.position[:2], 40.)/85.)
            self.slots = {i: (task.position[0]*(i+1)/count, task.position[1]*(i+1)/count, 40.) for i in range(count)}
            # Keep slot identity while the chain pivots. Reassigning everyone by
            # nearest distance makes vehicles cross and can break the backbone.
            for slot,uid in list(self.slot_owners.items()):
                if slot not in self.slots:
                    self._return(uid,'chain_shortened')
            for slot,uid in list(self.replacements.items()):
                self._return(uid,'handoff_replanned')
            self.replacements.clear()
            self.event('task_selected', poi_id=task.id, priority=task.priority, required_slots=count)
        # A partition can retain local neighbors while losing every GCS path.
        graph = self.router.network_graph
        gcs_connected = nx.node_connected_component(graph, -1) if -1 in graph else set()
        # First retain safe current owners; release or replace low-energy relays.
        for slot,uid in list(self.slot_owners.items()):
            f,u = self.flights[uid],self.state.uavs[uid]
            return_time = (math.hypot(u.position.x-f.home[0],u.position.y-f.home[1])+180)/5 + 120
            if not u.is_operational() or f.returning:
                self._release_slot(uid)
                continue
            gcs_path_lost = uid not in gcs_connected
            if f.remaining_s < return_time + 360 or gcs_path_lost:
                if slot not in self.replacements:
                    candidates = [i for i in self.flights if self.flights[i].slot is None
                                  and self._eligible(i, self.slots[slot])]
                    if candidates:
                        new = min(candidates, key=lambda i: self.state.uavs[i].distance_to(Vector3D(*self.slots[slot])))
                        self.replacements[slot] = new
                        self.flights[new].slot = slot
                        self.state.uavs[new].target_position = Vector3D(*self.slots[slot][:2],65.)
                        self.event('replacement_dispatched', old_uav_id=uid, new_uav_id=new, slot=slot,
                                   reason='gcs_path_lost' if gcs_path_lost else 'low_energy')
                new = self.replacements.get(slot)
                if new is not None and self.state.uavs[new].is_operational() and self.flights[new].armed and not self.flights[new].returning and self.flights[new].slot == slot and self.state.uavs[new].distance_to(Vector3D(*self.slots[slot][:2],65.)) <= 3 and self.state.uavs[new].connected_to_gcs:
                    self._return(uid, 'measured_relay_handoff')
                    self.slot_owners[slot] = new
                    self.replacements.pop(slot,None)
                    self.event('relay_handoff_completed', old_uav_id=uid, new_uav_id=new, slot=slot)
                elif f.remaining_s <= return_time:
                    self._return(uid, 'reserve_requires_return')
        for slot,goal in self.slots.items():
            if slot not in self.slot_owners:
                candidates = [i for i in self.flights if self.flights[i].slot is None and self._eligible(i,goal)]
                if not candidates:
                    continue
                uid = min(candidates, key=lambda i: self.state.uavs[i].distance_to(Vector3D(*goal)))
                self.slot_owners[slot] = uid
                self.flights[uid].slot = slot
                self.event('role_assigned', uav_id=uid, role='scout' if slot==max(self.slots) else 'relay', slot=slot)
            uid = self.slot_owners[slot]
            u = self.state.uavs[uid]
            u.target_position = Vector3D(*goal)
            u.role = UAVRole.SCOUT if slot == max(self.slots) else UAVRole.RELAY
        for uid,f in self.flights.items():
            if f.armed and f.slot is None and not f.returning:
                self._return(uid,'idle_energy_conservation')

    @staticmethod
    def _inside(pos):
        # Declared flight geofence includes the home and approach area.
        return -75 <= pos[0] <= 1075 and -500 <= pos[1] <= 500 and 0 <= pos[2] <= 100

    def _reachable_nodes(self, positions, reference_positions=None):
        live=[i for i,u in self.state.uavs.items() if u.is_operational()]
        d=self.disturbance_record
        if d and (d['type']=='failure' or self.state.time<d['time_s']+30):
            live=[i for i in live if i!=d['uav_id']]
        points=np.vstack([np.zeros(3),positions[live]])
        adjacent=np.linalg.norm(points[:,None,:]-points[None,:,:],axis=2)<=100+1e-8
        if reference_positions is not None:
            reference=np.vstack([np.zeros(3),reference_positions[live]])
            adjacent &= np.linalg.norm(reference[:,None,:]-reference[None,:,:],axis=2)<=100+1e-8
        reached={0};front=[0]
        while front:
            node=front.pop()
            for neighbor in np.flatnonzero(adjacent[node]):
                if int(neighbor) not in reached:
                    reached.add(int(neighbor));front.append(int(neighbor))
        return {live[i-1] for i in reached if i > 0}

    def _reachable_count(self, positions, reference_positions=None):
        return len(self._reachable_nodes(positions,reference_positions))

    def _connectivity_during_segment(self, starts, ends, active):
        # A range graph can change only at roots of ||r + t*v|| = range.
        # Evaluate each open interval between roots: exact for our linear motion.
        a=np.vstack([np.zeros(3),starts]);b=np.vstack([np.zeros(3),ends])
        r=a[:,None,:]-a[None,:,:]
        v=(b-a)[:,None,:]-(b-a)[None,:,:]
        qa=np.sum(v*v,axis=2);qb=2*np.sum(r*v,axis=2)
        qc=np.sum(r*r,axis=2)-100**2
        disc=qb*qb-4*qa*qc
        mask=np.triu((qa>1e-14)&(disc>=0),1)
        den=2*np.maximum(qa,1e-20)
        root=np.sqrt(np.maximum(disc,0))
        roots=np.concatenate([((-qb-root)/den)[mask],((-qb+root)/den)[mask]])
        cuts=np.unique(np.concatenate([[0.,1.],roots[(roots>0)&(roots<1)]]))
        downtime=0.;minimum=len(active)
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            connected=len(active & self._reachable_nodes(starts+(ends-starts)*((lo+hi)/2)))
            minimum=min(minimum,connected)
            if connected<len(active):
                downtime+=(hi-lo)*self.dt
        return minimum,float(downtime)

    def _move(self):
        ids = list(self.state.uavs)
        starts = np.array([self.state.uavs[i].position.to_array() for i in ids],dtype=float)
        ends = starts.copy()
        active={i for i,f in self.flights.items() if f.armed and self.state.uavs[i].is_operational()}
        connected_floor = self._reachable_count(starts)
        # Fixed priority plus alternate traversal reduces deadlock without bypassing vetoes.
        order = sorted(ids, key=lambda i:(not self.flights[i].returning, (i+int(self.state.time)//20)%len(ids)))
        for uid in order:
            u,f = self.state.uavs[uid],self.flights[uid]
            if not u.is_operational() or u.target_position is None or f.charge_started_s is not None:
                continue
            target = u.target_position.to_array().astype(float)
            if f.returning and np.linalg.norm(starts[uid,:2]-target[:2]) <= 3:
                f.landing_approach=True
            if f.returning and not f.landing_approach:
                target[2] = 90
                if starts[uid,2] < 89:
                    target = np.array([*starts[uid,:2],90.])
            if not f.returning and starts[uid,2] < 39 and np.linalg.norm(starts[uid,:2]-target[:2]) > 5:
                target = np.array([*starts[uid,:2],40.])
            delta = target-starts[uid]
            distance = np.linalg.norm(delta)
            if distance < 1e-9:
                continue
            if not f.armed:
                if f.returning:
                    continue
                f.armed, f.landed, f.settled_s = True, False, 0.
                active.add(uid)
                self.event('takeoff_started',uav_id=uid)
            speed=5.
            if f.slot is not None and not f.returning and self.slots and u.position.z>=39:
                speed=5.*(f.slot+1)/len(self.slots)
            desired = delta*min(1.,speed*self.dt/distance)
            candidates = [desired]
            if distance > 3:
                d = desired/(np.linalg.norm(desired)+1e-12)
                side = np.array([-d[1],d[0],0.])
                for sign in (1,-1):
                    offset=d+sign*side*1.5
                    candidates.append(offset/max(np.linalg.norm(offset),1e-9)*min(distance,5*self.dt))
                # Vertical takeoff/return can meet a relay directly overhead;
                # provide horizontal escape choices before backing down forever.
                candidates += [np.array([x*5*self.dt,y*5*self.dt,0.])
                               for x,y in ((-1,0),(0,1),(1,0),(0,-1))]
                candidates += [np.array([0.,0.,self.dt*3]),np.array([0.,0.,-self.dt*3])]
            candidates.append(np.zeros(3))
            for step in candidates:
                if np.linalg.norm(step) > self.constraints.max_speed_mps*self.dt+1e-12:
                    continue
                proposed = starts[uid]+step
                if not self._inside(proposed):
                    continue
                relative=starts[uid]-starts
                change=step-(ends-starts)
                denom=np.sum(change*change,axis=1)
                when=np.clip(-np.sum(relative*change,axis=1)/np.maximum(denom,1e-20),0,1)
                closest=np.linalg.norm(relative+change*when[:,None],axis=1)
                closest[uid]=np.inf
                if float(np.min(closest)) >= 20-1e-8:
                    trial=ends.copy();trial[uid]=proposed
                    reachable=self._reachable_count(trial,starts)
                    emergency_return=(f.returning and f.remaining_s <=
                        (math.hypot(u.position.x-f.home[0],u.position.y-f.home[1])+180)/5+60)
                    if emergency_return or reachable >= connected_floor:
                        ends[uid]=proposed
                        connected_floor=reachable
                        break
        self.interval_active=set(active)
        self.interval_connected,self.interval_downtime=self._connectivity_during_segment(starts,ends,active)
        distances=pair_distances(starts,ends)
        for first,second in zip(*np.where(np.triu(distances<1.,1))):
            self.collision_pairs.add((int(first),int(second)))
        minimum=float(np.min(distances))
        self.min_separation=min(self.min_separation,minimum)
        self.separation_violations += int(minimum < 20-1e-7)
        for uid in ids:
            u,f=self.state.uavs[uid],self.flights[uid]
            velocity=(ends[uid]-starts[uid])/self.dt
            u.position=Vector3D(*ends[uid]);u.velocity=Vector3D(*velocity)
            f.observed_at_s=self.state.time+self.dt
            self.max_speed=max(self.max_speed,float(np.linalg.norm(velocity)))
            self.max_altitude=max(self.max_altitude,float(ends[uid,2]))
            self.geofence_violations += int(not self._inside(ends[uid]))
            if f.armed and u.is_operational():
                f.sortie_s+=self.dt;f.remaining_s=max(0.,f.remaining_s-self.dt)
                f.max_sortie_s=max(f.max_sortie_s,f.sortie_s)
                if f.remaining_s <= 0:
                    self.exhaustions+=1;u.state=UAVState.FAILED;u.role=UAVRole.FAILED
                    self.event('battery_exhausted',event_time_s=self.state.time+self.dt,uav_id=uid)
            u.battery_level=f.remaining_s/1200
            at_home=math.dist(ends[uid],f.home)<=.25 and np.linalg.norm(velocity)<=.15
            if f.returning and at_home and u.is_operational():
                f.settled_s+=self.dt
                # In this kinematic model the local disarm action is synchronous,
                # but completion still requires subsequent settled pose evidence.
                f.armed=False
                if measured_landing_complete(ends[uid],velocity,f.home,f.observed_at_s,
                                             self.state.time+self.dt,f.armed,f.settled_s):
                    if not f.landed:
                        self.event('landing_confirmed',event_time_s=self.state.time+self.dt,uav_id=uid,position=ends[uid].tolist(),
                                   velocity=velocity.tolist(),sortie_s=f.sortie_s,
                                   disarm_acknowledged=True,observed_at_s=f.observed_at_s,evidence='kinematic_state')
                    f.landed=True;f.returning=False;f.charge_started_s=self.state.time+self.dt
                    u.target_position=None;u.state=UAVState.CHARGING;u.role=UAVRole.CHARGING
            else:
                f.settled_s=0
            if f.charge_started_s is not None:
                # Team assumption: 12-minute full recharge, no instantaneous reset.
                f.remaining_s=min(1200.,f.remaining_s+self.dt*1200/720)
                if f.remaining_s>=1200:
                    f.sortie_s=0.;f.charge_started_s=None
                    u.state=UAVState.IDLE;u.role=UAVRole.STANDBY
                    self.event('recharge_completed',event_time_s=self.state.time+self.dt,uav_id=uid)
            u.battery_level=f.remaining_s/1200

    def _network(self):
        self.router.update_network_topology()
        now=self.state.time
        if self.disturbance!='none' and now>=900 and self.disturbance_record is None:
            candidates=[i for i,f in self.flights.items() if f.armed and self.state.uavs[i].role==UAVRole.RELAY]
            if candidates:
                uid=candidates[len(candidates)//2]
                affected=[i for i,f in self.flights.items() if f.armed and i!=uid and self.state.uavs[i].connected_to_gcs]
                self.disturbance_record=dict(type=self.disturbance,time_s=now,uav_id=uid,affected=affected,recovered_s=None,delivered_after=[])
                self.event('disturbance_injected',disturbance=self.disturbance,uav_id=uid,affected=list(affected))
                if self.disturbance=='failure':
                    self.state.uavs[uid].state=UAVState.FAILED;self.state.uavs[uid].role=UAVRole.FAILED
                    self.state.uavs[uid].target_position=None
        d=self.disturbance_record
        if d and (d['type']=='failure' or now < d['time_s']+30):
            uid=d['uav_id']
            if uid in self.router.network_graph:
                self.router.network_graph.remove_node(uid)
            self.router.link_quality_cache={k:v for k,v in self.router.link_quality_cache.items() if uid not in k}
        for uid,u in self.state.uavs.items():
            u.connected_to_gcs=(uid in self.router.network_graph and nx.has_path(self.router.network_graph,uid,-1))
        self.router.compute_routing_tables()
        self.router.process_packet_forwarding(self.dt)
        for e in self.router.packet_events[self.packet_cursor:]:
            if e['type']!='delivered':
                continue
            payload=e.get('payload') or {}
            if 'poi_id' in payload:
                p=next(p for p in self.tasks if p.id==payload['poi_id'])
                if p.report_s is None:
                    p.report_s=e['time_s']
                    self.event('poi_received',poi_id=p.id,detection_time_s=p.detection_s,
                               packet_id=e['packet_id'],delay_s=p.report_s-p.detection_s)
            if d and e['creation_time_s']>=d['time_s'] and e['source_id'] in d['affected']:
                if e['source_id'] not in d['delivered_after']:
                    d['delivered_after'].append(e['source_id'])
        self.packet_cursor=len(self.router.packet_events)
        if d and d['affected'] and d['recovered_s'] is None and set(d['affected']) <= set(d['delivered_after']):
            d['recovered_s']=now
            self.event('service_recovery_confirmed',duration_s=now-d['time_s'])
        if now>=self.next_telemetry_s:
            self.next_telemetry_s=now+1
            for uid,f in self.flights.items():
                if f.armed and self.state.uavs[uid].is_operational():
                    self._send(uid,dict(kind='telemetry'),256)

    def _send(self,uid,payload,size=2048):
        self.router.send_packet(Packet(self.router.packets_sent,PacketType.DATA,uid,-1,
                                       ttl=32,payload_size=size,payload=payload))

    def _sense(self):
        now=self.state.time
        for p in self.tasks:
            if p.spawn_s>now or (p.surveyed and p.report_s is not None):
                continue
            candidates=[i for i,u in self.state.uavs.items() if self.flights[i].armed and u.is_operational()
                        and math.hypot(u.position.x-p.position[0],u.position.y-p.position[1])<=5
                        and 30<=u.position.z<=70]
            if not candidates:
                p.dwell_s=0
                continue
            uid=min(candidates)
            if p.detection_s is None:
                p.detection_s=now;p.detected_by=uid
                self.event('poi_detected',poi_id=p.id,uav_id=uid,observation_time_s=now,
                           source='synthetic_footprint')
            p.dwell_s+=self.dt
            if p.dwell_s>=30 and not p.surveyed:
                p.surveyed=True;self.event('poi_surveyed',poi_id=p.id,uav_id=uid)
            if p.report_s is None and now-p.last_report_attempt_s>=2:
                p.last_report_attempt_s=now
                self._send(uid,dict(kind='poi_report',poi_id=p.id,detection_time_s=p.detection_s))

    def step(self):
        self._network()
        self._plan()
        self._move()
        self.state.time=round(self.state.time+self.dt,9)
        self._sense()
        self.samples.append(dict(time_s=self.state.time,dt_s=self.dt,active=len(self.interval_active),
                                 connected=self.interval_connected,downtime_s=self.interval_downtime))
        if (self.tasks and all(p.surveyed and p.report_s is not None for p in self.tasks)
                and all(f.landed and not f.armed and self.state.uavs[i].is_operational() for i,f in self.flights.items())):
            self.completion_s=self.state.time

    def report(self):
        c=self.constraints
        completed=[p for p in self.tasks if p.surveyed and p.report_s is not None]
        late=[p.id for p in self.tasks if p.detection_s is not None and
              (p.report_s is None or p.report_s-p.detection_s>10+1e-8)]
        active_time=sum(s['dt_s'] for s in self.samples if s['active'])
        downtime=sum(s['downtime_s'] for s in self.samples)
        all_landed=all(f.landed and not f.armed and self.state.uavs[i].is_operational() for i,f in self.flights.items())
        maximum_sortie=max(f.max_sortie_s for f in self.flights.values())
        checks=dict(all_pois_surveyed_and_reported=len(completed)==len(self.tasks),
                    sample_has_ten_pois=len(self.tasks)==10,all_uavs_landed=all_landed,
                    mission_deadline=self.completion_s is not None and self.completion_s<=2700,
                    endurance=maximum_sortie<=1200,speed=self.max_speed<=5+1e-8,
                    altitude=self.max_altitude<=100,separation=self.separation_violations==0,
                    geofence=self.geofence_violations==0,no_battery_exhaustion=self.exhaustions==0,
                    continuous_connectivity=downtime<1e-8,
                    reporting_deadline=not late and all(p.detection_s is not None for p in self.tasks))
        return dict(evidence_scope='abstract kinematic simulation; synthetic PoI observations; not flight qualification',
                    schema_version=1,dt_s=self.dt,packet_loss_per_attempt=self.packet_loss,requirements=asdict(c),seed=self.seed,fleet_size=len(self.flights),
                    disturbance=self.disturbance_record,completion_time_s=self.completion_s,
                    elapsed_time_s=self.state.time,pois_completed=len(completed),pois_total=len(self.tasks),
                    priority_weighted_score=100*sum(p.priority for p in completed)/sum(p.priority for p in self.tasks),
                    packet_statistics=self.router.get_network_stats(),
                    connectivity_availability_percent=100*(active_time-downtime)/active_time if active_time else None,
                    communication_downtime_s=downtime,
                    minimum_separation_m=self.min_separation if math.isfinite(self.min_separation) else None,
                    maximum_speed_mps=self.max_speed,maximum_altitude_m=self.max_altitude,
                    maximum_sortie_s=maximum_sortie,battery_exhaustions=self.exhaustions,
                    collision_count=len(self.collision_pairs),separation_violation_steps=self.separation_violations,
                    geofence_violation_steps=self.geofence_violations,
                    relay_assignments=sum(e['type']=='role_assigned' for e in self.events),
                    relay_handoffs_completed=sum(e['type']=='relay_handoff_completed' for e in self.events),
                    network_reconfiguration_efficiency=None,
                    reporting_deadline_misses=late,tasks=[asdict(p) for p in self.tasks],
                    checks=checks,sample_constraints_passed=all(checks.values()))

    def run(self, output, max_time=2700):
        if not math.isfinite(max_time) or not 0<max_time<=2700:
            raise ValueError('max_time must be positive and no greater than 2700')
        output=Path(output);output.mkdir(parents=True,exist_ok=False)
        next_sample=0.
        with (output/'trajectory.jsonl').open('w',encoding='utf-8') as stream:
            while self.state.time+self.dt<=max_time+1e-8 and self.completion_s is None:
                self.step()
                if self.state.time>=next_sample:
                    stream.write(json.dumps(dict(time_s=self.state.time,
                        uavs=[dict(id=i,position=u.position.to_array().tolist(),role=u.role.value,
                                   battery=u.battery_level,armed=self.flights[i].armed,landed=self.flights[i].landed)
                              for i,u in self.state.uavs.items()],links=list(self.router.network_graph.edges())))+'\n')
                    next_sample+=2
        for name,data in [('metrics',self.report()),('events',self.events),('packets',self.router.packet_events),('connectivity',self.samples)]:
            (output/f'{name}.json').write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf-8')
        root=Path(__file__).resolve().parents[1]
        source_paths=['simulation/uavx.py','core/mission_safety.py','core/models.py',
                      'core/config.py','comm/mesh_router.py','comm/channel.py',
                      'comm/channel_legacy.py','run_simulation.py']
        manifest=dict(schema_version=1,python=platform.python_version(),numpy=np.__version__,
                      networkx=nx.__version__,source_sha256=self.source_sha256,
                      output_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir()) if p.is_file()})
        (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        return self.report()
