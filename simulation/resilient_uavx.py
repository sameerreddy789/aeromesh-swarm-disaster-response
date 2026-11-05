"""Scenario-driven UAV-X with protected relays and measured image transport.

Motion remains explicitly kinematic. The optional Panda3D scene supplies actual
rendered camera pixels; complete image receipts, not metadata offers, establish
report delivery. Failed aircraft never receive an exemption from strict checks.
"""
import base64
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import random

import networkx as nx
import numpy as np

from comm.mesh_router import MeshRouter
from core.config import RFConfig
from core.models import UAVRole, UAVState, Vector3D
from simulation.uavx import UAVXSimulator, SurveyTask, RangeChannel
from simulation.uavx_scenario import ScenarioConfig, ScenarioTask


def _protected_graph_score(adjacent, fleet_size):
    """Reachability first, then root-biconnected vertices; row zero is GCS.

    One Tarjan traversal avoids constructing a NetworkX graph for every trial
    movement. Closing a non-root block removes its child vertices while keeping
    the articulation parent; root blocks of at least three vertices supply the
    two internally node-disjoint paths counted by the motion admission rule.
    """
    neighbors = [np.flatnonzero(row).tolist() for row in adjacent]
    discovered = [-1] * len(neighbors)
    low = [0] * len(neighbors)
    stack = []
    visited = protected = 0

    def visit(node, parent):
        nonlocal visited, protected
        discovered[node] = low[node] = visited
        visited += 1
        stack.append(node)
        for other in neighbors[node]:
            if other == parent or other == node:
                continue
            if discovered[other] < 0:
                visit(other, node)
                low[node] = min(low[node], low[other])
                if low[other] >= discovered[node]:
                    size = 0
                    while True:
                        member = stack.pop()
                        size += 1
                        if member == other:
                            break
                    if node == 0 and size >= 2:
                        protected += size
            else:
                low[node] = min(low[node], discovered[other])

    visit(0, -1)
    return (visited - 1) * (fleet_size + 1) + protected


class ResilientUAVXSimulator(UAVXSimulator):
    schema_version = 2
    CHUNK_BYTES = 512

    def __init__(self, scenario: ScenarioConfig):
        if not isinstance(scenario, ScenarioConfig):
            raise TypeError('scenario must be a validated ScenarioConfig')
        tasks = ([SurveyTask(p.id, p.position, p.spawn_s, p.priority) for p in scenario.tasks]
                 if scenario.tasks is not None else None)
        super().__init__(seed=scenario.seed, fleet_size=scenario.fleet_size,
                         packet_loss=scenario.packet_loss, tasks=tasks)
        # Store the realized tasks as well as the seed so scenarios replay even
        # if future random-generation code changes.
        self.scenario = replace(scenario, tasks=tuple(
            ScenarioTask(p.id, p.position, p.spawn_s, p.priority) for p in self.tasks))
        self.relay_spacing_m = 44. if scenario.relay_protection else 85.
        self.planning_separation_m = min(22., scenario.pad_spacing_m)
        self.recharge_s = scenario.recharge_s
        columns, rows = scenario.staging_columns, math.ceil(scenario.fleet_size/scenario.staging_columns)
        for uid, flight in self.flights.items():
            flight.home = ((uid % columns - (columns-1)/2)*scenario.pad_spacing_m,
                           (uid // columns - (rows-1)/2)*scenario.pad_spacing_m, 0.)
            self.state.uavs[uid].position = Vector3D(*flight.home)
        self.router = MeshRouter(self.state, max_range_m=100., rng=random.Random(scenario.seed+1),
                                 hop_delay_s=self.dt, node_bandwidth_bytes_s=scenario.radio_bytes_per_s,
                                 queue_limit_bytes=scenario.radio_queue_bytes)
        self.router.cfg_rf = RFConfig(max_hops=32)
        self.router.rf_channel = RangeChannel(scenario.packet_loss)
        self.fault_records = []
        self._injected_faults = set()
        self._camera = None
        self._scene_metadata = None
        self._observations = {}
        self._capture_history = []
        self._capture_metadata = {}
        self._capture_bytes = {}
        self._assembled_images = {}
        self._image_bytes = {}
        self._chunks_received = {}
        self._received_images = {}
        self._output = None
        self.survivor_completion_s = None
        self._protection_samples = []
        self._next_protection_sample = 0.
        self._charge_admission = set()
        root = Path(__file__).resolve().parents[1]
        for name in ('simulation/resilient_uavx.py', 'simulation/uavx_scenario.py', 'simulation/camera_sensor.py'):
            self.source_sha256[name] = hashlib.sha256((root/name).read_bytes()).hexdigest()

    def _charging_allowed(self, uid):
        return uid in self._charge_admission

    def _select_task(self, available, current):
        priority = max(p.priority for p in available)
        if current is not None and current.priority >= priority:
            return current
        previous = next((p for p in self.tasks if p.id==self.active_id),None)
        origin = previous.position if previous is not None else (0.,0.,0.)
        # Among equally urgent announced tasks, reduce chain travel instead of
        # repeatedly crossing the full map in spawn order. Priority preemption
        # remains dominant and future task coordinates never enter this choice.
        return min((p for p in available if p.priority==priority),
                   key=lambda p:(math.dist(origin,p.position),p.spawn_s,p.id))

    def _move(self):
        # Admission is frozen for this interval. A pad released during the loop
        # becomes available next interval, never supplies two full dt charges.
        waiting = sorted((f.charge_started_s, i) for i,f in self.flights.items()
                         if f.charge_started_s is not None)
        count = self.scenario.charging_pads or len(self.flights)
        self._charge_admission = {i for _,i in waiting[:count]}
        super()._move()

    def _reachable_count(self, positions, reference_positions=None):
        if not self.scenario.relay_protection:
            return super()._reachable_count(positions, reference_positions)
        excluded, blocked_edges = self._radio_exclusions()
        live = [i for i,u in self.state.uavs.items() if u.is_operational() and i not in excluded]
        ids = [-1, *live]
        points = np.vstack([np.zeros(3), positions[live]])
        adjacent = np.linalg.norm(points[:,None,:]-points[None,:,:], axis=2) <= 100+1e-8
        if reference_positions is not None:
            reference = np.vstack([np.zeros(3), reference_positions[live]])
            # A retained edge exists at both ends of the linear interval, so it
            # also exists throughout. New endpoint-only links cannot disguise
            # a temporary loss of the second path while vehicles are moving.
            adjacent &= np.linalg.norm(reference[:,None,:]-reference[None,:,:], axis=2) <= 100+1e-8
        indices = {uid:index for index,uid in enumerate(ids)}
        for first,second in blocked_edges:
            if first in indices and second in indices:
                a,b = indices[first],indices[second]
                adjacent[a,b] = adjacent[b,a] = False
        # This hook is only the motion admission score, never an availability
        # metric. Connectivity remains dominant; within a connected component
        # retain redundant paths during deployments, replacements and pivots.
        return _protected_graph_score(adjacent, len(self.flights))

    def _prepare_output(self, output):
        self._output = output
        if self.scenario.camera_mode == 'panda3d':
            self._ensure_camera()

    def _ensure_camera(self):
        if self._camera is None:
            from simulation.camera_sensor import DisasterCamera
            self._camera = DisasterCamera(self.tasks, seed=self.seed, width=self.scenario.camera_width,
                                          height=self.scenario.camera_height)
            self._scene_metadata = self._camera.get_scene_metadata()
        return self._camera

    def close(self):
        if self._camera is not None:
            self._camera.close()
            self._camera = None

    def run(self, output, max_time=2700, observer=None):
        try:
            return super().run(output, max_time, observer)
        finally:
            self.close()

    def _resolve_target(self, fault):
        if not isinstance(fault.target, str):
            return fault.target
        relays = sorted((i for i,f in self.flights.items() if f.armed and not f.returning
                         and i in self.router.network_graph and self.state.uavs[i].is_operational()
                         and self.state.uavs[i].role == UAVRole.RELAY),
                        key=lambda i: self.state.uavs[i].position.norm())
        if fault.target == 'middle_active_relay':
            return relays[len(relays)//2] if relays else None
        active = sorted((i for i,f in self.flights.items() if f.armed
                         and self.state.uavs[i].connected_to_gcs),
                        key=lambda i: self.state.uavs[i].position.norm(), reverse=True)
        for uid in active:
            if nx.has_path(self.router.network_graph, -1, uid):
                path = nx.shortest_path(self.router.network_graph, -1, uid)
                if len(path) >= 3:
                    index = (len(path)-1)//2
                    return tuple(sorted((path[index],path[index+1])))
        return None

    def _update_faults(self):
        now = self.state.time
        for record in self.fault_records:
            if (record['duration_s'] is not None and record['cleared_s'] is None
                    and now >= record['time_s'] + record['duration_s']):
                record['cleared_s'] = now
                self.event('scenario_fault_cleared', fault_id=record['fault_id'], kind=record['kind'])
        self._mask_network()
        for index, fault in enumerate(self.scenario.faults):
            if index in self._injected_faults or fault.time_s > now:
                continue
            target = self._resolve_target(fault)
            if target is None:
                # A selector is resolved only once an eligible resource exists.
                # Both scheduled and actual times are retained, never backdated.
                continue
            affected = [i for i,f in self.flights.items() if f.armed and self.state.uavs[i].is_operational()
                        and self.state.uavs[i].connected_to_gcs
                        and (fault.kind == 'link_outage' or i != target)]
            record = dict(fault_id=index, kind=fault.kind, scheduled_s=fault.time_s, time_s=now,
                          target=list(target) if isinstance(target,tuple) else target,
                          duration_s=fault.duration_s, cleared_s=None, affected_sources=affected,
                          delivered_after=[], recovered_s=None)
            self.fault_records.append(record)
            self._injected_faults.add(index)
            self.event('scenario_fault_injected', fault_id=index, kind=fault.kind, target=record['target'],
                       scheduled_s=fault.time_s, affected_sources=affected)
            if fault.kind == 'vehicle_failure':
                u = self.state.uavs[target]
                u.state, u.role, u.target_position = UAVState.FAILED, UAVRole.FAILED, None
                u.velocity = Vector3D(0.,0.,0.)
            self._mask_network()

    def _radio_exclusions(self):
        nodes, edges = set(), set()
        for r in self.fault_records:
            if r['cleared_s'] is not None:
                continue
            if r['kind'] == 'link_outage':
                edges.add(tuple(r['target']))
            else:
                nodes.add(r['target'])
        return nodes, edges

    def _plan(self):
        super()._plan()
        obstacles = [u.position.to_array() for u in self.state.uavs.values() if not u.is_operational()]
        if not obstacles or self.shutdown:
            return
        # A frozen aircraft must remain an obstacle. A replacement can observe
        # the same ground site from another permitted altitude instead of being
        # sent indefinitely into the failed aircraft's occupied target point.
        for slot,goal in list(self.slots.items()):
            if all(math.dist(goal,point) >= self.planning_separation_m for point in obstacles):
                continue
            options=[(*goal[:2],height) for height in (40.,65.,30.,70.)]
            safe=[point for point in options if all(math.dist(point,obstacle) >= self.planning_separation_m
                                                    for obstacle in obstacles)]
            if safe:
                chosen=min(safe,key=lambda point:abs(point[2]-goal[2]))
                self.slots[slot]=chosen
                uid=self.slot_owners.get(slot)
                if uid is not None and not self.flights[uid].returning:
                    self.state.uavs[uid].target_position=Vector3D(*chosen)

    def _mask_network(self):
        blocked_nodes, blocked_edges = self._radio_exclusions()
        self.router.network_graph.remove_nodes_from(blocked_nodes)
        self.router.network_graph.remove_edges_from(blocked_edges)
        blocked_pairs = {frozenset(edge) for edge in blocked_edges}
        self.router.link_quality_cache = {pair: quality for pair,quality in self.router.link_quality_cache.items()
                                         if not blocked_nodes.intersection(pair) and frozenset(pair) not in blocked_pairs}
        graph = self.router.network_graph
        connected = nx.node_connected_component(graph,-1) if -1 in graph else set()
        for uid,u in self.state.uavs.items():
            u.connected_to_gcs = uid in connected

    def _network(self):
        self.router.update_network_topology()
        self._update_faults()
        graph = self.router.network_graph
        # UAV-X traffic is addressed only to the GCS. Avoid two all-destination
        # Dijkstra traversals per aircraft per tick; packet routes themselves
        # still use the router's measured current graph and normal retry logic.
        self.router.routing_table.clear()
        if -1 in graph:
            costs, paths = nx.single_source_dijkstra(graph, -1, weight='weight')
            for uid,path in paths.items():
                if uid != -1 and len(path)-1 <= self.router.cfg_rf.max_hops:
                    self.router.routing_table[uid][-1] = (path[-2], costs[uid])
        self.router.process_packet_forwarding(self.dt)
        for event in self.router.packet_events[self.packet_cursor:]:
            if event['type'] != 'delivered':
                continue
            payload = event.get('payload') or {}
            if payload.get('kind') == 'poi_image_chunk':
                self._accept_image_chunk(event)
            elif payload.get('kind') == 'poi_report' and self.scenario.camera_mode == 'footprint':
                task = next(p for p in self.tasks if p.id == payload['poi_id'])
                if task.report_s is None:
                    task.report_s = event['time_s']
                    self.event('poi_received', poi_id=task.id, detection_time_s=task.detection_s,
                               packet_id=event['packet_id'], delay_s=task.report_s-task.detection_s)
            for record in self.fault_records:
                if event['creation_time_s'] >= record['time_s'] and event['source_id'] in record['affected_sources']:
                    if event['source_id'] not in record['delivered_after']:
                        record['delivered_after'].append(event['source_id'])
        self.packet_cursor = len(self.router.packet_events)
        for record in self.fault_records:
            if (record['affected_sources'] and record['recovered_s'] is None
                    and set(record['affected_sources']) <= set(record['delivered_after'])):
                record['recovered_s'] = self.state.time
                self.event('scenario_service_recovered', fault_id=record['fault_id'],
                           duration_s=self.state.time-record['time_s'])
        if self.state.time >= self.next_telemetry_s:
            self.next_telemetry_s = self.state.time + 1.
            for uid,f in self.flights.items():
                if f.armed and self.state.uavs[uid].is_operational():
                    self._send(uid, dict(kind='telemetry'), 256)
        if self.state.time >= self._next_protection_sample:
            self._next_protection_sample = self.state.time + 5.
            # Node-disjoint paths are measured, not inferred from spare count.
            active = [i for i,f in self.flights.items() if f.armed and self.state.uavs[i].is_operational()]
            protected = []
            for component in nx.biconnected_components(graph):
                if -1 in component and len(component) >= 3:
                    protected.extend(i for i in active if i in component)
            self._protection_samples.append(dict(time_s=self.state.time, active=len(active),
                                                  protected=len(set(protected))))

    def _capture(self, task, uid):
        capture = self._ensure_camera().capture(uid, self.state.uavs[uid].position.to_array().tolist(),
                                                 self.state.time, poi_id=task.id)
        count = math.ceil(len(capture.image_bytes)/self.CHUNK_BYTES)
        attempt = sum(o['poi_id']==task.id for o in self._capture_history)
        suffix = '' if attempt==0 else f'-attempt-{attempt}'
        metadata = dict(capture.metadata, attempt=attempt, chunk_count=count,
                        detection_time_s=task.detection_s if task.detection_s is not None else self.state.time,
                        capture_file=f'sensing/captures/poi-{task.id}{suffix}.png', received_file=None, received_at_s=None)
        self._observations[task.id] = metadata
        self._capture_history.append(metadata)
        self._capture_metadata[metadata['capture_id']] = metadata
        self._capture_bytes[metadata['capture_id']] = capture.image_bytes
        self._image_bytes[task.id] = capture.image_bytes
        self._chunks_received[metadata['capture_id']] = {}

    def _accept_image_chunk(self, event):
        payload = event['payload']
        pid = payload['poi_id']
        identity = payload.get('capture_id')
        metadata = self._capture_metadata.get(identity)
        if metadata is None or identity in self._assembled_images:
            return
        if (payload.get('image_sha256') != metadata['sha256']
                or metadata['poi_id'] != pid
                or payload.get('detection_time_s') != metadata['detection_time_s']
                or payload.get('observation_time_s') != metadata['acquired_at_s']
                or event['source_id'] != metadata['uav_id']
                or payload.get('chunk_count') != metadata['chunk_count']):
            raise ValueError('image chunk provenance does not match its capture')
        index = payload['chunk_index']
        if type(index) is not int or not 0 <= index < metadata['chunk_count']:
            raise ValueError('invalid image chunk index')
        data = base64.b64decode(payload['data_b64'], validate=True)
        chunks = self._chunks_received[identity]
        if index in chunks and chunks[index] != data:
            raise ValueError('conflicting duplicate image chunk')
        chunks[index] = data
        if len(chunks) != metadata['chunk_count']:
            return
        image = b''.join(chunks[i] for i in range(metadata['chunk_count']))
        if len(image) != metadata['byte_length'] or hashlib.sha256(image).hexdigest() != metadata['sha256']:
            raise ValueError('reassembled image integrity failure')
        self._assembled_images[identity] = image
        task = next(p for p in self.tasks if p.id == pid)
        suffix = '' if metadata['attempt']==0 else f'-attempt-{metadata["attempt"]}'
        metadata.update(received_at_s=event['time_s'], received_file=f'sensing/received/poi-{pid}{suffix}.png')
        if task.report_s is None:
            self._received_images[pid] = image
            self._observations[pid] = metadata
            task.report_s = event['time_s']
            self.event('poi_received', poi_id=pid, detection_time_s=task.detection_s,
                       packet_id=event['packet_id'], delay_s=task.report_s-task.detection_s,
                       capture_id=identity, image_sha256=metadata['sha256'], image_bytes=len(image), chunks=len(chunks))

    def _sense(self):
        if self.scenario.camera_mode == 'footprint':
            return super()._sense()
        now = self.state.time
        for task in self.tasks:
            if task.spawn_s > now or (task.surveyed and task.report_s is not None):
                continue
            candidates = [i for i,u in self.state.uavs.items() if self.flights[i].armed and u.is_operational()
                          and math.hypot(u.position.x-task.position[0],u.position.y-task.position[1]) <= 5
                          and 30 <= u.position.z <= 70]
            if not candidates:
                task.dwell_s = 0.
            else:
                uid = min(candidates)
                if task.detection_s is None:
                    self._capture(task, uid)
                    task.detection_s, task.detected_by = now, uid
                    self.event('poi_detected', poi_id=task.id, uav_id=uid, observation_time_s=now,
                               source='panda3d_tinydisplay', image_sha256=self._observations[task.id]['sha256'])
                elif task.report_s is None and not self.state.uavs[self._observations[task.id]['uav_id']].is_operational():
                    # A new observing UAV can recover the mission after the
                    # original sensor carrier fails. Preserve the first deadline
                    # and original capture as evidence; never backdate the retry.
                    self._capture(task,uid)
                    self.event('poi_recaptured',poi_id=task.id,uav_id=uid,
                               observation_time_s=now,detection_time_s=task.detection_s,
                               capture_id=self._observations[task.id]['capture_id'])
                task.dwell_s += self.dt
                if task.dwell_s >= 30 and not task.surveyed:
                    task.surveyed = True
                    self.event('poi_surveyed', poi_id=task.id, uav_id=uid)
            if (task.detection_s is not None and task.report_s is None
                    and now-task.last_report_attempt_s >= 2
                    and self.state.uavs[self._observations[task.id]['uav_id']].is_operational()):
                task.last_report_attempt_s = now
                metadata, image = self._observations[task.id], self._image_bytes[task.id]
                for index in range(metadata['chunk_count']):
                    self._send(metadata['uav_id'], dict(kind='poi_image_chunk', poi_id=task.id,
                        priority=task.priority, detection_time_s=task.detection_s,
                        observation_time_s=metadata['acquired_at_s'],capture_id=metadata['capture_id'],
                        image_sha256=metadata['sha256'], chunk_count=metadata['chunk_count'], chunk_index=index,
                        data_b64=base64.b64encode(image[index*self.CHUNK_BYTES:(index+1)*self.CHUNK_BYTES]).decode('ascii')), 0)

    def step(self):
        super().step()
        survivors = [i for i,u in self.state.uavs.items() if u.is_operational()]
        if (self.survivor_completion_s is None and survivors
                and all(p.surveyed and p.report_s is not None for p in self.tasks)
                and all(self.flights[i].landed and not self.flights[i].armed for i in survivors)):
            self.survivor_completion_s = self.state.time

    def report(self):
        report = super().report()
        report['schema_version'] = 2
        report['evidence_scope'] = ('kinematic motion; Panda3D rendered RGB observations; bounded per-node radio service; '
                                    'not physical-flight qualification' if self.scenario.camera_mode == 'panda3d' else
                                    'kinematic motion; synthetic footprint observations; bounded per-node radio service')
        report['checks']['scheduled_faults_exercised'] = len(self._injected_faults) == len(self.scenario.faults)
        report['checks']['imagery_received'] = len(self._received_images) == len(self.tasks)
        report['sample_constraints_passed'] = all(report['checks'].values())
        report['scenario'] = self.scenario.to_dict()
        report['task_policy'] = 'highest priority first; nearest announced task after current task; no preemption for equal priority'
        report['faults'] = self.fault_records
        report['sensing'] = dict(mode=self.scenario.camera_mode, captures=len(self._capture_history),
                                 received_images=len(self._received_images),
                                 received_bytes=sum(map(len,self._received_images.values())))
        survivors = [i for i,u in self.state.uavs.items() if u.is_operational()]
        report['survivor_metrics'] = dict(operational_uavs=len(survivors), failed_uavs=len(self.flights)-len(survivors),
            all_survivors_landed=bool(survivors) and all(self.flights[i].landed and not self.flights[i].armed for i in survivors),
            completion_time_s=self.survivor_completion_s,
            note='Separate mission-continuity measure; never replaces strict all-UAV landing/completion.')
        report['relay_protection'] = dict(enabled=self.scenario.relay_protection, slot_spacing_m=self.relay_spacing_m,
                                         planning_separation_m=self.planning_separation_m, samples=self._protection_samples)
        dispatched = sum(e['type']=='replacement_dispatched' for e in self.events)
        report['network_reconfiguration_efficiency'] = report['relay_handoffs_completed']/dispatched if dispatched else None
        report['reconfiguration_efficiency_definition'] = 'measured completed relay handoffs / dispatched replacements; null if no dispatch'
        return report

    def _write_extra_evidence(self, output):
        paths = ['scenario.json','radio-service.json','sensing/observations.json']
        (output/'sensing/captures').mkdir(parents=True)
        (output/'sensing/received').mkdir()
        (output/'scenario.json').write_text(json.dumps(self.scenario.to_dict(),indent=2),encoding='utf-8')
        (output/'radio-service.json').write_text(json.dumps(self.router.radio_service_samples,indent=2),encoding='utf-8')
        for metadata in self._capture_history:
            identity=metadata['capture_id']
            (output/metadata['capture_file']).write_bytes(self._capture_bytes[identity])
            paths.append(metadata['capture_file'])
            if metadata['received_file']:
                (output/metadata['received_file']).write_bytes(self._assembled_images[identity])
                paths.append(metadata['received_file'])
        sensing = dict(mode=self.scenario.camera_mode,scene=self._scene_metadata,
                       observations=self._capture_history)
        (output/'sensing/observations.json').write_text(json.dumps(sensing,indent=2),encoding='utf-8')
        return paths
