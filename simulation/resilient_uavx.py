"""Scenario-driven UAV-X with protected relays and measured image transport.

Motion remains explicitly kinematic. The optional Panda3D scene supplies actual
rendered camera pixels; complete image receipts, not metadata offers, establish
report delivery. Failed aircraft never receive an exemption from strict checks.
"""
import base64
import copy
from dataclasses import replace
import hashlib
import heapq
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


def _protected_graph_score(adjacent, fleet_size, protected_nodes=None):
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
                    size = eligible = 0
                    while True:
                        member = stack.pop()
                        size += 1
                        eligible += int(protected_nodes is None or member in protected_nodes)
                        if member == other:
                            break
                    if node == 0 and size >= 2:
                        protected += eligible
            else:
                low[node] = min(low[node], discovered[other])

    visit(0, -1)
    return (visited - 1) * (fleet_size + 1) + protected


class ResilientUAVXSimulator(UAVXSimulator):
    schema_version = 2
    CHUNK_BYTES = 512
    TASK_HEADROOM_S = 180.

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
        self._landing_yields = {}
        self._withdrawal_reason = None
        self._idle_since_s = None
        self._next_assignment_s = 0.
        self._hold_ready_s = None
        self._hold_signature = None
        root = Path(__file__).resolve().parents[1]
        for name in ('simulation/resilient_uavx.py', 'simulation/uavx_scenario.py', 'simulation/camera_sensor.py'):
            self.source_sha256[name] = hashlib.sha256((root/name).read_bytes()).hexdigest()

    def _charging_allowed(self, uid):
        return uid in self._charge_admission

    def _transit_speed(self, uid):
        # Near relays and replacements must close gaps promptly in this
        # protected formation. Every movement still passes the base safety
        # and network admission checks, including the 5 m/s hard limit.
        return 5.

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

    def _withdrawal_path_s(self, positions=None):
        """Common retreat path estimate, including climb and home descent.

        Every participating relay may have to serve the most distant aircraft
        before landing. Use that common obligation rather than only the inner
        relay's short distance home. The additional energy reserve is applied
        separately; this is a planning estimate, not measured flight evidence.
        """
        points = list(positions or ())
        points.extend(tuple(u.position.to_array()) for uid,u in self.state.uavs.items()
                      if self.flights[uid].armed and u.is_operational())
        if not points:
            return 0.
        distance = max(math.hypot(p[0]-f.home[0],p[1]-f.home[1])
                       for p in points for f in self.flights.values())
        return distance / 5. + 36.

    def _budgeted_assignment(self, task, preserve_owners=False):
        """Find a complete injective assignment with a shared return reserve.

        First select feasible cohorts by added fleet flight time, then uncross
        deployment by minimizing squared travel and favor shared energy slack.
        Common deployment bounds
        enforce selected and retiring aircraft budgets during matching, so a
        bad cheapest assignment cannot hide a different feasible cohort.
        No ownership or flight clock changes during this search.
        """
        for uid,flight in self.flights.items():
            uav = self.state.uavs[uid]
            if uav.is_operational() and not all(math.isfinite(value) for value in (
                    *uav.position.to_array(), flight.remaining_s, flight.sortie_s)):
                # An unselected aircraft still owes a measured safe return;
                # unknown energy/position cannot silently waive that budget.
                return None
        goals = self._build_slots(task)
        work_s = max(0.,30.-task.dwell_s) + 10.
        retreat_s = self._withdrawal_path_s(goals.values())
        common_energy_s = work_s + retreat_s + 180.
        deadline_s = 2700.-self.state.time-work_s-retreat_s-30.
        if deadline_s < 0.:
            return None
        candidates = {}
        retirement_caps = {}
        graph = self.router.network_graph
        connected = nx.node_connected_component(graph,-1) if -1 in graph else set()
        for uid,f in self.flights.items():
            u = self.state.uavs[uid]
            available_s = min(f.remaining_s,1200.-f.sortie_s)
            if f.armed and not f.returning and u.is_operational():
                own_return_s = (math.hypot(u.position.x-f.home[0],u.position.y-f.home[1])+180.)/5.
                retirement_caps[uid] = available_s-own_return_s-180.
            if (not u.is_operational() or f.returning or f.charge_started_s is not None
                    or (f.armed and uid not in connected)):
                continue
            if not math.isfinite(available_s) or available_s < common_energy_s:
                continue
            candidates[uid] = available_s
        if len(candidates) < len(goals):
            return None

        def travel_time(uid, goal):
            position = self.state.uavs[uid].position.to_array()
            if position[2]<39. and math.dist(position[:2],goal[:2])>5.:
                return (40.-position[2]+math.dist((*position[:2],40.),goal))/5.
            return math.dist(position,goal)/5.

        transit, costs, squared = {}, {}, {}
        width = (1200000+1)*(len(self.flights)+1)
        for uid in candidates:
            position = self.state.uavs[uid].position.to_array()
            home = self.flights[uid].home
            owed_return_s = ((math.dist(position[:2],home[:2])+max(0.,90.-position[2])+90.)/5.
                             if self.flights[uid].armed else 0.)
            for slot,goal in goals.items():
                travel_s = travel_time(uid,goal)
                eventual_return_s = (math.dist(goal[:2],home[:2])+max(0.,90.-goal[2])+90.)/5.
                added_ms = round((travel_s+eventual_return_s-owed_return_s)*1000)
                transit[uid,slot] = travel_s
                costs[uid,slot] = ((-added_ms*width+round(candidates[uid]*1000))
                                   *(len(self.flights)+1)+int(self.slot_owners.get(slot)==uid))
                # For a straight chain, this convex cost assigns aircraft in
                # projected spatial order. Linear distance has near ties that
                # can place an outer owner behind an already settled relay.
                squared[uid,slot] = -round(math.dist(position,goal)**2*1000)

        # An in-progress handoff is already a commitment. Retain it where
        # possible; only reclaim that aircraft if a full mission cannot be
        # assigned without it.
        handoffs = {slot:uid for slot,uid in self.replacements.items()
                    if slot in goals and uid in candidates
                    and self._eligible(uid,(*goals[slot][:2],65.))}
        handoff_transit = {uid:travel_time(uid,(*goals[slot][:2],65.))
                           for slot,uid in handoffs.items()}
        # A feasible cohort has a nonempty interval from its longest travel to
        # its tightest selected/retiring/deadline cap. Enumerating those upper
        # endpoints is complete under this declared planning budget.
        bounds = sorted({min(deadline_s,cap) for cap in
                         [deadline_s,*(v-common_energy_s for v in candidates.values()),
                          *retirement_caps.values()] if math.isfinite(cap) and cap>=0.})

        def match(allowed, bound, objective, required=()):
            mandatory = set(required)
            if not mandatory<=allowed:
                return None
            edges = [(uid,slot) for uid in sorted(allowed) for slot in goals
                     if transit[uid,slot]<=bound+1e-8
                     and (not preserve_owners or self.slot_owners.get(slot) not in allowed
                          or uid==self.slot_owners[slot])]
            if not edges:
                return None
            values = [objective[pair] for pair in edges]
            bonus = (max(values)-min(values))*len(goals)+1
            graph = nx.Graph()
            graph.add_nodes_from(('slot',slot) for slot in goals)
            for uid,slot in edges:
                graph.add_edge(('uav',uid),('slot',slot),
                               weight=objective[uid,slot]+bonus*int(uid in mandatory))
            pairs = nx.max_weight_matching(graph,maxcardinality=True)
            assignment = {}
            for left,right in pairs:
                if left[0]=='slot': left,right=right,left
                assignment[right[1]]=left[1]
            if len(assignment)!=len(goals) or not mandatory<=set(assignment.values()):
                return None
            return assignment

        for reclaim_handoffs in (False,True):
            best = None
            for bound in bounds:
                allowed = {uid for uid,energy in candidates.items()
                           if energy+1e-8>=common_energy_s+bound}
                if not reclaim_handoffs:
                    allowed -= set(handoffs.values())
                if len(allowed)<len(goals):
                    continue
                safe_handoffs = {slot:uid for slot,uid in handoffs.items()
                                 if candidates[uid]+1e-8>=common_energy_s+bound
                                 and handoff_transit[uid]<=bound+1e-8}
                mandatory = {uid for uid,cap in retirement_caps.items() if cap<bound-1e-8}
                mandatory -= set(safe_handoffs.values())
                assignment = match(allowed,bound,costs,mandatory)
                if assignment is None:
                    continue
                # Keep the chosen energy cohort, but remove crossing role
                # order before movement. All new edges obey the same bound.
                assignment = match(set(assignment.values()),bound,squared)
                if assignment is None:
                    continue
                retained_handoffs = {slot:uid for slot,uid in handoffs.items()
                                     if slot in safe_handoffs and uid not in assignment.values()}
                deploy_s = max([transit[uid,slot] for slot,uid in assignment.items()]
                               + [handoff_transit[uid] for uid in retained_handoffs.values()])
                retained = set(assignment.values()) | set(retained_handoffs.values())
                if (any(candidates[uid]+1e-8<deploy_s+common_energy_s for uid in retained)
                        or deploy_s>deadline_s+1e-8
                        or any(cap+1e-8<deploy_s for uid,cap in retirement_caps.items() if uid not in retained)):
                    continue
                # A slightly shorter deployment is not useful if its oldest
                # owner immediately forces the whole formation to withdraw.
                # Prefer up to three minutes of headroom for later announced
                # work. Beyond that, reuse beats churning healthy aircraft for
                # a fresh pad cohort merely to maximize an unused battery.
                slack_s = min(candidates[uid] for uid in retained)-deploy_s-common_energy_s
                score = (min(self.TASK_HEADROOM_S,slack_s), sum(costs[uid,slot] for slot,uid in assignment.items()),
                         sum(squared[uid,slot] for slot,uid in assignment.items()),-deploy_s)
                if best is None or score>best[0]:
                    best = (score,dict(goals=goals,owners=assignment,handoffs=retained_handoffs,
                                       deployment_s=deploy_s,withdrawal_path_s=retreat_s,
                                       required_energy_s=deploy_s+common_energy_s))
            if best is not None:
                return best[1]
        if preserve_owners:
            return self._budgeted_assignment(task,preserve_owners=False)
        return None

    def _commit_formation(self, task, assignment):
        """Commit targets only; measured motion and sensing establish results."""
        old_active = self.active_id
        previous = dict(self.slot_owners)
        self.slots = assignment['goals']
        # Matching returns an unordered set of pairs. Later handoff dispatch
        # consumes spare aircraft in owner order, so canonicalize by slot to
        # make a seeded mission independent of Python's process hash seed.
        self.slot_owners = dict(sorted(assignment['owners'].items()))
        self.replacements = dict(sorted(assignment['handoffs'].items()))
        retained = set(self.slot_owners.values()) | set(self.replacements.values())
        for uid,f in self.flights.items():
            f.slot = None
            if uid not in retained and not f.returning:
                self._return(uid,'formation_energy_reassignment')
        for slot,uid in self.slot_owners.items():
            f,u = self.flights[uid],self.state.uavs[uid]
            f.slot = slot
            u.target_position = Vector3D(*self.slots[slot])
            u.role = UAVRole.SCOUT if slot==max(self.slots) else UAVRole.RELAY
            if previous.get(slot)!=uid:
                self.event('role_assigned',uav_id=uid,role=u.role.value,slot=slot)
        for slot,uid in self.replacements.items():
            self.flights[uid].slot = slot
            self.state.uavs[uid].target_position = Vector3D(*self.slots[slot][:2],65.)
            if task.id!=old_active:
                self.event('relay_handoff_retargeted',uav_id=uid,slot=slot)
        self.active_id = task.id
        self._idle_since_s = None
        self._hold_ready_s = self._hold_signature = None
        self._next_assignment_s = self.state.time + 5.
        self.event('formation_committed',poi_id=task.id,owners=dict(self.slot_owners),
                   deployment_budget_s=assignment['deployment_s'],
                   withdrawal_path_budget_s=assignment['withdrawal_path_s'],
                   required_energy_s=assignment['required_energy_s'])
        if task.id!=old_active:
            self.event('task_selected',poi_id=task.id,priority=task.priority,required_slots=len(self.slots))

    def _start_connected_withdrawal(self, reason):
        """Withdraw the entire active group before emergency reserves expire."""
        active = [uid for uid,f in self.flights.items()
                  if self.state.uavs[uid].is_operational() and (f.armed or not f.landed)]
        if self._withdrawal_reason is None and active:
            self._withdrawal_reason = reason
            self.event('connected_withdrawal_started',reason=reason,
                       path_budget_s=self._withdrawal_path_s(),
                       uav_ids=active)
        # Clear reservations together so release_slot cannot promote a relay
        # into a formation which is already withdrawing.
        self.slot_owners.clear()
        self.replacements.clear()
        self.slots.clear()
        for uid,f in self.flights.items():
            f.slot = None
            self._return(uid,reason)
        self._idle_since_s = None
        self._hold_ready_s = self._hold_signature = None

    def _plan_formation(self):
        now = self.state.time
        if self._withdrawal_reason is not None:
            pending = [uid for uid,f in self.flights.items()
                       if self.state.uavs[uid].is_operational() and (f.armed or not f.landed)]
            if pending:
                self._start_connected_withdrawal(self._withdrawal_reason)
                return
            self.event('connected_withdrawal_landed',reason=self._withdrawal_reason)
            self._withdrawal_reason = None
            self._next_assignment_s = now
        remaining = [p for p in self.tasks if not (p.surveyed and p.report_s is not None)]
        if not remaining:
            self.shutdown = True
            self._start_connected_withdrawal('mission_return')
            return
        if now + self._withdrawal_path_s() + 30. >= 2700.:
            self.shutdown = True
            self._start_connected_withdrawal('mission_deadline_return')
            return
        # Reconcile actual failures first, retaining the base promotion rules.
        for uid,f in list(self.flights.items()):
            if f.slot is not None and (f.returning or not self.state.uavs[uid].is_operational()):
                self._release_slot(uid)
        available = [p for p in remaining if p.spawn_s<=now]
        if not available:
            self._complete_idle_handoffs()
            if self._idle_since_s is None: self._idle_since_s = now
            # Holding a connected formation briefly avoids a return/relaunch
            # cycle that costs more flight time than the announcement gap.
            # The bound uses measured geometry, never future task times.
            grace_s = max(30.,2.*max(0.,self._withdrawal_path_s()-36.)+30.)
            # Idle time must not spend the allowance needed for another
            # survey and useful deployment. Otherwise a long announcement
            # gap leaves only return energy and delays the next charge cycle.
            # This uses current geometry, not future task times or positions.
            hard_reserve_s = self._withdrawal_path_s()+180.
            reserve_s = hard_reserve_s+40.+self.TASK_HEADROOM_S
            energies = [min(f.remaining_s,1200.-f.sortie_s)
                        for uid,f in self.flights.items()
                        if f.armed and not f.returning and self.state.uavs[uid].is_operational()]
            hard_low = any(energy<=hard_reserve_s for energy in energies)
            readiness_low = any(energy<=reserve_s for energy in energies)
            idle_s = now-self._idle_since_s
            if hard_low or (idle_s>=30. and readiness_low) or idle_s>=grace_s:
                self._start_connected_withdrawal('waiting_for_announced_tasks')
            return
        self._idle_since_s = None
        current = next((p for p in available if p.id==self.active_id),None)
        task = self._select_task(available,current)
        goals = self._build_slots(task)
        missing = set(goals)-set(self.slot_owners)
        common_floor = self._withdrawal_path_s(goals.values())+180.+max(0.,30.-task.dwell_s)+10.
        low = any(min(self.flights[uid].remaining_s,1200.-self.flights[uid].sortie_s)<=common_floor
                  for uid in self.slot_owners.values())
        needs_assignment = task.id!=self.active_id or bool(missing) or low
        if needs_assignment:
            if not self.slot_owners and now<self._next_assignment_s:
                return
            assignment = self._budgeted_assignment(task,preserve_owners=bool(missing) and not low and task.id==self.active_id)
            if assignment is None and task is not current:
                # Proximity alone can strand a ready small cohort while the
                # preferred task waits for more chargers. Try other announced
                # work at the same priority before waiting or withdrawing.
                # A current unfinished task retains equal-priority authority.
                alternatives = [p for p in available
                                if p.id!=task.id and p.priority==task.priority]
                while alternatives:
                    candidate = self._select_task(alternatives,None)
                    alternatives.remove(candidate)
                    proposal = self._budgeted_assignment(candidate)
                    if proposal is not None:
                        self.event('task_admission_alternative',preferred_poi_id=task.id,
                                   selected_poi_id=candidate.id,priority=candidate.priority)
                        task,assignment = candidate,proposal
                        break
            self._next_assignment_s = now+5.
            if assignment is None:
                self._complete_idle_handoffs()
                if self._can_hold_formation(common_floor):
                    # A newly announced task may need more charged aircraft.
                    # Keep the healthy existing assignment progressing while
                    # admission waits; do not launch a partial new formation
                    # or force a needless landing/recharge cycle.
                    active = next((p for p in self.tasks if p.id==self.active_id),None)
                    if active is not None and not (active.surveyed and active.report_s is not None):
                        return
                    signature = (task.id,tuple(sorted(p.id for p in available)),
                                 tuple(uid for uid,uav in self.state.uavs.items() if uav.is_operational()))
                    if self._hold_signature==signature and self._hold_ready_s is not None and now<self._hold_ready_s:
                        return
                    ready_s = self._forecast_charge_admission(task)
                    if ready_s is not None:
                        self._hold_signature,self._hold_ready_s = signature,ready_s
                        self.event('formation_waiting_for_charge',poi_id=task.id,
                                   expected_ready_s=ready_s,forecast_only=True)
                        return
                if any(f.armed and self.state.uavs[uid].is_operational() for uid,f in self.flights.items()):
                    self._start_connected_withdrawal('formation_admission_failed')
                return
            self._commit_formation(task,assignment)
        super()._plan()

    def _forecast_charge_admission(self, task):
        """Predict whether waiting for known chargers leaves a useful cohort.

        This decides hold versus return only. Isolated flight records and a
        copied planning clock never grant charge, mission or arrival credit;
        real admission still requires measured readiness on a later step.
        Returning-aircraft arrivals are not assumed by this forecast.
        """
        queued = sorted((f.charge_started_s,uid) for uid,f in self.flights.items()
                        if self._charge_eligible(uid))
        if not queued:
            return None
        servers = [0]*(self.scenario.charging_pads or len(self.flights))
        horizon_steps = max(0,math.ceil((2700.-self.state.time)/self.dt)-1)
        ready = {}
        for _,uid in queued:
            energy = self.flights[uid].remaining_s
            begin = heapq.heappop(servers)
            finish = horizon_steps+1
            # Repeat the actual floating-point step arithmetic. Rounding a
            # continuous duration can predict completion one interval early.
            # The mission horizon bounds work even for very slow chargers.
            for step in range(begin+1,horizon_steps+1):
                energy = self._charge_after_step(energy)
                if energy>=1200.:
                    finish = step
                    ready[uid] = step*self.dt
                    break
            heapq.heappush(servers,finish)
        next_roles = max((len(self._build_slots(p)) for p in self.tasks
                          if p.id!=task.id and p.spawn_s<=self.state.time
                          and not (p.surveyed and p.report_s is not None)),default=0)
        reserve_s = self._withdrawal_path_s(self._build_slots(task).values())+180.+max(0.,30.-task.dwell_s)+10.
        for wait_s in sorted(set(ready.values())):
            if self.state.time+wait_s>=2700.:
                break
            future = copy.copy(self)
            future.state = copy.copy(self.state)
            future.state.time += wait_s
            future.flights = {uid:replace(f) for uid,f in self.flights.items()}
            for uid,flight in future.flights.items():
                if flight.armed:
                    flight.remaining_s -= wait_s
                    flight.sortie_s += wait_s
                if uid in ready and ready[uid]<=wait_s:
                    flight.remaining_s,flight.sortie_s = 1200.,0.
                    flight.charge_started_s = None
            if not future._can_hold_formation(reserve_s):
                break
            proposal = future._budgeted_assignment(task)
            if proposal is not None:
                retained = set(proposal['owners'].values()) | set(proposal['handoffs'].values())
                # A smaller follow-up task needs a capable subset, not extra
                # endurance on every owner of the current larger formation.
                margins = sorted((min(future.flights[uid].remaining_s,1200.-future.flights[uid].sortie_s)
                                  -proposal['required_energy_s'] for uid in retained),reverse=True)
                needed = min(next_roles,len(margins))
                if not needed or margins[needed-1]>=self.TASK_HEADROOM_S:
                    return future.state.time
        return None

    def _can_hold_formation(self, reserve_s):
        """An existing connected cohort may wait only above its shared reserve."""
        if (not math.isfinite(reserve_s) or not self.slots
                or set(self.slot_owners)!=set(self.slots)):
            return False
        for uid,flight in self.flights.items():
            uav = self.state.uavs[uid]
            if flight.armed and uav.is_operational():
                if not all(math.isfinite(value) for value in (
                        *uav.position.to_array(),flight.remaining_s,flight.sortie_s)):
                    return False
                # Early hold returns also bypass the base planner for
                # replacements and unassigned aircraft; protect them too.
                if (not flight.returning
                        and min(flight.remaining_s,1200.-flight.sortie_s)<=reserve_s):
                    return False
        for slot,uid in self.slot_owners.items():
            flight,uav = self.flights[uid],self.state.uavs[uid]
            if (not uav.is_operational() or not flight.armed or flight.returning
                    or flight.slot!=slot or not uav.connected_to_gcs
                    or not all(math.isfinite(value) for value in (
                        *uav.position.to_array(),flight.remaining_s,flight.sortie_s))
                    or min(flight.remaining_s,1200.-flight.sortie_s)<=reserve_s):
                return False
        return True

    def _complete_idle_handoffs(self):
        """Finish already committed, observed handoffs without selecting future work.

        The base planner returns the fleet when no task is announced. The
        bounded formation hold intentionally overrides that behavior, but it
        must still retire an incumbent whose replacement has actually arrived.
        No dispatch, time, charge or predicted-arrival credit is introduced.
        """
        for slot,new in list(self.replacements.items()):
            old=self.slot_owners.get(slot)
            if old is None or old==new or slot not in self.slots:
                continue
            replacement=self.state.uavs[new]
            flight=self.flights[new]
            goal=Vector3D(*self.slots[slot][:2],65.)
            if (not replacement.is_operational() or not flight.armed
                    or flight.returning or flight.slot!=slot
                    or not replacement.connected_to_gcs
                    or not replacement.distance_to(goal)<=3.):
                continue
            self._return(old,'measured_relay_handoff')
            self.slot_owners[slot]=new
            self.replacements.pop(slot,None)
            replacement.target_position=Vector3D(*self.slots[slot])
            replacement.role=UAVRole.SCOUT if slot==max(self.slots) else UAVRole.RELAY
            self.event('relay_handoff_completed',old_uav_id=old,new_uav_id=new,slot=slot)

    def _move(self):
        # Admission is frozen for this interval. A pad released during the loop
        # becomes available next interval, never supplies two full dt charges.
        waiting = sorted((f.charge_started_s, i) for i,f in self.flights.items()
                         if self._charge_eligible(i))
        count = self.scenario.charging_pads or len(self.flights)
        self._charge_admission = {i for _,i in waiting[:count]}
        restored = self._clear_landing_corridors()
        try:
            super()._move()
        finally:
            for uid,(target,slot) in restored.items():
                # Slot identity must survive a failure during the temporary
                # yield so the next planner pass can release its ownership.
                self.flights[uid].slot = slot
                if self.state.uavs[uid].is_operational():
                    self.state.uavs[uid].target_position = target

    def _clear_landing_corridors(self):
        """Let returning vehicles descend through their own home-pad column.

        A relay hovering above a home pad can otherwise indefinitely obstruct
        landing. It temporarily yields sideways; ordinary speed, separation,
        geofence and network admission still govern every measured movement.
        """
        homes = {uid:f.home for uid,f in self.flights.items()
                 if f.returning and f.armed and self.state.uavs[uid].is_operational()
                 and math.dist(self.state.uavs[uid].position.to_array()[:2],f.home[:2])<80.}
        restored = {}
        active_yields = {}
        for uid,u in self.state.uavs.items():
            flight = self.flights[uid]
            if not flight.armed or flight.returning or not u.is_operational() or u.target_position is None:
                continue
            position = u.position.to_array()
            target = u.target_position.to_array()
            blocked = [rid for rid,home in homes.items()
                       if min(math.dist(position[:2],home[:2]),math.dist(target[:2],home[:2]))<30.]
            if not blocked:
                continue
            chosen = self._landing_yields.get(uid)
            if chosen is None or any(math.dist(chosen[:2],h[:2])<30. for h in homes.values()):
                candidates = []
                for radius in (35.,55.,75.):
                    for angle in np.linspace(0.,2*math.pi,16,endpoint=False):
                        point = (position[0]+radius*math.cos(angle),position[1]+radius*math.sin(angle),max(40.,position[2]))
                        if not self._inside(point) or any(math.dist(point[:2],h[:2])<30. for h in homes.values()):
                            continue
                        if any(math.dist(point,v.position.to_array())<25. for i,v in self.state.uavs.items() if i!=uid):
                            continue
                        candidates.append((radius,math.dist(point,target),point))
                    if candidates:
                        break
                if not candidates:
                    continue
                chosen = min(candidates)[2]
                self.event('landing_corridor_yield',uav_id=uid,home_uavs=blocked,target=list(chosen))
            active_yields[uid] = chosen
            restored[uid] = (u.target_position,flight.slot)
            u.target_position = Vector3D(*chosen)
            # Yield uses the normal 5 m/s cap rather than slow radial-slot
            # deployment pacing, while retaining all swept movement vetoes.
            flight.slot = None
        self._landing_yields = active_yields
        return restored

    def _reachable_count(self, positions, reference_positions=None):
        if not self.scenario.relay_protection:
            return super()._reachable_count(positions, reference_positions)
        excluded, blocked_edges = self._radio_exclusions()
        live = [i for i,u in self.state.uavs.items() if u.is_operational() and i not in excluded]
        ids = [-1, *live]
        points = np.vstack([np.zeros(3), positions[live]])
        adjacent = np.linalg.norm(points[:,None,:]-points[None,:,:], axis=2) <= 100+1e-8
        conservative = None
        if reference_positions is not None:
            reference = np.vstack([np.zeros(3), reference_positions[live]])
            # Continuous primary connectivity still uses edges retained across
            # the entire interval. Backup links may hand over to new neighbors;
            # requiring an identical backup graph can stall a changing swarm.
            conservative = adjacent & (np.linalg.norm(reference[:,None,:]-reference[None,:,:], axis=2) <= 100+1e-8)
        indices = {uid:index for index,uid in enumerate(ids)}
        for first,second in blocked_edges:
            if first in indices and second in indices:
                a,b = indices[first],indices[second]
                adjacent[a,b] = adjacent[b,a] = False
                if conservative is not None:
                    conservative[a,b] = conservative[b,a] = False
        # This hook is only the motion admission score, never an availability
        # metric. Connectivity remains dominant; within a connected component
        # retain redundant paths during deployments, replacements and pivots.
        # Retain backup paths for mission-carrying aircraft. A returning
        # aircraft still needs the continuous primary path, but requiring it
        # to keep two paths can delay recovery and consume its return reserve.
        mission_nodes = {indices[uid] for uid in live
                         if not self.flights[uid].returning
                         and (self.flights[uid].armed or self.state.uavs[uid].target_position is not None)}
        endpoint_score = _protected_graph_score(adjacent, len(self.flights), mission_nodes)
        if conservative is None:
            return endpoint_score
        reached = np.zeros(len(ids), dtype=bool)
        reached[0] = True
        while True:
            expanded = reached | np.any(conservative[reached], axis=0)
            if np.array_equal(expanded, reached):
                break
            reached = expanded
        width = len(self.flights)+1
        return (int(reached.sum())-1)*width + endpoint_score % width

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
        self._plan_formation()
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
