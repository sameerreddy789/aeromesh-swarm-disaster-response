"""Physics-specific cohort budgeting derived from the resilient planner.

The matching policy is preserved. Travel uses the physics command speed and
an eight-second startup/settling allowance. This separate implementation keeps
the previously qualified kinematic sources unchanged. Budgets remain estimates;
avoidance delays and the physical battery require further qualification.
"""
import math
import networkx as nx


class PhysicsBudgetMixin:
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
                own_return_s = (math.hypot(u.position.x-f.home[0],u.position.y-f.home[1])+180.)/self.command_speed_mps
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
                return (40.-position[2]+math.dist((*position[:2],40.),goal))/self.command_speed_mps + 8.
            return math.dist(position,goal)/self.command_speed_mps + 8.

        transit, costs, squared = {}, {}, {}
        width = (1200000+1)*(len(self.flights)+1)
        for uid in candidates:
            position = self.state.uavs[uid].position.to_array()
            home = self.flights[uid].home
            owed_return_s = ((math.dist(position[:2],home[:2])+max(0.,90.-position[2])+90.)/self.command_speed_mps
                             if self.flights[uid].armed else 0.)
            for slot,goal in goals.items():
                travel_s = travel_time(uid,goal)
                eventual_return_s = (math.dist(goal[:2],home[:2])+max(0.,90.-goal[2])+90.)/self.command_speed_mps
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

