"""Evidence-based metrics for the abstract simulator, not organizer log schema."""
import json
import math
from simulation.metrics import MetricsEngine


class ChallengeMetricsReporter:
    def __init__(self, mission_state):
        self.mission_state = mission_state
        self.metrics_engine = MetricsEngine(mission_state)

    def generate_full_report(self):
        state = self.mission_state
        router = getattr(state, "mesh_router", None)
        samples = getattr(state, "connectivity_samples", [])
        observed = sum(s["dt_s"] for s in samples if s["operational"])
        downtime = sum(s["dt_s"] for s in samples
                       if s["operational"] and s["connected"] < s["operational"])
        outages, start = [], None
        for sample in samples:
            disconnected = sample["operational"] and sample["connected"] < sample["operational"]
            if disconnected and start is None:
                start = sample["time_s"]
            elif not disconnected and start is not None:
                # Losing every UAV does not establish restored service.
                if sample["operational"]:
                    outages.append(sample["time_s"] - start)
                    start = None
        total_priority = sum(p.priority for p in state.pois.values())
        surveyed = state.get_surveyed_pois()
        minimum = state.min_uav_separation_observed
        return {
            "evidence_scope": "abstract simulation; connectivity uses last routing snapshot",
            "mission": {
                "coverage_percentage": state.calculate_coverage(),
                "completion_time_s": state.time if state.is_mission_complete() else None,
                "elapsed_time_s": state.time,
                "pois_surveyed": len(surveyed), "total_pois": len(state.pois),
                "priority_weighted_score": 100 * sum(p.priority for p in surveyed) / total_priority if total_priority else None,
                "all_returned_and_landed": bool(state.uavs) and all(u.is_operational() and u.position.z <= 0.5 and u.distance_to(state.gcs.position) <= 15 for u in state.uavs.values()),
            },
            "communication": {
                "packets_offered": router.packets_sent if router else None,
                "packets_delivered": router.packets_delivered if router else None,
                "packets_dropped": router.packets_dropped if router else None,
                "packets_pending": sum(map(len, router.packet_queues.values())) if router else None,
                "packet_delivery_ratio_percent": 100 * router.packets_delivered / router.packets_sent if router and router.packets_sent else None,
                "avg_end_to_end_latency_ms": 1000 * state.gcs.total_latency / state.gcs.packets_received if state.gcs.packets_received else None,
                "connectivity_availability_percent": 100 * (observed - downtime) / observed if observed else None,
                "communication_downtime_s": downtime if observed else None,
                "completed_outage_durations_s": outages,
                "unresolved_outage_start_s": start,
            },
            "autonomy": {
                "relay_dispatch_events": sum(e.get("type") in ("relay_handoff", "emergency_healing", "relay_promotion") for e in state.events),
                "fault_to_service_recovery_s": None,
                "note": "Dispatch is not arrival or restored service. Fault-correlated recovery is not instrumented.",
            },
            "safety": {
                "collision_pair_timesteps": state.collision_count,
                "min_inter_uav_separation_m": minimum if math.isfinite(minimum) else None,
                "battery_exhaustion_count": sum(int(u.battery_level <= 0) for u in state.uavs.values()),
                "geofence_entry_events": state.geofence_violation_count,
            },
            "not_measured": ["PoI detection-to-GCS reporting delay", "physical flight", "real image inference", "organizer standardized logs"],
        }

    def print_challenge_report(self):
        print(json.dumps(self.generate_full_report(), indent=2, allow_nan=False))
