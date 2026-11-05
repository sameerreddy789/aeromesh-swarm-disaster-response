"""Adversarial checks: missing observations and unreachable traffic cannot pass."""
import json
import pytest
from core.models import MissionState, UAV, Vector3D, Packet, PacketType
from comm.mesh_router import MeshRouter
from simulation.challenge_metrics import ChallengeMetricsReporter


def test_no_observations_are_unknown():
    report = ChallengeMetricsReporter(MissionState()).generate_full_report()
    assert report["communication"]["connectivity_availability_percent"] is None
    assert report["autonomy"]["fault_to_service_recovery_s"] is None
    json.dumps(report, allow_nan=False)


def test_disconnected_offered_packet_counts_as_loss():
    state = MissionState()
    state.uavs[0] = UAV(id=0, position=Vector3D(10000, 0, 50))
    router = MeshRouter(state)
    state.mesh_router = router
    packet = Packet(packet_id=0, packet_type=PacketType.DATA, source_id=0, dest_id=-1, payload_size=1024)
    assert not router.send_packet(packet)
    assert router.packets_sent == router.packets_dropped == state.gcs.packets_dropped == 1
    report = ChallengeMetricsReporter(state).generate_full_report()
    assert report["communication"]["packet_delivery_ratio_percent"] == 0


def test_outage_and_censored_recovery_are_measured():
    state = MissionState()
    state.connectivity_samples = [
        {"time_s": 0, "dt_s": 1, "operational": 2, "connected": 2},
        {"time_s": 1, "dt_s": 1, "operational": 2, "connected": 1},
        {"time_s": 2, "dt_s": 1, "operational": 2, "connected": 2},
        {"time_s": 3, "dt_s": 1, "operational": 2, "connected": 0},
    ]
    report = ChallengeMetricsReporter(state).generate_full_report()["communication"]
    assert report["connectivity_availability_percent"] == 50
    assert report["communication_downtime_s"] == 2
    assert report["completed_outage_durations_s"] == [1]
    assert report["unresolved_outage_start_s"] == 3


def test_hop_limit_counts_edges():
    state = MissionState()
    router = MeshRouter(state)
    router.network_graph.add_edges_from([(0, 1), (1, -1)])
    old_limit = router.cfg_rf.max_hops
    try:
        router.cfg_rf.max_hops = 2
        assert router._find_path(0, -1) == [0, 1, -1]
        router.cfg_rf.max_hops = 1
        assert router._find_path(0, -1) is None
    finally:
        router.cfg_rf.max_hops = old_limit
