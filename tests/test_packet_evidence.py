"""Packet provenance, terminal accounting and causal multi-hop forwarding."""
import random
import pytest
from core.models import MissionState, UAV, UAVState, Vector3D, Packet, PacketType
from comm.mesh_router import MeshRouter


def packet(packet_id=1, **kwargs):
    return Packet(packet_id=packet_id, packet_type=PacketType.DATA,
                  source_id=0, dest_id=-1, **kwargs)


def network(edges=((0, 1), (1, -1)), **kwargs):
    state = MissionState()
    state.uavs = {i: UAV(id=i, position=Vector3D((i + 1) * 40, 0, 0)) for i in (0, 1, 2)}
    router = MeshRouter(state, rng=random.Random(42), **kwargs)
    for u, v in edges:
        router.network_graph.add_edge(u, v, weight=1)
        router.link_quality_cache[u, v] = {"is_connected": True, "pdr": 1.0}
        router.link_quality_cache[v, u] = {"is_connected": True, "pdr": 1.0}
    return state, router


def test_unreachable_packet_has_one_offer_and_one_drop():
    state, router = network(edges=())
    p = packet()
    assert not router.send_packet(p)
    assert not router.send_packet(p)
    assert [e["type"] for e in router.packet_events] == ["offered", "dropped"]
    assert router.packets_sent == router.packets_dropped == state.gcs.packets_dropped == 1


def test_duplicate_id_cannot_change_original_or_repeat_terminal_outcome():
    state, router = network(edges=((0, -1),))
    assert router.send_packet(packet(payload={"poi_id": 4}))
    assert not router.send_packet(packet(payload={"poi_id": 99}))
    state.time = 0.5
    router.process_packet_forwarding(0.5)
    assert not router.send_packet(packet())
    router._deliver_packet(packet())
    assert router.packets_sent == router.packets_delivered == 1
    assert router.packet_events[-1]["payload"]["poi_id"] == 4
    assert len(router.packet_events) == 2


def test_packet_never_visits_two_hops_in_one_call_even_with_existing_queues():
    state, router = network()
    router.packet_queues[1]  # pre-existing downstream queue caused the old bug
    p = packet()
    router.send_packet(p)
    state.time = 0.5
    router.process_packet_forwarding(0.5)
    assert p.current_hop == 1
    assert router.packets_delivered == 0
    state.time = 1.0
    router.process_packet_forwarding(0.5)
    assert router.packets_delivered == 1
    assert router.packet_events[-1]["latency_s"] == 1.0


def test_every_packet_queued_at_tick_start_can_advance_once():
    state, router = network()
    for i in range(50):
        router.send_packet(packet(i))
    state.time = 0.5
    router.process_packet_forwarding(0.5)
    assert len(router.packet_queues[1]) == 50
    assert router.packets_delivered == 0
    state.time = 1.0
    router.process_packet_forwarding(0.5)
    assert router.packets_delivered == 50


@pytest.mark.parametrize("removed", [False, True])
def test_failed_or_removed_current_node_drops_without_crashing(removed):
    state, router = network()
    router.send_packet(packet())
    if removed:
        del state.uavs[0]
    else:
        state.uavs[0].state = UAVState.FAILED
    router.process_packet_forwarding(0.5)
    assert router.packets_dropped == 1
    assert router.packet_events[-1]["reason"] == "current_node_unavailable"


def test_failed_source_is_offered_and_rejected_even_with_stale_route():
    state, router = network()
    state.uavs[0].state = UAVState.FAILED
    assert not router.send_packet(packet())
    assert router.packet_events[-1]["reason"] == "source_unavailable"


def test_hop_budget_counts_edges_and_ttl_allows_arrival_at_zero():
    state, router = network()
    old = router.cfg_rf.max_hops
    try:
        router.cfg_rf.max_hops = 2
        assert router.send_packet(packet(ttl=2))
        router.process_packet_forwarding(0.5)
        router.process_packet_forwarding(0.5)
        assert router.packets_delivered == 1
        router.cfg_rf.max_hops = 1
        assert not router.send_packet(packet(2))
    finally:
        router.cfg_rf.max_hops = old


def test_detection_timestamp_payload_is_frozen_at_offer():
    state, router = network(edges=((0, -1),))
    state.time = 7.0
    payload = {"poi_id": 8, "detection_time_s": 5.5, "observation": {"sequence": 12}}
    router.send_packet(packet(payload=payload))
    payload["detection_time_s"] = 999
    payload["observation"]["sequence"] = 999
    state.time = 8.5
    router.process_packet_forwarding(0.5)
    event = router.packet_events[-1]
    assert event["type"] == "delivered"
    assert event["time_s"] == 8.5
    assert event["latency_s"] == 1.5
    assert event["payload"]["detection_time_s"] == 5.5
    assert event["payload"]["observation"]["sequence"] == 12
    assert event["source_id"] == 0 and event["dest_id"] == -1


def test_explicit_hop_delay_uses_simulation_clock():
    state, router = network(edges=((0, -1),), hop_delay_s=0.25)
    router.send_packet(packet())
    router.process_packet_forwarding(0.1)
    assert router.packets_delivered == 0
    state.time = 0.25
    router.process_packet_forwarding(0.1)
    assert router.packet_events[-1]["latency_s"] == 0.25


def test_hard_range_rejects_links_even_if_radio_model_accepts_them():
    state, router = network(max_range_m=100)
    state.uavs = {0: UAV(id=0, position=Vector3D(101, 0, 0))}
    router.rf_channel.calculate_link_quality = lambda *a, **kw: {
        "is_connected": True, "etx": 1, "pdr": 1.0}
    router.update_network_topology()
    assert not router.network_graph.has_edge(0, -1)
    state.uavs[0].position.x = 100
    router.update_network_topology()
    assert router.network_graph.has_edge(0, -1)


def test_reroute_is_only_a_recovery_after_delivery():
    state, router = network(edges=((0, 1), (1, -1), (0, 2), (2, -1)))
    p = packet()
    router.send_packet(p)
    router.link_quality_cache[0, p.path[1]]["is_connected"] = False
    router.process_packet_forwarding(0.5)
    assert router.reroutes_found == 1
    assert router.successful_recoveries == 0
    router.process_packet_forwarding(0.5)
    router.process_packet_forwarding(0.5)
    assert router.successful_recoveries == 1
