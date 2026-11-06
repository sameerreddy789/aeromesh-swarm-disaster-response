"""Finite serialized radio service, contention, priority and loss accounting."""
import base64
import random

import pytest

from comm.mesh_router import MeshRouter
from core.models import MissionState, Packet, PacketType, UAV, UAVState, Vector3D


def network(edges=((0, -1),), *, rate=1000., limit=100000, seed=42, loss=0.):
    state = MissionState()
    state.uavs = {i: UAV(id=i, position=Vector3D(20 * (i + 1), 0, 0)) for i in range(3)}
    router = MeshRouter(state, rng=random.Random(seed), node_bandwidth_bytes_s=rate,
                        queue_limit_bytes=limit, hop_delay_s=.5)
    for a, b in edges:
        router.network_graph.add_edge(a, b, weight=1)
        router.link_quality_cache[a, b] = {'is_connected': True, 'pdr': 1 - loss}
        router.link_quality_cache[b, a] = {'is_connected': True, 'pdr': 1 - loss}
    return state, router


def packet(uid=1, *, source=0, kind='telemetry', priority=1, data='', packet_type=PacketType.DATA):
    return Packet(uid, packet_type, source, -1, payload_size=0,
                  payload={'kind': kind, 'priority': priority, 'data': data})


def tick(state, router, seconds=.5):
    state.time = round(state.time + seconds, 9)
    router.process_packet_forwarding(seconds)


def delivered(router):
    return [e for e in router.packet_events if e['type'] == 'delivered']


def test_actual_base64_payload_size_cannot_be_hidden_by_declared_size():
    state, router = network()
    raw = bytes(range(256)) * 4
    encoded = base64.b64encode(raw).decode('ascii')
    small = packet()
    image = packet(2, kind='poi_image_chunk', data=encoded)
    assert router.send_packet(small) and router.send_packet(image)
    small_wire, image_wire = [e['wire_size_bytes'] for e in router.packet_events]
    assert image_wire > len(encoded) > len(raw)
    assert image_wire > small_wire + len(encoded)
    # Later payload mutations cannot reduce admitted service or alter provenance.
    image.payload['data'] = ''
    image.payload_size = 0
    for _ in range(20):
        tick(state, router)
    image_receipt = next(e for e in delivered(router) if e['packet_id'] == 2)
    assert image_receipt['payload']['data'] == encoded
    assert router.get_network_stats()['radio_service']['node_tx_bytes'][0] == small_wire + image_wire


def test_congestion_is_limited_by_shared_receiver_and_partial_service_progresses():
    state, router = network(edges=((0, -1), (1, -1)))
    packets = [packet(i, source=i % 2, data='x' * 600) for i in range(6)]
    for p in packets:
        assert router.send_packet(p)
    tick(state, router)
    assert not delivered(router)  # each frame is larger than one tick's capacity
    assert router._node_service_bytes[-1] == pytest.approx(500)
    for _ in range(20):
        tick(state, router)
    assert len(delivered(router)) == 6
    for sample in router.radio_service_samples:
        assert all(value <= 1000 * sample['elapsed_s'] + 1e-7
                   for value in sample['node_service_bytes'].values())
    assert router._node_rx_bytes[-1] == sum(e['wire_size_bytes'] for e in delivered(router))


def test_intermediate_transmit_and_receive_share_one_half_duplex_budget():
    state, router = network(edges=((0, 1), (1, -1)))
    assert router.send_packet(packet(1, source=0, data='a' * 500))
    assert router.send_packet(packet(2, source=1, data='b' * 500))
    tick(state, router)
    assert router._node_service_bytes[1] == pytest.approx(500)
    assert router._node_rx_bytes[1] + router._node_tx_bytes[1] == pytest.approx(500)
    for _ in range(8):
        tick(state, router)
    assert len(delivered(router)) == 2
    assert all(sample['node_service_bytes'].get(1, 0) <= sample['elapsed_s'] * 1000 + 1e-7
               for sample in router.radio_service_samples)


def test_emergency_image_and_control_take_priority_over_routine_imagery_and_telemetry():
    state, router = network(rate=2000)
    for p in [packet(1, kind='poi_image_chunk', data='r' * 1000),
              packet(2, kind='telemetry', data='t' * 1000),
              packet(3, kind='poi_image_chunk', priority=5),
              packet(4, packet_type=PacketType.CONTROL)]:
        assert router.send_packet(p)
    tick(state, router)
    assert [e['packet_id'] for e in delivered(router)] == [4, 3]


def test_old_bulk_traffic_progresses_under_continued_telemetry():
    state, router = network(rate=500)
    image = packet(1, kind='poi_image_chunk', data='i' * 500)
    router.send_packet(image)
    for i in range(30):
        router.send_packet(packet(i + 2, kind='telemetry'))
        tick(state, router)
        if image.packet_id in router.delivered_packet_ids:
            break
    assert image.packet_id in router.delivered_packet_ids
    assert router.packets_delivered < router.packets_sent  # continued overload is real


def test_priority_uses_the_offered_packet_type_and_payload():
    state, router = network(rate=1000)
    image = packet(1, kind='poi_image_chunk', data='x' * 1000)
    router.send_packet(image)
    router.send_packet(packet(2))
    image.packet_type = PacketType.CONTROL
    image.payload['priority'] = 999
    tick(state, router)
    assert [e['packet_id'] for e in delivered(router)] == [2]


def test_loss_and_retry_each_consume_the_full_wire_attempt():
    state, router = network(rate=1000, loss=1.)
    p = packet()
    router.send_packet(p)
    size = router.packet_events[0]['wire_size_bytes']
    for _ in range(4):
        tick(state, router)
    stats = router.get_network_stats()['radio_service']
    assert not delivered(router)
    assert router.packets_dropped == 1
    assert stats['attempts_started'] == stats['attempts_completed'] == 4
    assert stats['node_tx_bytes'][0] == stats['node_rx_bytes'][-1] == size * 4
    assert router.packet_events[-1]['reason'] == 'retries_exhausted'


def test_queue_limit_rejects_offered_source_traffic_and_counts_it_in_pdr():
    probe = packet()
    one = MeshRouter.packet_wire_size(probe)
    state, router = network(limit=one)
    assert router.send_packet(probe)
    assert not router.send_packet(packet(2))
    assert router.packets_sent == 2 and router.packets_dropped == 1
    assert router.packet_events[-1]['reason'] == 'queue_overflow'
    tick(state, router)
    stats = router.get_network_stats()
    assert stats['pdr'] == .5
    assert stats['radio_service']['queue_bytes'][0] == 0
    assert stats['radio_service']['peak_queue_bytes'][0] == one


def test_forwarding_queue_overflow_is_a_terminal_drop_not_invisible_loss():
    p = packet(1, source=0)
    size = MeshRouter.packet_wire_size(p)
    state, router = network(edges=((0, 1), (1, -1)), rate=1000, limit=size + 8)
    assert router.send_packet(p)
    # Force the resident packet to wait, so the receiver really has no room.
    resident = packet(2, source=1)
    assert router.send_packet(resident)
    router._ready_at[2] = 10.
    tick(state, router)
    assert router.packet_events[-1]['packet_id'] == 1
    assert router.packet_events[-1]['reason'] == 'queue_overflow'
    assert router.get_network_stats()['radio_service']['queue_bytes'][0] == 0
    assert router.get_network_stats()['radio_service']['queue_bytes'][1] > 0


def test_idle_time_and_repeated_same_clock_calls_cannot_bank_service():
    state, router = network(rate=1000)
    tick(state, router, 100.)
    p = packet(data='x' * 1000)
    router.send_packet(p)
    tick(state, router)
    initial = dict(router._node_service_bytes)
    assert initial[0] == pytest.approx(500)
    for _ in range(10):
        router.process_packet_forwarding(.5)
    assert dict(router._node_service_bytes) == initial
    assert not delivered(router)


def test_frozen_partial_attempt_is_cancelled_when_current_aircraft_fails():
    state, router = network(rate=100)
    router.send_packet(packet(data='x' * 1000))
    tick(state, router)
    state.uavs[0].state = UAVState.FAILED
    tick(state, router)
    stats = router.get_network_stats()['radio_service']
    assert stats['partial_attempts_aborted'] == 1
    assert stats['attempts_completed'] == 0
    assert stats['node_tx_bytes'][0] == pytest.approx(50)
    assert stats['queue_bytes'][0] == 0


def test_finite_mode_preserves_one_hop_per_call():
    state, router = network(edges=((0, 1), (1, -1)), rate=10**9)
    router.send_packet(packet())
    tick(state, router)
    assert not delivered(router)
    tick(state, router)
    assert delivered(router)[0]['hops'] == 2
    assert delivered(router)[0]['latency_s'] == 1.


def test_replay_with_same_seed_and_queue_pressure_is_identical():
    def run():
        state, router = network(rate=1000, loss=.4, limit=4000)
        for i in range(12):
            router.send_packet(packet(i, kind='poi_image_chunk' if i % 2 else 'telemetry', data='x' * 200))
        for _ in range(50):
            tick(state, router)
        return router.packet_events, router.radio_service_samples, router.get_network_stats()
    assert run() == run()


@pytest.mark.parametrize('kwargs', [dict(node_bandwidth_bytes_s=0),
                                   dict(node_bandwidth_bytes_s=float('inf')),
                                   dict(node_bandwidth_bytes_s=True),
                                   dict(queue_limit_bytes=0), dict(queue_limit_bytes=1.5)])
def test_invalid_service_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        MeshRouter(MissionState(), **kwargs)
