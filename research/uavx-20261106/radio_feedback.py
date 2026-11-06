"""Research-only transported-feedback/parity experiment; no production edits.

The receiver's one-shot NACK follows arrival of the final original data chunk.
That is an optimistic batch-boundary detector, not a deployed loss detector.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
from datetime import datetime, timezone
import hashlib
import itertools
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from comm.mesh_router import MeshRouter
from core.config import RFConfig
from core.models import MissionState, Packet, PacketType, UAV, Vector3D

RATE, QUEUE, CHUNK, SIZE = 131072.0, 1048576, 512, 32768
DEADLINE, HORIZON = 10.0, 30.0
PROFILES = ("none", "one_first_offer_erasure", "two_first_offer_erasures",
            "middle_link_burst_2_to_5", "all_links_burst_2_to_5")
POLICIES = ("single_with_ack", "full_retry_until_ack", "one_nack_repair", "xor1_with_ack")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ScriptedChannel:
    """Sample loss at whole-attempt completion, with specified correlation."""
    def __init__(self, state, source, middle_edge, profile):
        self.state, self.source = state, source
        self.middle_edge, self.profile = set(middle_edge), profile
        self.packet = self.edge = None
        self.forced_losses = 0

    def random(self):
        payload = self.packet.payload
        loss = False
        if "first_offer_erasure" in self.profile:
            indices = (32, 33) if self.profile.startswith("two") else (32,)
            loss = (self.edge[0] == self.source and payload.get("batch_id") == 0
                    and payload.get("chunk_index") in indices)
        elif "burst" in self.profile and 2.0 <= self.state.time < 5.0:
            loss = self.profile.startswith("all") or set(self.edge) == self.middle_edge
        if loss:
            self.forced_losses += 1
        return 1.0 if loss else 0.5


def one_case(dt, hops, profile, policy):
    state = MissionState()
    state.uavs = {i: UAV(i, position=Vector3D(0, 0, 0)) for i in range(30)}
    for i in range(hops):
        state.uavs[i].position = Vector3D(min(i + 1, 10) * 90,
                                         max(0, i + 1 - 10) * 90, 40)
    path, source = [-1, *range(hops)], hops - 1
    middle = (path[hops // 2], path[hops // 2 + 1])
    channel = ScriptedChannel(state, source, middle, profile)
    router = MeshRouter(state, max_range_m=100, rng=channel, hop_delay_s=dt,
                        node_bandwidth_bytes_s=RATE, queue_limit_bytes=QUEUE)
    router.cfg_rf = RFConfig(max_hops=32)
    router.network_graph.add_nodes_from([-1, *range(30)])
    for a, b in zip(path, path[1:]):
        router.network_graph.add_edge(a, b, weight=1.)
        router.link_quality_cache[a, b] = dict(is_connected=True, pdr=1., etx=1.)
        router.link_quality_cache[b, a] = dict(is_connected=True, pdr=1., etx=1.)
    original_service = router._service_attempt
    def observe_attempt(packet, node, next_node, available_at, interval_start, sample):
        channel.packet, channel.edge = packet, (node, next_node)
        return original_service(packet, node, next_node, available_at, interval_start, sample)
    router._service_attempt = observe_attempt

    raw = hashlib.shake_256(b"AeroMesh isolated feedback fixture 20261106").digest(SIZE)
    digest = hashlib.sha256(raw).hexdigest()
    chunks = [raw[i:i + CHUNK] for i in range(0, SIZE, CHUNK)]
    count = len(chunks)
    parity = bytes(CHUNK)
    for chunk in chunks:
        parity = bytes(a ^ b for a, b in zip(parity, chunk))
    received, parity_received = {}, None
    complete, ack_received, nack_received = None, None, None
    recovered_index, nack_sent = None, False
    next_retry, batches, cursor = 2.0, 0, 0
    acked_batches, controls, losses = set(), [], Counter()
    max_service_fraction, deadline_snapshot = 0., None

    def offer_data(indices, with_parity=False):
        nonlocal batches
        batch = batches
        batches += 1
        for index in indices:
            payload = dict(kind="poi_image_chunk", priority=1, capture_id="feedback-frame",
                           image_sha256=digest, raw_bytes=SIZE, chunk_count=count,
                           chunk_index=index, batch_id=batch,
                           data_b64=base64.b64encode(chunks[index]).decode("ascii"))
            router.send_packet(Packet(router.packets_sent, PacketType.DATA, source, -1,
                                      ttl=32, payload=payload, payload_size=0))
        if with_parity:
            payload = dict(kind="poi_image_chunk", priority=1, capture_id="feedback-frame",
                           image_sha256=digest, raw_bytes=SIZE, chunk_count=count,
                           chunk_index=-1, batch_id=batch, coding="xor-all-data-once",
                           data_b64=base64.b64encode(parity).decode("ascii"))
            router.send_packet(Packet(router.packets_sent, PacketType.DATA, source, -1,
                                      ttl=32, payload=payload, payload_size=0))

    def offer_control(kind, missing=None, batch=None):
        payload = dict(kind=kind, capture_id="feedback-frame", image_sha256=digest)
        if missing is not None:
            payload["missing_chunks"] = missing
        if batch is not None:
            payload["batch_id"] = batch
        router.send_packet(Packet(router.packets_sent, PacketType.CONTROL, -1, source,
                                  ttl=32, payload=payload, payload_size=0))

    def snapshot():
        events = router.packet_events
        offered = [e for e in events if e["type"] == "offered"]
        delivered = [e for e in events if e["type"] == "delivered"]
        is_control = lambda e: e["payload"]["kind"] in ("image_ack", "image_nack")
        return dict(at_s=state.time, complete_image_s=complete, source_ack_received_s=ack_received,
                    received_unique_data_chunks=len(received), recovered_chunk_index=recovered_index,
                    logical_offers=router.packets_sent, deliveries=router.packets_delivered,
                    drops=router.packets_dropped, pending=sum(map(len, router.packet_queues.values())),
                    packet_pdr=router.get_network_stats()["pdr"],
                    offered_wire_bytes=sum(e["wire_size_bytes"] for e in offered),
                    offered_control_packets=sum(is_control(e) for e in offered),
                    offered_control_wire_bytes=sum(e["wire_size_bytes"] for e in offered if is_control(e)),
                    delivered_control_packets=sum(is_control(e) for e in delivered),
                    service_tx_bytes=sum(router._node_tx_bytes.values()),
                    service_tx_plus_rx_bytes=sum(router._node_service_bytes.values()),
                    successful_hops=router.total_hops, attempts_completed=router.radio_completed_attempts,
                    queue_overflows=router.radio_queue_overflows)

    offer_data(range(count), with_parity=policy == "xor1_with_ack")
    for tick in range(1, round(HORIZON / dt) + 1):
        state.time = round(tick * dt, 9)
        router.process_packet_forwarding(dt)
        new_events = router.packet_events[cursor:]
        cursor = len(router.packet_events)
        for event in new_events:
            payload, kind = event["payload"], event["payload"]["kind"]
            if kind in ("image_ack", "image_nack"):
                controls.append(dict(time_s=event["time_s"], type=event["type"],
                                     kind=kind, packet_id=event["packet_id"],
                                     wire_bytes=event["wire_size_bytes"],
                                     missing_chunks=payload.get("missing_chunks"),
                                     reason=event.get("reason")))
            if event["type"] == "dropped":
                losses[event["reason"]] += 1
            if event["type"] != "delivered":
                continue
            if kind == "image_ack":
                if ack_received is None:
                    ack_received = state.time
                continue
            if kind == "image_nack":
                if nack_received is None:
                    nack_received = state.time
                    offer_data(payload["missing_chunks"])
                continue
            assert payload["image_sha256"] == digest
            decoded = base64.b64decode(payload["data_b64"], validate=True)
            index = payload["chunk_index"]
            if index == -1:
                parity_received = decoded
            else:
                assert index not in received or received[index] == decoded
                received[index] = decoded
            if complete is None and parity_received is not None and len(received) == count - 1:
                missing_index = next(i for i in range(count) if i not in received)
                decoded_missing = parity_received
                for chunk in received.values():
                    decoded_missing = bytes(a ^ b for a, b in zip(decoded_missing, chunk))
                received[missing_index], recovered_index = decoded_missing, missing_index
            just_completed = complete is None and len(received) == count
            if just_completed:
                assembled = b"".join(received[i] for i in range(count))
                assert len(assembled) == SIZE and hashlib.sha256(assembled).hexdigest() == digest
                complete = state.time
            # A duplicate batch's final data chunk re-ACKs a completed capture.
            if complete is not None and (just_completed or index == count - 1):
                batch = payload["batch_id"]
                if batch not in acked_batches:
                    offer_control("image_ack", batch=batch)
                    acked_batches.add(batch)
            if (policy == "one_nack_repair" and not nack_sent and complete is None
                    and index == count - 1 and payload["batch_id"] == 0):
                offer_control("image_nack", missing=[i for i in range(count) if i not in received])
                nack_sent = True
        sample = router.radio_service_samples[-1]
        max_service_fraction = max(max_service_fraction, max(
            sample["node_service_bytes"].values(), default=0.) / (RATE * sample["elapsed_s"]))
        if policy == "full_retry_until_ack" and ack_received is None and state.time >= next_retry - 1e-9:
            offer_data(range(count))
            next_retry += 2.
        if abs(state.time - DEADLINE) < 1e-8:
            deadline_snapshot = snapshot()
    # Include control offers generated at the final tick in the diagnostic log.
    for event in router.packet_events[cursor:]:
        if event["payload"]["kind"] in ("image_ack", "image_nack"):
            controls.append(dict(time_s=event["time_s"], type=event["type"],
                                 kind=event["payload"]["kind"], packet_id=event["packet_id"],
                                 wire_bytes=event["wire_size_bytes"],
                                 missing_chunks=event["payload"].get("missing_chunks"),
                                 reason=event.get("reason")))
    final = snapshot()
    assert deadline_snapshot is not None
    assert final["logical_offers"] == final["deliveries"] + final["drops"] + final["pending"]
    assert max_service_fraction <= 1 + 1e-8
    return dict(dt_s=dt, hops=hops, profile=profile, policy=policy, raw_frame_bytes=SIZE,
                chunk_count=count, image_complete_s=complete, source_ack_received_s=ack_received,
                source_nack_received_s=nack_received, image_complete_by_deadline=complete is not None and complete <= DEADLINE,
                source_knows_complete_by_deadline=ack_received is not None and ack_received <= DEADLINE,
                censored_at_s=HORIZON if complete is None else None, application_batches=batches,
                recovered_chunk_index=recovered_index, forced_loss_attempts=channel.forced_losses,
                terminal_drop_reasons=dict(losses), control_events=controls,
                deadline=deadline_snapshot, final=final, max_node_service_fraction=max_service_fraction)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--max-wall-seconds", type=float, default=115.)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to((ROOT / "research/uavx-20261106").resolve()):
        raise ValueError("Output outside authorized research directory")
    output.mkdir(parents=True, exist_ok=False)
    watched = [ROOT / "comm/mesh_router.py", ROOT / "core/models.py", Path(__file__).resolve(),
               ROOT / "research/uavx-20261106/radio_sensitivity.py",
               ROOT / "research/uavx-20261106/radio-sensitivity-01/cases.jsonl"]
    before = {str(p.relative_to(ROOT)): sha(p) for p in watched}
    started, began = datetime.now(timezone.utc).isoformat(), time.monotonic()
    cases = list(itertools.product((.5, .1), (5, 14), PROFILES, POLICIES))
    if args.limit is not None:
        cases = cases[:args.limit]
    rows = []
    with (output / "cases.jsonl").open("w", encoding="utf-8") as stream:
        for dt, hops, profile, policy in cases:
            if time.monotonic() - began > args.max_wall_seconds:
                break
            row = one_case(dt, hops, profile, policy)
            rows.append(row)
            stream.write(json.dumps(row, allow_nan=False) + "\n")
            stream.flush()
            if len(rows) % 20 == 0:
                print(f"Completed {len(rows)}/{len(cases)} cases in {time.monotonic()-began:.2f}s", flush=True)
    after = {str(p.relative_to(ROOT)): sha(p) for p in watched}
    manifest = dict(scope="Isolated research protocol comparison using unchanged MeshRouter; not a flight run",
                    started_utc=started, finished_utc=datetime.now(timezone.utc).isoformat(),
                    wall_time_s=time.monotonic() - began, planned_cases=len(cases), completed_cases=len(rows),
                    source_sha256_before=before, source_sha256_after=after, source_stable=before == after,
                    fleet_identities=30, radio_chain_hops=[5, 14], dt_s=[.5, .1],
                    raw_frame_bytes=SIZE, chunk_bytes=CHUNK, deadline_s=DEADLINE, horizon_s=HORIZON,
                    node_bandwidth_bytes_s=RATE, queue_limit_bytes=QUEUE,
                    policies=list(POLICIES), profiles=list(PROFILES),
                    feedback="Real reverse-path CONTROL packets with priority zero; charged same shared TX/RX calendars. ACK on first verified completion and each received duplicate batch final chunk; full retries stop only at first received ACK. Queued duplicate data is not cancelled.",
                    nack="One NACK triggered by receipt of original final data chunk, listing currently missing indices. One selective repair batch. No repeated NACK/loss-detector timer; missing final marker may prevent repair. This is deliberately optimistic feedback timing, not a complete reliable protocol.",
                    fec="One raw 512-byte XOR parity chunk over64 equal-length data chunks. All serialized bytes charged; recover exactly one missing data chunk. Final length and SHA256 verified. No general multi-erasure FEC claim.",
                    channel="Loss sampled at whole serialized attempt completion. One/two original middle data chunks forced lost on all source attempts. Burst profiles fail every completed attempt on selected bidirectional links while2<=t<5; link topology/quality view remains stale-connected.",
                    caveats=["No background traffic, route discovery, interference physics, clock uncertainty or vehicle motion.",
                             "Static selected logical chain; all declared adjacent distances <=100m. Other30-fleet identities inactive.",
                             "Deterministic counterexamples, not probability estimates or matched-mean IID/burst comparison.",
                             "hop_delay_s changes with dt as in current constructor; numerical and model delay remain coupled.",
                             "Thirty-second observation cutoff does not extend ten-second deadline."])
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(dict(output=str(output), completed=len(rows), planned=len(cases),
                          wall_time_s=manifest["wall_time_s"], source_stable=before == after)), flush=True)


if __name__ == "__main__":
    main()
