"""Research-only isolated MeshRouter sensitivity experiment.

No production source changes. This is a logical radio fixture, not a flight run.
The fleet has 30 identities; only the selected chain radios participate.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
from pathlib import Path
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from comm.mesh_router import MeshRouter
from core.config import RFConfig
from core.models import MissionState, Packet, PacketType, UAV, Vector3D

RATE = 131072.0
QUEUE = 1048576
CHUNK = 512
DEADLINE = 10.0
HORIZON = 15.0


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class AttemptRng:
    """IID draws, optionally a scripted loss of one logical chunk's first offer."""
    def __init__(self, seed, missing_packet=None, source=None):
        self.rng = random.Random(seed)
        self.missing_packet = missing_packet
        self.source = source
        self.packet = None
        self.node = None
        self.forced_losses = 0

    def random(self):
        if (self.packet is not None and self.packet.packet_id == self.missing_packet
                and self.node == self.source):
            self.forced_losses += 1
            return 1.0
        return self.rng.random()


def one_case(dt, hops, size, loss, mode, seed=42, omit=False):
    state = MissionState()
    state.uavs = {i: UAV(i, position=Vector3D(0, 0, 0)) for i in range(30)}
    for i in range(hops):
        # L-shaped geometry makes every declared link <=100 m and fits the map.
        state.uavs[i].position = Vector3D(min(i + 1, 10) * 90,
                                         max(0, i + 1 - 10) * 90, 40)
    source = hops - 1
    count = math.ceil(size / CHUNK)
    rng = AttemptRng(seed, count // 2 if omit else None, source)
    router = MeshRouter(state, max_range_m=100, rng=rng, hop_delay_s=dt,
                        node_bandwidth_bytes_s=RATE, queue_limit_bytes=QUEUE)
    router.cfg_rf = RFConfig(max_hops=32)
    router.network_graph.add_nodes_from([-1, *range(30)])
    path = [-1, *range(hops)]
    for a, b in zip(path, path[1:]):
        router.network_graph.add_edge(a, b, weight=1 / (1 - loss))
        quality = {"is_connected": True, "pdr": 1 - loss, "etx": 1 / (1 - loss)}
        router.link_quality_cache[a, b] = quality
        router.link_quality_cache[b, a] = quality

    original_service = router._service_attempt
    def observe_attempt(packet, node, next_node, available_at, interval_start, sample):
        rng.packet, rng.node = packet, node
        return original_service(packet, node, next_node, available_at, interval_start, sample)
    router._service_attempt = observe_attempt
    payload_bytes = (bytes(range(256)) * math.ceil(size / 256))[:size]
    digest = hashlib.sha256(payload_bytes).hexdigest()
    received = {}
    complete = None
    cursor = 0
    rounds = 0
    next_retry = 2.0
    deadline_snapshot = None
    packet_latencies = []
    link_service_max_fraction = 0.0
    offered_identity_counts = Counter()

    def offer_frame():
        nonlocal rounds
        rounds += 1
        for index in range(count):
            data = payload_bytes[index * CHUNK:(index + 1) * CHUNK]
            payload = dict(kind="poi_image_chunk", poi_id=0, priority=1,
                           detection_time_s=0.0, observation_time_s=0.0,
                           capture_id="research-frame", image_sha256=digest,
                           chunk_count=count, chunk_index=index,
                           data_b64=base64.b64encode(data).decode("ascii"))
            router.send_packet(Packet(router.packets_sent, PacketType.DATA, source, -1,
                                      ttl=32, payload_size=0, payload=payload))
            offered_identity_counts[index] += 1

    def snapshot():
        offers = [e for e in router.packet_events if e["type"] == "offered"]
        deliveries = [e for e in router.packet_events if e["type"] == "delivered"]
        stats = router.get_network_stats()
        return dict(at_s=state.time, complete_image_bytes=size if complete is not None else 0,
                    complete_image_s=complete, unique_received_chunks=len(received),
                    unique_received_chunk_bytes=sum(map(len, received.values())),
                    logical_offers=router.packets_sent,
                    logical_deliveries=router.packets_delivered,
                    terminal_drops=router.packets_dropped,
                    pending=sum(map(len, router.packet_queues.values())),
                    pdr=stats["pdr"],
                    offered_wire_bytes=sum(e["wire_size_bytes"] for e in offers),
                    delivered_wire_bytes=sum(e["wire_size_bytes"] for e in deliveries),
                    attempts_started=router.radio_attempts,
                    attempts_completed=router.radio_completed_attempts,
                    successful_hops=router.total_hops,
                    transmitted_service_bytes=sum(router._node_tx_bytes.values()),
                    aborted_partial_attempts=router.radio_aborted_partial_attempts,
                    queue_overflows=router.radio_queue_overflows)

    offer_frame()
    for tick in range(1, round(HORIZON / dt) + 1):
        state.time = round(tick * dt, 9)
        router.process_packet_forwarding(dt)
        for event in router.packet_events[cursor:]:
            if event["type"] != "delivered":
                continue
            data = event["payload"]
            assert data["image_sha256"] == digest and data["chunk_count"] == count
            index = data["chunk_index"]
            decoded = base64.b64decode(data["data_b64"], validate=True)
            assert index not in received or received[index] == decoded
            received[index] = decoded
            packet_latencies.append(event["latency_s"])
            if complete is None and len(received) == count:
                assembled = b"".join(received[i] for i in range(count))
                assert len(assembled) == size
                assert hashlib.sha256(assembled).hexdigest() == digest
                complete = state.time
        cursor = len(router.packet_events)
        sample = router.radio_service_samples[-1]
        if sample["elapsed_s"]:
            link_service_max_fraction = max(link_service_max_fraction, max(
                sample["node_service_bytes"].values(), default=0.0) / (RATE * sample["elapsed_s"]))
        if abs(state.time - DEADLINE) < 1e-8:
            deadline_snapshot = snapshot()
        # Deliberately matches current shared-state retry-stop, with no ACK packet.
        if mode == "oracle_stop_full_retry" and complete is None and state.time + 1e-8 >= next_retry:
            offer_frame()
            next_retry += 2.0
    final = snapshot()
    assert deadline_snapshot is not None
    assert final["logical_offers"] == final["logical_deliveries"] + final["terminal_drops"] + final["pending"]
    assert link_service_max_fraction <= 1 + 1e-8
    return dict(dt_s=dt, hops=hops, raw_frame_bytes=size, chunk_bytes=CHUNK,
                chunk_count=count, iid_loss_per_attempt=loss, seed=seed, mode=mode,
                scripted_first_offer_chunk_loss=omit, forced_losses=rng.forced_losses,
                known_minimum_hop_stage_s=hops * dt, image_complete_s=complete,
                image_complete_by_deadline=complete is not None and complete <= DEADLINE + 1e-8,
                censored_at_s=HORIZON if complete is None else None,
                application_offer_rounds=rounds,
                chunk_offer_multiplicity=dict(Counter(offered_identity_counts.values())),
                deadline=deadline_snapshot, final=final,
                max_node_service_fraction=link_service_max_fraction,
                delivered_packet_latency_mean_s=sum(packet_latencies) / len(packet_latencies) if packet_latencies else None,
                delivered_packet_latency_max_s=max(packet_latencies) if packet_latencies else None)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--max-wall-seconds", type=float, default=115)
    args = parser.parse_args()
    output = args.output.resolve()
    research_root = (ROOT / "research/uavx-20261106").resolve()
    if not output.is_relative_to(research_root):
        raise ValueError("Output must remain under the authorized research directory")
    output.mkdir(parents=True, exist_ok=False)
    watched = [ROOT / "comm/mesh_router.py", ROOT / "core/models.py", Path(__file__).resolve()]
    before = {str(p.relative_to(ROOT)): sha(p) for p in watched}
    started = datetime.now(timezone.utc).isoformat()
    began = time.monotonic()
    cases = [(*p, False) for p in itertools.product(
        (.5, .25, .1), (1, 5, 10, 14), (512, 32768, 262144),
        (0., .02, .1), ("single_offer", "oracle_stop_full_retry"))]
    cases += [(dt, h, 32768, 0., mode, True)
              for dt, h, mode in itertools.product((.5, .25, .1), (1, 5, 10, 14),
                                                   ("single_offer", "oracle_stop_full_retry"))]
    if args.limit is not None:
        cases = cases[:args.limit]
    completed = []
    with (output / "cases.jsonl").open("w", encoding="utf-8") as stream:
        for index, (dt, hops, size, loss, mode, omit) in enumerate(cases):
            if time.monotonic() - began > args.max_wall_seconds:
                print("Wall-time research budget reached; partial results explicitly retained.", flush=True)
                break
            row = one_case(dt, hops, size, loss, mode, omit=omit)
            completed.append(row)
            stream.write(json.dumps(row, allow_nan=False) + "\n")
            stream.flush()
            if (index + 1) % 12 == 0:
                print(f"Completed {index+1}/{len(cases)} isolated cases in {time.monotonic()-began:.2f}s", flush=True)
    after = {str(p.relative_to(ROOT)): sha(p) for p in watched}
    manifest = dict(scope="Isolated unchanged MeshRouter experiment; not a flight or official benchmark",
                    started_utc=started, finished_utc=datetime.now(timezone.utc).isoformat(),
                    wall_time_s=time.monotonic()-began, fleet_identities=30,
                    chain_scope="Only selected logical chain radios participate; remaining identities have inactive radios.",
                    geometry="Declared adjacent chain links <=100 m; static L-shape at z40; no motion simulated.",
                    node_bandwidth_bytes_s=RATE, queue_limit_bytes=QUEUE,
                    deadline_s=DEADLINE, observation_horizon_s=HORIZON,
                    telemetry_or_control_background=False, iid_seeds_per_case=1,
                    acknowledgment="No ACK; current shared-state immediate retry stop is deliberately reproduced.",
                    missing_chunk="Force all link retries of the first logical offer of one middle chunk to fail at its source; later application copies use baseline loss.",
                    planned_cases=len(cases), completed_cases=len(completed),
                    all_cases_completed=len(completed)==len(cases),
                    source_sha256_before=before, source_sha256_after=after, source_stable=before==after,
                    caveats=["One fixed seed is sensitivity evidence, not a reliability estimate.",
                             "Synthetic byte arrays stand in for already encoded image bytes; no camera or recognition.",
                             "No physical RF interference, protocol discovery, clock error or flight model.",
                             "Delivery after15s is right-censored; ten-second failures are still failures.",
                             "Changing dt changes the router hop-delay setting as in the current mission constructor."])
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(dict(output=str(output), completed=len(completed), planned=len(cases),
                          wall_time_s=manifest["wall_time_s"], source_stable=manifest["source_stable"])), flush=True)


if __name__ == "__main__":
    main()

