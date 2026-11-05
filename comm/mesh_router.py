"""
Dynamic Multi-Hop Mesh Routing Engine
Implements AODV-inspired dynamic routing with ETX (Expected Transmission Count) metrics,
packet queuing, forwarding, and latency tracking.
"""
import math
import random
import json
from copy import deepcopy
import networkx as nx
from typing import Dict, List, Optional, Tuple, Set
from collections import defaultdict, deque
from core.models import UAV, GCS, Packet, PacketType, MissionState, Vector3D
from core.config import config
from comm.channel import RFChannel


class MeshRouter:
    """Dynamic multi-hop routing engine for UAV mesh network"""

    def __init__(self, mission_state: MissionState, event_manager=None,
                 max_range_m: Optional[float] = None, rng=None,
                 hop_delay_s: float = 0.0,
                 node_bandwidth_bytes_s: Optional[float] = None,
                 queue_limit_bytes: Optional[int] = None):
        """Create an abstract router.

        ``rng`` controls packet-loss draws and accepts ``random.Random(seed)``.
        RF channel sampling retains its existing NumPy RNG. ``hop_delay_s``
        is the minimum time between forwarding attempts; simulation time is
        owned by the caller, never wall-clock time.

        With ``node_bandwidth_bytes_s``, each node (including GCS) has a single
        half-duplex serial service calendar shared by TX and RX. Disjoint links
        may operate in parallel. Packets can consume several calls' capacity;
        partial service is preemptible at call boundaries. Loss is sampled only
        after a whole attempt, whose serialized bytes have consumed capacity.
        Idle capacity is never carried into a later interval. This is an
        abstract link-service model, not an RF propagation/interference model.
        ``queue_limit_bytes`` caps each node's resident serialized bytes.
        """
        if max_range_m is not None and (not math.isfinite(max_range_m) or max_range_m <= 0):
            raise ValueError("max_range_m must be positive and finite")
        if not math.isfinite(hop_delay_s) or hop_delay_s < 0:
            raise ValueError("hop_delay_s must be nonnegative and finite")
        if node_bandwidth_bytes_s is not None and (
                isinstance(node_bandwidth_bytes_s, bool)
                or not math.isfinite(node_bandwidth_bytes_s)
                or node_bandwidth_bytes_s <= 0):
            raise ValueError("node_bandwidth_bytes_s must be positive and finite")
        if queue_limit_bytes is not None and (
                isinstance(queue_limit_bytes, bool)
                or not isinstance(queue_limit_bytes, int) or queue_limit_bytes <= 0):
            raise ValueError("queue_limit_bytes must be a positive integer")
        self.max_range_m = max_range_m
        self.hop_delay_s = hop_delay_s
        self.rng = rng if rng is not None else random
        self.mission_state = mission_state
        self.node_bandwidth_bytes_s = node_bandwidth_bytes_s
        self.queue_limit_bytes = queue_limit_bytes
        self._bounded_radio = node_bandwidth_bytes_s is not None or queue_limit_bytes is not None
        self._wire_sizes: Dict[int, int] = {}
        self._packet_types: Dict[int, PacketType] = {}
        self._queued_at: Dict[int, int] = {}
        self._queued_bytes: Dict[int, int] = defaultdict(int)
        self._queue_peaks: Dict[int, int] = defaultdict(int)
        self._partial_bytes: Dict[int, float] = {}
        self._partial_edges: Dict[int, Tuple[int, int]] = {}
        self._last_service_time = float(mission_state.time)
        self._node_service_bytes: Dict[int, float] = defaultdict(float)
        self._node_tx_bytes: Dict[int, float] = defaultdict(float)
        self._node_rx_bytes: Dict[int, float] = defaultdict(float)
        self.radio_service_samples: List[Dict] = []
        self.radio_attempts = 0
        self.radio_completed_attempts = 0
        self.radio_aborted_partial_attempts = 0
        self.radio_queue_overflows = 0
        self.rf_channel = RFChannel()
        self.cfg_rf = config.rf
        # Optional DynamicEventManager; when set, active RF interference zones
        # degrade link SNR during topology evaluation (see comm/channel.py).
        self.event_manager = event_manager

        # Routing tables: {node_id: {dest_id: (next_hop_id, etx_cost)}}
        self.routing_table: Dict[int, Dict[int, Tuple[int, float]]] = defaultdict(dict)

        # Link quality cache: {(node_a, node_b): link_quality_dict}
        self.link_quality_cache: Dict[Tuple[int, int], Dict] = {}

        # Packet queues per UAV
        self.packet_queues: Dict[int, deque] = defaultdict(deque)

        # Network topology graph
        self.network_graph = nx.Graph()

        # Statistics
        self.packets_sent = 0
        self.packets_delivered = 0
        self.packets_dropped = 0
        self.total_hops = 0

        # Resilience and recovery statistics
        self.reroute_attempts = 0
        self.successful_recoveries = 0
        self.ttl_drops = 0
        self.recovery_latencies: List[float] = []
        self.delivered_packet_ids: Set[int] = set()
        self.terminal_packet_ids: Set[int] = set()
        self.packet_events: List[Dict] = []
        self._offered_packets: Dict[int, Dict] = {}
        self._ready_at: Dict[int, float] = {}
        self.reroutes_found = 0

        self.last_routing_update = 0.0

    def update_network_topology(self):
        """
        Scans all UAV positions, calculates RF link quality, and builds the network graph.
        Updates neighbor lists and connectivity status for each UAV.
        """
        current_time = self.mission_state.time
        gcs = self.mission_state.gcs
        uavs = self.mission_state.uavs

        # Clear previous topology
        self.network_graph.clear()
        self.link_quality_cache.clear()

        # Add GCS as node -1
        self.network_graph.add_node(-1, pos=gcs.position)

        # Add all operational UAVs
        for uav_id, uav in uavs.items():
            if uav.is_operational():
                self.network_graph.add_node(uav_id, pos=uav.position)

        # Evaluate all pairwise links
        nodes = list(self.network_graph.nodes())

        for i, node_a in enumerate(nodes):
            for node_b in nodes[i+1:]:
                pos_a = self._get_node_position(node_a)
                pos_b = self._get_node_position(node_b)

                if self.max_range_m is not None and (pos_a - pos_b).norm() > self.max_range_m:
                    continue
                link_quality = self.rf_channel.calculate_link_quality(
                    pos_a, pos_b, interference_event=self.event_manager)

                if link_quality['is_connected']:
                    etx = link_quality['etx']
                    self.network_graph.add_edge(node_a, node_b, weight=etx, **link_quality)
                    self.link_quality_cache[(node_a, node_b)] = link_quality
                    self.link_quality_cache[(node_b, node_a)] = link_quality

        # Update UAV neighbor lists and GCS connectivity
        gcs.connected_uavs.clear()

        for uav_id, uav in uavs.items():
            uav.neighbor_uavs.clear()
            uav.connected_to_gcs = False
            if not uav.is_operational():
                continue

            if self.network_graph.has_node(uav_id):
                neighbors = list(self.network_graph.neighbors(uav_id))
                uav.neighbor_uavs = [n for n in neighbors if n >= 0]

                # Check GCS connectivity (direct or multi-hop)
                if -1 in neighbors:
                    uav.connected_to_gcs = True
                    gcs.connected_uavs.append(uav_id)
                else:
                    # Check if path to GCS exists
                    if nx.has_path(self.network_graph, uav_id, -1):
                        uav.connected_to_gcs = True

    def compute_routing_tables(self):
        """
        Computes shortest-path routing tables using Dijkstra's algorithm with ETX weights.
        Each UAV knows the next hop towards any destination (including GCS).
        """
        self.routing_table.clear()

        # For each source node, compute shortest paths to all destinations
        for source in self.network_graph.nodes():
            if source == -1:
                continue  # GCS doesn't forward packets

            try:
                # Dijkstra from source to all nodes
                paths = nx.single_source_dijkstra_path(self.network_graph, source, weight='weight')
                costs = nx.single_source_dijkstra_path_length(self.network_graph, source, weight='weight')

                for dest, path in paths.items():
                    if len(path) > 1 and dest != source:
                        next_hop = path[1]  # First hop in path
                        etx_cost = costs[dest]
                        self.routing_table[source][dest] = (next_hop, etx_cost)

            except (nx.NetworkXNoPath, nx.NodeNotFound):
                pass

    def route_packet(self, packet: Packet) -> Optional[int]:
        """
        Determines the next hop for a packet based on routing table.
        Returns next_hop UAV ID, or None if no route exists.
        """
        current_hop = packet.path[packet.current_hop] if packet.path else packet.source_id
        dest_id = packet.dest_id

        if current_hop == dest_id:
            return None  # Already at destination

        if current_hop not in self.routing_table:
            return None  # No routing info

        if dest_id in self.routing_table[current_hop]:
            next_hop, _ = self.routing_table[current_hop][dest_id]
            return next_hop

        return None

    def _record_offer(self, packet: Packet) -> bool:
        """Freeze one logical packet's provenance; duplicate IDs are idempotent."""
        if packet.packet_id in self._offered_packets:
            return False
        if self._bounded_radio:
            self._wire_sizes[packet.packet_id] = self.packet_wire_size(packet)
            self._packet_types[packet.packet_id] = packet.packet_type
        self.packets_sent += 1
        self._offered_packets[packet.packet_id] = {
            "packet_id": packet.packet_id,
            "source_id": packet.source_id,
            "dest_id": packet.dest_id,
            "payload": deepcopy(packet.payload),
            "creation_time_s": packet.creation_time,
        }
        if self._bounded_radio:
            self._offered_packets[packet.packet_id]["wire_size_bytes"] = self._wire_sizes[packet.packet_id]
        self._event("offered", packet)
        return True

    def _event(self, event_type: str, packet: Packet, **details) -> None:
        event = deepcopy(self._offered_packets[packet.packet_id])
        event.update(type=event_type, time_s=self.mission_state.time, **details)
        self.packet_events.append(event)

    def _node_operational(self, node_id: int) -> bool:
        if node_id == -1:
            return True
        node = self.mission_state.uavs.get(node_id)
        return node is not None and node.is_operational()

    @staticmethod
    def packet_wire_size(packet: Packet) -> int:
        """Size of the canonical UTF-8 JSON envelope plus explicit payload padding.

        Image bytes must be carried in the JSON payload (for example base64),
        whose encoded length is counted. ``payload_size`` can request zero
        padding for synthetic traffic; it can never understate serialized data.
        Mutable path/retry fields are local routing metadata, not this envelope.
        """
        if (isinstance(packet.payload_size, bool)
                or not math.isfinite(packet.payload_size) or packet.payload_size < 0):
            raise ValueError("payload_size must be nonnegative and finite")
        options = dict(sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)
        payload = json.dumps(packet.payload, **options).encode('utf-8')
        envelope = json.dumps(dict(packet_id=packet.packet_id,
                                   packet_type=packet.packet_type.value,
                                   source_id=packet.source_id, dest_id=packet.dest_id,
                                   creation_time_s=packet.creation_time,
                                   payload=packet.payload), **options).encode('utf-8')
        return len(envelope) + max(0, math.ceil(packet.payload_size) - len(payload))

    def _priority_key(self, packet: Packet):
        """Control first; emergency reports before telemetry and bulk imagery.

        Non-control traffic ages one priority level every two simulation seconds
        so a sustained telemetry stream cannot indefinitely starve old images.
        Provenance and packet type are frozen at offer time for this decision.
        """
        record = self._offered_packets.get(packet.packet_id, {})
        payload = record.get('payload') or {}
        kind = payload.get('kind', '')
        priority = payload.get('priority', 1)
        emergency = isinstance(priority, (int, float)) and not isinstance(priority, bool) and priority > 1
        if self._packet_types.get(packet.packet_id, packet.packet_type) == PacketType.CONTROL:
            rank = 0
        elif emergency or kind in ('poi_report', 'emergency', 'emergency_report'):
            rank = 1
        elif kind in ('poi_image_chunk', 'unreal_camera_chunk', 'image_chunk', 'imagery', 'bulk'):
            rank = 3
        else:
            rank = 2
        created = record.get('creation_time_s', packet.creation_time)
        age = max(0., self.mission_state.time - created)
        if rank:
            rank = max(1, rank - int(age / 2.))
        return rank, created, packet.packet_id

    def _forget_resident(self, packet: Packet):
        node = self._queued_at.pop(packet.packet_id, None)
        if node is not None:
            self._queued_bytes[node] -= self._wire_sizes[packet.packet_id]

    def _enqueue(self, node: int, packet: Packet) -> bool:
        if self._bounded_radio and self._queued_at.get(packet.packet_id) != node:
            size = self._wire_sizes[packet.packet_id]
            if self.queue_limit_bytes is not None and self._queued_bytes[node] + size > self.queue_limit_bytes:
                self.radio_queue_overflows += 1
                self._record_drop(packet, 'queue_overflow')
                return False
            self._forget_resident(packet)
            self._queued_at[packet.packet_id] = node
            self._queued_bytes[node] += size
            self._queue_peaks[node] = max(self._queue_peaks[node], self._queued_bytes[node])
        self.packet_queues[node].append(packet)
        return True

    def _cancel_partial(self, packet: Packet):
        if self._partial_bytes.pop(packet.packet_id, 0) > 0:
            self.radio_aborted_partial_attempts += 1
        self._partial_edges.pop(packet.packet_id, None)

    def _service_attempt(self, packet: Packet, node: int, next_node: int,
                         available_at: Dict[int, float], interval_start: float,
                         sample: Dict) -> bool:
        """Consume serialized service on both endpoints; return whole-attempt completion."""
        if self.node_bandwidth_bytes_s is None:
            return True
        packet_id = packet.packet_id
        edge = (node, next_node)
        if packet_id in self._partial_edges and self._partial_edges[packet_id] != edge:
            self._cancel_partial(packet)
        # The configured minimum hop delay can include serialization. A newly
        # offered packet cannot spend capacity from before its offer timestamp.
        earliest = self._ready_at.get(packet_id, self.mission_state.time) - self.hop_delay_s
        start = max(interval_start, earliest,
                    available_at.get(node, interval_start), available_at.get(next_node, interval_start))
        remaining_time = max(0., self.mission_state.time - start)
        remaining_bytes = self._wire_sizes[packet_id] - self._partial_bytes.get(packet_id, 0.)
        amount = min(remaining_bytes, remaining_time * self.node_bandwidth_bytes_s)
        if amount <= 1e-9:
            return False
        if packet_id not in self._partial_edges:
            self._partial_edges[packet_id] = edge
            self.radio_attempts += 1
        end = start + amount / self.node_bandwidth_bytes_s
        available_at[node] = available_at[next_node] = end
        for endpoint in edge:
            self._node_service_bytes[endpoint] += amount
            sample['node_service_bytes'][endpoint] = sample['node_service_bytes'].get(endpoint, 0.) + amount
        self._node_tx_bytes[node] += amount
        self._node_rx_bytes[next_node] += amount
        self._partial_bytes[packet_id] = self._partial_bytes.get(packet_id, 0.) + amount
        if self._partial_bytes[packet_id] + 1e-8 < self._wire_sizes[packet_id]:
            return False
        self._partial_bytes.pop(packet_id, None)
        self._partial_edges.pop(packet_id, None)
        self.radio_completed_attempts += 1
        return True

    def send_packet(self, packet: Packet) -> bool:
        """Offer once, then enqueue or record a terminal drop.

        A repeated packet ID is rejected without changing denominators,
        provenance, or the original packet's terminal outcome.
        """
        if packet.packet_id in self._offered_packets:
            return False
        packet.creation_time = self.mission_state.time
        self._record_offer(packet)
        if not self._node_operational(packet.source_id):
            self._record_drop(packet, "source_unavailable")
            return False
        path = self._find_path(packet.source_id, packet.dest_id)
        if not path:
            self._record_drop(packet, "no_route")
            return False
        packet.path = path
        packet.current_hop = 0
        self._ready_at[packet.packet_id] = self.mission_state.time + self.hop_delay_s
        return self._enqueue(packet.source_id, packet)

    def _record_drop(self, packet: Packet, reason: str) -> None:
        if packet.packet_id in self.terminal_packet_ids:
            return
        self.terminal_packet_ids.add(packet.packet_id)
        self._forget_resident(packet)
        self._cancel_partial(packet)
        self._ready_at.pop(packet.packet_id, None)
        self.packets_dropped += 1
        if packet.dest_id == -1:
            self.mission_state.gcs.packets_dropped += 1
        self._event("dropped", packet, reason=reason)

    def _reroute(self, packet: Packet, current_node: int, next_node: int) -> bool:
        self._cancel_partial(packet)
        self.reroute_attempts += 1
        alternative = self._find_alternative_path(
            current_node, packet.dest_id,
            broken_edge=(current_node, next_node),
            visited_nodes=set(packet.path[:packet.current_hop + 1]),
        )
        if not alternative or len(alternative) <= 1:
            return False
        remaining_hops = len(alternative) - 1
        if (packet.current_hop + remaining_hops > self.cfg_rf.max_hops
                or remaining_hops > packet.ttl):
            return False
        packet.path = packet.path[:packet.current_hop] + alternative
        packet.reroute_count += 1
        packet.retry_count = 0
        self.reroutes_found += 1
        # Route computation is not service restoration or packet delivery.
        self._ready_at[packet.packet_id] = self.mission_state.time + self.hop_delay_s
        return self._enqueue(current_node, packet)

    def process_packet_forwarding(self, dt: float):
        """Service each queued packet once and at most one hop per packet per call.

        Snapshot outgoing packets before forwarding so iteration order cannot
        let one packet traverse several pre-existing queues in the same tick.
        Queue wait and retry wait remain in end-to-end simulation latency.
        """
        interval_start = self._last_service_time
        available_at = {}
        sample = None
        if self.node_bandwidth_bytes_s is not None:
            now = self.mission_state.time
            if (not math.isfinite(dt) or dt <= 0 or not math.isfinite(now)
                    or now < self._last_service_time):
                raise ValueError('bounded radio requires positive dt and a finite nondecreasing simulation clock')
            sample = dict(time_s=now, elapsed_s=now - interval_start, node_service_bytes={})
            self._last_service_time = now
        outgoing = []
        for node_id, queue in list(self.packet_queues.items()):
            outgoing.extend((node_id, queue.popleft()) for _ in range(len(queue)))
        if self._bounded_radio:
            outgoing.sort(key=lambda item: self._priority_key(item[1]))
        processed = set()
        for node_id, packet in outgoing:
            packet_id = packet.packet_id
            if packet_id in processed or packet_id in self.terminal_packet_ids:
                continue
            processed.add(packet_id)
            if not self._node_operational(node_id):
                self._record_drop(packet, "current_node_unavailable")
                continue
            if self.mission_state.time + 1e-9 < self._ready_at.get(packet_id, 0):
                self.packet_queues[node_id].appendleft(packet)
                continue
            if (not packet.path or packet.current_hop < 0
                    or packet.current_hop >= len(packet.path)
                    or packet.path[packet.current_hop] != node_id):
                self._record_drop(packet, "invalid_path")
                continue
            if node_id == packet.dest_id:
                self._deliver_packet(packet)
                continue
            if packet.ttl <= 0 or packet.current_hop >= self.cfg_rf.max_hops:
                self.ttl_drops += 1
                self._record_drop(packet, "hop_limit")
                continue
            if packet.current_hop + 1 >= len(packet.path):
                self._record_drop(packet, "invalid_destination")
                continue
            next_node = packet.path[packet.current_hop + 1]
            link_quality = self.link_quality_cache.get((node_id, next_node))
            connected = (self._node_operational(next_node) and link_quality is not None
                         and link_quality.get("is_connected", False))
            if connected and self.max_range_m is not None:
                connected = ((self._get_node_position(node_id) -
                              self._get_node_position(next_node)).norm() <= self.max_range_m)
            if not connected:
                if not self._reroute(packet, node_id, next_node):
                    self._record_drop(packet, "route_lost")
                continue
            if not self._service_attempt(packet, node_id, next_node, available_at, interval_start, sample):
                self.packet_queues[node_id].append(packet)
                continue
            if self.rng.random() >= link_quality["pdr"]:
                if packet.retry_count < packet.max_retries:
                    packet.retry_count += 1
                    self._ready_at[packet_id] = self.mission_state.time + self.hop_delay_s
                    self.packet_queues[node_id].append(packet)
                elif not self._reroute(packet, node_id, next_node):
                    self._record_drop(packet, "retries_exhausted")
                continue
            packet.advance_hop()
            packet.ttl -= 1
            packet.retry_count = 0
            self.total_hops += 1
            self._ready_at[packet_id] = self.mission_state.time + self.hop_delay_s
            if next_node == packet.dest_id:
                self._deliver_packet(packet)
            else:
                self._enqueue(next_node, packet)
        if sample is not None:
            self.radio_service_samples.append(sample)

    def _deliver_packet(self, packet: Packet):
        """Record terminal delivery once, including offered-to-receipt latency."""
        if packet.packet_id in self.terminal_packet_ids:
            return
        # Support existing direct delivery callers without corrupting accounting.
        self._record_offer(packet)
        self.terminal_packet_ids.add(packet.packet_id)
        self._forget_resident(packet)
        self._cancel_partial(packet)
        self.delivered_packet_ids.add(packet.packet_id)
        self._ready_at.pop(packet.packet_id, None)
        self.packets_delivered += 1
        provenance = self._offered_packets[packet.packet_id]
        latency = self.mission_state.time - provenance["creation_time_s"]
        if packet.reroute_count > 0:
            self.successful_recoveries += 1
            # This is rerouted-packet end-to-end latency, not fault recovery time.
            self.recovery_latencies.append(latency)
        self._event("delivered", packet, latency_s=latency, hops=packet.current_hop,
                    reroute_count=packet.reroute_count)
        if packet.dest_id == -1:
            self.mission_state.gcs.packets_received += 1
            self.mission_state.gcs.total_latency += latency

    def _find_path(self, source_id: int, dest_id: int) -> Optional[List[int]]:
        """Finds shortest path from source to destination using network graph"""
        if source_id not in self.network_graph or dest_id not in self.network_graph:
            return None

        try:
            path = nx.shortest_path(self.network_graph, source_id, dest_id, weight='weight')
            if len(path) - 1 > self.cfg_rf.max_hops:
                return None
            return path
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None

    def _find_alternative_path(self, source_id: int, dest_id: int,
                               broken_edge: Optional[Tuple[int, int]] = None,
                               visited_nodes: Optional[Set[int]] = None) -> Optional[List[int]]:
        """
        Finds shortest alternative path excluding the broken edge and avoiding loops.
        """
        if source_id not in self.network_graph or dest_id not in self.network_graph:
            return None

        try:
            H = self.network_graph.copy()
            for node in list(H.nodes()):
                if not self._node_operational(node):
                    H.remove_node(node)
            for u, v in list(H.edges()):
                quality = self.link_quality_cache.get((u, v))
                if quality is not None and not quality.get("is_connected", False):
                    H.remove_edge(u, v)
                elif self.max_range_m is not None and (
                    self._get_node_position(u) - self._get_node_position(v)
                ).norm() > self.max_range_m:
                    H.remove_edge(u, v)
            if broken_edge:
                u, v = broken_edge
                if H.has_edge(u, v):
                    H.remove_edge(u, v)
                if H.has_edge(v, u):
                    H.remove_edge(v, u)

            # Prevent loops: remove already-visited intermediate nodes
            if visited_nodes:
                for node in visited_nodes:
                    if node != source_id and node != dest_id and H.has_node(node):
                        H.remove_node(node)

            path = nx.shortest_path(H, source_id, dest_id, weight='weight')
            if len(path) - 1 > self.cfg_rf.max_hops:
                return None
            return path
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None

    def _get_node_position(self, node_id: int) -> Vector3D:
        """Gets position of a node (UAV or GCS)"""
        if node_id == -1:
            return self.mission_state.gcs.position
        else:
            return self.mission_state.uavs[node_id].position

    def get_network_stats(self) -> Dict:
        """Returns network performance statistics"""
        pdr_overall = self.packets_delivered / max(1, self.packets_sent)
        avg_latency = self.mission_state.gcs.total_latency / max(1, self.mission_state.gcs.packets_received)
        avg_hops = sum(e["hops"] for e in self.packet_events if e["type"] == "delivered") / max(1, self.packets_delivered)
        avg_rec_lat = (sum(self.recovery_latencies) / len(self.recovery_latencies)) if self.recovery_latencies else 0.0

        result = {
            'packets_sent': self.packets_sent,
            'packets_delivered': self.packets_delivered,
            'packets_dropped': self.packets_dropped,
            'reroute_attempts': self.reroute_attempts,
            'reroutes_found': self.reroutes_found,
            'successful_recoveries': self.successful_recoveries,
            'ttl_drops': self.ttl_drops,
            'avg_recovery_latency': avg_rec_lat,
            'pdr': pdr_overall,
            'avg_latency': avg_latency,
            'avg_hops': avg_hops,
            'num_nodes': self.network_graph.number_of_nodes(),
            'num_edges': self.network_graph.number_of_edges()
        }
        if self._bounded_radio:
            result['radio_service'] = {
                'node_bandwidth_bytes_s': self.node_bandwidth_bytes_s,
                'queue_limit_bytes': self.queue_limit_bytes,
                'duplex': 'shared_tx_rx_half_duplex',
                'scheduling': 'control_then_emergency_then_telemetry_then_bulk; non-control aging every 2 s',
                'wire_encoding': 'canonical_utf8_json_with_explicit_payload_size_padding',
                'attempts_started': self.radio_attempts,
                'attempts_completed': self.radio_completed_attempts,
                'partial_attempts_aborted': self.radio_aborted_partial_attempts,
                'queue_overflow_drops': self.radio_queue_overflows,
                'node_service_bytes': dict(self._node_service_bytes),
                'node_tx_bytes': dict(self._node_tx_bytes),
                'node_rx_bytes': dict(self._node_rx_bytes),
                'queue_bytes': dict(self._queued_bytes),
                'peak_queue_bytes': dict(self._queue_peaks),
            }
        return result
