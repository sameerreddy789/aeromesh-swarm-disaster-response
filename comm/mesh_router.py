"""
Dynamic Multi-Hop Mesh Routing Engine
Implements AODV-inspired dynamic routing with ETX (Expected Transmission Count) metrics,
packet queuing, forwarding, and latency tracking.
"""
import math
import random
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
                 hop_delay_s: float = 0.0):
        """Create an abstract router.

        ``rng`` controls packet-loss draws and accepts ``random.Random(seed)``.
        RF channel sampling retains its existing NumPy RNG. ``hop_delay_s``
        is the minimum time between forwarding attempts; simulation time is
        owned by the caller, never wall-clock time.
        """
        if max_range_m is not None and (not math.isfinite(max_range_m) or max_range_m <= 0):
            raise ValueError("max_range_m must be positive and finite")
        if not math.isfinite(hop_delay_s) or hop_delay_s < 0:
            raise ValueError("hop_delay_s must be nonnegative and finite")
        self.max_range_m = max_range_m
        self.hop_delay_s = hop_delay_s
        self.rng = rng if rng is not None else random
        self.mission_state = mission_state
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
        self.packets_sent += 1
        self._offered_packets[packet.packet_id] = {
            "packet_id": packet.packet_id,
            "source_id": packet.source_id,
            "dest_id": packet.dest_id,
            "payload": deepcopy(packet.payload),
            "creation_time_s": packet.creation_time,
        }
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
        self.packet_queues[packet.source_id].append(packet)
        return True

    def _record_drop(self, packet: Packet, reason: str) -> None:
        if packet.packet_id in self.terminal_packet_ids:
            return
        self.terminal_packet_ids.add(packet.packet_id)
        self._ready_at.pop(packet.packet_id, None)
        self.packets_dropped += 1
        if packet.dest_id == -1:
            self.mission_state.gcs.packets_dropped += 1
        self._event("dropped", packet, reason=reason)

    def _reroute(self, packet: Packet, current_node: int, next_node: int) -> bool:
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
        self.packet_queues[current_node].append(packet)
        return True

    def process_packet_forwarding(self, dt: float):
        """Service each queued packet once and at most one hop per packet per call.

        Snapshot outgoing packets before forwarding so iteration order cannot
        let one packet traverse several pre-existing queues in the same tick.
        Queue wait and retry wait remain in end-to-end simulation latency.
        """
        outgoing = []
        for node_id, queue in list(self.packet_queues.items()):
            outgoing.extend((node_id, queue.popleft()) for _ in range(len(queue)))
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
                self.packet_queues[next_node].append(packet)

    def _deliver_packet(self, packet: Packet):
        """Record terminal delivery once, including offered-to-receipt latency."""
        if packet.packet_id in self.terminal_packet_ids:
            return
        # Support existing direct delivery callers without corrupting accounting.
        self._record_offer(packet)
        self.terminal_packet_ids.add(packet.packet_id)
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

        return {
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
