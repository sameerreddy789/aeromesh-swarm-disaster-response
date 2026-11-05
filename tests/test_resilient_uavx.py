"""End-to-end resilience, camera transport and fault semantics."""
import base64
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import networkx as nx
import numpy as np
import pytest

from core.models import UAVRole, UAVState, Vector3D
from simulation.resilient_uavx import ResilientUAVXSimulator, _protected_graph_score
from simulation.uavx_scenario import ScenarioConfig, ScenarioTask, FaultSpec
from verify_submission import verify


def near_config(**kwargs):
    return ScenarioConfig(fleet_size=12, tasks=(ScenarioTask(0,(175.,0.,0.)),),packet_loss=0,**kwargs)


def test_fast_protection_score_matches_independent_graph_analysis():
    rng=np.random.default_rng(9218)
    cases=[]
    for size in range(1,6):
        edges=[(i,j) for i in range(size) for j in range(i+1,size)]
        for mask in range(1<<len(edges)):
            adjacent=np.eye(size,dtype=bool)
            for bit,(a,b) in enumerate(edges):
                if mask & (1<<bit): adjacent[a,b]=adjacent[b,a]=True
            cases.append(adjacent)
    for _ in range(200):
        size=int(rng.integers(1,74))
        adjacent=np.triu(rng.random((size,size))<rng.uniform(),1)
        adjacent|=adjacent.T
        cases.append(adjacent)
    for adjacent in cases:
        graph=nx.from_numpy_array(adjacent)
        reached=nx.node_connected_component(graph,0)-{0}
        protected=set()
        for block in nx.biconnected_components(graph):
            if 0 in block and len(block)>=3: protected.update(block-{0})
        assert _protected_graph_score(adjacent,72)==len(reached)*73+len(protected)


def test_displaced_landing_reacquires_home_above_grounded_neighbors():
    sim=ResilientUAVXSimulator(ScenarioConfig(camera_mode='footprint',fleet_size=60))
    uid=15;flight=sim.flights[uid];u=sim.state.uavs[uid]
    u.position=Vector3D(.7508815661391881,-51.48191917963271,14.643677710160313)
    u.target_position=Vector3D(*flight.home);u.state=UAVState.RETURNING
    flight.armed=True;flight.landed=False;flight.returning=True
    flight.landing_approach=True;flight.remaining_s=88.
    initial_altitude=u.position.z
    sim._move();sim.state.time+=sim.dt
    assert not flight.landing_approach and u.position.z>initial_altitude
    for _ in range(160):
        sim._move();sim.state.time+=sim.dt
        if flight.landed: break
    assert flight.landed and not flight.armed
    assert sim.exhaustions==0 and sim.min_separation>=22.-1e-8
    assert sim.max_speed<=5.+1e-8


def test_landing_airborne_detour_does_not_restart_blocked_descent():
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint'))
    flight=sim.flights[0];u=sim.state.uavs[0]
    u.position=Vector3D(flight.home[0]+8.,flight.home[1],50.)
    u.target_position=Vector3D(*flight.home);u.state=UAVState.RETURNING
    flight.armed=True;flight.landed=False;flight.returning=True
    flight.landing_approach=True;flight.remaining_s=88.
    sim._move()
    assert flight.landing_approach
    assert u.position.z<50.


def settled_backbone(protected=True):
    sim = ResilientUAVXSimulator(near_config(relay_protection=protected, camera_mode='footprint'))
    task = sim.tasks[0]
    task.position = (700.,0.,0.)
    slots = sim._build_slots(task)
    # Use a long backbone for a structural single-relay/link loss test without
    # pretending that manually placed nodes establish flight evidence.
    graph = nx.Graph()
    points = {-1:(0.,0.,0.), **slots}
    for first,p in points.items():
        graph.add_node(first)
        for second,q in points.items():
            if first != second and __import__('math').dist(p,q)<=100:
                graph.add_edge(first,second)
    return graph, max(slots)


def test_protected_geometry_has_two_node_disjoint_gcs_paths():
    graph,source = settled_backbone()
    assert nx.node_connectivity(graph,-1,source) >= 2
    for uid in set(graph)-{-1,source}:
        damaged = graph.copy();damaged.remove_node(uid)
        assert nx.has_path(damaged,-1,source)
    for edge in list(graph.edges):
        damaged = graph.copy();damaged.remove_edge(*edge)
        assert nx.has_path(damaged,-1,source)
    unprotected,source = settled_backbone(False)
    assert nx.node_connectivity(unprotected,-1,source)==1


def test_movement_admission_preserves_second_path_while_column_deploys():
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint'))
    for uid,u in sim.state.uavs.items():
        if uid>=4:
            u.state=UAVState.FAILED
    positions=np.zeros((12,3))
    positions[:4]=[(40.,0.,40.),(80.,0.,40.),(120.,0.,40.),(160.,0.,40.)]
    ahead=positions.copy();ahead[3]=(215.,0.,40.)
    # Both positions retain the same four GCS-reachable vehicles, but racing
    # ahead leaves the scout with a bridge instead of two independent paths.
    assert sim._reachable_nodes(positions)==sim._reachable_nodes(ahead)=={0,1,2,3}
    assert sim._reachable_count(ahead,positions)<sim._reachable_count(positions)
    sim.scenario=replace(sim.scenario,relay_protection=False)
    assert sim._reachable_count(ahead,positions)==sim._reachable_count(positions)==4


def test_backup_handoff_can_change_edges_while_primary_path_is_retained():
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint'))
    for uid,u in sim.state.uavs.items():
        if uid>=4:
            u.state=UAVState.FAILED
    positions=np.zeros((12,3))
    positions[:4]=[(40.,0.,40.),(80.,0.,40.),(120.,0.,40.),(160.,0.,40.)]
    exchanged=positions.copy();exchanged[[1,2]]=exchanged[[2,1]]
    assert sim._reachable_count(exchanged)==sim._reachable_count(positions)
    # The protected endpoint may use new backup edges, while an end-to-end
    # primary path must remain in the intersection for the whole interval.
    assert sim._reachable_nodes(exchanged,positions)=={0,1,2,3}
    assert sim._reachable_count(exchanged,positions)==sim._reachable_count(positions)
    disconnected=positions.copy();disconnected[:4,0]+=500.
    assert sim._reachable_count(disconnected,positions)<sim._reachable_count(positions)


def test_moving_scout_cannot_outrun_its_second_relay_path():
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint'))
    for uid,u in sim.state.uavs.items():
        if uid>=4:
            u.state=UAVState.FAILED
        else:
            u.position=Vector3D(40.*(uid+1),0.,40.)
            sim.flights[uid].armed=True
            sim.flights[uid].landed=False
    sim.state.uavs[3].target_position=Vector3D(215.,0.,40.)
    for _ in range(30):
        sim._move();sim.state.time+=sim.dt
        positions={-1:(0.,0.,0.),**{i:u.position.to_array() for i,u in sim.state.uavs.items() if i<4}}
        graph=nx.Graph()
        graph.add_edges_from((i,j) for i,p in positions.items() for j,q in positions.items()
                             if i!=j and np.linalg.norm(np.array(p)-q)<=100+1e-8)
        assert nx.node_connectivity(graph,-1,3)>=2
        assert sim.interval_downtime==0.
    assert sim.state.uavs[3].position.x<=180.+1e-8
    assert sim.min_separation>=22.
    assert sim.flights[3].remaining_s==1185.


@pytest.mark.parametrize('kind',['radio_outage','vehicle_failure'])
def test_explicit_fault_affects_real_connectivity_and_preserves_failure(kind):
    fault = FaultSpec(kind,0.,0,None if kind=='vehicle_failure' else 2.)
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint',faults=(fault,)))
    sim.flights[0].armed=True;sim.flights[0].landed=False
    sim.state.uavs[0].position=Vector3D(50.,0.,40.)
    sim._network()
    assert sim.fault_records[0]['target']==0
    assert 0 not in sim.router.network_graph
    assert 0 not in sim._reachable_nodes(__import__('numpy').array([u.position.to_array() for u in sim.state.uavs.values()]))
    sim.state.time=2.
    sim._network()
    if kind=='radio_outage':
        assert 0 in sim.router.network_graph
        assert sim.fault_records[0]['cleared_s']==2.
    else:
        assert 0 not in sim.router.network_graph
        assert sim.state.uavs[0].state==UAVState.FAILED
        assert not sim.report()['checks']['all_uavs_landed']


def test_link_outage_blocks_selected_edge_without_disabling_endpoints():
    fault=FaultSpec('link_outage',0.,(-1,0),2.)
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint',faults=(fault,)))
    sim._network()
    assert 0 in sim.router.network_graph
    assert not sim.router.network_graph.has_edge(-1,0)
    assert (0,-1) not in sim.router.link_quality_cache
    sim.state.time=2.;sim._network()
    assert sim.router.network_graph.has_edge(-1,0)


def test_fault_after_early_mission_cannot_be_reported_exercised():
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint',faults=(FaultSpec('radio_outage',2600.,0,30.),)))
    assert not sim.report()['checks']['scheduled_faults_exercised']


def test_configured_charging_capacity_has_a_real_queue():
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint',charging_pads=1))
    for uid in (0,1):
        sim.flights[uid].remaining_s=600.
        sim.flights[uid].charge_started_s=0.
    sim._move()
    assert sim.flights[0].remaining_s>600
    assert sim.flights[1].remaining_s==600


def test_charging_pad_cannot_charge_two_aircraft_in_one_interval():
    sim=ResilientUAVXSimulator(near_config(camera_mode='footprint',charging_pads=1))
    sim.flights[0].remaining_s=1199.9
    sim.flights[1].remaining_s=600.
    for uid in (0,1): sim.flights[uid].charge_started_s=0.
    sim._move()
    assert sim.flights[0].remaining_s==1200.
    assert sim.flights[1].remaining_s==600.
    sim.state.time+=sim.dt;sim._move()
    assert sim.flights[1].remaining_s>600.


@pytest.fixture(scope='module')
def image_run(tmp_path_factory):
    folder=tmp_path_factory.mktemp('camera-mission')/'run'
    sim=ResilientUAVXSimulator(near_config())
    result=sim.run(folder,300)
    return folder,sim,result


def test_camera_image_reaches_gcs_as_complete_bytes(image_run):
    folder,sim,result=image_run
    assert result['checks']['imagery_received']
    assert result['checks']['all_uavs_landed']
    assert result['checks']['reporting_deadline']
    assert result['minimum_separation_m']>=22.
    capture=(folder/'sensing/captures/poi-0.png').read_bytes()
    received=(folder/'sensing/received/poi-0.png').read_bytes()
    assert received==capture and len(received)>1000
    metadata=json.loads((folder/'sensing/observations.json').read_text())['observations'][0]
    assert metadata['sha256']==hashlib.sha256(received).hexdigest()
    delivered=[e for e in sim.router.packet_events if e['type']=='delivered' and (e.get('payload') or {}).get('kind')=='poi_image_chunk']
    unique={}
    complete_at=None
    for event in delivered:
        payload=event['payload'];unique.setdefault(payload['chunk_index'],base64.b64decode(payload['data_b64']))
        if len(unique)==metadata['chunk_count']:
            complete_at=event['time_s'];break
    assert complete_at==metadata['received_at_s']==result['tasks'][0]['report_s']
    assert b''.join(unique[i] for i in sorted(unique))==received
    assert not verify(folder)['evidence_errors']


def test_one_fragment_does_not_count_as_image_receipt():
    sim=ResilientUAVXSimulator(near_config())
    try:
        sim.state.time=4.
        sim.state.uavs[0].position=Vector3D(175.,0.,40.)
        sim._capture(sim.tasks[0],0)
        task=sim.tasks[0];task.detection_s=4.;task.detected_by=0
        metadata=sim._observations[0]
        sim._accept_image_chunk(dict(source_id=0,time_s=5.,packet_id=0,payload=dict(kind='poi_image_chunk',poi_id=0,
            capture_id=metadata['capture_id'],observation_time_s=metadata['acquired_at_s'],
            detection_time_s=4.,image_sha256=metadata['sha256'],chunk_index=0,chunk_count=metadata['chunk_count'],
            data_b64=base64.b64encode(sim._image_bytes[0][:512]).decode('ascii'))))
        assert metadata['chunk_count']>1
        assert task.report_s is None and not sim._received_images
    finally:
        sim.close()


def test_reobserve_after_camera_carrier_failure_preserves_original_deadline(tmp_path):
    config=near_config(faults=(FaultSpec('vehicle_failure',57.5,3),))
    sim=ResilientUAVXSimulator(config)
    report=sim.run(tmp_path/'recapture',600)
    assert report['pois_completed']==1
    assert report['sensing']['captures']==2
    assert report['sensing']['received_images']==1
    assert report['tasks'][0]['detection_s']==57.5
    assert report['reporting_deadline_misses']==[0]
    assert not report['checks']['all_uavs_landed']
    assert report['survivor_metrics']['all_survivors_landed']
    assert not verify(tmp_path/'recapture')['evidence_errors']
