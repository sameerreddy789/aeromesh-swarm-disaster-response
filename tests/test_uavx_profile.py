"""Small end-to-end fixtures plus adversarial sample-profile boundaries."""
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
import pytest
from core.models import UAVRole, UAVState, Vector3D
from simulation.uavx import UAVXSimulator, SurveyTask, pair_distances


def near_task(uid=0, spawn=0):
    return SurveyTask(uid, (175., 0., 0.), spawn, 1.)


def test_survey_report_and_measured_home_landing(tmp_path):
    sim=UAVXSimulator(fleet_size=5, tasks=[near_task()], packet_loss=0)
    report=sim.run(tmp_path/'run', max_time=300)
    assert report['pois_completed']==1
    assert report['completion_time_s'] is not None
    assert report['checks']['all_uavs_landed']
    assert report['minimum_separation_m'] >= 20-1e-8
    assert report['checks']['speed'] and report['checks']['endurance']
    assert report['checks']['reporting_deadline']
    assert not report['sample_constraints_passed']  # one PoI != official ten
    detections=[e for e in sim.events if e['type']=='poi_detected']
    received=[e for e in sim.events if e['type']=='poi_received']
    assert received[0]['detection_time_s']==detections[0]['observation_time_s']
    assert 0 < received[0]['delay_s'] <= 10
    assert any(e['type']=='landing_confirmed' for e in sim.events)
    json.loads((tmp_path/'run'/'metrics.json').read_text())


def test_future_poi_prevents_premature_completion(tmp_path):
    sim=UAVXSimulator(fleet_size=5,tasks=[near_task(),near_task(1,250)],packet_loss=0)
    report=sim.run(tmp_path/'run',max_time=200)
    assert report['completion_time_s'] is None
    assert report['tasks'][1]['detection_s'] is None
    assert all(f.remaining_s>0 for f in sim.flights.values())


def test_empty_fleet_and_invalid_scenario_rejected():
    for kwargs in [dict(fleet_size=0),dict(tasks=[]),dict(dt=1),dict(packet_loss=1),
                   dict(tasks=[SurveyTask(0,(2000,0,0),0,1)])]:
        with pytest.raises(ValueError): UAVXSimulator(**kwargs)


def test_failed_vehicle_at_home_is_not_landed_success():
    sim=UAVXSimulator(fleet_size=1,tasks=[near_task()])
    sim.state.uavs[0].role=UAVRole.FAILED
    sim.state.uavs[0].state=UAVState.FAILED
    assert not sim.report()['checks']['all_uavs_landed']


def test_crossing_paths_detected_between_samples():
    assert np.min(pair_distances(np.array([[-20.,0,40],[20.,0,40]]),
                                 np.array([[20.,0,40],[-20.,0,40]])))==0


def test_unreported_detection_stays_failed_after_deadline():
    p=near_task();p.detection_s=0.;p.surveyed=True
    sim=UAVXSimulator(fleet_size=1,tasks=[p])
    sim.state.time=20
    assert sim.report()['reporting_deadline_misses']==[0]
    assert not sim.report()['checks']['reporting_deadline']


def test_range_cutoff_includes_three_dimensional_distance():
    sim=UAVXSimulator(fleet_size=1,tasks=[near_task()])
    sim.state.uavs[0].position=Vector3D(80,0,61)
    sim.router.update_network_topology()
    assert not sim.state.uavs[0].connected_to_gcs


def test_cli_runs_without_hardware_and_is_seed_reproducible(tmp_path):
    root=Path(__file__).resolve().parents[1]
    for label in ('a','b'):
        result=subprocess.run([sys.executable,'run_simulation.py','--profile','uavx',
                               '--max-time','3','--output',str(tmp_path/label)],
                              cwd=root,text=True,capture_output=True,timeout=30)
        assert result.returncode==0,result.stderr
    for name in ('metrics.json','events.json','packets.json','trajectory.jsonl'):
        assert (tmp_path/'a'/name).read_bytes()==(tmp_path/'b'/name).read_bytes()


def test_return_reconciles_inflight_replacement():
    sim=UAVXSimulator(fleet_size=5,tasks=[near_task()])
    sim.slots={0:(58.,0.,40.)};sim.slot_owners={0:4};sim.replacements={0:0}
    for uid in (0,4):
        sim.flights[uid].slot=0;sim.flights[uid].armed=True;sim.flights[uid].landed=False
    sim._return(4,'reserve')
    assert sim.slot_owners=={0:0}
    assert not sim.replacements
    sim._return(0,'reserve')
    assert not sim.slot_owners and sim.flights[0].slot is None
    assert sim.state.uavs[0].target_position.to_array().tolist()==list(sim.flights[0].home)


def test_vertical_return_can_escape_an_overhead_relay():
    sim=UAVXSimulator(fleet_size=2,tasks=[near_task()])
    sim.state.uavs[0].position=Vector3D(80,0,40)
    sim.state.uavs[1].position=Vector3D(80,0,65)
    f=sim.flights[0];f.armed=True;f.landed=False
    sim._return(0,'test')
    for _ in range(30):
        sim._move();sim.state.time+=sim.dt
    assert abs(sim.state.uavs[0].position.x-80)>10
    assert sim.separation_violations==0


def test_manifest_and_packet_receipts_checked(tmp_path):
    from verify_submission import verify
    sim=UAVXSimulator(fleet_size=2,tasks=[near_task()])
    sim.run(tmp_path/'run',max_time=3)
    result=verify(tmp_path/'run')
    assert not result['evidence_errors']
    assert not result['sample_run_passed']
    path=tmp_path/'run'/'packets.json'
    path.write_text('[]')
    assert verify(tmp_path/'run')['evidence_errors']


def test_returning_replacement_cannot_have_home_target_overwritten():
    sim=UAVXSimulator(fleet_size=5,tasks=[near_task()])
    sim.active_id=0;sim.slots={0:(58.,0.,40.)};sim.slot_owners={0:4};sim.replacements={0:0}
    for uid in (0,4):
        sim.flights[uid].slot=0;sim.flights[uid].armed=True;sim.flights[uid].landed=False
        sim.state.uavs[uid].position=Vector3D(58,0,65 if uid==0 else 40)
        sim.state.uavs[uid].connected_to_gcs=True
    sim.flights[0].remaining_s=200
    sim.flights[4].remaining_s=500
    sim._plan()
    assert sim.flights[0].returning
    assert sim.state.uavs[0].target_position.to_array().tolist()==list(sim.flights[0].home)
    assert not any(e['type']=='relay_handoff_completed' and e.get('new_uav_id')==0 for e in sim.events)


def test_landing_event_uses_supporting_observation_timestamp():
    sim=UAVXSimulator(fleet_size=1,tasks=[near_task()])
    f=sim.flights[0];f.armed=True;f.landed=False;f.returning=True;f.settled_s=1.5
    sim._move()
    landing=next(e for e in sim.events if e['type']=='landing_confirmed')
    assert landing['time_s']==landing['observed_at_s']==sim.dt


def test_decimal_step_landing_starts_charging_on_next_model_interval():
    sim=UAVXSimulator(fleet_size=1,dt=.1,tasks=[near_task()])
    sim.state.time=2.2
    flight=sim.flights[0]
    flight.armed,flight.landed,flight.returning=True,False,True
    flight.settled_s=2.
    flight.remaining_s=1200.
    sim._move()
    sim.state.time=round(sim.state.time+sim.dt,9)
    assert flight.landed and not flight.armed
    assert flight.observed_at_s==flight.charge_started_s==sim.state.time==2.3
    sim._move()
    assert flight.charge_started_s is None
    recharge=next(e for e in sim.events if e['type']=='recharge_completed')
    assert recharge['time_s']==2.4


@pytest.mark.parametrize('disturbance',['outage','failure'])
def test_fault_injection_records_event_and_removes_radio(disturbance):
    sim=UAVXSimulator(fleet_size=2,tasks=[near_task()],disturbance=disturbance)
    sim.state.time=900
    for uid,x in [(0,80),(1,160)]:
        sim.state.uavs[uid].position=Vector3D(x,0,40)
        sim.flights[uid].armed=True;sim.flights[uid].landed=False
    sim.state.uavs[0].role=UAVRole.RELAY
    sim.state.uavs[1].role=UAVRole.SCOUT
    sim._network()
    assert sim.disturbance_record['uav_id']==0
    assert not sim.state.uavs[1].connected_to_gcs
    event=next(e for e in sim.events if e['type']=='disturbance_injected')
    assert event['disturbance']==disturbance
    if disturbance=='outage':
        sim.state.time=930;sim._network()
        assert sim.state.uavs[1].connected_to_gcs
    else:
        assert not sim.state.uavs[0].is_operational()
        assert not sim.report()['checks']['all_uavs_landed']


def test_landing_approach_does_not_reclimb_after_lateral_avoidance():
    sim=UAVXSimulator(fleet_size=1,tasks=[near_task()])
    f=sim.flights[0];f.armed=True;f.landed=False
    sim.state.uavs[0].position=Vector3D(f.home[0],f.home[1],80)
    sim._return(0,'test');f.landing_approach=True
    sim.state.uavs[0].position.x+=2.5  # one avoidance sidestep
    sim._move()
    assert sim.state.uavs[0].position.z < 80
    assert f.landing_approach


def test_chain_shrinking_cancels_removed_slot_replacement():
    tasks=[SurveyTask(0,(575.,0.,0.),0.,1.),SurveyTask(1,(175.,0.,0.),1.,5.)]
    sim=UAVXSimulator(fleet_size=10,tasks=tasks)
    sim._network();sim._plan()
    sim.replacements[6]=9;sim.flights[9].slot=6
    sim.flights[9].armed=True;sim.flights[9].landed=False;sim.flights[9].remaining_s=400
    sim.state.time=1;sim._plan()
    assert 6 not in sim.slot_owners and 6 not in sim.replacements
    assert sim.flights[9].slot is None and sim.flights[9].returning


def test_radio_loss_dispatches_replacement_without_waiting_for_low_battery():
    sim=UAVXSimulator(fleet_size=5,tasks=[near_task()])
    sim._network();sim._plan()
    uid=sim.slot_owners[0]
    sim.router.network_graph.remove_node(uid)
    sim._plan()
    assert any(e['type']=='replacement_dispatched' and e['old_uav_id']==uid for e in sim.events)


def test_partition_with_neighbors_dispatches_replacement_from_actual_gcs_path():
    sim=UAVXSimulator(fleet_size=5,tasks=[near_task()])
    sim._network();sim._plan()
    uid=sim.slot_owners[0]
    partition={uid,sim.slot_owners[1]}
    graph=sim.router.network_graph
    graph.remove_edges_from([(a,b) for a,b in graph.edges if (a in partition)!=(b in partition)])
    assert graph.degree(uid)>0
    assert sim.state.uavs[uid].connected_to_gcs  # Cached flag predates the partition.
    assert sim.flights[uid].remaining_s==1200
    sim._plan()
    assert any(e['type']=='replacement_dispatched' and e['old_uav_id']==uid
               and e['reason']=='gcs_path_lost' for e in sim.events)


def test_healthy_multihop_path_does_not_dispatch_unnecessary_replacement():
    sim=UAVXSimulator(fleet_size=5,tasks=[near_task()])
    sim._network();sim._plan()
    graph=sim.router.network_graph
    graph.remove_edges_from(list(graph.edges))
    graph.add_edges_from([(-1,0),(0,1),(1,2),(2,3),(3,4)])
    for u in sim.state.uavs.values():
        u.connected_to_gcs=False  # Recovery decisions must use the actual graph.
    sim._plan()
    assert not sim.replacements
    assert not any(e['type']=='replacement_dispatched' for e in sim.events)


def test_missing_gcs_node_is_guarded_and_dispatches_path_recovery():
    sim=UAVXSimulator(fleet_size=5,tasks=[near_task()])
    sim._network();sim._plan()
    sim.router.network_graph.remove_node(-1)
    assert all(sim.router.network_graph.degree(uid)>0 for uid in sim.slot_owners.values())
    sim._plan()
    dispatched=[e for e in sim.events if e['type']=='replacement_dispatched']
    assert dispatched and all(e['reason']=='gcs_path_lost' for e in dispatched)


def test_motion_veto_enforces_speed_even_with_invalid_slot_ratio():
    sim=UAVXSimulator(fleet_size=2,tasks=[near_task()])
    sim.slots={0:(60,0,40),1:(120,0,40),2:(175,0,40)}
    sim.state.uavs[0].position=Vector3D(0,0,40)
    sim.state.uavs[0].target_position=Vector3D(175,0,40)
    sim.flights[0].slot=6;sim.flights[0].armed=True;sim.flights[0].landed=False
    sim._move()
    assert sim.max_speed <= 5+1e-8
