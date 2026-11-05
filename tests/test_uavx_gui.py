"""The live UI must observe the real mission without changing its outcomes."""
import json
from copy import deepcopy
from io import BytesIO
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
pygame = pytest.importorskip('pygame')
from core.models import Vector3D
from simulation.uavx import UAVXSimulator
from visualization.uavx_gui import UAVXVisualizer, visible_links, latest_received_camera
from verify_submission import verify


def test_gui_observes_same_engine_and_preserves_evidence(tmp_path):
    baseline = UAVXSimulator(fleet_size=5)
    baseline.run(tmp_path/'headless', max_time=3)
    live = UAVXSimulator(fleet_size=5)
    viewer = UAVXVisualizer(real_time=False)
    try:
        report = live.run(tmp_path/'gui', max_time=3, observer=viewer)
        for name in ('metrics.json','events.json','packets.json','trajectory.jsonl','connectivity.json'):
            assert (tmp_path/'headless'/name).read_bytes() == (tmp_path/'gui'/name).read_bytes()
        assert report['elapsed_time_s'] == 3
        assert not verify(tmp_path/'gui')['evidence_errors']
        status=json.loads((tmp_path/'gui/viewer/live.json').read_text())
        assert status['time_s']==3
        assert status['positions']['0']==live.state.uavs[0].position.to_array().tolist()
        assert (tmp_path/'gui/viewer/final.png').stat().st_size > 1000
    finally:
        viewer.close()


def test_gui_shows_edges_not_fictitious_direct_gcs_links():
    sim=UAVXSimulator(fleet_size=2)
    sim.state.uavs[0].position=Vector3D(80,0,40)
    sim.state.uavs[1].position=Vector3D(160,0,40)
    sim._network()
    assert sim.state.uavs[1].connected_to_gcs
    edges={frozenset(e) for e in visible_links(sim)}
    assert frozenset((-1,0)) in edges
    assert frozenset((0,1)) in edges
    assert frozenset((-1,1)) not in edges
    sim.router.network_graph.remove_node(0)
    assert visible_links(sim)==[]


def test_closing_gui_records_incomplete_run_without_false_success(tmp_path):
    sim=UAVXSimulator(fleet_size=5)
    viewer=UAVXVisualizer(real_time=False)
    try:
        viewer.closed=True
        report=sim.run(tmp_path/'closed',max_time=30,observer=viewer)
        assert report['elapsed_time_s']==0
        assert not report['sample_constraints_passed']
        assert not verify(tmp_path/'closed')['evidence_errors']
    finally:
        viewer.close()


def test_gui_controls_do_not_change_simulation_constraints(tmp_path):
    sim=UAVXSimulator(fleet_size=5)
    viewer=UAVXVisualizer(real_time=False)
    try:
        viewer.start(sim,tmp_path)
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN,key=pygame.K_SPACE))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN,key=pygame.K_EQUALS))
        viewer.handle_events()
        assert viewer.paused and viewer.speed==24
        assert sim.state.time==0
        assert sim.constraints.max_speed_mps==5
    finally:
        viewer.close()


def test_visualization_import_does_not_require_web_or_plot_dependencies():
    root=Path(__file__).resolve().parents[1]
    command="import sys, visualization; assert 'flask' not in sys.modules; assert 'matplotlib' not in sys.modules"
    result=subprocess.run([sys.executable,'-c',command],cwd=root,capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def camera_fixture():
    sim = UAVXSimulator(fleet_size=5)
    sim.state.time = 12.
    sim.scenario = SimpleNamespace(camera_mode='panda3d', faults=(object(),))
    sim.fault_records = []
    sim._observations = {0: dict(acquired_at_s=2., received_at_s=None)}
    sim._received_images = {}
    return sim


def png_bytes(color):
    surface = pygame.Surface((24, 16))
    surface.fill(color)
    stream = BytesIO()
    pygame.image.save(surface, stream, 'fixture.png')
    return stream.getvalue()


def test_camera_preview_waits_for_complete_gcs_receipt(tmp_path):
    sim = camera_fixture()
    # Onboard imagery and scene truth must never become a GCS preview.
    sim._image_bytes = {0: png_bytes((240, 20, 20))}
    sim._camera = SimpleNamespace(capture=lambda *a, **k: pytest.fail('viewer invoked sensor'))
    viewer = UAVXVisualizer(real_time=False)
    try:
        viewer.start(sim, tmp_path)
        assert latest_received_camera(sim) is None
        assert viewer._camera_surface is None
        status = json.loads((tmp_path/'viewer/live.json').read_text())
        assert status['gcs_camera'] is None
    finally:
        viewer.close()


@pytest.mark.parametrize('width,height', [(1400, 900), (1050, 720)])
def test_received_camera_pixels_and_ages_are_shown_without_changing_flight_state(tmp_path, width, height):
    sim = camera_fixture()
    sim._image_bytes = {0: png_bytes((240, 20, 20))}
    sim._received_images[0] = png_bytes((20, 30, 240))
    sim._observations[0]['received_at_s'] = 10.
    before = (deepcopy(sim.flights), deepcopy(sim._observations),
              {uid: u.position.to_array().tolist() for uid,u in sim.state.uavs.items()}, sim.state.time)
    viewer = UAVXVisualizer(real_time=False, width=width, height=height)
    try:
        viewer.start(sim, tmp_path)
        assert viewer._camera_surface.get_at((2,2))[:3] == (20, 30, 240)
        status = json.loads((tmp_path/'viewer/live.json').read_text())
        assert status['gcs_camera'] == dict(poi_id=0, acquired_at_s=2., received_at_s=10.,
                                             capture_age_s=10., receipt_age_s=2.)
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_v))
        viewer.handle_events()
        assert not viewer.show_camera
        viewer.render()
        after = (sim.flights, sim._observations,
                 {uid: u.position.to_array().tolist() for uid,u in sim.state.uavs.items()}, sim.state.time)
        assert before == after
    finally:
        viewer.close()


def test_camera_preview_rejects_missing_or_future_receipt_metadata():
    sim = camera_fixture()
    sim._received_images[0] = b'not_yet_validated'
    assert latest_received_camera(sim) is None
    sim._observations[0]['received_at_s'] = 13.
    assert latest_received_camera(sim) is None
    sim._observations[0]['received_at_s'] = 1.
    assert latest_received_camera(sim) is None


def test_camera_preview_tracks_latest_receipt_and_handles_decode_error(tmp_path):
    sim = camera_fixture()
    sim._observations[0]['received_at_s'] = 10.
    sim._received_images[0] = png_bytes((0, 100, 200))
    sim._observations[1] = dict(acquired_at_s=8., received_at_s=11.)
    sim._received_images[1] = b'invalid_png'
    viewer = UAVXVisualizer(real_time=False)
    try:
        viewer.start(sim, tmp_path)
        assert latest_received_camera(sim)['poi_id'] == 1
        assert viewer._camera_surface is None
        assert not viewer.closed
    finally:
        viewer.close()
