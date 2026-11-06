"""Actual offscreen RGB scene captures, not plotted map markers."""
import hashlib
from io import BytesIO
import json

import pytest

pytest.importorskip('panda3d')
from PIL import Image
from simulation.camera_sensor import DisasterCamera


TASKS = [{'id': 0, 'position': (100., 50., 0.), 'spawn_s': 0.},
         {'id': 1, 'position': (220., 120., 0.), 'spawn_s': 100.}]


def test_capture_has_real_scene_pixels_and_traceable_bytes(tmp_path):
    with DisasterCamera(TASKS, output_dir=tmp_path) as camera:
        shot = camera.capture(3, (100., 50., 40.), 1., poi_id=0)
        metadata = shot.metadata
        image = Image.open(BytesIO(shot.image_bytes)).convert('RGB')
        assert image.size == (128, 128)
        assert len(image.getcolors(128 * 128)) > 20
        assert hashlib.sha256(shot.image_bytes).hexdigest() == metadata['sha256']
        assert len(shot.image_bytes) == metadata['byte_length']
        assert len(shot.image_bytes) < 8192
        assert metadata['position_xyz_m'] == [100., 50., 40.]
        assert metadata['uav_id'] == 3 and metadata['acquired_at_s'] == 1.
        assert metadata['renderer'] == 'Panda3D TinyDisplay CPU'
        assert metadata['renderer_version'] == '1.10.16'
        assert metadata['active_scene_parcels'] == [0]
        path = tmp_path / metadata['image_path']
        assert path.read_bytes() == shot.image_bytes
        assert json.loads(path.with_suffix('.json').read_text()) == metadata
        assert camera.get_scene_metadata()['scene_sha256'] == metadata['scene_sha256']


def test_pose_changes_image_and_same_pose_is_deterministic():
    with DisasterCamera(TASKS) as camera:
        first = camera.capture(0, (100., 50., 40.), 1., poi_id=0)
        second = camera.capture(0, (100., 50., 40.), 1., poi_id=0)
        moved = camera.capture(0, (110., 52., 40.), 1., poi_id=0)
        assert first.image_bytes == second.image_bytes
        assert first.metadata == second.metadata
        assert first.image_bytes != moved.image_bytes
        assert 'image_path' not in first.metadata
    with DisasterCamera(TASKS) as camera:
        assert camera.capture(0, (100., 50., 40.), 1., poi_id=0).image_bytes == first.image_bytes


def test_future_disaster_parcel_does_not_appear_early():
    with DisasterCamera(TASKS) as camera:
        before = camera.capture(1, (220., 120., 40.), 99., poi_id=1)
        after = camera.capture(1, (220., 120., 40.), 100., poi_id=1)
        replay_before = camera.capture(1, (220., 120., 40.), 99., poi_id=1)
        assert before.image_bytes != after.image_bytes
        assert before.image_bytes == replay_before.image_bytes
        assert before.metadata['active_scene_parcels'] == [0]
        assert after.metadata['active_scene_parcels'] == [0, 1]


def test_closed_camera_and_invalid_capture_inputs_are_rejected():
    with DisasterCamera(TASKS) as camera:
        for position, stamp in [((100, 50, 0), 1), ((100, float('nan'), 40), 1),
                                ((100, 50, 40), -1), ((100, 50, 40), float('inf'))]:
            with pytest.raises(ValueError):
                camera.capture(0, position, stamp)
        with pytest.raises(ValueError):
            camera.capture(0, (100, 50, 40), 1, poi_id=999)
    with pytest.raises(RuntimeError, match='closed'):
        camera.capture(0, (100, 50, 40), 1)
    camera.close()
