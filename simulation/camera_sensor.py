"""Traceable RGB captures from an offscreen procedural disaster scene.

Panda3D's CPU TinyDisplay renderer supplies an actual pinhole camera image at
the caller's measured simulation pose. This module never moves a UAV. The
scenery and observations are simulated; neither physics flight, a physical
camera nor learned survivor recognition is claimed.
"""
from dataclasses import dataclass
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path
import random
from typing import Mapping, Optional, Sequence


@dataclass(frozen=True)
class CameraCapture:
    image_bytes: bytes
    metadata: dict


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')


def _number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if positive and value <= 0:
        raise ValueError(f'{name} must be positive')
    return float(value)


class DisasterCamera:
    """One independent, display-free scene renderer.

    ``tasks`` accepts objects with ``id``, ``position``, ``spawn_s`` or mappings
    with those keys. Task parcels appear only at their emergence timestamp.
    ``output_dir=None`` keeps every capture in memory. With an output directory,
    content-identified PNGs and JSON sidecars are saved under ``frames/``.
    """

    SCENE_VERSION = 'aeromesh-disaster-parcels-v1'
    EVIDENCE_SCOPE = ('Panda3D TinyDisplay RGB rendering of a procedural disaster scene; '
                      'camera pose supplied by the kinematic simulator; not physical-flight '
                      'or physical-camera qualification')

    def __init__(self, tasks, seed=42, width=128, height=128, output_dir=None):
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ValueError('seed must be an integer')
        for name, value in [('width', width), ('height', height)]:
            if isinstance(value, bool) or not isinstance(value, int) or not 32 <= value <= 512:
                raise ValueError(f'{name} must be an integer from 32 to 512')
        self.width, self.height, self.seed = width, height, seed
        self.output_dir = Path(output_dir) if output_dir is not None else None
        self.tasks = []
        for item in tasks:
            read = item.__getitem__ if isinstance(item, Mapping) else lambda key: getattr(item, key)
            identity = read('id')
            if isinstance(identity, bool) or not isinstance(identity, int):
                raise ValueError('task id must be an integer')
            position = tuple(read('position'))
            if len(position) != 3:
                raise ValueError('task position must contain three coordinates')
            position = tuple(_number(v, 'task position') for v in position)
            spawn = _number(read('spawn_s'), 'task spawn_s')
            if spawn < 0 or position[2] != 0:
                raise ValueError('task spawn_s must be nonnegative and task ground z must be zero')
            self.tasks.append({'id': identity, 'position': list(position), 'spawn_s': spawn})
        if len({t['id'] for t in self.tasks}) != len(self.tasks):
            raise ValueError('task ids must be unique')
        self.tasks.sort(key=lambda t: t['id'])
        self.closed = False
        self.engine = self.window = self.scene = None
        self._parcels = {}
        self._initialise_renderer()

    def _initialise_renderer(self):
        try:
            from panda3d import core as p3d
            from PIL import Image
        except ImportError as exc:
            raise RuntimeError('Scene camera dependencies unavailable; install requirements-sensing.txt') from exc
        self._p3d, self._Image = p3d, Image
        # Explicit module selection prevents fallback to a desktop/GPU pipe.
        pipe = p3d.GraphicsPipeSelection.getGlobalPtr().makeModulePipe('p3tinydisplay')
        if pipe is None:
            raise RuntimeError('Panda3D CPU TinyDisplay pipe is unavailable')
        self.engine = p3d.GraphicsEngine()
        fb = p3d.FrameBufferProperties()
        fb.setRgbColor(True)
        fb.setRgbaBits(8, 8, 8, 8)
        fb.setDepthBits(24)
        self.window = self.engine.makeOutput(
            pipe, 'AeroMesh offscreen camera', 0, fb,
            p3d.WindowProperties.size(self.width, self.height), p3d.GraphicsPipe.BFRefuseWindow)
        if self.window is None:
            self.close()
            raise RuntimeError('Panda3D could not create an offscreen CPU camera buffer')
        self.window.setClearColor((0.68, 0.73, 0.75, 1.0))
        self.window.setClearColorActive(True)
        self.scene = p3d.NodePath('disaster-scene')
        self.scene.setLightOff()
        self.lens = p3d.PerspectiveLens()
        self.vertical_fov_deg = 60.0
        self.horizontal_fov_deg = math.degrees(2 * math.atan(
            math.tan(math.radians(self.vertical_fov_deg / 2)) * self.width / self.height))
        self.lens.setFov(self.horizontal_fov_deg, self.vertical_fov_deg)
        self.lens.setNearFar(0.1, 1000.0)
        camera = p3d.Camera('downward-rgb-camera', self.lens)
        camera.setScene(self.scene)
        self.camera = self.scene.attachNewNode(camera)
        region = self.window.makeDisplayRegion()
        region.setCamera(self.camera)
        self.texture = p3d.Texture('rgb-readback')
        self.window.addRenderTexture(self.texture, p3d.GraphicsOutput.RTMCopyRam,
                                     p3d.GraphicsOutput.RTPColor)
        self._build_scene()
        self.scene_metadata = {
            'scene_version': self.SCENE_VERSION,
            'seed': self.seed,
            'tasks': self.tasks,
            'renderer': 'Panda3D TinyDisplay CPU',
            'renderer_version': p3d.PandaSystem.getVersionString(),
            'graphics_pipe': pipe.getType().getName(),
            'rendering_mode': 'offscreen; no graphics window; no external assets',
            'world_axes': 'Cartesian metres; x east, y north, z up',
            'scenery_visibility': 'Each disaster parcel becomes visible at task.spawn_s',
            'evidence_scope': self.EVIDENCE_SCOPE,
        }
        self.scene_sha256 = hashlib.sha256(_canonical(self.scene_metadata)).hexdigest()

    def _box(self, parent, name, center, size, color, hpr=(0, 0, 0)):
        """Six shaded quad faces, built in code so no asset download is needed."""
        p = self._p3d
        data = p.GeomVertexData(name, p.GeomVertexFormat.getV3c4(), p.Geom.UHStatic)
        vertices = p.GeomVertexWriter(data, 'vertex')
        colors = p.GeomVertexWriter(data, 'color')
        faces = [
            ([(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)], .64),
            ([(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)], 1.),
            ([(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], .73),
            ([(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)], .86),
            ([(1, 1, 0), (0, 1, 0), (0, 1, 1), (1, 1, 1)], .78),
            ([(0, 1, 0), (0, 0, 0), (0, 0, 1), (0, 1, 1)], .91),
        ]
        triangles = p.GeomTriangles(p.Geom.UHStatic)
        for face_index, (points, shade) in enumerate(faces):
            for point in points:
                vertices.addData3f(*[(coordinate - .5) for coordinate in point])
                colors.addData4f(*(channel * shade for channel in color), 1.0)
            start = face_index * 4
            triangles.addVertices(start, start + 1, start + 2)
            triangles.addVertices(start, start + 2, start + 3)
        triangles.closePrimitive()
        geometry = p.Geom(data)
        geometry.addPrimitive(triangles)
        node = p.GeomNode(name)
        node.addGeom(geometry)
        path = parent.attachNewNode(node)
        path.setPos(*center)
        path.setScale(*size)
        path.setHpr(*hpr)
        path.setTwoSided(True)
        return path

    def _build_scene(self):
        self._box(self.scene, 'terrain', (500, 0, -1), (1400, 1400, 2), (.50, .48, .37))
        # A modest road grid supplies orientation and context between parcels.
        for index in range(-5, 7):
            self._box(self.scene, f'east-road-{index}', (500, index * 100, .012),
                      (1300, 6, .02), (.28, .30, .31))
        for index in range(0, 12):
            self._box(self.scene, f'north-road-{index}', (index * 100, 0, .025),
                      (6, 1300, .02), (.28, .30, .31))
        for task in self.tasks:
            rng = random.Random(self.seed * 100003 + task['id'] * 104729)
            parcel = self.scene.attachNewNode(f'disaster-parcel-{task["id"]}')
            parcel.setPos(*task['position'])
            # Damaged masonry, tilted roof slabs and scattered debris form a
            # three-dimensional disaster site, not a PoI symbol overlaid on a map.
            self._box(parcel, 'concrete-foundation', (0, 0, .16), (18, 15, .3), (.57, .57, .53))
            for x, y, sx, sy, height in [(-8, 0, .8, 15, 4.7), (0, 7, 17, .7, 3.5),
                                         (8, 3, .6, 7, 2.8), (-1, -6, 8, .8, 1.8)]:
                self._box(parcel, 'broken-wall', (x, y, height / 2), (sx, sy, height),
                          (.65 + rng.uniform(-.07, .07), .47, .34))
            self._box(parcel, 'fallen-roof', (2, 1, 2.2), (12, 6, .5), (.33, .35, .37),
                      (rng.uniform(-35, 35), rng.uniform(12, 25), rng.uniform(-20, 20)))
            for index in range(24):
                sx, sy, sz = rng.uniform(.5, 2.5), rng.uniform(.4, 2), rng.uniform(.2, 1.3)
                self._box(parcel, f'debris-{index}', (rng.uniform(-13, 13), rng.uniform(-11, 11), sz / 2),
                          (sx, sy, sz), (rng.uniform(.40, .68), rng.uniform(.34, .53), rng.uniform(.25, .41)),
                          (rng.uniform(0, 180), rng.uniform(-15, 15), rng.uniform(-15, 15)))
            self._box(parcel, 'relief-tarpaulin', (-11, 9, .15), (5, 4, .2), (.14, .35, .51))
            self._box(parcel, 'damaged-vehicle', (12, -7, .8), (2, 4.3, 1.5), (.57, .18, .15), (25, 0, 0))
            parcel.hide()
            self._parcels[task['id']] = parcel

    def get_scene_metadata(self):
        """JSON-safe configuration and fingerprint; contains scenario truth."""
        result = json.loads(_canonical(self.scene_metadata))
        result['scene_sha256'] = self.scene_sha256
        return result

    def capture(self, uav_id, position_xyz: Sequence[float], acquired_at_s, poi_id=None):
        if self.closed:
            raise RuntimeError('camera has been closed')
        if isinstance(uav_id, bool) or not isinstance(uav_id, int) or uav_id < 0:
            raise ValueError('uav_id must be a nonnegative integer')
        position = tuple(position_xyz)
        if len(position) != 3:
            raise ValueError('position_xyz must contain three coordinates')
        position = tuple(_number(v, 'camera position') for v in position)
        acquired = _number(acquired_at_s, 'acquired_at_s')
        if acquired < 0 or not .2 < position[2] < 900:
            raise ValueError('capture time must be nonnegative and camera height must be in (0.2, 900) m')
        if poi_id is not None and poi_id not in self._parcels:
            raise ValueError('unknown poi_id')
        active = []
        for task in self.tasks:
            if task['spawn_s'] <= acquired:
                self._parcels[task['id']].show()
                active.append(task['id'])
            else:
                self._parcels[task['id']].hide()
        self.camera.setPos(*position)
        self.camera.lookAt(self._p3d.Point3(position[0], position[1], 0), self._p3d.Vec3(0, 1, 0))
        self.engine.renderFrame()
        self.engine.syncFrame()
        raw = bytes(self.texture.getRamImageAs('RGB'))
        if len(raw) != self.width * self.height * 3:
            raise RuntimeError('camera readback is incomplete')
        # Panda's RAM texture rows start at the bottom. This image conversion is
        # explicit; optical image coordinates below start at the top left.
        image = self._Image.frombytes('RGB', (self.width, self.height), raw)
        image = image.transpose(self._Image.Transpose.FLIP_TOP_BOTTOM)
        buffer = BytesIO()
        image.save(buffer, format='PNG', optimize=True)
        image_bytes = buffer.getvalue()
        image_hash = hashlib.sha256(image_bytes).hexdigest()
        focal = self.height / (2 * math.tan(math.radians(self.vertical_fov_deg / 2)))
        metadata = {
            'schema_version': 1,
            'uav_id': uav_id,
            'poi_id': poi_id,
            'acquired_at_s': acquired,
            'media_type': 'image/png',
            'width': self.width,
            'height': self.height,
            'byte_length': len(image_bytes),
            'sha256': image_hash,
            'position_xyz_m': list(position),
            'camera_direction_world': [0., 0., -1.],
            'image_up_world': [0., 1., 0.],
            'world_from_optical': [[1., 0., 0., position[0]], [0., -1., 0., position[1]],
                                   [0., 0., -1., position[2]], [0., 0., 0., 1.]],
            'intrinsics': {'fx': focal, 'fy': focal, 'cx': self.width / 2., 'cy': self.height / 2.,
                           'vertical_fov_deg': self.vertical_fov_deg,
                           'horizontal_fov_deg': self.horizontal_fov_deg,
                           'model': 'configured ideal pinhole; zero distortion',
                           'pixel_coordinates': 'top-left origin; pixel centres at (i+0.5,j+0.5)'},
            'scene_sha256': self.scene_sha256,
            'active_scene_parcels': active,
            'renderer': self.scene_metadata['renderer'],
            'renderer_version': self.scene_metadata['renderer_version'],
            'evidence_scope': self.EVIDENCE_SCOPE,
        }
        metadata['capture_id'] = hashlib.sha256(_canonical(metadata)).hexdigest()[:24]
        if self.output_dir is not None:
            frames = self.output_dir / 'frames'
            frames.mkdir(parents=True, exist_ok=True)
            relative = Path('frames') / (metadata['capture_id'] + '.png')
            metadata['image_path'] = relative.as_posix()
            image_path = self.output_dir / relative
            if image_path.exists() and image_path.read_bytes() != image_bytes:
                raise FileExistsError(f'different data already exists at {image_path}')
            image_path.write_bytes(image_bytes)
            image_path.with_suffix('.json').write_bytes(_canonical(metadata) + b'\n')
        return CameraCapture(image_bytes=image_bytes, metadata=metadata)

    def close(self):
        if self.engine is not None:
            self.engine.removeAllWindows()
        if self.scene is not None:
            self.scene.removeNode()
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()

