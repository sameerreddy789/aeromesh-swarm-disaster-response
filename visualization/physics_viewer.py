"""Asset-free 3D observer for the AeroMesh rigid-body physics profile.

The viewer never advances Bullet, commands aircraft, or writes a physical body
transform. Drone geometry is parented to the real Bullet NodePaths. Camera,
labels, shadows and illustrative propeller spin are presentation only; no motor
RPM or airframe calibration is claimed. ``DisasterCamera`` owns a separate
GraphicsEngine and is not used or reconfigured by this observer.
"""
from collections import deque
import json
import math
from pathlib import Path
import time


class PhysicsVisualizer:
    """Native Panda3D observer implementing ``UAVXSimulator.run``'s contract."""

    BG = (0.047, 0.075, 0.114, 1)
    PANEL = (0.078, 0.118, 0.169, .97)
    TEXT = (.906, .941, .969, 1)
    MUTED = (.569, .639, .702, 1)
    CYAN = (.282, .753, .886, 1)
    GREEN = (.349, .820, .608, 1)
    AMBER = (.957, .718, .345, 1)
    RED = (.945, .424, .451, 1)

    def __init__(self, speed=4., width=1400, height=900, real_time=True,
                 offscreen=False):
        if not math.isfinite(speed) or not .25 <= speed <= 100:
            raise ValueError('Viewer speed must be between 0.25 and 100')
        if width < 640 or height < 480:
            raise ValueError('Viewer dimensions must be at least 640 x 480')
        self.speed, self.real_time = float(speed), bool(real_time)
        self.width, self.height, self.offscreen = width, height, offscreen
        self.base = self.sim = self.output = self.final_report = None
        self.closed = self.paused = self.restart_requested = False
        self.show_links = True
        self.selected = None
        self.follow_selected = False
        self.focus = [500., 0., 0.]
        self.distance, self.azimuth, self.elevation = 2200., 0., 67.
        self._drag = self._last_mouse = None
        self._owned_visuals, self._rotors, self._indicators = [], {}, {}
        self._markers, self._shadows, self._pois = {}, {}, {}
        self._hud, self._hud_cards = {}, []
        self._last_size = None
        self._frames = deque(maxlen=60)
        self._last_render = self._last_save = self._next_step = 0.
        self._started_wall = time.monotonic()
        self._last_save_error = None
        self._captures = set()

    def start(self, sim, output):
        if self.base is not None:
            raise RuntimeError('Use a fresh PhysicsVisualizer for a new physics world')
        if not hasattr(sim, 'physics') or not hasattr(sim.physics, 'bodies'):
            raise TypeError('PhysicsVisualizer requires a simulator with real physics bodies')
        from panda3d import core as p
        from direct.showbase.ShowBase import ShowBase
        self.p = p
        p.loadPrcFileData('aeromesh-physics-viewer', '\n'.join([
            'window-title AeroMesh UAV-X | 3D Physics Swarm',
            f'win-size {self.width} {self.height}',
            'audio-library-name null', 'sync-video false',
            'show-frame-rate-meter false',
            'framebuffer-multisample true', 'multisamples 2',
        ]))
        self.base = ShowBase(windowType='none')
        # The sensing camera may have loaded TinyDisplay first. Select the GPU
        # pipe explicitly so its CPU backend does not silently own this window.
        pipe = p.GraphicsPipeSelection.getGlobalPtr().makeModulePipe('pandagl')
        if pipe is None:
            pipe = p.GraphicsPipeSelection.getGlobalPtr().makeModulePipe('p3tinydisplay')
        self.base.openDefaultWindow(pipe=pipe, type='offscreen' if self.offscreen else 'onscreen')
        if self.base.win is None:
            self.base.destroy()
            self.base = None
            raise RuntimeError('Panda3D could not create the physics viewer window')
        self.base.userExit = self._request_close
        self.base.disableMouse()
        self.base.setBackgroundColor(*self.BG)
        self.base.camLens.setNearFar(.05, 12000.)
        self.base.camLens.setFov(55.)
        self.sim, self.output = sim, Path(output) / 'viewer'
        self.output.mkdir(parents=True, exist_ok=False)
        self._started_wall = self._next_step = time.monotonic()
        self.selected = min(sim.flights) if sim.flights else None
        self.scene = self.base.render.attachNewNode('observer-scene')
        # Reparent only the identity world root. Bullet keeps exclusive authority
        # over every physical body's position, orientation and velocity.
        self._world_parent = sim.physics.root.getParent()
        sim.physics.root.reparentTo(self.base.render)
        self._setup_lights()
        self._build_ground()
        radius = float(sim.physics.config.body_radius_m)
        for uid, body in sim.physics.bodies.items():
            self._build_drone(uid, body, radius)
        self._build_pois()
        self.link_node = None
        self._setup_hud()
        self._bind_controls()
        self._fit_active()
        self.render()
        self.save_live(force=True)

    def _setup_lights(self):
        p = self.p
        ambient = p.AmbientLight('observer-ambient')
        ambient.setColor((.55, .61, .67, 1))
        sun = p.DirectionalLight('observer-sun')
        sun.setColor((.86, .84, .78, 1))
        ambient_np = self.scene.attachNewNode(ambient)
        sun_np = self.scene.attachNewNode(sun)
        sun_np.setHpr(-25, -62, 0)
        self.base.render.setLight(ambient_np)
        self.base.render.setLight(sun_np)

    def _box(self, parent, name, center, size, color):
        p = self.p
        data = p.GeomVertexData(name, p.GeomVertexFormat.getV3n3c4(), p.Geom.UHStatic)
        vertex, normal, colors = [p.GeomVertexWriter(data, field)
                                  for field in ('vertex', 'normal', 'color')]
        faces = [
            ((0, 0, -1), [(0,0,0),(0,1,0),(1,1,0),(1,0,0)]),
            ((0, 0, 1), [(0,0,1),(1,0,1),(1,1,1),(0,1,1)]),
            ((0, -1, 0), [(0,0,0),(1,0,0),(1,0,1),(0,0,1)]),
            ((1, 0, 0), [(1,0,0),(1,1,0),(1,1,1),(1,0,1)]),
            ((0, 1, 0), [(1,1,0),(0,1,0),(0,1,1),(1,1,1)]),
            ((-1, 0, 0), [(0,1,0),(0,0,0),(0,0,1),(0,1,1)]),
        ]
        triangles = p.GeomTriangles(p.Geom.UHStatic)
        for index, (n, points) in enumerate(faces):
            for point in points:
                vertex.addData3f(*(value-.5 for value in point))
                normal.addData3f(*n)
                colors.addData4f(*color)
            a = 4*index
            triangles.addVertices(a, a+1, a+2)
            triangles.addVertices(a, a+2, a+3)
        geom = p.Geom(data)
        geom.addPrimitive(triangles)
        node = p.GeomNode(name)
        node.addGeom(geom)
        result = parent.attachNewNode(node)
        result.setPos(*center)
        result.setScale(*size)
        return result

    def _cylinder(self, parent, name, radius, height, color, segments=16):
        p = self.p
        data = p.GeomVertexData(name, p.GeomVertexFormat.getV3n3c4(), p.Geom.UHStatic)
        vertex, normal, colors = [p.GeomVertexWriter(data, field)
                                  for field in ('vertex', 'normal', 'color')]
        triangles = p.GeomTriangles(p.Geom.UHStatic)
        for index in range(segments):
            a, b = 2*math.pi*index/segments, 2*math.pi*(index+1)/segments
            ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
            points = [((radius*ca,radius*sa,-height/2),(ca,sa,0)),
                      ((radius*cb,radius*sb,-height/2),(cb,sb,0)),
                      ((radius*cb,radius*sb,height/2),(cb,sb,0)),
                      ((radius*ca,radius*sa,height/2),(ca,sa,0)),
                      ((0,0,height/2),(0,0,1)),
                      ((radius*ca,radius*sa,height/2),(0,0,1)),
                      ((radius*cb,radius*sb,height/2),(0,0,1))]
            for point, n in points:
                vertex.addData3f(*point)
                normal.addData3f(*n)
                colors.addData4f(*color)
            offset = index*7
            triangles.addVertices(offset,offset+1,offset+2)
            triangles.addVertices(offset,offset+2,offset+3)
            triangles.addVertices(offset+4,offset+5,offset+6)
        geom = p.Geom(data)
        geom.addPrimitive(triangles)
        node = p.GeomNode(name)
        node.addGeom(geom)
        return parent.attachNewNode(node)

    def _lines(self, parent, name, segments, color, width=1.):
        lines = self.p.LineSegs(name)
        lines.setColor(*color)
        lines.setThickness(width)
        for a, b in segments:
            lines.moveTo(*a)
            lines.drawTo(*b)
        path = parent.attachNewNode(lines.create())
        path.setLightOff()
        return path

    def _ring(self, parent, name, radius, color, width=1.5):
        points = [(radius*math.cos(i*math.pi/24), radius*math.sin(i*math.pi/24), 0.)
                  for i in range(49)]
        return self._lines(parent, name, zip(points[:-1], points[1:]), color, width)

    def _build_ground(self):
        self._box(self.scene, 'flat-ground-visual', (500,0,-.14), (1400,1250,.25),
                  (.24,.31,.30,1))
        segments = [((x,-500,.015),(x,500,.015)) for x in range(75,1076,100)]
        segments += [((75,y,.015),(1075,y,.015)) for y in range(-500,501,100)]
        self._lines(self.scene, '100m-grid', segments, (.38,.46,.45,1), 1.)
        corners = [(75,-500,.04),(1075,-500,.04),(1075,500,.04),(75,500,.04),(75,-500,.04)]
        self._lines(self.scene, 'task-area-boundary', zip(corners[:-1],corners[1:]),
                    self.CYAN, 2.)
        for uid, flight in self.sim.flights.items():
            pad = self.scene.attachNewNode(f'home-pad-{uid}')
            pad.setPos(flight.home[0], flight.home[1], .035)
            self._ring(pad, 'pad-boundary', 5., (.52,.60,.61,1), 1.5)
            self._lines(pad, 'pad-H', [((-1.5,-2,0),(-1.5,2,0)),
                                     ((1.5,-2,0),(1.5,2,0)),
                                     ((-1.5,0,0),(1.5,0,0))], self.MUTED, 1.2)
        gcs = self.sim.state.gcs.position
        self._box(self.scene, 'gcs-base', (gcs.x,gcs.y,1.2),(3,3,2.4),(.16,.24,.28,1))
        mast = self._cylinder(self.scene,'gcs-antenna',.18,7.,self.TEXT)
        mast.setPos(gcs.x,gcs.y,5.8)
        self._ring(self.scene, 'gcs-ground-ring', 9., self.GREEN, 2.).setPos(gcs.x,gcs.y,.08)

    def _build_drone(self, uid, body, r):
        # All transformations below apply only to newly-created visual children.
        visual = body.attachNewNode(f'uav-{uid}-visual-components')
        self._owned_visuals.append(visual)
        white, graphite = (.78,.84,.87,1), (.10,.14,.18,1)
        self._box(visual,'fuselage',(0,0,0),(.60*r,.85*r,.30*r),white)
        self._box(visual,'battery',(0,-.08*r,.20*r),(.45*r,.56*r,.18*r),graphite)
        self._box(visual,'front-camera',(0,.46*r,-.03*r),(.23*r,.19*r,.19*r),graphite)
        self._box(visual,'camera-glass',(0,.565*r,-.03*r),(.15*r,.022*r,.11*r),(.18,.43,.51,1))
        self._indicators[uid] = self._box(visual,'status-light',(0,.26*r,.175*r),
                                          (.22*r,.10*r,.018*r),self.TEXT)
        rotors = []
        arm_length = float(self.sim.physics.config.arm_length_m)
        for index,(sx,sy) in enumerate(((1,1),(-1,1),(-1,-1),(1,-1))):
            x,y = arm_length/math.sqrt(2)*sx,arm_length/math.sqrt(2)*sy
            arm = self._box(visual,f'arm-{index}',(x/2,y/2,0),(.10*r,arm_length,.11*r),graphite)
            arm.setH(-45 if sx==sy else 45)
            motor = self._cylinder(visual,f'motor-{index}',.095*r,.17*r,graphite)
            motor.setPos(x,y,.105*r)
            rotor = visual.attachNewNode(f'illustrative-propeller-{index}')
            rotor.setPos(x,y,.205*r)
            self._box(rotor,'blade-a',(0,0,0),(.49*r,.045*r,.014*r),(.20,.24,.27,1))
            self._box(rotor,'blade-b',(0,0,.004*r),(.045*r,.49*r,.014*r),(.20,.24,.27,1))
            rotors.append(rotor)
            self._box(visual,f'landing-leg-{index}',(.32*r*sx,.29*r*sy,-.47*r),
                      (.055*r,.055*r,.70*r),graphite)
        for sx in (-1,1):
            self._box(visual,'landing-skid',(.32*r*sx,0,-.84*r),(.07*r,.90*r,.07*r),white)
        self._rotors[uid] = rotors
        marker = self.scene.attachNewNode(f'telemetry-marker-{uid}')
        self._ring(marker,'screen-facing-ring',1.,self.TEXT,2.).setP(90)
        marker.setBillboardPointEye()
        marker.setDepthWrite(False)
        self._markers[uid] = marker
        shadow = self._cylinder(self.scene,f'ground-projection-{uid}',r, .005,(.09,.14,.14,1),24)
        shadow.setLightOff()
        self._shadows[uid] = shadow

    def _build_pois(self):
        for task in self.sim.tasks:
            node = self.scene.attachNewNode(f'poi-annotation-{task.id}')
            node.setPos(task.position[0],task.position[1],.09)
            self._ring(node,'poi-ring',8.,self.TEXT,2.)
            self._lines(node,'poi-cross',[((-3,0,0),(3,0,0)),((0,-3,0),(0,3,0))],self.TEXT,1.5)
            if task.priority>1:
                self._ring(node,'priority-ring',12.,self.TEXT,1.2)
            self._pois[task.id] = node
            node.hide()

    def _load_font(self, names):
        for name in names:
            path = Path('C:/Windows/Fonts')/name
            if path.exists():
                font = self.base.loader.loadFont(str(self.p.Filename.fromOsSpecific(str(path))))
                if font:
                    return font
        return None

    def _setup_hud(self):
        self._fonts = {
            'title': self._load_font(('bahnschrift.ttf','segoeuib.ttf')),
            'body': self._load_font(('segoeui.ttf',)),
            'data': self._load_font(('consola.ttf','segoeui.ttf')),
        }
        self.hud_root = self.base.pixel2d.attachNewNode('observer-HUD')
        self.hud_root.setLightOff()
        self.hud_root.setDepthTest(False)
        self.hud_root.setDepthWrite(False)
        for key,size,kind in [('title',27,'title'),('scope',13,'body'),
                              ('status',17,'data'),('metrics',15,'data'),
                              ('selected',20,'title'),('details',14,'data'),
                              ('controls',13,'body'),('view',13,'data'),
                              ('result',15,'body')]:
            text = self.p.TextNode(key)
            text.setTextColor(*self.TEXT)
            if self._fonts[kind]:
                text.setFont(self._fonts[kind])
            path = self.hud_root.attachNewNode(text)
            path.setScale(size)
            self._hud[key] = (text,path)
        self._layout_hud()

    def _layout_hud(self):
        width,height = self.base.win.getXSize(),self.base.win.getYSize()
        if (width,height)==self._last_size:
            return
        self.width,self.height,self._last_size = width,height,(width,height)
        for card in self._hud_cards:
            card.removeNode()
        self._hud_cards=[]
        for name,x0,x1,y0,y1 in [('header',0,width,0,98),
                                 ('inspector',width-300,width-18,118,348),
                                 ('footer',0,width,height-44,height)]:
            card = self.p.CardMaker(name)
            card.setFrame(x0,x1,-y1,-y0)
            node = self.hud_root.attachNewNode(card.generate())
            node.setColor(*self.PANEL)
            node.setTransparency(self.p.TransparencyAttrib.MAlpha)
            node.setBin('fixed',0)
            self._hud_cards.append(node)
        positions = {'title':(24,36),'scope':(25,61),'status':(24,86),
                     'metrics':(max(440,width-675),36),
                     'selected':(width-283,149),'details':(width-283,177),
                     'controls':(24,height-17),'view':(max(600,width-385),height-17),
                     'result':(max(440,width-675),65)}
        for key,(x,y) in positions.items():
            self._hud[key][1].setPos(x,0,-y)
            self._hud[key][1].setBin('fixed',1)

    def _text(self, key, value, color=None):
        text = self._hud[key][0]
        if text.getText()!=value:
            text.setText(value)
        text.setTextColor(*(color or self.TEXT))

    def _bind_controls(self):
        for event,fn,args in [
            ('escape',self._request_close,[]),('space',self._toggle_pause,[]),
            ('+',self._change_speed,[2.]),('=',self._change_speed,[2.]),
            ('-',self._change_speed,[.5]),('wheel_up',self._zoom,[.85]),
            ('wheel_down',self._zoom,[1/.85]),('home',self._fit,[]),
            ('a',self._fit_active,[]),
            ('l',self._toggle_links,[]),('tab',self._select_next,[]),
            ('f',self._focus_selected,[]),('r',self._restart,[]),
            ('mouse3',self._begin_drag,['pan']),('mouse3-up',self._end_drag,[]),
            ('mouse2',self._begin_drag,['orbit']),('mouse2-up',self._end_drag,[]),
            ('mouse1',self._select_click,[])]:
            self.base.accept(event,fn,args)

    def _request_close(self):
        self.closed=True

    def _toggle_pause(self):
        self.paused=not self.paused
        self._next_step=time.monotonic()

    def _change_speed(self, factor):
        self.speed=max(.25,min(100.,self.speed*factor))

    def _zoom(self, factor):
        self.distance=max(1.5,min(4200.,self.distance*factor))

    def _fit(self):
        self.focus,self.distance,self.azimuth,self.elevation=[500.,0.,0.],2200.,0.,67.
        self.follow_selected=False

    def _fit_active(self):
        """Frame launch pads, aircraft and announced work; never future tasks."""
        points=[flight.home for flight in self.sim.flights.values()]
        points.extend(tuple(u.position.to_array()) for u in self.sim.state.uavs.values())
        points.extend(task.position for task in self.sim.tasks if task.spawn_s<=self.sim.state.time)
        if not points:
            self._fit()
            return
        left,right=min(p[0] for p in points),max(p[0] for p in points)
        bottom,top=min(p[1] for p in points),max(p[1] for p in points)
        self.focus=[(left+right)/2,(bottom+top)/2,0.]
        self.distance=max(180.,2.2*max(right-left,top-bottom))
        self.azimuth,self.elevation=0.,67.
        self.follow_selected=False

    def _toggle_links(self):
        self.show_links=not self.show_links

    def _select_next(self):
        ids=sorted(self.sim.flights)
        if ids:
            self.selected=ids[(ids.index(self.selected)+1)%len(ids)] if self.selected in ids else ids[0]

    def _focus_selected(self):
        if self.selected is not None:
            self.follow_selected=True
            self.distance,self.azimuth,self.elevation=3.4,35.,30.

    def _restart(self):
        if self.final_report is not None:
            self.restart_requested=True

    def _mouse(self):
        watcher=self.base.mouseWatcherNode
        if watcher is None or not watcher.hasMouse():
            return None
        point=watcher.getMouse()
        return point.x,point.y

    def _begin_drag(self, kind):
        self._drag,self._last_mouse=kind,self._mouse()

    def _end_drag(self):
        self._drag=self._last_mouse=None

    def _select_click(self):
        mouse=self._mouse()
        if mouse is None:
            return
        nearest=None
        for uid,uav in self.sim.state.uavs.items():
            pos=self.base.cam.getRelativePoint(self.base.render,self.p.Point3(*uav.position.to_array()))
            point=self.p.Point2()
            if self.base.camLens.project(pos,point):
                pixels=math.hypot((point.x-mouse[0])*self.width/2,
                                  (point.y-mouse[1])*self.height/2)
                if pixels<24 and (nearest is None or pixels<nearest[0]):
                    nearest=pixels,uid
        if nearest:
            self.selected=nearest[1]

    def _update_camera(self):
        mouse=self._mouse()
        if self._drag and mouse and self._last_mouse:
            dx,dy=mouse[0]-self._last_mouse[0],mouse[1]-self._last_mouse[1]
            if self._drag=='orbit':
                self.azimuth-=dx*85.
                self.elevation=max(8.,min(89.,self.elevation+dy*65.))
            else:
                self.follow_selected=False
                angle=math.radians(self.azimuth)
                shift=self.distance*.42
                self.focus[0]-=shift*(dx*math.cos(angle)+dy*math.sin(angle))
                self.focus[1]-=shift*(-dx*math.sin(angle)+dy*math.cos(angle))
        self._last_mouse=mouse
        if self.follow_selected and self.selected in self.sim.state.uavs:
            self.focus=list(map(float,self.sim.state.uavs[self.selected].position.to_array()))
        a,e=math.radians(self.azimuth),math.radians(self.elevation)
        horizontal=self.distance*math.cos(e)
        # A centimetre near plane at kilometre viewing distances loses enough
        # depth precision to make the ground grid flicker through the terrain.
        self.base.camLens.setNearFar(max(.03,min(30.,self.distance*.02)),
                                     max(1500.,self.distance*4.))
        self.base.camera.setPos(self.focus[0]+horizontal*math.sin(a),
                                self.focus[1]-horizontal*math.cos(a),
                                self.focus[2]+self.distance*math.sin(e))
        self.base.camera.lookAt(*self.focus)

    def _role_color(self, uid):
        uav,flight=self.sim.state.uavs[uid],self.sim.flights[uid]
        if not uav.is_operational(): return self.RED
        if flight.returning: return self.AMBER
        if not flight.armed: return self.MUTED
        return self.CYAN if uav.role.value=='relay' else self.GREEN

    def _update_scene(self):
        sim=self.sim
        for uid,uav in sim.state.uavs.items():
            color=self._role_color(uid)
            self._indicators[uid].setColorScale(*color)
            pos=tuple(map(float,uav.position.to_array()))
            marker=self._markers[uid]
            marker.setPos(*pos)
            camera_distance=(self.base.camera.getPos()-self.p.Point3(*pos)).length()
            marker.setScale(max(.3,camera_distance*.0028))
            marker.setColorScale(*(self.TEXT if uid==self.selected else color))
            if camera_distance<30: marker.hide()
            else: marker.show()
            self._shadows[uid].setPos(pos[0],pos[1],.012)
            observation=sim.physics.observe(uid)
            thrusts=getattr(observation,'rotor_thrusts_n',())
            if sim.flights[uid].armed and uav.is_operational():
                for index,rotor in enumerate(self._rotors[uid]):
                    if index<len(thrusts) and thrusts[index]>0:
                        rotor.setH((sim.state.time*1150*(1 if index%2 else -1))%360)
        for task in sim.tasks:
            node=self._pois[task.id]
            if task.spawn_s>sim.state.time:
                node.hide()
                continue
            node.show()
            complete=task.surveyed and task.report_s is not None
            node.setColorScale(*(self.GREEN if complete else self.AMBER if task.priority>1 else self.RED))
        if self.link_node is not None:
            self.link_node.removeNode()
            self.link_node=None
        if self.show_links:
            positions={uid:tuple(u.position.to_array()) for uid,u in sim.state.uavs.items()}
            positions[-1]=tuple(sim.state.gcs.position.to_array())
            segments=[]
            for a,b in sim.router.network_graph.edges():
                if a not in positions or b not in positions: continue
                if a!=-1 and b!=-1 and not(sim.flights[a].armed or sim.flights[b].armed): continue
                segments.append((positions[a],positions[b]))
            self.link_node=self._lines(self.scene,'actual-radio-links',segments,(.22,.59,.69,1),1.)

    def _update_hud(self):
        sim=self.sim
        airborne=sum(f.armed for f in sim.flights.values())
        completed=sum(p.surveyed and p.report_s is not None for p in sim.tasks)
        linked=sum(f.armed and sim.state.uavs[uid].connected_to_gcs for uid,f in sim.flights.items())
        mode='FINISHED' if self.final_report is not None else 'PAUSED' if self.paused else 'LIVE'
        self._text('title','AEROMESH  /  UAV-X')
        self._text('scope','RIGID-BODY PHYSICS  /  PARAMETERS UNCALIBRATED',self.MUTED)
        self._text('status',f'{mode}  Mission {sim.state.time:07.1f} / 2700 s  |  Physics {sim.physics.time_s:07.2f} s  |  {len(sim.flights)} UAVs',
                   self.AMBER if self.paused else self.GREEN)
        self._text('metrics',f'POIs {completed:02d}/{len(sim.tasks):02d}   AIRBORNE {airborne:02d}   GCS LINK {linked:02d}/{airborne:02d}')
        if self.final_report is not None:
            checks=self.final_report.get('checks',{})
            failed=sum(not value for value in checks.values())
            self._text('result',f'{len(checks)-failed}/{len(checks)} checks pass  |  '+
                       ('Inspect failed checks in the saved report' if failed else 'Mission evidence saved'),
                       self.AMBER if failed else self.GREEN)
        else:
            actual=sim.state.time/max(.001,time.monotonic()-self._started_wall)
            self._text('result',f'{self.speed:g}x playback requested  /  {actual:.1f}x achieved',self.MUTED)
        uid=self.selected
        if uid is not None:
            uav,flight=sim.state.uavs[uid],sim.flights[uid]
            observed=sim.physics.observe(uid)
            velocity=getattr(observed,'velocity',(0.,0.,0.))
            thrusts=getattr(observed,'rotor_thrusts_n',())
            x,y,z=uav.position.to_array()
            self._text('selected',f'UAV {uid:02d}   /   {uav.role.value.upper()}',self._role_color(uid))
            thrust_line=' '.join(f'{v:.1f}' for v in thrusts) if len(thrusts)==4 else 'unavailable'
            self._text('details',f'X {x:8.2f} m\nY {y:8.2f} m\nZ {z:8.2f} m\n'
                       f'Speed {math.sqrt(sum(v*v for v in velocity)):6.2f} m/s\n'
                       f'Energy {flight.remaining_s:6.1f} s\n'
                       f'Rotor thrust (N)\n{thrust_line}\n\nTab: select   F: close view',self.MUTED)
        self._text('controls','Space: pause   +/-: speed   Wheel: zoom   Right-drag: pan   Middle-drag: orbit   A: fit work   Home: full map   L: links   Esc: stop',self.MUTED)
        fps=(len(self._frames)-1)/(self._frames[-1]-self._frames[0]) if len(self._frames)>1 and self._frames[-1]>self._frames[0] else 0.
        self._text('view',f'{fps:4.0f} FPS   |   '+('FOLLOW SELECTED' if self.follow_selected else 'EAGLE OBSERVER'),self.CYAN)

    def render(self):
        if self.base is None or self.base.win is None or not self.base.win.isValid():
            return
        self._layout_hud()
        self._update_camera()
        self._update_scene()
        self._update_hud()
        self.base.taskMgr.step()
        now=time.monotonic()
        self._frames.append(now)
        self._last_render=now

    def before_step(self, sim):
        while not self.closed:
            now=time.monotonic()
            if now-self._last_render>=1/30:
                self.render()
                self.save_live()
            if self.closed: return False
            if not self.paused and (not self.real_time or now>=self._next_step):
                self._next_step=now+sim.dt/self.speed
                return True
            time.sleep(.004)
        return False

    def after_step(self, sim):
        if time.monotonic()-self._last_render>=1/30:
            self.render()
            self.save_live()

    def update(self, sim):
        """Alias for callers that update an observer outside ``run``."""
        self.after_step(sim)

    def physics_tick(self, sim):
        """Render after a measured control interval without advancing either clock.

        The mission clock changes at coarse planner boundaries. The separate
        physics clock in the HUD/live status labels intermediate observations.
        Pause/stop requests are consumed by the runner's next before_step.
        """
        if time.monotonic()-self._last_render>=1/30:
            self.render()
            self.save_live()
        return not self.closed

    def _screenshot(self, path):
        if self.base and self.base.win and self.base.win.isValid():
            return bool(self.base.win.saveScreenshot(self.p.Filename.fromOsSpecific(str(path))))
        return False

    def save_live(self, force=False):
        now=time.monotonic()
        if self.sim is None or (not force and now-self._last_save<2.): return
        sim=self.sim
        status={
            'time_s':sim.state.time,'physics_time_s':sim.physics.time_s,
            'timing_note':'Mission time advances at planner boundaries; physics time labels intermediate telemetry',
            'paused':self.paused,'speed':self.speed,
            'source':'AeroMesh Bullet rigid-body telemetry; viewer has no flight authority',
            'evidence_scope':'Uncalibrated rigid-body simulation; illustrative rotor spin is not RPM evidence',
            'output':str(self.output.parent.resolve()),
            'airborne':sum(f.armed for f in sim.flights.values()),
            'landed':sum(f.landed for f in sim.flights.values()),
            'completed_pois':sum(p.surveyed and p.report_s is not None for p in sim.tasks),
            'positions':{uid:u.position.to_array().tolist() for uid,u in sim.state.uavs.items()},
            'selected_uav':self.selected,'camera_focus':self.focus,'camera_distance_m':self.distance,
            'final_checks':None if self.final_report is None else self.final_report.get('checks'),
            'viewer_save_error':self._last_save_error,
        }
        try:
            temporary=self.output/'live.tmp'
            temporary.write_text(json.dumps(status,indent=2,allow_nan=False)+'\n',encoding='utf-8')
            for attempt in range(3):
                try:
                    temporary.replace(self.output/'live.json')
                    break
                except PermissionError:
                    if attempt==2: raise
                    time.sleep(.01)
            if not self._screenshot(self.output/'live.png'):
                self._last_save_error='Viewer framebuffer was not available for screenshot'
            for mark in (0,60,240,905,940,1600):
                if sim.state.time>=mark and mark not in self._captures:
                    if self._screenshot(self.output/f'frame-{mark:04d}.png'):
                        self._captures.add(mark)
        except (OSError,ValueError) as exc:
            self._last_save_error=f'{type(exc).__name__}: {exc}'
        self._last_save=now

    def finish(self, sim, report):
        self.final_report=report
        self.render()
        self.save_live(force=True)
        self._screenshot(self.output/'final.png')

    def wait_at_end(self, repeat=False, auto_close=False):
        if auto_close or self.closed: return False
        started=time.monotonic()
        while not self.closed:
            self.render()
            if self.restart_requested or (repeat and time.monotonic()-started>=4):
                self.restart_requested=False
                self.paused=False
                return True
            time.sleep(1/30)
        return False

    def close(self):
        self.closed=True
        for visual in self._owned_visuals:
            visual.removeNode()
        self._owned_visuals=[]
        if self.sim is not None:
            if self._world_parent.isEmpty(): self.sim.physics.root.detachNode()
            else: self.sim.physics.root.reparentTo(self._world_parent)
        if self.base is not None:
            self.base.destroy()
            self.base=None
