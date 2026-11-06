"""Live desktop observer for UAVXSimulator; presentation never owns flight state."""
from collections import deque
from io import BytesIO
import json
import math
from pathlib import Path
import time

import pygame


def visible_links(sim):
    """Topology edges only: end-to-end reachability is not a direct radio link."""
    positions = {uid: u.position.to_array() for uid, u in sim.state.uavs.items()}
    positions[-1] = sim.state.gcs.position.to_array()
    return [(a, b) for a, b in sim.router.network_graph.edges()
            if a in positions and b in positions
            and math.dist(positions[a], positions[b]) <= sim.constraints.radio_range_m + 1e-8]


def latest_received_camera(sim):
    """Read only complete GCS receipts; never use onboard captures or scene truth."""
    received = getattr(sim, '_received_images', {})
    observations = getattr(sim, '_observations', {})
    eligible = []
    for poi_id, pixels in received.items():
        metadata = observations.get(poi_id, {})
        acquired, arrived = metadata.get('acquired_at_s'), metadata.get('received_at_s')
        if (not isinstance(pixels, bytes) or not pixels
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in (acquired, arrived))
                or not 0 <= acquired <= arrived <= sim.state.time):
            continue
        eligible.append((arrived, poi_id, pixels, metadata))
    if not eligible:
        return None
    arrived, poi_id, pixels, metadata = max(eligible, key=lambda item: (item[0], item[1]))
    return dict(poi_id=poi_id, pixels=pixels, acquired_at_s=metadata['acquired_at_s'],
                received_at_s=arrived, capture_age_s=sim.state.time-metadata['acquired_at_s'],
                receipt_age_s=sim.state.time-arrived)


class UAVXVisualizer:
    BG, PANEL, GRID = (12, 19, 29), (20, 30, 43), (34, 47, 62)
    TEXT, MUTED = (231, 240, 247), (145, 163, 179)
    CYAN, GREEN, AMBER, RED = (72, 192, 226), (89, 209, 155), (244, 183, 88), (241, 108, 115)

    def __init__(self, speed=12., width=1400, height=900, real_time=True):
        if not math.isfinite(speed) or not .25 <= speed <= 100:
            raise ValueError('GUI speed must be between 0.25 and 100')
        pygame.display.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.display.set_caption('AeroMesh UAV-X | Live Swarm Simulation')
        self.font = pygame.font.SysFont('segoeui', 17)
        self.small = pygame.font.SysFont('segoeui', 14)
        self.large = pygame.font.SysFont('segoeui', 27, bold=True)
        self.speed, self.real_time = speed, real_time
        self.paused = self.closed = self.restart_requested = False
        self.show_links = self.show_trails = True
        self.show_camera = True
        self._camera_cache_key = self._camera_surface = None
        self.selected, self.zoom, self.pan = 0, 1., [0., 0.]
        self.dragging = False
        self.sim = self.output = self.final_report = None
        self.trails = {}
        self.last_render = self.next_step = self.last_save = 0.
        self.started_wall = time.monotonic()
        self.captures = set()

    def start(self, sim, output):
        self.sim = sim
        self.output = Path(output) / 'viewer'
        self.output.mkdir()
        self.final_report = None
        self.trails = {uid: deque(maxlen=250) for uid in sim.flights}
        self.started_wall = self.next_step = time.monotonic()
        self.last_save = self.last_render = 0.
        self.captures = set()
        self._camera_cache_key = self._camera_surface = None
        self.render()
        self.save_live(force=True)

    def layout(self):
        width, height = self.screen.get_size()
        self.map_rect = pygame.Rect(25, 120, max(400, width - 410), max(300, height - 190))
        self.panel_x = self.map_rect.right + 25
        self.scale = min(self.map_rect.width / 1260, self.map_rect.height / 1160) * self.zoom
        self.center = self.map_rect.center

    def project(self, x, y):
        return (round(self.center[0] + (x - 500) * self.scale + self.pan[0]),
                round(self.center[1] - y * self.scale + self.pan[1]))

    def text(self, value, xy, color=None, font=None):
        self.screen.blit((font or self.font).render(str(value), True, color or self.TEXT), xy)

    def button(self, label, rect, active=False):
        pygame.draw.rect(self.screen, (35, 64, 77) if active else self.GRID, rect, border_radius=6)
        self.text(label, (rect.x + 12, rect.y + 6), self.TEXT, self.small)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.closed = True
            elif event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode((max(1050, event.w), max(720, event.h)), pygame.RESIZABLE)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.closed = True
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                    self.speed = min(100., self.speed * 2)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    self.speed = max(.25, self.speed / 2)
                elif event.key == pygame.K_l:
                    self.show_links = not self.show_links
                elif event.key == pygame.K_t:
                    self.show_trails = not self.show_trails
                elif event.key == pygame.K_v and self.has_camera():
                    self.show_camera = not self.show_camera
                elif event.key == pygame.K_HOME:
                    self.zoom, self.pan = 1., [0., 0.]
                elif event.key == pygame.K_r and self.final_report is not None:
                    self.restart_requested = True
            elif event.type == pygame.MOUSEWHEEL:
                self.zoom = max(.6, min(5., self.zoom * (1.15 ** event.y)))
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    if self.pause_button.collidepoint(event.pos):
                        self.paused = not self.paused
                    elif self.links_button.collidepoint(event.pos):
                        self.show_links = not self.show_links
                    elif self.map_rect.collidepoint(event.pos):
                        points = [(math.dist(event.pos, self.project(u.position.x, u.position.y)), uid)
                                  for uid, u in self.sim.state.uavs.items()]
                        if points and min(points)[0] <= 20:
                            self.selected = min(points)[1]
                elif event.button == 3:
                    self.dragging = True
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 3:
                self.dragging = False
            elif event.type == pygame.MOUSEMOTION and self.dragging:
                self.pan[0] += event.rel[0]
                self.pan[1] += event.rel[1]

    def before_step(self, sim):
        while not self.closed:
            self.handle_events()
            now = time.monotonic()
            if now - self.last_render >= 1 / 30:
                self.render()
                self.save_live()
            if not self.paused and (not self.real_time or now >= self.next_step):
                self.next_step = now + sim.dt / self.speed
                return not self.closed
            pygame.time.wait(5)
        return False

    def after_step(self, sim):
        for uid, u in sim.state.uavs.items():
            self.trails[uid].append((u.position.x, u.position.y))

    def finish(self, sim, report):
        self.final_report = report
        self.render()
        self.save_live(force=True)
        pygame.image.save(self.screen, self.output / 'final.png')

    def save_live(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_save < 1:
            return
        sim = self.sim
        status = dict(time_s=sim.state.time, paused=self.paused, speed=self.speed,
                      source='AeroMesh UAVXSimulator live state', evidence_scope='kinematic simulation',
                      output=str(self.output.parent.resolve()),
                      airborne=sum(f.armed for f in sim.flights.values()),
                      landed=sum(f.landed for f in sim.flights.values()),
                      completed_pois=sum(p.surveyed and p.report_s is not None for p in sim.tasks),
                      positions={uid:u.position.to_array().tolist() for uid,u in sim.state.uavs.items()},
                      final_checks=None if self.final_report is None else self.final_report['checks'])
        if self.has_camera():
            receipt = latest_received_camera(sim)
            status['gcs_camera'] = ({key: value for key, value in receipt.items() if key != 'pixels'}
                                    if receipt else None)
        temporary = self.output / 'live.tmp'
        temporary.write_text(json.dumps(status, indent=2), encoding='utf-8')
        temporary.replace(self.output / 'live.json')
        pygame.image.save(self.screen, self.output / 'live.png')
        for mark in (0, 60, 240, 905, 940, 1600):
            if sim.state.time >= mark and mark not in self.captures:
                pygame.image.save(self.screen, self.output / f'frame-{mark:04d}.png')
                self.captures.add(mark)
        self.last_save = now

    def render(self):
        if self.sim is None:
            return
        self.layout()
        s, screen = self.sim, self.screen
        width, height = screen.get_size()
        screen.fill(self.BG)
        self.text('AEROMESH  /  UAV-X', (26, 20), self.TEXT, self.large)
        state_label = 'FINISHED' if self.final_report else 'PAUSED' if self.paused else 'LIVE'
        self.text(f'{state_label}   {len(s.flights)} UAVs   |   Seed {s.seed}   |   {s.state.time:07.1f} / 2700 s',
                  (28, 64), self.GREEN if not self.paused else self.AMBER)
        self.pause_button = pygame.Rect(width - 340, 24, 130, 33)
        self.links_button = pygame.Rect(width - 195, 24, 165, 33)
        self.button('Resume' if self.paused else 'Pause', self.pause_button, self.paused)
        self.button('Radio links: ' + ('on' if self.show_links else 'off'), self.links_button, self.show_links)
        pygame.draw.rect(screen, self.PANEL, self.map_rect, border_radius=8)
        screen.set_clip(self.map_rect)
        for x in range(75, 1076, 100):
            pygame.draw.line(screen, self.GRID, self.project(x, -500), self.project(x, 500))
        for y in range(-500, 501, 100):
            pygame.draw.line(screen, self.GRID, self.project(75, y), self.project(1075, y))
        arena = pygame.Rect(self.project(75, 500), (1000*self.scale, 1000*self.scale))
        pygame.draw.rect(screen, (77, 103, 120), arena, 1)
        self.text('DISASTER AREA  1000 x 1000 m', self.project(95, 480), self.MUTED, self.small)
        if self.show_trails:
            for points in self.trails.values():
                if len(points) >= 2:
                    pygame.draw.lines(screen, (42, 73, 91), False, [self.project(*p) for p in points], 1)
        if self.show_links:
            positions = {uid:u.position for uid,u in s.state.uavs.items()}
            positions[-1] = s.state.gcs.position
            for a,b in visible_links(s):
                if a != -1 and b != -1 and not (s.flights[a].armed or s.flights[b].armed):
                    continue
                p,q=positions[a],positions[b]
                pygame.draw.line(screen, (49,110,133),self.project(p.x,p.y),self.project(q.x,q.y),1)
        for p in s.tasks:
            if p.spawn_s > s.state.time:
                continue
            xy = self.project(*p.position[:2])
            received = p.report_s is not None
            color = self.GREEN if p.surveyed and received else self.AMBER if received else self.RED
            pygame.draw.circle(screen, color, xy, 8, 2)
            if p.priority > 1:
                pygame.draw.circle(screen, color, xy, 12, 1)
            self.text(f'P{p.id}', (xy[0]+11,xy[1]-10),color,self.small)
        gcs=self.project(0,0)
        pygame.draw.rect(screen,self.GREEN,(gcs[0]-7,gcs[1]-7,14,14),2)
        self.text('GCS',(gcs[0]-16,gcs[1]+12),self.GREEN,self.small)
        for uid,u in s.state.uavs.items():
            f=s.flights[uid]
            color=self.RED if not u.is_operational() else self.MUTED if not f.armed else self.AMBER if f.returning else self.CYAN if u.role.value=='relay' else self.GREEN
            xy=self.project(u.position.x,u.position.y)
            for dx,dy in [(-4,-4),(4,-4),(-4,4),(4,4)]:
                pygame.draw.line(screen,color,xy,(xy[0]+dx,xy[1]+dy),1)
                pygame.draw.circle(screen,color,(xy[0]+dx,xy[1]+dy),2,1)
            pygame.draw.circle(screen,color,xy,2)
            if uid==self.selected:
                pygame.draw.circle(screen,self.TEXT,xy,12,1)
                self.text(f'U{uid}  {u.position.z:.0f} m',(xy[0]+15,xy[1]+2),self.TEXT,self.small)
        screen.set_clip(None)
        self.draw_panel(width,height)
        actual=s.state.time/max(.001,time.monotonic()-self.started_wall)
        self.text(f'Playback {self.speed:g}x requested / {actual:.1f}x effective', (28,height-53),self.MUTED,self.small)
        self.text('Space pause  |  +/- speed  |  L links  |  T trails  |  Wheel zoom  |  Right-drag pan  |  Home fit',
                  (28,height-29),self.MUTED,self.small)
        self.text('LIVE KINEMATIC MODEL', (width-210,height-29),self.MUTED,self.small)
        pygame.display.flip()
        self.last_render=time.monotonic()

    def draw_panel(self,width,height):
        s=self.sim
        x,y=self.panel_x,125
        done=sum(p.surveyed and p.report_s is not None for p in s.tasks)
        stats=s.router.get_network_stats()
        airborne=sum(f.armed for f in s.flights.values())
        connected=sum(f.armed and s.state.uavs[i].connected_to_gcs for i,f in s.flights.items())
        rows=[('MISSION',f'{done} / {len(s.tasks)} PoIs surveyed + received'),
              ('FLEET',f'{airborne} airborne  |  {sum(f.landed for f in s.flights.values())} landed'),
              ('NETWORK',f'{connected} / {airborne} airborne connected'),
              ('PACKET DELIVERY',f'{stats["pdr"]*100:.2f}%   |   {stats["avg_latency"]:.2f} s mean latency')]
        row_spacing = 56 if self.has_camera() and height < 820 else 65
        for label,value in rows:
            self.text(label,(x,y),self.MUTED,self.small)
            self.text(value,(x,y+23));y+=row_spacing
        if self.has_camera() and self.show_camera:
            y = self.draw_camera(x, y, width, height)
        else:
            y = self.draw_details(x, y, height)
        scenario = getattr(s, 'scenario', None)
        if scenario is not None:
            injected = len(getattr(s, 'fault_records', []))
            faults = len(getattr(scenario, 'faults', ()))
            self.text(f'Faults: {injected} / {faults} injected',
                      (x, min(height-156, y+12)), self.AMBER, self.small)
            y += 30
        if self.final_report:
            failed=[k for k,v in self.final_report['checks'].items() if not v]
            self.text('SAMPLE CHECKS PASS' if not failed else f'{len(failed)} CHECKS FAILED',
                      (x,min(height-130,y+20)),self.GREEN if not failed else self.RED)
            self.text('R: new run   Esc: close',(x,min(height-103,y+46)),self.MUTED,self.small)
        elif scenario is None:
            self.text(f'Fault scenario: {s.disturbance}',(x,min(height-112,y+20)),self.AMBER,self.small)

    def has_camera(self):
        return getattr(getattr(self.sim, 'scenario', None), 'camera_mode', None) == 'panda3d'

    def draw_camera(self, x, y, width, height):
        self.text('SIMULATED CAMERA / GCS RECEIVED', (x, y+4), self.CYAN, self.small)
        rect = pygame.Rect(x, y+32, max(160, width-x-26), min(180, max(70, height-y-280)))
        pygame.draw.rect(self.screen, self.PANEL, rect)
        receipt = latest_received_camera(self.sim)
        if receipt is None:
            self._camera_cache_key = self._camera_surface = None
            self.text('Awaiting image receipt', (x+12, rect.centery-10), self.MUTED, self.small)
            self.text('Images appear after complete delivery.', (x, rect.bottom+10), self.MUTED, self.small)
            bottom = rect.bottom+52
        else:
            key = (receipt['poi_id'], receipt['received_at_s'], receipt['pixels'])
            if key != self._camera_cache_key:
                self._camera_cache_key = key
                try:
                    self._camera_surface = pygame.image.load(BytesIO(receipt['pixels']), 'received.png').convert()
                except (pygame.error, ValueError):
                    self._camera_surface = None
            if self._camera_surface is not None:
                image_width, image_height = self._camera_surface.get_size()
                ratio = min(rect.width/image_width, rect.height/image_height)
                thumb = pygame.transform.smoothscale(self._camera_surface,
                                                      (max(1, round(image_width*ratio)), max(1, round(image_height*ratio))))
                self.screen.blit(thumb, thumb.get_rect(center=rect.center))
            else:
                self.text('Received image cannot be decoded', (x+8, rect.centery-10), self.AMBER, self.small)
            self.text(f'PoI {receipt["poi_id"]}  |  captured {receipt["capture_age_s"]:.1f} s ago',
                      (x, rect.bottom+10), self.TEXT, self.small)
            self.text(f'GCS received {receipt["receipt_age_s"]:.1f} s ago',
                      (x, rect.bottom+32), self.MUTED, self.small)
            bottom = rect.bottom+58
        self.text('V: drone details and events', (x, bottom), self.MUTED, self.small)
        return bottom+20

    def draw_details(self, x, y, height):
        s = self.sim
        compact = self.has_camera() and height < 820
        u=s.state.uavs[self.selected];f=s.flights[self.selected]
        self.text(f'SELECTED  U{self.selected}',(x,y+4),self.CYAN)
        lines=[f'Role: {"returning" if f.returning else u.role.value}',
               f'Height: {u.position.z:.1f} m  |  battery: {100*u.battery_level:.0f}%',
               f'Flight time: {f.sortie_s:.1f} / 1200 s',
               f'GCS path: {"connected" if u.connected_to_gcs else "unavailable"}']
        if compact:
            lines = [f'{"Returning" if f.returning else u.role.value.title()}  |  GCS: {"connected" if u.connected_to_gcs else "unavailable"}',
                     lines[1], lines[2]]
        for i,line in enumerate(lines):
            self.text(line,(x,y+34+i*23),self.TEXT,self.small)
        y+=112 if compact else 142
        self.text('RECENT EVENTS',(x,y),self.MUTED,self.small);y+=25
        for e in s.events[-(2 if compact else 5):]:
            label=e['type'].replace('_',' ')
            if 'poi_id' in e:label+=f' P{e["poi_id"]}'
            if 'uav_id' in e:label+=f' U{e["uav_id"]}'
            self.text(f'{e["time_s"]:6.1f}  {label[:32]}',(x,y),self.MUTED,self.small);y+=23
        if self.has_camera():
            self.text('V: GCS camera image', (x, y+6), self.MUTED, self.small)
            y += 26
        return y

    def wait_at_end(self, repeat=False, auto_close=False):
        if auto_close or self.closed:
            return False
        start=time.monotonic()
        while not self.closed:
            self.handle_events()
            self.render()
            if self.restart_requested or (repeat and time.monotonic()-start>=4):
                self.restart_requested=False
                self.paused=False
                return True
            pygame.time.wait(33)
        return False

    def close(self):
        pygame.quit()
