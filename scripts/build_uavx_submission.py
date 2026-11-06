"""Build a clearly labelled proposal draft and trajectory video from measured runs.

Optional authoring dependencies: reportlab, Pillow, and ffmpeg on PATH.
Usage: python scripts/build_uavx_submission.py --runs RUN1 RUN2 ... --output DIR
"""
import argparse
import hashlib
from html import escape
import math
import sys
import json
from pathlib import Path
import shutil
import subprocess
from PIL import Image, ImageDraw, ImageFont
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import mm
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, Image as PdfImage
from reportlab.graphics.shapes import Drawing, Rect, String, Line


def frame_image(frame, tasks, index=0, camera_receipts=None, faults=None, playback=30):
    image=Image.new('RGB',(1280,720),'#f4f7fa');d=ImageDraw.Draw(image)
    font_path=Path('C:/Windows/Fonts/arial.ttf')
    font=ImageFont.truetype(str(font_path),18) if font_path.exists() else ImageFont.load_default()
    big=ImageFont.truetype(str(font_path),28) if font_path.exists() else font
    def pos(p): return (90+(p[0]+75)*.58,640-(p[1]+500)*.52)
    d.text((35,22),'AeroMesh | UAV-X measured kinematic replay',fill='#142e46',font=big)
    d.text((35,60),f"Simulation {frame['time_s']:.1f} s  |  {playback}x replay  |  recorded simulation states",fill='#456070',font=font)
    active_faults=[fault for fault in (faults or []) if frame['time_s']>fault['time_s']
                   and (fault['cleared_s'] is None or frame['time_s']<=fault['cleared_s'])]
    if active_faults:
        fault=active_faults[-1]
        d.text((35,88),f"FAULT ACTIVE: {fault['kind']} / target {fault['target']} / injected at {fault['time_s']:.1f}s",fill='#ad3b56',font=font)
    elif faults and frame['time_s']>min(fault['time_s'] for fault in faults):
        d.text((35,88),'Fault interval ended; mission and received data remain visible',fill='#268864',font=font)
    d.rectangle((pos((75,500))[0],120,pos((1075,-500))[0],640),outline='#8098a6',width=2)
    d.text((790,145),'Recorded simulation states',fill='#142e46',font=big)
    d.text((790,194),'Blue: relay   Orange: scout',fill='#456070',font=font)
    d.text((790,225),'Gray: grounded   Red: failed',fill='#456070',font=font)
    d.text((790,275),'Recorded radio topology',fill='#456070',font=font)
    d.text((790,304),'Kinematic motion / simulated scene',fill='#456070',font=font)
    nodes={u['id']:u['position'] for u in frame['uavs'] if u['role']!='failed'}
    nodes[-1]=(0,0,0)
    import math
    for a,b in frame.get('links',[]):
        if a in nodes and b in nodes and math.dist(nodes[a],nodes[b])<=100 and (nodes[a][2]>1 or nodes[b][2]>1):
            d.line((*pos(nodes[a]),*pos(nodes[b])),fill='#c3d4df',width=1)
    if camera_receipts is not None:
        available=[record for record in camera_receipts if record['received_at_s']<=frame['time_s']]
        if available:
            record=max(available,key=lambda record:record['received_at_s'])
            view=record['image'].copy();view.thumbnail((260,245))
            image.paste(view,(790,355))
            d.text((790,610),f"GCS image: P{record['poi_id']} / simulated camera",fill='#142e46',font=font)
            d.text((790,638),f"Acquired {record['acquired_at_s']:.1f}s / received {record['received_at_s']:.1f}s",fill='#456070',font=font)
        else:
            d.text((790,365),'Awaiting complete GCS image',fill='#142e46',font=font)
    else:
        d.text((790,365),'Footprint observations only',fill='#142e46',font=font)
        d.text((790,394),'No camera imagery in this run',fill='#142e46',font=font)
    for p in tasks:
        if p['spawn_s']<=frame['time_s']:
            x,y=pos(p['position']);reported=p['report_s'] is not None and p['report_s']<=frame['time_s']
            d.rectangle((x-6,y-6,x+6,y+6),fill='#268864' if reported else '#ad3b56')
            d.text((x+8,y-13),str(p['id']),fill='#142e46',font=font)
    for u in frame['uavs']:
        x,y=pos(u['position'])
        color='#bb3344' if u['role']=='failed' else '#ea9224' if u['role']=='scout' else '#1682aa' if u['armed'] else '#84929a'
        d.ellipse((x-4,y-4,x+4,y+4),fill=color)
    x,y=pos((0,0));d.rectangle((x-7,y-7,x+7,y+7),fill='#142e46');d.text((x-12,y+10),'GCS',fill='#142e46',font=font)
    d.text((35,677),'PoIs: red pending / green received  |  See metrics.json for every constraint result',fill='#456070',font=font)
    return image


def make_video(run,out):
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg: raise RuntimeError('ffmpeg is required to build the demonstration video')
    metrics=json.loads((run/'metrics.json').read_text())
    faults=metrics.get('faults',[])
    frames=[json.loads(line) for line in (run/'trajectory.jsonl').read_text().splitlines()]
    camera_receipts=None
    if metrics.get('sensing',{}).get('mode')=='panda3d':
        observations=json.loads((run/'sensing/observations.json').read_text(encoding='utf-8'))['observations']
        camera_receipts=[]
        for record in observations:
            if record['received_at_s'] is not None:
                with Image.open(run/record['received_file']) as source:
                    camera_receipts.append(dict(record,image=source.convert('RGB')))
    command=[ffmpeg,'-hide_banner','-loglevel','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','15','-i','-',
             '-an','-c:v','libx264','-preset','fast','-crf','24','-pix_fmt','yuv420p',str(out/'demo.mp4')]
    proc=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        for i,frame in enumerate(frames):
            slow=any(fault['time_s']-10<=frame['time_s']<=fault['time_s']+40 for fault in faults)
            data=frame_image(frame,metrics['tasks'],i,camera_receipts,faults,5 if slow else 30).tobytes()
            for _ in range(6 if slow else 1): proc.stdin.write(data)
        proc.stdin.close()
        error=proc.stderr.read().decode(errors='replace');code=proc.wait()
        if code: raise RuntimeError(error)
    except Exception:
        proc.kill();proc.wait();raise
    # The midpoint can be a recharge pause. Use an actual active frame for
    # the report figure; the video still retains every recorded interval.
    representative = max(frames, key=lambda frame: (
        sum(u['armed'] for u in frame['uavs']),
        max(math.hypot(*u['position'][:2]) for u in frame['uavs'])))
    frame_image(representative,metrics['tasks'],camera_receipts=camera_receipts,faults=faults).save(out/'trajectory.png')


def architecture():
    dr=Drawing(475,188)
    boxes=[(0,126,'Scenario / GCS'),(166,126,'Mission planner'),(332,126,'Safety checks'),
           (0,32,'GCS receipts'),(166,32,'Mesh router'),(332,32,'UAV states / sensing')]
    for x,y,label in boxes:
        dr.add(Rect(x,y,142,44,fillColor=colors.HexColor('#e9f1f5'),strokeColor=colors.HexColor('#28758a')))
        dr.add(String(x+8,y+18,label,fontName='Helvetica',fontSize=10,fillColor=colors.HexColor('#142e46')))
    for x1,y1,x2,y2 in [(142,148,166,148),(308,148,332,148),(404,126,404,76),(332,54,308,54),(166,54,142,54)]:
        dr.add(Line(x1,y1,x2,y2,strokeColor=colors.HexColor('#28758a'),strokeWidth=1.5))
    return dr


def software_diagram():
    dr=Drawing(475,108)
    names=['Scenario','Mission / roles','Safety predicates','Kinematics',
           'GCS receipt','Packet queue','Detection time','State sample']
    for index,label in enumerate(names):
        col=index%4;row=index//4;x=col*121;y=68-row*58
        dr.add(Rect(x,y,108,30,fillColor=colors.HexColor('#f4f7fa'),strokeColor=colors.HexColor('#8098a6')))
        dr.add(String(x+7,y+11,label,fontName='Helvetica',fontSize=9))
        if col<3:dr.add(Line(x+108,y+15,x+121,y+15,strokeColor=colors.HexColor('#28758a')))
    dr.add(Line(417,68,417,40,strokeColor=colors.HexColor('#28758a')))
    return dr


def report_profile(metrics):
    """Use recorded configuration, never current defaults, to describe a run."""
    scenario=metrics.get('scenario') or {}
    protection=metrics.get('relay_protection') or {}
    return dict(
        version=metrics.get('schema_version',1),
        fleet=metrics['fleet_size'], seed=metrics['seed'],
        camera=(metrics.get('sensing') or {}).get('mode','footprint')=='panda3d',
        protected=protection.get('enabled',False),
        spacing=protection.get('slot_spacing_m',85.),
        planning_separation=protection.get('planning_separation_m',20.),
        bandwidth=scenario.get('radio_bytes_per_s'), queue=scenario.get('radio_queue_bytes'),
        recharge=scenario.get('recharge_s',720.), pads=scenario.get('charging_pads'),
        pad_spacing=scenario.get('pad_spacing_m',25.),
        width=scenario.get('camera_width',128), height=scenario.get('camera_height',128),
    )


def _display_number(value, digits=1):
    return 'not observed' if value is None else f'{value:.{digits}f}'


def _case_label(metrics):
    if metrics.get('schema_version',1)>=2:
        scenario=metrics.get('scenario') or {}
        labels={'vehicle_failure':'failure','link_outage':'link','radio_outage':'radio'}
        kind='+'.join(f"{labels.get(f['kind'],f['kind'])}@{f['time_s']:g}s" for f in scenario.get('faults',[])) or 'nominal'
        loss=scenario.get('packet_loss',.02)
        suffix=f"; loss {100*loss:g}%" if loss!=.02 else ''
        pads=scenario.get('charging_pads')
        if pads is not None and pads!=metrics['fleet_size']:
            suffix+=f"; {pads} pads"
        return f"{kind}; s{metrics['seed']}; {metrics['fleet_size']} UAVs{suffix}"
    else:
        disturbance=metrics.get('disturbance') or {}
        kind=disturbance.get('kind',disturbance.get('type','fault')) if disturbance else 'nominal'
    return f"{kind}; seed {metrics['seed']}"


def make_pdf(runs,out,video_url=None):
    rows=[(p.name,json.loads((p/'metrics.json').read_text(encoding='utf-8'))) for p in runs]
    if not rows:
        raise ValueError('At least one measured run is required')
    if len(rows)>24:
        raise ValueError('The eight-page proposal supports at most 24 run rows; keep a larger matrix in its separate evidence summary')
    primary=rows[0][1]
    profile=report_profile(primary)
    v2=profile['version']>=2
    camera=profile['camera']
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='BodyU',fontName='Helvetica',fontSize=9.5,leading=12.5,spaceAfter=8,textColor=colors.HexColor('#233d50')))
    styles.add(ParagraphStyle(name='SmallU',fontName='Helvetica',fontSize=8,leading=10.5,spaceAfter=5,textColor=colors.HexColor('#4b6170')))
    styles.add(ParagraphStyle(name='MatrixU',fontName='Helvetica',fontSize=6.7,leading=8.1,spaceAfter=0,textColor=colors.HexColor('#233d50')))
    story=[]
    def p(text):story.append(Paragraph(text,styles['BodyU']))
    def title(text):story.append(Paragraph(text,styles['Heading1']));story.append(Spacer(1,6))
    def sub(text):story.append(Paragraph(text,styles['Heading2']))
    def new():story.append(PageBreak())
    def table(data,widths,compact=False):
        style=styles['MatrixU'] if compact else styles['SmallU']
        wrapped=[[Paragraph(escape(str(c)),style) for c in row] for row in data]
        t=Table(wrapped,colWidths=widths,repeatRows=1,hAlign='LEFT')
        padding=3 if compact else 6
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e9f1f5')),('VALIGN',(0,0),(-1,-1),'TOP'),
                              ('BOTTOMPADDING',(0,0),(-1,-1),padding),('TOPPADDING',(0,0),(-1,-1),padding),
                              ('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor('#28758a'))]))
        story.append(t)
    title('AeroMesh | UAV-X')
    p('<b>Technical proposal draft - 6 November 2026</b>')
    p('PUSHPAK Grand Challenge 2026 / Resilient BVLOS Swarm Challenge')
    table([['Submission identity','Status'],['Team name / registration ID','Not supplied'],['Institution / category','Not supplied'],
           ['Leader / members / contributions','Not supplied'],['Stage 2 qualification','Not confirmed'],['Document status','Technical draft; not submitted']], [180,295])
    sub('1. Abstract / executive summary')
    p('AeroMesh studies disaster reconnaissance with a range-limited multi-hop aerial network. A ground station outside the disaster area assigns emerging points of interest to a moving relay chain. Energy admission, stable relay identities, priority handling, reserve-driven return and packet receipts connect mission planning to measurable results. The supplied sample limits govern endurance, deadline, range, speed, altitude, separation and reporting delay [1,2].')
    if v2:
        p('The scenario-driven implementation adds configured fault schedules, denser relay placement, finite shared transmit/receive service and bounded queues. Camera-enabled runs render a procedural disaster scene through Panda3D; original PNG bytes are fragmented, routed and reassembled at the GCS before a report is credited. Node-disjoint connectivity is sampled from the actual topology. Resource use and failures remain visible in the evidence.')
    else:
        p('This baseline uses geometric sensor-footprint observations and idealized, unlimited parallel radio service. It provides a fast algorithm regression model; it does not demonstrate scene-camera sensing or bounded bandwidth. Garuda design patterns inform measured landing and timestamped observations without importing its single-aircraft backend.')
    completion=_display_number(primary['completion_time_s']) if primary['completion_time_s'] is not None else 'incomplete at the recorded cutoff'
    p(f"The first listed run (seed {primary['seed']}, {primary['fleet_size']} UAVs) reports {primary['pois_completed']}/{primary['pois_total']} PoIs; completion is {completion}{' s' if primary['completion_time_s'] is not None else ''}, with {primary['connectivity_availability_percent']:.3f}% connectivity availability. It passes {sum(bool(v) for v in primary['checks'].values())}/{len(primary['checks'])} implemented checks. The matrix includes every run supplied to this draft. Motion remains kinematic; the results do not qualify physical flight or unseen organizer scenarios. Team identity, official log compliance and final review remain outstanding.")
    new();title('2. Problem and mission analysis')
    p('An earthquake or landslide disrupts terrestrial communications. UAVs must survey designated locations, collect imagery and situational data, maintain an aerial path to the GCS, and adapt to failures and recharge. Coverage, delivery, endurance and safe recovery are separate requirements [1,2].')
    table([['Supplied sample constraint','Configured value'],['Operational disaster area','1000 m x 1000 m'],['Mission / per-sortie flight limit','2700 s / 1200 s'],['Radio / speed / altitude','100 m / 5 m/s / 100 m'],['Minimum separation / reporting deadline','20 m / 10 s'],['PoIs','10 required by the sample; realized tasks retained in evidence'],['Completion','Survey and receipt of all PoIs; own-pad landing for every UAV']], [245,230])
    sub('2.1 Declared resources and assumptions')
    p(f"The first run uses {profile['fleet']} aircraft and {profile['pad_spacing']:g} m home-pad spacing. GCS=(0,0,0), the disaster square is x=75..1075 m, y=-500..500 m, and the flight geofence includes staging and approach. These coordinates, fleet count and charging resources are team assumptions, not organizer approvals. Each other run's scenario and resource configuration is retained with its logs.")
    if v2:
        chargers=str(profile['pads']) if profile['pads'] is not None else 'one per home pad (no shared-pad limit)'
        p(f"Full recharge is configured as {profile['recharge']:g} s; charger availability is {chargers}. Explicit task coordinates/times/priorities and link, radio or aircraft faults can be supplied in validated JSON. Generated tasks retain the seed and realized values for deterministic replay. Larger fleets and denser relays consume more staging, charging and energy resources; those costs must be accepted before a competition claim.")
    else:
        p('Default generation places the first task at t=0 and remaining tasks in 60..1800 s. A high-priority task can preempt current work. The baseline assumes 720 s recharge and does not model shared charger contention. Fleet shortages and late tasks can leave the mission incomplete.')
    p('The custom model and communication assumptions require organizer acceptance as an equivalent framework. A particular 3D engine, physical drones and decentralized control are not established mandatory requirements in the supplied documents; the report template permits centralized control with justification [1,4].')
    new();title('3. System and software architecture')
    story.append(architecture());p('<b>Figure 1.</b> Mission demand produces bounded motion; sensed observations become packets, and only GCS receipts establish delivery.')
    module_rows=[['Module','Responsibility'],['simulation/uavx.py; resilient_uavx.py' if v2 else 'simulation/uavx.py','Mission, roles, energy, kinematic motion and recorded outcomes'],['core/mission_safety.py','Energy admission, settled landing and swept separation'],['comm/mesh_router.py','Range graph, routing, retries and immutable outcomes; bounded radio service in scenario runs'],['simulation/uavx_scenario.py; camera_sensor.py' if v2 else 'Sensor footprint','Validated scenario and CPU scene-camera captures' if v2 else 'Geometric proximity triggers synthetic observations'],['verify_submission.py; verification/','Source/artifact hashes and evidence reconciliation']]
    table(module_rows,[220,255])
    story.append(software_diagram())
    p('<b>Figure 2.</b> Task emergence -&gt; priority/role/energy decision -&gt; movement vetoes -&gt; state update -&gt; timestamped sensing -&gt; packet queues -&gt; GCS receipt -&gt; checks. All runtime intervals use one simulation clock.')
    p('Python, NumPy and NetworkX form the algorithm runtime. Panda3D 1.10.16 TinyDisplay and Pillow supply the optional CPU camera; pygame supplies the live view. The camera receives a kinematic pose and never supplies flight authority. No physical-hardware adapter is invoked by this path.' if camera else 'Python, NumPy and NetworkX form the minimal runtime; pygame supplies the optional live view. No physical-hardware adapter is invoked by this path. A legacy profile preserves earlier scenarios separately.')
    new();title('4. Swarm autonomy, roles and energy')
    p(f"The centralized planner selects the highest-priority announced unfinished task and places relay slots on a radial chain. The first run's nominal slot spacing is {profile['spacing']:g} m. Scenario admission requires complete matching: every required role receives a distinct eligible aircraft. Squared travel costs reduce crossed assignments; canonical slot order keeps replacement dispatch reproducible. If the preferred new task is infeasible, announced alternatives of equal priority are tried. An unfinished active task retains equal-priority authority; future task coordinates are excluded.")
    if profile['protected']:
        p('Denser placement creates skip links: with 44 m nominal slots, a single missing settled relay can leave an 88 m gap inside the 100 m range. This topology is a protection mechanism, not a guarantee for moving/depleted aircraft. The run samples actual biconnected components containing the GCS to measure which active sources have two internally node-disjoint paths. Deployment, re-tasking and return can still be unprotected.')
        p('Ordinary motion admission retains a primary GCS path throughout the movement interval and evaluates backup paths for mission-assigned aircraft at the proposed endpoint. Backup links may hand over to new neighbors. Returning aircraft retain a primary path but may release backup links to preserve their return reserve. Emergency return can override the guard; continuous two-path protection is not guaranteed. Actual outage and delivery results are measured separately, and earlier failed attempts remain archived.')
    else:
        p('The baseline 85 m chain does not provide a continuous alternate path across every relay loss: removing an intermediate relay can leave a gap beyond the 100 m range. Dispatching a spare after the break cannot itself prove uninterrupted service.')
    p(f"Task admission budgets deployment, dwell, report delivery and common withdrawal against the 1200 s sortie cap and 2700 s deadline. Scenario matching checks retiring aircraft and favors up to 180 s of extra common energy headroom before added flight cost. A healthy connected cohort may continue unfinished work while a larger task waits, subject to measured energy and return reserves. Energy clocks never reset on reassignment. Full recharge is {profile['recharge']:g} s; this is a declared endurance model, not a calibrated battery model.")
    if v2:
        p('With no announced work, a 30 s idle grace delays soft readiness withdrawal; the hard return reserve and a geometry-based wait limit still apply. A completed formation may wait for known chargers only when a forecast finds feasible admission and headroom for the announced follow-up cohort. The forecast uses copied flight records and time, repeats the runtime charge arithmetic at each step, and respects the finite-pad queue. It grants no real charge or mission credit: actual readiness is checked again before assignment. New announced work or aircraft failure invalidates the cached hold.')
    p('A relay handoff requires measured replacement arrival and a GCS path, including during a short gap in announced tasks. Invalid numeric state cannot establish arrival or a safe mission budget. Replacements are reconciled when owners return or slots disappear. Loss of a GCS path triggers recovery even when local neighbors remain connected; return reserve takes priority over waiting indefinitely.')
    p('Nominal relay altitude is 40 m, replacement approach 65 m and return routing 90 m where feasible. Every accepted segment is checked for speed, separation, geofence and connectivity. The motion planner can still stall or incur downtime; centralized full-state access and resource assumptions remain qualification limits.')
    new();title('5. Communication, sensing and fault recovery')
    p(f"The network uses a 100 m three-dimensional cutoff and {100*primary.get('packet_loss_per_attempt',.02):g}% independent loss per attempt. A logical packet advances at most one hop per {primary.get('dt_s',.5):g} s tick. Queue and retry time remain in measured latency; no-route offers remain in packet-delivery-ratio accounting.")
    if v2:
        p(f"Each node, including the GCS, shares a finite TX/RX budget of {profile['bandwidth']/1024:g} KiB/s and a {profile['queue']/1024:g} KiB queue in the first run. Serialized payload/envelope bytes and configured wire size determine service cost. Failed transmission attempts consume capacity; large packets require continued service across multiple ticks without banking idle capacity. Emergency/control priority precedes ordinary bulk traffic. Per-interval service samples retain used bytes; aggregate statistics retain queue peaks and drops. This is a bounded network-service model, not calibrated RF propagation or channel-wide interference.")
    else:
        p('Parallel service is unlimited in this baseline. Bandwidth contention, channel interference and physical RF propagation are not represented. Immutable offers and at most one terminal delivery/drop prevent duplicate IDs or reroute attempts from inflating success.')
    if camera:
        p(f"A downward {profile['width']} x {profile['height']} Panda3D TinyDisplay camera renders damaged walls, debris and other procedural scenery at the UAV's recorded pose. Parcels become visible at their task emergence time. Capture records retain acquisition time, source UAV, configured pinhole intrinsics, scene fingerprint and PNG SHA-256. Geometric footprint entry triggers capture; no learned survivor detector is claimed.")
        p('The actual PNG bytes are fragmented into mesh payloads. The GCS records delivery only after all chunks arrive and the assembled length and SHA-256 match the original capture. Retrying cannot refresh the acquisition timestamp. Captures, received images and supporting packet events are retained separately; the 10 s check applies from the first observation to complete image receipt. If a sensor carrier fails, a replacement can recapture from another safe altitude; the first observation deadline and both capture records are retained.')
    else:
        p('A geometric sensor footprint produces a synthetic report with an immutable detection timestamp. The first GCS receipt determines delay. This mode has no scene-camera image payload and cannot be presented as imagery-collection evidence.')
    p('Scenario faults distinguish a missing link, an unavailable sole radio and permanent vehicle failure. Targets, injection times, durations and resolved selections are logged. Alternate relays cannot restore a disabled endpoint radio. A failed aircraft is excluded from charging and pad capacity; recharge cannot revive it. A permanently frozen aircraft remains an obstacle and fails strict all-UAV landing. Separately labelled surviving-fleet completion never replaces strict checks.' if v2 else 'The baseline injects one 30 s relay radio outage or permanent relay failure after t=900 s. The disabled endpoint and permanently stranded vehicle remain strict failures; receipt from surviving sources is a separate service-recovery measurement.')
    new();title('6. Safety and technical merit')
    p(f"Safety uses closest approach over simultaneous linear motion segments, not sampled endpoints alone. The first run plans for {profile['planning_separation']:g} m separation while the sample's measured minimum remains 20 m. Speed and geofence violations are vetoed; observed extrema, collision pairs and violations are retained. Kinematic margin is not proof against wind, tracking error, finite airframe size or arbitrary physical motion.")
    p('Landing requires fresh, settled position/velocity evidence at each own home pad, two consecutive settled seconds and disarm. An empty fleet, failed aircraft, return command or hover near home cannot establish mission completion. The deadline and charge checks are not relaxed to turn a fault case into a pass.')
    p('Garuda contributed design references for measured completion, target-plus-return energy, paired observation timing and evidence integrity [3]. No Garuda source, simulator assets or weights were copied. Its single-client scene reload is not used as a swarm backend.')
    p('The contribution is a traceable coupling of mission roles, energy, reporting and measured end states. The scenario extension adds testable fault semantics, actual rendered image-byte transport, finite radio service and measured topology protection. These are established engineering mechanisms; optimal routing, universal fault tolerance and deployed physical performance are not claimed.' if v2 else 'The contribution is traceable coupling of relay planning, bounded motion, reporting deadlines and measured end states. These are established engineering mechanisms applied to the challenge; a new optimal control or routing algorithm is not claimed.')
    p('Regression coverage includes unreachable traffic, duplicates, per-tick hop limits, stale landing evidence, crossing paths, task emergence and replacement cancellation. Scenario tests additionally cover input validation, radio congestion/service accounting, camera provenance and image reassembly. The accompanying validation record gives the exact test result; this generator does not infer a test pass from successful PDF creation.' if v2 else 'Regression tests cover unreachable traffic, duplicate IDs, hop limits, stale landing evidence, crossing paths, future PoIs and replacement cancellation. The accompanying validation record gives the exact test result; successful artifact generation is not a test pass.')
    sub('Measured evaluation metrics for R01')
    packet_stats=primary['packet_statistics']
    recoveries=[f"{fault['recovered_s']-fault['time_s']:.1f} s" if fault.get('recovered_s') is not None else 'not recovered' for fault in primary.get('faults',[])]
    efficiency=primary.get('network_reconfiguration_efficiency')
    table([['Metric','Measured result on the first listed scenario'],
           ['PoI completion / priority score',f"{100*primary['pois_completed']/max(primary['pois_total'],1):.1f}% / {primary['priority_weighted_score']:.1f}"],
           ['Strict mission completion',_display_number(primary['completion_time_s'])+' s' if primary['completion_time_s'] is not None else 'Incomplete'],
           ['Packet delivery / mean latency',f"{100*packet_stats['pdr']:.3f}% / {1000*packet_stats['avg_latency']:.1f} ms"],
           ['Connectivity / total downtime',f"{primary['connectivity_availability_percent']:.3f}% / {primary['communication_downtime_s']:.3f} s"],
           ['Relay assignments / completed handoffs',f"{primary['relay_assignments']} / {primary['relay_handoffs_completed']}"],
           ['Post-fault source-group receipt recovery',', '.join(recoveries) if recoveries else 'No scheduled fault in this run'],
           ['Reconfiguration efficiency',f"{100*efficiency:.2f}% completed handoffs / dispatched replacements" if efficiency is not None else 'No replacement dispatch; undefined'],
           ['Collision pairs / minimum separation',f"{primary['collision_count']} / {primary['minimum_separation_m']:.4f} m"],
           ['Exhaustions / geofence violation steps',f"{primary['battery_exhaustions']} / {primary['geofence_violation_steps']}"]],[185,290],compact=True)
    p('These are project-defined measurements mapped to the published categories. Recovery uses fresh GCS receipts from the affected surviving-source group; efficiency uses the stated ratio. The organizer standardised log format and definitions were not supplied, so official equivalence is unconfirmed.')
    new();title('7. Simulation results')
    data=[['ID / case','PoIs','Time (s)','PDR %','Conn. %','Checks']]
    for index,(name,r) in enumerate(rows,1):
        data.append([f"R{index:02d}: {_case_label(r)}",f"{r['pois_completed']}/{r['pois_total']}",
                     _display_number(r['completion_time_s']) if r['completion_time_s'] is not None else 'Incomplete',
                     f"{100*r['packet_statistics']['pdr']:.2f}",f"{r['connectivity_availability_percent']:.2f}",
                     'PASS' if r['sample_constraints_passed'] else 'FAIL'])
    table(data,[145,45,75,65,65,80],compact=True)
    story.append(Spacer(1,7))
    picture_width=320 if len(rows)>8 else 390
    story.append(PdfImage(str(out/'trajectory.png'),width=picture_width,height=picture_width*720/1280))
    p('<b>Figure 3.</b> Measured kinematic replay from R01; any labelled camera inset uses the latest received simulated scene image.' if camera else '<b>Figure 3.</b> Measured kinematic replay from R01 with synthetic footprint observations; this mode supplies no camera imagery.')
    passed=sum(bool(r['sample_constraints_passed']) for _,r in rows)
    failed_keys=sorted({key for _,r in rows for key,value in r['checks'].items() if not value})
    p(f"All {len(rows)} supplied runs are shown: {passed} pass and {len(rows)-passed} fail. R01 minimum separation: {_display_number(primary['minimum_separation_m'],4)} m; longest sortie: {_display_number(primary['maximum_sortie_s'])} s; battery exhaustions: {primary['battery_exhaustions']}; reporting misses: {len(primary['reporting_deadline_misses'])}.")
    p('Failed checks across this matrix: '+escape(', '.join(failed_keys) if failed_keys else 'none')+'. Complete run names, metrics, fault/survivor results and manifests are in validation-results.json. No fault or incomplete run is relabelled as a successful submission.')
    new();title('8. Reproducibility, team and references')
    if v2:
        p('Qualified host: Windows 11, Python 3.10.11. From the supplied source folder: (1) <font face="Courier">python -m venv .venv</font>; (2) <font face="Courier">.venv/Scripts/python.exe -m pip install -r requirements-repro.txt</font>; (3) <font face="Courier">.venv/Scripts/python.exe run_simulation.py --scenario-file PATH/TO/RUN/scenario.json --output evidence/new-run</font>. Add <font face="Courier">--gui</font> for the live view. Faults and resources belong in the scenario JSON. Use a new output path for every run. On Linux use <font face="Courier">.venv/bin/python</font>; that host is not qualified here.')
    else:
        p('Create an isolated Python environment and install requirements-uavx.txt. Run <font face="Courier">python run_simulation.py --profile uavx --seed 42 --output evidence/new-run</font>. Add <font face="Courier">--disturbance outage</font> or <font face="Courier">--disturbance failure</font> for baseline faults. Use a new output directory.')
    p('Run <font face="Courier">python verify_submission.py PATH/TO/RUN</font> with the exact source snapshot. The verifier checks source-byte and artifact hashes plus consistency; different line endings can change source hashes. An evidence error blocks this artifact builder. A consistent fault run may still fail mission checks and is preserved as such. Hashes are not independent dynamics proof or tamperproof authority.')
    p('All runs retain metrics, events, packet outcomes, connectivity, trajectories and manifests. Schema-2 runs also retain realized scenario JSON, radio-service samples, capture metadata and the original/received PNGs. The demo replays recorded motion at 30x with a 5x window around configured faults; a labelled image inset appears only after its logged GCS receipt. It does not qualify physical flight.' if v2 else 'Runs retain metrics.json, events.json, packets.json, connectivity.json, trajectory.jsonl and manifest.json. The local demo replays recorded motion at approximately 30x and contains no camera evidence. The full source and exact commands accompany the package.')
    p('Team name, registration ID, institution, qualification, members and contributions were not supplied. The root repository license remains unresolved despite older MIT references. Dependency licences retain their own terms. These fields and rights must be resolved before final submission.')
    sub('References')
    p('[1] Supplied UAV-X/PUSHPAK 2026 challenge PDF, pp. 3-5 and 8. [2] Supplied sample scenario for GC 1.pptx, slide 1. [3] Inspected Garuda_OS energy_model.py, pas_client.py, visual_worker.py and shot_recorder.py. [4] Supplied Grand Challenge 1 report template. [5] AeroMesh source, scenarios and run manifests. [6] Panda3D 1.10.16 / TinyDisplay and Pillow (scene-camera rendering), where enabled.')
    sub('Submission checklist')
    if video_url:
        p('<b>Demonstration video:</b> <link href="'+escape(video_url,quote=True)+'">Open the recorded AeroMesh mission and received-camera demonstration</link>. The archive also includes the original MP4.')
    p('Prepared locally: source, installation instructions, architecture, working simulation, measured logs, report draft and demo. Outstanding: team/qualification details, official scenario and log-format compliance, organizer acceptance of resources/model, and final review. No email/form entry has been sent; the supplied email instructions conflict with the Forms URL in the request.')
    def footer(canvas,doc):
        canvas.setStrokeColor(colors.HexColor('#c3d4df'));canvas.line(20*mm,17*mm,190*mm,17*mm)
        canvas.setFont('Helvetica',8);canvas.setFillColor(colors.HexColor('#4b6170'))
        canvas.drawString(20*mm,12*mm,'AeroMesh UAV-X | Technical draft | Kinematic simulation')
        canvas.drawRightString(190*mm,12*mm,str(doc.page))
    doc=SimpleDocTemplate(str(out/'UAVX-technical-proposal-draft.pdf'),pagesize=(210*mm,297*mm),
                          leftMargin=20*mm,rightMargin=20*mm,topMargin=18*mm,bottomMargin=23*mm)
    doc.build(story,onFirstPage=footer,onLaterPages=footer)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--video-url',help='Evaluator-accessible video URL to include in the draft')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    if str(root) not in sys.path: sys.path.insert(0,str(root))
    from verify_submission import verify
    validation=[]
    for run in args.runs:
        result=verify(run)
        if result['evidence_errors']:
            parser.error(f'Invalid source/evidence for {run}: {result["evidence_errors"]}')
        validation.append(dict(run=str(run),verification=result,
                               metrics=json.loads((run/'metrics.json').read_text(encoding='utf-8')),
                               manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))))
    args.output.mkdir(parents=True,exist_ok=False)
    make_video(args.runs[0],args.output);make_pdf(args.runs,args.output,args.video_url)
    (args.output/'validation-results.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
    (args.output/'README.txt').write_text('TECHNICAL DRAFT - NOT SUBMITTED\nDemo is a measured kinematic replay at 30x, with 5x fault windows and received-only simulated scene-camera imagery where available.\nTeam identity, official scenario/log compliance and final submission review remain outstanding.\nSource and reproduction: repository README.md and docs/UAVX_VALIDATION.md.\n',encoding='utf-8')


if __name__=='__main__': main()
