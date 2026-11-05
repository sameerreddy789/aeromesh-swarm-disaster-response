"""Build a clearly labelled proposal draft and trajectory video from measured runs.

Optional authoring dependencies: reportlab, Pillow, and ffmpeg on PATH.
Usage: python scripts/build_uavx_submission.py --runs RUN1 RUN2 ... --output DIR
"""
import argparse
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


def frame_image(frame, tasks, index=0):
    image=Image.new('RGB',(1280,720),'#f4f7fa');d=ImageDraw.Draw(image)
    font_path=Path('C:/Windows/Fonts/arial.ttf')
    font=ImageFont.truetype(str(font_path),18) if font_path.exists() else ImageFont.load_default()
    big=ImageFont.truetype(str(font_path),28) if font_path.exists() else font
    def pos(p): return (90+(p[0]+75)*.58,640-(p[1]+500)*.52)
    d.text((35,22),'AeroMesh | UAV-X trajectory replay',fill='#142e46',font=big)
    d.text((35,60),f"Simulation {frame['time_s']:.1f} s  |  30x replay  |  abstract kinematics",fill='#456070',font=font)
    d.rectangle((pos((75,500))[0],120,pos((1075,-500))[0],640),outline='#8098a6',width=2)
    d.text((790,145),'Recorded simulation states',fill='#142e46',font=big)
    d.text((790,194),'Blue: relay   Orange: scout',fill='#456070',font=font)
    d.text((790,225),'Gray: grounded   Red: failed',fill='#456070',font=font)
    d.text((790,275),'Range links are reconstructed',fill='#456070',font=font)
    d.text((790,304),'from recorded 3D positions.',fill='#456070',font=font)
    d.text((790,365),'No onboard camera imagery',fill='#142e46',font=font)
    d.text((790,394),'or physical flight is shown.',fill='#142e46',font=font)
    nodes={u['id']:u['position'] for u in frame['uavs'] if u['role']!='failed'}
    nodes[-1]=(0,0,0)
    import math
    keys=list(nodes)
    for j,a in enumerate(keys):
        for b in keys[j+1:]:
            if math.dist(nodes[a],nodes[b])<=100 and (nodes[a][2]>1 or nodes[b][2]>1):
                d.line((*pos(nodes[a]),*pos(nodes[b])),fill='#c3d4df',width=1)
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
    d.text((35,677),'Synthetic PoIs: red pending / green received  |  See metrics.json for every constraint result',fill='#456070',font=font)
    return image


def make_video(run,out):
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg: raise RuntimeError('ffmpeg is required to build the demonstration video')
    metrics=json.loads((run/'metrics.json').read_text())
    frames=[json.loads(line) for line in (run/'trajectory.jsonl').read_text().splitlines()]
    command=[ffmpeg,'-hide_banner','-loglevel','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','15','-i','-',
             '-an','-c:v','libx264','-preset','fast','-crf','24','-pix_fmt','yuv420p',str(out/'demo.mp4')]
    proc=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        for i,frame in enumerate(frames): proc.stdin.write(frame_image(frame,metrics['tasks'],i).tobytes())
        proc.stdin.close()
        error=proc.stderr.read().decode(errors='replace');code=proc.wait()
        if code: raise RuntimeError(error)
    except Exception:
        proc.kill();proc.wait();raise
    frame_image(frames[len(frames)//2],metrics['tasks']).save(out/'trajectory.png')


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


def make_pdf(runs,out):
    rows=[(p.name,json.loads((p/'metrics.json').read_text())) for p in runs]
    nominal=rows[0][1]
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='BodyU',fontName='Helvetica',fontSize=10,leading=14,spaceAfter=10,textColor=colors.HexColor('#233d50')))
    styles.add(ParagraphStyle(name='SmallU',fontName='Helvetica',fontSize=8,leading=11,spaceAfter=7,textColor=colors.HexColor('#4b6170')))
    story=[]
    def p(text):story.append(Paragraph(text,styles['BodyU']))
    def title(text):story.append(Paragraph(text,styles['Heading1']));story.append(Spacer(1,8))
    def sub(text):story.append(Paragraph(text,styles['Heading2']))
    def new():story.append(PageBreak())
    def table(data,widths):
        wrapped=[[Paragraph(str(c),styles['SmallU']) for c in row] for row in data]
        t=Table(wrapped,colWidths=widths,repeatRows=1,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e9f1f5')),('VALIGN',(0,0),(-1,-1),'TOP'),
                              ('BOTTOMPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),7),
                              ('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor('#28758a'))]))
        story.append(t)
    title('AeroMesh | UAV-X')
    p('<b>Technical proposal draft â€” 5 November 2026</b>')
    p('PUSHPAK Grand Challenge 2026 Â· Resilient BVLOS Swarm Challenge')
    table([['Submission identity','Status'],['Team name / registration ID','Not supplied'],['Institution / category','Not supplied'],
           ['Leader / members / contributions','Not supplied'],['Stage 2 qualification','Not confirmed'],['Document status','Technical draft; not submitted']], [180,295])
    sub('1. Abstract / executive summary')
    p('AeroMesh explores autonomous disaster reconnaissance with a range-limited multi-hop aerial network. A ground station outside a simulated disaster area assigns emerging points of interest to a mobile relay chain. The supplied sample constraints govern flight duration, mission deadline, range, speed, altitude, separation and report latency. The implementation combines energy admission, stable relay identities, prioritized re-tasking, reserve-driven return and explicit packet-delivery evidence. Garuda design patterns inform measured landing and timestamped observations; its single-aircraft simulator backend is not transplanted into the swarm.')
    p(f"The nominal seed-42 run completes {nominal['pois_completed']}/{nominal['pois_total']} PoIs with {nominal['fleet_size']} vehicles. Completion time is {nominal['completion_time_s']} seconds; connectivity availability is {nominal['connectivity_availability_percent']:.3f}%. Every claim is tied to recorded simulation states and packet events. Additional nominal, outage and failure runs expose robustness limits. This is an abstract, centralized kinematic proof of concept with synthetic observations and an idealized radio-service model. It does not qualify physical aircraft, onboard perception, distributed middleware or deployment readiness. Team identity, organizer log conversion and final submission review remain outstanding.")
    new();title('2. Problem and mission analysis')
    p('An earthquake or landslide disrupts terrestrial communications. UAVs must survey assigned disaster sites and deliver situational information to a GCS through aerial relays. Coverage alone is insufficient: reporting, network continuity, endurance and safe recovery must be measured together [1,2].')
    table([['Supplied sample constraint','Configured value'],['Operational disaster area','1000 m Ã— 1000 m'],['Mission / per-sortie flight limit','2700 s / 1200 s'],['Radio / speed / altitude','100 m / 5 m/s / 100 m'],['Minimum separation / reporting deadline','20 m / 10 s'],['PoIs','10, seeded random positions and spawn times'],['Completion','Survey and receipt of all PoIs; own-pad landing for every UAV']], [245,230])
    sub('2.1 Assumptions and trade-offs')
    p('GCS=(0,0,0); the disaster square is x=75..1075 m and y=-500..500 m. A declared flight geofence includes staging and approach. Home pads are spaced 25 m apart. These exact coordinates and fleet size are team assumptions, not organizer-specified geometry. Thirty vehicles provide relay coverage and a replacement cohort; an 18-vehicle development run failed full coverage.')
    p('The first task is present at t=0 and remaining spawn times lie in 60..1800 s. A high-priority task can preempt work. Reserving aircraft for connectivity reduces immediate survey capacity; returning relays before reserves are exhausted can create temporary communication loss. Both trade-offs are retained in the logs.')
    new();title('3. System and software architecture')
    story.append(architecture());p('<b>Figure 1.</b> System data flow: mission demand produces bounded motion; sensed observations become packets and only GCS receipts establish delivery.')
    table([['Module','Responsibility'],['simulation/uavx.py','Scenario, central planner, relay roles, kinematic motion, sensor footprint, measured results'],['core/mission_safety.py','Finite-input energy admission, settled landing predicate, exact swept separation helper'],['comm/mesh_router.py','3D hard range, routing, queues, per-hop loss/retry, immutable packet outcomes'],['verify_submission.py','Artifact hashes, packet/receipt reconciliation and explicit failed checks']], [205,270])
    story.append(software_diagram())
    p('<b>Figure 2 / software pipeline.</b> Spawned task -&gt; priority selection -&gt; role/energy admission -&gt; movement candidate -&gt; separation/geofence/connectivity veto -&gt; state update -&gt; observation timestamp -&gt; queued packet -&gt; GCS receive timestamp -&gt; evidence checks. All runtime intervals use one simulation clock.')
    p('Python 3.10, NumPy and NetworkX form the minimal runtime. A separate pinned environment supports reproduction. No hardware adapter is imported by the UAV-X execution path. The older scenario runner remains available under an explicit legacy profile.')
    new();title('4. Swarm autonomy, roles and energy')
    p('A central planner selects the highest-priority available unfinished task. It places relay slots along a radial chain with nominal 85 m spacing, leaving margin below the 100 m link cutoff. Stable slot ownership reduces crossing trajectories during re-tasking. Higher-priority tasks preempt the chain; unfinished work remains pending.')
    p('Task admission accounts for travel, survey dwell, relay wait, return distance, landing and reserve against both remaining endurance and the 45-minute deadline. A constant endurance budget implements the supplied 20-minute limit; it is not a calibrated battery model. Recharge takes 720 s from empty as a declared assumption.')
    p('Relays with declining reserve dispatch replacements. A completed handoff requires arrival and connectivity, rather than a command acknowledgement. Replacements in transit are reconciled if an owner leaves early; an already-returning replacement cannot be promoted. Conservative reserve return takes priority over waiting indefinitely for a perfect handoff.')
    p('Nominal relay altitude is 40 m, replacement approach 65 m, and return routing uses 90 m where feasible. Horizontal escape candidates prevent a descending/ascending aircraft from repeatedly backing away from an overhead relay. Every accepted segment remains subject to the swept-separation and geofence checks.')
    sub('Scope and limitations')
    p('This greedy planner is not complete or optimal. It may stall, leave PoIs unfinished, or incur downtime under resource shortages and disturbances. It has central state access and is not a qualified decentralized CBBA deployment. No universal completion or safety guarantee for arbitrary physical trajectories is asserted.')
    new();title('5. Resilient BVLOS communication')
    p('MeshRouter applies a 100 m cutoff to three-dimensional link distance. Within range, the development radio model uses 2% independent per-attempt loss. The router advances a packet by at most one hop per 0.5 s tick, including queue/retry wait in latency. Parallel service is unbounded: bandwidth contention, interference physics and real RF propagation remain outside this model.')
    p('Every logical packet records an immutable offer, then at most one terminal delivery or drop. No-route traffic is included in the denominator. Duplicate IDs cannot inflate delivery counts. Failed or missing next-hop/current nodes cause reroute or an explicit drop. Finding an alternative route does not count as successful recovery until a rerouted packet arrives.')
    p('A synthetic PoI observation carries its original detection timestamp through the mesh. The first GCS receipt determines reporting delay. Late or absent receipts fail the deadline; the detector timestamp is never refreshed to improve a score. Survey completion and delivery are separate state transitions.')
    p('The outage benchmark disables one active relay radio for 30 s after t=900 s. The failure benchmark permanently disables an active relay. Recovery requires fresh post-fault packets from the affected surviving source set; unrecovered cases stay null. Under the abstract failure model, the disabled vehicle remains an obstacle and fails all-vehicle landing.')
    p('Connectivity availability is evaluated from range-graph changes during each linear movement interval. Communication downtime is retained even when the mission later completes. A strict continuous-connectivity check can therefore fail in a recovering outage run without concealing mission progress.')
    new();title('6. Safety and technical merit')
    p('Safety predicates reject non-finite values. Separation uses closest approach over simultaneous linear segments, including endpoints; safe sampled endpoints alone are insufficient. Proposed speed and geofence violations are vetoed. Observed maxima, minimum separation, collision pairs and violation counts remain in the report.')
    p('Landing is confirmed only at the vehicleâ€™s own home pad using fresh, settled pose/velocity evidence and a disarmed state. Two seconds of consecutive settling are required. A failed aircraft, an empty fleet, a return command or a hover near home cannot establish successful mission completion. Evidence timestamps match the observation boundary.')
    p('Garuda contributions are design inputs: measured completion rather than acknowledgement, target-plus-return energy budgeting, paired observation timestamps, and evidence integrity [3]. No Garuda source, assets or weight files were copied. Its unmodified AirSim client reloads the scene and would be unsuitable as one independent client per swarm vehicle.')
    p('The technical contribution is a traceable coupling of relay planning, bounded motion, reporting deadlines and measured end states. These are established engineering ideas applied to this challenge; no claim of a new optimal routing or control algorithm is made. Hidden-scenario comparison, calibrated energy and RF models, and high-fidelity multi-UAV flight are future qualification work.')
    p('An isolated environment passed the documented targeted regression suite. Tests cover unreachable traffic, duplicate packet IDs, edge-count hop limits, stale/invalid landing evidence, crossing paths, future PoIs, replacement cancellation, fault injection, and deterministic CLI outputs. Full scenario outcomes are reported separately on the next page.')
    new();title('7. Simulation results')
    data=[['Run','PoIs','Time (s)','PDR (%)','Conn. (%)','All checks']]
    for name,r in rows:
        data.append([name.replace('qualified-','').replace('final2-','').replace('final-',''),f"{r['pois_completed']}/{r['pois_total']}",r['completion_time_s'] or 'Incomplete',f"{100*r['packet_statistics']['pdr']:.2f}",f"{r['connectivity_availability_percent']:.2f}",'PASS' if r['sample_constraints_passed'] else 'FAIL'])
    table(data,[125,50,80,65,75,80])
    story.append(Spacer(1,12));story.append(PdfImage(str(out/'trajectory.png'),width=475,height=267))
    p('<b>Figure 3.</b> Reconstructed trajectory snapshot from the first measured run. This is a kinematic visualization with synthetic PoIs, not camera imagery.')
    p(f"Nominal minimum separation: {nominal['minimum_separation_m']:.6f} m. Longest sortie: {nominal['maximum_sortie_s']:.1f} s. Battery exhaustions: {nominal['battery_exhaustions']}. Reporting misses: {len(nominal['reporting_deadline_misses'])}. Per-run JSON contains every failed check and fault recovery observation. Partial runs and faults are not relabelled as successful submissions.")
    for name,r in rows:
        failed=[key.replace('_',' ') for key,value in r['checks'].items() if not value]
        if failed:
            fault=r.get('disturbance')
            recovery=(fault['recovered_s']-fault['time_s']) if fault and fault.get('recovered_s') is not None else None
            detail=f" Service recovery: {recovery:.1f} s." if recovery is not None else ''
            p(f"<b>{name.replace('final2-','')}:</b> Failed checks: {', '.join(failed)}.{detail}")
    new();title('8. Reproducibility, team and references')
    p('Create an isolated Python environment and install requirements-uavx.txt. Run: <font face="Courier">python run_simulation.py --profile uavx --seed 42 --output evidence/run-42</font>. Use <font face="Courier">--disturbance outage</font> or <font face="Courier">--disturbance failure</font> for fault cases. Verify with <font face="Courier">python verify_submission.py evidence/run-42</font>. Exact commands and targeted tests are in README.md.')
    p('Each run retains metrics.json, events.json, packets.json, connectivity.json, trajectory.jsonl and manifest.json. The manifest hashes execution sources and artifacts. A proposal/video can be regenerated with scripts/build_uavx_submission.py using those runs. The included demo.mp4 replays recorded positions at 30x; it does not establish physical or onboard-vision performance.')
    p('Team name, registration ID, institution, eligibility, membership and individual contributions were not supplied. These fields must be completed before submission. The original AeroMesh repositoryâ€™s historical MIT claim has no root license file; ownership and licensing need confirmation. Third-party dependencies retain their own terms.')
    sub('References')
    p('[1] UAV-X: Resilient BVLOS Swarm Challenge, supplied PUSHPAK Grand Challenge 2026 PDF, pp. 3â€“5 and 8. [2] sample scenario for GC 1.pptx, supplied slide 1. [3] Garuda_OS, inspected energy_model.py, pas_client.py, visual_worker.py and shot_recorder.py; evidence limited to its documented single-drone fixture. [4] Report Template â€“ Grand challenge 1.pdf, supplied report structure. [5] AeroMesh source and run manifests supplied with this draft.')
    sub('Submission checklist')
    p('Prepared: source changes, installation instructions, architecture, working abstract simulation, measured logs, this report draft and local demo. Outstanding: team details, official Stage 2 log/schema conversion, organizer scenario validation, final review and an evaluator-accessible video link. No email/form submission has been made. The supplied PDF specifies email submission; the conflicting Forms URL was not treated as authority to submit.')
    def footer(canvas,doc):
        canvas.setStrokeColor(colors.HexColor('#c3d4df'));canvas.line(20*mm,17*mm,190*mm,17*mm)
        canvas.setFont('Helvetica',8);canvas.setFillColor(colors.HexColor('#4b6170'))
        canvas.drawString(20*mm,12*mm,'AeroMesh UAV-X | Technical draft | Abstract simulation only')
        canvas.drawRightString(190*mm,12*mm,str(doc.page))
    doc=SimpleDocTemplate(str(out/'UAVX-technical-proposal-draft.pdf'),pagesize=(210*mm,297*mm),
                          leftMargin=20*mm,rightMargin=20*mm,topMargin=18*mm,bottomMargin=23*mm)
    doc.build(story,onFirstPage=footer,onLaterPages=footer)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    make_video(args.runs[0],args.output);make_pdf(args.runs,args.output)
    (args.output/'README.txt').write_text('TECHNICAL DRAFT â€” NOT SUBMITTED\nDemo is an abstract trajectory replay at 30x, not physical flight.\nTeam identity, official scenario/log compliance and accessible video hosting remain outstanding.\nSource and reproduction: repository README.md and docs/UAVX_VALIDATION.md.\n',encoding='utf-8')


if __name__=='__main__': main()
