"""Reconstruct image receipts and check bounded-radio/scenario evidence."""
import base64
from collections import defaultdict
import hashlib
from io import BytesIO
import json
import math


def verify_images(folder, metrics, sensing, delivered, receipts, agree, events):
    from verify_submission import integer, number, vector
    mode = metrics['scenario']['camera_mode']
    agree('sensing mode', sensing['mode'], mode)
    agree('sensing summary mode', metrics['sensing']['mode'], mode)
    observations = sensing['observations']
    agree('camera capture count', metrics['sensing']['captures'], len(observations))
    if mode == 'footprint':
        if observations or sensing['scene'] is not None:
            raise ValueError('Footprint run must not claim scene-camera observations')
        agree('camera receipt count', metrics['sensing']['received_images'], 0)
        agree('camera received bytes', metrics['sensing']['received_bytes'], 0)
        return receipts, 0
    from PIL import Image
    scene = dict(sensing['scene'])
    scene_digest = scene.pop('scene_sha256')
    canonical = json.dumps(scene, sort_keys=True, separators=(',',':'), allow_nan=False).encode('utf-8')
    agree('camera scene fingerprint', hashlib.sha256(canonical).hexdigest(), scene_digest)
    agree('camera scene seed', scene['seed'], metrics['seed'])
    agree('camera scene tasks', scene['tasks'], sorted(
        [{k:p[k] for k in ('id','position','spawn_s')} for p in metrics['tasks']], key=lambda p:p['id']))
    if scene['renderer'] != 'Panda3D TinyDisplay CPU' or not scene['renderer_version']:
        raise ValueError('Invalid scene renderer provenance')
    tasks = {p['id']:p for p in metrics['tasks']}
    images = {o['capture_id']:o for o in observations}
    if len(images)!=len(observations) or not {o['poi_id'] for o in observations} <= set(tasks):
        raise ValueError('Unknown camera PoI')
    rebuilt_receipts, count, received_bytes = {}, 0, 0
    chunks = defaultdict(dict)
    completed_at = {}
    for event in sorted(delivered,key=lambda e:(e['time_s'],e['packet_id'])):
        payload = event.get('payload') or {}
        if 'poi_id' not in payload:
            continue
        pid = payload['poi_id']
        identity=payload.get('capture_id')
        if payload.get('kind') != 'poi_image_chunk' or identity not in images:
            raise ValueError('Camera run has unknown/non-image PoI traffic')
        observation = images[identity]
        agree('image PoI',pid,observation['poi_id'])
        agree('image packet source', event['source_id'], observation['uav_id'])
        agree('image acquisition time', payload['observation_time_s'], observation['acquired_at_s'])
        agree('image original detection time', payload['detection_time_s'], observation['detection_time_s'])
        agree('image packet digest', payload['image_sha256'], observation['sha256'])
        agree('image fragment count', payload['chunk_count'], observation['chunk_count'])
        index = integer(payload['chunk_index'], 'chunk index', maximum=observation['chunk_count']-1)
        content = base64.b64decode(payload['data_b64'],validate=True)
        expected = (folder/observation['capture_file']).read_bytes()[index*512:(index+1)*512]
        if content != expected:
            raise ValueError('Delivered image bytes do not match the captured fragment')
        if index in chunks[identity] and chunks[identity][index] != content:
            raise ValueError('Conflicting image fragment')
        chunks[identity][index] = content
        if identity not in completed_at and len(chunks[identity]) == observation['chunk_count']:
            completed_at[identity] = event['time_s']
            rebuilt_receipts.setdefault(pid,[]).append(event)
    received_pois=set()
    for identity, observation in images.items():
        pid=observation['poi_id']
        task = tasks[pid]
        capture = (folder/observation['capture_file']).read_bytes()
        integer(observation['byte_length'],'PNG size',1,4_000_000)
        agree('PNG length', len(capture), observation['byte_length'])
        agree('PNG fingerprint', hashlib.sha256(capture).hexdigest(), observation['sha256'])
        agree('capture scene', observation['scene_sha256'], scene_digest)
        agree('capture renderer', observation['renderer'], scene['renderer'])
        agree('capture renderer version', observation['renderer_version'], scene['renderer_version'])
        agree('image fragment inventory', observation['chunk_count'], math.ceil(len(capture)/512))
        integer(observation['uav_id'],'camera source',0,metrics['fleet_size']-1)
        number(observation['acquired_at_s'],'camera acquisition time',minimum=task['detection_s'],maximum=metrics['elapsed_time_s'])
        agree('capture first detection time',observation['detection_time_s'],task['detection_s'])
        if observation['attempt']==0:
            agree('capture source', observation['uav_id'], task['detected_by'])
            agree('capture time', observation['acquired_at_s'], task['detection_s'])
        else:
            prior=[o for o in observations if o['poi_id']==pid and o['attempt']==observation['attempt']-1]
            if len(prior)!=1 or prior[0]['acquired_at_s']>observation['acquired_at_s']:
                raise ValueError('Recapture lacks ordered previous observation')
            matching=[e for e in events if e['type']=='poi_recaptured' and e['capture_id']==identity]
            if len(matching)!=1:
                raise ValueError('Recapture lacks event evidence')
            agree('recapture source',matching[0]['uav_id'],observation['uav_id'])
            agree('recapture timestamp',matching[0]['observation_time_s'],observation['acquired_at_s'])
            agree('recapture deadline',matching[0]['detection_time_s'],task['detection_s'])
        agree('capture scene visibility', observation['active_scene_parcels'],
              sorted(p['id'] for p in metrics['tasks'] if p['spawn_s'] <= observation['acquired_at_s']))
        x,y,z = vector(observation['position_xyz_m'],'camera position')
        if math.hypot(x-task['position'][0],y-task['position'][1])>5+1e-7 or not 30<=z<=70:
            raise ValueError('Camera pose outside the survey footprint')
        with Image.open(BytesIO(capture)) as image:
            if image.format != 'PNG' or image.mode != 'RGB':
                raise ValueError('Capture is not RGB PNG imagery')
            agree('camera resolution', list(image.size), [metrics['scenario']['camera_width'],metrics['scenario']['camera_height']])
            image.verify()
        when = completed_at.get(identity)
        agree('complete image receipt time', observation['received_at_s'], when)
        if when is None:
            if observation['received_file'] is not None:
                raise ValueError('Undelivered image has a GCS file')
        else:
            if observation['received_file'] is None:
                raise ValueError('Delivered image lacks a GCS file')
            reassembled = b''.join(chunks[identity][i] for i in range(observation['chunk_count']))
            if reassembled != capture or (folder/observation['received_file']).read_bytes() != reassembled:
                raise ValueError('GCS image differs from received bytes')
            if pid not in received_pois and when==min(e['time_s'] for e in rebuilt_receipts[pid]):
                received_pois.add(pid)
                count += 1
                received_bytes += len(reassembled)
    agree('image receipt count', metrics['sensing']['received_images'], count)
    agree('image received bytes', metrics['sensing']['received_bytes'], received_bytes)
    return rebuilt_receipts, count


def verify_radio(folder, metrics, packets, agree):
    from verify_submission import number, integer
    samples = json.loads((folder/'radio-service.json').read_text(encoding='utf-8'))
    config, stats = metrics['scenario'], metrics['packet_statistics']['radio_service']
    agree('radio capacity', stats['node_bandwidth_bytes_s'], config['radio_bytes_per_s'])
    agree('radio queue limit', stats['queue_limit_bytes'], config['radio_queue_bytes'])
    if stats['duplex'] != 'shared_tx_rx_half_duplex':
        raise ValueError('Unknown radio duplex model')
    if stats['wire_encoding'] != 'canonical_utf8_json_with_explicit_payload_size_padding':
        raise ValueError('Unknown radio wire encoding')

    def node_map(value, label, maximum=math.inf, integral=False):
        if not isinstance(value, dict):
            raise ValueError('Invalid radio node map: '+label)
        for node, amount in value.items():
            identity = integer(int(node), 'radio node', -1, metrics['fleet_size']-1)
            if node != str(identity):
                raise ValueError('Noncanonical radio node identity')
            (integer if integral else number)(amount, label, maximum=maximum)
        return value

    def at_least(label, actual, minimum):
        # These are consistency lower bounds, not reconstruction of each RF
        # attempt. In-flight, failed and retried traffic can require more bytes.
        if actual + max(1e-6, minimum*1e-10) < minimum:
            raise ValueError('Radio evidence understates '+label)

    previous, totals = 0., defaultdict(float)
    if not isinstance(samples, list):
        raise ValueError('Radio ledger must be an array')
    for sample in samples:
        time = number(sample['time_s'],'radio time',minimum=previous,maximum=metrics['elapsed_time_s'])
        duration = number(sample['elapsed_s'],'radio duration',maximum=metrics['dt_s']+1e-7)
        agree('radio timeline', duration, time-previous)
        for node, value in node_map(sample['node_service_bytes'], 'radio serviced bytes',
                                    maximum=config['radio_bytes_per_s']*duration+1e-6).items():
            totals[node] += value
        previous = time
    if metrics['elapsed_time_s'] and (not samples or not math.isclose(previous+metrics['dt_s'],metrics['elapsed_time_s'])):
        raise ValueError('Radio ledger omits an execution interval')
    for key in ('node_service_bytes','node_tx_bytes','node_rx_bytes','queue_bytes','peak_queue_bytes'):
        node_map(stats[key], key, maximum=config['radio_queue_bytes'] if 'queue' in key else math.inf,
                 integral='queue' in key)
    # Include TX-only or RX-only keys too; extra totals cannot hide outside the
    # service-ledger key set. Nodes absent from a sparse map contribute zero.
    nodes = set(totals)
    for key in ('node_service_bytes','node_tx_bytes','node_rx_bytes','queue_bytes','peak_queue_bytes'):
        nodes.update(stats[key])
    for node in nodes:
        agree('radio service ledger', totals.get(node,0), stats['node_service_bytes'].get(node,0))
        agree('shared TX/RX usage', stats['node_service_bytes'].get(node,0),
              stats['node_tx_bytes'].get(node,0)+stats['node_rx_bytes'].get(node,0))
        at_least('peak queue occupancy', stats['peak_queue_bytes'].get(node,0), stats['queue_bytes'].get(node,0))
    transmitted, received = sum(stats['node_tx_bytes'].values()), sum(stats['node_rx_bytes'].values())
    agree('radio TX/RX totals', transmitted, received)
    started = integer(stats['attempts_started'], 'radio attempts started')
    completed_attempts = integer(stats['attempts_completed'], 'radio attempts completed', maximum=started)
    aborted = integer(stats['partial_attempts_aborted'], 'radio attempts aborted', maximum=started-completed_attempts)
    integer(stats['queue_overflow_drops'], 'radio queue overflow drops')
    offers, terminals = {}, set()
    delivery_bytes = delivered_hop_bytes = delivered_hops = 0
    source_bytes = defaultdict(int)
    payload_padding = {'telemetry':256, 'poi_report':2048, 'poi_image_chunk':0}
    encoding = dict(sort_keys=True, separators=(',',':'), allow_nan=False, ensure_ascii=False)
    for event in packets:
        size = integer(event['wire_size_bytes'],'packet wire bytes',1)
        if event['type']=='offered':
            offers[event['packet_id']] = size
            payload = event['payload']
            kind = (payload or {}).get('kind')
            if kind not in payload_padding:
                raise ValueError('Unknown UAV-X radio payload kind')
            payload_bytes = json.dumps(payload, **encoding).encode('utf-8')
            # The current UAV-X protocol sends DATA frames with these fixed
            # padding rules. Reconstruct encoding independently of MeshRouter.
            envelope = json.dumps(dict(packet_id=event['packet_id'], packet_type='data',
                                       source_id=event['source_id'], dest_id=event['dest_id'],
                                       creation_time_s=event['creation_time_s'], payload=payload), **encoding).encode('utf-8')
            expected_size = len(envelope) + max(0, payload_padding[kind]-len(payload_bytes))
            agree('serialized radio frame size', size, expected_size)
        else:
            agree('immutable wire size', size, offers[event['packet_id']])
            terminals.add(event['packet_id'])
            if event['type']=='delivered':
                hops = integer(event['hops'], 'radio delivered hops', 1, 32)
                delivery_bytes += size
                delivered_hop_bytes += size*hops
                delivered_hops += hops
                source_bytes[str(event['source_id'])] += size
    pending = {identity:size for identity,size in offers.items() if identity not in terminals}
    agree('resident queued packet bytes', sum(stats['queue_bytes'].values()), sum(pending.values()))
    if started-completed_attempts-aborted > len(pending):
        raise ValueError('More partial radio attempts than queued packets')
    at_least('completed link attempts', completed_attempts, delivered_hops)
    at_least('transmitted successful-hop bytes', transmitted, delivered_hop_bytes)
    at_least('received successful-hop bytes', received, delivered_hop_bytes)
    at_least('GCS received bytes', stats['node_rx_bytes'].get('-1',0), delivery_bytes)
    for node, amount in source_bytes.items():
        at_least('source transmitted bytes', stats['node_tx_bytes'].get(node,0), amount)
    if offers:
        at_least('completed-attempt serialization', transmitted, completed_attempts*min(offers.values()))
    elif started or transmitted or received:
        raise ValueError('Radio service has no offered traffic')
    agree('radio overflow drops', stats['queue_overflow_drops'], sum(e['type']=='dropped' and e.get('reason')=='queue_overflow' for e in packets))


def verify_faults(metrics, events, delivered, agree):
    from verify_submission import integer, number
    specs = metrics['scenario']['faults']
    records = metrics['faults']
    seen = set()
    injections = [e for e in events if e['type']=='scenario_fault_injected']
    agree('fault injection count',len(injections),len(records))
    for record in records:
        identity = integer(record['fault_id'],'fault identity',maximum=len(specs)-1)
        if identity in seen:
            raise ValueError('Repeated scheduled fault identity')
        seen.add(identity)
        spec = specs[identity]
        agree('fault kind',record['kind'],spec['kind'])
        agree('fault scheduled time',record['scheduled_s'],spec['time_s'])
        agree('fault duration',record['duration_s'],spec.get('duration_s'))
        stamp = number(record['time_s'],'fault injection time',minimum=spec['time_s'],maximum=metrics['elapsed_time_s'])
        target = record['target']
        if isinstance(spec['target'],(int,list)):
            agree('explicit fault target',target,spec['target'])
        if record['kind']=='link_outage':
            if not isinstance(target,list) or len(set(target))!=2:
                raise ValueError('Invalid failed link')
            for node in target: integer(node,'fault link node',-1,metrics['fleet_size']-1)
        else:
            integer(target,'fault UAV',0,metrics['fleet_size']-1)
        matched = [e for e in injections if e['fault_id']==identity]
        if len(matched)!=1:
            raise ValueError('Fault lacks unique injection event')
        for key in ('kind','target','scheduled_s','time_s','affected_sources'):
            agree('fault event '+key,record[key],matched[0][key])
        affected = record['affected_sources']
        if len(set(affected))!=len(affected):
            raise ValueError('Duplicate fault source group')
        for node in affected: integer(node,'affected source',0,metrics['fleet_size']-1)
        first = {uid:min((e['time_s'] for e in delivered if e['source_id']==uid and e['creation_time_s']>=stamp),default=None)
                 for uid in affected}
        agree('post-fault received sources',set(record['delivered_after']),{uid for uid,t in first.items() if t is not None})
        expected = max(first.values()) if first and all(t is not None for t in first.values()) else None
        agree('post-fault recovery time',record['recovered_s'],expected)
        if record['duration_s'] is None:
            agree('permanent failure clearing',record['cleared_s'],None)
        else:
            cleared = next((e['time_s'] for e in events if e['type']=='scenario_fault_cleared' and e['fault_id']==identity),None)
            agree('fault clearing event',record['cleared_s'],cleared)
            if cleared is not None:
                number(cleared,'fault clearance',minimum=stamp+record['duration_s'],maximum=metrics['elapsed_time_s'])
