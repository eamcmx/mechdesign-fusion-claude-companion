# FlapDetail (Chapter 53) against the acceptance criteria of its prompt, measured on what Fusion holds after the run:
# the motor unit's bodies along the actuator's axis; the gearhead's shaft, its key, the pinion's hub, the end screw and
# washer; the hardened zones and the faces' classes; no interference at 0 to 40 deg; the refusal.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import flap_detail as FDET
importlib.reload(FDET)
import adsk.core

PATH = '__COMPANION__/projects/flap_actuation/FlapDetail/FlapDetail.py'
OUTER = ('Actuator housing', 'Gear cover', 'Cover screw', 'Trunnion', 'Motor unit', 'Motor screw', 'Encoder', 'Piston rod',
         'Rod seal', 'Jam nut', 'Rod end')


def ybox(bodies):
    """The bodies' extent in the actuator's frame (its axis y), mm: their boxes in the root's frame turned back by
    the actuator's occurrence (posed flap up, the actuator swung a little about the gimbal)."""
    return [round(v * 10, 3) for v in fc_box(bodies)]


def fc_box(bodies):
    bb = None
    for b in bodies:
        x = b.boundingBox
        v = [x.minPoint.x, x.maxPoint.x, x.minPoint.y, x.maxPoint.y, x.minPoint.z, x.maxPoint.z]
        bb = v if bb is None else [min(bb[0], v[0]), max(bb[1], v[1]), min(bb[2], v[2]), max(bb[3], v[3]),
                                   min(bb[4], v[4]), max(bb[5], v[5])]
    return bb


def native_bodies(o):
    return [b for b in o.component.bRepBodies]


def attr(f, name='fit'):
    a = f.attributes.itemByName('mdfc', name)
    return a.value if a else None


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    named = lambda nm: [o for o in leaves if o.component.name.split(' (')[0] == nm]
    G, M, P = FDET.GEARHEAD, FDET.MOTOR, FDET.PIN
    last = ns['_last']
    # the motor unit, in its component's frame (the actuator's, the unit sits at its parent's origin)
    mu = named('Motor unit')[0]
    spans = {}
    for b in mu.component.bRepBodies:
        bx = b.boundingBox
        spans[b.name] = (round(bx.minPoint.y * 10, 2), round(bx.maxPoint.y * 10, 2), round((bx.maxPoint.x - bx.minPoint.x) * 10, 2))
    want = {'Flange': (40.0, 46.0, 60.0), 'Gearhead GP 52 C 26-1': (46.0, 111.0, 52.0),
            'Motor EC-i 52 180 W': (111.0, 191.1, 52.0), 'Brake to specification': (191.1, 216.0, 52.0)}
    fc.ok(spans == want, 'the motor unit\'s bodies along the axis and across: %s' % sorted(spans.items()))
    fc.ok(abs(spans['Brake to specification'][1] - 40.0 - FDET.SPEC['L']) < 0.01,
          'the unit ends %.1f mm from its flange\'s face, the specification\'s %.0f; the brake %.1f long'
          % (spans['Brake to specification'][1] - 40.0, FDET.SPEC['L'], spans['Brake to specification'][1] - spans['Brake to specification'][0]))
    # the shaft, its key, the pinion's hub, the screw and the washer
    sh = named('Gearhead shaft')[0]
    sb = {b.name: b.boundingBox for b in sh.component.bRepBodies}
    shaft = sb['Gearhead shaft']
    key = sb.get('Key DIN 6885-A 4x4x20')
    fc.ok(abs(shaft.minPoint.y * 10 - P['shaft'][0]) < 0.01 and abs(shaft.maxPoint.y * 10 - P['shaft'][1]) < 0.01
          and abs((shaft.maxPoint.x - shaft.minPoint.x) * 10 - 12.0) < 0.01,
          'the gearhead\'s shaft 12 mm across from %.2f to %.2f mm' % (shaft.minPoint.y * 10, shaft.maxPoint.y * 10))
    fc.ok(key is not None and abs((key.maxPoint.y - key.minPoint.y) * 10 - 20.0) < 0.01
          and abs(key.minPoint.y * 10 - P['key'][0]) < 0.01 and abs((key.maxPoint.x - key.minPoint.x) * 10 - 4.0) < 0.01,
          'the key 4 wide and 20 long, from %.1f mm' % (key.minPoint.y * 10 if key else 0))
    pn = named('Pinion z18')[0]
    pb = pn.component.bRepBodies.item(0).boundingBox
    hub = [round(f.geometry.radius * 20, 3) for f in pn.component.bRepBodies.item(0).faces
           if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - P['r_hub']) < 1e-3]
    fc.ok(abs(pb.minPoint.y * 10 - P['hub'][0]) < 0.01 and hub, 'the pinion\'s hub %s mm across from %.2f mm'
          % (sorted(set(hub)), pb.minPoint.y * 10))
    ws, sc = named('Pinion end washer'), named('Cap screw ISO 4762 M4x10')
    wb = ws[0].component.bRepBodies.item(0).boundingBox if ws else None
    scb = sc[0].component.bRepBodies.item(0).boundingBox if sc else None
    fc.ok(wb is not None and scb is not None and abs(wb.maxPoint.y * 10 - P['shaft'][0]) < 0.01
          and pb.minPoint.y * 10 - (P['shaft'][0] + 1.0) > 0.5 - 0.01
          and abs(scb.maxPoint.y * 10 - (P['washer'][0] + P['screw'][1])) < 0.02,
          "the end washer on the shaft's end at %.2f mm, the hub %.2f behind it (0.5 at maxon's shortest shaft), "
          'the M4 x 10 to %.2f, %.1f mm of it in the shaft'
          % (wb.maxPoint.y * 10 if wb else 0, pb.minPoint.y * 10 - (wb.maxPoint.y * 10 if wb else 0),
             scb.maxPoint.y * 10 if scb else 0,
             (scb.maxPoint.y * 10 - P['shaft'][0]) if scb else 0))
    fc.ok(len([o for o in leaves if o.component.name.startswith('Motor screw M5x12')]) == 4,
          'the unit\'s flange on the housing by its four M5 x 12, as Chapter 33 drew it')
    # the zones and the faces
    zs = last['zones']
    lens = {k: [round((f.boundingBox.maxPoint.x - f.boundingBox.minPoint.x) * 10, 3) for f in v] for k, v in zs.items()}
    fc.ok(sorted(zs) == ['Hinge pin L', 'Hinge pin R', 'Horn pin'] and all(v == [16.0] for v in lens.values())
          and all(attr(f, 'treat') == ns['HARD'] for v in zs.values() for f in v),
          'three zones hardened, each 16 mm long: %s' % lens)
    seats = {k: [attr(f) for f in v] for k, v in zs.items()}
    fc.ok(all(v == [ns['SEAT']] for v in seats.values()), 'their classes %s' % seats)
    pins = {}
    for o in leaves:
        if o.component.name in zs:
            pins[o.component.name] = sorted(attr(f) for b in o.bRepBodies for f in b.faces
                                            if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - 9.99) < 1e-3
                                            and f not in zs[o.component.name])
    fc.ok(all(v == [ns['BESIDE'], ns['BESIDE']] for v in pins.values()),
          'beside each seat, in the cheeks or the spacers, %s' % pins)
    hand, new = last['hand'], last['new']
    vals = sorted({attr(f) for f in hand})
    fc.ok(vals == sorted({ns['SEAT'], ns['BESIDE'], ns['KEYED'], ns['KEYWAY']}) and len(hand) == 12 and len(new) == 3,
          'by hand: the three pins\' nine faces, the pinion\'s bore and its keyway\'s two sides, %d faces, %d of them '
          'new %s' % (len(hand), len(new), vals))
    tr = [attr(f) for o in leaves if o.component.name.startswith('Trunnion') for b in o.bRepBodies for f in b.faces if attr(f) and 'm6' in attr(f)]
    fc.ok(len(tr) == 2, 'the trunnions\' seats in the spar fitting\'s bearings %s' % tr)
    carried = sum(1 for o in leaves for b in o.bRepBodies for f in b.faces if attr(f))
    fc.ok(carried == sum(last['mf']['coloured'].values()) + len(new),
          'every other decided face as Chapter 51 decides it: %d faces carry a class, %d coloured by the rules and %d '
          'more by hand' % (carried, sum(last['mf']['coloured'].values()), len(new)))
    # no interference over the travel
    near = [x for x in leaves if not x.fullPathName.startswith('Actuator') or any(x.component.name.startswith(k) for k in OUTER)]
    hits = []
    for d in (0.0, 10.0, 20.0, 30.0, 40.0):
        ns['pose'](d)
        adsk.doEvents()
        h = fc.interference(near)
        if h:
            hits.append((d, h))
    ns['pose'](0.0)
    fc.ok(not hits, 'no interference among the %d parts at 0, 10, 20, 30 and 40 deg %s' % (len(near), hits or ''))
    act = [x for x in leaves if x.fullPathName.startswith('Actuator')]
    h = fc.interference(act)
    fc.ok(not h, 'none inside the actuator\'s %d parts, flap up %s' % (len(act), h or ''))
    fc.refused(ns, 'falls short', i_gh=21.0)
    fc.verdict()
