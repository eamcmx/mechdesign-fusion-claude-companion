# WristDetail (Chapter 52) against the acceptance criteria of its prompt, measured on what Fusion holds after the run:
# the gear motors' length and section and their output shafts; the slip ring's stator, flange and screw, the pin gone;
# the collar and spacers S4 and S5 inside SKF's band; the key in the stop disc; the front side gears' bores M7; no
# interference at rest and at nine poses; the refusal.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import wrist_detail as WDET
importlib.reload(WDET)
import adsk.core

PATH = '__COMPANION__/scripts/WristDetail/WristDetail.py'


def ybox(o):
    bb = None
    for b in o.bRepBodies:
        bx = b.boundingBox
        bb = [bx.minPoint.x, bx.maxPoint.x, bx.minPoint.y, bx.maxPoint.y, bx.minPoint.z, bx.maxPoint.z] if bb is None else \
            [min(bb[0], bx.minPoint.x), max(bb[1], bx.maxPoint.x), min(bb[2], bx.minPoint.y), max(bb[3], bx.maxPoint.y),
             min(bb[4], bx.minPoint.z), max(bb[5], bx.maxPoint.z)]
    return [round(v * 10, 3) for v in bb]


def radii(o):
    return sorted({round(f.geometry.radius * 20, 3) for b in o.bRepBodies for f in b.faces
                   if isinstance(f.geometry, adsk.core.Cylinder)})


def attr(f):
    a = f.attributes.itemByName('mdfc', 'fit')
    return a.value if a else None


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    named = lambda nm: [o for o in leaves if o.component.name.split(' (')[0] == nm]
    G, Mo, S = WDET.GEARHEAD, WDET.MOTOR, WDET.SLIPRING
    last = ns['_last']
    L = last['L']
    # the gear motors
    gm = named('Servo motor with brake and 15:1 gearhead')
    boxes = [ybox(o) for o in gm]
    lens = [round(b[3] - b[2], 3) for b in boxes]
    want = G['body'] + G['adapter'] + Mo['k']
    fc.ok(len(gm) == 2 and all(abs(x - want - 2.0) < 0.01 for x in lens),
          'the two gear motors %s mm along y with the spigot\'s 2 mm, the unit and the motor %.1f; ends at %s'
          % (lens, want, [b[2] if abs(b[2]) > abs(b[3]) else b[3] for b in boxes]))
    fc.ok(all(abs((b[1] - b[0]) - Mo['flange']) < 0.01 and abs((b[5] - b[4]) - Mo['flange'] - 6.0) < 0.01 for b in boxes),
          'their section %s mm, 40 square with the connector\'s 6' % [(round(b[1] - b[0], 2), round(b[5] - b[4], 2)) for b in boxes])
    outs = named('Motor output')
    sh = [r for o in outs for r in radii(o) if abs(r - G['shaft']) < 0.01]
    fc.ok(len(outs) == 2 and len(sh) >= 1, 'the output shafts %s mm across, the PLE040\'s %.0f' % (sorted(set(sh)), G['shaft']))
    # the slip ring
    st = named('Slip ring stator')
    sb = ybox(st[0]) if st else None
    rr = radii(st[0]) if st else []
    pins = [b for o in st for b in o.bRepBodies if b.name == 'pin']
    fc.ok(st and S['D'] in rr and S['flange'] in rr and abs((sb[5] - sb[4]) - S['L']) < 0.01 and not pins,
          'the slip ring\'s stator %s across, %.2f long, its flange among them; pins left %d'
          % (rr, sb[5] - sb[4] if sb else 0, len(pins)))
    fc.ok(len(named('Cap screw ISO 4762 M3x10')) == 1 and not named('Cap screw ISO 4762 M3x8'),
          'the bracket\'s screw M3 x 10, the M3 x 8 gone')
    # the abutments
    ab = WDET.DETAIL_ABUT
    sk = WDET.skf_page()['6002-2RSH']
    rs = named('Roll shaft, bored')[0]
    collar = [r for r in radii(rs) if r > 15.0]
    s4, s5 = named('Spacer S4'), named('Spacer S5')
    od = lambda os_: [max(radii(o)) for o in os_]
    fc.ok(collar == [ab] and od(s4) == [ab] and od(s5) == [ab] and sk['da min'] <= ab <= sk['da max'],
          'the collar %s, S4 %s, S5 %s mm across, inside the 6002\'s %.1f to %.1f' % (collar, od(s4), od(s5), sk['da min'], sk['da max']))
    # the key and the stop disc
    key = named('Key DIN 6885 A 5x5x10')
    disc = named('Stop disc')
    kb, db = ybox(key[0]), ybox(disc[0])
    fc.ok(len(key) == 1 and not named('Key DIN 6885 A 5x5x8') and abs((kb[3] - kb[2]) - 10.0) < 0.01
          and abs(kb[2] - db[2] - 1.0) < 0.01 and abs(kb[3] - db[3]) < 0.01,
          'the key from %.1f to %.1f mm, in the stop disc from %.1f to %.1f' % (kb[2], kb[3], db[2], db[3]))
    # the front side gears' bores and the other decided faces
    fr = last['front']
    fc.ok(len(fr) == 2 and all(attr(f) == ns['FRONT_CLS'] + ' Ra 3.2' for f in fr),
          'the front side gears\' bores %s' % [attr(f) for f in fr])
    others = [attr(f) for o in named('Side bevel gear with 36-groove pulley') for b in o.bRepBodies for f in b.faces
              if attr(f) and f not in fr and attr(f).startswith('M7')]
    carried = sum(1 for o in leaves for b in o.bRepBodies for f in b.faces if attr(f))
    fc.ok(not others and carried == sum(last['mf']['coloured'].values()),
          'every other decided face as Chapter 51 decides it: %d faces carry a class, %d coloured' % (carried, sum(last['mf']['coloured'].values())))
    # no interference at rest nor over the range
    pose = ns['pose']
    pose(0.0, 0.0)
    h = fc.interference(leaves)
    fc.ok(not h, 'no interference among the %d parts at rest %s' % (len(leaves), h or ''))
    bad = []
    for p_ in (-120.0, 0.0, 120.0):
        for r_ in (0.0, 90.0, 180.0):
            pose(p_, r_)
            h = fc.interference(leaves)
            if h:
                bad.append((p_, r_, h[:2]))
    pose(0.0, 0.0)
    fc.ok(not bad, 'no interference at pitch -120, 0, 120 and roll 0, 90, 180 deg %s' % (bad or ''))
    fc.refused(ns, 'falls short', contact=123.0)
    fc.verdict()
