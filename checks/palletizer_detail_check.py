# PalletizerDetail (Chapter 55) against the acceptance criteria of its prompt, measured on what Fusion holds after each
# run: the bag gripper (by the dialog, as a user runs it), then the robot, the compliance unit and the vacuum gripper
# (by build()); each one's new geometry, its classes and designations, no interference at rest and moved; the refusal.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import palletizer_detail as XP
importlib.reload(XP)
import adsk.core

PATH = '__COMPANION__/scripts/PalletizerDetail/PalletizerDetail.py'


def radii(o):
    return sorted({round(f.geometry.radius * 10, 3) for b in o.bRepBodies for f in b.faces
                   if isinstance(f.geometry, adsk.core.Cylinder)})


def span(o, a):
    b = o.boundingBox
    return round(getattr(b.minPoint, a) * 10, 2), round(getattr(b.maxPoint, a) * 10, 2)


def attr(f, name='fit'):
    a = f.attributes.itemByName('mdfc', name)
    return a.value if a else None


def leaves():
    root = fc.design().rootComponent
    return [x for x in root.allOccurrences if x.component.bRepBodies.count]


def named(ls, start):
    return [o for o in ls if o.component.name.startswith(start)]


def material(o):
    return {b.material.name for b in o.bRepBodies}


def check_bag(ns):
    ls = leaves()
    E = XP.jaw_ends()
    ep = named(ls, 'End plate')
    rr = [radii(o) for o in ep]
    xs = [span(o, 'x') for o in ep]
    fc.ok(len(ep) == 2 and all({28.0, 31.0, 32.5}.issubset(r) for r in rr)
          and all(abs(max(abs(a), abs(b)) - E['boss'][1]) < 0.05 for a, b in xs),
          'the end plates\' bosses: the lip 56, the seat 62, the DIN 472 groove 65, out to %.0f mm: %s, %s'
          % (E['boss'][1], [r for r in rr[0] if 27 < r < 33], xs))
    c472, c471 = named(ls, 'Circlip DIN 472 62x2'), named(ls, 'Circlip DIN 471 30x1.5')
    fc.ok(len(c472) == 4 and len(c471) == 2, 'circlips: %d DIN 472 62 x 2 in the bosses, %d DIN 471 30 x 1.5 on the short '
          'stubs' % (len(c472), len(c471)))
    welds = named(ls, 'Fillet weld')
    wx = sorted(tuple(sorted(abs(v) for v in span(o, 'x'))) for o in welds)
    fc.ok(len(welds) == 4 and all(abs(a - 401.0) < 0.05 and abs(b - 401.0 - ns['_last']['weld']) < 0.05 for a, b in wx),
          'the plugs\' fillet welds, 4, from 401 to %.0f mm, the plugs set 4 mm into the tube: %s'
          % (401.0 + ns['_last']['weld'], wx[0]))
    stubs = named(ls, 'Stub shaft')
    short = [o for o in stubs if 'short' in o.component.name]
    sx = sorted(tuple(sorted(abs(v) for v in span(o, 'x'))) for o in short)
    fc.ok(len(sx) == 2 and all(abs(a - 393.0) < 0.05 and abs(b - E['short']['end']) < 0.05 for a, b in sx)
          and all(18.0 in radii(o) and 14.3 in radii(o) for o in short),
          'the short stubs from the plug at 393 to %.1f, the shoulder 36 and the DIN 471 groove 28.6: %s %s'
          % (E['short']['end'], sx, sorted({o.component.name for o in stubs})))
    jaw = [o for o in ls if o.component.name.startswith(('Tube', 'Stub shaft'))] + welds
    fc.ok(len(jaw) == 10 and all(material(o) == {'Steel AISI 1020 107 HR'} for o in jaw),
          'the jaws\' tubes, stubs and welds in 1020 (%d parts)' % len(jaw))
    seats = {}
    for o in ep:
        for b in o.bRepBodies:
            for f in b.faces:
                if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - 31.0) < 1e-3 and attr(f):
                    seats[attr(f)] = seats.get(attr(f), 0) + 1
    fc.ok(seats.get('J7 Ra 3.2', 0) == 2 and seats.get('H7 Ra 3.2', 0) == 2,
          'the bosses\' seats: J7 at the two locating ends, H7 at the two floating ends: %s' % seats)
    ground = sum(1 for o in stubs for b in o.bRepBodies for f in b.faces if attr(f, 'process'))
    d6206 = {o.component.description for o in named(ls, 'Bearing 6206')}
    fc.ok(ground >= 4 and d6206 == {'SKF 6206-2RS1/C4'},
          'the stubs\' seats marked ground after welding (%d faces); the bearings %s' % (ground, sorted(d6206)))
    c = fc.interference(ls)
    sp = ns['_last']['mod']._last
    g, G_ = sp['g'], sp['groups']
    g.set_state(G_, phi=g.L['open'])
    adsk.doEvents()
    c2 = fc.interference(leaves())
    g.set_state(G_, phi=0.0)
    adsk.doEvents()
    fc.ok(not c and not c2, 'the bag gripper: no interference among %d parts, closed nor open %s %s' % (len(ls), c or '', c2 or ''))


def check_robot(ns):
    ls = leaves()
    ok_eyes, eye_seats = [], {}
    for e in XP.eyes():
        o = named(ls, e['name'] + ' eye')
        b = XP.SRB[e['bearing']]
        r = radii(o[0]) if o else []
        ok_eyes.append(bool(o) and {e['lip_d'] / 2, b['D'] / 2, e['groove_d'] / 2}.issubset(r))
        for x in o:
            for bb in x.bRepBodies:
                for f in bb.faces:
                    if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - b['D'] / 2) < 1e-3:
                        eye_seats[attr(f)] = eye_seats.get(attr(f), 0) + 1
    fc.ok(all(ok_eyes), 'the six eyes: the shoulders 64 and 110, the seats 72 and 120, the DIN 472 grooves 75 and 124')
    pins = {e['pin']: e for e in XP.eyes()}
    okp = []
    for p, e in pins.items():
        o = named(ls, 'Pin %s' % p)
        r = radii(o[0]) if o else []
        y0, y1 = span(o[0], 'y') if o else (0, 0)
        okp.append(bool(o) and {e['collar_d'] / 2, XP.SRB[e['bearing']]['d'] / 2}.issubset(r))
    fc.ok(all(okp), 'the six pins: the collars 43 and 76 against the inner rings, the seats 35 and 65')
    cl, ew, sc = named(ls, 'Circlip DIN 472'), named(ls, 'End washer pin'), named(ls, 'Cap screw M')
    fc.ok(len(cl) == 6 and len(ew) == 6 and len([o for o in sc if 'pin' in o.component.name]) == 6,
          'six DIN 472 circlips in the eyes, six end washers and their screws on the pins')
    fc.ok(eye_seats.get('H7 Ra 3.2', 0) >= 6 and not eye_seats.get('K7 Ra 3.2'),
          'the eyes\' seats H7, none K7: %s' % eye_seats)
    tri = []
    for o in named(ls, 'Triangle plate'):
        for b in o.bRepBodies:
            for f in b.faces:
                if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - 36.0) < 1e-3 and attr(f):
                    tri.append(attr(f))
    desc = {o.component.name: o.component.description for o in named(ls, 'Bearing ')
            if any(k in o.component.name for k in ('22213', '22207', '61836', '61856'))}
    want = {'22213 E Parallel': 'SKF 22213 E/C3', '22207 E Levelling': 'SKF 22207 E', '22207 E triangle': 'SKF 22207 E/C3',
            '61836': 'SKF 61836/C4', '61856 MA': 'SKF 61856 MA/C4'}
    good = all(v == next(w for k, w in want.items() if k in n) for n, v in desc.items())
    fc.ok(tri and set(tri) == {'K7 Ra 3.2'} and good and len(desc) == 9,
          'the triangle\'s seat K7 %s; the clearance classes in the designations: %s'
          % (sorted(set(tri)), sorted(set(desc.values()))))
    c = fc.interference(ls)
    RB = ns['_last']['mod']
    import palletizer_design as P
    ws = P.workspace()
    RB.set_pose(RB._last['groups'], 0.0, ws['reach_q2'], ws['reach_q3'])
    adsk.doEvents()
    c2 = fc.interference(leaves())
    fc.ok(not c and not c2, 'the robot: no interference among %d parts at the pick pose nor at full reach %s %s'
          % (len(ls), c or '', c2 or ''))


def check_unit(ns):
    ls = leaves()
    G, P = XP.gland(), XP.gallery_plug()
    car = named(ls, 'Carriage')[0]
    rc = radii(car)
    gw = [round((f.boundingBox.maxPoint.z - f.boundingBox.minPoint.z) * 10, 3) for b in car.bRepBodies for f in b.faces
          if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - G['groove_d'] / 2) < 1e-3]
    fc.ok(G['groove_d'] / 2 in rc and 12.5 not in rc and gw == [G['width']],
          'the O-ring\'s groove %.2f across and %.2f wide (ERIKS: gland depth %.2f), none 25: %s'
          % (G['groove_d'], G['width'], XP.GLAND['E'], gw))
    orr = named(ls, 'O-ring')[0]
    fc.ok(radii(orr) == [10.0, G['groove_d'] / 2], 'the O-ring squeezed from 2.5 to %.2f, its section kept: %s, %.2f long'
          % (XP.GLAND['E'], radii(orr), span(orr, 'z')[1] - span(orr, 'z')[0]))
    plugs = named(ls, 'Plug ')
    fc.ok(len(plugs) == 6 and all(radii(o) == [P['d'] / 2, P['dk'] / 2] for o in plugs)
          and all(P['cb_d'] / 2 in radii(x) for x in ls if x.component.name in ('Housing', 'Carriage')),
          'six DIN 908 M10 x 1 plugs, their collars in counterbores %.0f across' % P['cb_d'])
    tube = named(ls, 'Tube 20x16')[0]
    zones = sorted(attr(f) for b in tube.bRepBodies for f in b.faces
                   if isinstance(f.geometry, adsk.core.Cylinder) and abs(f.geometry.radius * 10 - 10.0) < 1e-3)
    fc.ok(zones == ['g6 Ra 0.4', 's6 Ra 0.8'], 'the tube\'s outside in two zones: %s' % zones)
    desc = {o.component.description for o in ls if o.component.name.startswith(('Block HGH25CA', 'Rail HGR25'))}
    fc.ok(desc == {'HIWIN HGH25CAZ0C', 'HIWIN HGR25R218C'}, 'the guideways %s' % sorted(desc))
    c = fc.interference(ls)
    CU = ns['_last']['mod']
    CU.set_state(CU._last['groups'], lift=CU.L['stroke'])
    adsk.doEvents()
    c2 = fc.interference([o for o in leaves() if not o.component.name.startswith('Spring')])
    CU.set_state(CU._last['groups'], lift=0.0)
    fc.ok(not c and not c2, 'the unit: no interference among %d parts at rest, nor risen %.0f mm but for the springs, '
          'drawn at their installed length %s %s' % (len(ls), CU.L['stroke'], c or '', c2 or ''))


def check_vacuum(ns):
    ls = leaves()
    J = XP.vacuum_joint()
    sc, wa, nu = named(ls, 'Cap screw M6x%d' % J['L']), named(ls, 'Washer ISO 7089 6'), named(ls, 'Nut ISO 4032 M6')
    pl = named(ls, 'Plate')[0]
    fc.ok(len(sc) == 16 and len(wa) == 32 and len(nu) == 16 and 3.3 in radii(pl) and 3.0 not in radii(pl)
          and not named(ls, 'Cap screw M6x14'),
          'the plate on 16 through bolts M6 x %d, 32 washers and 16 nuts, its holes 6.6 through, none tapped'
          % J['L'])
    sl = named(ls, 'Sleeve ')
    m4 = named(ls, 'Cap screw M4x12')
    fc.ok(len(sl) == 36 and all(24.0 in radii(o) for o in sl) and len(m4) == 108 and 2.0 in radii(pl),
          'the 36 sleeves on their flanges 48 across, held by 108 M4 x 12 tapped into the plate')
    c = fc.interference(ls)
    fc.ok(not c, 'the vacuum gripper: no interference among %d parts %s' % (len(ls), c or ''))


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    check_bag(ns)
    for k, f in (('robot', check_robot), ('unit', check_unit), ('vacuum', check_vacuum)):
        print('  ' + ns['build'](k).split('\n')[0])
        fc.mark()
        f(ns)
    fc.refused(ns, 'under', key='bag', weld=3.0)
    fc.verdict()
