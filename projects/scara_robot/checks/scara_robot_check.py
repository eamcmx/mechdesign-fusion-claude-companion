# ScaraRobot (Chapter 38) against the acceptance criteria of its prompt, measured on what Fusion built: the parts
# counted; the joints and the rigid holds; no interference where the links meet, at rest, on the standard cycle's
# ends, with the arm folded both ways and the tool at both ends of its stroke; the robot's mass properties
# measured part by part and summed for the equations of motion (a, b, d of Chapter 35); the tool's height.
import math, sys, importlib, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/projects/scara_robot/ScaraRobot/ScaraRobot.py'
EXPECTED = {'Ball screw spline shaft SP20': 1, 'Base column': 1, 'Base foot': 1, 'Base top flange': 1,
            'Belt 3MR-474-09 J3': 1, 'Belt 3MR-489-09 J4': 1, 'CSG-20-50-2UH housing': 1, 'CSG-45-50-2UH housing': 1,
            'Case bolt M5': 8, 'Case bolt M8': 12, 'Cross rollers': 2, 'First arm beam': 1, 'Flange bolt M4': 8,
            'Flexspline': 2, 'J3 motor bolt M5': 4, 'Module plate J1': 1, 'Module plate J2': 1, 'Motor bolt M4': 4,
            'Motor bolt M5': 4, 'Motor bolt M8': 4, 'Motor shaft J1': 1, 'Motor shaft J2': 1, 'Motor shaft J3': 1,
            'Motor shaft J4': 1, 'Output flange': 2, 'Pulley 18T motor J4': 1, 'Pulley 28T motor J3': 1,
            'Pulley 56T nut J3': 1, 'Pulley 72T nut J4': 1, 'Pulley screw M5': 12, 'Screw nut SP20 flange': 1,
            'Screw nut SP20 rotor': 1, 'Second arm structure': 1, 'Servo motor J1': 1, 'Servo motor J2': 1,
            'Servo motor J3': 1, 'Servo motor J4': 1, 'Spline nut SP20 flange': 1, 'Spline nut SP20 rotor': 1,
            'Tool flange': 1, 'Wave generator': 2}
MEET = ('Base top flange', 'Module plate J1', 'CSG-4', 'Case bolt M8', 'Output flange 4', 'First arm beam',
        'Module plate J2', 'CSG-20-50-2UH housing', 'Case bolt M5', 'Output flange 20', 'Servo motor J2', 'Motor bolt M5',
        'Second arm structure', 'Spline nut SP20 flange', 'Spline nut SP20 rotor',
        'Pulley 72T nut J4', 'Pulley screw M5', 'Flange bolt M4', 'Belt 3MR-489-09 J4', 'Pulley 18T motor J4',
        'Motor shaft J4', 'Motor bolt M4', 'Ball screw spline shaft SP20', 'Tool flange', 'Servo motor J1')


def link_of(o):
    path = o.fullPathName
    top = path.split('+')[0].split(':')[0]
    if top == 'Base':
        return 1 if 'J1 output' in path else 0
    if top == 'First arm':
        return 2 if 'J2 output' in path else 1
    return 2


def mass_props(leaves):
    """Every part's mass, centre and own inertia about a vertical axis through its centre (kg, mm, kg m2), with the
    robot at rest (the arms along +X), summed into the first link's inertia about J1, the second link's mass, centre
    and inertia about J2."""
    rows = []
    for o in leaves:
        pp = o.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        m = pp.mass
        c = pp.centerOfMass
        ok, xx, yy, zz, xy, yz, xz = pp.getXYZMomentsOfInertia()
        x, y = c.x * 10, c.y * 10
        Izc = (zz - m * (c.x ** 2 + c.y ** 2)) * 1e-4
        rows.append((link_of(o), o.component.name, m, x, y, c.z * 10, Izc))
    out = {}
    for k in (0, 1, 2):
        rr = [r for r in rows if r[0] == k]
        m = sum(r[2] for r in rr)
        cx = sum(r[2] * r[3] for r in rr) / m
        cy = sum(r[2] * r[4] for r in rr) / m
        xo = 350.0 if k == 2 else 0.0
        I = sum(r[6] + r[2] * ((r[3] - xo) ** 2 + r[4] ** 2) * 1e-6 for r in rr)
        out[k] = dict(m=m, cx=cx - xo, cy=cy, I=I, n=len(rr))
    return out, rows


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = sorted(x.component.name for x in root.occurrences)
    fc.ok(subs == ['Base', 'First arm', 'Second arm'], 'three groups: %s' % subs)
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds' % (sum(pl.values()), len(pl)))
    print('  ' + json.dumps(sorted(pl.items())))
    if EXPECTED is not None:
        fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    sd = ns['sd']
    mp_, rows = mass_props(leaves)
    for k, nm in ((0, 'the base and J1\'s stator'), (1, 'the first link'), (2, 'the second link')):
        d = mp_[k]
        print('  %s: %d parts, %.3f kg, centre %.1f, %.1f mm from %s, inertia about it %.4f kg m2'
              % (nm, d['n'], d['m'], d['cx'], d['cy'], 'J2' if k == 2 else 'J1', d['I']))
    print('MASSPROPS ' + json.dumps({k: v for k, v in mp_.items()}))
    print('ROWS ' + json.dumps(rows))
    meet = [x for x in leaves if any(x.component.name.startswith(p) for p in MEET)]
    b = sd.brief()
    qa = sd.ik(b, b['x0'], -b['across'] / 2)
    qb = sd.ik(b, b['x0'], b['across'] / 2)
    poses = [(0.0, 0.0, 0.0, 0.0), (math.degrees(qa[0]), math.degrees(qa[1]), 0.0, 0.0),
             (math.degrees(qb[0]), math.degrees(qb[1]), 200.0, 180.0), (0.0, 150.0, 200.0, 90.0),
             (0.0, -150.0, 200.0, 0.0), (132.0, -150.0, 0.0, 0.0), (-132.0, 150.0, 100.0, 0.0)]
    for q1, q2, j3, j4 in poses:
        ns['pose'](q1, q2, j3, j4)
        h = fc.interference(meet)
        fc.ok(not h, 'no interference among the %d parts where the links meet at J1 %.1f, J2 %.1f deg, J3 %.0f mm, J4 %.0f deg %s'
              % (len(meet), q1, q2, j3, j4, h or ''))
    o = sd.integration([list(r) for r in rows])
    s45 = o['s45']
    fc.ok(s45['j1']['Lh'] >= 20000.0 and o['s40']['j1']['Lh'] < 20000.0,
          'on the measured arm, J1 lasts %.0f h on the CSG-45 and %.0f h on the CSG-40 (M11 %.4f kg m2)'
          % (s45['j1']['Lh'], o['s40']['j1']['Lh'], s45['M11']))
    fc.ok(s45['j2']['Tpk'] <= sd.CSG[20]['Trep'] and s45['j2']['Tav'] <= sd.CSG[20]['Tav'],
          'J2 on the CSG-20: peak %.2f N m against %d, average %.2f against %d, life %.0f h on radial moves alone'
          % (s45['j2']['Tpk'], sd.CSG[20]['Trep'], s45['j2']['Tav'], sd.CSG[20]['Tav'], s45['j2']['Lh']))
    ns['pose'](0.0, 0.0, 0.0, 0.0)
    by = {x.component.name: x for x in leaves}
    tf = by['Tool flange']
    z_tool = min(f.boundingBox.minPoint.z * 10 for body in tf.bRepBodies for f in body.faces
                 if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType)
    print('  the tool flange\'s underside %.2f mm above the floor at J3 0' % z_tool)
    fc.ok(abs(z_tool - (sd.heights(45)['tool_top0'] - 10.0)) < 0.01, 'the tool flange where heights() puts it')
    fc.refused(ns, 'CSG-40 or a CSG-45', size1=32.0)
    fc.verdict()
