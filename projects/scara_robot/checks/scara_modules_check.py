# ScaraJointModules (Chapter 36) against the acceptance criteria of its prompt, measured on what Fusion built: the
# J1 unit's average torque and life worked again here from the cycle; the parts counted from Fusion's components; no
# interference at rest and with each output turned; each output turning one fiftieth of its input, the other way;
# the units' outline dimensions as the catalogue gives them; the modules' masses; a refusal.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/scara_robot/ScaraJointModules/ScaraJointModules.py'
EXPECTED = {'CSG-20-50-2UH housing': 1, 'CSG-40-50-2UH housing': 1, 'Case bolt M5': 8, 'Case bolt M8': 10, 'Cross rollers': 2, 'Flexspline': 2, 'Module plate J1': 1, 'Module plate J2': 1, 'Motor bolt M5': 4, 'Motor bolt M8': 4, 'Motor shaft J1': 1, 'Motor shaft J2': 1, 'Output flange': 2, 'Servo motor J1': 1, 'Servo motor J2': 1, 'Wave generator': 2}


def own_numbers():
    """The J1 load pattern again: the horizontal leg there and back, each a cycloid of 0.7175 rad in 0.20 s, 0.05 s at
    rest after each; the torque M11 times the acceleration; the catalogue's cube mean weighted by speed and time."""
    L1, L2 = 0.35, 0.25
    c2a = (400 ** 2 + 150 ** 2 - 350 ** 2 - 250 ** 2) / (2 * 350 * 250)
    q2 = math.acos(c2a)
    q1a = math.atan2(-150, 400) - math.atan2(250 * math.sin(q2), 350 + 250 * math.cos(q2))
    q1b = math.atan2(150, 400) - math.atan2(250 * math.sin(q2), 350 + 250 * math.cos(q2))
    d = q1b - q1a
    I1 = 3.5 * L1 ** 2 / 12 + 3.5 * 0.175 ** 2 + 3.0 * L1 ** 2
    c2 = (2.0 * 0.125 + 2.0 * 0.080) / 4.0
    I2c = 2.0 * L2 ** 2 / 12 + 2.0 * (0.125 - c2) ** 2 + 2.0 * (0.080 - c2) ** 2
    dd = I2c + 4.0 * c2 ** 2 + 4.0 * L2 ** 2
    bb = (4.0 * c2 + 4.0 * L2) * L1
    M11 = I1 + 8.0 * L1 ** 2 + dd + 2 * bb * math.cos(q2)
    num = den = tt = 0.0
    n = 400
    for k in range(2 * n):
        t = 0.2 * ((k % n) + 0.5) / n
        w = d / 0.2 * (1 - math.cos(2 * math.pi * t / 0.2))
        a = 2 * math.pi * d / 0.04 * math.sin(2 * math.pi * t / 0.2)
        rpm = abs(w) * 60 / (2 * math.pi)
        num += (0.2 / n) * rpm * abs(M11 * a) ** 3
        den += (0.2 / n) * rpm
        tt += 0.2 / n
    tt += 2 * 0.05
    Tav = (num / den) ** (1 / 3)
    nav = den / tt
    Lh = 10000 * (178 / Tav) ** 3 * (2000 / (nav * 50))
    return dict(Tav=Tav, Lh=Lh)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    own = own_numbers()
    o = ns['design']()
    s = o['sel1']
    ok = abs(s['Tav'] - own['Tav']) < 0.01 * own['Tav'] and abs(s['Lh'] - own['Lh']) < 0.03 * own['Lh']
    fc.ok(ok, 'J1: the average torque %.2f N m and the life %.0f h worked again (the module: %.2f, %.0f)'
          % (own['Tav'], own['Lh'], s['Tav'], s['Lh']))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(subs == ['J1 module', 'J2 module'], 'two modules: %s' % subs)
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds: %s' % (sum(pl.values()), len(pl), sorted(pl.items())))
    if EXPECTED is not None:
        fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    c = fc.interference(leaves)
    fc.ok(not c, 'no interference among the %d parts at rest %s' % (len(leaves), c or ''))
    J = ns['_last']['joints']
    hits = []
    for joint in (1, 2):
        jo, so, ji, si = J[joint]
        for deg in (7.2, 30.0):
            jo.jointMotion.rotationValue = math.radians(deg)
            adsk.doEvents()
            turns_in = ji.jointMotion.rotationValue / math.radians(deg)
            fc.ok(abs(abs(turns_in) - 50) < 1e-6 and turns_in * so * si < 0,
                  'J%d: the output turned %.1f deg, the input %.1f deg: %.4f to one, the other way' % (joint, deg, math.degrees(ji.jointMotion.rotationValue), turns_in))
            h = fc.interference(leaves)
            if h:
                hits.append((joint, deg, h))
        jo.jointMotion.rotationValue = 0.0
        adsk.doEvents()
    fc.ok(not hits, 'no interference with each output turned 7.2 and 30 deg %s' % (hits or ''))
    by = {x.component.name: x for x in leaves}
    for size in (o['size1'], 20):
        A = 2 * max(f.geometry.radius * 10 for body in by['CSG-%d-50-2UH housing' % size].bRepBodies for f in body.faces
                    if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType)
        B = max(f.boundingBox.maxPoint.z * 10 for body in by['Output flange %d' % size].bRepBodies for f in body.faces
                if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType)
        u = ns['sd'].CSG[size]
        fc.ok(abs(A - u['A']) < 0.01 and abs(B - u['B']) < 0.01,
              'CSG-%d: housing %.2f mm across, %.2f mm from the input face to the output flange (A %s, B %s)' % (size, A, B, u['A'], u['B']))
    for joint in (1, 2):
        mass = sum(x.physicalProperties.mass for x in leaves if x.fullPathName.startswith('J%d module' % joint)
                   and not x.component.name.startswith('Servo motor'))
        print('  J%d module: %.2f kg without its motor, %.2f kg with the motor at its specified %.1f kg'
              % (joint, mass, mass + ns['sd'].MOTOR[joint]['mass'], ns['sd'].MOTOR[joint]['mass']))
    for size in (o['size1'], 20):
        drawn = sum(x.physicalProperties.mass for x in leaves if x.component.name.endswith(' %d' % size)
                    or x.component.name.startswith('CSG-%d' % size))
        print('  CSG-%d unit as drawn: %.2f kg; the catalogue: %.2f kg' % (size, drawn, ns['sd'].CSG[size]['mass']))
    fc.refused(ns, 'falls short', size1=32.0)
    fc.verdict()
