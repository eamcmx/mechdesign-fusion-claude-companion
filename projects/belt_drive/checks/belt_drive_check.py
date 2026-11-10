# BeltDrive (Chapter 20) against the acceptance criteria of its prompt, measured on what Fusion built: the drive's
# numbers recomputed independently (the centre distance by Newton's method, not the script's bisection); the belt's
# volume against width x thickness x the length of its centre line; pulley 1 stepped round and pulley 2 measured.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/belt_drive/BeltDrive/BeltDrive.py'


def length(C, a, b):
    phi = math.asin((b - a) / C)
    return 2 * C * math.cos(phi) + math.pi * (a + b) + 2 * phi * (b - a)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    a, b = 20 * 2 / math.pi / 2, 40 * 2 / math.pi / 2
    C = 60.0
    for _ in range(50):                                            # Newton on L(C) = 180, the derivative numerically
        f = length(C, a, b) - 180.0
        d = (length(C + 1e-6, a, b) - length(C - 1e-6, a, b)) / 2e-6
        C -= f / d
    phi = math.asin((b - a) / C)
    k = ns['belt'](20, 40, 90)
    want = dict(pd1=2 * a, pd2=2 * b, C=C, wrap1=math.pi - 2 * phi, wrap2=math.pi + 2 * phi,
                mesh1=20 * (math.pi - 2 * phi) / (2 * math.pi), ratio=2.0)
    bad = [x for x in want if abs(k[x] - want[x]) > 1e-6]
    fc.ok(not bad, 'PD %.4f and %.4f, C %.4f, wraps %.3f and %.3f deg, %.2f teeth in mesh %s'
          % (k['pd1'], k['pd2'], k['C'], math.degrees(k['wrap1']), math.degrees(k['wrap2']), k['mesh1'], bad or ''))
    fc.ok((round(k['C'], 3), round(math.degrees(k['wrap1']), 3), round(k['mesh1'], 2)) == (59.66, 167.749, 9.32),
          'the rounded values of the prompt')
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Base plate', 'Belt', 'Pulley 1', 'Pulley 2', 'Shaft 1', 'Shaft 2'], 'six components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    x2 = fc.position_mm(fc.occ('Pulley 2'))[0]
    fc.ok(abs(x2 - C) < 1e-6, 'pulley 2 at x = C = %.4f mm (measured %.4f)' % (C, x2))
    t = 0.63
    rm1, rm2 = a - 0.254 + t / 2, b - 0.254 + t / 2
    v_want = 6.0 * t * length(C, rm1, rm2)
    v = fc.volume_mm3(fc.occ('Belt'))
    fc.ok(abs(v - v_want) < 1e-3, 'the belt %.3f mm^3: width x thickness x centre line = %.3f mm^3' % (v, v_want))
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 5 and root.motionLinks.count == 1,
          'five As-built Joints (three rigid, two revolute), one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    j1 = fc.joint('Pulley 1 turns')
    R2 = fc.rotation(fc.occ('Pulley 2'))
    worst, hits = 0.0, []
    for i in range(1, 13):
        j1.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        t2 = fc.turn_deg(fc.rotation(fc.occ('Pulley 2')), R2, (0, 0, 1))
        worst = max(worst, abs(((t2 - 15.0 * i + 180) % 360) - 180))
        hit = fc.interference()
        if hit:
            hits.append((30 * i, hit))
    fc.ok(worst < 1e-3, 'pulley 2 at +15 deg per 30 deg step of pulley 1, the same way (worst %.5f deg)' % worst)
    fc.ok(not hits, 'no interference at any of the 12 steps %s' % (hits or ''))
    for i in range(11, -1, -1):
        j1.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
    fc.refused(ns, 'too short', n_b=40)
    fc.refused(ns, 'Only 3.96 teeth', z1=12, z2=60, n_b=70)
    fc.verdict()
