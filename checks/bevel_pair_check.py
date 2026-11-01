# BevelPair (Chapter 16) against the acceptance criteria of its prompt, measured on what Fusion built: the numbers
# recomputed independently; the gear's turn about +X measured on its body when the pinion is driven about +Z; no tooth
# collision while the pinion turns through one tooth pitch.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/BevelPair/BevelPair.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    m, z1, z2, al, b = 2.0, 20, 40, math.radians(20), 12.0
    d1 = math.atan(z1 / z2)
    d2 = math.radians(90) - d1
    R = m * z2 / 2 / math.sin(d2)
    rv1, rv2 = m * z1 / 2 / math.cos(d1), m * z2 / 2 / math.cos(d2)
    eps = (math.sqrt((rv1 + m) ** 2 - (rv1 * math.cos(al)) ** 2) + math.sqrt((rv2 + m) ** 2 - (rv2 * math.cos(al)) ** 2)
           - (rv1 + rv2) * math.sin(al)) / (math.pi * m * math.cos(al))
    want = dict(d1=d1, d2=d2, R=R, k=(R - b) / R, zv1=z1 / math.cos(d1), zv2=z2 / math.cos(d2), eps=eps)
    p = ns['pair'](m, z1, z2, 20.0, b)
    bad = [x for x in want if abs(p[x] - want[x]) > 1e-9]
    fc.ok(not bad, 'all values recomputed independently %s' % (bad or ''))
    got = (round(math.degrees(p['d1']), 3), round(math.degrees(p['d2']), 3), round(p['R'], 3), round(p['zv1'], 2),
           round(p['zv2'], 2), round(p['eps'], 3))
    fc.ok(got == (26.565, 63.435, 44.721, 22.36, 89.44, 1.712), 'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Gear', 'Pinion', 'Shaft 1', 'Shaft 2'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 2 and root.motionLinks.count == 1,
          'two revolute As-built Joints, one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    jp = fc.joint('Pinion turns')
    Rp, Rg = fc.rotation(fc.occ('Pinion')), fc.rotation(fc.occ('Gear'))
    for v in (math.radians(90), -math.radians(90)):
        jp.jointMotion.rotationValue = v
        adsk.doEvents()
        tp = fc.turn_deg(fc.rotation(fc.occ('Pinion')), Rp, (0, 0, 1))
        tg = fc.turn_deg(fc.rotation(fc.occ('Gear')), Rg, (1, 0, 0))
        if abs(abs(tp) - 90) < 1e-3:
            break
    fc.ok(abs(tg + tp * z1 / z2) < 1e-3 and abs(abs(tp) - 90) < 1e-3,
          'the pinion turned %.3f deg about +Z: the gear turned %.3f deg about +X (want %.3f)' % (tp, tg, -tp * z1 / z2))
    worst = []
    for i in range(13):
        jp.jointMotion.rotationValue = 0
        adsk.doEvents()
        jp.jointMotion.rotationValue = math.radians(i * 360.0 / z1 / 12)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Pinion'), fc.occ('Gear')])
        if hit:
            worst.append((round(i * 360.0 / z1 / 12, 2), hit))
    fc.ok(not worst, 'no tooth collision while the pinion turns through 18 deg in 13 positions %s' % (worst or ''))
    jp.jointMotion.rotationValue = 0
    adsk.doEvents()
    fc.refused(ns, 'more than R / 3 = 14.91', b=16.0)
    fc.refused(ns, 'z_v1 = z1 / cos(delta1) = 16.02', z1=15)
    fc.verdict()
