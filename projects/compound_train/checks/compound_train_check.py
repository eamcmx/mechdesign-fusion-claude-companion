# CompoundTrain (Chapter 13) against the acceptance criteria of its prompt, measured on what Fusion built: the train's
# numbers recomputed independently, the turns of the cluster and the output read from the joints AND the bodies, and
# no tooth collision while the input turns through one tooth pitch.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/compound_train/CompoundTrain/CompoundTrain.py'


def eps(m, za, zb):
    al = math.radians(20)
    ra, rb = m * za / 2, m * zb / 2
    return (math.sqrt((ra + m) ** 2 - (ra * math.cos(al)) ** 2) + math.sqrt((rb + m) ** 2 - (rb * math.cos(al)) ** 2)
            - (ra + rb) * math.sin(al)) / (math.pi * m * math.cos(al))


def check_train(z1, z2, z3, z4, a):
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == sorted(['Base plate', 'Spindle', 'Countershaft', 'Gear 1', 'Cluster', 'Spacer', 'Gear 4']),
          'seven components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    x = [fc.position_mm(fc.occ(n))[0] for n in ('Cluster', 'Countershaft')]
    fc.ok(all(abs(v - a) < 1e-6 for v in x), 'cluster and countershaft at x = a = %.2f mm (%.3f, %.3f)' % (a, x[0], x[1]))
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 3 + ['RigidJointMotion'] * 3,
          'As-built Joints only, three revolute and three rigid')
    fc.ok(root.motionLinks.count == 2, 'two Motion Links: %s' % [l.name for l in root.motionLinks])
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    j1, j2, j4 = fc.joint('Gear 1 turns'), fc.joint('Cluster turns'), fc.joint('Gear 4 turns')
    R2, R4 = fc.rotation(fc.occ('Cluster')), fc.rotation(fc.occ('Gear 4'))
    j1.jointMotion.rotationValue = math.radians(90)
    adsk.doEvents()
    w2, w4 = -90.0 * z1 / z2, 90.0 * z1 * z3 / (z2 * z4)
    v2, v4 = math.degrees(j2.jointMotion.rotationValue), math.degrees(j4.jointMotion.rotationValue)
    b2 = fc.turn_deg(fc.rotation(fc.occ('Cluster')), R2, (0, 0, 1))
    b4 = fc.turn_deg(fc.rotation(fc.occ('Gear 4')), R4, (0, 0, 1))
    fc.ok(max(abs(v2 - w2), abs(b2 - w2), abs(v4 - w4), abs(b4 - w4)) < 1e-3,
          'gear 1 driven 90 deg: cluster joint %.3f, body %.3f; gear 4 joint %.3f, body %.3f (want %.3f and %.3f)'
          % (v2, b2, v4, b4, w2, w4))
    worst = []
    for k in range(13):
        ang = k * 360.0 / z1 / 12
        j1.jointMotion.rotationValue = math.radians(ang)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Gear 1'), fc.occ('Cluster'), fc.occ('Gear 4')])
        if hit:
            worst.append((round(ang, 2), hit))
    fc.ok(not worst, 'no tooth collision while gear 1 turns 0 to %.1f deg in 13 positions %s' % (360.0 / z1, worst or ''))
    j1.jointMotion.rotationValue = 0
    adsk.doEvents()


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    t = ns['train'](1.5, 2.0, 20, 60, 20)
    want = dict(a=60.0, z4=40.0, ratio=6.0, eps1=eps(1.5, 20, 60), eps2=eps(2.0, 20, 40))
    bad = [k for k in want if abs(t[k] - want[k]) > 1e-9]
    fc.ok(not bad, 'a %.2f, z4 %.0f, ratio %.3f, contact ratios %.3f and %.3f %s'
          % (t['a'], t['z4'], t['ratio'], t['eps1'], t['eps2'], bad or ''))
    fc.ok((round(t['eps1'], 3), round(t['eps2'], 3)) == (1.671, 1.635), 'the rounded contact ratios of the prompt')
    fc.run_dialog(ns)
    check_train(20, 60, 20, 40, 60.0)
    msg = ns['build'](z3=24)
    fc.mark()
    fc.ok('24 : 36 teeth' in msg and '= 4.500' in msg, 'z3 = 24 gives z4 = 36 and a ratio of 4.5 in the message')
    check_train(20, 60, 24, 36, 60.0)
    fc.refused(ns, '= 68.57 teeth', m2=1.75)
    fc.refused(ns, 'undercut', z3=12)
    fc.verdict()
