# CarDifferential (Chapter 27) against the acceptance criteria of its prompt, measured on what Fusion built: the
# numbers recomputed independently; the pinion stepped round 30 deg at a time (Chapter 18: never one jump) with the
# case and both half shafts measured on their bodies; the left side gear turned in the case and the right one, the
# spiders and the half shafts measured; no collision at rest, at the pinion's steps and across one side-gear tooth.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/CarDifferential/CarDifferential.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    z1, z2, zs, zp, N = 17, 58, 22, 15, 2
    d1 = math.atan(z1 / z2)
    dp = math.atan(zp / zs)
    want = dict(ratio=z2 / z1, d1=d1, R_f=4.0 * z2 / 2 / math.cos(d1), zv1=z1 / math.cos(d1), dp=dp,
                R_d=5.0 * zs / 2 / math.cos(dp), zvp=zp / math.cos(dp), T_c=200 * 3.5 * z2 / z1)
    a = ns['axle'](4.0, z1, z2, 30.0, 5.0, zs, zp, N, 20.0, 20.0)
    bad = [k for k in want if abs(a[k] - want[k]) > 1e-9]
    fc.ok(not bad, 'all values recomputed independently %s' % (bad or ''))
    got = (round(a['ratio'], 4), round(math.degrees(a['d1']), 3), round(a['R_f'], 3), round(a['zv1'], 2),
           round(math.degrees(a['dp']), 3), round(a['R_d'], 3), round(a['zvp'], 2), round(a['T_c'], 2),
           round(a['T_half'], 2))
    fc.ok(got == (3.4118, 16.336, 120.88, 17.72, 34.287, 66.568, 18.15, 2388.24, 1194.12),
          'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(len(names) == 12, '12 components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds.count('RevoluteJointMotion') == 6 and kinds.count('RigidJointMotion') == 3
          and root.motionLinks.count == 4, 'six revolute and three rigid As-built Joints, four Motion Links: %s' % kinds)
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    # the final drive: the pinion one turn round in 30 deg steps
    jp = fc.joint('Pinion turns')
    R0 = {n: fc.rotation(fc.occ(n)) for n in ('Pinion', 'Case', 'Half shaft L', 'Half shaft R')}
    worst, hits, sgn = 0.0, [], None
    for i in range(1, 13):
        jp.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        tp = fc.turn_deg(fc.rotation(fc.occ('Pinion')), R0['Pinion'], (1, 0, 0))
        if sgn is None:
            sgn = 1 if tp > 0 else -1
        tp_total = 30.0 * i * sgn                                   # the pinion's turn so far, its sign measured
        want_c = tp_total * z1 / z2
        for n in ('Case', 'Half shaft L', 'Half shaft R'):
            t = fc.turn_deg(fc.rotation(fc.occ(n)), R0[n], (0, 1, 0))
            worst = max(worst, abs(t - want_c))
        hit = fc.interference([fc.occ('Pinion'), fc.occ('Ring gear')])
        if hit:
            hits.append((30 * i, hit))
    t_case = fc.turn_deg(fc.rotation(fc.occ('Case')), R0['Case'], (0, 1, 0))
    fc.ok(worst < 1e-3 and abs(abs(t_case) - 360.0 * z1 / z2) < 1e-3,
          'one pinion turn (%s about +X): the case and both half shafts turned %.3f deg about +Y, in step at every '
          '30 deg (worst %.1e deg)' % ('positive' if sgn > 0 else 'negative', t_case, worst))
    fc.ok(not hits, 'the pinion and the ring never collide at the 12 steps %s' % (hits or ''))
    for i in range(11, -1, -1):
        jp.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()

    # the differential: the left side gear turned in the case
    jl = fc.joint('Side gear L turns in case')
    names = ['Half shaft L', 'Half shaft R'] + ['Spider %d' % (i + 1) for i in range(N)]
    axes = {'Half shaft L': (0, 1, 0), 'Half shaft R': (0, 1, 0)}
    for i in range(N):
        p = 2 * math.pi * i / N
        axes['Spider %d' % (i + 1)] = (math.sin(p), 0, math.cos(p))
    R0 = {n: fc.rotation(fc.occ(n)) for n in names}
    jl.jointMotion.rotationValue = math.radians(30.0)
    adsk.doEvents()
    t = {n: fc.turn_deg(fc.rotation(fc.occ(n)), R0[n], axes[n]) for n in names}
    dL = t['Half shaft L']
    fc.ok(abs(abs(dL) - 30.0) < 1e-3 and abs(t['Half shaft R'] + dL) < 1e-3,
          'side gear L turned in the case: half shaft L %.3f deg, half shaft R %.3f deg about +Y' % (dL, t['Half shaft R']))
    sp = [t['Spider %d' % (i + 1)] for i in range(N)]
    fc.ok(all(abs(s + dL * zs / zp) < 1e-3 for s in sp),
          'each spider turned on its pin: %s deg (want %.3f, opposite to the side gear)' %
          (', '.join('%.3f' % s for s in sp), -dL * zs / zp))
    jl.jointMotion.rotationValue = 0
    adsk.doEvents()
    gears = [fc.occ('Side gear L'), fc.occ('Side gear R')] + [fc.occ('Spider %d' % (i + 1)) for i in range(N)]
    hits = []
    for i in range(13):
        jl.jointMotion.rotationValue = math.radians(i * 360.0 / zs / 12)
        adsk.doEvents()
        hit = fc.interference(gears)
        if hit:
            hits.append((round(i * 360.0 / zs / 12, 2), hit))
    fc.ok(not hits, 'no tooth collision while side gear L turns through one tooth pitch in 13 positions %s' % (hits or ''))
    jl.jointMotion.rotationValue = 0
    adsk.doEvents()
    fc.refused(ns, '13.67 virtual teeth', z_p=12)
    fc.refused(ns, '2 z_s / N = 14.67', N=3)
    fc.refused(ns, 'R_d / 3 = 22.19', b_d=24.0)
    fc.verdict()
