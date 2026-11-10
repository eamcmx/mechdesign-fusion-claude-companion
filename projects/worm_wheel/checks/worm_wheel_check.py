# WormWheel (Chapter 18) against the acceptance criteria of its prompt, measured on what Fusion built: the numbers
# recomputed independently; the wheel's turn about +Z measured on its body when the worm is driven one turn about +X;
# no collision at the 12 steps of that turn.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/worm_wheel/WormWheel/WormWheel.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    m, q, z2, ax = 2.0, 10.0, 30, math.radians(20)
    lam = math.atan(1 / q)
    pn = math.atan(math.tan(ax) * math.cos(lam))
    want = dict(d1=q * m, L=math.pi * m, lam=lam, d2=m * z2, a=(q * m + m * z2) / 2, ratio=30.0, phi_n=pn,
                eff05=(math.cos(pn) - 0.05 * math.tan(lam)) / (math.cos(pn) + 0.05 / math.tan(lam)),
                eff10=(math.cos(pn) - 0.10 * math.tan(lam)) / (math.cos(pn) + 0.10 / math.tan(lam)),
                back10=(math.cos(pn) - 0.10 / math.tan(lam)) / (math.cos(pn) + 0.10 * math.tan(lam)),
                f_lock=math.cos(pn) * math.tan(lam))
    w = ns['worm'](m, q, z2, 20.0, 1)
    bad = [k for k in want if abs(w[k] - want[k]) > 1e-9]
    fc.ok(not bad, 'all values recomputed independently %s' % (bad or ''))
    bw = math.radians(round(math.degrees(8 * math.tan(lam) / 30), 1)) * 30 / math.tan(lam)
    got = (round(w['L'], 4), round(math.degrees(w['lam']), 4), w['a'], round(bw, 3), round(math.degrees(w['phi_n']), 3),
           round(w['eff05'], 3), round(w['eff10'], 3), round(w['back10'], 3), round(w['f_lock'], 4))
    fc.ok(got == (6.2832, 5.7106, 40.0, 7.854, 19.909, 0.649, 0.479, -0.063, 0.094), 'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Wheel', 'Wheel shaft', 'Worm', 'Worm shaft'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 2 and root.motionLinks.count == 1,
          'two revolute As-built Joints, one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    jw = fc.joint('Worm turns')
    Rw, Rg = fc.rotation(fc.occ('Worm')), fc.rotation(fc.occ('Wheel'))
    # Fusion moves a revolute joint to the angle nearest its present value (the same modulo 360 deg): asked for 360
    # from 0 it stays at 0. So the worm is stepped round, 30 deg at a time, never reset in between.
    worst = []
    for i in range(1, 13):
        jw.jointMotion.rotationValue = math.radians(i * 30.0)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Worm'), fc.occ('Wheel')])
        if hit:
            worst.append((i * 30, hit))
        if i == 3:
            tw = fc.turn_deg(fc.rotation(fc.occ('Worm')), Rw, (1, 0, 0))
            tg = fc.turn_deg(fc.rotation(fc.occ('Wheel')), Rg, (0, 0, 1))
    fc.ok(abs(tw - 90) < 1e-3 and abs(tg + 3) < 1e-3,
          'the worm turned %.3f deg about +X: the wheel turned %.4f deg about +Z (want -3.0000)' % (tw, tg))
    vw = math.degrees(jw.jointMotion.rotationValue)
    vg = math.degrees(fc.joint('Wheel turns').jointMotion.rotationValue)
    tg360 = fc.turn_deg(fc.rotation(fc.occ('Wheel')), Rg, (0, 0, 1))
    fc.ok(abs(vw - 360) < 1e-6 and abs(tg360 + 12) < 1e-3 and abs(abs(vg) - 12) < 1e-3,
          'one worm turn (joint %.1f deg): the wheel turned %.4f deg about +Z, its joint %.4f' % (vw, tg360, vg))
    fc.ok(not worst, 'no interference at the 12 steps of 30 deg through one worm turn %s' % (worst or ''))
    for i in range(11, -1, -1):
        jw.jointMotion.rotationValue = math.radians(i * 30.0)
        adsk.doEvents()
    fc.refused(ns, 'undercut', z2=16)
    fc.refused(ns, 'Lead angle 26.6 deg', q=2.0)
    fc.verdict()
