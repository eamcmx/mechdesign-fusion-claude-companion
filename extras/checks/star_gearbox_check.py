# StarGearbox (Chapter 32) against the acceptance criteria of its prompt, measured on what Fusion built: the numbers
# recomputed independently; the sun turned in 30 deg steps with the ring and the stars measured on their bodies; no
# tooth collision while the sun turns through one tooth pitch in steps Fusion keeps exactly (tenths of a degree).
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/extras/StarGearbox/StarGearbox.py'
M, ZS, ZP, N = 3.0, 32, 33, 5


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    zr = ZS + 2 * ZP
    Ts = 20e6 / (10000.0 * 2 * math.pi / 60.0)
    want = dict(zr=zr, a=M * (ZS + ZP) / 2, ratio=zr / ZS, n_fan=-10000.0 * ZS / zr, T_s=Ts, T_r=Ts * zr / ZS,
                T_c=Ts * (1 + zr / ZS), Ft=Ts / (N * M * ZS / 2000.0), heat=0.005 * 20e6)
    g = ns['star'](M, ZS, ZP, N, 20.0)
    bad = [k for k in want if abs(g[k] - want[k]) > 1e-6]
    fc.ok(not bad and g['phases_equal'], 'all values recomputed independently %s' % (bad or ''))
    got = (g['zr'], round(g['a'], 3), round(g['ratio'], 4), round(g['n_fan'], 1), round(g['T_s'], 1), round(g['T_r'], 1),
           round(g['T_c'], 1), round(g['Ft']), round(g['heat'] / 1000))
    fc.ok(got == (98, 97.5, 3.0625, -3265.3, 19098.6, 58489.4, 77588.0, 79577, 100), 'the rounded values of the prompt %s'
          % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(len(names) == 8, '8 components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 7 and root.motionLinks.count == 6,
          'seven revolute As-built Joints, six Motion Links')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    js = fc.joint('Sun (from the turbine)')
    sun, ring = fc.occ('Sun'), fc.occ('Ring')
    stars = [fc.occ('Star %d' % (i + 1)) for i in range(N)]
    R0 = {o.component.name: fc.rotation(o) for o in [sun, ring] + stars}
    for i in range(1, 4):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
    t_sun = fc.turn_deg(fc.rotation(sun), R0['Sun'], (0, 0, 1))
    t_ring = fc.turn_deg(fc.rotation(ring), R0['Ring'], (0, 0, 1))
    t_st = [fc.turn_deg(fc.rotation(s), R0[s.component.name], (0, 0, 1)) for s in stars]
    fc.ok(abs(t_sun - 90.0) < 1e-6 and abs(t_ring + 90.0 * ZS / zr) < 1e-3,
          'the sun turned %.3f deg: the ring %.3f deg (want %.3f)' % (t_sun, t_ring, -90.0 * ZS / zr))
    fc.ok(all(abs(t + 90.0 * ZS / ZP) < 1e-3 for t in t_st), 'each star %s deg (want %.3f)'
          % (', '.join('%.3f' % t for t in t_st), -90.0 * ZS / ZP))
    for i in range(2, -1, -1):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
    hits = []
    for k in range(0, 14):
        js.jointMotion.rotationValue = math.radians(0.9 * k)
        adsk.doEvents()
        hit = fc.interference([sun, ring] + stars)
        if hit:
            hits.append((round(0.9 * k, 1), hit))
    fc.ok(not hits, 'no tooth collision while the sun turns 0 to 11.7 deg in 0.9 deg steps %s' % (hits or ''))
    js.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.refused(ns, '= 32.50 is not whole', n=4)
    fc.refused(ns, 'Neighbouring stars touch', zs=20, zp=40)
    fc.refused(ns, 'below 18', zs=16)
    fc.verdict()
