# FlapActuator (Chapter 31) against the acceptance criteria of its prompt, measured on what Fusion built: the
# numbers recomputed independently (the flap's angle for a nut position by Newton's method, where the script
# bisects); the screw turned in 144 deg steps (never past half a turn, Chapter 18) with the nut and the flap measured
# on their bodies; no interference at any step.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/FlapActuator/FlapActuator.py'
L, D0, MU, R, PHI0, ZS, LK, C, S, V = 5.0, 16.0, 0.005, 100.0, math.radians(-45.0), 60.0, 150.0, 400.0, 1500.0, 40.0


def nut(d):
    """The nut's pin for the flap down by d: on z = ZS at LK from the horn's tip, forward of it."""
    tx, tz = R * math.sin(PHI0 + d), R * math.cos(PHI0 + d)
    return tx - math.sqrt(LK * LK - (ZS - tz) ** 2)


def flap_for(xn, d=0.3):
    for _ in range(60):
        f = nut(d) - xn
        g = (nut(d + 1e-7) - nut(d - 1e-7)) / 2e-7
        d -= f / g
    return d


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    g = ns['drive'](L, D0, MU, R, -45.0, ZS, LK, C, S, V)
    lam = math.atan(L / (math.pi * D0))
    a = math.radians(45.0)
    eta = (math.cos(a) - MU * math.tan(lam)) / (math.cos(a) + MU / math.tan(lam))
    eta_b = (math.cos(a) - MU / math.tan(lam)) / (math.cos(a) + MU * math.tan(lam))
    q = 0.5 * 1.225 * V * V
    M40 = q * 0.4 * 1.5 * 1.2 * 0.45 * 0.4
    d40 = math.radians(40.0)
    F40 = M40 / ((nut(d40 + 1e-6) - nut(d40 - 1e-6)) / 2e-6 / 1000.0)
    ok = (abs(g['travel'] - (nut(d40) - nut(0.0))) < 1e-9 and abs(g['sc']['eta'] - eta) < 1e-12
          and abs(g['sc']['eta_b'] - eta_b) < 1e-12 and abs(g['M_full'] - M40) < 1e-9 and abs(g['F_max'] - F40) < 1e-3)
    fc.ok(ok, 'travel, efficiencies, hinge moment and nut force recomputed independently')
    got = (round(math.degrees(g['sc']['lam']), 3), round(g['travel'], 3), round(g['turns'], 3), round(g['mu_min'], 2),
           round(g['sc']['eta'], 4), round(g['sc']['eta_b'], 4), round(g['sc']['mu_lock'], 4), round(g['q'], 1),
           round(g['M_full'], 3), round(g['F_max'], 1), round(g['T_max'], 4), round(g['Tb_max'], 4))
    fc.ok(got == (5.681, 66.939, 13.388, 49.09, 0.933, 0.9283, 0.0703, 980.0, 127.008, 1245.1, 1.062, 0.9197),
          'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(len(names) == 5, '5 components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds.count('RevoluteJointMotion') == 4 and kinds.count('SliderJointMotion') == 1
          and root.motionLinks.count == 1, 'four revolute and one slider As-built Joints, one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    js = fc.joint('Screw turns')
    nut_o, flap, scr = fc.occ('Nut'), fc.occ('Flap'), fc.occ('Screw')
    x0, RF, RS = fc.position_mm(nut_o)[0], fc.rotation(flap), fc.rotation(scr)
    js.jointMotion.rotationValue = math.radians(36.0)
    adsk.doEvents()
    s = 1 if fc.position_mm(nut_o)[0] > x0 else -1                # +value runs the nut aft?
    turned = fc.turn_deg(fc.rotation(scr), RS, (1, 0, 0))
    hand = fc.position_mm(nut_o)[0] - x0
    js.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.ok((turned > 0) != (hand > 0), 'a right-hand screw: %.1f deg about +x moved the nut %.3f mm along x'
          % (turned, hand))
    travel = g['travel']
    steps = int(travel // 2.0)
    targets = [2.0 * k for k in range(1, steps + 1)] + [travel]
    worst_n = worst_f = 0.0
    hits = []
    for u in targets:
        js.jointMotion.rotationValue = math.radians(u / L * 360.0) * s
        adsk.doEvents()
        u_held = math.degrees(abs(js.jointMotion.rotationValue)) / 360.0 * L     # the angle Fusion holds, in mm
        un = fc.position_mm(nut_o)[0] - x0
        worst_n = max(worst_n, abs(un - u_held))
        d = math.degrees(flap_for(nut(0.0) + un))
        df = fc.turn_deg(fc.rotation(flap), RF, (0, 1, 0))
        worst_f = max(worst_f, abs(df - d))
        hit = fc.interference()
        if hit:
            hits.append((round(u, 3), hit))
    fc.ok(worst_n < 1e-6, 'the nut at %.0f mm per turn of the screw at all %d steps (worst %.1e mm)'
          % (L, len(targets), worst_n))
    fc.ok(worst_f < 1e-3, 'the flap at the angle worked out for the nut, at every step (worst %.1e deg); at the end %.3f deg'
          % (worst_f, df))
    fc.ok(abs(df - 40.0) < 0.01, 'full travel brings the flap to 40 deg (%.4f)' % df)
    fc.ok(not hits, 'nothing interferes at any step %s' % (hits or ''))
    for u in reversed([0.0] + targets[:-1]):
        js.jointMotion.rotationValue = math.radians(u / L * 360.0) * s
        adsk.doEvents()
    fc.refused(ns, 'within 33.74 deg', zs=100.0)
    fc.refused(ns, 'lock itself (at 0.0703)', mu=0.08)
    fc.refused(ns, 'cannot reach', l=10.0)
    fc.verdict()
