# SteeringRack (Chapter 29) against the acceptance criteria of its prompt, measured on what Fusion built: each
# knuckle's angle worked out here by Newton's method on the tie rod's length (the script intersects circles), the
# Ackermann error and the transmission angle recomputed; the pinion stepped 36 deg at a time each way and the rack
# and both knuckles measured on their bodies; no interference at any step.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/SteeringRack/SteeringRack.py'
T, WB, A, BETA, XR, C, P = 1400.0, 2500.0, 150.0, math.radians(15.0), -170.0, 300.0, 50.0


def tip(side, d):
    """Arm tip for a knuckle turned by d (anticlockwise about +z); side +1 left, -1 right."""
    ang = math.pi + side * BETA + d
    return (A * math.cos(ang), side * T / 2 + A * math.sin(ang))


LTR = math.hypot(tip(1, 0)[0] - XR, tip(1, 0)[1] - C)


def delta(side, u, d0=0.0):
    """The knuckle's angle for rack travel u: Newton on |tip(d) - ball joint| = tie rod, from d0."""
    d = d0
    for _ in range(60):
        f = lambda dd: math.hypot(tip(side, dd)[0] - XR, tip(side, dd)[1] - (side * C + u)) - LTR
        g = (f(d + 1e-7) - f(d - 1e-7)) / 2e-7
        step = f(d) / g
        d -= step
        if abs(step) < 1e-14:
            break
    return d


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    s = ns['steering'](T, WB, A, 15.0, XR, C, P)
    worst, dl, dr, u, mu = 0.0, 0.0, 0.0, 0.0, 180.0
    while True:                                                    # 0.5 mm steps, continuing each root
        u += 0.5
        dl, dr = delta(1, u, dl), delta(-1, u, dr)
        if abs(math.degrees(dr)) > 30.0:
            break
        ideal = math.atan(1 / (1 / math.tan(abs(dr)) + T / WB))
        worst = max(worst, abs(math.degrees(abs(dl) - ideal)))
        for side, d in ((1, dl), (-1, dr)):
            K, q = (0.0, side * T / 2), tip(side, d)
            v1, v2 = (q[0] - K[0], q[1] - K[1]), (XR - q[0], side * C + u - q[1])
            mu = min(mu, math.degrees(math.acos(abs(v1[0] * v2[0] + v1[1] * v2[1]) / (math.hypot(*v1) * math.hypot(*v2)))))
    fc.ok(abs(s['Ltr'] - LTR) < 1e-9 and abs(abs(s['worst']) - worst) < 1e-6 and abs(s['mu_min'] - mu) < 1e-6,
          'tie rod, worst Ackermann error and transmission angle recomputed independently (%.3f, %.3f, %.2f)'
          % (LTR, worst, mu))
    got = (round(s['Ltr'], 3), s['u_max'], round(s['inner_at_max'], 3), round(s['outer_at_max'], 3),
           round(abs(s['worst']), 3), round(s['mu_min'], 2), round(s['ratio'], 2), round(s['turns'], 2))
    fc.ok(got == (362.049, 62.0, 29.721, 23.971, 0.719, 35.19, 17.87, 2.48), 'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(len(names) == 7, '7 components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds.count('RevoluteJointMotion') == 7 and kinds.count('SliderJointMotion') == 1
          and root.motionLinks.count == 1, 'seven revolute and one slider As-built Joints, one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    jp = fc.joint('Pinion turns')
    rack, kl, kr = fc.occ('Rack'), fc.occ('Knuckle L'), fc.occ('Knuckle R')
    y0 = fc.position_mm(rack)[1]
    RL, RR = fc.rotation(kl), fc.rotation(kr)
    jp.jointMotion.rotationValue = math.radians(36.0)
    adsk.doEvents()
    sp = 1 if fc.position_mm(rack)[1] > y0 else -1                # the joint's sign: +y of rack per +value
    jp.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    worst_u = worst_d = 0.0
    hits = []
    for direction in (1, -1):
        dl = dr = 0.0
        k = 0
        while True:
            k += 1
            u = min(5.0 * k, 62.0) * direction
            jp.jointMotion.rotationValue = math.radians(u / P * 360.0) * sp
            adsk.doEvents()
            worst_u = max(worst_u, abs(fc.position_mm(rack)[1] - y0 - u))
            dl, dr = delta(1, u, dl), delta(-1, u, dr)
            ml = fc.turn_deg(fc.rotation(kl), RL, (0, 0, 1))
            mr = fc.turn_deg(fc.rotation(kr), RR, (0, 0, 1))
            worst_d = max(worst_d, abs(ml - math.degrees(dl)), abs(mr - math.degrees(dr)))
            hit = fc.interference()
            if hit:
                hits.append((u, hit))
            if abs(u) >= 62.0:
                break
        print('  at %+.0f mm of rack: knuckle L %.3f deg, knuckle R %.3f deg (worked out %.3f, %.3f)'
              % (u, ml, mr, math.degrees(dl), math.degrees(dr)))
        while k > 0:                                               # back to the middle, 36 deg at a time
            k -= 1
            jp.jointMotion.rotationValue = math.radians(min(5.0 * k, 62.0) * direction / P * 360.0) * sp
            adsk.doEvents()
    fc.ok(worst_u < 1e-4, 'the rack at %.0f mm per pinion turn at every step (worst %.1e mm)' % (P, worst_u))
    fc.ok(worst_d < 1e-3, 'both knuckles on the angles worked out here at every step, both ways (worst %.1e deg)'
          % worst_d)
    fc.ok(not hits, 'nothing interferes at any step %s' % (hits or ''))
    fc.refused(ns, 'up to 1.080 deg', beta=math.degrees(math.atan(700.0 / 2500.0)),
               x_r=-150.0 * math.cos(math.atan(700.0 / 2500.0)), c=400.0)
    fc.refused(ns, 'within 30.6 deg', beta=12.0, x_r=-195.0, c=400.0)
    fc.refused(ns, '32.8 mm long', c=640.0)
    fc.verdict()
