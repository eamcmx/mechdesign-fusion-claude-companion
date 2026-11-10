# HelicalPair (Chapter 14) against the acceptance criteria of its prompt, measured on what Fusion built: the numbers
# recomputed independently; the twist of each gear read from the angles of its teeth's tips at the bottom, middle and
# top faces; the mesh driven and swept for collisions; the herringbone the same way.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/helical_pair/HelicalPair/HelicalPair.py'
MN, Z1, Z2, BETA, AN, B = 2.0, 20, 40, math.radians(20), math.radians(20), 12.0


def twist_residual(o, z, ra, z0, bw, phase, want):
    """Points along every tooth-tip edge, each compared with where the tip should be at its height: the untwisted
    tooth centre plus want(f) degrees, f the height as a fraction of the face width. The tips of a tooth lie symmetric
    about its centre, so the mean residual is zero when the twist is right at every height, and the wrong hand
    cannot pass, though at the top face it may land a whole tooth pitch away and look right."""
    p = 2 * math.pi / z
    res = []
    for e in o.component.bRepBodies.item(0).edges:
        a, b = e.startVertex.geometry, e.endVertex.geometry
        if abs(math.hypot(a.x, a.y) * 10 - ra) < 1e-3 and abs(math.hypot(b.x, b.y) * 10 - ra) < 1e-3                 and abs(a.z - b.z) * 10 > 1e-3:
            ev = e.evaluator
            _, t0, t1 = ev.getParameterExtents()
            for i in range(11):
                _, q = ev.getPointAtParameter(t0 + (t1 - t0) * i / 10)
                r = math.atan2(q.y, q.x) - phase - math.radians(want((q.z * 10 - z0) / bw))
                res.append(((r + p / 2) % p) - p / 2)
    if not res:
        return None, 0
    ref = res[0]
    res = [ref + ((r - ref + p / 2) % p) - p / 2 for r in res]
    m = sum(res) / len(res)
    return math.degrees(((m + p / 2) % p) - p / 2), len(res)


def check_pair(a, herring, bw):
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Base plate', 'Gear 1', 'Gear 2', 'Shaft 1', 'Shaft 2'], 'five components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    x2 = fc.position_mm(fc.occ('Gear 2'))[0]
    fc.ok(abs(x2 - a) < 1e-6, 'gear 2 at x = a = %.3f mm (measured %.3f)' % (a, x2))
    mt = MN / math.cos(BETA)
    for name, z, phase, hand in (('Gear 1', Z1, 0.0, 1), ('Gear 2', Z2, math.pi - math.pi / Z2, -1)):
        th = hand * math.degrees(bw * math.tan(BETA) / (mt * z / 2))
        per = th / 2 if herring else th                       # Fusion keeps each sweep's twist in tenths of a degree
        tol = abs(per - round(per, 1)) + 1e-3
        want = (lambda f, th=th: th * min(f, 1 - f)) if herring else (lambda f, th=th: th * f)
        m, n = twist_residual(fc.occ(name), z, mt * z / 2 + MN, 1.0, bw, phase, want)
        fc.ok(n > 0 and abs(m) <= tol, '%s: %d points along the tooth tips, mean %.4f deg from a %s twist of %.3f deg'
              % (name, n, m if n else float('nan'), 'herringbone' if herring else 'helical', th))
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 4 and root.motionLinks.count == 1,
          'four As-built Joints, no plain Joints, one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    j1, j2 = fc.joint('Gear 1 turns'), fc.joint('Gear 2 turns')
    R0 = fc.rotation(fc.occ('Gear 2'))
    j1.jointMotion.rotationValue = math.radians(90)
    adsk.doEvents()
    v2 = math.degrees(j2.jointMotion.rotationValue)
    b2 = fc.turn_deg(fc.rotation(fc.occ('Gear 2')), R0, (0, 0, 1))
    fc.ok(abs(v2 + 45) < 1e-3 and abs(b2 + 45) < 1e-3, 'gear 1 driven 90 deg: gear 2 joint %.3f, body %.3f' % (v2, b2))
    worst = []
    for k in range(13):
        ang = k * 360.0 / Z1 / 12
        j1.jointMotion.rotationValue = math.radians(ang)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Gear 1'), fc.occ('Gear 2')])
        if hit:
            worst.append((round(ang, 2), hit))
    fc.ok(not worst, 'no tooth collision while gear 1 turns 0 to 18 deg in 13 positions %s' % (worst or ''))
    j1.jointMotion.rotationValue = 0
    adsk.doEvents()


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    mt = MN / math.cos(BETA)
    at = math.atan(math.tan(AN) / math.cos(BETA))
    r1, r2 = mt * Z1 / 2, mt * Z2 / 2
    rb1, rb2 = r1 * math.cos(at), r2 * math.cos(at)
    ea = (math.sqrt((r1 + MN) ** 2 - rb1 ** 2) + math.sqrt((r2 + MN) ** 2 - rb2 ** 2) - (r1 + r2) * math.sin(at)) \
        / (math.pi * mt * math.cos(at))
    th1 = round(math.degrees(B * math.tan(BETA) / r1), 1)              # gear 1's twist in whole tenths of a degree
    bw = math.radians(th1) * r1 / math.tan(BETA)                       # the face width that gives it: b'
    want = dict(mt=mt, at=at, a=r1 + r2, th1=bw * math.tan(BETA) / r1, th2=bw * math.tan(BETA) / r2,
                zmin=2 * math.cos(BETA) / math.sin(at) ** 2, eps_a=ea, eps_b=bw * math.sin(BETA) / (math.pi * MN))
    h = ns['helical'](MN, Z1, Z2, 20.0, 20.0, bw)
    bad = [k for k in want if abs(h[k] - want[k]) > 1e-9]
    fc.ok(not bad, 'all values recomputed independently %s' % (bad or ''))
    got = (round(h['mt'], 5), round(math.degrees(h['at']), 3), round(h['a'], 3), round(bw, 3), round(math.degrees(h['th1']), 3),
           round(math.degrees(h['th2']), 3), round(h['zmin'], 2), round(h['eps_a'], 3), round(h['eps_b'], 3))
    fc.ok(got == (2.12836, 21.173, 63.851, 12.043, 11.8, 5.9, 14.41, 1.504, 0.656), 'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    check_pair(want['a'], False, bw)
    print('  herringbone:')
    print('  ' + ns['build'](herringbone=True).splitlines()[4])
    fc.mark()
    check_pair(want['a'], True, bw)
    fc.refused(ns, 'z_min = 2 cos(beta) / sin^2(alpha_t) = 14.41', z1=14)
    fc.refused(ns, 'from 5 to 45', beta=50.0)
    fc.verdict()
