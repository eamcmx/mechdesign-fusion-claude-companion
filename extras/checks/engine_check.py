# SingleCylinderEngine (Chapter 28) against the acceptance criteria of its prompt, measured on what Fusion built: the
# numbers recomputed independently; the crank stepped 30 deg at a time through two turns (never one jump, Chapter 18)
# with the piston and the camshaft measured on their bodies; each valve set to the lift law written here from the
# prompt, and the gap between each roller and its lobe measured by Fusion at every step.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/extras/SingleCylinderEngine/SingleCylinderEngine.py'
B, S, L, H = 60.0, 54.0, 90.0, 6.0
R = S / 2
EVENTS = {'in': (-10.0, 240.0), 'ex': (490.0, 240.0)}          # opening crank angle and open duration


def lift(which, theta):
    """The prompt's lift law, written again here: cycloidal rise over D/4 cam degrees, return over D/4."""
    o, D = EVENTS[which]
    t = ((theta - o) / 2.0) % 360.0
    b = D / 4.0
    if t < b:
        u = t / b
        return H * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
    if t < 2 * b:
        u = (t - b) / b
        return H * (1 - (u - math.sin(2 * math.pi * u) / (2 * math.pi)))
    return 0.0


def x_pin(theta_deg):
    t = math.radians(theta_deg)
    return R * math.cos(t) + math.sqrt(L * L - (R * math.sin(t)) ** 2)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    e = ns['engine'](B, S, L, 10, 50, 50, 10)
    ok_num = (abs(e['lam'] - 0.3) < 1e-12 and abs(e['Vd'] - math.pi / 4 * B * B * S) < 1e-9
              and e['ev'] == {'io': -10.0, 'ic': 230.0, 'eo': 490.0, 'ec': 730.0} and e['overlap'] == 20.0
              and all(abs(ns['piston_x'](R, L, math.radians(a)) - x_pin(a)) < 1e-12 for a in range(0, 360, 15))
              and all(abs(ns['valve_lift'](e, H, w, a) - lift(w, a)) < 1e-12 for w in ('in', 'ex') for a in range(0, 1440, 5)))
    fc.ok(ok_num, 'all values recomputed independently (lambda, swept volume, events, x(theta), the lift law)')
    phi = ns['cam'](15.0, 5.0, H, 240.0)['phi']
    got = (round(e['lam'], 4), round(e['Vd'] / 1000, 2), e['d_in'], e['d_ex'], e['overlap'], round(phi, 2))
    fc.ok(got == (0.3, 152.68, 240.0, 240.0, 20.0, 26.64), 'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(len(names) == 9, '9 components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds.count('RevoluteJointMotion') == 4 and kinds.count('SliderJointMotion') == 3
          and kinds.count('RigidJointMotion') == 2 and root.motionLinks.count == 1,
          'four revolute, three slider and two rigid As-built Joints, one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    jc = fc.joint('Crank turns')
    jv = {'in': fc.joint('Inlet valve slides'), 'ex': fc.joint('Exhaust valve slides')}
    occ_v = {'in': fc.occ('Inlet valve'), 'ex': fc.occ('Exhaust valve')}
    crank, cam, piston = fc.occ('Crankshaft'), fc.occ('Camshaft'), fc.occ('Piston')
    Rc0, Rk0 = fc.rotation(crank), fc.rotation(cam)
    y0 = fc.position_mm(piston)[1]
    # which way the joints go: a crank turning anticlockwise about +Z, valves lifting along +Y
    jc.jointMotion.rotationValue = 0.1
    adsk.doEvents()
    sc = 1 if fc.turn_deg(fc.rotation(crank), Rc0, (0, 0, 1)) > 0 else -1
    jc.jointMotion.rotationValue = 0.0
    sv = {}
    for w in ('in', 'ex'):
        jv[w].jointMotion.slideValue = 0.1
        adsk.doEvents()
        sv[w] = 1 if fc.position_mm(occ_v[w])[1] > 0 else -1
        jv[w].jointMotion.slideValue = 0.0
    adsk.doEvents()
    mm = adsk.core.Application.get().measureManager
    cam_b = cam.bRepBodies.item(0)
    worst_x = worst_k = 0.0
    gaps, hits = [], []
    for i in range(1, 25):
        d = 30.0 * i                                              # crank turned so far, from TDC of compression
        theta = 360.0 + d
        jc.jointMotion.rotationValue = math.radians(d) * sc
        for w in ('in', 'ex'):
            jv[w].jointMotion.slideValue = lift(w, theta) * 0.1 * sv[w]
        adsk.doEvents()
        tc = fc.turn_deg(fc.rotation(crank), Rc0, (0, 0, 1)) % 360.0
        worst_x = max(worst_x, abs(fc.position_mm(piston)[1] - y0 + (R + L) - x_pin(d)))
        tk = fc.turn_deg(fc.rotation(cam), Rk0, (0, 0, 1))
        want = -d / 2.0
        worst_k = max(worst_k, abs(((tk - want + 180.0) % 360.0) - 180.0), abs(((tc - d + 180.0) % 360.0) - 180.0))
        for w in ('in', 'ex'):
            gaps.append((theta, w, mm.measureMinimumDistance(cam_b, occ_v[w].bRepBodies.item(0)).value * 10))
        hit = fc.interference()
        if hit:
            hits.append((theta, hit))
    fc.ok(worst_x < 1e-4, 'the piston pin on x(theta) at all 24 steps of two turns (worst %.1e mm)' % worst_x)
    fc.ok(worst_k < 1e-3, 'the crank at theta and the camshaft at -theta / 2 at every step (worst %.1e deg)' % worst_k)
    lo, hi = min(g for _, _, g in gaps), max(g for _, _, g in gaps)
    print('  roller to lobe (mm): ' + ', '.join('%d %s %.4f' % (t, w, g) for t, w, g in gaps[::4]))
    fc.ok(0.045 <= lo and hi <= 0.055, 'each roller 0.05 mm from its lobe at every step: %.4f to %.4f mm' % (lo, hi))
    fc.ok(not hits, 'nothing interferes at any of the 24 steps %s' % (hits or ''))
    for i in range(23, -1, -1):
        jc.jointMotion.rotationValue = math.radians(30.0 * i) * sc
        adsk.doEvents()
    for w in ('in', 'ex'):
        jv[w].jointMotion.slideValue = 0.0
    adsk.doEvents()
    fc.refused(ns, 'lambda = 0.360', l=75.0)
    fc.refused(ns, 'pressure angle of 31.0', IVO=0.0, IVC=20.0)
    fc.refused(ns, 'bends to 11.18 mm', Rb=8.0, Rr=12.0)
    fc.verdict()
