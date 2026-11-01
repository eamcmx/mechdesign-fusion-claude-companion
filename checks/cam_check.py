# CamFollower (Chapter 23) against the acceptance criteria of its prompt, measured on what Fusion built: the cam
# stepped through one turn, 10 deg at a time; at each step its real turn measured on its body, the follower set to the
# lift worked here from the motion law (not the script's function), then Interference and Fusion's own minimum
# distance between the roller and the cam.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/CamFollower/CamFollower.py'
H = 10.0


def s(t):
    """Cycloidal rise over 120 deg, dwell 60, cycloidal return over 120, dwell 60; t in degrees."""
    t = t % 360.0
    if t < 120:
        u = t / 120.0
        return H * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
    if t < 180:
        return H
    if t < 300:
        u = (t - 180) / 120.0
        return H * (1 - u + math.sin(2 * math.pi * u) / (2 * math.pi))
    return 0.0


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    c = ns['cam'](25.0, 6.0, H, 120.0, 60.0, 120.0)
    worst_s = max(abs(c['s'](math.radians(k / 10.0)) - s(k / 10.0)) for k in range(3600))
    fc.ok(worst_s < 1e-9, 'the script\'s lift agrees with the motion law (worst %.1e mm)' % worst_s)
    fc.ok((round(c['phi'], 2), round(c['rho'], 2)) == (14.97, 29.38),
          'largest pressure angle %.2f deg, smallest radius of curvature %.2f mm' % (c['phi'], c['rho']))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Cam', 'Follower', 'Ground'], 'three components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion', 'SliderJointMotion'] and root.motionLinks.count == 0,
          'a revolute and a slider As-built Joint, no Motion Link')
    jc, jf = fc.joint('Cam turns'), fc.joint('Follower slides')
    Rc = fc.rotation(fc.occ('Cam'))
    mm = adsk.core.Application.get().measureManager
    cam_b, fol_b = fc.occ('Cam').bRepBodies.item(0), fc.occ('Follower').bRepBodies.item(0)
    gaps, hits = [], []
    for k in range(0, 36):
        jc.jointMotion.rotationValue = math.radians(10.0 * k)
        adsk.doEvents()
        turned = fc.turn_deg(fc.rotation(fc.occ('Cam')), Rc, (0, 0, 1)) % 360.0
        jf.jointMotion.slideValue = s(turned) * 0.1
        adsk.doEvents()
        hit = fc.interference([fc.occ('Cam'), fc.occ('Follower')])
        if hit:
            hits.append((10 * k, hit))
        gaps.append((10 * k, mm.measureMinimumDistance(cam_b, fol_b).value * 10))
    lo, hi = min(g for _, g in gaps), max(g for _, g in gaps)
    print('  smallest distance roller to cam (mm): ' + ', '.join('%d: %.4f' % (a, g) for a, g in gaps[::3]))
    fc.ok(not hits, 'no interference at the 36 steps %s' % (hits or ''))
    fc.ok(0.049 <= lo and hi <= 0.052, 'the gap measured by Fusion from %.4f to %.4f mm (0.05 wanted)' % (lo, hi))
    for k in range(35, -1, -1):
        jc.jointMotion.rotationValue = math.radians(10.0 * k)
        adsk.doEvents()
    jf.jointMotion.slideValue = 0
    fc.refused(ns, 'pressure angle reaches 38.7', beta1=40.0)
    fc.refused(ns, '= 380 deg', beta2=200.0)
    fc.verdict()
