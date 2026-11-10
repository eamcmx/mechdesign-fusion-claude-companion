# FourBar (Chapter 22) against the acceptance criteria of its prompt, measured on what Fusion built: the crank stepped
# through one turn, 30 deg at a time; at every step the rocker's turn measured on its body and compared with
# Freudenstein's equation, worked here independently of the script's circle intersection; no interference.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/four_bar/FourBar/FourBar.py'
A, B, C, D = 15.0, 55.0, 50.0, 55.0


def freudenstein(t2):
    """theta4 from K1 cos(t4) - K2 cos(t2) + K3 = cos(t2 - t4), solved in closed form."""
    K1, K2, K3 = D / A, D / C, (A * A - B * B + C * C + D * D) / (2 * A * C)
    a_ = math.cos(t2) - K1 - K2 * math.cos(t2) + K3
    b_ = -2 * math.sin(t2)
    c_ = K1 - (K2 + 1) * math.cos(t2) + K3
    return 2 * math.atan2(-b_ - math.sqrt(b_ * b_ - 4 * a_ * c_), 2 * a_)               # the minus root: the open branch


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    k = ns['linkage'](A, B, C, D)
    t4f = [math.degrees(freudenstein(math.radians(x))) for x in range(360)]
    t4f = [((t + 180) % 360) - 180 for t in t4f]
    mu = lambda t2: math.degrees(math.acos((B * B + C * C - (A * A + D * D - 2 * A * D * math.cos(t2))) / (2 * B * C)))
    # the rocker's extremes in closed form: at the toggle positions crank and coupler lie in one line, O2C = b +- a,
    # and the triangle O2 O4 C gives the rocker's angle at O4 (the upper branch)
    ext = [180.0 - math.degrees(math.acos((D * D + C * C - r * r) / (2 * D * C))) for r in (B - A, B + A)]
    want = dict(t4_0=t4f[0], t4_min=min(ext), t4_max=max(ext), swing=max(ext) - min(ext), mu_min=mu(0), mu_max=mu(math.pi))
    bad = [x for x in want if abs(k[x] - want[x]) > 1e-5]
    fc.ok(not bad, 'the script\'s circles agree with Freudenstein: theta4 %.3f at the start, swing %.3f (%.3f to %.3f), '
                   'mu %.2f to %.2f %s' % (k['t4_0'], k['swing'], k['t4_min'], k['t4_max'], k['mu_min'], k['mu_max'], bad or ''))
    fc.ok((round(k['t4_0'], 3), round(k['swing'], 3), round(k['mu_min'], 2), round(k['mu_max'], 2)) == (105.59, 39.007, 44.47, 83.48),
          'the rounded values of the prompt')
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Coupler', 'Crank', 'Ground', 'Rocker'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 4 and
          all(type(j.jointMotion).__name__ == 'RevoluteJointMotion' for j in root.asBuiltJoints), 'four revolute As-built Joints')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    jc = fc.joint('Crank turns')
    Rr = fc.rotation(fc.occ('Rocker'))
    worst, hits, rows = 0.0, [], []
    for i in range(1, 13):
        t2 = 30.0 * i
        jc.jointMotion.rotationValue = math.radians(t2)
        adsk.doEvents()
        turned = fc.turn_deg(fc.rotation(fc.occ('Rocker')), Rr, (0, 0, 1))
        want_t = math.degrees(freudenstein(math.radians(t2))) - math.degrees(freudenstein(0.0))
        want_t = ((want_t + 180) % 360) - 180
        worst = max(worst, abs(turned - want_t))
        rows.append('%d: %.3f' % (t2, turned))
        hit = fc.interference()
        if hit:
            hits.append((t2, hit))
    print('  the rocker turned (deg): ' + ', '.join(rows))
    fc.ok(worst < 1e-3, 'at every step the rocker\'s body where Freudenstein puts it (worst %.5f deg)' % worst)
    fc.ok(not hits, 'no interference at the 12 steps %s' % (hits or ''))
    for i in range(11, -1, -1):
        jc.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
    fc.refused(ns, 'Not a crank-rocker', a=60.0)
    fc.refused(ns, 'falls to 38.57 deg', a=20.0)
    fc.refused(ns, 'falls to 31.90 deg', a=20.0, b=38.0, c=40.0, d=55.0)   # the acute angle at 180 deg of the crank
    fc.verdict()
