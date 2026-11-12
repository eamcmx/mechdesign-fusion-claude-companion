# CycloidalDrive (Chapter 19) against the acceptance criteria of its prompt, measured on what Fusion built: the input
# stepped round 30 deg at a time (a joint jumps the short way round otherwise, Chapter 18); at every step the output's
# turn, the disc's centre and its own turn measured on the bodies, and no interference.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/cycloidal_drive/CycloidalDrive/CycloidalDrive.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    n_p, R, d_pin, e = 11, 30.0, 6.0, 1.5
    k = ns['cycloid'](n_p, R, d_pin, e)
    want = dict(n_d=10, ratio=-10, K1=e * n_p / R, r_min=R - 3 - 0.1 - e, r_max=R - 3 - 0.1 + e)
    bad = [x for x in want if abs(k[x] - want[x]) > 1e-9]
    fc.ok(not bad, 'N_d %d, ratio %d, K1 %.3f, outline %.2f to %.2f mm %s' % (k['n_d'], k['ratio'], k['K1'], k['r_min'],
                                                                           k['r_max'], bad or ''))
    out = ns['disc_outline'](n_p, R, d_pin, e)
    rs = [math.hypot(x, y) for x, y in out]
    fc.ok(abs(min(rs) - 25.4) < 1e-6 and abs(max(rs) - 28.4) < 1e-6,
          'the outline\'s points from %.4f to %.4f mm from the disc\'s centre' % (min(rs), max(rs)))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Disc', 'Housing', 'Input', 'Output'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 3 and root.motionLinks.count == 2,
          'three revolute As-built Joints, two Motion Links')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    mm = adsk.core.Application.get().measureManager
    g = mm.measureMinimumDistance(fc.occ('Disc').bRepBodies.item(0), fc.occ('Housing').bRepBodies.item(0)).value * 10
    fc.ok(abs(g - ns['least_gap'](n_p, R, d_pin, e)) < 0.005 and g > 0.09,
          'the disc %.3f mm from the ring pins at rest, measured by Fusion (the chords\' worst, %.3f, worked out)'
          % (g, ns['least_gap'](n_p, R, d_pin, e)))
    j_in = fc.joint('Input turns')
    Ro, Rd = fc.rotation(fc.occ('Output')), fc.rotation(fc.occ('Disc'))
    worst_o, worst_c, worst_d, hits = 0.0, 0.0, 0.0, []
    for i in range(1, 13):
        th = 30.0 * i
        j_in.jointMotion.rotationValue = math.radians(th)
        adsk.doEvents()
        to = fc.turn_deg(fc.rotation(fc.occ('Output')), Ro, (0, 0, 1))
        td = fc.turn_deg(fc.rotation(fc.occ('Disc')), Rd, (0, 0, 1))
        worst_o = max(worst_o, abs(((to + th / 10 + 180) % 360) - 180))
        worst_d = max(worst_d, abs(((td + th / 10 + 180) % 360) - 180))
        p = adsk.core.Point3D.create(e * 0.1, 0, 0)
        p.transformBy(fc.occ('Disc').transform2)
        worst_c = max(worst_c, math.hypot(p.x * 10 - e * math.cos(math.radians(th)), p.y * 10 - e * math.sin(math.radians(th))))
        hit = fc.interference()
        if hit:
            hits.append((th, hit))
    fc.ok(worst_o < 1e-3, 'output at -input / 10 at every step (worst %.5f deg); after one turn %.3f deg' % (worst_o, to))
    fc.ok(worst_d < 1e-3, 'the disc itself turned -input / 10 at every step (worst %.5f deg)' % worst_d)
    fc.ok(worst_c < 1e-6, 'the disc\'s centre on the eccentric circle at every step (worst %.2e mm)' % worst_c)
    fc.ok(not hits, 'no interference at any of the 12 steps %s' % (hits or ''))
    for i in range(11, -1, -1):
        j_in.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
    fc.refused(ns, 'overlap', n_p=40)
    fc.refused(ns, 'K1 = e N_p / R = 1.100', e=3.0)
    fc.refused(ns, 'valleys are too sharp', e=2.5)
    fc.verdict()
