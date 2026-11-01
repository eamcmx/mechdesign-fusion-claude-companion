# BallScrewAxis (Chapter 21) against the acceptance criteria of its prompt, measured on what Fusion built: the values
# recomputed here from the catalogue's formulas, the parts and joints, and the screw stepped through one turn with the
# carriage measured and every step checked for interference (the balls must stay in both grooves).
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/BallScrewAxis/BallScrewAxis.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    k = ns['ballscrew']()
    P = 5.0
    want = dict(d_m=16.6, R_g=0.52 * 3.175, d_k=16.6 - 2 * 0.52 * 3.175, per_turn=16, balls=48,
                T_a=500 * P / (2 * math.pi * 0.9), T_c=500 * P * 0.8 / (2 * math.pi),
                L=(7310 / 600.0) ** 3 * 1e6, l_s=190.0)
    want['L_h'] = want['L'] / 60000.0
    want['F_k'] = 4.072e5 * 0.25 * want['d_k'] ** 4 / 190.0 ** 2
    want['n_k'] = 2.71e8 * 0.446 * want['d_k'] / 190.0 ** 2
    bad = [x for x in want if abs(k[x] - want[x]) > 1e-6 * max(1.0, abs(want[x]))]
    fc.ok(not bad, 'd_m %.2f, R_g %.3f, d_k %.3f; %d balls a turn, %d; T_a %.2f, T_c %.2f N.mm; L %.4e, L_h %.0f h; '
                   'F_k %.0f N, n_k %.0f rpm %s' % (k['d_m'], k['R_g'], k['d_k'], k['per_turn'], k['balls'], k['T_a'],
                                                    k['T_c'], k['L'], k['L_h'], k['F_k'], k['n_k'], bad or ''))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Balls', 'Carriage', 'Frame', 'Guide rod', 'Nut', 'Screw'], 'six components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 5 and root.motionLinks.count == 1,
          'five As-built Joints, one Motion Link')
    fc.ok(fc.design().rootComponent is not None and fc.volume_mm3(fc.occ('Balls')) > 0,
          'the balls: %.1f mm^3 against 48 spheres of %.1f' % (fc.volume_mm3(fc.occ('Balls')),
                                                              48 * 4 / 3 * math.pi * (3.175 / 2) ** 3))
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    js = fc.joint('Screw turns')
    x0 = fc.position_mm(fc.occ('Carriage'))[0]
    worst, hits = 0.0, []
    for i in range(1, 13):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        dx = fc.position_mm(fc.occ('Carriage'))[0] - x0
        worst = max(worst, abs(dx + P * i / 12.0))
        hit = fc.interference()
        if hit:
            hits.append((30 * i, hit))
    fc.ok(worst < 1e-6, 'the carriage at -P/12 per 30 deg step (worst %.2e mm); after one turn %.4f mm' % (worst, dx))
    fc.ok(not hits, 'no interference at any of the 12 steps %s' % (hits or ''))
    for i in range(11, -1, -1):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
    fc.refused(ns, 'grooves overlap', P=3.0)
    fc.refused(ns, 'below 10 000 h', F=3000.0)
    fc.verdict()
