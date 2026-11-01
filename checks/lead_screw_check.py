# LeadScrewAxis (Chapter 20) against the acceptance criteria of its prompt, measured on what Fusion built: the screw's
# numbers recomputed independently; the screw stepped through one turn about +x (Chapter 17: never in one jump); at
# every step the carriage's travel by its joint and its body, and no contact between the threads.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/LeadScrewAxis/LeadScrewAxis.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    d, P, F, f = 12.0, 3.0, 500.0, 0.15
    d2, sec = d - P / 2, 1 / math.cos(math.radians(15))
    T = F * d2 / 2 * (P + math.pi * f * d2 * sec) / (math.pi * d2 - f * P * sec)
    want = dict(L=3.0, d2=d2, d3=d - P - 0.5, D1=d - P, D4=d + 0.5, lam=math.atan(P / (math.pi * d2)), T=T,
                eff=F * P / (2 * math.pi * T), T_low=F * d2 / 2 * (math.pi * f * d2 * sec - P) / (math.pi * d2 + f * P * sec))
    s = ns['screw'](d, P, 1, F, f)
    bad = [k for k in want if abs(s[k] - want[k]) > 1e-9]
    fc.ok(not bad and s['locks'], 'all values recomputed independently, self-locking %s' % (bad or ''))
    got = (round(math.degrees(s['lam']), 3), round(s['T'], 2), round(s['eff'], 3), round(s['T_low'], 2))
    fc.ok(got == (5.197, 655.63, 0.364, 166.56), 'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Carriage', 'Frame', 'Guide rod', 'Screw'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion', 'RigidJointMotion', 'SliderJointMotion']
          and root.motionLinks.count == 1, 'As-built Joints: rigid, revolute, slider; one Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    js, jc = fc.joint('Screw turns'), fc.joint('Carriage slides')
    Rs = fc.rotation(fc.occ('Screw'))
    x0 = fc.position_mm(fc.occ('Carriage'))[0]
    worst, hits = 0.0, []
    for i in range(1, 13):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Screw'), fc.occ('Carriage')])
        if hit:
            hits.append((30 * i, hit))
        if i == 1:
            sgn = 1 if fc.turn_deg(fc.rotation(fc.occ('Screw')), Rs, (1, 0, 0)) > 0 else -1
        dx = fc.position_mm(fc.occ('Carriage'))[0] - x0
        worst = max(worst, abs(dx + 0.25 * i * sgn))
    vj = jc.jointMotion.slideValue * 10
    fc.ok(worst < 1e-4 and abs(abs(dx) - 3.0) < 1e-4 and abs(vj - dx) < 1e-4,
          'one screw turn (%s about +x): the carriage moved %.4f mm, its joint %.4f mm; -0.25 mm per positive 30 deg step '
          '(worst %.2e mm)' % ('positive' if sgn > 0 else 'negative', dx, vj, worst))
    fc.ok(sgn > 0 and dx < 0, 'a positive turn about +x moves the carriage towards -x (a right-hand thread)')
    fc.ok(not hits, 'the screw and the nut never touch at the 12 steps %s' % (hits or ''))
    for i in range(11, -1, -1):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
    fc.refused(ns, 'from 8 to 40', d=6.0)
    fc.refused(ns, 'Thread depth P / 2 = 3.50 mm', P=7.0)
    fc.verdict()
