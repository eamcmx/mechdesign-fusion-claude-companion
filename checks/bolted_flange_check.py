# BoltedFlange (Chapter 8) against the acceptance criteria of its prompt, measured on what Fusion built. The joint's
# numbers are recomputed here from the formulas, independently of the script, and compared with what it reports.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/BoltedFlange/BoltedFlange.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    # by hand: M8, 8.8, grip 20 mm, 20 kN on 4 bolts
    E, d, At, l = 207000.0, 8.0, 36.6, 20.0
    Fp = At * 580.0
    Fi = 0.75 * Fp
    kb = At * E / l
    km = 0.5774 * math.pi * E * d / (2 * math.log(5 * (0.5774 * l + 0.5 * d) / (0.5774 * l + 2.5 * d)))
    C = kb / (kb + km)
    P = 5000.0
    want = dict(Fp=Fp, Fi=Fi, T=0.2 * Fi * d / 1000, kb=kb, km=km, C=C, P=P, Fb=Fi + C * P, Fm=Fi - (1 - C) * P,
                np=Fp / (Fi + C * P), n0=Fi / (P * (1 - C)))
    j = ns['joint']('M8', 4, 10.0, 10.0, 20000.0)
    bad = [k for k in want if abs(j[k] - want[k]) > 1e-6 * max(1.0, abs(want[k]))]
    fc.ok(not bad, 'Fp %.0f, Fi %.0f, T %.2f N.m, kb %.0f, km %.0f, C %.5f, Fb %.0f, Fm %.0f, np %.3f, n0 %.3f %s'
          % (j['Fp'], j['Fi'], j['T'], j['kb'], j['km'], j['C'], j['Fb'], j['Fm'], j['np'], j['n0'], bad or ''))
    fc.ok((round(j['Fp']), round(j['Fi']), round(j['T'], 2), round(j['C'], 5), round(j['np'], 3), round(j['n0'], 3))
          == (21228, 15921, 25.47, 0.18531, 1.26, 3.908), 'the rounded values of the prompt')
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == sorted(['Plate 1', 'Plate 2'] + ['Bolt %d' % i for i in range(1, 5)] + ['Nut %d' % i for i in range(1, 5)]),
          'plates, 4 bolts, 4 nuts: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 9 and
          all(type(x.jointMotion).__name__ == 'RigidJointMotion' for x in root.asBuiltJoints), '9 rigid As-built Joints')
    worst = 0.0
    for i in range(4):
        a = 2 * math.pi * (i + 1 - 1) / 4
        for name in ('Bolt %d' % (i + 1), 'Nut %d' % (i + 1)):
            x, y, z = fc.position_mm(fc.occ(name))
            worst = max(worst, abs(x - 30 * math.cos(a)), abs(y - 30 * math.sin(a)))
    fc.ok(worst < 1e-6, 'each bolt and nut on the 60 mm bolt circle at 360 i / 4 degrees (worst %.2e mm)' % worst)
    c = fc.interference()
    fc.ok(not c, 'no interference %s' % (c or ''))
    fc.refused(ns, 'no room for a socket', N=12)
    fc.refused(ns, 'n0 = Fi / (P (1 - C)) = 0.977', F_ext=80000.0)
    fc.verdict()
