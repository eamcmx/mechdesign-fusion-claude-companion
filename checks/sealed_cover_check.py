# SealedCover (Chapter 11) against the acceptance criteria of its prompt, measured on what Fusion built. The seal's
# numbers are recomputed here from the formulas of the prompt, independently of the script, and compared; then the
# interference check must find the squeeze volume, and nothing else.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/SealedCover/SealedCover.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    # by hand, the defaults: W 2.62, d1 45, L 1.95, G 3.5, p 1 MPa, 4 bolts
    W, d1, L, G, p, N = 2.62, 45.0, 1.95, 3.5, 1.0, 4
    c = W - L
    Do = d1 + 2 * W
    r, Rc = W / 2, d1 / 2 + W / 2
    A = r * r * math.acos((r - c) / r) - (r - c) * math.sqrt(2 * r * c - c * c)
    Fp = p * math.pi * Do ** 2 / 4
    want = dict(c=c, squeeze=100 * c / W, Do=Do, Di=Do - 2 * G, fill=100 * math.pi * W * W / 4 / (G * L), Rc=Rc,
                V=A * 2 * math.pi * Rc, Fp=Fp, nc=N * 0.75 * 14.2 * 580 / Fp)
    s = ns['seal'](W, d1, L, G, p, N)
    bad = [k for k in want if abs(s[k] - want[k]) > 1e-9 * max(1.0, abs(want[k]))]
    fc.ok(not bad, 'c %.2f, squeeze %.2f %%, D_o %.2f, D_i %.2f, fill %.2f %%, V %.3f mm^3, F_p %.1f N, n_c %.2f %s'
          % (s['c'], s['squeeze'], s['Do'], s['Di'], s['fill'], s['V'], s['Fp'], s['nc'], bad or ''))
    got = (round(s['c'], 2), round(s['squeeze'], 2), round(s['Do'], 2), round(s['Di'], 2), round(s['fill'], 2),
           round(s['V'], 3), round(s['Fp'], 1), round(s['nc'], 2))
    fc.ok(got == (0.67, 25.57, 50.24, 43.24, 78.99, 162.793, 1982.4, 12.46), 'the rounded values of the prompt %s' % (got,))

    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == sorted(['Housing', 'Cover', 'O-ring'] + ['Bolt %d' % i for i in range(1, 5)]),
          'housing, cover, O-ring and 4 bolts: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(fc.occ('Housing').isGrounded, 'the housing is grounded')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 6 and
          all(type(j.jointMotion).__name__ == 'RigidJointMotion' for j in root.asBuiltJoints), '6 rigid As-built Joints')
    torus = 2 * math.pi ** 2 * Rc * r * r
    v = fc.volume_mm3(fc.occ('O-ring'))
    fc.ok(abs(v - torus) < 1e-3, 'the O-ring at its free size: %.3f mm^3 (2 pi^2 R_c r^2 = %.3f)' % (v, torus))
    worst = 0.0
    for i in range(4):
        a = 2 * math.pi * i / 4
        x, y, z = fc.position_mm(fc.occ('Bolt %d' % (i + 1)))
        worst = max(worst, abs(x - 31.5 * math.cos(a)), abs(y - 31.5 * math.sin(a)))
    fc.ok(worst < 1e-6, 'each bolt on the 63 mm bolt circle at 360 (i - 1) / 4 degrees (worst %.2e mm)' % worst)
    c = fc.interference()
    pair = len(c) == 1 and sorted(c[0][:2]) == ['Cover', 'O-ring']
    fc.ok(pair and abs(c[0][2] - want['V']) < 1e-3,
          'exactly one overlap, the O-ring with the cover, of %.3f mm^3: %s' % (want['V'], c))
    fc.refused(ns, 'Squeeze 35.11 percent', L=1.70)
    fc.refused(ns, 'Gland fill 92.16 percent', G=3.0)
    fc.refused(ns, 'n_c = N F_i / F_p = 1.78', p=7.0)
    fc.verdict()
