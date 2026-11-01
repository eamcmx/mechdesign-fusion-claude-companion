# SpringPlunger (Chapter 9) against the acceptance criteria of its prompt, measured on what Fusion built. The spring's
# numbers are recomputed here from the formulas of the prompt, independently of the script, and compared.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/SpringPlunger/SpringPlunger.py'


def box_mm(o):
    b = o.component.bRepBodies.item(0).boundingBox
    return [v * 10 for v in (b.minPoint.x, b.minPoint.y, b.minPoint.z, b.maxPoint.x, b.maxPoint.y, b.maxPoint.z)]


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    # by hand, the defaults: d 1.5, D 12, N_t 10, L0 36, L1 30, stroke 8, pin 3
    d, D, Nt, L0, L1, st, dp = 1.5, 12.0, 10, 36.0, 30.0, 8.0, 3.0
    G = 81700.0
    C = D / d
    KB = (4 * C + 2) / (4 * C - 3)
    Na = Nt - 1
    k = d ** 4 * G / (8 * D ** 3 * Na)
    F1, F2, Fs = k * (L0 - L1), k * (L0 - (L1 - st)), k * (L0 - d * Nt)
    Ssy = 0.45 * 2211.0 / d ** 0.145
    tau = lambda F: KB * 8 * F * D / (math.pi * d ** 3)
    want = dict(C=C, KB=KB, Na=Na, Ls=d * Nt, k=k, F1=F1, F2=F2, Fs=Fs, tau2=tau(F2), taus=tau(Fs), Ssy=Ssy,
                n2=Ssy / tau(F2), ns=Ssy / tau(Fs), xi=Fs / F2 - 1, taup=F2 / (2 * math.pi * dp ** 2 / 4))
    s = ns['spring'](d, D, Nt, L0, L1, st, dp)
    bad = [x for x in want if abs(s[x] - want[x]) > 1e-9 * max(1.0, abs(want[x]))]
    fc.ok(not bad, 'C %.3f, K_B %.5f, N_a %d, L_s %.2f, k %.5f, F1 %.2f, F2 %.2f, F_s %.2f, tau2 %.1f, tau_s %.1f, '
          'S_sy %.1f, n2 %.3f, n_s %.3f, xi %.3f, tau_p %.2f %s' % (s['C'], s['KB'], s['Na'], s['Ls'], s['k'], s['F1'],
          s['F2'], s['Fs'], s['tau2'], s['taus'], s['Ssy'], s['n2'], s['ns'], s['xi'], s['taup'], bad or ''))
    got = (round(s['KB'], 5), round(s['k'], 5), round(s['F1'], 2), round(s['F2'], 2), round(s['Fs'], 2), round(s['tau2'], 1),
           round(s['taus'], 1), round(s['Ssy'], 1), round(s['n2'], 3), round(s['ns'], 3), round(s['xi'], 3), round(s['taup'], 2))
    fc.ok(got == (1.17241, 3.32438, 19.95, 46.54, 69.81, 494.0, 741.1, 938.1, 1.899, 1.266, 0.5, 3.29),
          'the rounded values of the prompt %s' % (got,))

    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == sorted(['Base', 'Rod', 'Pin', 'Spring', 'Sleeve', 'Retaining ring']), 'six components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(fc.occ('Base').isGrounded, 'the base is grounded')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds == ['RigidJointMotion'] * 4 + ['SliderJointMotion'],
          'As-built Joints only, four rigid and one slider: %s' % kinds)
    b = box_mm(fc.occ('Spring'))
    fc.ok(abs(b[5] - b[2] - 30.0) < 1e-3 and abs(b[2]) < 1e-3, 'the spring stands on the base, %.3f mm high' % (b[5] - b[2]))
    od = max(b[3] - b[0], b[4] - b[1])
    fc.ok(abs(od - 13.5) < 0.05, 'the spring\'s outside diameter %.3f mm (13.5 wanted)' % od)
    r = box_mm(fc.occ('Retaining ring'))
    fc.ok(abs(r[2] - 38.0) < 1e-6 and abs(r[5] - 38.8) < 1e-6, 'the ring from z = %.3f to %.3f mm, on top of the sleeve' % (r[2], r[5]))
    c = fc.interference()
    fc.ok(not c, 'no interference anywhere at rest %s' % (c or ''))
    j = fc.joint('Sleeve slides')
    lim = j.jointMotion.slideLimits
    fc.ok(lim.isMinimumValueEnabled and lim.isMaximumValueEnabled and abs(lim.minimumValue * 10 + 8.0) < 1e-9
          and abs(lim.maximumValue) < 1e-9, 'the slider is limited to -8 .. 0 mm')
    z0 = fc.position_mm(fc.occ('Sleeve'))[2]
    j.jointMotion.slideValue = -0.8
    import adsk
    adsk.doEvents()
    z1 = fc.position_mm(fc.occ('Sleeve'))[2]
    fc.ok(abs(z1 - z0 + 8.0) < 1e-6, 'driven to the full stroke, the sleeve moved %.3f mm' % (z1 - z0))
    j.jointMotion.slideValue = -1.2
    adsk.doEvents()
    z2 = fc.position_mm(fc.occ('Sleeve'))[2]
    print('  asked for 12 mm: the sleeve stands %.3f mm below rest' % (z0 - z2))
    j.jointMotion.slideValue = 0.0
    adsk.doEvents()
    fc.refused(ns, 'xi = F_s / F2 - 1 = 0.050', stroke=14.0)
    fc.refused(ns, 'C = D / d = 13.3', D=20.0)
    fc.refused(ns, 'does not clear', dr=12)
    fc.verdict()
