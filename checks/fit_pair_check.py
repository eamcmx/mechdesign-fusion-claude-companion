# FitPair (Chapter 6) against the acceptance criteria of its prompt, measured on what Fusion built.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/FitPair/FitPair.py'


def diameters():
    """The pin's and the hole's diameters, measured on the bodies' cylindrical faces (mm)."""
    pin = fc.occ('Pin').component.bRepBodies.item(0)
    blk = fc.occ('Block').component.bRepBodies.item(0)
    cyl = lambda body: [f for f in body.faces if f.geometry.surfaceType == 1]     # 1 = cylinder
    dp = cyl(pin)[0].geometry.radius * 20
    dh = cyl(blk)[0].geometry.radius * 20
    return dh, dp


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    lim = ns['limits']
    f = lim(20.0, 'g6')
    fc.ok((f['EI'], f['ES'], f['ei'], f['es'], f['c_min'], f['c_max'], f['kind']) == (0, 21, -20, -7, 7, 41, 'clearance'),
          '20 H7/g6: hole 0/+21, pin -20/-7 um; clearance 7 to 41 um, %s' % f['kind'])
    f = lim(15.0, 'k6')
    fc.ok((f['c_min'], f['c_max'], f['kind']) == (-12, 17, 'transition'), '15 H7/k6: from %d to %d um, %s' % (f['c_min'], f['c_max'], f['kind']))
    fc.run_dialog(ns)                                       # 20 H7/g6 at maximum material
    root = fc.design().rootComponent
    dh, dp = diameters()
    fc.ok(abs(dh - 20.000) < 1e-6 and abs(dp - 19.993) < 1e-6, 'H7/g6 at maximum material: hole %.4f, pin %.4f mm' % (dh, dp))
    fc.ok(not fc.interference(), 'no interference')
    fc.ok(type(root.asBuiltJoints.item(0).jointMotion).__name__ == 'RevoluteJointMotion', 'the joint is revolute')
    ns['build'](condition='minimum material')
    fc.mark()
    dh, dp = diameters()
    fc.ok(abs(dh - 20.021) < 1e-6 and abs(dp - 19.980) < 1e-6 and not fc.interference(),
          'H7/g6 at minimum material: hole %.4f, pin %.4f mm, no interference' % (dh, dp))
    ns['build'](shaft='p6')
    fc.mark()
    root = fc.design().rootComponent
    dh, dp = diameters()
    want = math.pi / 4 * (20.035 ** 2 - 20.000 ** 2) * 20
    c = fc.interference()
    vol = sum(x[2] for x in c)
    fc.ok(abs(dh - 20.000) < 1e-6 and abs(dp - 20.035) < 1e-6, 'H7/p6 at maximum material: hole %.4f, pin %.4f mm' % (dh, dp))
    fc.ok(abs(vol - want) < 0.01, 'interference found %.3f mm^3 (pi/4 * (20.035^2 - 20^2) * 20 = %.3f)' % (vol, want))
    fc.ok(type(root.asBuiltJoints.item(0).jointMotion).__name__ == 'RigidJointMotion', 'the joint is rigid')
    ns['build'](shaft='p6', condition='minimum material')
    fc.mark()
    dh, dp = diameters()
    want = math.pi / 4 * (20.022 ** 2 - 20.021 ** 2) * 20
    vol = sum(x[2] for x in fc.interference())
    print('  H7/p6 at minimum material: hole %.4f, pin %.4f mm; interference found %.3f mm^3 (by hand %.3f)' % (dh, dp, vol, want))
    fc.refused(ns, 'outside the ISO 286 table', D=60.0)
    fc.refused(ns, 'not in the table', shaft='r6')
    fc.verdict()
