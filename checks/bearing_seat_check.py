# BearingSeat (Chapter 8) against the acceptance criteria of its prompt, measured on what Fusion built.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/BearingSeat/BearingSeat.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    L10, L10h, s0 = ns['rating']('6002', 1000.0, 500.0)
    g = ns['geometry']('6002')
    fc.ok(abs(L10 - 1601.613) < 0.001 and abs(L10h - 26693.55) < 0.01 and abs(s0 - 5.70) < 1e-9,
          '6002: L10 %.3f Mrev, L10h %.2f h, s0 %.2f' % (L10, L10h, s0))
    fc.ok(abs(g['dm'] - 23.5) < 1e-9 and abs(g['Dw'] - 5.1) < 1e-9 and g['Z'] == 11 and abs(g['k'] - 0.391489) < 1e-6,
          '6002: dm %.2f, Dw %.2f, Z %d, n_cage/n %.6f' % (g['dm'], g['Dw'], g['Z'], g['k']))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Balls', 'Housing', 'Inner ring', 'Outer ring', 'Shaft'], 'five components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each (the balls are one body)')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 4 and root.motionLinks.count == 1,
          'As-built Joints %d, plain Joints %d, Motion Links %d' % (root.asBuiltJoints.count, root.joints.count, root.motionLinks.count))
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    parts = ('Inner ring', 'Balls', 'Outer ring', 'Housing', 'Shaft')
    R0 = {n: fc.rotation(fc.occ(n)) for n in parts}
    fc.joint('Shaft turns').jointMotion.rotationValue = math.radians(90)
    adsk.doEvents()
    a = {n: fc.turn_deg(fc.rotation(fc.occ(n)), R0[n], (0, 0, 1)) for n in parts}
    fc.ok(abs(a['Inner ring'] - 90) < 1e-6 and abs(a['Shaft'] - 90) < 1e-6, 'shaft at 90: shaft %.3f, inner ring %.3f deg' % (a['Shaft'], a['Inner ring']))
    fc.ok(abs(a['Balls'] - 90 * 0.391489) < 0.001, 'balls %.3f deg (0.391489 * 90 = %.3f)' % (a['Balls'], 90 * 0.391489))
    fc.ok(abs(a['Outer ring']) < 1e-9 and abs(a['Housing']) < 1e-9, 'outer ring %.3f, housing %.3f deg' % (a['Outer ring'], a['Housing']))
    c = fc.interference()
    fc.ok(not c, 'no interference at 90 degrees %s' % (c or ''))
    fc.joint('Shaft turns').jointMotion.rotationValue = 0
    L10, L10h, s0 = ns['rating']('61902', 1000.0, 500.0)
    g = ns['geometry']('61902')
    fc.ok(abs(L10 - 663.054) < 0.001 and round(L10h) == 11051 and g['Z'] == 13 and abs(g['k'] - 0.409302) < 1e-6,
          '61902: L10 %.3f Mrev, L10h %.0f h, Z %d, n_cage/n %.6f' % (L10, L10h, g['Z'], g['k']))
    msg = ns['build'](bearing='61902')
    fc.mark()
    fc.ok(not fc.interference(), '61902 builds with no interference')
    fc.refused(ns, 'L10h = 417 h', Fr=2000.0)
    fc.verdict()
