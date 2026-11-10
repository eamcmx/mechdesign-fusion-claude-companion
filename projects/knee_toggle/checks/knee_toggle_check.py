# KneeToggle (Chapter 23) against the acceptance criteria of its prompt, measured on what Fusion built: the upper link
# stepped from 30 to 2 deg, 2 deg at a time; at every step the ram's position measured on its body; between every two
# steps the knee's sideways travel over the ram's travel (virtual work, on Fusion's positions) against the formula.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/knee_toggle/KneeToggle/KneeToggle.py'
L = 40.0


def knee_world(alpha_deg):
    """Where the knee is, from the upper link's body: its point at the start transformed by the link's placement."""
    k0 = (L * math.sin(math.radians(30)), -L * math.cos(math.radians(30)))
    p = adsk.core.Point3D.create(k0[0] * 0.1, k0[1] * 0.1, 0)
    p.transformBy(fc.occ('Upper link').transform2)
    return p.x * 10, p.y * 10


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    forces = [100.0 / (2 * math.tan(math.radians(a))) for a in (30, 10, 5, 2, 1)]
    got = [round(100.0 * ns['toggle'](L, a)['ma'], 2) for a in (30, 10, 5, 2, 1)]
    fc.ok(got == [round(f, 2) for f in forces] == [86.6, 283.56, 571.5, 1431.81, 2864.5],
          'F at 30, 10, 5, 2, 1 deg: %s N' % got)
    fc.ok(abs(2 * L * (1 - math.cos(math.radians(30))) - 10.718) < 1e-3, 'the stroke 10.718 mm')
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Frame', 'Lower link', 'Ram', 'Upper link'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 3 + ['SliderJointMotion'],
          'three revolute and one slider As-built Joints closing the loop')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    ju = fc.joint('Upper link turns')
    y0 = fc.position_mm(fc.occ('Ram'))[1]
    worst_y, worst_ma, hits, prev, rows = 0.0, 0.0, [], None, []
    for alpha in range(30, 0, -2):
        ju.jointMotion.rotationValue = math.radians(alpha - 30)
        adsk.doEvents()
        dy = fc.position_mm(fc.occ('Ram'))[1] - y0                        # the ram's travel from the start
        want_dy = -2 * L * math.cos(math.radians(alpha)) + 2 * L * math.cos(math.radians(30))
        worst_y = max(worst_y, abs(dy - want_dy))
        kx, ky = knee_world(alpha)
        if prev:
            measured = (prev[0] - kx) / abs(dy - prev[1])               # knee travel over ram travel
            a1, a2 = math.radians(alpha + 2), math.radians(alpha)
            formula = (L * math.sin(a1) - L * math.sin(a2)) / (2 * L * (math.cos(a2) - math.cos(a1)))
            worst_ma = max(worst_ma, abs(measured - formula) / formula)
            rows.append('%d-%d: %.4f' % (alpha + 2, alpha, measured))
        prev = (kx, dy)
        hit = fc.interference()
        if hit:
            hits.append((alpha, hit))
    print('  knee travel over ram travel, measured: ' + ', '.join(rows))
    fc.ok(worst_y < 1e-4, 'the ram at -2 l cos(alpha) at every step (worst %.2e mm)' % worst_y)
    fc.ok(worst_ma < 1e-6, 'the virtual-work ratio from Fusion\'s positions equals the formula\'s (worst %.1e relative)' % worst_ma)
    fc.ok(not hits, 'no interference at the 15 steps %s' % (hits or ''))
    for alpha in range(4, 32, 2):
        ju.jointMotion.rotationValue = math.radians(alpha - 30)
        adsk.doEvents()
    fc.refused(ns, 'from 5 to 60', alpha0=70.0)
    fc.refused(ns, 'from 1 to 10', over=12.0)
    fc.verdict()
