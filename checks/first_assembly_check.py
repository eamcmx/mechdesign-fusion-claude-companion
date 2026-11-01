# FirstAssembly (Chapter 2) against the acceptance criteria of its prompt, measured on what Fusion built.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/FirstAssembly/FirstAssembly.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Block', 'Shaft'], 'two components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    block, shaft = fc.occ('Block'), fc.occ('Shaft')
    x, y, z = fc.position_mm(shaft)
    fc.ok(abs(x - 15) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6, 'the shaft is at (%.3f, %.3f, %.3f) mm' % (x, y, z))
    vs, vb = fc.volume_mm3(shaft), fc.volume_mm3(block)
    ws, wb = math.pi * 5 ** 2 * 50, 60 * 40 * 20 - math.pi * 5.1 ** 2 * 20
    fc.ok(abs(vs - ws) < 0.05, 'shaft volume %.2f mm^3 (pi * 5^2 * 50 = %.2f)' % (vs, ws))
    fc.ok(abs(vb - wb) < 0.05, 'block volume %.2f mm^3 (60 * 40 * 20 - pi * 5.1^2 * 20 = %.2f)' % (vb, wb))
    fc.ok(block.isGrounded, 'the block is grounded')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 1, 'one As-built Joint, no plain Joints')
    c = fc.interference()
    fc.ok(not c, 'no interference %s' % (c or ''))
    j = fc.joint('Shaft turns')
    R0, B0 = fc.rotation(shaft), fc.position_mm(block)
    j.jointMotion.rotationValue = math.radians(90)
    import adsk
    adsk.doEvents()
    a = fc.turn_deg(fc.rotation(shaft), R0, (0, 0, 1))
    fc.ok(abs(abs(a) - 90) < 1e-6 and fc.position_mm(block) == B0, 'joint at 90 deg: the shaft turned %.3f deg about z, the block stayed' % a)
    p = fc.position_mm(shaft)
    fc.ok(abs(p[0] - 15) < 1e-6 and abs(p[1]) < 1e-6, 'the shaft turned in place, still at (%.3f, %.3f) mm' % p[:2])
    j.jointMotion.rotationValue = 0
    fc.refused(ns, 'Only 2.9 mm', x=22.0)
    fc.refused(ns, 'longer than the block', L_s=15.0)
    fc.verdict()
