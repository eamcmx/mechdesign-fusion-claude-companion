# CONTROL for the prompt line "Create the Shaft component WHERE IT BELONGS ... Never move it afterwards": the shaft is
# created at the origin and moved to (x, y) right after (occurrence.transform), the way a script without that line does
# it. Its body and the joint are built afterwards, as in the real script. Expected: the later features undo the move,
# the shaft stands at the origin, inside the solid block, and the interference check finds the overlap.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/first_assembly/FirstAssembly/FirstAssembly.py'
OLD = "    shaft = place(root, 'Shaft', x, y, 0.0)\n"
NEW = ("    shaft = place(root, 'Shaft')\n"
       "    mv = adsk.core.Matrix3D.create()\n"
       "    mv.translation = V3(x * CM, y * CM, 0)\n"
       "    shaft.transform = mv\n"
       "    print('  moved the shaft to (%.2f, %.2f) mm' % (shaft.transform2.translation.x * 10, shaft.transform2.translation.y * 10))\n")


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    x, y, z = fc.position_mm(fc.occ('Shaft'))
    print('  after the build the shaft is at (%.2f, %.2f) mm' % (x, y))
    c = fc.interference()
    fc.ok(abs(x - 15) < 1e-6, 'the shaft is at x = 15 mm')
    fc.ok(not c, 'no interference %s' % (c or ''))
    print('CONTROL: the shaft moved after creation; the checks above MUST fail')
