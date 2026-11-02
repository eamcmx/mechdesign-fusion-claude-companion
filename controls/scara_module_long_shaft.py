# CONTROL for the prompt line "the motor's shaft ends inside the wave generator's bore": the shaft is made to reach
# up to the flexspline's diaphragm, as a motor with a standard long shaft would. The module still turns, but the
# shaft's end sits in the flexspline, which turns 50 times slower the other way: the interference check must find it.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/ScaraJointModules/ScaraJointModules.py'
OLD = "    part(inp, 'Motor shaft J%d' % joint, along_z(x0, 0, z_f, g['z_wg'][1] - 1.0, g['U'] / 2))"
NEW = "    part(inp, 'Motor shaft J%d' % joint, along_z(x0, 0, z_f, g['z_h0'] - 1.0, g['U'] / 2))"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    c = fc.interference(leaves)
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    print('CONTROL: the motor shaft reaches the flexspline; the interference check MUST fail')
