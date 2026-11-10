# CONTROL for the prompt line "the guide key on the rod runs in a slot along the housing's bore, to the gland": the
# slot is cut only to 200 mm from the gimbal. Everything fits with the flap up; as the rod goes out, the guide key runs
# into the end of the slot, which only a check made with the rod moved can see.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/flap_actuation/FlapActuator/FlapActuator.py'
OLD = "           mp.box(-L['slot_w'] / 2, L['slot_w'] / 2, L['tube'][0] - 0.01, L['tube'][1], 0.0, L['slot_top']),"
NEW = "           mp.box(-L['slot_w'] / 2, L['slot_w'] / 2, L['tube'][0] - 0.01, 200.0, 0.0, L['slot_top']),"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    c = fc.interference(leaves)
    fc.ok(not c, 'retracted: no interference %s' % (c or ''))
    jr = fc.joint('Rod slides')
    lim = jr.jointMotion.slideLimits
    ext = lim.maximumValue if abs(lim.maximumValue) > abs(lim.minimumValue) else lim.minimumValue
    jr.jointMotion.slideValue = ext
    adsk.doEvents()
    c = fc.interference(leaves)
    fc.ok(not c, 'extended: no interference %s' % (c or ''))
    print('CONTROL: the guide key\'s slot is short; the check with the rod out MUST fail')
