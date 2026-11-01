# CONTROL for the prompt line "Each driven gear is turned so that a tooth of its driver sits in one of its gaps on the
# line of centres", broken for the second stage only: Gear 4 is left unturned, its tooth 0 pointing at the cluster's
# tooth 0. Expected: stage 2 (module 2, 20 and 40 teeth, b = 10, backlash 0.2) is then the mirror image of Chapter 12's
# control, so the clash between the cluster and Gear 4 at rest is the same 186.449 mm^3; stage 1 stays clear.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/CompoundTrain/CompoundTrain.py'
OLD = "gear(g4.component, s2, m2, z4, b, math.pi / z4, bore)"
NEW = "gear(g4.component, s2, m2, z4, b, 0.0, bore)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps: %s' % (len(c), c))
    fc.ok(not c, 'no interference at rest')
    print('CONTROL: gear 4 not turned half a tooth; the check above MUST fail')
