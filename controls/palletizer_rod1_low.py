# CONTROL for the prompt line "rod 1 from a pivot on the turntable 180 mm behind the shoulder, at its height": the
# pivot is put 20 mm lower. The first levelling loop is then a four-bar, not a parallelogram: the model builds and
# assembles, but the elbow plate turns as the upper arm moves, and the load tilts. The check of the tilt must fail.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/Palletizer/Palletizer.py'
OLD = "A1 = (-30.0, 700.0) "
NEW = "A1 = (-30.0, 680.0) "


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    br = fc.occ('Bracket')
    R0 = fc.rotation(br)
    j2 = fc.joint('J2 upper arm')
    worst = 0.0
    for v in (10.0, 20.0, 30.0, -10.0, -20.0):
        j2.jointMotion.rotationValue = math.radians(v)
        adsk.doEvents()
        t = fc.turn_deg(fc.rotation(br), R0, (0, 1, 0))
        worst = max(worst, abs(t))
        print('  the shoulder turned %+.0f deg: the load tilted %.3f deg' % (v, t))
    fc.ok(worst < 1e-6, 'the load level in every pose (worst tilt %.3f deg)' % worst)
    print('CONTROL: a levelling loop that is not a parallelogram; the check above MUST fail')
