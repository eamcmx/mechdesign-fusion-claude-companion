# CONTROL for the prompt line "a Motion Link from the motor shaft to J1 at n : 1, reversed: with the circular spline
# held, the flexspline turns against the wave generator": the link is not reversed. Expected: the arm turns the same
# way as the motor, +1.8 deg for each 90 deg of the motor, and the check of the arm's turn fails.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/JointDrive/JointDrive.py'
OLD = "    li.isReversed = True "
NEW = "    li.isReversed = False"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    js, arm = fc.joint('Motor'), fc.occ('Arm 1')
    R0 = fc.rotation(arm)
    js.jointMotion.rotationValue = math.radians(90.0)
    adsk.doEvents()
    a = fc.turn_deg(fc.rotation(arm), R0, (0, 0, 1))
    print('  the motor shaft turned 90 deg: the arm %.3f deg' % a)
    fc.ok(abs(a + 1.8) < 1e-6, 'the arm turned -1.8 deg, against the motor')
    print('CONTROL: the gearhead turning the arm the same way as the motor; the check above MUST fail')
