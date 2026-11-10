# CONTROL for the prompt line "q1 = atan2(y, x) - atan2(L2 sin q2, L1 + L2 cos q2)": the minus is written as a plus.
# Expected: the joints are set, Fusion holds them, and the tool is where forward kinematics puts it, but far from the
# target; the check of the distance to the target fails at every target.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/extras/ScaraArm/ScaraArm.py'
OLD = "        q1 = math.degrees(math.atan2(y, x) - b)"
NEW = "        q1 = math.degrees(math.atan2(y, x) + b)"
TARGETS = [(300, 200, 300, 0), (420, -100, 200, -45), (0, -400, 182, 30)]


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    s = ns['scara']()
    j1, j2, j3, j4 = [fc.joint(n) for n in ('J1', 'J2', 'J3', 'J4')]
    tool = fc.occ('Tool')
    worst = 0.0
    for x, y, z, phi in TARGETS:
        for q1, q2, d3, q4 in ns['ik'](s, x, y, z, phi):
            j1.jointMotion.rotationValue = math.radians(q1)
            j2.jointMotion.rotationValue = math.radians(q2)
            j3.jointMotion.slideValue = -d3 / 10.0
            j4.jointMotion.rotationValue = math.radians(q4)
            adsk.doEvents()
            d = math.dist(fc.position_mm(tool), (x, y, z))
            worst = max(worst, d)
            print('  target (%g, %g, %g): asked q1 %.3f q2 %.3f, the tool %.3f mm away' % (x, y, z, q1, q2, d))
    fc.ok(worst <= (s['reach'] + s['L2']) * math.radians(0.05), 'the tool within the rounding of every target (worst '
          '%.3f mm)' % worst)
    print('CONTROL: the inverse kinematics with the wrong sign; the check above MUST fail')
