# CONTROL for the prompt line "a = atan2(s r10, -s r20)": the minus is left out. Expected: the joints are set and
# Fusion holds them, but the flange is turned away from every target; the check of the orientation fails.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/SphericalWrist/SphericalWrist.py'
OLD = "        a = math.degrees(math.atan2(s * R[1][0], -s * R[2][0]))"
NEW = "        a = math.degrees(math.atan2(s * R[1][0], s * R[2][0]))"
TARGETS = [(40, -30, 15), (-70, 20, 100), (10, 80, -45)]


def R_axis(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {'x': [[1, 0, 0], [0, c, -s], [0, s, c]], 'y': [[c, 0, s], [0, 1, 0], [-s, 0, c]],
            'z': [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    js = list(fc.design().rootComponent.asBuiltJoints)
    flange = fc.occ('Flange')
    R0 = fc.rotation(flange)
    worst = 0.0
    for yaw, pit, rol in TARGETS:
        T = mm(mm(R_axis('z', yaw), R_axis('y', pit)), R_axis('x', rol))
        for q in ns['ik'](T, 110.0):
            for j, v in zip(js, q):
                j.jointMotion.rotationValue = math.radians(v)        # all three joints measured +1 by the check
            adsk.doEvents()
            R1 = fc.rotation(flange)
            Rm = [[sum(R1[i][k] * R0[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
            ax = math.degrees(math.acos(max(-1.0, min(1.0, sum(Rm[i][0] * T[i][0] for i in range(3))))))
            worst = max(worst, ax)
            print('  yaw %g pitch %g roll %g: asked (%.3f, %.3f, %.3f), the tool axis %.3f deg from the target'
                  % (yaw, pit, rol, q[0], q[1], q[2], ax))
    fc.ok(worst <= 0.15, 'the tool axis within the rounding of every target (worst %.3f deg)' % worst)
    print('CONTROL: the inverse with a sign left out; the check above MUST fail')
