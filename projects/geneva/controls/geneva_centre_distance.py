# CONTROL for the prompt line "centre distance C = a / sin(pi / n), so that the pin enters and leaves each slot along
# it": the wheel is put 1 mm further away. Expected: the pin no longer enters the slot along it; set by the true
# Geneva law for the drawn geometry, the parts would still clear, but set by the law of the prompt (worked with the
# prompt's C, as the check does) the pin runs into the wheel during the index.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/geneva/Geneva/Geneva.py'
OLD = "    C = a / math.sin(math.pi / n)\n"
NEW = "    C = a / math.sin(math.pi / n) + 1.0\n"
N, A = 4, 30.0
C = A / math.sin(math.pi / N)
ALPHA = math.pi / 2 - math.pi / N


def wheel_turn(phi):
    theta = math.pi + phi
    if theta < 2 * math.pi - ALPHA:
        return 0.0
    if theta > 2 * math.pi + ALPHA:
        return -2 * math.pi / N
    gamma = lambda x: math.atan2(A * math.sin(x), A * math.cos(x) - C)
    return ((gamma(theta - 2 * math.pi) - gamma(-ALPHA) + math.pi) % (2 * math.pi)) - math.pi


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    jd, jw = fc.joint('Driver turns'), fc.joint('Wheel turns')
    rows = []
    for k in range(1, 37):
        jd.jointMotion.rotationValue = math.radians(10.0 * k)
        adsk.doEvents()
        jw.jointMotion.rotationValue = wheel_turn(math.radians(10.0 * k))
        adsk.doEvents()
        hit = fc.interference([fc.occ('Driver'), fc.occ('Wheel')])
        if hit:
            rows.append((10 * k, round(sum(h[2] for h in hit), 3)))
    print('  steps with the driver in the wheel (driver deg, mm^3): %s' % rows)
    fc.ok(not rows, 'no interference at the 36 steps')
    print('CONTROL: the wheel 1 mm too far; the check MUST fail')
