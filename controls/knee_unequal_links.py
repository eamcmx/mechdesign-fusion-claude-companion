# CONTROL for the prompt line "two equal links from a fixed pivot down to a ram": the ram's pin is drawn 5 mm lower,
# so the lower link is longer than the upper one. The loop still closes and Fusion moves it, but the ram no longer
# follows -2 l cos(alpha), and the toggle straightens at another place: the position check must fail.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/KneeToggle/KneeToggle.py'
OLD = "R=(0.0, -2 * l * math.cos(a)),"
NEW = "R=(0.0, -2 * l * math.cos(a) - 5.0),"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    ju = fc.joint('Upper link turns')
    y0 = fc.position_mm(fc.occ('Ram'))[1]
    worst = 0.0
    for alpha in range(30, 0, -2):
        ju.jointMotion.rotationValue = math.radians(alpha - 30)
        adsk.doEvents()
        dy = fc.position_mm(fc.occ('Ram'))[1] - y0
        want = -2 * 40.0 * math.cos(math.radians(alpha)) + 2 * 40.0 * math.cos(math.radians(30))
        worst = max(worst, abs(dy - want))
    print('  the ram strays from -2 l cos(alpha) by up to %.4f mm' % worst)
    fc.ok(worst < 1e-4, 'the ram at -2 l cos(alpha) at every step')
    print('CONTROL: unequal links; the position check MUST fail')
