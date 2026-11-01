# CONTROL for the prompt line "One Motion Link: 360 deg of the screw for one lead of the carriage, in the direction
# that keeps the nut's groove on the screw's": the link is reversed, so the nut runs one lead the wrong way per turn.
# At rest nothing is wrong; as the screw turns, the balls in the nut leave the screw's groove and cut into its land.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/BallScrewAxis/BallScrewAxis.py'
OLD = "    li.isReversed = sign > 0                                       # towards -x for a positive turn about +x"
NEW = "    li.isReversed = sign < 0"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    fc.ok(not fc.interference(), 'no interference at rest')
    js = fc.joint('Screw turns')
    hits = []
    for i in (1, 2, 3):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        hit = [h for h in fc.interference() if 'Balls' in h[:2]]
        if hit:
            hits.append((30 * i, round(sum(h[2] for h in hit), 3)))
    print('  steps with the balls in collision (screw deg, mm^3): %s' % hits)
    fc.ok(not hits, 'no interference at 30, 60 and 90 deg')
    print('CONTROL: the carriage runs the wrong way; the clash check MUST fail')
