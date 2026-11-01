# CONTROL for the prompt line "the carriage going towards -x when the screw turns positively about +x (a right-hand
# thread)": the Motion Link's direction is flipped. Expected: nothing touches at rest; as soon as the screw turns, the
# carriage moves the wrong way along the thread and the nut's thread runs into the screw's.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/LeadScrewAxis/LeadScrewAxis.py'
OLD = "li.isReversed = sign > 0                                       # towards -x for a positive turn about +x"
NEW = "li.isReversed = sign < 0"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    fc.ok(not fc.interference(), 'no interference at rest')
    js = fc.joint('Screw turns')
    hits = []
    for i in range(1, 7):
        js.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Screw'), fc.occ('Carriage')])
        if hit:
            hits.append((30 * i, round(hit[0][2], 3)))
    print('  steps with the threads in contact (screw deg, mm^3): %s' % hits)
    fc.ok(not hits, 'the screw and the nut never touch')
    print('CONTROL: the carriage driven the wrong way; the stepped check MUST fail')
