# CONTROL for the prompt line "A Motion Link ... NOT reversed (a belt keeps the direction)": the link is reversed, as
# it would be for two gears in mesh. Expected: nothing collides (the pulleys only turn inside the belt), so the
# interference check passes; the motion check must catch it, pulley 2 turning -15 deg per step instead of +15.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/BeltDrive/BeltDrive.py'
OLD = "li.isReversed = False                                         # a belt keeps the direction"
NEW = "li.isReversed = True"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    j1 = fc.joint('Pulley 1 turns')
    R2 = fc.rotation(fc.occ('Pulley 2'))
    hits, turns = [], []
    for i in range(1, 4):
        j1.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        turns.append(round(fc.turn_deg(fc.rotation(fc.occ('Pulley 2')), R2, (0, 0, 1)), 3))
        hits += fc.interference()
    print('  pulley 2 after 30, 60, 90 deg of pulley 1: %s deg; overlaps: %s' % (turns, hits))
    fc.ok(not hits, 'no interference')
    fc.ok(abs(turns[0] - 15) < 1e-3, 'pulley 2 at +15 deg per 30 deg step')
    print('CONTROL: the belt link reversed; only the motion check can catch it')
