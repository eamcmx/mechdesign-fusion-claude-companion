# CONTROL for the prompt line "The groove is cut in the rod: diameter d2, width m, its upper flank level with the ring's
# top face": the groove is left out. Expected: the ring, inside diameter d2 = 7.6 mm, sits in the 8 mm rod's solid
# metal over its 330 degrees: pi * (4^2 - 3.8^2) * (330 / 360) * 0.8 = 3.594 mm^3.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/SpringPlunger/SpringPlunger.py'
OLD = "    extrude(c, annulus(sk, dr / 2 + 1.0, d2 / 2), m, CUT, body)                    # the groove\n"
NEW = "\n"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps, total %.3f mm^3' % (len(c), sum(x[2] for x in c)))
    fc.ok(not c, 'no interference %s' % (c or ''))
    print('CONTROL: no groove in the rod; the check above MUST fail')
