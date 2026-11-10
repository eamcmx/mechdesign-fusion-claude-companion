# CONTROL for the prompt line "the ring's tooth spaces at k 2 pi / Zr + phi_r with phi_r = 0 for Zp odd, pi / Zr for
# Zp even": the ring is given the even-star phase, pi / Zr, though its stars have 33 teeth. Expected: every star's
# tooth meets a ring tooth instead of a space, and the interference check finds the ring overlapping all five stars.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/extras/StarGearbox/StarGearbox.py'
OLD = "    phi_r = 0.0 if zp % 2 else math.pi / zr"
NEW = "    phi_r = math.pi / zr"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps: %s' % (len(c), c))
    fc.ok(not c, 'no interference at rest')
    print('CONTROL: the ring given the wrong phase; the check above MUST fail')
