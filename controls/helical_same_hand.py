# CONTROL for the prompt line "Gear 1 right-handed, gear 2 left-handed: two external helical gears mesh only with
# opposite hands": gear 2 is made right-handed too. Expected: the gears still mesh at the bottom face, where the
# phase puts a tooth in a gap, but gear 2's teeth turn the wrong way as z rises, and by the top face they sit
# 11.8 deg (gear 2's twist twice over) from where they should: the interference check finds the gears overlapping.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/HelicalPair/HelicalPair.py'
OLD = "math.pi - math.pi / z2, -h['th2'], herringbone)"
NEW = "math.pi - math.pi / z2, h['th2'], herringbone)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps: %s' % (len(c), c))
    fc.ok(not c, 'no interference at rest')
    print('CONTROL: both gears right-handed; the check above MUST fail')
