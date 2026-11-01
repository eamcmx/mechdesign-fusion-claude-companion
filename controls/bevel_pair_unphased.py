# CONTROL for the prompt line "The pinion's tooth 0 points at the gear (world +X); the gear is turned half a tooth so
# that a gap meets it": the gear is left unturned, its tooth 0 pointing back at the pinion's. Expected: the two heels'
# teeth meet head on and the interference check finds the pinion and the gear overlapping.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/BevelPair/BevelPair.py'
OLD = "bevel_gear(gear, g2, math.pi / z2)"
NEW = "bevel_gear(gear, g2, 0.0)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps: %s' % (len(c), c))
    fc.ok(not c, 'no interference at rest')
    print('CONTROL: the gear not turned half a tooth; the check above MUST fail')
