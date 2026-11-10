# CONTROL for the prompt line "the right side gear is turned to do the same with spider 0": the right side gear is
# built like the left one, its tooth 0 pointing at spider 0 unturned. With spiders of 15 teeth (odd) each spider shows
# the right side gear a tooth, so the right gear's tooth meets it head on. Expected: the interference check finds the
# right side gear overlapping both spiders.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/extras/CarDifferential/CarDifferential.py'
OLD = "bevel_gear(side_r, gsd, right_phase)"
NEW = "bevel_gear(side_r, gsd, 0.0)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps: %s' % (len(c), c))
    fc.ok(not c, 'no interference at rest')
    print('CONTROL: the right side gear not turned; the check above MUST fail')
