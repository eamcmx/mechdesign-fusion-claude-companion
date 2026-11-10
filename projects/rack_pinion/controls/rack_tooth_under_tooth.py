# CONTROL for the prompt line "A gap of the rack is centred under the pinion's tooth 0 at x = 0": the rack's teeth are
# shifted half a pitch, so a rack tooth stands under the pinion's tooth 0. Expected: the two teeth meet head on at
# x = 0 and the interference check finds the pinion and the rack overlapping.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/rack_pinion/RackPinion/RackPinion.py'
OLD = "xs = [(k + 0.5) * p - (n_r // 2) * p for k in range(n_r)]"
NEW = "xs = [k * p - (n_r // 2) * p for k in range(n_r)]"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps: %s' % (len(c), c))
    fc.ok(not c, 'no interference at rest')
    print('CONTROL: a rack tooth under the pinion tooth; the check above MUST fail')
