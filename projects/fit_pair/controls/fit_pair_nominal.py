# CONTROL for the prompt line "The model is made at the chosen limits, not at the nominal size": the hole and the pin
# are both drawn at the nominal 20 mm, as a CAD model usually is. Expected: the H7/p6 pair at maximum material shows no
# interference at all, where the real parts overlap by 22.010 mm^3: a model at nominal size hides every fit.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/fit_pair/FitPair/FitPair.py'
EDITS = [("    d_hole = D + (f['EI'] if maxmat else f['ES']) * UM\n", "    d_hole = D\n"),
         ("    d_pin = D + (f['es'] if maxmat else f['ei']) * UM\n", "    d_pin = D\n")]


def run(_context):
    ns = fc.load(PATH, EDITS)
    ns['build'](shaft='p6')
    fc.mark()
    vol = sum(x[2] for x in fc.interference())
    want = math.pi / 4 * (20.035 ** 2 - 20.000 ** 2) * 20
    fc.ok(abs(vol - want) < 0.01, 'interference %.3f mm^3 (the real parts: %.3f)' % (vol, want))
    print('CONTROL: hole and pin at the nominal size; the check above MUST fail')
