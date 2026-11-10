# CONTROL for the prompt line "O-ring section centre at radius R_c = d1 / 2 + W / 2, height -L + W / 2 (resting on the
# groove bottom)": the O-ring is centred in the groove's depth instead, at -L / 2. Expected: it rises 0.335 mm above
# the housing's face and sinks 0.335 mm into the groove bottom, two overlaps of A(0.335) * 2 pi R_c = 60.144 mm^3 each,
# and the interference no longer measures the squeeze.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/sealed_cover/SealedCover/SealedCover.py'
OLD = "zc=-L + W / 2,"
NEW = "zc=-L / 2,"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps, total %.3f mm^3: %s' % (len(c), sum(x[2] for x in c), c))
    fc.ok(len(c) == 1 and abs(c[0][2] - 162.793) < 1e-3, 'exactly one overlap, the O-ring with the cover, of 162.793 mm^3')
    print('CONTROL: O-ring centred in the groove depth; the check above MUST fail')
