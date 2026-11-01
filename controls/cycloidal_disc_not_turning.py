# CONTROL for the prompt line "Motion Links: Input to Disc, N_d x 360 deg of the input for (N_d + 1) x 360 deg of the
# disc, reversed": the + 1 is left out, so the disc turns back exactly as much as the eccentric turns forward and
# keeps its orientation while it orbits. At rest nothing is wrong; as the input turns, the lobes run into the pins.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/CycloidalDrive/CycloidalDrive.py'
OLD = "link(root, j_in, j_d, 360 * n_d, 360 * (n_d + 1), 'Disc rolls')"
NEW = "link(root, j_in, j_d, 360 * n_d, 360 * n_d, 'Disc rolls')"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    fc.ok(not fc.interference(), 'no interference at rest')
    j_in = fc.joint('Input turns')
    hits = []
    for i in range(1, 13):
        j_in.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        hit = [h for h in fc.interference() if 'Disc' in h[:2]]
        if hit:
            hits.append((30 * i, round(sum(h[2] for h in hit), 3)))
    print('  steps with the disc in collision (input deg, mm^3): %s' % hits)
    fc.ok(not hits, 'no interference at any of the 12 steps')
    print('CONTROL: the disc does not turn on its eccentric; the stepped check MUST fail')
