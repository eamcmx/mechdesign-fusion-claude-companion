# CONTROL for the prompt line "the rocker's end C where the circle of radius b round B meets the circle of radius c
# round D, on the upper branch (the open linkage)": C is taken on the lower branch, the crossed linkage. The same four
# lengths and joints assemble, Fusion turns it without a clash, but the rocker swings to other angles: the check
# against Freudenstein's open branch must fail.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/four_bar/FourBar/FourBar.py'
OLD = "return (bx, by), (bx + x * ux - h * uy, by + x * uy + h * ux)"
NEW = "return (bx, by), (bx + x * ux + h * uy, by + x * uy - h * ux)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    jc = fc.joint('Crank turns')
    Rr = fc.rotation(fc.occ('Rocker'))
    rows, hits = [], []
    for i in range(1, 4):
        jc.jointMotion.rotationValue = math.radians(30.0 * i)
        adsk.doEvents()
        rows.append(round(fc.turn_deg(fc.rotation(fc.occ('Rocker')), Rr, (0, 0, 1)), 3))
        hits += fc.interference()
    print('  the crossed rocker turned %s deg at 30, 60, 90 deg of the crank (open: -8.032, -8.190, -2.290); overlaps %s'
          % (rows, hits))
    fc.ok(abs(rows[0] + 8.032) < 1e-3, 'the rocker where Freudenstein\'s open branch puts it')
    print('CONTROL: the crossed linkage; the motion check MUST fail')
