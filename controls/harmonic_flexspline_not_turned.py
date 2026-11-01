# CONTROL for the prompt line "the flexspline turned as a whole by rho = -phi (z_c - z_f) / z_f": the flexspline is
# bent where the wave generator points but not turned, as if it stood still. At phi = 0 nothing is wrong; as the wave
# generator turns, the long axis brings flexspline teeth onto the circular spline's teeth instead of into its spaces.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/HarmonicDrive/HarmonicDrive.py'
OLD = "    return -phi * (k['z_c'] - k['z_f']) / k['z_f']"
NEW = "    return 0.0"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    fc.ok(not fc.interference(), 'no interference at phi = 0')
    hits = []
    for a in (45, 90):
        ns['turn_to'](a)
        adsk.doEvents()
        hit = [h for h in fc.interference() if 'Flexspline' in h[:2]]
        if hit:
            hits.append((a, round(sum(h[2] for h in hit), 3)))
    print('  angles with the flexspline in collision (wave generator deg, mm^3): %s' % hits)
    fc.ok(not hits, 'no interference at 45 and 90 deg')
    print('CONTROL: the flexspline is bent but not turned; the clash check MUST fail')
