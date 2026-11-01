# CONTROL for the prompt line "the bearing's seat, a shoulder of d_S against which the bearing is pressed, a flange":
# the half shaft is drawn as it first was, with the shoulder inboard of the bearing and the retaining ring outboard,
# between the bearing and the flange. Everything fits and turns, and no part touches another; but the bearing cannot
# be put on: the flange is part of the shaft and stops it from the outer end, the 74 mm shoulder from the inner end.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/AxleHousing/AxleHousing.py'
SWAPS = [
    ("    mp.union(sh, mp.cyl((0, s * (Y_INNER + 59.99), 0), (0, s * (y0 - RING_W), 0), Dz['d_body'] / 2.0))  # the body\n"
     "    mp.union(sh, mp.cyl((0, s * (y0 - RING_W - 0.01), 0), (0, s * (y1 + 0.01), 0), Bb['d'] / 2.0))  # the seat\n"
     "    mp.union(sh, mp.cyl((0, s * y1, 0), (0, s * y_fl, 0), Bb['dS'] / 2.0))                    # the shoulder\n",
     "    mp.union(sh, mp.cyl((0, s * (Y_INNER + 59.99), 0), (0, s * (y0 - 40.0), 0), Dz['d_body'] / 2.0))\n"
     "    mp.union(sh, mp.cyl((0, s * (y0 - 40.01), 0), (0, s * y0, 0), Bb['dS'] / 2.0))\n"
     "    mp.union(sh, mp.cyl((0, s * (y0 - 0.01), 0), (0, s * y_fl, 0), Bb['d'] / 2.0))\n"),
    ("    mp.add_body(o, mp.tube((0, s * (y0 - RING_W), 0), (0, s * y0, 0), Bb['dS'] / 2.0, Bb['d'] / 2.0), 'Retaining ring',\n",
     "    mp.add_body(o, mp.tube((0, s * y1, 0), (0, s * (y1 + 8.0), 0), RET_IN - 0.5, Bb['d'] / 2.0), 'Retaining ring',\n"),
]


def span(entity, s):
    bb = entity.boundingBox
    a, b = s * bb.minPoint.y * 10, s * bb.maxPoint.y * 10
    return min(a, b), max(a, b)


def run(_context):
    ns = fc.load(PATH, SWAPS)
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    c = fc.interference(list(leaves.values()))
    print('  interference at rest: %s' % (c or 'none'))
    ring_in, ring_out = span(leaves['Bearing 6213 inner ring R'].bRepBodies.item(0), 1)
    inboard = 0.0
    for f in leaves['Half shaft R'].bRepBodies.item(0).faces:
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and span(f, 1)[1] <= ring_out + 0.01:
            inboard = max(inboard, f.geometry.radius * 10)
    print('  the shaft inboard of the bearing reaches %.2f mm in radius; the bearing\'s bore is 32.50' % inboard)
    fc.ok(inboard <= 32.5 + 1e-3, 'the bearing goes on from the shaft\'s inner end')
    print('CONTROL: the shoulder is inboard of the bearing; the assembly check MUST fail')
