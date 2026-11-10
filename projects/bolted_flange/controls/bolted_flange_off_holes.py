# CONTROL for the prompt lines "Bolt i on the bolt circle at angle 360 (i - 1) / N degrees" and "each bolt goes through a
# clearance hole in both plates": the bolts are placed half a pitch (45 degrees) away from their holes. Expected: each
# shank runs through both plates' solid metal, pi * 4^2 * 10 = 502.655 mm^3 per bolt per plate, 8 overlaps in all,
# 4021.24 mm^3.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/bolted_flange/BoltedFlange/BoltedFlange.py'
OLD = "        a = 2 * math.pi * i / N\n        x, y = R * math.cos(a), R * math.sin(a)\n"
NEW = "        a = 2 * math.pi * (i + 0.5) / N\n        x, y = R * math.cos(a), R * math.sin(a)\n"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    shank_plate = [x for x in c if 'Plate' in x[0] or 'Plate' in x[1]]
    print('  %d overlaps, total %.3f mm^3' % (len(c), sum(x[2] for x in c)))
    fc.ok(not c, 'no interference')
    print('CONTROL: bolts off their holes; the check above MUST fail')
