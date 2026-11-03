# CONTROL for the prompt line "four hanger bolts at x = +-25, y = +-50": the hangers moved in to x = +-20. Expected:
# the two springs on each side, 44 mm across, now stand 40 mm apart, so they overlap: the interference check fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/ComplianceUnit/ComplianceUnit.py'
OLD = "L = cu.LAY\n"
NEW = "L = dict(cu.LAY, hangers=((20.0, 50.0), (-20.0, 50.0), (-20.0, -50.0), (20.0, -50.0)))\n"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    c = fc.interference(leaves)
    vol = round(sum(v for a, b, v in c), 1)
    fc.ok(not c, 'no interference (%d clashes, %.1f mm3) %s' % (len(c), vol, sorted(set((a, b) for a, b, v in c))[:3]))
    print('CONTROL: the hangers moved in to x = +-20; the check above MUST fail')
