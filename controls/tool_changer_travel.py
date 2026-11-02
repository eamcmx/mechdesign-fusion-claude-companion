# CONTROL for the prompt line "Free, each segment drawn in by 3 mm, inside the tool plate's bore of 65.25 mm": the
# segments drawn in by 2.5 mm only. Expected: free, the segments still reach 65.5 mm, over the lip's bore, so the tool
# plate cannot come off: the removal test fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/ToolChanger/ToolChanger.py'
OLD = "TRAVEL = 3.0                              # the segments' radial travel, mm"
NEW = "TRAVEL = 2.5                              # the segments' radial travel, mm"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    G = ns['_last']['groups']
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    hits = []
    for dz in (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0):
        ns['set_state'](G, False, dz)
        c = fc.interference(leaves)
        if c:
            hits.append((dz, len(c), round(sum(v for a, b, v in c), 1)))
    fc.ok(not hits, 'free: the tool plate comes off through 45 mm %s' % (hits or ''))
    print('CONTROL: the segments drawn in by 2.5 mm; the check above MUST fail')
