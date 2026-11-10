# CONTROL for the prompt line "The fixed bearing BK25 at x = -880 and the supported BF25 at +880, outside the jaws'
# carriages at their widest": the fixed bearing moved in to x = -860. Expected: with the jaws at 650 mm the -x jaw's
# carriage, which reaches x = -854, runs into the fixed bearing's seat: the range check fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/palletizer/ClampGripper/ClampGripper.py'
OLD = "X_BK, X_BF = -880.0, 880.0               # the bearing units' centres"
NEW = "X_BK, X_BF = -860.0, 880.0               # the bearing units' centres"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    G = ns['_last']['groups']
    gripper = [x for x in root.allOccurrences if x.component.bRepBodies.count and not x.fullPathName.startswith('Pack')]
    hits = []
    for j in range(200, 651, 25):
        ns['set_state'](G, float(j))
        c = fc.interference(gripper)
        if c:
            hits.append((j, [(a, b, v) for a, b, v in c]))
    fc.ok(not hits, 'the jaws from 200 to 650 mm in 25 mm steps: nothing in the way %s' % (hits or ''))
    print('CONTROL: the fixed bearing moved in to x = -860; the check above MUST fail')
