# CONTROL for the prompt line "each zone's manifold 150 x 60 between the zone's two rows of cups": the manifold made
# 120 mm wide across. Expected: it reaches y = +-60 from the zone's middle, over the sleeves of the cups at +-70
# (their outside at +-55): the interference check fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/palletizer/VacuumGripper/VacuumGripper.py'
OLD = "    mx, my = L['manifold_xy']"
NEW = "    mx, my = 150.0, 120.0"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    c = fc.interference(leaves)
    vol = round(sum(v for a, b, v in c), 1)
    fc.ok(not c, 'no interference (%d clashes, %.1f mm3) %s' % (len(c), vol, sorted(set((a, b) for a, b, v in c))[:3]))
    print('CONTROL: the manifolds made 120 mm wide; the check above MUST fail')
