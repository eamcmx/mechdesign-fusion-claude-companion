# CONTROL for the prompt line "each tine's foot under the bag to its tip at y = +-40": the feet made 60 mm longer, to
# y = -20. Expected: the two jaws' tines stand in the same planes, so closed their feet overlap by 40 mm in the middle
# under the bag: the interference check fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/BagGripper/BagGripper.py'
OLD = "        mp.union(tn, mp.box(x - t2, x + t2, L['tip_y'], L['leg_y'][1], L['foot_z'][0], L['foot_z'][1]))"
NEW = "        mp.union(tn, mp.box(x - t2, x + t2, -20.0, L['leg_y'][1], L['foot_z'][0], L['foot_z'][1]))"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    c = fc.interference(leaves)
    vol = round(sum(v for a, b, v in c), 1)
    fc.ok(not c, 'no interference (%d clashes, %.1f mm3) %s' % (len(c), vol, sorted(set((a, b) for a, b, v in c))[:3]))
    print('CONTROL: the tines\' feet made to reach y = -20; the check above MUST fail')
