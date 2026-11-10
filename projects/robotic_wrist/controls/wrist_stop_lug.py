# CONTROL for the prompt line "the stop disc's lug on +z at rest, the block on -z": the lug is cut on the disc's
# other side, as a disc keyed half a turn round would put it. The links and the flange are untouched; the lug then
# sits inside the block at rest, and the interference check at rest must fail.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/robotic_wrist/WristIntegration/WristIntegration.py'


def run(_context):
    ns = fc.load(PATH, edits=[("sector(z0, z1, r0, r1, -half, half)", "sector(z0, z1, r0, r1, 180 - half, 180 + half)")])
    print(ns['VERSION'])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    ns['pose'](0.0, 0.0)
    h = fc.interference(leaves)
    fc.ok(not h, 'no interference among the %d parts at rest %s' % (len(leaves), h or ''))
    print('CONTROL: the lug on the wrong side of the disc; the interference check MUST fail')
    fc.verdict()
