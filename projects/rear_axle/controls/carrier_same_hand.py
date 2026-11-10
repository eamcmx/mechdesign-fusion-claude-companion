# CONTROL for the prompt line "the crown wheel's teeth of the opposite hand to the pinion's": the crown wheel is built
# with the pinion's hand. At the mean cone distance the teeth still meet tooth to gap; towards the toe and the heel
# the crown wheel's teeth turn the wrong way and cut into the pinion's.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/rear_axle/RearAxleCarrier/RearAxleCarrier.py'
OLD = "    sc = spiral_member(g['z2'], g['z1'], g, False)"
NEW = "    sc = spiral_member(g['z2'], g['z1'], g, True)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    hit = fc.interference([leaves['Pinion shaft z12'], leaves['Crown wheel z47']])
    print('  the pinion and the crown wheel at rest: %s' % hit)
    fc.ok(not hit, 'no tooth collision at rest')
    print('CONTROL: the two members have the same hand; the clash check MUST fail')
