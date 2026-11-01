# CONTROL for the prompt line "a spacer from the gear's hub to the inner ring of the output's second bearing": the
# spacer is made 2 mm short. Expected: the model builds and nothing collides, but the gear can slide 2 mm along its
# shaft; the check of the stack along the output shaft fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/SpurReducer/SpurReducer.py'
OLD = "mp.tube((a, hub, 0), (a, Y_IN, 0), 22.5, s['d_gear'] / 2)"
NEW = "mp.tube((a, hub, 0), (a, Y_IN - 2.0, 0), 22.5, s['d_gear'] / 2)"


def yspan(occ):
    bb = occ.bRepBodies.item(0).boundingBox
    return bb.minPoint.y * 10, bb.maxPoint.y * 10


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    leaves = {x.component.name: x for x in fc.design().rootComponent.allOccurrences if x.component.bRepBodies.count}
    c = fc.interference(list(leaves.values()))
    print('  interference: %s' % (c or 'none'))
    gear, spacer, ring = leaves['Gear z95'], leaves['Spacer'], leaves['Bearing 6206 B inner ring']
    stack = [('shoulder to gear', -25.0 - yspan(gear)[0]), ('gear to spacer', yspan(spacer)[0] - yspan(gear)[1]),
             ('spacer to bearing', yspan(ring)[0] - yspan(spacer)[1])]
    fc.ok(all(abs(v) < 1e-6 for _, v in stack), 'the output shaft\'s stack without play: %s'
          % ', '.join('%s %.3f mm' % x for x in stack))
    print('CONTROL: a spacer 2 mm short; the check above MUST fail')
