# CONTROL for the prompt line "the drum's cavity clear of the necks' walls": the housing is drawn as AxleHousing was
# before the axle was put together, the front and rear necks' tubes standing inside the drum. The housing alone passes
# all its checks, for nothing is inside it; with the carrier in it, the rear neck's wall cuts through the bearing caps,
# their bolts and the crown wheel, and the interference check fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/rear_axle/RearAxle/RearAxle.py'
HOUSING = '__COMPANION__/projects/rear_axle/AxleHousing/AxleHousing.py'
CUT = "    mp.cut(body, mp.cyl((0, -(DRUM_Y - WALL), 0), (0, DRUM_Y - WALL, 0), DRUM_R - WALL))\n"


def run(_context):
    ns = fc.load(PATH)
    src = open(HOUSING, encoding='utf-8').read()
    if src.count(CUT) != 1:
        raise RuntimeError('control edit not found exactly once')
    exec(compile(src.replace(CUT, ''), HOUSING, 'exec'), ns['HSG'].__dict__)
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    c = fc.interference(list(leaves.values()))
    print('  interference at rest: %s' % (c or 'none'))
    fc.ok(not c, 'no interference among the %d parts at rest' % len(leaves))
    print('CONTROL: the necks stand inside the drum; the interference check MUST fail')
