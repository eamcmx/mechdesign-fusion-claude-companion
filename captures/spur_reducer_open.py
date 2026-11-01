# Pictures for Chapter 27: SpurReducer with the cover and its bolts hidden, the gears and bearings in the base.
# every subassembly); EXPLODE moves the parts apart along the directions given, for an exploded view.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

HIDE = ('Housing cover', 'Filler plug M16') + tuple('Screw M10x45 %d' % k for k in range(1, 9)) + tuple('Dowel 8x24 %d' % k for k in (1, 2))
EXPLODE = False


def run(_context):
    ns = fc.load('__COMPANION__/scripts/SpurReducer/SpurReducer.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    for o in root.allOccurrences:
        if o.component.name in HIDE:
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
