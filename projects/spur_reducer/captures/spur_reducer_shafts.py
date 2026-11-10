# Pictures for Chapter 27: the two shaft assemblies of SpurReducer, the housing hidden.
# every subassembly); EXPLODE moves the parts apart along the directions given, for an exploded view.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

HIDE = ('Housing',)
EXPLODE = False


def run(_context):
    ns = fc.load('__COMPANION__/projects/spur_reducer/SpurReducer/SpurReducer.py')
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
