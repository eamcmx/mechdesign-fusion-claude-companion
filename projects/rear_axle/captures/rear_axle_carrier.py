# Pictures for Chapter 29: RearAxleCarrier with its defaults; HIDE lists the components hidden for the view (searched
# in every subassembly).
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

HIDE = ()


def run(_context):
    ns = fc.load('__COMPANION__/projects/rear_axle/RearAxleCarrier/RearAxleCarrier.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    for comp in fc.design().allComponents:                     # a still picture needs no joints; their glyphs go
        for coll in (comp.motionLinks, comp.asBuiltJoints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    for o in root.allOccurrences:
        if any(o.component.name.startswith(h) for h in HIDE):
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
