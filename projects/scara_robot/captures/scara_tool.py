# Pictures for Chapter 37: J3 and J4 at the second arm's end, the near wall hidden; HIDE lists the components hidden
# for the view (searched in every subassembly).
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

HIDE = ('Side wall right',)


def run(_context):
    ns = fc.load('__COMPANION__/projects/scara_robot/ScaraToolJoints/ScaraToolJoints.py')
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
