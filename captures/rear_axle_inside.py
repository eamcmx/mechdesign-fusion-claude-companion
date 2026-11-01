# Pictures for Chapter 31: RearAxle with the housing, the carrier's casting and the differential's case hidden, to
# show the half shafts' splines in the side gears; HIDE lists the components hidden for the view (searched
# in every subassembly).
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

HIDE = ('Banjo housing', 'Rear cover', 'Cover screw', 'Carrier stud', 'Carrier nut', 'Breather', 'Filler plug',
        'Drain plug', 'Carrier casting', 'Bearing cap', 'Cap bolt', 'Adjuster', 'Differential case', 'Crown wheel bolt',
        'Spider washer', 'Side gear washer', 'Bearing C1', 'Bearing C2', 'Oil seal', 'Retainer plate',
        'Bearing 6213 outer ring')


def run(_context):
    ns = fc.load('__COMPANION__/scripts/RearAxle/RearAxle.py')
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
