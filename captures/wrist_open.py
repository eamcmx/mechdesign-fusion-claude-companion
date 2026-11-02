# Pictures for Chapter 39: the wrist opened, at rest: the cheeks, their caps and bearings, the tie bars and their
# screws hidden, so that the side gears with their pulleys, the carrier, the output gear, the belts and the motors
# show.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

HIDE = ('Side cheek', 'Bearing cap', 'Tie bar', 'Cap screw ISO 4762 M4x12', 'Cap screw ISO 4762 M8x20',
        'Cap screw ISO 4762 M5x16', 'Dowel')


def run(_context):
    ns = fc.load('__COMPANION__/scripts/WristDifferential/WristDifferential.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    root.isJointsFolderLightBulbOn = False
    for o in root.allOccurrences:
        if any(o.component.name.startswith(h) for h in HIDE):
            o.isLightBulbOn = False
        if o.component.name.startswith('Bearing 6002') and abs(o.boundingBox.minPoint.y * 10) > 50:
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
