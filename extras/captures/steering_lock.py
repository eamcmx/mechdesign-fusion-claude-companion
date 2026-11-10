# Pictures for Chapter 29: SteeringRack with its defaults, the pinion turned to 62 mm of rack: full right lock.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def run(_context):
    ns = fc.load('__COMPANION__/extras/SteeringRack/SteeringRack.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    jp = fc.joint('Pinion turns')
    rack = fc.occ('Rack')
    y0 = fc.position_mm(rack)[1]
    jp.jointMotion.rotationValue = math.radians(36.0)
    adsk.doEvents()
    sp = 1 if fc.position_mm(rack)[1] > y0 else -1
    for k in range(2, 13):
        jp.jointMotion.rotationValue = math.radians(min(5.0 * k, 62.0) / 50.0 * 360.0) * sp
        adsk.doEvents()
    jp.jointMotion.rotationValue = math.radians(62.0 / 50.0 * 360.0) * sp
    adsk.doEvents()
    adsk.core.Application.get().activeViewport.fit()
