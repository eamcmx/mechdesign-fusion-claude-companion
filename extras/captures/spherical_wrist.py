# Pictures for Chapter 35: SphericalWrist with its defaults, turned to POSE (J4, J5, J6 in deg).
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

POSE = (30.0, 60.0, -40.0)


def run(_context):
    ns = fc.load('__COMPANION__/extras/SphericalWrist/SphericalWrist.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    for j, v in zip(root.asBuiltJoints, POSE):
        j.jointMotion.rotationValue = math.radians(v)
    adsk.doEvents()
    adsk.core.Application.get().activeViewport.fit()
