# Pictures for Chapter 35: SphericalWrist at rest, J5 = 0: the singular pose, the two rolls in line.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

POSE = (0.0, 0.0, 0.0)


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
