# Pictures for Chapter 42: the crank turned down to q3 = 60 deg, seen from behind on the crank's side, its block clear of the base's flange (J2 at POSE[0] with the balancer set to it,
# J3 at POSE[1]), from the camera EYE; the view taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

POSE = (20.0, 60.0)
EYE = (-1.0, 1.1, 0.35)
TARGET = (0.0, 0.0, 70.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/projects/palletizer/PalletizerDrives/PalletizerDrives.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    L = ns['_last']
    (j2, s2), (j3, s3) = L['joints'][2], L['joints'][3]
    j2.jointMotion.rotationValue = s2 * math.radians(POSE[0])
    j3.jointMotion.rotationValue = -s3 * math.radians(POSE[1])
    adsk.doEvents()
    ns['set_balancer'](L['cyls'], POSE[0])
    root = fc.design().rootComponent
    root.isJointsFolderLightBulbOn = False
    for j in root.asBuiltJoints:
        j.isLightBulbOn = False
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 600.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
