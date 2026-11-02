# Pictures for Chapter 40: the integrated wrist from the left (+y) side, the stop cap and the stop disc outside the
# left cheek, at a pitch of POSE[0] and a roll of POSE[1]; the camera set here, the view taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

POSE = (60.0, 30.0)
EYE = (0.95, 1.15, 0.75)
HIDE = ()


def run(_context):
    ns = fc.load('__COMPANION__/scripts/WristIntegration/WristIntegration.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    ns['pose'](*POSE)
    root = fc.design().rootComponent
    root.isJointsFolderLightBulbOn = False
    for o in root.allOccurrences:
        if any(o.component.name.startswith(h) for h in HIDE):
            o.isLightBulbOn = False
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(0, 0, -4.0)
    cam.eye = adsk.core.Point3D.create(*[60.0 * c / n for c in EYE])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
