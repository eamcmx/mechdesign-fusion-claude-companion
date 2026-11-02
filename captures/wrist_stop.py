# Pictures for Chapter 40: the stop at its contact, seen along the pitch axis from +y; the motors, brackets, belts,
# base plate and tie bars hidden; the stop disc's lug on the buffer pad at 128 deg.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

POSE = (128.0, 0.0)
EYE = (0.05, 1.0, 0.02)
HIDE = ('Servo motor', 'Motor bracket', 'Motor output', 'Belt HTD', 'Base plate', 'Tie bar', 'Cap screw ISO 4762 M5x16', 'Dowel', 'Cap screw ISO 4762 M5x12')


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
