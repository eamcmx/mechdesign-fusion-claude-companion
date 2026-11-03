# Pictures for Chapter 48: the compliance unit at rest, from the camera EYE (in front, a little above); the view
# taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EYE = (1.0, -0.9, 0.5)
TARGET = (0.0, 0.0, -13.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ComplianceUnit/ComplianceUnit.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 400.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
