# Pictures for Chapter 44: the tool changer locked, from the camera EYE (below and to the side, the tool plate's
# underside with its ISO pattern in view); the view taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EYE = (1.0, -1.1, -0.55)
TARGET = (0.0, 0.0, -5.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ToolChanger/ToolChanger.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 120.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
