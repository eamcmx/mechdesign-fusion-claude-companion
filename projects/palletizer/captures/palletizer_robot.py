# Pictures for Chapter 43: the whole palletizer at the standard cycle's low point (J1 at POSE[0]), from the camera EYE;
# the view taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import palletizer_design as pd
import adsk.core

lo = pd.waypoints()['lo']
POSE = (0.0, lo['q2'], lo['q3'])
EYE = (1.0, -1.35, 0.55)
TARGET = (60.0, 0.0, 140.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/projects/palletizer/PalletizerRobot/PalletizerRobot.py')
    print(ns['build'](*POSE).splitlines()[0])
    fc.mark()
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 900.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
