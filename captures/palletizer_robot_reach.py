# Pictures for Chapter 43: the whole palletizer at full reach (psi = 25 deg), seen from behind and above on the J3 side: the parallel rod beside J3's reducer;
# the view taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import palletizer_design as pd
import adsk.core

ws = pd.workspace()
POSE = (0.0, ws['reach_q2'], ws['reach_q3'])
EYE = (-0.55, 1.0, 0.75)
TARGET = (40.0, 50.0, 120.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/scripts/PalletizerRobot/PalletizerRobot.py')
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
