# Pictures for Chapter 45: the clamp gripper open at its widest, the pack hidden, from below: the screw's right-hand
# half and left-hand half with a nut in each jaw, the bearing units at its ends, the blocks on the rails.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EYE = (0.35, -0.75, -1.0)
TARGET = (-5.0, 0.0, -20.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/projects/palletizer/ClampGripper/ClampGripper.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    G = ns['_last']['groups']
    ns['set_state'](G, 650.0)
    G['Pack'].isLightBulbOn = False
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 400.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
