# Pictures for Chapter 47: the bag gripper with its jaws open 45 deg and the press up, from the +x end, along the
# jaws' hinges: the tines swung out and down between the rollers, cylinder A in front.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EYE = (1.0, -0.12, 0.06)
TARGET = (0.0, 0.0, -30.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/projects/palletizer/BagGripper/BagGripper.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    ns['set_state'](ns['_last']['groups'], 45.0, e=0.0)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 400.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
