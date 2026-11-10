# Pictures for Chapter 46: the vacuum gripper from above, the layer hidden: the beams on the plate, the six manifolds
# with their ejectors, the sleeves of the compensators with their collars.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EYE = (0.55, -0.85, 1.0)
TARGET = (0.0, 0.0, -10.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/projects/palletizer/VacuumGripper/VacuumGripper.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    G = ns['_last']['groups']
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
