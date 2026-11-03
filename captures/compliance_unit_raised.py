# Pictures for Chapter 48: the compliance unit raised its whole stroke, from the -y side along the rails' faces, the
# top plate's cheek toward the camera hidden so that the blocks, the springs and the sensor show.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EYE = (0.25, -1.0, 0.12)
TARGET = (0.0, 0.0, -13.0)          # cm


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ComplianceUnit/ComplianceUnit.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    G = ns['_last']['groups']
    ns['set_state'](G, ns['L']['stroke'])
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 400.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
