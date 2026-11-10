# Pictures for Chapter 55: the robot's levelling rod 2 at the wrist, built by PalletizerDetail.py, seen close from its
# pin's free end: the eye with its DIN 472 circlip, the pin's end washer and screw, the H plate behind; the robot at
# its pick pose, the camera on the pin H.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.35, -0.85, 0.40)      # from the pin's free side (-y), a little from the front and above
SIZE = 360.0                    # mm across the view


def run(_context):
    ns = fc.load('__COMPANION__/projects/palletizer/PalletizerDetail/PalletizerDetail.py')
    print(ns['build']('robot').splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    pin = [o for o in root.allOccurrences if o.component.name == 'Pin H'][0]
    bb = pin.boundingBox
    c = [(getattr(bb.minPoint, a) + getattr(bb.maxPoint, a)) / 2.0 for a in 'xyz']
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(v * v for v in VIEW))
    cam.target = adsk.core.Point3D.create(*c)
    cam.eye = adsk.core.Point3D.create(*(c[i] + 300 * VIEW[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = False
    cam.viewExtents = SIZE * 0.1
    vp.camera = cam
    vp.refresh()
    print('pin H at %s mm' % [round(v * 10, 1) for v in c])
