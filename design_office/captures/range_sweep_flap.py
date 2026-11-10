# Pictures for Chapter 56: the flap at its full 40 deg, built by FlapDetail.py, where the sweep finds its closest
# approach: the rod end SI 20 ES 0.86 mm from the horn's fork. Only the horn, its pin, the rod end and the piston rod
# are shown, seen along the span from the rod end's side; the camera on the rod end.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

DELTA = 40.0
KEEP = ('Horn', 'Horn pin', 'Rod end SI 20 ES', 'Piston rod')
EYE = (1.0, 0.0, 0.0)            # along the span, towards the camera


def run(_context):
    ns = fc.load('__COMPANION__/projects/flap_actuation/FlapDetail/FlapDetail.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    ns['pose'](DELTA)
    adsk.doEvents()
    des = fc.design()
    root = des.rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    rod_end = None
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        k = o.component.name in KEEP
        o.isLightBulbOn = k
        if o.component.name == 'Rod end SI 20 ES':
            rod_end = o
    for top in root.occurrences:
        top.isLightBulbOn = True
    b = rod_end.boundingBox
    t = [(getattr(b.minPoint, a) + getattr(b.maxPoint, a)) / 2.0 for a in 'xyz']
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    cam.target = adsk.core.Point3D.create(*t)
    cam.eye = adsk.core.Point3D.create(*(t[i] + 300 * EYE[i] for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = False
    cam.viewExtents = 9.0
    vp.camera = cam
    vp.refresh()
    print('rod end centre (mm): %.1f %.1f %.1f' % tuple(10 * c for c in t))
