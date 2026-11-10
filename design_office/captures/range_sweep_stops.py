# Pictures for Chapter 56: the bag gripper's +x end with this chapter's stops, built by RangeSweep.py, seen along the
# hinge from +x: jaw A's lever carried on as a striker, the closing and the opening DYSC on their brackets, the end
# plate, the cylinder's rod and rod end; the cylinders' bodies, jaw B, the press, the bag and the station hidden. PHI is the jaws' opening: 0 the
# closing stop's contact, 45 the opening stop's.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PHI = 45.0
EYE = (1.0, 0.10, 0.06)                      # towards the camera: from +x, a little from +y and above
HIDE = ('Cylinder', 'Press', 'Bag', 'Roller', 'Station')


def keep(o):
    nm = o.component.name
    if any(nm.startswith(h) for h in HIDE) or o.fullPathName.startswith(('Bag', 'Station', 'Press')):
        return False
    if nm.endswith(' B') or nm.endswith(' 2') and nm.startswith(('Stop bracket', 'Shock absorber', 'Lock nut')):
        return False
    b = o.boundingBox
    return b.maxPoint.x * 10 > 300.0                    # the +x end only


def run(_context):
    ns = fc.load('__COMPANION__/design_office/RangeSweep/RangeSweep.py')
    msg, setter = ns['machine']('bag')
    print(msg.splitlines()[0])
    fc.mark()
    setter(('jaws', PHI))
    des = fc.design()
    root = des.rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    shown = 0
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        k = keep(o)
        o.isLightBulbOn = k
        shown += k
    for top in root.occurrences:
        if top.component.bRepBodies.count == 0:
            top.isLightBulbOn = True
    print('shown: %d' % shown)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in EYE))
    t = (49.0, 22.0, -12.0)                             # cm: the end plate's middle, between the two stops
    cam.target = adsk.core.Point3D.create(*t)
    cam.eye = adsk.core.Point3D.create(*(t[i] + 300 * EYE[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = False
    cam.viewExtents = 32.0
    vp.camera = cam
    vp.refresh()
