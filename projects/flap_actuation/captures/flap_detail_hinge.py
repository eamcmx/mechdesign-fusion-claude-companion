# Pictures for Chapter 53: one hinge of the flap opened, built by FlapDetail.py: everything hidden but the right
# hinge's bracket, its GE 20 ES and the fitting's two cheeks, the pin drawn out of them along the span and its nut
# off its end, so that the pin's seat, split out and red, shows beside the faces that sit in the cheeks. The joints
# are deleted and the design turned to direct modelling first.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

KEEP = ('Hinge bracket R', 'Bearing GE 20 ES hinge R', 'Hinge fitting Ro', 'Hinge fitting Ri', 'Hinge pin R',
        'Hinge pin nut R')
MOVES = {'Hinge pin R': (-75.0, 0, 0), 'Hinge pin nut R': (40.0, 0, 0)}     # mm along the span
VIEW = (-0.55, -0.80, 0.45)                                               # towards the camera


def run(_context):
    ns = fc.load('__COMPANION__/projects/flap_actuation/FlapDetail/FlapDetail.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints, comp.joints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    for top in root.occurrences:
        top.isGrounded = False
    shown = 0
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        nm = o.component.name
        if nm in KEEP:
            shown += 1
            v = MOVES.get(nm)
            if v:
                m = o.transform2
                t = m.translation
                m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
                o.transform2 = m
        else:
            o.isLightBulbOn = False
    print('shown: %d' % shown)
    h = 1100.0
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in VIEW))
    cam.target = adsk.core.Point3D.create((h - 20.0) * 0.1, 0, 0)
    cam.eye = adsk.core.Point3D.create(*((h - 20.0) * 0.1 + 300 * VIEW[0] / n if i == 0 else 300 * VIEW[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
