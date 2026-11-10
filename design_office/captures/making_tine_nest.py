# Picture for Chapter 66: the sixteen tines as Fusion's Arrange nested them on the plate, with the laser's path, built by
# Making.py's tine part; the sixteen occurrences Arrange copied from are hidden, the nested copies shown, seen from
# above in the Design workspace.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion


def run(_context):
    ns = fc.load('__COMPANION__/design_office/Making/Making.py')
    print(ns['build']('tine').splitlines()[0])
    fc.mark()
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeDocument.products.itemByProductType('DesignProductType'))
    root = des.rootComponent
    hidden = 0
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        if o.component.name == 'P13-012 Tine':
            o.isLightBulbOn = False
            hidden += 1
    # the Design workspace: Manufacture draws the setup's stock as a solid block over the parts
    app.userInterface.workspaces.itemById('FusionSolidEnvironment').activate()
    adsk.doEvents()
    # the camera straight down on the nest, fitted to the copies (Fusion's own top view fits the whole plate's envelope)
    cut = [o.bRepBodies.item(0) for o in root.allOccurrences if o.assemblyContext is not None and o.bRepBodies.count]
    lo = [min(b.boundingBox.minPoint.x for b in cut), min(b.boundingBox.minPoint.y for b in cut)]
    hi = [max(b.boundingBox.maxPoint.x for b in cut), max(b.boundingBox.maxPoint.y for b in cut)]
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    vp = app.activeViewport
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    cam.target = adsk.core.Point3D.create(cx, cy, 0.5)
    cam.eye = adsk.core.Point3D.create(cx, cy, 300.0)
    cam.upVector = adsk.core.Vector3D.create(0, 1, 0)
    cam.isFitView = False
    cam.viewExtents = max(hi[0] - lo[0], hi[1] - lo[1]) * 0.9
    vp.camera = cam
    vp.refresh()
    adsk.doEvents()
    print('hid %d originals; the %d nested copies shown, %.1f x %.1f mm' % (hidden, len(cut), (hi[0] - lo[0]) * 10, (hi[1] - lo[1]) * 10))
