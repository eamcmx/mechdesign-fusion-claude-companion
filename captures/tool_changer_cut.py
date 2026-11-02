# Pictures for Chapter 44: the tool changer locked, cut open: every body copied with the quarter x > 0, y < 0 taken
# away and the originals hidden, so the piston, its cam, a segment in its window and the tool plate's lip show in
# section. The design is turned to direct modelling first.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

EYE = (1.0, -1.25, 0.35)
TARGET = (0.0, 0.0, -5.5)          # cm
STATE = True


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ToolChanger/ToolChanger.py')
    print(ns['build'](locked=STATE).splitlines()[0])
    fc.mark()
    des = fc.design()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    tbm = adsk.fusion.TemporaryBRepManager.get()
    root = des.rootComponent
    for o in list(root.allOccurrences):
        comp = o.component
        if comp.bRepBodies.count == 0:
            continue
        m = o.transform2.copy()
        p = o.assemblyContext
        while p:
            m.transformBy(p.transform2)
            p = p.assemblyContext
        inv = m.copy()
        inv.invert()
        for b in list(comp.bRepBodies):
            tb = tbm.copy(b)
            box = tbm.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(15.0, -15.0, -5.0),
                                adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0), 30.0, 30.0, 40.0))
            tbm.transform(box, inv)
            tbm.booleanOperation(tb, box, adsk.fusion.BooleanTypes.DifferenceBooleanType)
            nb = comp.bRepBodies.add(tb)
            if b.material:
                nb.material = b.material
            b.isLightBulbOn = False
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 120.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
