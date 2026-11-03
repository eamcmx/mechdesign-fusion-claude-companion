# Pictures for Chapter 50: the +y jaw's driven end detailed by StandardParts.py, closed, from the +x, +y side: the end
# plate, cylinder A and its rod, the cylinders' brackets, the bag and the station hidden, so that the shaft's shoulder,
# the 6206-2RS1, the spacer, the lever's boss, the end washer and its screw show. Taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EYE = (1.0, 0.75, 0.42)
TARGET = (46.0, 30.0, -19.0)        # cm: the driven end, between the bearing and the hub
EXTENT = 26.0                       # cm across the view
HIDE = ('End plate 1', 'Bracket', 'Bag', 'Roller')
HIDE_GROUPS = ('Cylinder A', 'Rod A', 'Bag', 'Station')


def run(_context):
    ns = fc.load('__COMPANION__/scripts/StandardParts/StandardParts.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    for o in root.occurrences:
        if o.component.name in HIDE_GROUPS:
            o.isLightBulbOn = False
    for o in root.allOccurrences:
        if o.component.bRepBodies.count and o.component.name.startswith(HIDE):
            o.isLightBulbOn = False
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 300.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = False
    cam.viewExtents = EXTENT
    vp.camera = cam
    vp.refresh()
