# Pictures for Chapter 60: the bag gripper's +x end, built and coloured by FatigueSpots.py: the stop brackets and the
# key red, the brackets' bolts and nuts yellow (the stub's seats keep their fits' colours), the end plate see-through so that the long stub, its
# bearing and its circlip show inside it; the brackets' bolts, washers and nuts and the stub's bearing and circlip found
# by their nearness; everything else hidden.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.55, 0.70, 0.45)          # towards the camera: from +x and +y, above
SHOW = ('End plate 1', 'Lever A', 'Stop bracket closed 1', 'Stop bracket open 1', 'Stub shaft long A', 'Key 8x7x25 A',
        'Shock absorber DYSC-25-25-Y1F closed 1', 'Shock absorber DYSC-25-25-Y1F open 1',
        'Lock nut DYSC-25-25-Y1F closed 1', 'Lock nut DYSC-25-25-Y1F open 1')
NEAR_BRACKET = ('Cap screw ISO 4762 M8x40 stop ', 'Nut ISO 4032 M8 stop ', 'Washer ISO 7089 8 stop ')
NEAR_STUB = ('Bearing 6206', 'Circlip DIN 472')


def centre(o):
    """A part's middle in cm as the picture places it: its mesh's points through the occurrence's own transform."""
    m = o.transform2
    R = [[m.getCell(i, j) for j in range(4)] for i in range(3)]
    pts = []
    for b in o.component.bRepBodies:
        calc = b.meshManager.createMeshCalculator()
        calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh)
        c = calc.calculate().nodeCoordinatesAsDouble
        pts += [[R[i][0] * c[k] + R[i][1] * c[k + 1] + R[i][2] * c[k + 2] + R[i][3] for i in range(3)] for k in range(0, len(c), 3)]
    return [sum(p[i] for p in pts) / len(pts) for i in range(3)]


def run(_context):
    ns = fc.load('__COMPANION__/design_office/FatigueSpots/FatigueSpots.py')
    print(ns['build']('all').splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    plate = [o for o in leaves if o.component.name == 'End plate 1'][0]
    by = {o.component.name: o for o in leaves}
    keep = [by[n] for n in SHOW if n in by]
    ctr = centre
    dist = lambda a, b: sum((a[k] - b[k]) ** 2 for k in range(3)) ** 0.5
    brackets = [ctr(by[n]) for n in ('Stop bracket closed 1', 'Stop bracket open 1')]
    stub = ctr(by['Stub shaft long A'])
    for o in leaves:
        n = o.component.name
        if n.startswith(NEAR_BRACKET) and min(dist(ctr(o), b) for b in brackets) < 6.0:        # cm
            keep.append(o)
        if n.startswith(NEAR_STUB) and dist(ctr(o), stub) < 6.0:
            keep.append(o)
    for o in leaves:
        o.isLightBulbOn = o in keep
    for top in root.occurrences:
        if any(s_.fullPathName.startswith(top.fullPathName) for s_ in keep):
            top.isLightBulbOn = True
    for b in plate.bRepBodies:
        b.opacity = 0.25
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    print('shown: %s' % sorted(o.component.name for o in keep))
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    lo = [min(getattr(o.boundingBox.minPoint, a) for o in keep) for a in 'xyz']
    hi = [max(getattr(o.boundingBox.maxPoint, a) for o in keep) for a in 'xyz']
    c = [(lo[i] + hi[i]) / 2 for i in range(3)]
    n = math.sqrt(sum(v * v for v in VIEW))
    cam.target = adsk.core.Point3D.create(*c)
    cam.eye = adsk.core.Point3D.create(*(c[i] + 300 * VIEW[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
