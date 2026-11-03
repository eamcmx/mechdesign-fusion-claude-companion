# Pictures for Chapter 61: the tool changer locked and cut open as Chapter 44 cut it (every body copied with the quarter
# x > 0, y < 0 taken away, the originals hidden), seen close on segment 1 in its window: the cam's 5 deg cone and its
# bottom over the tool plate's groove floor, the segment between the cam and the lip's 15 deg face. The design is
# turned to direct modelling first. The script prints the parts' anchors, inside the window shown, for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

EYE = (0.25, -1.0, 0.12)            # towards the camera: almost square to the cut face y = 0
TARGET = (57.0, 0.0, -86.0)         # mm: between the cam's bottom and the lip
HALF = 24.0                         # mm, the window's half height; its half width is 1.6 times that
ITEMS = ['Master body', 'Segment 1', 'Cam', 'Tool plate ISO 9409-1-160-6-M10', 'Piston with rod']


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ToolChanger/ToolChanger.py')
    print(ns['build'](locked=True).splitlines()[0])
    fc.mark()
    des = fc.design()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    tbm = adsk.fusion.TemporaryBRepManager.get()
    root = des.rootComponent
    nodes = {}
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
            if comp.name in ITEMS:
                nodes.setdefault(comp.name, []).extend(world_nodes(nb, m))
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*[c * 0.1 for c in TARGET])
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] * 0.1 + 60.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = False
    cam.viewExtents = 2 * HALF * 0.1
    vp.camera = cam
    vp.refresh()
    print('ANCHORS ' + json.dumps(anchors(vp.camera, nodes)))


def world_nodes(b, m):
    R = [[m.getCell(i, j) for j in range(4)] for i in range(3)]
    calc = b.meshManager.createMeshCalculator()
    calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.NormalQualityTriangleMesh)
    c = calc.calculate().nodeCoordinatesAsDouble
    return [tuple(10 * (R[i][0] * c[k] + R[i][1] * c[k + 1] + R[i][2] * c[k + 2] + R[i][3]) for i in range(3))
            for k in range(0, len(c), 3)]


def anchors(cam, nodes):
    """Each item's point on the cut face (y = 0) nearest its middle inside the window, in mm across and up the view."""
    e, t, up = cam.eye, cam.target, cam.upVector
    d = [t.x - e.x, t.y - e.y, t.z - e.z]
    nd = math.sqrt(sum(c * c for c in d))
    d = [c / nd for c in d]
    r = [d[1] * up.z - d[2] * up.y, d[2] * up.x - d[0] * up.z, d[0] * up.y - d[1] * up.x]
    nr = math.sqrt(sum(c * c for c in r))
    r = [c / nr for c in r]
    u = [r[1] * d[2] - r[2] * d[1], r[2] * d[0] - r[0] * d[2], r[0] * d[1] - r[1] * d[0]]
    dot = lambda a, b: a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    uv = lambda p: (dot(p, r), dot(p, u))
    ut, vt = uv([t.x * 10, t.y * 10, t.z * 10])
    hw, hh = 1.6 * HALF, HALF
    inside = lambda q: abs(q[0] - ut) < 0.9 * hw and abs(q[1] - vt) < 0.9 * hh
    items = {}
    for k, name in enumerate(ITEMS, 1):
        face = [uv(p) for p in nodes.get(name, []) if abs(p[1]) < 0.05 and inside(uv(p))]
        if not face:
            continue
        cu = sum(q[0] for q in face) / len(face)
        cv = sum(q[1] for q in face) / len(face)
        items[k] = [name, min(face, key=lambda q: (q[0] - cu) ** 2 + (q[1] - cv) ** 2)]
    return dict(right=r, up=u, extent=[ut - hw, ut + hw, vt - hh, vt + hh], items=items, target=[ut, vt],
                radius=cam.viewExtents * 10)
