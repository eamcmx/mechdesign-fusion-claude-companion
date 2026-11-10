# Pictures for Chapter 57: the detailed palletizer robot at q2 = q3 = 0, built by RangeSweep.py through
# PalletizerDetail.py, taken apart into the moving bodies Chapter 57 weighs: each group moved apart as Chapter 43 took the
# concept apart, so that every body stands alone with Part 9's parts in it (the rods' eyes, circlips and pins, the end
# washers). The joints deleted and the design turned to direct modelling first; the script prints one anchor a body for
# the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.80, -1.00, 0.50)      # towards the camera
TOP = {'Turret': (0, 0, 500), 'Lower arm': (0, 0, 500), 'Crank': (0, 900, 500), 'Parallel rod': (0, 650, 900),
       'Upper arm': (0, 0, 1100), 'Wrist': (700, 0, 1250), 'Triangle': (0, -500, 1250),
       'Levelling rod 1': (0, -800, 700), 'Levelling rod 2': (0, -800, 1500),
       'Balancer barrel 1': (750, 0, 500), 'Balancer barrel 2': (750, 0, 500),
       'Balancer rod 1': (750, 0, 650), 'Balancer rod 2': (750, 0, 650)}
# one part a body, the eleven bodies of Chapter 43's form and the base
ITEMS = ['Column', 'Turret plate', 'Lower arm box', 'Crank web', 'Parallel rod body', 'Levelling rod 1 body',
         'Triangle plate', 'Upper arm box', 'Levelling rod 2 body', 'Wrist housing', 'Tool flange', 'Balancer barrel 1']


def shift(o, v):
    m = o.transform2
    t = m.translation
    m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
    o.transform2 = m


def run(_context):
    ns = fc.load('__COMPANION__/design_office/RangeSweep/RangeSweep.py')
    msg, setter = ns['machine']('robot')
    print(msg.splitlines()[0])
    fc.mark()
    setter(('outline', (0.0, 0.0)))
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints, comp.joints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    for o in root.allOccurrences:
        try:
            (o.nativeObject or o).isGroundToParent = False
        except Exception:
            pass
    for o in root.occurrences:
        o.isGrounded = False
    moved = 0
    for o in root.occurrences:
        v = TOP.get(o.component.name)
        if v:
            shift(o, v)
            moved += 1
    print('groups moved: %d of %s' % (moved, sorted(o.component.name for o in root.occurrences)))
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    nodes = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count and o.isLightBulbOn and o.isVisible:
            nodes.setdefault(o.component.name, world_nodes(o))
    missing = [n for n in ITEMS if n not in nodes]
    print('missing items: %s' % missing)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in VIEW))
    pts = [p for v in nodes.values() for p in v]
    ctr = [0.5 * (min(p[i] for p in pts) + max(p[i] for p in pts)) for i in range(3)]
    cam.target = adsk.core.Point3D.create(*(c * 0.1 for c in ctr))
    cam.eye = adsk.core.Point3D.create(*((ctr[i] + 9000 * VIEW[i] / n) * 0.1 for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
    cam = vp.camera
    cam.viewExtents = cam.viewExtents * 1.06
    vp.camera = cam
    vp.refresh()
    print('ANCHORS ' + json.dumps(anchors(vp.camera, nodes)))


def world_nodes(o):
    m = o.transform2
    R = [[m.getCell(i, j) for j in range(4)] for i in range(3)]
    pts = []
    for b in o.component.bRepBodies:
        try:
            c = b.meshManager.displayMeshes.bestMesh.nodeCoordinatesAsDouble
        except RuntimeError:
            calc = b.meshManager.createMeshCalculator()
            calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh)
            c = calc.calculate().nodeCoordinatesAsDouble
        for k in range(0, len(c), 3):
            x, y, z = c[k], c[k + 1], c[k + 2]
            pts.append(tuple(10 * (R[i][0] * x + R[i][1] * y + R[i][2] * z + R[i][3]) for i in range(3)))
    return pts


def anchors(cam, nodes):
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
    every = [uv(p) for pts in nodes.values() for p in pts]
    ext = [min(q[0] for q in every), max(q[0] for q in every), min(q[1] for q in every), max(q[1] for q in every)]
    items = {}
    for k, name in enumerate(ITEMS, 1):
        pts = nodes.get(name)
        if not pts:
            continue
        near = sorted(pts, key=lambda p: dot(p, d))[:max(1, len(pts) // 3)]
        cu = sum(uv(p)[0] for p in pts) / len(pts)
        cv = sum(uv(p)[1] for p in pts) / len(pts)
        items[k] = [name, min((uv(p) for p in near), key=lambda q: (q[0] - cu) ** 2 + (q[1] - cv) ** 2)]
    tg = [t.x * 10, t.y * 10, t.z * 10]
    return dict(right=r, up=u, extent=ext, items=items, target=list(uv(tg)), radius=cam.viewExtents * 10)
