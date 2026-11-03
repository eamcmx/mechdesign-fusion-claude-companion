# Pictures for Chapter 60: the long stub shaft's end of jaw A taken apart, built by FatigueSpots.py (coloured): the stub,
# welded to the jaw's tube (hidden), stays; the lever comes off along the stub's axis furthest, the circlip next, the bearing after it, and
# the key lifts out of its keyseat. Everything else hidden, the joints deleted and the design turned to direct
# modelling. The script prints the items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.35, 0.85, 0.40)
ITEMS = ['Stub shaft long A', 'Bearing', 'Circlip', 'Key 8x7x25 A', 'Lever A']


def run(_context):
    ns = fc.load('__COMPANION__/scripts/FatigueSpots/FatigueSpots.py')
    print(ns['build']('all').splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints, comp.joints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    stub = [o for o in leaves if o.component.name == 'Stub shaft long A'][0]
    sb = stub.boundingBox
    sy, sz = (sb.minPoint.y + sb.maxPoint.y) / 2, (sb.minPoint.z + sb.maxPoint.z) / 2
    out = 1.0 if sb.maxPoint.x > 0 else -1.0                     # the stub's end points out along x

    def near_stub(o):
        b = o.boundingBox
        return abs((b.minPoint.y + b.maxPoint.y) / 2 - sy) < 1.0 and abs((b.minPoint.z + b.maxPoint.z) / 2 - sz) < 1.0 \
            and (b.minPoint.x * out > (sb.minPoint.x if out > 0 else -sb.maxPoint.x) * 1.0 - 0.5)
    pick = {}
    for o in leaves:
        n = o.component.name
        if n in ('Stub shaft long A', 'Key 8x7x25 A', 'Lever A'):
            pick[n] = o
        elif n.startswith('Bearing 6206') and near_stub(o):
            pick['Bearing'] = o
        elif n.startswith('Circlip DIN 472') and near_stub(o):
            pick['Circlip'] = o
    moves = {'Lever A': (out * 220.0, 0, 0), 'Circlip': (out * 140.0, 0, 0), 'Bearing': (out * 80.0, 0, 0),
             'Key 8x7x25 A': (0, 0, 45.0), 'Stub shaft long A': (0, 0, 0)}
    shown = list(pick.values())
    for o in leaves:
        o.isLightBulbOn = o in shown
    for key, o in pick.items():
        v = moves[key]
        try:
            o.nativeObject.isGroundToParent = False
        except Exception:
            pass
        m = o.transform2
        t = m.translation
        m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
        o.transform2 = m
    for top in root.occurrences:
        if any(s.fullPathName.startswith(top.fullPathName) for s in shown):
            top.isLightBulbOn = True
    print('parts shown: %s' % sorted(pick))
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    nodes = {k: world_nodes(o) for k, o in pick.items()}
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in VIEW))
    allp = [p for pts in nodes.values() for p in pts]
    c = [sum(p[i] for p in allp) / len(allp) * 0.1 for i in range(3)]
    cam.target = adsk.core.Point3D.create(*c)
    cam.eye = adsk.core.Point3D.create(*(c[i] + 300 * VIEW[i] / n for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
    cam = vp.camera
    cam.viewExtents = cam.viewExtents * 1.08
    vp.camera = cam
    vp.refresh()
    print('ANCHORS ' + json.dumps(anchors(vp.camera, nodes)))


def world_nodes(o):
    m = o.transform2
    R = [[m.getCell(i, j) for j in range(4)] for i in range(3)]
    pts = []
    for b in o.component.bRepBodies:
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
    tg = [t.x * 10, t.y * 10, t.z * 10]
    ut, vt = uv(tg)
    H = cam.viewExtents * 10 / 2
    Wd = H * 1.6
    every = [uv(p) for pts in nodes.values() for p in pts]
    ext = [min(q[0] for q in every), max(q[0] for q in every), min(q[1] for q in every), max(q[1] for q in every)]
    items = {}
    for k, name in enumerate(ITEMS, 1):
        pts = nodes.get(name, [])
        if not pts:
            continue
        near = sorted(pts, key=lambda p: dot(p, d))[:max(1, len(pts) // 3)]
        cu = sum(uv(p)[0] for p in pts) / len(pts)
        cv = sum(uv(p)[1] for p in pts) / len(pts)
        items[k] = [name, min((uv(p) for p in near), key=lambda q: (q[0] - cu) ** 2 + (q[1] - cv) ** 2)]
    return dict(right=r, up=u, extent=ext, items=items, target=[ut, vt], radius=cam.viewExtents * 10)
