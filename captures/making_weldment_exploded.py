# Picture for Chapter 66: jaw A's weldment P13-016 taken apart in the order it is welded, built by RangeSweep.py as Part
# 10 left the bag gripper: the tube and the two fillet welds at its ends stay; the two stub shafts move out along the
# tube's axis, each from its own end; the eight tines move down from the tube's underside. Everything else hidden, the
# joints deleted and the design turned to direct modelling. The script prints the items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.30, -0.80, 0.52)       # towards the camera: from the front, a little above and to the right
OUT, DOWN = 110.0, 150.0         # mm: the stubs out along x, the tines down


def move_of(name, cx):
    """mm along (x, y, z) for each piece of jaw A shown, by its name and the x of its middle; None for the rest."""
    if name in ('Tube A', 'Fillet weld long A', 'Fillet weld short A'):
        return (0.0, 0.0, 0.0)
    if name in ('Stub shaft long A', 'Stub shaft short A'):
        return (OUT if cx > 0 else -OUT, 0.0, 0.0)
    if name.startswith('Tine ') and name.endswith(' A'):
        return (0.0, 0.0, -DOWN)
    return None


ITEMS = ['Tube A', 'Stub shaft long A', 'Stub shaft short A', 'Tine 2 A']


def run(_context):
    rs = fc.load('__COMPANION__/scripts/RangeSweep/RangeSweep.py')
    msg, _ = rs['machine']('bag')
    print(msg.splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints, comp.joints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    shown = []
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        bb = o.boundingBox
        v = move_of(o.component.name, (bb.minPoint.x + bb.maxPoint.x) * 5)
        o.isLightBulbOn = v is not None
        if v is None:
            continue
        shown.append(o)
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
    print('parts shown: %d (%s)' % (len(shown), ', '.join(sorted(o.component.name for o in shown))))
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    nodes = {o.component.name: world_nodes(o) for o in shown}
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
        pts = nodes[name]
        near = sorted(pts, key=lambda p: dot(p, d))[:max(1, len(pts) // 3)]
        cu = sum(uv(p)[0] for p in pts) / len(pts)
        cv = sum(uv(p)[1] for p in pts) / len(pts)
        items[k] = [name, min((uv(p) for p in near), key=lambda q: (q[0] - cu) ** 2 + (q[1] - cv) ** 2)]
    tg = [t.x * 10, t.y * 10, t.z * 10]
    return dict(right=r, up=u, extent=ext, items=items, target=list(uv(tg)), radius=cam.viewExtents * 10)
