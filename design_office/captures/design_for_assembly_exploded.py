# Pictures for Chapter 65: the bag gripper's +x end after the fixings, built by DesignForAssembly.py, taken apart and
# seen from above and the front. The beam stays, cut short at x = 200 for the picture; the end plate with its spigot
# moves out along x, the cylinder bracket further; the two M8 x 20 come out of the beam's side walls along their own
# axes; the bracket's two M8 x 30 move out of the end plate towards the beam, the side they go in from. Everything
# else hidden (the jaws, the levers, the cylinders, the stops), the joints deleted and the design turned to direct
# modelling. The script prints the items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.05, -0.55, 0.85)       # towards the camera: from above, and from -y
CUT_X = 200.0                    # mm: the beam and the jaw shown from here to their +x ends


def move_of(name, xmin):
    """mm along (x, y, z) for each part shown, by its name; None for a part not shown."""
    if name == 'Beam':
        return (0.0, 0.0, 0.0)
    if name == 'End plate 1':
        return (120.0, 0.0, 0.0)
    if name == 'Bracket 1':
        return (240.0, 0.0, 0.0)
    if name == 'Cap screw M8x20 spigot 1':
        return (0.0, 70.0, 0.0)
    if name == 'Cap screw M8x20 spigot 2':
        return (0.0, -70.0, 0.0)
    if name in ('Cap screw M8x30 bracket 1', 'Cap screw M8x30 bracket 2'):
        return (40.0, 0.0, 0.0)
    return None


ITEMS = ['Beam', 'End plate 1', 'Cap screw M8x20 spigot 2', 'Bracket 1', 'Cap screw M8x30 bracket 1']


def run(_context):
    ns = fc.load('__COMPANION__/design_office/DesignForAssembly/DesignForAssembly.py')
    print(ns['build']('bag').splitlines()[0])
    fc.mark()
    import mdfc_parts as mp
    des = fc.design()
    root = des.rootComponent
    for name in ('Beam',):                                    # cut short for the picture
        o = [x for x in root.allOccurrences if x.component.name == name][0]
        tb = ns['_world'](o)
        mp.cut(tb, mp.box(-2000.0, CUT_X, -2000.0, 2000.0, -2000.0, 2000.0))
        ns['_replace'](o, tb)
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints, comp.joints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    shown = []
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        v = move_of(o.component.name, o.boundingBox.minPoint.x * 10)
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
    print('parts shown: %d' % len(shown))
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
