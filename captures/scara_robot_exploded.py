# Pictures for Chapter 38: the robot taken apart by its links along J1's and J2's axes: the base's top flange, the J1
# module plate, the CSG-45 unit's housing, rollers, output flange with its flexspline and case bolts upwards; the
# first arm higher, with the J2 module's plate, unit and bolts above it and its motor below; the second arm highest,
# whole (Chapter 37 took it apart). The joints are deleted and the design turned to direct modelling first; the
# script prints the items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.75, -0.80, 0.45)      # towards the camera
HIDE = ()
TOP = {'First arm': (0, 0, 230), 'Second arm': (0, 0, 420)}
MOVES = {                        # mm, within the groups
    'Base top flange': (0, 0, 25), 'Module plate J1': (0, 0, 55), 'CSG-45-50-2UH housing': (0, 0, 95),
    'Cross rollers 45': (0, 0, 135), 'Output flange 45': (0, 0, 175), 'Flexspline 45': (0, 0, 175),
    'Case bolt M8': (0, 0, 215),
    'Module plate J2': (0, 0, 35), 'CSG-20-50-2UH housing': (0, 0, 65), 'Cross rollers 20': (0, 0, 90),
    'Output flange 20': (0, 0, 115), 'Flexspline 20': (0, 0, 115), 'Case bolt M5': (0, 0, 145),
    'Servo motor J2': (0, 0, -45), 'Motor bolt M5': (0, 0, -60), 'Motor shaft J2': (0, 0, -45),
    'Wave generator 20': (0, 0, -45),
}
ITEMS = ['Base foot', 'Base column', 'Base top flange', 'Module plate J1', 'CSG-45-50-2UH housing', 'Output flange 45',
         'First arm beam', 'Module plate J2', 'CSG-20-50-2UH housing', 'Output flange 20', 'Servo motor J2',
         'Second arm structure', 'Ball screw spline shaft SP20']


def move_of(name, table):
    best = ''
    for k in table:
        if name.startswith(k) and len(k) > len(best):
            best = k
    return list(table[best]) if best else [0.0, 0.0, 0.0]


def shift(o, v):
    m = o.transform2
    t = m.translation
    m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
    o.transform2 = m


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ScaraRobot/ScaraRobot.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    for top in root.occurrences:
        top.isGrounded = False
        v = move_of(top.component.name, TOP)
        if any(v):
            shift(top, v)
    moved = 0
    for o in root.allOccurrences:
        if o.assemblyContext is None:
            continue
        try:
            o.nativeObject.isGroundToParent = False
        except Exception:
            pass
        v = move_of(o.component.name, MOVES)
        if any(v):
            shift(o, v)
            moved += 1
    for o in root.allOccurrences:
        if any(o.component.name.startswith(h) for h in HIDE):
            o.isLightBulbOn = False
    print('parts moved: %d' % moved)
    nodes = {o.component.name: world_nodes(o) for o in root.allOccurrences
             if o.component.bRepBodies.count and o.isLightBulbOn and o.isVisible}
    ends = trails(root, nodes)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in VIEW))
    pts = [p for v in nodes.values() for p in v]
    ctr = [0.5 * (min(p[i] for p in pts) + max(p[i] for p in pts)) for i in range(3)]
    cam.target = adsk.core.Point3D.create(*(c * 0.1 for c in ctr))
    cam.eye = adsk.core.Point3D.create(*((ctr[i] + 3000 * VIEW[i] / n) * 0.1 for i in range(3)))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
    cam = vp.camera
    cam.viewExtents = cam.viewExtents * 1.10
    vp.camera = cam
    vp.refresh()
    print('ANCHORS ' + json.dumps(anchors(vp.camera, nodes, ends)))


def world_nodes(o):
    m = o.transform2
    R = [[m.getCell(i, j) for j in range(4)] for i in range(3)]
    pts = []
    for b in o.component.bRepBodies:
        c = b.meshManager.displayMeshes.bestMesh.nodeCoordinatesAsDouble
        for k in range(0, len(c), 3):
            x, y, z = c[k], c[k + 1], c[k + 2]
            pts.append(tuple(10 * (R[i][0] * x + R[i][1] * y + R[i][2] * z + R[i][3]) for i in range(3)))
    return pts


def anchors(cam, nodes, ends):
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
    every = [uv(p) for pts in nodes.values() for p in pts] + [uv(p) for p in ends]
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


def trails(root, nodes):
    """J1's and J2's axes over their parts."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Explosion axes'
    P = lambda x, y, z: adsk.core.Point3D.create(x * 0.1, y * 0.1, z * 0.1)
    out = []
    for x0 in (0.0, 350.0):
        on = [p for pts in nodes.values() for p in pts if abs(p[0] - x0) < 40 and abs(p[1]) < 40]
        z0, z1 = min(p[2] for p in on) - 15.0, max(p[2] for p in on) + 15.0
        ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(x0, 0, z0), P(x0, 0, z1))
        ln.isCenterLine = True
        out += [(x0, 0, z0), (x0, 0, z1)]
    try:
        sk.arePointsShown = False
    except Exception:
        pass
    return out
