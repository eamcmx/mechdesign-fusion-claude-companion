# Pictures for Chapter 34: the flap system taken apart at the actuator's station and the right hinge, the wing's
# skins and spar, the flap's panel and the left hinge hidden. The actuator draws down out of the fitting with its
# trunnions out sideways, the fitting comes off the spar with its bolts out aft and its nuts forwards, the horn's pin
# draws out along the span; at the hinge the pin draws out, the bracket drops out of the flap's fork and its bolts come
# out of the rib. The joints are deleted and the design turned to direct modelling first; the script prints the
# items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (-0.55, -0.62, 0.55)     # towards the camera: from the span's left, ahead and above
NEAR = -650.0                   # the right hinge drawn this much nearer the actuator's station
HIDE = ('Upper skin', 'Lower skin', 'Rear spar web', 'Flap panel', 'Hinge bracket L', 'Hinge rib L',
        'Bearing GE 20 ES hinge L', 'Hinge fitting L', 'Hinge pin L', 'Hinge pin nut L', 'Bracket bolt M8x25 L',
        'Bracket nut M8 L')
TOP = {'Actuator': (0, 0, -260)}
MOVES = {                        # mm: X along the span, Y aft, Z up
    'Trunnion R': (90, 0, 0), 'Trunnion L': (-90, 0, 0),
    'Spar fitting': (0, -60, 0), 'Fitting bolt': (0, 70, 0), 'Fitting nut': (0, -130, 0),
    'Bearing GE 20 ES fitting R': (40, -60, 0), 'Bearing GE 20 ES fitting L': (-40, -60, 0),
    'Horn pin nut': (90, 0, 0), 'Horn pin': (-150, 0, 0), 'Spacer R': (45, 0, 0), 'Spacer L': (-45, 0, 0),
    'Hinge pin nut R': (NEAR + 80, 0, 0), 'Hinge pin R': (NEAR - 110, 0, 0), 'Hinge bracket R': (NEAR, 0, -130),
    'Bearing GE 20 ES hinge R': (NEAR, 0, -130), 'Bracket bolt M8x25 R': (NEAR - 90, 0, -130),
    'Bracket nut M8 R': (NEAR + 70, 0, -130), 'Hinge rib R': (NEAR, 0, 0), 'Hinge fitting R': (NEAR, 0, 0),
}
ITEMS = ['Spar fitting', 'Bearing GE 20 ES fitting R', 'Fitting bolt M8x35 1', 'Actuator housing', 'Horn', 'Horn pin',
         'Spacer R', 'Hinge rib R', 'Hinge bracket R', 'Bearing GE 20 ES hinge R', 'Hinge fitting Ro', 'Hinge pin R']


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
    ns = fc.load('__COMPANION__/scripts/FlapSystem/FlapSystem.py')
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
    """The lines the parts draw out along: the actuator down from the gimbal, the trunnions and the horn's pin along
    the span, the hinge's pin along the hinge line."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Explosion lines'
    P = lambda x, y, z: adsk.core.Point3D.create(x * 0.1, y * 0.1, z * 0.1)
    lines = [((0, -420, 114), (0, -420, -146)), ((-170, -420, -146), (170, -420, -146)),
             ((-200, -41.04, 112.76), (120, -41.04, 112.76)), ((1100 + NEAR - 200, 0, 0), (1100 + NEAR + 100, 0, 0))]
    for a, b in lines:
        ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(*a), P(*b))
        ln.isCenterLine = True
    try:
        sk.arePointsShown = False
    except Exception:
        pass
    return [a for a, _ in lines] + [b for _, b in lines]
