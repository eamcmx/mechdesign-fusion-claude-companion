# Pictures for Chapter 30: AxleHousing exploded. Every part moves along the direction it is put together in: each
# wheel end leaves its tube outwards along the axle, and its parts spread along it (the studs out of the flange; the
# bearing's inner ring and balls and the retaining ring inwards off the seat, the way they are pressed on from the
# shaft's inner end); the bearing's outer ring, the retainer plate and the seal come out of the tube's end; the rear cover and its screws come off backwards, the carrier's studs and nuts
# forwards, the plugs and the breather out of their holes. As in Chapter 29, the joints are deleted and the design is
# turned to direct modelling first. The script prints the items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

END_OUT = 300.0                 # each wheel end out of its tube (mm along y)
VIEW = (0.55, -0.62, 0.56)      # towards the camera: from the front, the left and above
MOVES = {
    'Rear cover': (-120, 0, 0), 'Cover screw': (-170, 0, 0), 'Filler plug': (-150, 0, 0), 'Drain plug': (-150, 0, 0),
    'Breather': (0, 0, 60), 'Carrier stud': (90, 0, 0), 'Carrier nut': (140, 0, 0),
    'Oil seal 74x100x10 R': (0, -70, 0), 'Oil seal 74x100x10 L': (0, 70, 0),
    'Bearing 6213 outer ring R': (0, -100, 0), 'Bearing 6213 outer ring L': (0, 100, 0),
    'Retainer plate R': (0, -125, 0), 'Retainer plate L': (0, 125, 0),
    'Wheel stud M14 R': (0, -150, 0), 'Wheel stud M14 L': (0, 150, 0),
    'Retaining ring R': (0, 95, 0), 'Retaining ring L': (0, -95, 0),
    'Bearing 6213 inner ring R': (0, 78, 0), 'Bearing 6213 inner ring L': (0, -78, 0),
    'Bearing 6213 balls R': (0, 78, 0), 'Bearing 6213 balls L': (0, -78, 0),
}
ITEMS = ['Banjo housing', 'Rear cover', 'Half shaft R', 'Bearing 6213 inner ring R', 'Bearing 6213 balls R',
         'Bearing 6213 outer ring R', 'Retaining ring R', 'Retainer plate R', 'Oil seal 74x100x10 R',
         'Wheel stud M14 R 1', 'Carrier stud M12 1', 'Carrier nut M12 1', 'Cover screw M8x20 1', 'Breather',
         'Filler plug M22', 'Drain plug M22']


def move_of(name):
    for k, v in MOVES.items():
        if name.startswith(k):
            return list(v)
    return [0.0, 0.0, 0.0]


def run(_context):
    ns = fc.load('__COMPANION__/scripts/AxleHousing/AxleHousing.py')
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
        if top.component.name.startswith('Wheel end'):
            s = 1 if top.component.name.endswith('L') else -1
            m = top.transform2
            m.translation = adsk.core.Vector3D.create(0, s * END_OUT * 0.1, 0)
            top.transform2 = m
    moved = 0
    for top in root.occurrences:
        for o in top.childOccurrences:
            o.nativeObject.isGroundToParent = False
            v = move_of(o.component.name)
            if any(v):
                m = o.transform2
                t = m.translation
                m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
                o.transform2 = m
                moved += 1
    print('parts moved: %d' % moved)
    nodes = {o.component.name: world_nodes(o) for o in root.allOccurrences if o.component.bRepBodies.count}
    ends = trails(root, nodes)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    n = math.sqrt(sum(c * c for c in VIEW))
    cam.target = adsk.core.Point3D.create(0, 0, 0)
    cam.eye = adsk.core.Point3D.create(*(300 * c / n for c in VIEW))
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
    cam = vp.camera
    cam.viewExtents = cam.viewExtents * 1.12
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
    """The axle's centre line over all the parts spread along it, 15 mm past each end."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Explosion axis'
    P = lambda x, y, z: adsk.core.Point3D.create(x * 0.1, y * 0.1, z * 0.1)
    on = [p for pts in nodes.values() for p in pts if abs(p[0]) < 60 and abs(p[2]) < 60]
    y0, y1 = min(p[1] for p in on) - 15.0, max(p[1] for p in on) + 15.0
    ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(0, y0, 0), P(0, y1, 0))
    ln.isCenterLine = True
    try:
        sk.arePointsShown = False
    except Exception:
        pass
    return [(0, y0, 0), (0, y1, 0)]
