# Pictures for Chapter 48: the compliance unit taken apart: the top plate up with its screws, the cheeks out along x
# with their rails and screws further out, the spring seats up, the springs and hanger bolts down, the sensor out,
# the carriage down with its blocks out. The design is turned to direct modelling first; the anchors are printed.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.75, -1.00, 0.55)      # towards the camera
TOP = {'Carriage': (0, 0, -200)}
IN = {
    'Body': [('Top plate', (0, 0, 160)), ('Cap screw M10x25', (0, 0, 250)), ('Dowel pin', (0, 0, 220)),
             ('Cap screw M8x30', (0, 0, 300)), ('Cheek 1', (110, 0, 0)), ('Cheek 2', (-110, 0, 0)),
             ('Rail HGR25 1', (60, 0, 0)), ('Rail HGR25 2', (-60, 0, 0))] +
            [('Cap screw M6x20 %d' % k, (170 if k <= 4 else -170, 0, 0)) for k in range(1, 9)] +
            [('Spring seat', (0, 0, 70)), ('Hanger bolt', (0, 0, -330)), ('Spring', (0, 0, -120)),
             ('Sensor M12', (0, 90, 0)), ('Sensor bracket', (0, 60, 60))],
    'Carriage': [('Block HGH25CA 1', (60, 0, 0)), ('Block HGH25CA 2', (60, 0, 0)), ('Block HGH25CA 3', (-60, 0, 0)),
                 ('Block HGH25CA 4', (-60, 0, 0))],
}
AT = {}
ITEMS = ['Top plate', 'Cap screw M10x25 2', 'Dowel pin 10x17', 'Cheek 1', 'Cap screw M8x30 1', 'Rail HGR25 1',
         'Cap screw M6x20 1', 'Spring seat 1', 'Hanger bolt 1', 'Spring 1', 'Sensor M12', 'Sensor bracket', 'Carriage',
         'Block HGH25CA 1']


def shift(o, v):
    m = o.transform2
    t = m.translation
    m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
    o.transform2 = m


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ComplianceUnit/ComplianceUnit.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints):
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
    plan = []
    for o in root.allOccurrences:
        nm = o.component.name
        ctx = o.assemblyContext
        parent = ctx.component.name if ctx else None
        if parent is None:
            if nm in TOP:
                plan.append((o, TOP[nm]))
            continue
        top = o.fullPathName.split('+')[0].split(':')[0]
        for pre, v in IN.get(top, []):
            if nm.startswith(pre):
                plan.append((o, v))
                break
    for o, v in plan:
        shift(o, v)
    print('parts moved: %d' % len(plan))
    for o in root.occurrences:
        if o.component.name in ():
            o.isLightBulbOn = False
    nodes = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count and o.isLightBulbOn and o.isVisible:
            nodes[o.component.name] = world_nodes(o)
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
        c = b.meshManager.displayMeshes.bestMesh.nodeCoordinatesAsDouble
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
        key = name if name in nodes else [k for k in nodes if k.startswith(name + ' (')][0]   # Fusion's ' (1)'
        pts = nodes[key]
        near = sorted(pts, key=lambda p: dot(p, d))[:max(1, len(pts) // 3)]
        cu = sum(uv(p)[0] for p in pts) / len(pts)
        cv = sum(uv(p)[1] for p in pts) / len(pts)
        items[k] = [name, min((uv(p) for p in near), key=lambda q: (q[0] - cu) ** 2 + (q[1] - cv) ** 2)]
        if name in AT:
            items[k] = [name, list(uv(AT[name]))]
    tg = [t.x * 10, t.y * 10, t.z * 10]
    return dict(right=r, up=u, extent=ext, items=items, target=list(uv(tg)), radius=cam.viewExtents * 10)
