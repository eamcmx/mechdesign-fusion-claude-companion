# Pictures for Chapter 47: the bag gripper taken apart: the adapter and its screws up, the beam's screws up out of
# it, the end plates out along x with their bearings, screws and brackets, the cylinders out beyond them, the jaws
# out along y with their tines down and their shafts and levers out, the press plate and its rods down, the bag and
# the station hidden. The design is turned to direct modelling first; the script prints the items' anchors.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.70, -1.00, 0.60)      # towards the camera
TOP = {'Adapter': (0, 0, 330), 'Jaw A': (0, 260, -60), 'Jaw B': (0, -260, -60), 'Cylinder A': (330, 0, 120),
       'Rod A': (330, 120, 120), 'Cylinder B': (-330, 0, 120), 'Rod B': (-330, -120, 120), 'Press': (0, 0, -260)}
IN = {
    'Adapter': [('Cap screw M10x25', (0, 0, 90)), ('Dowel pin', (0, 0, 60))],
    'Frame': [('Cap screw M8x20', (0, 0, 220)), ('Cap screw M6x16', (0, 0, 160)), ('Cylinder DSBC-32-50', (0, 0, -90)),
              ('End block 1', (90, 0, 0)), ('End block 2', (-90, 0, 0)), ('End plate 1', (170, 0, 0)),
              ('End plate 2', (-170, 0, 0)), ('Bearing 6206 1', (230, 0, 0)), ('Bearing 6206 2', (230, 0, 0)),
              ('Bearing 6206 3', (-230, 0, 0)), ('Bearing 6206 4', (-230, 0, 0)), ('Bracket 1', (250, 0, 0)),
              ('Bracket 2', (-250, 0, 0))] +
             [('Cap screw M8x30 %d' % k, (290 if k <= 4 else -290, 0, 0)) for k in range(1, 9)],
    'Jaw A': [('Tine', (0, 0, -110)), ('Lever', (130, 0, 0)), ('Stub shaft long', (70, 0, 0)), ('Stub shaft short', (-70, 0, 0))],
    'Jaw B': [('Tine', (0, 0, -110)), ('Lever', (-130, 0, 0)), ('Stub shaft long', (-70, 0, 0)), ('Stub shaft short', (70, 0, 0))],
}
AT = {'Press plate': (280.0, 120.0, -570.0), 'Adapter plate': (60.0, -75.0, 330.0)}     # the plate's top near its back corner, clear of the front jaw
ITEMS = ['Adapter plate', 'Cap screw M10x25 2', 'Dowel pin 10x17', 'Beam', 'Cap screw M8x20 1', 'End block 1',
         'End plate 1', 'Cap screw M8x30 1', 'Bearing 6206 2', 'Bracket 1', 'Cylinder DSBC-32-50 1', 'Cap screw M6x16 1',
         'Press plate', 'Press rod 1', 'Tube B', 'Stub shaft long B', 'Stub shaft short B', 'Tine 1 B', 'Lever B',
         'Cylinder DSBC-80-125-C A', 'Piston rod A', 'Rod end A']


def shift(o, v):
    m = o.transform2
    t = m.translation
    m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
    o.transform2 = m


def run(_context):
    ns = fc.load('__COMPANION__/scripts/BagGripper/BagGripper.py')
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
        if o.component.name in ('Bag', 'Station'):
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
