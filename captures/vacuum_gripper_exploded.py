# Pictures for Chapter 46: the vacuum gripper taken apart: the adapter and its screws up, the beams and their screws
# above the plate, the manifolds and ejectors up, the sleeves up out of the plate, the springs down, the zones'
# plungers and cups further down, the layer hidden. The design is turned to direct modelling first; the script prints
# the items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.60, -1.00, 0.65)      # towards the camera
TOP = {'Adapter': (0, 0, 330)}
for k in range(1, 7):
    TOP['Zone %d' % k] = (0, 0, -170)
IN = {
    'Adapter': [('Cap screw M10x25', (0, 0, 90)), ('Dowel pin', (0, 0, 60))],
    'Frame': [('Beam', (0, 0, 150)), ('Cap screw M8x20', (0, 0, 250)), ('Cap screw M6x14', (0, 0, 300)), ('Manifold', (0, 0, 60)),
              ('Ejector', (0, 0, 110)), ('Sleeve', (0, 0, 45)), ('Spring', (0, 0, -75))],
}
for k in range(1, 7):
    IN['Zone %d' % k] = [('Cup SPB1 80', (0, 0, -70))]
AT = {'Plate': (-520.0, -400.0, -116.0)}     # the plate's point for its balloon: its top near the front edge
ITEMS = ['Adapter plate', 'Cap screw M10x25 2', 'Dowel pin 10x17', 'Beam 1', 'Cap screw M8x20 1', 'Plate', 'Cap screw M6x14 1', 'Sleeve 1',
         'Spring 1', 'Manifold 1', 'Ejector SCPSi 2-14 1', 'Plunger 1-1', 'Cup SPB1 80 1-1']


def shift(o, v):
    m = o.transform2
    t = m.translation
    m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
    o.transform2 = m


def run(_context):
    ns = fc.load('__COMPANION__/scripts/VacuumGripper/VacuumGripper.py')
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
        if o.component.name == 'Pack':
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
