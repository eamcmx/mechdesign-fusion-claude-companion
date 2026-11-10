# Pictures for Chapter 51: the integrated wrist given its materials and fits by MaterialsFits.py, then taken apart as
# Chapter 40 took it, so that the faces the rules decided show in their colours: the bearing seats red, the located
# and keyed fits green, the dowel holes and key seats black. The joints are deleted and the design turned to direct
# modelling first; the anchors are printed as Chapter 40's are, though Chapter 51 sets no balloons on this view.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.85, -0.70, 0.50)      # towards the camera
OUT_Y = {                        # mm outward along y, by component name (each part's own side)
    'Input drive': 95, 'Belt HTD 350-5M-15': 95, 'Motor output': 95,
    'Side cheek': 175, 'Bearing 6002-2RSH': 200, 'Bearing cap': 230, 'Cap screw ISO 4762 M4x12': 255,
    'Cap screw ISO 4762 M8x20': 200, 'Stop cap': 230, 'Countersunk screw ISO 10642 M4x12': 255, 'Buffer pad': 230,
}
IN_CARRIER = {'Spacer S5': (0, 215, 0), 'Key DIN 6885 A 5x5x8': (0, 260, 0), 'Stop disc': (0, 280, 0),
              'End washer 18x5.5x2.5': (0, 300, 0), 'Cap screw ISO 4762 M5x12': (0, 315, 0),
              'Slip ring stator': (130, 0, -40), 'Slip ring bracket': (130, 0, -40), 'Cap screw ISO 4762 M3x8': (130, 0, -70)}
UP_Z = {'Roll output': 110}      # the group; then inside it, further
IN_ROLL = {'Output bevel gear': 25, 'Key DIN 6885 A 5x5x18': 0, 'Tool flange': 60, 'Retaining ring DIN 471-15, flange': 95,
           'Spacer S4': 10, 'Slip ring rotor': (130, 0, -150)}
DOWN_Z = {'Base plate': -70, 'Cap screw ISO 4762 M5x16': -110, 'Dowel ISO 8734': -100, 'Cap screw ISO 4762 M5x12': -110}
ITEMS = ['Tool flange ISO 9409-1-31.5-4-M5', 'Retaining ring DIN 471-15, flange', 'Roll shaft, bored', 'Output bevel gear',
         'Slip ring rotor', 'Slip ring stator', 'Slip ring bracket', 'Carrier block', 'Trunnion with stop end', 'Spacer S5',
         'Key DIN 6885 A 5x5x8', 'Stop disc', 'Stop cap', 'Buffer pad', 'Countersunk screw ISO 10642 M4x12', 'Side cheek',
         'Bearing 6002-2RSH', 'Side bevel gear with 36-groove pulley']


def pick(name, table):
    best = ''
    for k in table:
        if name.startswith(k) and len(k) > len(best):
            best = k
    return table[best] if best else 0.0


def shift(o, v):
    m = o.transform2
    t = m.translation
    m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
    o.transform2 = m


def side_of(o):
    b = o.boundingBox
    yc = (b.minPoint.y + b.maxPoint.y) / 2
    return 1.0 if yc > 0 else -1.0


def run(_context):
    ns = fc.load('__COMPANION__/design_office/MaterialsFits/MaterialsFits.py')
    print(ns['build']('wrist').splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for comp in des.allComponents:
        for coll in (comp.motionLinks, comp.asBuiltJoints):
            for i in reversed(range(coll.count)):
                coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    plan = []
    for o in root.allOccurrences:
        nm = o.component.name
        top = o.assemblyContext is None
        if top:
            dy = pick(nm, OUT_Y)
            dz = pick(nm, UP_Z)
            plan.append((o, (0.0, side_of(o) * dy, dz)))
        else:
            parent = o.assemblyContext.component.name
            if parent == 'Frame':
                dy = pick(nm, OUT_Y)
                dz = pick(nm, DOWN_Z)
                if dy or dz:
                    plan.append((o, (0.0, side_of(o) * dy, dz)))
            elif parent == 'Carrier':
                v = IN_CARRIER.get(nm)
                if v and (v[1] == 0 or side_of(o) > 0):
                    plan.append((o, v))
            elif parent == 'Roll output':
                dz = pick(nm, IN_ROLL)
                if dz:
                    plan.append((o, tuple(dz) if isinstance(dz, tuple) else (0.0, 0.0, dz)))
    for o in root.occurrences:
        o.isGrounded = False
    for o in root.allOccurrences:
        if o.assemblyContext is not None:
            try:
                o.nativeObject.isGroundToParent = False
            except Exception:
                pass
    for o, v in plan:
        shift(o, v)
    print('parts moved: %d' % len(plan))
    nodes = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count and o.isLightBulbOn and o.isVisible:
            pts = world_nodes(o)
            if o.component.name not in nodes or side_of(o) < 0:     # the near (-y) one of a pair
                nodes[o.component.name] = pts
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
    cam.viewExtents = cam.viewExtents * 1.08
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
    """The pitch axis across the whole explosion, and the roll axis up through the roll output."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Explosion axes'
    P = lambda x, y, z: adsk.core.Point3D.create(x * 0.1, y * 0.1, z * 0.1)
    out = []
    ys = [p[1] for pts in nodes.values() for p in pts if abs(p[0]) < 40 and abs(p[2]) < 40]
    zs = [p[2] for pts in nodes.values() for p in pts if abs(p[0]) < 30 and abs(p[1]) < 30]
    y0, y1 = min(ys) - 10.0, max(ys) + 10.0
    z0, z1 = -30.0, max(zs) + 10.0
    for a, b in (((0, y0, 0), (0, y1, 0)), ((0, 0, z0), (0, 0, z1))):
        ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(*a), P(*b))
        ln.isCenterLine = True
        out += [a, b]
    try:
        sk.arePointsShown = False
    except Exception:
        pass
    return out
