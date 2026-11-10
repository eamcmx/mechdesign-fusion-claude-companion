# Pictures for Chapter 55: one jaw of the bag gripper in detail taken apart along its axis, built by
# PalletizerDetail.py: jaw A with both end plates, its two 6206s, their circlips, the plugs' welds and the driven end's
# spacer, key, lever, end washer and screw; everything else hidden, the joints deleted and the design turned to direct
# modelling. The faces show Chapter 51's colours and this chapter's classes; the script prints the items' anchors for
# the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.30, -0.80, 0.52)      # towards the camera: from the front (-y), a little from +x, and above
# moves along x, mm, of each part shown (its name without the jaw's letter); the rest of the gripper is hidden
MOVES = {'Tube': 0, 'Tine': 0, 'Fillet weld long': 20, 'Stub shaft long': 70, 'End plate 1': 200, 'Bearing 6206 1': 260,
         'Circlip DIN 472 62x2 1': 300, 'Spacer': 340, 'Key 8x7x25': 380, 'Lever': 420, 'End washer': 470,
         'Cap screw M8x20': 510, 'Fillet weld short': -20, 'Stub shaft short': -70, 'End plate 2': -200,
         'Bearing 6206 4': -560, 'Circlip DIN 471 30x1.5': -620, 'Circlip DIN 472 62x2 4': -680}   # past the plate's outline
LIFT = {'Key 8x7x25': 40.0}     # mm up, so that the key stands clear of the stub


def key_of(name):
    base = name[:-2] if name.endswith((' A', ' B')) else name
    jaw = name[-1] if name.endswith((' A', ' B')) else None
    for k in MOVES:
        if base == k or (k == 'Tine' and base.startswith('Tine ')):
            return k, jaw
    return None, jaw


def run(_context):
    ns = fc.load('__COMPANION__/projects/palletizer/PalletizerDetail/PalletizerDetail.py')
    print(ns['build']('bag').splitlines()[0])
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
        k, jaw = key_of(o.component.name)
        keep = k is not None and jaw in (None, 'A')
        o.isLightBulbOn = keep
        if not keep:
            continue
        shown.append(o)
        try:
            o.nativeObject.isGroundToParent = False
        except Exception:
            pass
        m = o.transform2
        t = m.translation
        m.translation = adsk.core.Vector3D.create(t.x + MOVES[k] * 0.1, t.y, t.z + LIFT.get(k, 0.0) * 0.1)
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


ITEMS = ['End plate 1', 'Tube A', 'Stub shaft long A', 'Fillet weld long A', 'Bearing 6206 1', 'Circlip DIN 472 62x2 1',
         'Spacer A', 'Key 8x7x25 A', 'Lever A', 'End washer A', 'Cap screw M8x20 A', 'End plate 2', 'Stub shaft short A',
         'Fillet weld short A', 'Bearing 6206 4', 'Circlip DIN 471 30x1.5 A', 'Circlip DIN 472 62x2 4', 'Tine 1 A']


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
