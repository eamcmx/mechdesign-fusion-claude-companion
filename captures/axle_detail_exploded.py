# Pictures for Chapter 54: the rear axle's carrier in detail taken apart as Chapter 29 took it, built by AxleDetail.py
# inside Chapter 31's axle: the housing and the wheel ends hidden, the joints deleted and the design turned to direct
# modelling, the differential drawn out and every part along the way it goes in. The faces show Chapter 51's colours
# and this chapter's classes; the script prints the items' anchors for the balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

DIFF_OUT = -380.0               # the differential, towards the rear (mm along x)
VIEW = (0.62, 0.55, 0.56)       # towards the camera: from the front, the right and above
# moves of the parts, relative to their subassembly, mm (x, y, z); a spider's move is along its arm
MOVES = {
    'Carrier casting': (0, 0, 330), 'Pinion seal': (105, 0, 330),
    'Bearing A 32309 cup': (-110, 0, 0), 'Bearing B 32308 cup': (120, 0, 0),
    'Pinion shaft': (-150, 0, 0), 'Pinion depth shim': (-112, 0, 0), 'Bearing A 32309 cone': (-62, 0, 0),
    'Bearing B 32308 cone': (62, 0, 0), 'Companion flange': (125, 0, 0), 'Washer': (172, 0, 0), 'Pinion nut': (200, 0, 0),
    'Bearing cap C1': (-230, 0, 0), 'Bearing cap C2': (-230, 0, 0), 'Cap bolt': (-300, 0, 0),
    'Adjuster C1': (0, -115, 0), 'Adjuster C2': (0, 115, 0),
    'Bearing C1 32213 cup': (DIFF_OUT, -60, 0), 'Bearing C2 32213 cup': (DIFF_OUT, 60, 0),
    'Crown wheel z47': (0, 190, 0), 'Crown wheel bolt': (0, -90, 0),
    'Bearing C1 32213 cone': (0, -150, 0), 'Bearing C2 32213 cone': (0, 150, 0),
    'Side gear L': (0, 110, 0), 'Side gear R': (0, -110, 0),
    'Side gear washer L': (0, 85, 0), 'Side gear washer R': (0, -85, 0),
}
ARM = {'Spider ': 62.0, 'Spider washer ': 42.0}
ARMS = {1: (0, 0, 1), 2: (1, 0, 0), 3: (0, 0, -1), 4: (-1, 0, 0)}


def move_of(name):
    for k, v in MOVES.items():
        if name.startswith(k):
            return list(v)
    for k, d in ARM.items():
        if name.startswith(k) and name[len(k):].strip().isdigit():
            a = ARMS[int(name[len(k):])]
            return [d * c for c in a]
    return [0.0, 0.0, 0.0]


def run(_context):
    ns = fc.load('__COMPANION__/scripts/AxleDetail/AxleDetail.py')
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
        if top.component.name == 'Housing' or top.component.name.startswith('Wheel end'):
            top.isLightBulbOn = False
        if top.component.name == 'Differential assembly':
            m = top.transform2
            m.translation = adsk.core.Vector3D.create(DIFF_OUT * 0.1, 0, 0)
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
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    nodes = {o.component.name: world_nodes(o) for o in root.allOccurrences if o.component.bRepBodies.count
             and not o.fullPathName.startswith(('Housing', 'Wheel end'))}
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
    cam.viewExtents = cam.viewExtents * 1.15                       # a margin, so that nothing touches the edge
    vp.camera = cam
    vp.refresh()
    print('ANCHORS ' + json.dumps(anchors(vp.camera, nodes, ends)))


ITEMS = ['Carrier casting', 'Bearing cap C1', 'Adjuster C1', 'Pinion shaft z12', 'Crown wheel z47', 'Differential case',
         'Cross', 'Spider 1', 'Side gear L', 'Spider washer 1', 'Side gear washer L', 'Pinion depth shim', 'Spacer',
         'Companion flange', 'Washer', 'Pinion nut M30', 'Bearing A 32309 cone', 'Bearing A 32309 cup',
         'Bearing B 32308 cone', 'Bearing B 32308 cup', 'Bearing C2 32213 cone', 'Bearing C2 32213 cup',
         'Pinion seal 60x85x10', 'Crown wheel bolt M12x35 3', 'Cap bolt M12x90 C1 upper']


def world_nodes(o):
    m = o.transform2
    R = [[m.getCell(i, j) for j in range(4)] for i in range(3)]
    pts = []
    for b in o.component.bRepBodies:
        try:
            c = b.meshManager.displayMeshes.bestMesh.nodeCoordinatesAsDouble
        except RuntimeError:          # a body Fusion has not meshed for display (after the detail's cuts): mesh it
            calc = b.meshManager.createMeshCalculator()
            calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh)
            c = calc.calculate().nodeCoordinatesAsDouble
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
    """Centre lines: the pinion's axis over its parts, and the axle's over the differential's, 15 mm past each end."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Explosion axes'
    P = lambda x, y, z: adsk.core.Point3D.create(x * 0.1, y * 0.1, z * 0.1)
    ends = []
    on = [p for pts in nodes.values() for p in pts if abs(p[1]) < 60 and abs(p[2]) < 60 and p[0] > -280]
    x0, x1 = min(p[0] for p in on) - 15.0, max(p[0] for p in on) + 15.0
    ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(x0, 0, 0), P(x1, 0, 0))
    ln.isCenterLine = True
    ends += [(x0, 0, 0), (x1, 0, 0)]
    on = [p for pts in nodes.values() for p in pts if abs(p[0] - DIFF_OUT) < 70 and abs(p[2]) < 70]
    y0, y1 = min(p[1] for p in on) - 15.0, max(p[1] for p in on) + 15.0
    ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(DIFF_OUT, y0, 0), P(DIFF_OUT, y1, 0))
    ln.isCenterLine = True
    ends += [(DIFF_OUT, y0, 0), (DIFF_OUT, y1, 0)]
    try:
        sk.arePointsShown = False
    except Exception:
        pass
    return ends
