# Pictures for Chapter 33: the flap actuator taken apart, each part along the way it goes in. The gears come out of
# the housing's nose forwards, with the washer, the end bolt and the cover in front of them; the ring nut, the thrust
# bearings and the screw draw aft out of the bore, and behind them the rod's parts in the order they go on; the motor
# unit and the encoder draw aft off the gearbox's face; the trunnions out sideways. As in Chapters 29 to 31, the
# joints are deleted and the design turned to direct modelling first; the script prints the items' anchors for the
# balloons.
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

VIEW = (0.62, -0.45, 0.50)      # towards the camera: from the side, ahead and above
MOVES = {                        # mm in the actuator's frame: x along the span, y aft along the actuator, z up
    'Cover screw': (0, -165, 0), 'Gear cover': (0, -135, 0), 'End bolt': (0, -105, 0), 'Washer M8 1': (0, -75, 0),
    'Screw gear': (0, -45, 0), 'Idler': (0, -45, 0), 'Bearing 6201': (0, -45, 0), 'Pinion z18': (0, -45, 0),
    'Encoder pinion': (0, -45, 0),
    'Ring nut': (0, 225, 0), 'Bearing BSD': (0, 270, 0), 'Ball screw': (0, 330, 0), 'Key 8x7': (0, 330, 0),
    'End stop': (0, 330, 0), 'Washer M8 2': (0, 330, 0), 'Stop bolt': (0, 330, 0),
    'Rod seal': (0, 350, 0), 'Nut retaining ring': (0, 550, 0), 'Ball nut': (0, 570, 0), 'Nut key': (0, 570, -12),
    'Piston rod': (0, 680, 0), 'Guide key': (0, 680, 14), 'Jam nut': (0, 730, 0), 'Rod end': (0, 760, 0),
    'Motor unit': (0, 60, 0), 'Gearhead shaft': (0, 60, 0), 'Motor screw': (0, 90, 0),
    'Encoder': (0, 50, 0), 'Encoder shaft': (0, 50, 0),
    'Trunnion R': (45, 0, 0), 'Trunnion L': (-45, 0, 0),
}
ITEMS = ['Gear cover', 'Screw gear z54', 'Idler gear z30', 'Pinion z18', 'Actuator housing', 'Motor unit',
         'Trunnion R', 'Ring nut M66x1.5', 'Bearing BSD 2562 CGA 1', 'Ball screw 32x10', 'Ball nut R32-10T4-RSI',
         'Piston rod', 'Rod end SI 20 ES']


def move_of(name):
    best = ''
    for k in MOVES:
        if name.startswith(k) and len(k) > len(best):
            best = k
    return list(MOVES[best]) if best else [0.0, 0.0, 0.0]


def run(_context):
    ns = fc.load('__COMPANION__/scripts/FlapActuator/FlapActuator.py')
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
    cy = 0.5 * (min(p[1] for pts in nodes.values() for p in pts) + max(p[1] for pts in nodes.values() for p in pts))
    cam.target = adsk.core.Point3D.create(0, cy * 0.1, 0)
    cam.eye = adsk.core.Point3D.create(*(300 * c / n + (cy * 0.1 if i == 1 else 0) for i, c in enumerate(VIEW)))
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
    p = o.assemblyContext
    while p:
        m.transformBy(p.transform2)
        p = p.assemblyContext
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
    """The actuator's axis over all the parts spread along it, and the lines the motor unit, the encoder and the
    trunnions draw out along."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Explosion axis'
    P = lambda x, y, z: adsk.core.Point3D.create(x * 0.1, y * 0.1, z * 0.1)
    on = [p for pts in nodes.values() for p in pts if abs(p[0]) < 40 and abs(p[2]) < 40]
    y0, y1 = min(p[1] for p in on) - 15.0, max(p[1] for p in on) + 15.0
    lines = [((0, y0, 0), (0, y1, 0)), ((0, 40.0, -99.0), (0, 110.0, -99.0)),
             ((-42.43, 40.0, -42.43), (-42.43, 100.0, -42.43)), ((48.0, 0, 0), (130.0, 0, 0)),
             ((-48.0, 0, 0), (-130.0, 0, 0))]
    for a, b in lines:
        ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(*a), P(*b))
        ln.isCenterLine = True
    try:
        sk.arePointsShown = False
    except Exception:
        pass
    return [a for a, _ in lines] + [b for _, b in lines]
