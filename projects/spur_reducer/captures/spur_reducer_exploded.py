# Pictures for Chapter 27: SpurReducer exploded. Every part moves along the direction it is put together in: the
# shaft assemblies lift out of the base, the cover and its screws go up, the nuts down, and the parts on each shaft
# slide off along its axis in the order of assembly. The joints and the link are deleted first (this design is never
# saved), every part is let go of its parent, and the design is turned to direct modelling: in a parametric design
# each move made from a script throws away the one before, until a snapshot captures it. TRAILS draws the axes of the explosion.
# The script prints the items' anchors on the view for the balloons of the figure (tools/capture.py keeps them).
import sys, importlib, math, json
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

LIFT = 240.0                    # the shaft assemblies, out of the base (mm)
COVER = 520.0                   # the cover, up
VIEW = (-0.821, -0.383, 0.423)  # towards the camera: from the input's side, 25 deg round from -x and 25 deg up
TRAILS = True
HIDE = ()

# axial moves along y, from the end each part goes on from; the output shaft leaves its gear towards -y
AXIAL = {
    'Bearing 6204 A': -165.0, 'Cap input through': -215.0, 'Lip seal 20x35x7': -250.0, 'Cap screw M6x16 inL': -285.0,
    'Bearing 6204 B': 80.0, 'Cap input blind': 120.0, 'Cap screw M6x16 inR': 155.0,
    'Output shaft': -190.0, 'Key 10x8x42 gear': -190.0, 'Key 8x7x52 output': -190.0,
    'Bearing 6206 A': -270.0, 'Cap output blind': -320.0, 'Cap screw M6x16 outL': -355.0,
    'Spacer': 130.0, 'Bearing 6206 B': 175.0, 'Cap output through': 225.0, 'Lip seal 30x47x7': 262.0,
    'Cap screw M6x16 outR': 300.0,
}
RADIAL = {'Key 5x5x28 input': 22.0, 'Key 10x8x42 gear': 45.0, 'Key 8x7x52 output': 25.0}   # up, out of the keyseat
OTHER = {'Housing cover': (0, 0, COVER), 'Filler plug M16': (0, 0, COVER + 70.0), 'Screw M10x45': (0, 0, COVER + 95.0),
         'Nut M10': (0, 0, -70.0), 'Dowel 8x24': (0, 0, 60.0), 'Drain plug M16': (300.0, 0, 0)}
SHAFT_PARTS = ('Bearing 6204', 'Bearing 6206', 'Cap input', 'Cap output', 'Lip seal', 'Cap screw')   # held by the housing


def move_of(name, nested_in_shaft):
    """The move (mm) of a part, relative to its parent: the shaft assemblies are lifted as a whole."""
    v = [0.0, 0.0, 0.0]
    for k, t in OTHER.items():
        if name.startswith(k):
            v = list(t)
    for k, dy in AXIAL.items():
        if name.startswith(k):
            v[1] += dy
    for k, dz in RADIAL.items():
        if name.startswith(k):
            v[2] += dz
    if not nested_in_shaft and name.startswith(SHAFT_PARTS):
        v[2] += LIFT                                                 # the outer rings, caps and seals go up too
    return v


def run(_context):
    ns = fc.load('__COMPANION__/projects/spur_reducer/SpurReducer/SpurReducer.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    for coll in (root.motionLinks, root.asBuiltJoints):
        for i in reversed(range(coll.count)):
            coll.item(i).deleteMe()
    des.designType = adsk.fusion.DesignTypes.DirectDesignType     # a parametric design keeps only the last move
    moved = 0
    for top in root.occurrences:                                     # the shaft assemblies first, as a whole
        top.isGrounded = False
        if top.component.name.endswith('shaft assembly'):
            m = top.transform2
            m.translation = adsk.core.Vector3D.create(0, 0, LIFT * 0.1)
            top.transform2 = m
    for top in root.occurrences:                                     # then each part, through its proxy in the root
        shaft = top.component.name.endswith('shaft assembly')
        for o in top.childOccurrences:
            o.nativeObject.isGroundToParent = False
            v = move_of(o.component.name, shaft)
            if any(v):
                m = o.transform2
                t = m.translation
                m.translation = adsk.core.Vector3D.create(t.x + v[0] * 0.1, t.y + v[1] * 0.1, t.z + v[2] * 0.1)
                o.transform2 = m
                moved += 1
    g = [o for o in root.allOccurrences if o.component.name == 'Gear z95'][0]
    print('parts moved: %d; the gear now at z = %.1f mm' % (moved, g.transform2.translation.z * 10))
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    for o in root.allOccurrences:
        if o.component.name in HIDE:
            o.isLightBulbOn = False
    nodes = {o.component.name: world_nodes(o) for o in root.allOccurrences if o.component.bRepBodies.count}
    ends = trails(root, nodes) if TRAILS else []
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
    print('ANCHORS ' + json.dumps(anchors(vp.camera, nodes, ends)))


# the parts list's items and the part each balloon points at
ITEMS = ['Housing base', 'Housing cover', 'Cap input through', 'Cap input blind', 'Cap output through',
         'Cap output blind', 'Pinion shaft', 'Output shaft', 'Gear z95', 'Spacer', 'Key 5x5x28 input',
         'Key 10x8x42 gear', 'Key 8x7x52 output', 'Bearing 6204 A outer ring', 'Bearing 6206 B outer ring',
         'Lip seal 20x35x7', 'Lip seal 30x47x7', 'Cap screw M6x16 outR1', 'Screw M10x45 2', 'Nut M10 2',
         'Dowel 8x24 1', 'Drain plug M16', 'Filler plug M16']
# a balloon on a housing half points at the middle of its end wall, the flange being 20 mm proud of it
FACE = {'Housing base': lambda lo, hi: (lo[0] + 20.0, 0.0, (lo[2] + hi[2]) / 2),
        'Housing cover': lambda lo, hi: (lo[0] + 20.0, 0.0, (lo[2] + hi[2]) / 2)}


def world_nodes(o):
    """The nodes of the display meshes of an occurrence's bodies, in world mm."""
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
    """Every node projected on the view (mm across, mm up): the extent of the whole picture, and for each item the
    node nearest the middle of its part among the third of its nodes nearest the camera, which are in sight."""
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
        if name in FACE:
            lo = [min(p[i] for p in pts) for i in range(3)]
            hi = [max(p[i] for p in pts) for i in range(3)]
            items[k] = [name, list(uv(FACE[name](lo, hi)))]
            continue
        near = sorted(pts, key=lambda p: dot(p, d))[:max(1, len(pts) // 3)]
        cu = sum(uv(p)[0] for p in pts) / len(pts)
        cv = sum(uv(p)[1] for p in pts) / len(pts)
        items[k] = [name, min((uv(p) for p in near), key=lambda q: (q[0] - cu) ** 2 + (q[1] - cv) ** 2)]
    tg = [t.x * 10, t.y * 10, t.z * 10]
    return dict(right=r, up=u, extent=ext, items=items, target=list(uv(tg)), radius=cam.viewExtents * 10)


def trails(root, nodes):
    """Centre lines: each shaft's axis over the length its parts are spread on, at the height they were lifted to,
    15 mm past the last part at each end, and the paths of the two plugs and the two dowels. Returns the shafts'
    lines' ends."""
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Explosion axes'
    P = lambda x, y, z: adsk.core.Point3D.create(x * 0.1, y * 0.1, z * 0.1)
    ends = []
    for x in (0.0, 171.0):
        on = [p for pts in nodes.values() for p in pts if abs(p[0] - x) < 60 and abs(p[2] - LIFT) < 60]
        y0, y1 = min(p[1] for p in on) - 15.0, max(p[1] for p in on) + 15.0
        ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(x, y0, LIFT), P(x, y1, LIFT))
        ln.isCenterLine = True
        ends += [(x, y0, LIFT), (x, y1, LIFT)]
    for name, axis, back in (('Drain plug M16', 0, OTHER['Drain plug M16'][0]),
                             ('Filler plug M16', 2, OTHER['Filler plug M16'][2] - COVER),
                             ('Dowel 8x24 1', 2, OTHER['Dowel 8x24'][2]), ('Dowel 8x24 2', 2, OTHER['Dowel 8x24'][2])):
        pts = nodes[name]                                            # from where the part was to where it is
        c = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
        lo = min(p[axis] for p in pts)
        p0, p1 = list(c), list(c)
        p0[axis], p1[axis] = lo - back, lo
        ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(*p0), P(*p1))
        ln.isCenterLine = True
    try:
        sk.arePointsShown = False
    except Exception:
        pass
    return ends
