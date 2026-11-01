# =============================================================================
# Palletizer.py  -  Chapter 36 of "Mechanical Design in Autodesk Fusion with Claude": a four-axis palletizing robot.
# A turntable, an upper arm and a forearm; a crank beside the shoulder drives the forearm through a parallelogram, so
# that its motor stays on the turntable and sets the forearm's angle directly; two more parallelograms keep the load
# level in every pose. The joint angles for each box of a pallet, and the reach. Written from PROMPT.txt beside it;
# checked by checks/palletizer_check.py.
#
#   INPUTS        the dialog: the pallet's distance from the robot, the number of layers
#   DERIVED       fk() and ik() in the arm's plane; pattern(): the place point of every box; the angles for each,
#                 within the limits of the upper arm, the forearm and the angle between them
#   HARD RULES    build() refuses 1 to 5 layers only, and a pallet with a box out of reach, naming the box and the
#                 limit it breaks
#   CONVENTIONS   z up, the arm in the xz plane at J1 = 0; components at the origin, bodies in world coordinates,
#                 each link in its own layer along y; As-built revolute joints, the direction tried and the sign
#                 measured; the three parallelograms closed by joints, so that Fusion solves them
#   ACCEPTANCE    the message; the load level at every pose; the tool where fk puts it for the angles held; every
#                 box reachable; the motors' gravity torques by virtual work against levers; no interference
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'Palletizer 1.0 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
A = (150.0, 700.0)                 # the shoulder (J2) in the arm's plane: x, z (mm)
L2, L3, E = 900.0, 1000.0, 250.0   # upper arm, forearm, the forearm's tail and the crank
A1 = (-30.0, 700.0)                # the first levelling rod's pivot on the turntable, 180 mm behind the shoulder
OFF1, OFF2 = (-180.0, 0.0), (0.0, 150.0)   # the elbow plate's two arms: to rod 1 and to rod 2
H_TCP = 220.0                      # the tool point, the load's bottom, below the wrist pivot W
T2_LIM, T3_LIM, G_LIM = (40.0, 130.0), (-70.0, 30.0), (25.0, 155.0)   # upper arm, forearm, angle between (deg)
BOX, PALLET_TOP = (400.0, 300.0, 200.0), 150.0
LOAD = (300.0, 200.0, 100.0)       # a steel block standing for the payload
Y = dict(ua=(-30, 30), fa=(35, 75), cr=(80, 100), r3=(105, 125), pl=(-75, -35), r1=(-100, -80), r2=(-125, -105),
         br=(-30, 30))


def add(p, q, k=1.0):
    return (p[0] + k * q[0], p[1] + k * q[1])


def ang(deg):
    return (math.cos(math.radians(deg)), math.sin(math.radians(deg)))


def points(t2, t3):
    """The pivots in the arm's plane for the upper arm at t2 and the forearm at t3 (deg from horizontal)."""
    B = add(A, ang(t2), L2)
    W = add(B, ang(t3), L3)
    C = add(A, ang(t3 + 180.0), E)                         # the crank, parallel to the forearm's tail
    D = add(B, ang(t3 + 180.0), E)
    return dict(A=A, B=B, C=C, D=D, W=W, A1=A1, B1=add(B, OFF1), B2=add(B, OFF2), W2=add(W, OFF2))


def fk(q1, t2, t3):
    """The tool point (x, y, z) for the turntable at q1 and the arm at t2, t3 (deg)."""
    W = points(t2, t3)['W']
    r, z = W[0], W[1] - H_TCP
    return (r * math.cos(math.radians(q1)), r * math.sin(math.radians(q1)), z)


def ik(x, y, z):
    """(q1, t2, t3) for a tool point, the elbow up; with the reason if a limit is broken."""
    q1 = math.degrees(math.atan2(y, x))
    wr, wz = math.hypot(x, y) - A[0], z + H_TCP - A[1]
    d = math.hypot(wr, wz)
    c = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * d)
    if abs(c) > 1:
        return None, 'beyond the arm\'s reach'
    t2 = math.degrees(math.atan2(wz, wr) + math.acos(c))
    B = add(A, ang(t2), L2)
    t3 = math.degrees(math.atan2(wz + A[1] - B[1], wr + A[0] - B[0]))
    for v, (lo, hi), name in ((t2, T2_LIM, 'the upper arm'), (t3, T3_LIM, 'the forearm'),
                              (t2 - t3, G_LIM, 'the angle between the arms')):
        if not lo <= v <= hi:
            return None, '%s at %.1f deg (limits %.0f to %.0f)' % (name, v, lo, hi)
    return (q1, t2, t3), ''


def pattern(xc, layers):
    """The place point of every box, the middle of its top: two rows of four on a 800 x 1200 mm pallet."""
    out = []
    for k in range(layers):
        z = PALLET_TOP + (k + 1) * BOX[2]
        for i in (-1, 1):
            for j in (-3, -1, 1, 3):
                out.append((xc + i * BOX[0] / 2, j * BOX[1] / 2, z))
    return out


# ---------- Fusion building blocks (mm at the interface) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cyl(p0, p1, r):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    return tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM,
                                                                   (y1 - y0) * CM, (z1 - z0) * CM))


def bar(p, q, w, ys):
    """A flat bar in the xz plane from p to q, w wide, round ends, between y = ys[0] and ys[1]."""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    ux, uz = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    c = P3((p[0] + q[0]) / 2 * CM, (ys[0] + ys[1]) / 2 * CM, (p[1] + q[1]) / 2 * CM)
    body = tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(ux, 0, uz), V3(0, 1, 0), L * CM,
                                                                  (ys[1] - ys[0]) * CM, w * CM))
    union(body, cyl((p[0], ys[0], p[1]), (p[0], ys[1], p[1]), w / 2), cyl((q[0], ys[0], q[1]), (q[0], ys[1], q[1]), w / 2))
    return body


def boolean(a, kind, *bs):
    for b in bs:
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


def add_body(comp, tb, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def new_part(root, name, tb):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    add_body(occ.component, tb, name)
    return occ


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_at(root, moving, fixed, p, axis, name):
    """An As-built revolute joint through p = (x, z) at y = 0, turning `moving` about the world `axis`; the joint
    directions are tried and the one that turns the part about `axis` is kept. Returns (joint, sign)."""
    J = adsk.fusion.JointDirections
    sk = moving.component.sketches.add(moving.component.xZConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(sk.modelToSketchSpace(P3(p[0] * CM, 0, p[1] * CM))).createForAssemblyContext(moving)
    for direction in (J.YAxisJointDirection, J.ZAxisJointDirection, J.XAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[c][k] for k in range(3)) for c in range(3)] for a in range(3)]
        w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        dot = sum(w[i] * axis[i] for i in range(3))
        if abs(dot) > 0.02 and sum(w[i] * w[i] for i in range(3)) - dot * dot < 4e-4:
            j.name = name
            return j, (1 if dot > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about %s' % (moving.name, axis))


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the robot ----------
def build(xc=1200.0, layers=3):
    layers = int(round(layers))
    if not 1 <= layers <= 5:
        raise ValueError('%d layers: from 1 to 5.' % layers)
    sols = []
    for n, (x, y, z) in enumerate(pattern(xc, layers)):
        q, why = ik(x, y, z)
        if q is None:
            raise ValueError('Box %d, its top at (%.0f, %.0f, %.0f) mm, is out of reach: %s.' % (n + 1, x, y, z, why))
        sols.append(q)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    p = points(90.0, 0.0)                                  # built with the upper arm up and the forearm level
    B, W = p['B'], p['W']

    base = new_part(root, 'Base', cyl((0, 0, 0), (0, 0, 400), 300.0))
    base.isGrounded = True
    tt = cyl((0, 0, 401), (0, 0, 500), 280.0)
    for y0, y1 in ((140, 160), (-160, -140)):
        union(tt, box(-80.0, 230.0, y0, y1, 499.999, 740.0))
    tt = new_part(root, 'Turntable', tt)
    ua = new_part(root, 'Upper arm', bar(A, B, 80.0, Y['ua']))
    fa = new_part(root, 'Forearm', bar(p['D'], W, 60.0, Y['fa']))
    cr = new_part(root, 'Crank', bar(A, p['C'], 40.0, Y['cr']))
    r3 = new_part(root, 'Rod 3', bar(p['C'], p['D'], 30.0, Y['r3']))
    plb = bar(B, p['B1'], 40.0, Y['pl'])
    union(plb, bar(B, p['B2'], 40.0, Y['pl']))
    pl = new_part(root, 'Elbow plate', plb)
    r1 = new_part(root, 'Rod 1', bar(A1, p['B1'], 30.0, Y['r1']))
    r2 = new_part(root, 'Rod 2', bar(p['B2'], p['W2'], 30.0, Y['r2']))
    br = new_part(root, 'Bracket', bar(p['W2'], add(W, (0.0, -80.0)), 40.0, Y['br']))       # its round end at -100
    lx, ly, lz = LOAD
    tb = box(W[0] - 150.0, W[0] + 150.0, -100.0, 100.0, W[1] - 120.5, W[1] - 100.5)          # the tool's plate
    union(tb, box(W[0] - lx / 2, W[0] + lx / 2, -ly / 2, ly / 2, W[1] - H_TCP, W[1] - 120.5 + 0.001))
    tool = new_part(root, 'Tool', tb)

    Yax, Zax = (0, 1, 0), (0, 0, 1)
    revolute_at(root, tt, base, (0.0, 0.0), Zax, 'J1 turntable')
    revolute_at(root, ua, tt, A, Yax, 'J2 upper arm')
    revolute_at(root, cr, tt, A, Yax, 'J3 crank')
    revolute_at(root, fa, ua, B, Yax, 'Elbow')
    revolute_at(root, r3, cr, p['C'], Yax, 'Rod 3 on crank')
    revolute_at(root, r3, fa, p['D'], Yax, 'Rod 3 on forearm')                 # closes the drive's parallelogram
    revolute_at(root, pl, ua, B, Yax, 'Elbow plate')
    revolute_at(root, r1, tt, A1, Yax, 'Rod 1 on turntable')
    revolute_at(root, r1, pl, p['B1'], Yax, 'Rod 1 on plate')                  # closes the first levelling one
    revolute_at(root, br, fa, W, Yax, 'Wrist')
    revolute_at(root, r2, pl, p['B2'], Yax, 'Rod 2 on plate')
    revolute_at(root, r2, br, p['W2'], Yax, 'Rod 2 on bracket')                # closes the second levelling one
    revolute_at(root, tool, br, W, Zax, 'J4 tool')

    for occ, c in ((base, 'Grey'), (tt, 'Grey'), (ua, 'Yellow'), (fa, 'Yellow'), (cr, 'Blue'), (r3, 'Blue'),
                   (pl, 'Red'), (r1, 'Red'), (r2, 'Red'), (br, 'Grey'), (tool, 'Blue')):
        colour(design, occ, 'Paint - Enamel Glossy (%s)' % c)
    app.activeViewport.fit()

    far = max(range(len(sols)), key=lambda k: math.hypot(*pattern(xc, layers)[k][:2]))
    return ('Palletizer built: shoulder at (%.0f, %.0f) mm, upper arm %.0f mm, forearm %.0f mm, crank and tail %.0f mm.\n'
            'Limits: upper arm %.0f to %.0f deg, forearm %.0f to %.0f deg, between them %.0f to %.0f deg.\n\n'
            'Pallet at %.0f mm, %d layers of 8 boxes: all %d within reach.\nThe farthest box: J1 %.2f, upper arm %.2f, '
            'forearm %.2f deg.\n\nThe crank turns the forearm to its angle directly; the plates and rods keep the tool '
            'level.\n\n[%s]' % (A[0], A[1], L2, L3, E, T2_LIM[0], T2_LIM[1], T3_LIM[0], T3_LIM[1], G_LIM[0], G_LIM[1],
                                xc, layers, len(sols), sols[far][0], sols[far][1], sols[far][2], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('xc', 'Pallet distance', 'mm', 1200.0), ('layers', 'Layers', '', 3)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            v = {}
            for key, _, unit, _ in FIELDS:
                v[key] = ins.itemById(key).value / CM if unit == 'mm' else ins.itemById(key).value
            ui.messageBox(build(**v), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('xc', 'Pallet distance', 'mm', VI(1200.0 * CM))
        ins.addIntegerSpinnerCommandInput('layers', 'Layers', 1, 9, 1, 3)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'PalletizerCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Palletizer', 'A four-axis palletizing robot')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Palletizer could not start:\n' + traceback.format_exc())
