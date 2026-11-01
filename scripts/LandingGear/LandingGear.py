# =============================================================================
# LandingGear.py  -  Chapter 30 of "Mechanical Design in Autodesk Fusion with Claude": the main landing gear of a
# light aircraft. A leg turns forward on its trunnion to retract; a drag brace of two links, folding at a knee, holds
# it down. Down and locked, the brace stands a few degrees past straight (over centre), so a load that would retract
# the leg drives the knee further the locked way. Driving the upper brace through straight unlocks and retracts it.
# Written from PROMPT.txt beside it; checked by checks/landing_gear_check.py.
#
#   INPUTS        the dialog: leg L, wheel radius Rw, brace point on the leg b, airframe pivot (Ax, Az), upper link
#                 a1, over-centre angle, drag load P, gear weight W and its centre's distance r_g
#   DERIVED       gear(): the lower link a2 for the over-centre angle, the dead centre and the leg's travel aft to
#                 unlock, the drive angles down, at dead centre and up, the brace force for P, the drive torque
#   HARD RULES    build() refuses an over-centre angle outside 1 to 10 deg, a brace that does not fold steadily
#                 through 90 deg of retraction, and a knee folded below 20 deg when up
#   CONVENTIONS   x forward, z up, the trunnion at the origin, everything turning about y; bodies from
#                 TemporaryBRepManager; four As-built revolute joints closing one loop
#   ACCEPTANCE    the message; the leg on the angle worked out here at every step of the drive; the leg moving aft
#                 to dead centre first; no interference; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'LandingGear 1.0 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
UP = math.radians(90.0)                        # the leg retracts 90 deg forward
Y_LEG, Y_UPPER, Y_LOWER, Y_FRAME = (-20.0, 20.0), (25.0, 35.0), (40.0, 50.0), (-65.0, -25.0)


def leg_point(s, alpha):
    """The point s along the leg (from the trunnion) with the leg turned alpha forward of straight down."""
    return (s * math.sin(alpha), -s * math.cos(alpha))


def knee(A, a1, a2, Bq, side):
    """The knee: where the upper link's circle about A meets the lower link's about B; side +1 is the left of the
    line from A to B (the fold side), -1 the right (the locked side)."""
    dx, dy = Bq[0] - A[0], Bq[1] - A[1]
    D = math.hypot(dx, dy)
    m = (a1 * a1 - a2 * a2 + D * D) / (2 * D)
    h = math.sqrt(max(0.0, a1 * a1 - m * m))
    mx, my = A[0] + m * dx / D, A[1] + m * dy / D
    return (mx - side * h * dy / D, my + side * h * dx / D)


def gear(L=900.0, Rw=250.0, b=400.0, Ax=125.0, Az=75.0, a1=300.0, over=3.0, P=10000.0, W=300.0, r_g=500.0):
    A = (Ax, Az)
    B0 = leg_point(b, 0.0)
    d0 = math.hypot(A[0] - B0[0], A[1] - B0[1])
    cp = math.cos(math.radians(180.0 - over))
    a2 = a1 * cp + math.sqrt((a1 * cp) ** 2 - a1 * a1 + d0 * d0)   # knee at 180 - over when down
    lo, hi = math.radians(-30.0), 0.0                               # dead centre: A to B = a1 + a2, aft of down
    for _ in range(200):
        mid = (lo + hi) / 2
        Bm = leg_point(b, mid)
        if math.hypot(A[0] - Bm[0], A[1] - Bm[1]) > a1 + a2:
            lo = mid
        else:
            hi = mid
    al_dc = (lo + hi) / 2
    th = lambda K: math.atan2(K[1] - A[1], K[0] - A[0])
    th_down = th(knee(A, a1, a2, B0, -1))
    Bdc = leg_point(b, al_dc)
    th_dc = math.atan2(Bdc[1] - A[1], Bdc[0] - A[0])
    Bup = leg_point(b, UP)
    d_up = math.hypot(A[0] - Bup[0], A[1] - Bup[1])
    th_up = th(knee(A, a1, a2, Bup, 1))
    c_up = (a1 * a1 + a2 * a2 - d_up * d_up) / (2 * a1 * a2)
    phi_up = math.degrees(math.acos(max(-1.0, min(1.0, c_up))))    # 0 when the brace cannot reach the up position
    ds = [math.hypot(A[0] - leg_point(b, UP * k / 90)[0], A[1] - leg_point(b, UP * k / 90)[1]) for k in range(91)]
    steady = all(ds[k + 1] < ds[k] for k in range(90))
    arm = abs((B0[0] - A[0]) * A[1] - A[0] * (B0[1] - A[1])) / d0  # the trunnion to the brace's line, down
    F_brace = P * (L + Rw) / arm
    return dict(A=A, a2=a2, d0=d0, al_dc=al_dc, th_down=th_down, th_dc=th_dc, th_up=th_up, phi_up=phi_up,
                steady=steady, d_up=d_up, arm=arm, F_brace=F_brace, over=over, b=b, a1=a1, L=L, Rw=Rw, W=W, r_g=r_g)


def drive_angle(g, alpha):
    """The upper link's angle for the leg at alpha, on the fold side (after dead centre)."""
    A = g['A']
    K = knee(A, g['a1'], g['a2'], leg_point(g['b'], alpha), 1)
    return math.atan2(K[1] - A[1], K[0] - A[0])


def drive_torque(g, alpha, e=1e-6):
    """Torque at the upper link's pivot to lift the gear's weight, by virtual work (N m): the weight's torque about
    the trunnion, W r_g sin(alpha), times d(alpha) / d(theta)."""
    dth = (drive_angle(g, alpha + e) - drive_angle(g, alpha - e)) / (2 * e)
    return g['W'] * g['r_g'] / 1000.0 * math.sin(alpha) / dth


# ---------- Fusion building blocks (mm at the interface; the mechanism's plane is xz) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cyl(p0, p1, r):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def pin(p, y0, y1, r):
    """A pin along y through the point p = (x, z)."""
    return cyl((p[0], y0, p[1]), (p[0], y1, p[1]), r)


def bar(p, q, w, ys):
    """A flat bar in the xz plane from p to q, w wide, round ends, between y = ys[0] and ys[1]."""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    ux, uz = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    c = P3((p[0] + q[0]) / 2 * CM, (ys[0] + ys[1]) / 2 * CM, (p[1] + q[1]) / 2 * CM)
    body = tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(ux, 0, uz), V3(0, 1, 0), L * CM,
                                                                  (ys[1] - ys[0]) * CM, w * CM))
    union(body, pin(p, ys[0], ys[1], w / 2), pin(q, ys[0], ys[1], w / 2))
    return body


def boolean(a, kind, *bs):
    for b in bs:
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


def cut(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.DifferenceBooleanType, *bs)


def add_body(comp, tb, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def new_part(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_at(root, moving, fixed, p, name):
    """An As-built revolute joint at the point p = (x, z) turning `moving` about world y; the joint directions are
    tried and the one that turns the part about y is kept (Chapter 17). Returns (joint, sign)."""
    J = adsk.fusion.JointDirections
    sk = moving.component.sketches.add(moving.component.xZConstructionPlane)
    sk.isVisible = False
    sp = sk.modelToSketchSpace(P3(p[0] * CM, 0, p[1] * CM))                      # the xz sketch's own axes differ
    pt = sk.sketchPoints.add(sp).createForAssemblyContext(moving)
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
        if abs(w[1]) > 0.02 and abs(w[0]) < 0.02 and abs(w[2]) < 0.02:
            j.name = name
            return j, (1 if w[1] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about y' % moving.name)


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the gear ----------
def build(L=900.0, Rw=250.0, b=400.0, Ax=125.0, Az=75.0, a1=300.0, over=3.0, P=10000.0, W=300.0, r_g=500.0):
    if not 1.0 <= over <= 10.0:
        raise ValueError('An over-centre angle of %.1f deg: keep it from 1 to 10 deg.' % over)
    g = gear(L, Rw, b, Ax, Az, a1, over, P, W, r_g)
    if not g['steady']:
        raise ValueError('The brace does not fold steadily: the distance from its pivot to the leg grows somewhere in '
                         'the 90 deg of retraction (a2 = %.1f mm).' % g['a2'])
    if g['phi_up'] < 20.0:
        raise ValueError('Retracted, the knee is folded to %.2f deg: below 20 deg the brace locks itself.' % g['phi_up'])
    A, a2 = g['A'], g['a2']
    B0 = leg_point(b, 0.0)
    K0 = knee(A, a1, a2, B0, -1)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    fr = new_part(root, 'Airframe')
    x0, x1, z0, z1 = -60.0, Ax + 60.0, -40.0, Az + 60.0                          # a plate behind the leg
    fb = tbm().createBox(adsk.core.OrientedBoundingBox3D.create(
        P3((x0 + x1) / 2 * CM, (Y_FRAME[0] + Y_FRAME[1]) / 2 * CM, (z0 + z1) / 2 * CM),
        V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM, (Y_FRAME[1] - Y_FRAME[0]) * CM, (z1 - z0) * CM))
    union(fb, pin((0.0, 0.0), Y_FRAME[0], Y_LEG[1], 10.0), pin(A, Y_FRAME[0], Y_UPPER[1], 8.0))
    add_body(fr.component, fb, 'Airframe')
    fr.isGrounded = True

    leg = new_part(root, 'Leg')
    axle = leg_point(L, 0.0)
    lb = bar((0.0, 0.0), axle, 50.0, Y_LEG)
    union(lb, pin((0.0, 0.0), Y_LEG[0], Y_LEG[1], 25.0), pin(B0, Y_LEG[1] - 0.001, Y_LOWER[1], 8.0))
    union(lb, pin(axle, -50.0, 50.0, Rw))                                          # the wheel
    cut(lb, pin((0.0, 0.0), Y_LEG[0] - 1, Y_LEG[1] + 1, 10.2))                     # the trunnion's bore
    add_body(leg.component, lb, 'Leg')

    up = new_part(root, 'Upper brace')
    ub = bar(A, K0, 30.0, Y_UPPER)
    union(ub, pin(K0, Y_UPPER[1] - 0.001, Y_LOWER[1], 8.0))                        # the knee's pin
    cut(ub, pin(A, Y_UPPER[0] - 1, Y_UPPER[1] + 1, 8.2))
    add_body(up.component, ub, 'Upper brace')

    lo = new_part(root, 'Lower brace')
    lw = bar(K0, B0, 30.0, Y_LOWER)
    cut(lw, pin(K0, Y_LOWER[0] - 1, Y_LOWER[1] + 1, 8.2), pin(B0, Y_LOWER[0] - 1, Y_LOWER[1] + 1, 8.2))
    add_body(lo.component, lw, 'Lower brace')

    revolute_at(root, leg, fr, (0.0, 0.0), 'Leg turns')
    revolute_at(root, up, fr, A, 'Upper brace turns')
    revolute_at(root, lo, up, K0, 'Knee')
    revolute_at(root, lo, leg, B0, 'Lower brace on leg')                              # closes the loop

    colour(design, fr, 'Paint - Enamel Glossy (Grey)')
    colour(design, leg, 'Paint - Enamel Glossy (Blue)')
    colour(design, up, 'Paint - Enamel Glossy (Yellow)')
    colour(design, lo, 'Paint - Enamel Glossy (Red)')
    app.activeViewport.fit()

    tq = max(abs(drive_torque(g, g['al_dc'] + (UP - g['al_dc']) * k / 400)) for k in range(1, 400))
    return ('Landing gear built: leg %.0f mm, wheel %.0f mm, brace pivot (%.0f, %.0f) mm, upper link %.0f mm, '
            'lower link %.3f mm.\n\n'
            'Down and locked, the knee is %.1f deg past straight. To unlock, the leg moves %.4f deg aft to dead centre.\n'
            'Drive angle of the upper brace: %.3f deg down, %.3f deg at dead centre, %.3f deg up (knee %.2f deg).\n'
            'The brace line passes %.3f mm from the trunnion: a drag load of %.0f N at the ground puts %.0f N in the '
            'brace.\nLifting %.0f N at %.0f mm takes up to %.2f N m at the upper brace.\n\n'
            'Drive Joints on "Upper brace turns" unlocks and retracts the gear.\n\n[%s]'
            % (L, Rw, Ax, Az, a1, a2, over, -math.degrees(g['al_dc']), math.degrees(g['th_down']),
               math.degrees(g['th_dc']), math.degrees(g['th_up']), g['phi_up'], g['arm'], P, g['F_brace'], W, r_g,
               tq, VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('L', 'Leg, trunnion to axle', 'mm', 900.0), ('Rw', 'Wheel radius', 'mm', 250.0),
          ('b', 'Brace point on the leg', 'mm', 400.0), ('Ax', 'Brace pivot, forward', 'mm', 125.0),
          ('Az', 'Brace pivot, up', 'mm', 75.0), ('a1', 'Upper brace link', 'mm', 300.0),
          ('over', 'Over-centre angle', 'deg', 3.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            v = {}
            for key, _, unit, _ in FIELDS:
                x = ins.itemById(key).value
                v[key] = x / CM if unit == 'mm' else math.degrees(x)
            ui.messageBox(build(**v), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, unit, d in FIELDS:
            ins.addValueInput(key, label, unit, VI(d * CM if unit == 'mm' else math.radians(d)))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'LandingGearCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Landing gear', 'Retracting gear with an over-centre brace')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Landing gear could not start:\n' + traceback.format_exc())
