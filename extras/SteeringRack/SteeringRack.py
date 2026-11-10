# =============================================================================
# SteeringRack.py  -  Chapter 29 of "Mechanical Design in Autodesk Fusion with Claude": rack-and-pinion steering. A
# pinion moves a rack across the car; tie rods from the rack's ends turn the two steering knuckles on their kingpins.
# The steering arms' angle and the rack's position are chosen so that the wheels follow Ackermann's condition to
# within a set error. Written from PROMPT.txt beside it; checked by checks/steering_check.py.
#
#   INPUTS        the dialog: track t, wheelbase L, arm a, arm angle beta, rack position x_r, ball-joint half
#                 spacing c, rack travel per pinion turn p
#   DERIVED       steering(): the tie rod's length, each knuckle's angle for a rack travel, the travel to 30 deg of
#                 inner steer, the worst Ackermann error, the smallest transmission angle, the steering ratio
#   HARD RULES    build() refuses an Ackermann error over 1 deg up to 30 deg of inner steer, a transmission angle
#                 below 35 deg, and a tie rod shorter than 100 mm
#   CONVENTIONS   x forward, y left, z up; kingpins at (0, +-t/2); bodies from TemporaryBRepManager; As-built Joints
#                 closing a loop on each side; one Motion Link pinion to rack
#   ACCEPTANCE    the message; each knuckle on the angle worked out here at every step of the rack's travel; no
#                 interference; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'SteeringRack 1.0 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
INNER_MAX = 30.0                            # deg of inner steer over which Ackermann is held
ERR_MAX, MU_MIN, TIE_MIN = 1.0, 35.0, 100.0   # deg, deg, mm
Z_RACK, Z_ROD, Z_ARM = (-15.0, 15.0), (15.0, 25.0), (25.0, 35.0)   # the parts' levels (mm)


def arm_tip(t, a, beta, side, delta):
    """The steering arm's tip; side +1 left (+y), -1 right; delta positive turns the wheel left (about +z)."""
    ang = math.pi + side * beta + delta                            # backwards and inwards, then turned by delta
    return (a * math.cos(ang), side * t / 2 + a * math.sin(ang))


def knuckle(t, a, beta, x_r, c, Ltr, side, u):
    """The knuckle's angle for a rack travel u (+y): where the arm tip's circle meets the tie rod's circle, on the
    branch of the straight-ahead position."""
    K = (0.0, side * t / 2)
    J = (x_r, side * c + u)
    dx, dy = J[0] - K[0], J[1] - K[1]
    D = math.hypot(dx, dy)
    A = (a * a - Ltr * Ltr + D * D) / (2 * D)
    h = math.sqrt(max(0.0, a * a - A * A))
    mx, my = K[0] + A * dx / D, K[1] + A * dy / D
    rest = arm_tip(t, a, beta, side, 0.0)
    p = min([(mx + h * dy / D, my - h * dx / D), (mx - h * dy / D, my + h * dx / D)],
            key=lambda q: math.hypot(q[0] - rest[0], q[1] - rest[1]))
    d = math.atan2(p[1] - K[1], p[0] - K[0]) - (math.pi + side * beta)
    return (d + math.pi) % (2 * math.pi) - math.pi, p


def steering(t, L, a, beta_deg, x_r, c, p_turn):
    """Everything worked out from the inputs, at 0.5 mm steps of rack travel to the left (+y): a right turn."""
    beta = math.radians(beta_deg)
    tip0 = arm_tip(t, a, beta, 1, 0.0)
    Ltr = math.hypot(tip0[0] - x_r, tip0[1] - c)
    worst, mu_min, u, rows = 0.0, 180.0, 0.0, []
    while True:
        u += 0.5
        dl, pl = knuckle(t, a, beta, x_r, c, Ltr, 1, u)
        dr, pr = knuckle(t, a, beta, x_r, c, Ltr, -1, u)
        inner, outer = (dr, dl) if abs(dr) >= abs(dl) else (dl, dr)
        if abs(math.degrees(inner)) > INNER_MAX:
            break
        ideal = math.atan(1.0 / (1.0 / math.tan(abs(inner)) + t / L))
        err = math.degrees(abs(outer) - ideal)
        if abs(err) > abs(worst):
            worst = err
        for side, pt, d in ((1, pl, dl), (-1, pr, dr)):              # angle between tie rod and arm
            K = (0.0, side * t / 2)
            J = (x_r, side * c + u)
            v1, v2 = (pt[0] - K[0], pt[1] - K[1]), (J[0] - pt[0], J[1] - pt[1])
            cosm = abs(v1[0] * v2[0] + v1[1] * v2[1]) / (math.hypot(*v1) * math.hypot(*v2))
            mu_min = min(mu_min, math.degrees(math.acos(min(1.0, cosm))))
        rows.append((u, dl, dr))
    e = 0.01
    dd = (knuckle(t, a, beta, x_r, c, Ltr, 1, e)[0] + knuckle(t, a, beta, x_r, c, Ltr, -1, e)[0]) / 2 / e   # rad/mm
    ratio = (360.0 / p_turn) / abs(math.degrees(dd))                                                       # wheel/road
    u_max = rows[-1][0]
    return dict(Ltr=Ltr, worst=worst, mu_min=mu_min, u_max=u_max, ratio=ratio, rows=rows,
                inner_at_max=math.degrees(abs(rows[-1][2])), outer_at_max=math.degrees(abs(rows[-1][1])),
                turns=2 * u_max / p_turn, beta=beta)


# ---------- Fusion building blocks (mm at the interface) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cyl(p0, p1, r):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    return tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM,
                                                                   (y1 - y0) * CM, (z1 - z0) * CM))


def bar(p, q, w, z0, z1):
    """A flat bar from p to q in the XY plane, w wide, with round ends."""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    ux, uy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    c = P3((p[0] + q[0]) / 2 * CM, (p[1] + q[1]) / 2 * CM, (z0 + z1) / 2 * CM)
    b = tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(ux, uy, 0), V3(-uy, ux, 0), L * CM, w * CM,
                                                               (z1 - z0) * CM))
    union(b, cyl((p[0], p[1], z0), (p[0], p[1], z1), w / 2), cyl((q[0], q[1], z0), (q[0], q[1], z1), w / 2))
    return b


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


def place(root, name, x_axis=(1, 0, 0), z_axis=(0, 0, 1), origin=(0.0, 0.0, 0.0)):
    zx, zy, zz = z_axis
    xx, xy, xz = x_axis
    y_axis = (zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx)
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P3(*[v * CM for v in origin]), V3(*x_axis), V3(*y_axis), V3(*z_axis))
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def joint(root, moving, fixed, pt_world, kind, name):
    """An As-built joint at a world point: revolute about Z, slider along Y, revolute about the part's own z."""
    m = moving.transform2.copy()
    m.invert()
    p = P3(*[v * CM for v in pt_world])
    p.transformBy(m)
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    jp = sk.sketchPoints.add(p).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(jp))
    J = adsk.fusion.JointDirections
    if kind == 'slider':
        inp.setAsSliderJointMotion(J.YAxisJointDirection)
    else:
        inp.setAsRevoluteJointMotion(J.ZAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_about(root, moving, fixed, axis, name):
    """An As-built revolute joint at the part's origin that turns it about the world `axis`: the joint directions
    are tried in turn and the one that really turns the part about `axis` is kept (Chapter 17). Returns (joint,
    sign)."""
    J = adsk.fusion.JointDirections
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    for direction in (J.ZAxisJointDirection, J.XAxisJointDirection, J.YAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.2
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
        w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        dot = sum(w[i] * axis[i] for i in range(3))
        if abs(dot) > 0.1:
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


# ---------- the steering ----------
def build(t=1400.0, L=2500.0, a=150.0, beta=15.0, x_r=-170.0, c=300.0, p=50.0):
    s = steering(t, L, a, beta, x_r, c, p)
    if s['Ltr'] < TIE_MIN:
        raise ValueError('The tie rods would be %.1f mm long: shorter than %.0f mm.' % (s['Ltr'], TIE_MIN))
    if abs(s['worst']) > ERR_MAX:
        raise ValueError('The outer wheel misses Ackermann by up to %.3f deg before the inner one reaches %.0f deg: '
                         'more than %.1f deg.' % (s['worst'], INNER_MAX, ERR_MAX))
    if s['mu_min'] < MU_MIN:
        raise ValueError('The tie rod and the arm come within %.1f deg of a straight line (transmission angle): '
                         'below %.0f deg.' % (s['mu_min'], MU_MIN))
    b = s['beta']
    tips = {side: arm_tip(t, a, b, side, 0.0) for side in (1, -1)}

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    fr = place(root, 'Frame')
    xb = x_r - 70.0                                                # the cross member, behind the rack
    f = box(xb - 20, xb + 20, -t / 2 - 15, t / 2 + 15, 55, 75)
    for side in (1, -1):
        union(f, box(xb - 20, 15, side * t / 2 - 15, side * t / 2 + 15, 55, 75))       # side member to the kingpin
        union(f, cyl((0, side * t / 2, -80), (0, side * t / 2, 75), 10))                 # the kingpin
    union(f, box(xb - 20, x_r + 25, -60, 60, -50, -20), box(xb - 20, xb + 20, -15, 15, -50, 75))   # rack support
    add_body(fr.component, f, 'Frame')
    fr.isGrounded = True

    rk = place(root, 'Rack')
    rb = box(x_r - 15, x_r + 15, -c - 15, c + 15, Z_RACK[0], Z_RACK[1])
    for side in (1, -1):
        union(rb, cyl((x_r, side * c, Z_RACK[1] - 0.001), (x_r, side * c, Z_ROD[1]), 7.8))   # pins for the tie rods
    add_body(rk.component, rb, 'Rack')

    knuckles, rods = {}, {}
    for side, name in ((1, 'L'), (-1, 'R')):
        k = place(root, 'Knuckle ' + name)
        K = (0.0, side * t / 2)
        kb = cyl((K[0], K[1], -50), (K[0], K[1], 50), 25)                                # the boss round the kingpin
        union(kb, bar(K, tips[side], 24, Z_ARM[0], Z_ARM[1]))                            # the steering arm
        union(kb, cyl((tips[side][0], tips[side][1], Z_ROD[0]), (tips[side][0], tips[side][1], Z_ARM[0] + 0.001), 7.8))
        union(kb, cyl((K[0], K[1] + side * 24, 0), (K[0], K[1] + side * 200, 0), 20))   # the stub axle
        union(kb, cyl((K[0], K[1] + side * 200, 0), (K[0], K[1] + side * 320, 0), 280))  # the wheel
        cut(kb, cyl((K[0], K[1], -51), (K[0], K[1], 51), 10.2))                          # the kingpin's bore
        add_body(k.component, kb, 'Knuckle ' + name)
        knuckles[side] = k
        r_ = place(root, 'Tie rod ' + name)
        J = (x_r, side * c)
        tb = bar(J, tips[side], 20, Z_ROD[0], Z_ROD[1])
        cut(tb, cyl((J[0], J[1], Z_ROD[0] - 1), (J[0], J[1], Z_ROD[1] + 1), 8.0),
            cyl((tips[side][0], tips[side][1], Z_ROD[0] - 1), (tips[side][0], tips[side][1], Z_ROD[1] + 1), 8.0))
        add_body(r_.component, tb, 'Tie rod ' + name)
        rods[side] = r_

    rp = p / (2 * math.pi)                                                             # the pinion's pitch radius
    zp = Z_RACK[1] + rp + 0.2
    pn = place(root, 'Pinion', x_axis=(0, 1, 0), z_axis=(1, 0, 0), origin=(x_r, 0, zp))
    pb = cyl((0, 0, -25), (0, 0, 25), rp)                                              # in the pinion's frame
    union(pb, cyl((0, 0, 25), (0, 0, 200), 6))                                         # the column's stub
    add_body(pn.component, pb, 'Pinion')

    jr = joint(root, rk, fr, (x_r, 0, 0), 'slider', 'Rack slides')
    jk = {}
    for side, name in ((1, 'L'), (-1, 'R')):
        jk[side] = joint(root, knuckles[side], fr, (0, side * t / 2, 0), 'revolute', 'Knuckle %s turns' % name)
        joint(root, rods[side], knuckles[side], (tips[side][0], tips[side][1], 20), 'revolute',
              'Tie rod %s on arm' % name)
        joint(root, rods[side], rk, (x_r, side * c, 20), 'revolute', 'Tie rod %s on rack' % name)   # closes a loop
    jp, sp = revolute_about(root, pn, fr, (1, 0, 0), 'Pinion turns')
    jr.jointMotion.slideValue = 0.1                                # which way the rack's joint goes
    sr = 1 if rk.transform2.translation.y > 0 else -1
    jr.jointMotion.slideValue = 0.0
    li = root.motionLinks.createInput(jp, jr)
    li.valueOne = VS('360 deg')                                    # one turn of the pinion ...
    li.valueTwo = VI(p * CM)                                       # ... moves the rack p
    li.isReversed = sp * sr < 0                                    # a turn about +x moves the rack towards +y
    root.motionLinks.add(li).name = 'Pinion and rack'

    colour(design, fr, 'Paint - Enamel Glossy (Grey)')
    colour(design, rk, 'Paint - Enamel Glossy (Yellow)')
    colour(design, pn, 'Paint - Enamel Glossy (Blue)')
    for side in (1, -1):
        colour(design, rods[side], 'Paint - Enamel Glossy (Red)')
    app.activeViewport.fit()

    return ('Rack-and-pinion steering built: track %.0f mm, wheelbase %.0f mm, arms %.0f mm at %.2f deg, rack at '
            'x = %.0f mm with its ball joints %.0f mm apart.\n\n'
            'Tie rods %.3f mm. Rack travel to %.0f deg of inner steer: %.1f mm (outer wheel %.3f deg).\n'
            'Worst Ackermann error %.3f deg; smallest transmission angle %.2f deg.\n'
            'Steering ratio %.2f : 1 at straight ahead; %.2f turns of the pinion lock to lock.\n\n'
            'Drive Joints on "Pinion turns": the rack and both wheels follow.\n\n[%s]'
            % (t, L, a, beta, x_r, 2 * c, s['Ltr'], INNER_MAX, s['u_max'], s['outer_at_max'], s['worst'],
               s['mu_min'], s['ratio'], s['turns'], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('t', 'Track (kingpin to kingpin)', 'mm', 1400.0), ('L', 'Wheelbase', 'mm', 2500.0),
          ('a', 'Steering arm', 'mm', 150.0), ('beta', 'Arm angle to the car axis', 'deg', 15.0),
          ('x_r', 'Rack position (x)', 'mm', -170.0), ('c', 'Half spacing of the rack ball joints', 'mm', 300.0),
          ('p', 'Rack travel per pinion turn', 'mm', 50.0)]


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
        cid = 'SteeringRackCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Rack-and-pinion steering', 'Steering with Ackermann')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Steering could not start:\n' + traceback.format_exc())
