# =============================================================================
# WormWheel.py  -  Chapter 18 of "Mechanical Design in Autodesk Fusion with Claude": a single-start Archimedean worm
# driving a worm wheel at right angles, assembled and ready to turn with Drive Joints. The worm's thread is its
# transverse section swept along its axis with a twist; the wheel is a helical gear of the worm's lead angle.
# Written from PROMPT.txt beside it; checked by checks/worm_wheel_check.py.
#
#   INPUTS        the dialog: m, q, z2, alpha_x, b, starts
#   DERIVED       worm(): d1, L, lambda, d2, a, ratio, phi_n, the efficiencies; the wheel's face width b'
#   HARD RULES    build() refuses an undercut wheel and a thread that leaves no space; the wheel turned so that a
#                 gap meets the thread at the mid-plane; the wheel the same hand as the worm
#   CONVENTIONS   components created where they belong, local z along each axis; the thread and the wheel's teeth
#                 swept with a twist (kept by Fusion in tenths of a degree); As-built Joints with the direction tried
#                 and the sign measured; a Motion Link at the ratio
#   ACCEPTANCE    the message; one worm turn turns the wheel by 360 / ratio; no collision through one worm turn
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'WormWheel 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
BACKLASH = 0.2                             # mm, on the pitch line and the pitch circle
DS = 8.0                                   # mm, both shafts
BORE_CLEAR = 0.2                           # mm, each bore is this much larger than its shaft
LEADS = 5                                  # the worm is this many leads long
Z_MIN = 18


def worm(m, q, z2, alpha_x_deg, starts):
    ax = math.radians(alpha_x_deg)
    d1 = q * m
    lam = math.atan(starts / q)
    phi_n = math.atan(math.tan(ax) * math.cos(lam))
    eff = lambda f: (math.cos(phi_n) - f * math.tan(lam)) / (math.cos(phi_n) + f / math.tan(lam))
    back = lambda f: (math.cos(phi_n) - f / math.tan(lam)) / (math.cos(phi_n) + f * math.tan(lam))
    return dict(d1=d1, L=starts * math.pi * m, lam=lam, d2=m * z2, a=(d1 + m * z2) / 2.0, ratio=z2 / starts,
                phi_n=phi_n, eff05=eff(0.05), eff10=eff(0.10), back10=back(0.10), f_lock=math.cos(phi_n) * math.tan(lam))


# ---------- the sections (all in cm and radians) ----------
def inv(a):
    return math.tan(a) - a


def involute_pt(rb, t):
    return (rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t)))


def rotate(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def wheel_tooth(m, z, at, backlash, phase, n=8):
    """Chapter 12's tooth, the wheel's transverse section: module m and pressure angle at in the transverse plane."""
    r = m * z / 2.0
    rb, ra, rf = r * math.cos(at), r + m, r - 1.25 * m
    r_lo = max(rb, rf)
    t_at = lambda rr: 0.0 if rr <= rb else math.sqrt((rr / rb) ** 2 - 1.0)
    half = (math.pi * m / 2.0 - backlash / 2.0) / (2.0 * r)
    rot = half + inv(at)
    right, left = [], []
    for i in range(n + 1):
        t = t_at(r_lo) + (t_at(ra) - t_at(r_lo)) * i / n
        x, y = involute_pt(rb, t)
        right.append(rotate([(x, -y)], rot)[0])
        left.append(rotate([(x, y)], -rot)[0])
    a_l, a_r = math.atan2(left[0][1], left[0][0]), math.atan2(right[0][1], right[0][0])
    r_in = rf * 0.985
    pts = [(r_in * math.cos(a_r), r_in * math.sin(a_r)), (r_in * math.cos(a_l), r_in * math.sin(a_l))]
    if r_lo > r_in + 1e-9:
        pts.append((r_lo * math.cos(a_l), r_lo * math.sin(a_l)))
    pts.extend(left[1:] if r_lo <= r_in + 1e-9 else left)
    a0, a1 = math.atan2(left[-1][1], left[-1][0]), math.atan2(right[-1][1], right[-1][0])
    pts.extend((ra * math.cos(a0 + (a1 - a0) * j / 5), ra * math.sin(a0 + (a1 - a0) * j / 5)) for j in range(1, 5))
    pts.extend(reversed(right))
    clean = [pts[0]]
    for q in pts[1:]:
        if math.hypot(q[0] - clean[-1][0], q[1] - clean[-1][1]) > 1e-7:
            clean.append(q)
    return rotate(clean, phase), rf


def thread_section(m, r1, lead, ax, backlash, phase, n=10):
    """The worm's thread cut square to its axis. The axial profile is straight-sided (an Archimedean worm): at radius
    rho the thread is hw(rho) wide either side of its centre line along the axis, and a point of the centre line at
    angle phi lies lead * phi / (2 pi) along the axis. So, in the plane of the cut, the thread fills the angles
    |phi| <= 2 pi hw(rho) / lead: a lobe between two spirals, closed inside the core."""
    rf, ra = r1 - 1.25 * m, r1 + m
    hw = lambda rho: (math.pi * m / 2.0 - backlash / 2.0) / 2.0 - (rho - r1) * math.tan(ax)
    ang = lambda rho: 2 * math.pi * hw(rho) / lead
    r_in = rf * 0.985
    flank = [rf + (ra - rf) * i / n for i in range(n + 1)]
    pts = [(r_in, -ang(rf))]
    pts += [(rho, -ang(rho)) for rho in flank]
    pts += [(ra, -ang(ra) + 2 * ang(ra) * j / 6) for j in range(1, 6)]
    pts += [(rho, ang(rho)) for rho in reversed(flank)]
    pts += [(r_in, ang(rf))]
    pts += [(r_in, ang(rf) - 2 * ang(rf) * j / 24) for j in range(1, 24)]     # back round the core, through 0
    xy = [(rho * math.cos(p), rho * math.sin(p)) for rho, p in pts]
    return rotate(xy, phase), rf


def polyline(sk, pts):
    """One closed polyline whose lines share their sketch points."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


# ---------- Fusion building blocks (mm at the interface) ----------
def place(root, name, origin=(0, 0, 0), x_axis=(1, 0, 0), z_axis=(0, 0, 1)):
    """A component created where it belongs: origin, local z along z_axis, local x along x_axis."""
    zx, zy, zz = z_axis
    xx, xy, xz = x_axis
    y_axis = (zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx)          # y = z x x
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P3(*[v * CM for v in origin]), V3(*x_axis), V3(*y_axis), V3(*z_axis))
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, prof, length_mm, op=NEW, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def tube(comp, z0, length, r_out, r_in, name=None):
    """A cylinder (r_in = 0) or a tube along the local z axis, from z0 for `length` mm, as a new body."""
    sk = comp.sketches.add(plane_at(comp, z0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), r_out * CM)
    if r_in > 0:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), r_in * CM)
        prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    else:
        prof = sk.profiles.item(0)
    body = extrude(comp, prof, length).bodies.item(0)
    if name:
        body.name = name
    return body


def twisted(comp, body, outline, z0, length, twist):
    """The outline at local height z0 swept up the local z axis for `length` mm, turning by `twist` radians."""
    sk = comp.sketches.add(plane_at(comp, z0))
    polyline(sk, outline)
    ps = comp.sketches.add(comp.xYConstructionPlane)
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(P3(0, 0, z0 * CM), P3(0, 0, (z0 + length) * CM))
    ps.isVisible = False
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False), JOIN)
    inp.twistAngle = VI(twist)
    inp.participantBodies = [body]
    return sw.add(inp)


def origin_point(occ):
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(occ)


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_about(root, moving, fixed, axis, name):
    """An As-built revolute joint that turns `moving` about the world `axis`: the X, Y and Z joint directions are
    tried in turn and the one that really turns the part about `axis` is kept. Returns (joint, sign)."""
    J = adsk.fusion.JointDirections
    for direction in (J.ZAxisJointDirection, J.XAxisJointDirection, J.YAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(origin_point(moving)))
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


# ---------- the assembly ----------
def build(m=2.0, q=10.0, z2=30, alpha_x=20.0, b=8.0, starts=1):
    z2, starts = int(round(z2)), int(round(starts))
    w = worm(m, q, z2, alpha_x, starts)
    if z2 < Z_MIN:
        raise ValueError('%d teeth: below %d teeth the wheel is undercut.' % (z2, Z_MIN))
    if w['lam'] > math.radians(25):
        raise ValueError('Lead angle %.1f deg: more than 25 degrees; use more teeth or a larger q.' % math.degrees(w['lam']))
    ax, lam, r1, r2 = math.radians(alpha_x), w['lam'], w['d1'] / 2, w['d2'] / 2
    # the wheel's teeth turn by b tan(lambda) / r2 over the face width; Fusion keeps a twist in tenths of a degree,
    # so the face width is chosen to make it exact (Chapter 14)
    step = math.radians(0.1)
    b_asked, b = b, round(b * math.tan(lam) / r2 / step) * step * r2 / math.tan(lam)
    theta = b * math.tan(lam) / r2
    Lw = LEADS * w['L']                                            # the worm's length
    z_mid = b / 2.0                                                # the worm's axis lies in the wheel's mid-plane

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    worm_frame = dict(origin=(0, -w['a'], z_mid), x_axis=(0, 1, 0), z_axis=(1, 0, 0))  # local z along +X, local x at the wheel
    wshaft = place(root, 'Worm shaft', **worm_frame)
    tube(wshaft.component, -Lw / 2 - 10, Lw + 20, DS / 2, 0, 'Worm shaft')
    wshaft.isGrounded = True
    gshaft = place(root, 'Wheel shaft')
    tube(gshaft.component, -5.0, b + 10.0, DS / 2, 0, 'Wheel shaft')
    gshaft.isGrounded = True

    wm = place(root, 'Worm', **worm_frame)
    c = wm.component
    rf = r1 - 1.25 * m
    core = tube(c, -Lw / 2, Lw, rf, (DS + BORE_CLEAR) / 2, 'Worm')
    # the thread, centred on local x (towards the wheel) at the middle of the worm: at the start face, LEADS / 2 leads
    # earlier, it is turned back by LEADS * pi; a positive twist of LEADS turns is a right-hand thread
    outline, _ = thread_section(m * CM, r1 * CM, w['L'] * CM, ax, BACKLASH * CM, -LEADS * math.pi)
    twisted(c, core, outline, -Lw / 2, Lw, 2 * math.pi * LEADS)

    wh = place(root, 'Wheel')
    c = wh.component
    # a gap of the wheel faces the thread (direction -y) at the mid-plane; the teeth turn theta over the width,
    # anticlockwise as z rises (right hand, like the worm)
    phase = -math.pi / 2 - math.pi / z2 - theta / 2
    pts, wrf = wheel_tooth(m * CM, z2, ax, BACKLASH * CM, phase)
    wheel_body = tube(c, 0.0, b, wrf / CM, (DS + BORE_CLEAR) / 2, 'Wheel')
    sweep = twisted(c, wheel_body, pts, 0.0, b, theta)
    feats = adsk.core.ObjectCollection.create()
    feats.add(sweep)
    pats = c.features.circularPatternFeatures
    pi = pats.createInput(feats, c.zConstructionAxis)
    pi.quantity = VI(z2)
    pi.totalAngle = VS('360 deg')
    pats.add(pi)

    jw, sw = revolute_about(root, wm, wshaft, (1, 0, 0), 'Worm turns')
    jg, sg = revolute_about(root, wh, gshaft, (0, 0, 1), 'Wheel turns')
    li = root.motionLinks.createInput(jw, jg)
    li.valueOne = VS('%d deg' % (360 * z2))                      # z2 / starts turns of the worm ...
    li.valueTwo = VS('%d deg' % (360 * starts))                  # ... for one turn of the wheel
    li.isReversed = sw * sg > 0                                  # world turns opposite: wheel = -(starts / z2) worm
    root.motionLinks.add(li).name = 'Worm %d:%d' % (starts, z2)
    colour(design, wm, 'Paint - Enamel Glossy (Blue)')
    colour(design, wh, 'Paint - Enamel Glossy (Yellow)')
    app.activeViewport.fit()

    return ('Worm and wheel built: module %.2f, q = %.1f, %d start(s), wheel %d teeth.\n\n'
            'Worm pitch diameter d1 = q m = %.2f mm, lead L = %.4f mm, lead angle lambda = %.4f deg\n'
            'Wheel pitch diameter d2 = %.2f mm; centre distance a = %.2f mm; ratio %.1f\n'
            'Wheel face width %.3f mm (asked %.3f): its teeth turn %.1f deg over it\n'
            'Normal pressure angle %.3f deg; efficiency %.3f at f = 0.05, %.3f at f = 0.10; back-driving at f = 0.10: '
            '%.3f; self-locking above f = %.4f\n\n[%s]'
            % (m, q, starts, z2, w['d1'], w['L'], math.degrees(lam), w['d2'], w['a'], w['ratio'], b, b_asked,
               math.degrees(theta), math.degrees(w['phi_n']), w['eff05'], w['eff10'], w['back10'], w['f_lock'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(m=ins.itemById('m').value / CM, q=ins.itemById('q').value, z2=ins.itemById('z2').value,
                     alpha_x=math.degrees(ins.itemById('alpha_x').value), b=ins.itemById('b').value / CM,
                     starts=ins.itemById('starts').value)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('m', 'Axial module', 'mm', VI(2.0 * CM))
        ins.addValueInput('q', 'Diameter factor q = d1 / m', '', VI(10.0))
        ins.addIntegerSpinnerCommandInput('z2', 'Wheel teeth', 6, 200, 1, 30)
        ins.addValueInput('alpha_x', 'Axial pressure angle', 'deg', VI(math.radians(20.0)))
        ins.addValueInput('b', 'Wheel face width', 'mm', VI(8.0 * CM))
        ins.addIntegerSpinnerCommandInput('starts', 'Worm starts', 1, 4, 1, 1)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'WormWheelCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Worm and wheel', 'A worm driving a wheel at right angles')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Worm and wheel could not start:\n' + traceback.format_exc())
