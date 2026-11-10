# =============================================================================
# BallScrewAxis.py  -  Chapter 21 of "Mechanical Design in Autodesk Fusion with Claude": a linear axis on a ball
# screw. The screw's groove and the nut's are the same helical tube, cut by sweeping its section square to the axis
# with one turn of twist per lead; three circuits of balls sit on the tube's centre line. Written from PROMPT.txt
# beside it; checked by extras/checks/ball_screw_check.py.
#
#   INPUTS        the dialog: d_s, P, D_w, F, C_dyn, n
#   DERIVED       ballscrew(): d_m, R_g, d_k, lands, balls per turn, T_a, T_c, L, L_h, F_k, n_k;
#                 groove_section(): the points closer than R_g to the helix, in the plane square to the axis
#   HARD RULES    ballscrew() refuses d_s / D_w outside 3-10, a land under 1 mm, L_h under 10 000 h, 0.5 F_k < 2 F
#   CONVENTIONS   a new design; parts at the origin, drawn in place; the groove one closed polyline swept with a
#                 twist; the balls one body; As-built Joints, the screw's direction tried and its sign measured;
#                 a revolute-to-slider Motion Link
#   ACCEPTANCE    the message; -P / 12 per 30 deg step of the screw; no interference at rest or at any step
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'BallScrewAxis 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
H = 30.0                                   # mm, height of the screw's axis above the base plate
LG = 150.0                                 # mm, the grooved length, centred on x = 0
GAP = 20.0                                 # mm, from each end of the groove to a bearing block
END = 12.0                                 # mm, thickness of each bearing block
DJ, DR = 12.0, 12.0                        # mm, journal and guide rod diameters
ROD_Y = 40.0                               # mm, the guide rod's y
FLANGE_D, FLANGE_T, NUT_D, NUT_L = 54.0, 12.0, 30.0, 46.0
CIRCUITS = 3


def ballscrew(d_s=16.0, P=5.0, D_w=3.175, F=500.0, C_dyn=7310.0, n=1000.0):
    if not 3.0 <= d_s / D_w <= 10.0:
        raise ValueError('d_s / D_w = %.2f: keep it from 3 to 10.' % (d_s / D_w))
    d_m = d_s + 0.6
    R_g = 0.52 * D_w
    land = P - 2 * R_g
    if land < 1.0:
        raise ValueError('The land between grooves is %.2f mm (P - 2 R_g): at least 1 mm is needed%s.'
                         % (land, ': the grooves overlap' if land < 0 else ''))
    turn = math.hypot(math.pi * d_m, P)
    per_turn = int(turn / (1.03 * D_w))
    L = (C_dyn / (1.2 * F)) ** 3 * 1e6
    L_h = L / (60.0 * n)
    if L_h < 10000.0:
        raise ValueError('Rating life L_h = %.0f h is below 10 000 h. Choose a nut with a larger C_dyn.' % L_h)
    d_k = d_m - 2 * R_g
    l_s = LG + 2 * GAP
    F_k = 4.072e5 * 0.25 * d_k ** 4 / l_s ** 2
    if 0.5 * F_k < 2 * F:
        raise ValueError('Buckling: 0.5 F_k = %.0f N is less than twice the load.' % (0.5 * F_k))
    return dict(d_s=d_s, P=P, D_w=D_w, d_m=d_m, R_g=R_g, d_k=d_k, land=land, r_land=(d_s - 0.4) / 2,
                r_bore=(d_s + 1.6) / 2, per_turn=per_turn, balls=per_turn * CIRCUITS, turn=turn,
                T_a=F * P / (2 * math.pi * 0.90), T_c=F * P * 0.80 / (2 * math.pi), L=L, L_h=L_h, l_s=l_s, F_k=F_k,
                n_k=2.71e8 * 0.446 * d_k / l_s ** 2)


def _dist(rho, th, r_c, k):
    """Distance from the point (rho, th) of the plane x = 0 to the helix (r_c cos s, r_c sin s, k s), k = P / 2 pi:
    the nearest s lies between 0 and th, where g(s) = -rho r_c sin(th - s) + k^2 s changes sign."""
    a, b = (0.0, th) if th >= 0 else (th, 0.0)
    g = lambda s: -rho * r_c * math.sin(th - s) + k * k * s
    ga = g(a)
    for _ in range(60):
        m = (a + b) / 2
        if (g(m) > 0) == (ga > 0):
            a, ga = m, g(m)
        else:
            b = m
    s = (a + b) / 2
    return math.sqrt(max(rho * rho + r_c * r_c - 2 * rho * r_c * math.cos(th - s) + (k * s) ** 2, 0.0))


def _span(th, r_c, R, k):
    """(rho_in, rho_out) where the plane at angle th cuts the tube, or None."""
    lo, hi = r_c - R, r_c + R
    for _ in range(60):                                            # the rho nearest the helix: golden section
        m1, m2 = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        if _dist(m1, th, r_c, k) < _dist(m2, th, r_c, k):
            hi = m2
        else:
            lo = m1
    mid = (lo + hi) / 2
    if _dist(mid, th, r_c, k) >= R:
        return None

    def edge(inside, outside):
        for _ in range(60):
            m = (inside + outside) / 2
            if _dist(m, th, r_c, k) < R:
                inside = m
            else:
                outside = m
        return (inside + outside) / 2
    return edge(mid, r_c - R - 1.0), edge(mid, r_c + R + 1.0)


def groove_section(r_c, R, P, phase, n=120):
    """The tube's section square to the axis: (u, v) in mm round the axis, u along +y, v along +z; the helix crosses
    the angle `phase` in this plane. A closed polyline: the inner edge one way, the outer edge back."""
    k = P / (2 * math.pi)
    lo, hi = 0.0, math.pi                                          # the tube's angular half-width in the plane
    for _ in range(60):
        m = (lo + hi) / 2
        if _span(m, r_c, R, k):
            lo = m
        else:
            hi = m
    tmax = lo
    ths = [-tmax * math.cos(math.pi * i / n) for i in range(1, n)]
    spans = [_span(t, r_c, R, k) for t in ths]
    tip = lambda t: sum(_span(t * 0.99999, r_c, R, k)) / 2
    pts = [(tip(-tmax), -tmax)] + [(sp[0], t) for t, sp in zip(ths, spans)] + [(tip(tmax), tmax)]
    pts += [(sp[1], t) for t, sp in reversed(list(zip(ths, spans)))]
    return [(rho * math.cos(t + phase), rho * math.sin(t + phase)) for rho, t in pts]


def ball_centres(k):
    """Three circuits of one turn each, centred on x = 0 at the start; every centre on the helix (angle 2 pi x / P)."""
    out = []
    x0 = -CIRCUITS * k['P'] / 2.0
    for j in range(k['balls']):
        s = 2 * math.pi * x0 / k['P'] + 2 * math.pi * j / k['per_turn']
        out.append((k['P'] * s / (2 * math.pi), k['d_m'] / 2 * math.cos(s), k['d_m'] / 2 * math.sin(s)))
    return out


# ---------- Fusion building blocks (mm at the interface) ----------
def new_part(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def cross_plane(comp, x_mm):
    """A plane square to the screw's axis at x."""
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.yZConstructionPlane, VI(x_mm * CM))
    return comp.constructionPlanes.add(ci)


def box(comp, x0, x1, y0, y1, z0, z1, op=NEW, body=None):
    sk = comp.sketches.add(plane_at(comp, z0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(x0 * CM, y0 * CM, 0), P3(x1 * CM, y1 * CM, 0))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), op)
    inp.setDistanceExtent(False, VI((z1 - z0) * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def along_x(comp, x0, x1, y, z, r, op=NEW, body=None, r_in=0.0):
    """A cylinder (or a tube, or a hole) of radius r along x from x0 to x1, its axis at (y, z)."""
    sk = comp.sketches.add(cross_plane(comp, x0))
    c = sk.modelToSketchSpace(P3(x0 * CM, y * CM, z * CM))
    sk.sketchCurves.sketchCircles.addByCenterRadius(c, r * CM)
    if r_in > 0:
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, r_in * CM)
        prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    else:
        prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI((x1 - x0) * CM)),
                         adsk.fusion.ExtentDirections.PositiveExtentDirection)
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def groove(comp, body, k, x0, x1):
    """The ball groove cut into `body` from x0 to x1: the tube's section at x0 swept to x1, turning once per lead
    (right-hand: positive about +x)."""
    section = groove_section(k['d_m'] / 2, k['R_g'], k['P'], 2 * math.pi * x0 / k['P'])
    sk = comp.sketches.add(cross_plane(comp, x0))
    sk.isComputeDeferred = True
    pts = [sk.modelToSketchSpace(P3(x0 * CM, u * CM, (H + v) * CM)) for u, v in section]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    ps = comp.sketches.add(plane_at(comp, H))
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(P3(x0 * CM, 0, 0), P3(x1 * CM, 0, 0))
    ps.isVisible = False
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False), CUT)
    inp.twistAngle = VI(2 * math.pi * (x1 - x0) / k['P'])
    inp.participantBodies = [body]
    return sw.add(inp)


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_about(root, moving, fixed, point, axis, name):
    """An As-built revolute joint through the world `point` that turns `moving` about the world `axis`: the joint
    directions are tried and the one that really turns the part about `axis` is kept. Returns (joint, sign)."""
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.XAxisJointDirection, Jd.ZAxisJointDirection, Jd.YAxisJointDirection):
        sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
        sk.isVisible = False
        pt = sk.sketchPoints.add(P3(*[v * CM for v in point])).createForAssemblyContext(moving)
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.2
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][c] * r0[b][c] for c in range(3)) for b in range(3)] for a in range(3)]
        w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        dot = sum(w[i] * axis[i] for i in range(3))
        if abs(dot) > 0.1:
            j.name = name
            return j, (1 if dot > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about %s' % (moving.name, axis))


def as_built(root, moving, fixed, kind, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, H * CM)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    if kind == 'slider':
        inp.setAsSliderJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the axis ----------
def build(d_s=16.0, P=5.0, D_w=3.175, F=500.0, C_dyn=7310.0, n=1000.0):
    k = ballscrew(d_s, P, D_w, F, C_dyn, n)
    x_in = LG / 2 + GAP                                            # the bearing blocks' inner faces
    travel = LG / 2 - NUT_L / 2 - 2.0
    y0, y1, top = -FLANGE_D / 2 - 8, ROD_Y + DR / 2 + 10, H + FLANGE_D / 2 + 6

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    frame = new_part(root, 'Frame')
    c = frame.component
    body = box(c, -x_in - END, x_in + END, y0, y1, -6.0, 0.0).bodies.item(0)
    body.name = 'Frame'
    for xa, xb in ((-x_in - END, -x_in), (x_in, x_in + END)):
        box(c, xa, xb, y0, y1, 0.0, top, JOIN, body)
    along_x(c, -x_in - END, x_in + END, 0.0, H, (DJ + 0.2) / 2, CUT, body)           # the journals' bores
    along_x(c, -x_in - END, x_in + END, ROD_Y, H, DR / 2, CUT, body)                 # the guide rod's bores
    frame.isGrounded = True

    rod = new_part(root, 'Guide rod')
    along_x(rod.component, -x_in - END, x_in + END, ROD_Y, H, DR / 2).bodies.item(0).name = 'Guide rod'

    sc = new_part(root, 'Screw')
    c = sc.component
    body = along_x(c, -x_in - END, x_in + END, 0.0, H, DJ / 2).bodies.item(0)        # journals, end to end
    body.name = 'Screw'
    along_x(c, -LG / 2, LG / 2, 0.0, H, k['r_land'], JOIN, body)                     # the grooved part
    groove(c, body, k, -LG / 2, LG / 2)

    nut = new_part(root, 'Nut')
    c = nut.component
    xa = -NUT_L / 2
    body = along_x(c, xa, xa + FLANGE_T, 0.0, H, FLANGE_D / 2).bodies.item(0)
    body.name = 'Nut'
    along_x(c, xa + FLANGE_T, -xa, 0.0, H, NUT_D / 2, JOIN, body)
    along_x(c, xa, -xa, 0.0, H, k['r_bore'], CUT, body)                               # the bore
    groove(c, body, k, xa, -xa)

    balls = new_part(root, 'Balls')
    tbm = adsk.fusion.TemporaryBRepManager.get()
    ball = None
    for x, y, z in ball_centres(k):
        s = tbm.createSphere(P3(x * CM, y * CM, (H + z) * CM), k['D_w'] / 2 * CM)
        if ball is None:
            ball = s
        else:
            tbm.booleanOperation(ball, s, adsk.fusion.BooleanTypes.UnionBooleanType)
    bf = balls.component.features.baseFeatures.add()
    bf.startEdit()
    balls.component.bRepBodies.add(ball, bf)
    bf.finishEdit()
    bf.bodies.item(0).name = 'Balls'

    car = new_part(root, 'Carriage')
    c = car.component
    body = box(c, xa - 10.0, xa, y0 + 4, y1 - 4, 4.0, top - 4).bodies.item(0)
    body.name = 'Carriage'
    along_x(c, xa - 10.0, xa, 0.0, H, k['r_bore'] + 1.0, CUT, body)                  # clear of the screw
    along_x(c, xa - 10.0, xa, ROD_Y, H, (DR + 0.4) / 2, CUT, body)                   # the guide's bore

    as_built(root, rod, frame, 'rigid', 'Guide rod fixed')
    js, sign = revolute_about(root, sc, frame, (0, 0, H), (1, 0, 0), 'Screw turns')
    jc = as_built(root, car, frame, 'slider', 'Carriage slides')
    lim = jc.jointMotion.slideLimits
    lim.isMinimumValueEnabled, lim.minimumValue = True, -travel * CM
    lim.isMaximumValueEnabled, lim.maximumValue = True, travel * CM
    as_built(root, nut, car, 'rigid', 'Nut on carriage')
    as_built(root, balls, nut, 'rigid', 'Balls in nut')
    li = root.motionLinks.createInput(js, jc)
    li.valueOne = VS('360 deg')                                    # one turn of the screw ...
    li.valueTwo = VI(k['P'] * CM)                                  # ... moves the carriage one lead
    li.isReversed = sign > 0                                       # towards -x for a positive turn about +x
    root.motionLinks.add(li).name = 'Lead %.1f mm' % k['P']
    colour(design, frame, 'Paint - Enamel Glossy (Grey)')
    colour(design, sc, 'Steel - Satin')
    colour(design, nut, 'Paint - Enamel Glossy (Blue)')
    colour(design, balls, 'Paint - Enamel Glossy (Red)')
    colour(design, car, 'Paint - Enamel Glossy (Yellow)')
    app.activeViewport.fit()

    return ('Ball screw axis built: d_s = %.1f mm, lead P = %.1f mm, balls of %.3f mm.\n\n'
            'd_m = %.2f, R_g = %.3f, d_k = %.3f mm; land %.2f mm; %d balls a turn, %d in all\n'
            'Against F = %.0f N: T_a = %.2f N.mm to push, T_c = %.2f N.mm back (NOT self-locking: a brake is needed)\n'
            'Rating life L = %.4e revolutions, L_h = %.0f h at %.0f rpm\n'
            'Between the bearing blocks l_s = %.0f mm: F_k = %.0f N, n_k = %.0f rpm\n'
            'Carriage travel plus and minus %.1f mm; %.2f mm per screw turn, towards -x for a positive turn\n\n[%s]'
            % (k['d_s'], k['P'], k['D_w'], k['d_m'], k['R_g'], k['d_k'], k['land'], k['per_turn'], k['balls'], F,
               k['T_a'], k['T_c'], k['L'], k['L_h'], n, k['l_s'], k['F_k'], k['n_k'], travel, k['P'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(d_s=ins.itemById('d_s').value / CM, P=ins.itemById('P').value / CM,
                     D_w=ins.itemById('D_w').value / CM, F=ins.itemById('F').value,
                     C_dyn=ins.itemById('C_dyn').value, n=ins.itemById('n').value)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('d_s', 'Nominal diameter', 'mm', VI(16.0 * CM))
        ins.addValueInput('P', 'Lead', 'mm', VI(5.0 * CM))
        ins.addValueInput('D_w', 'Ball diameter', 'mm', VI(3.175 * CM))
        ins.addValueInput('F', 'Axial load (N)', '', VI(500.0))
        ins.addValueInput('C_dyn', 'Dynamic load rating of the nut (N)', '', VI(7310.0))
        ins.addValueInput('n', 'Average speed (rpm)', '', VI(1000.0))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'BallScrewAxisCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Ball screw axis', 'A linear axis on a ball screw')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Ball screw axis could not start:\n' + traceback.format_exc())
