# =============================================================================
# LeadScrewAxis.py  -  Chapter 21 of "Mechanical Design in Autodesk Fusion with Claude": a linear axis. A trapezoidal
# lead screw turns in a frame; a carriage carrying the nut slides on a guide rod. Both threads are their sections
# swept along the axis with a twist. Written from PROMPT.txt beside it; checked by checks/lead_screw_check.py.
#
#   INPUTS        the dialog: d, P, starts, F, f
#   DERIVED       screw(): d2, d3, D1, D4, L, lambda, the torque, the efficiency, self-locking, the lowering torque
#   HARD RULES    build() refuses a screw outside 8-40 mm, a pitch outside 2-10 mm, a thread deeper than d / 4;
#                 a right-hand thread; the nut's thread half a pitch round; the threads never touch
#   CONVENTIONS   a new design; parts at the origin, drawn in place; each thread one closed polyline swept with a
#                 whole number of turns; cm and radians inside the API; As-built Joints, the screw's direction
#                 tried and its sign measured; a revolute-to-slider Motion Link
#   ACCEPTANCE    the message; -0.25 mm per 30 deg step of the screw; no contact between the threads
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'LeadScrewAxis 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
FLANK = math.radians(15.0)                 # half the 30 deg thread angle
AC = 0.25                                  # mm, clearance at every crest and root
J = 0.2                                    # mm, axial clearance between the threads
H = 20.0                                   # mm, height of the screw's axis above the plate
LEADS = 20                                 # the threaded length, in leads
CAR = 20.0                                 # mm, length of the carriage
END = 10.0                                 # mm, thickness of each end block
DJ, DR = 8.0, 8.0                          # mm, journal and guide rod diameters
ROD_Y = 20.0                               # mm, the guide rod's y


def screw(d, P, starts, F, f):
    L = starts * P
    d2 = d - P / 2.0
    lam = math.atan(L / (math.pi * d2))
    sec = 1.0 / math.cos(FLANK)
    T = F * d2 / 2 * (L + math.pi * f * d2 * sec) / (math.pi * d2 - f * L * sec)
    locks = math.pi * f * d2 * sec > L
    T_low = F * d2 / 2 * (math.pi * f * d2 * sec - L) / (math.pi * d2 + f * L * sec)
    return dict(L=L, d2=d2, d3=d - P - 2 * AC, D1=d - P, D4=d + 2 * AC, lam=lam, T=T, eff=F * L / (2 * math.pi * T),
                locks=locks, T_low=T_low)


def thread_section(r_attach, r_free, r_pitch, P, lead, phase, n=10):
    """A thread cut square to its axis, (u, v) in mm round the axis: at radius rho the thread is h(rho) either side
    of its centre line along the axis, h narrowing towards the free radius at the flank angle; a point of the centre
    line at angle phi lies lead phi / (2 pi) along the axis, so the thread fills |phi| <= 2 pi h(rho) / lead. It is
    attached on the side of r_attach (the screw's core, or the nut's body) and closed by an arc just inside that."""
    s = 1.0 if r_free > r_attach else -1.0                         # +1: an outside thread, -1: an inside one
    h = lambda rho: (P / 2.0 - J / 2.0) / 2.0 - s * (rho - r_pitch) * math.tan(FLANK)
    ang = lambda rho: 2 * math.pi * h(rho) / lead
    r_body = r_attach * (1.0 - 0.015 * s)
    flank = [r_attach + (r_free - r_attach) * i / n for i in range(n + 1)]
    pts = [(r_body, -ang(r_attach))]
    pts += [(rho, -ang(rho)) for rho in flank]
    pts += [(r_free, -ang(r_free) + 2 * ang(r_free) * j / 6) for j in range(1, 6)]
    pts += [(rho, ang(rho)) for rho in reversed(flank)]
    pts += [(r_body, ang(r_attach))]
    pts += [(r_body, ang(r_attach) - 2 * ang(r_attach) * j / 24) for j in range(1, 24)]    # back round, through 0
    return [(rho * math.cos(p + phase), rho * math.sin(p + phase)) for rho, p in pts]


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


def thread(comp, body, section, x0, x1, lead):
    """A thread section (u, v round the axis at y = 0, z = H) on the plane at x0, swept to x1 turning once per lead
    (a right-hand thread: positive about +x)."""
    sk = comp.sketches.add(cross_plane(comp, x0))
    pts = [sk.modelToSketchSpace(P3(x0 * CM, u * CM, (H + v) * CM)) for u, v in section]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    ps = comp.sketches.add(plane_at(comp, H))
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(P3(x0 * CM, 0, 0), P3(x1 * CM, 0, 0))
    ps.isVisible = False
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False), JOIN)
    inp.twistAngle = VI(2 * math.pi * (x1 - x0) / lead)
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
        D = [[sum(r1[a][k] * r0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
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
def build(d=12.0, P=3.0, starts=1, F=500.0, f=0.15):
    starts = int(round(starts))
    if not 8.0 <= d <= 40.0 or not 2.0 <= P <= 10.0:
        raise ValueError('Screw %.1f x %.1f: keep d from 8 to 40 mm and P from 2 to 10 mm.' % (d, P))
    if P / 2.0 > d / 4.0:
        raise ValueError('Thread depth P / 2 = %.2f mm is more than d / 4 = %.2f mm.' % (P / 2.0, d / 4.0))
    s = screw(d, P, starts, F, f)
    L = s['L']
    Ls = LEADS * L                                                 # the threaded length
    x_in = Ls / 2 + 2.0                                            # the end blocks' inner faces
    travel = x_in - CAR / 2 - 2.0

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    frame = new_part(root, 'Frame')
    c = frame.component
    y0, y1, top = -d / 2 - 12, ROD_Y + DR / 2 + 8, H + d / 2 + 8
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
    along_x(c, -Ls / 2, Ls / 2, 0.0, H, s['d3'] / 2, JOIN, body)                      # the core
    for k in range(starts):                                                          # one thread per start
        sec = thread_section(s['d3'] / 2, d / 2, s['d2'] / 2, P, L, 2 * math.pi * (-Ls / 2) / L + 2 * math.pi * k / starts)
        thread(c, body, sec, -Ls / 2, Ls / 2, L)

    car = new_part(root, 'Carriage')
    c = car.component
    body = box(c, -CAR / 2, CAR / 2, y0 + 2, y1 - 2, 4.0, top - 2).bodies.item(0)
    body.name = 'Carriage'
    along_x(c, -CAR / 2, CAR / 2, 0.0, H, s['D4'] / 2, CUT, body)                    # the nut's bore
    along_x(c, -CAR / 2, CAR / 2, ROD_Y, H, (DR + 0.4) / 2, CUT, body)                # the guide's bore
    phase = 2 * math.pi * (-CAR / 2) / L + math.pi / starts                         # half a pitch round
    for k in range(starts):
        sec = thread_section(s['D4'] / 2, s['D1'] / 2, s['d2'] / 2, P, L, phase + 2 * math.pi * k / starts)
        thread(c, body, sec, -CAR / 2, CAR / 2, L)

    as_built(root, rod, frame, 'rigid', 'Guide rod fixed')
    js, sign = revolute_about(root, sc, frame, (0, 0, H), (1, 0, 0), 'Screw turns')
    jc = as_built(root, car, frame, 'slider', 'Carriage slides')
    lim = jc.jointMotion.slideLimits
    lim.isMinimumValueEnabled, lim.minimumValue = True, -travel * CM
    lim.isMaximumValueEnabled, lim.maximumValue = True, travel * CM
    li = root.motionLinks.createInput(js, jc)
    li.valueOne = VS('360 deg')                                    # one turn of the screw ...
    li.valueTwo = VI(L * CM)                                       # ... moves the carriage one lead
    li.isReversed = sign > 0                                       # towards -x for a positive turn about +x
    root.motionLinks.add(li).name = 'Lead %.1f mm' % L
    colour(design, frame, 'Paint - Enamel Glossy (Grey)')
    colour(design, sc, 'Steel - Satin')
    colour(design, car, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    lock = ('self-locking: lowering the load takes %.2f N.mm' % s['T_low']) if s['locks'] else \
           'NOT self-locking: the load drives the screw back (a brake is needed)'
    return ('Lead screw axis built: Tr %.0f x %.0f, %d start(s), lead L = %.2f mm.\n\n'
            'd2 = %.2f, d3 = %.2f, D1 = %.2f, D4 = %.2f mm; lead angle %.3f deg\n'
            'Against F = %.0f N with f = %.2f: T = %.2f N.mm, efficiency %.3f; %s\n'
            'Carriage travel plus and minus %.1f mm; %.2f mm per screw turn, towards -x for a positive turn\n\n[%s]'
            % (d, P, starts, L, s['d2'], s['d3'], s['D1'], s['D4'], math.degrees(s['lam']), F, f, s['T'], s['eff'], lock,
               travel, L, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(d=ins.itemById('d').value / CM, P=ins.itemById('P').value / CM, starts=ins.itemById('starts').value,
                     F=ins.itemById('F').value, f=ins.itemById('f').value)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('d', 'Screw major diameter', 'mm', VI(12.0 * CM))
        ins.addValueInput('P', 'Pitch', 'mm', VI(3.0 * CM))
        ins.addIntegerSpinnerCommandInput('starts', 'Starts', 1, 4, 1, 1)
        ins.addValueInput('F', 'Axial load on the nut (N)', '', VI(500.0))
        ins.addValueInput('f', 'Thread friction coefficient (assumed)', '', VI(0.15))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'LeadScrewAxisCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Lead screw axis', 'A lead screw and a sliding carriage')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Lead screw axis could not start:\n' + traceback.format_exc())
