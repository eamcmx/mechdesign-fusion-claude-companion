# =============================================================================
# LeadScrewAxis.py  -  a linear axis: a trapezoidal lead screw turning in a frame, and a carriage with the nut
# sliding on a guide rod. Drive Joints on the screw ('Screw turns') moves the carriage through a Motion Link.
# The dialog takes d, P, the number of starts, the axial load F and the friction coefficient f; the message
# reports d2, d3, D1, D4, L, the lead angle, the torque to move the load against it, the efficiency and whether
# the screw locks itself (with the torque to lower the load).
#
# Geometry, all in mm here and turned into cm (the API's unit) only where Fusion is called:
#   screw axis      world x at y = 0, z = 20; the threaded length is 20 leads centred on x = 0
#   threads         each one a section square to the axis (ONE closed polyline) swept along the axis with a
#                   twist of one turn per lead; right-hand: the thread fills |phi - 2 pi x / L| <= 2 pi h(rho) / L,
#                   phi measured about +x from +y towards +z; the nut fills the same angles turned by pi / starts
#   clearances      0.25 mm at every crest and root, 0.2 mm along the axis (0.1 mm on each flank)
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'LeadScrewAxis v1.0 (prompt test, 2026-11-11)'
CMD_ID = 'mdfcLeadScrewAxisPromptTest'

CM = 0.1                                  # mm to cm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

ALPHA = math.radians(15.0)                # flank angle (30 deg included)
ZA = 20.0                                 # the screw's axis: y = 0, z = 20
LEADS = 20                                # threaded length, in leads
NUT_LEN = 20.0                            # carriage (and nut) length along x
ROD_D, ROD_Y, ROD_Z = 8.0, 20.0, 20.0     # guide rod
ROD_BORE = ROD_D + 0.2                    # the carriage's bore on the rod (my choice: 0.1 mm a side)
END_GAP = 1.0                             # thread end to the end block's inner face (my choice)
BLOCK_T = 12.0                            # end block thickness along x (my choice)
JOURNAL_IN = 8.0                          # journal length inside the end block's bore
TRAVEL_CLEAR = 2.0                        # the carriage stays this far from the end blocks
STEP = math.radians(3.0)                  # largest angle between two polyline points

_handlers = []


# ---------------------------------------------------------------------------------------------- calculations
def screw(d, P, starts, F, f):
    """The screw's numbers (mm, N, N.mm, radians)."""
    n = int(round(starts))
    L = n * P
    d2 = d - P / 2.0
    d3 = d - P - 0.5
    D1 = d - P
    D4 = d + 0.5
    sec = 1.0 / math.cos(ALPHA)
    lam = math.atan(L / (math.pi * d2))
    T = F * d2 / 2.0 * (L + math.pi * f * d2 * sec) / (math.pi * d2 - f * L * sec)
    eff = F * L / (2.0 * math.pi * T)
    locks = math.pi * f * d2 * sec > L
    T_low = F * d2 / 2.0 * (math.pi * f * d2 * sec - L) / (math.pi * d2 + f * L * sec)
    return dict(d=d, P=P, starts=n, F=F, f=f, L=L, d2=d2, d3=d3, D1=D1, D4=D4, alpha=ALPHA, sec=sec, lam=lam,
                T=T, eff=eff, locks=locks, T_low=T_low, friction_lead=math.pi * f * d2 * sec)


def check_inputs(d, P, starts, F, f):
    """Refuse what the prompt forbids (and what cannot be built), with ValueError."""
    tol = 1e-9
    if not (8.0 - tol <= d <= 40.0 + tol):
        raise ValueError('d = %g mm: the screw major diameter must be from 8 to 40 mm.' % d)
    if not (2.0 - tol <= P <= 10.0 + tol):
        raise ValueError('P = %g mm: the pitch must be from 2 to 10 mm.' % P)
    if P / 2.0 > d / 4.0 + tol:
        raise ValueError('Thread depth P / 2 = %.2f mm is more than d / 4 = %.2f mm: refused.' % (P / 2.0, d / 4.0))
    if starts < 1 or abs(starts - round(starts)) > tol:
        raise ValueError('starts = %g: the number of starts must be a whole number, 1 or more.' % starts)
    if F <= 0.0 or f < 0.0:
        raise ValueError('F = %g N, f = %g: the load must be more than 0 and the friction coefficient not negative.'
                         % (F, f))
    L = round(starts) * P
    d2 = d - P / 2.0
    if math.pi * d2 - f * L / math.cos(ALPHA) <= 0.0:
        raise ValueError('f = %g: with this friction no torque can move the load (pi d2 <= f L sec(alpha)).' % f)


def geometry(d, P, starts):
    """Every size the model needs, in mm."""
    n = int(round(starts))
    L = n * P
    g = dict(n=n, L=L, P=P)
    g['r1'] = d / 2.0                         # screw crest
    g['r2'] = (d - P / 2.0) / 2.0             # pitch radius
    g['r3'] = (d - P - 0.5) / 2.0             # screw root
    g['R1'] = (d - P) / 2.0                   # nut crest
    g['R4'] = (d + 0.5) / 2.0                 # nut root
    g['h2'] = (P / 2.0 - 0.1) / 2.0           # half-thickness on the pitch line, screw and nut alike
    g['x_thread'] = LEADS * L / 2.0           # threaded length from -x_thread to +x_thread
    g['x_in'] = g['x_thread'] + END_GAP       # inner faces of the end blocks
    g['x_out'] = g['x_in'] + BLOCK_T          # outer faces
    g['dj'] = min(8.0, g['r3'] * 2.0 - 0.5)   # journal diameter: 8 mm, or less on a small screw
    g['bore'] = g['dj'] + 0.5                 # the end blocks' bores
    g['x_journal'] = g['x_in'] + JOURNAL_IN   # journal ends
    g['travel'] = g['x_in'] - NUT_LEN / 2.0 - TRAVEL_CLEAR
    # frame and carriage outlines (my choices: walls of 6 mm on the frame, 5 mm on the carriage)
    g['fy0'], g['fy1'] = -(g['R4'] + 6.0), ROD_Y + ROD_D / 2.0 + 6.0
    g['fz1'] = ZA + max(g['R4'], ROD_D / 2.0) + 6.0
    g['cy0'], g['cy1'] = -(g['R4'] + 5.0), ROD_Y + ROD_BORE / 2.0 + 5.0
    g['cz0'], g['cz1'] = ZA - (g['R4'] + 5.0), ZA + (g['R4'] + 5.0)
    return g


def h_screw(g, rho):
    """Half-thickness of the screw's thread along the axis at radius rho (narrower towards the crest)."""
    return g['h2'] - (rho - g['r2']) * math.tan(ALPHA)


def h_nut(g, rho):
    """Half-thickness of the nut's thread along the axis at radius rho (narrower towards its crest, inside)."""
    return g['h2'] + (rho - g['r2']) * math.tan(ALPHA)


def star(r_b, r_t, n, psi0, hw):
    """A closed outline (rho, phi), phi rising all the way round: n lobes from radius r_b out to r_t, lobe k
    centred on psi0 + 2 pi k / n, its half-angle hw(rho) at radius rho."""
    pts = []

    def arc(r, a0, a1):
        m = max(1, int(math.ceil(abs(a1 - a0) / STEP)))
        return [(r, a0 + (a1 - a0) * i / m) for i in range(1, m)]

    m = max(6, int(math.ceil(abs(hw(r_b) - hw(r_t)) / STEP)))
    rs = [r_b + (r_t - r_b) * i / m for i in range(m + 1)]
    for k in range(n):
        c = psi0 + 2.0 * math.pi * k / n
        pts += [(r, c - hw(r)) for r in rs]                              # out along one flank
        pts += arc(r_t, c - hw(r_t), c + hw(r_t))                       # over the lobe's tip
        pts += [(r, c + hw(r)) for r in reversed(rs)]                    # back down the other flank
        pts += arc(r_b, c + hw(r_b), c + 2.0 * math.pi / n - hw(r_b))   # along the base to the next lobe
    return pts


def screw_section(g, x):
    """The screw's section square to the axis at x: the core to the root and the thread lobes to the crest."""
    L, n = g['L'], g['n']
    return star(g['r3'], g['r1'], n, 2.0 * math.pi * x / L, lambda r: 2.0 * math.pi * h_screw(g, r) / L)


def nut_hole_section(g, x):
    """The nut's hole at x: inside the nut's crest everywhere, out to its root between the nut's thread lobes.
    The nut's lobes are centred on pi / n + 2 pi x / L, so the hole's lobes sit on the screw's (2 pi x / L)."""
    L, n = g['L'], g['n']
    return star(g['R1'], g['R4'], n, 2.0 * math.pi * x / L,
                lambda r: math.pi / n - 2.0 * math.pi * h_nut(g, r) / L)


def to_world(x, pts):
    """(rho, phi) about the screw's axis at x to world (x, y, z) in mm."""
    return [(x, r * math.cos(a), ZA + r * math.sin(a)) for r, a in pts]


# ---------------------------------------------------------------------------------------------- Fusion helpers
def _new_occ(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def _pt(sk, x, y, z):
    return sk.modelToSketchSpace(P3(x * CM, y * CM, z * CM))


def _x_plane(comp, x):
    """A construction plane square to x at x (mm), its place checked."""
    if abs(x) < 1e-12:
        return comp.yZConstructionPlane
    for s in (1.0, -1.0):
        ci = comp.constructionPlanes.createInput()
        ci.setByOffset(comp.yZConstructionPlane, VI(s * x * CM))
        pl = comp.constructionPlanes.add(ci)
        if abs(pl.geometry.origin.x - x * CM) < 1e-7:
            pl.isLightBulbOn = False
            return pl
        pl.deleteMe()
    raise RuntimeError('could not place a plane at x = %g mm' % x)


def _polyline(sk, world_pts):
    """ONE closed polyline through the world points (mm)."""
    clean = [world_pts[0]]
    for q in world_pts[1:]:
        if math.dist(q, clean[-1]) > 1e-6:
            clean.append(q)
    if math.dist(clean[0], clean[-1]) < 1e-6:
        clean.pop()
    lines = sk.sketchCurves.sketchLines
    sk.isComputeDeferred = True
    q = [_pt(sk, *p) for p in clean]
    first = lines.addByTwoPoints(q[0], q[1])
    prev = first.endSketchPoint
    for p in q[2:]:
        prev = lines.addByTwoPoints(prev, p).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    return sk


def _axis_line(comp, x0, x1):
    """A sketch line along the screw's axis from x0 to x1 (mm)."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    ln = sk.sketchCurves.sketchLines.addByTwoPoints(_pt(sk, x0, 0, ZA), _pt(sk, x1, 0, ZA))
    sk.isVisible = False
    return ln


def _circle_profile(comp, y, z, dia):
    """A circle square to x on the YZ plane (x = 0); returns its profile."""
    sk = comp.sketches.add(comp.yZConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(sk, 0, y, z), dia / 2.0 * CM)
    return sk.profiles.item(0)


def _extrude_sym(comp, prof, length, op, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setSymmetricExtent(VI(length * CM), True)
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def _inside(body, x, rho, phi):
    p = P3(x * CM, rho * math.cos(phi) * CM, (ZA + rho * math.sin(phi)) * CM)
    return body.pointContainment(p) == adsk.fusion.PointContainment.PointInsidePointContainment


MODES = ((1.0, False), (-1.0, False), (1.0, True), (-1.0, True))   # (twist sign, path drawn from x1 to x0)


def _twisted_sweep(comp, sk, x0, x1, twist_deg, op, first, right_hand, target=None):
    """Sweep the sketch's profile (drawn at x0) along the axis to x1 with a twist of twist_deg. Which way Fusion
    turns a positive twist is not taken on trust: each mode (sign, path direction) is tried, `first` first, and
    the result probed (right_hand(body) is True for a right-hand thread). Returns (feature, body, mode used)."""
    errors = []
    for mode in (first,) + tuple(md for md in MODES if md != first):
        sg, rev = mode
        try:
            line = _axis_line(comp, x1, x0) if rev else _axis_line(comp, x0, x1)
            path = comp.features.createPath(line, False)
            sw = comp.features.sweepFeatures
            inp = sw.createInput(sk.profiles.item(0), path, op)
            inp.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
            inp.twistAngle = VS('%.1f deg' % (sg * twist_deg))
            if target is not None:
                inp.participantBodies = [comp.bRepBodies.item(0)]
            feat = sw.add(inp)
        except Exception as e:
            errors.append('%s: %s' % (mode, e))
            continue
        if feat is None:
            errors.append('%s: the sweep failed' % (mode,))
            continue
        body = feat.bodies.item(0) if target is None else comp.bRepBodies.item(0)
        if right_hand(body):
            return feat, body, mode
        errors.append('%s: not a right-hand thread' % (mode,))
        feat.deleteMe()
    raise RuntimeError('no sweep gave a right-hand thread: ' + '; '.join(errors))


def _rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def _turn_about_x(R1, R0):
    """(angle in deg about +x of R1 R0^T, True when that rotation keeps the x axis)."""
    D = [[sum(R1[a][k] * R0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    return math.degrees(math.atan2(D[2][1] - D[1][2], D[1][1] + D[2][2])), abs(D[0][0] - 1.0) < 1e-6


def _settle():
    for _ in range(3):
        adsk.doEvents()


def _x_mm(occ):
    return occ.transform2.translation.x * 10.0


def _tries(axes):
    D = adsk.fusion.JointDirections
    return [(D.XAxisJointDirection, None), (D.CustomJointDirection, axes[0]), (D.CustomJointDirection, axes[1]),
            (D.ZAxisJointDirection, None), (D.YAxisJointDirection, None)]


def _revolute(root, occ_s, occ_f, geo, axes):
    """'Screw turns', about the screw's axis: each direction tried, the sign of a 30 deg turn measured about +x.
    Returns (joint, sign): sign +1 when a positive joint value turns the screw positively about +x."""
    fallback = None
    for direction, ent in _tries(axes):
        try:
            inp = root.asBuiltJoints.createInput(occ_s, occ_f, geo())
            if ent is None:
                inp.setAsRevoluteJointMotion(direction)
            else:
                inp.setAsRevoluteJointMotion(direction, ent)
            j = root.asBuiltJoints.add(inp)
        except Exception:
            continue
        if j is None:
            continue
        try:
            R0 = _rotation(occ_s)
            j.jointMotion.rotationValue = math.radians(30.0)
            _settle()
            ang, about_x = _turn_about_x(_rotation(occ_s), R0)
            j.jointMotion.rotationValue = 0.0
            _settle()
        except Exception:
            j.deleteMe()
            continue
        if about_x and abs(ang - 30.0) < 1e-3:
            j.name = 'Screw turns'
            return j, 1
        if about_x and abs(ang + 30.0) < 1e-3 and fallback is None:
            fallback = (direction, ent)
        j.deleteMe()
    if fallback is None:
        raise RuntimeError('no direction gave a revolute joint about the screw axis')
    inp = root.asBuiltJoints.createInput(occ_s, occ_f, geo())
    if fallback[1] is None:
        inp.setAsRevoluteJointMotion(fallback[0])
    else:
        inp.setAsRevoluteJointMotion(fallback[0], fallback[1])
    j = root.asBuiltJoints.add(inp)
    j.name = 'Screw turns'
    return j, -1


def _slider(root, occ_c, occ_f, geo, axes):
    """'Carriage slides', along x: each direction tried, the sign of a 1 mm slide measured along +x."""
    fallback = None
    for direction, ent in _tries(axes):
        try:
            inp = root.asBuiltJoints.createInput(occ_c, occ_f, geo())
            if ent is None:
                inp.setAsSliderJointMotion(direction)
            else:
                inp.setAsSliderJointMotion(direction, ent)
            j = root.asBuiltJoints.add(inp)
        except Exception:
            continue
        if j is None:
            continue
        try:
            t0 = occ_c.transform2.translation
            j.jointMotion.slideValue = 1.0 * CM
            _settle()
            t1 = occ_c.transform2.translation
            j.jointMotion.slideValue = 0.0
            _settle()
        except Exception:
            j.deleteMe()
            continue
        dx, dy, dz = (t1.x - t0.x) * 10, (t1.y - t0.y) * 10, (t1.z - t0.z) * 10
        along_x = abs(dy) < 1e-6 and abs(dz) < 1e-6 and abs(abs(dx) - 1.0) < 1e-6
        if along_x and dx > 0:
            j.name = 'Carriage slides'
            return j, 1
        if along_x and fallback is None:
            fallback = (direction, ent)
        j.deleteMe()
    if fallback is None:
        raise RuntimeError('no direction gave a slider along x')
    inp = root.asBuiltJoints.createInput(occ_c, occ_f, geo())
    if fallback[1] is None:
        inp.setAsSliderJointMotion(fallback[0])
    else:
        inp.setAsSliderJointMotion(fallback[0], fallback[1])
    j = root.asBuiltJoints.add(inp)
    j.name = 'Carriage slides'
    return j, -1


# ---------------------------------------------------------------------------------------------- the model
def build(d=12.0, P=3.0, starts=1, F=500.0, f=0.15):
    """Build the axis in a new design; returns the message text. Raises ValueError for refused inputs."""
    check_inputs(d, P, starts, F, f)
    s = screw(d, P, starts, F, f)
    g = geometry(d, P, starts)
    L, n = g['L'], g['n']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
    CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

    # ---- Frame: plate z = -6..0, two end blocks with the journal bores
    occ_f = _new_occ(root, 'Frame')
    occ_f.isGrounded = True
    cf = occ_f.component
    sk = cf.sketches.add(cf.xYConstructionPlane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(_pt(sk, -g['x_out'], g['fy0'], 0), _pt(sk, g['x_out'], g['fy1'], 0))
    ext = cf.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), NEW)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(6.0 * CM)),
                         adsk.fusion.ExtentDirections.NegativeExtentDirection)
    body_f = ext.add(inp).bodies.item(0)
    sk = cf.sketches.add(cf.xYConstructionPlane)
    for xa, xb in ((g['x_in'], g['x_out']), (-g['x_out'], -g['x_in'])):
        sk.sketchCurves.sketchLines.addTwoPointRectangle(_pt(sk, xa, g['fy0'], 0), _pt(sk, xb, g['fy1'], 0))
    profs = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        profs.add(sk.profiles.item(i))
    inp = ext.createInput(profs, JOIN)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(g['fz1'] * CM)),
                         adsk.fusion.ExtentDirections.PositiveExtentDirection)
    inp.participantBodies = [body_f]
    ext.add(inp)
    _extrude_sym(cf, _circle_profile(cf, 0, ZA, g['bore']), 2 * g['x_out'] + 2.0, CUT, cf.bRepBodies.item(0))
    cf.bRepBodies.item(0).name = 'Frame'
    # the joints' point and two axis lines (one each way) on the screw's axis
    skj = cf.sketches.add(cf.xZConstructionPlane)
    jpt = skj.sketchPoints.add(_pt(skj, 0, 0, ZA))
    ax_pos = skj.sketchCurves.sketchLines.addByTwoPoints(_pt(skj, -5, 0, ZA), _pt(skj, 5, 0, ZA))
    skj2 = cf.sketches.add(cf.xZConstructionPlane)
    ax_neg = skj2.sketchCurves.sketchLines.addByTwoPoints(_pt(skj2, 5, 0, ZA), _pt(skj2, -5, 0, ZA))
    skj.isVisible = False
    skj2.isVisible = False

    # ---- Guide rod: 8 mm at y = 20, z = 20, between the end blocks
    occ_r = _new_occ(root, 'Guide rod')
    cr = occ_r.component
    _extrude_sym(cr, _circle_profile(cr, ROD_Y, ROD_Z, ROD_D), 2 * g['x_in'], NEW)
    cr.bRepBodies.item(0).name = 'Guide rod'

    # ---- Screw: 20 leads of thread swept with 20 turns, journals into the end blocks
    occ_s = _new_occ(root, 'Screw')
    cs = occ_s.component
    x0 = -g['x_thread']
    sk = _polyline(cs.sketches.add(_x_plane(cs, x0)), to_world(x0, screw_section(g, x0)))
    xp = x0 + g['P'] / 4.0                                  # a quarter pitch in: the lobe's centre if right-hand
    probe_s = lambda b: (_inside(b, xp, g['r2'], 2.0 * math.pi * xp / L)                  # thread there
                         and not _inside(b, xp, g['r2'], math.pi / n + 2.0 * math.pi * xp / L))   # and a gap
    feat, body_s, mode = _twisted_sweep(cs, sk, x0, g['x_thread'], LEADS * 360.0, NEW, MODES[0], probe_s)
    sk.isVisible = False
    _extrude_sym(cs, _circle_profile(cs, 0, ZA, g['dj']), 2 * g['x_journal'], JOIN, body_s)
    cs.bRepBodies.item(0).name = 'Screw'

    # ---- Carriage: a block 20 mm long round the screw and the rod, the nut threaded in it
    occ_c = _new_occ(root, 'Carriage')
    cc = occ_c.component
    sk = cc.sketches.add(cc.yZConstructionPlane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(_pt(sk, 0, g['cy0'], g['cz0']), _pt(sk, 0, g['cy1'], g['cz1']))
    sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(sk, 0, ROD_Y, ROD_Z), ROD_BORE / 2.0 * CM)
    prof = None
    for i in range(sk.profiles.count):
        if sk.profiles.item(i).profileLoops.count == 2:
            prof = sk.profiles.item(i)
    _extrude_sym(cc, prof, NUT_LEN, NEW)
    xn = -NUT_LEN / 2.0
    skn = _polyline(cc.sketches.add(_x_plane(cc, xn)), to_world(xn, nut_hole_section(g, xn)))
    xq = xn + g['P'] / 4.0                                  # the nut's lobe centre there if right-hand
    probe_n = lambda b: (_inside(b, xq, g['r2'], math.pi / n + 2.0 * math.pi * xq / L)    # nut thread there
                         and not _inside(b, xq, g['r2'], 2.0 * math.pi * xq / L))         # the screw's room
    nut_twist = round(NUT_LEN / L * 360.0, 1)               # Fusion keeps a twist in tenths of a degree
    feat_n, body_c, mode_n = _twisted_sweep(cc, skn, xn, -xn, nut_twist, CUT, mode, probe_n, cc.bRepBodies.item(0))
    skn.isVisible = False
    cc.bRepBodies.item(0).name = 'Carriage'

    # ---- As-built joints and the Motion Link
    def geo():
        return adsk.fusion.JointGeometry.createByPoint(jpt.createForAssemblyContext(occ_f))
    axes = (ax_pos.createForAssemblyContext(occ_f), ax_neg.createForAssemblyContext(occ_f))
    rigid = None
    for gm in (None, geo()):                  # a rigid as-built joint needs no geometry; given one if refused
        try:
            rin = root.asBuiltJoints.createInput(occ_r, occ_f, gm)
            rin.setAsRigidJointMotion()
            rigid = root.asBuiltJoints.add(rin)
            if rigid:
                break
        except Exception:
            rigid = None
    if rigid is None:
        raise RuntimeError('the rigid joint of the guide rod could not be made')
    rigid.name = 'Guide rod fixed'
    j_s, sg_rev = _revolute(root, occ_s, occ_f, geo, axes)
    j_c, sg_sl = _slider(root, occ_c, occ_f, geo, axes)
    lim = j_c.jointMotion.slideLimits           # symmetric about the rest position, so the same either sign
    lim.isMinimumValueEnabled = True
    lim.minimumValue = -g['travel'] * CM
    lim.isMaximumValueEnabled = True
    lim.maximumValue = g['travel'] * CM

    # 360 deg of the screw (positive about +x when sg_rev = +1) for L of the carriage towards -x
    ml = None
    for typed in (False, True):
        try:
            mi = root.motionLinks.createInput(j_s, j_c)
            if typed:
                mi.motionOne = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
                mi.motionTwo = adsk.fusion.JointMotionTypes.SliderJointSlideMotionType
            mi.valueOne = VS('360 deg')
            mi.valueTwo = VS('%.6f mm' % L)
            mi.isReversed = sg_rev * sg_sl > 0    # +360 of the joint must give -sg_rev * L of carriage
            ml = root.motionLinks.add(mi)
            if ml:
                break
        except Exception:
            ml = None
    if ml is None:
        raise RuntimeError('the Motion Link could not be made')
    ml.name = 'Screw drives carriage'
    want = -sg_rev * L / 12.0                 # carriage x (mm) for +30 deg of the joint
    x_rest = _x_mm(occ_c)
    j_s.jointMotion.rotationValue = math.radians(30.0)
    _settle()
    moved = _x_mm(occ_c) - x_rest
    if abs(moved + want) < abs(moved - want):  # the wrong way: reverse the link
        ml.isReversed = not ml.isReversed
        j_s.jointMotion.rotationValue = 0.0
        _settle()
        j_s.jointMotion.rotationValue = math.radians(30.0)
        _settle()
        moved = _x_mm(occ_c) - x_rest
    j_s.jointMotion.rotationValue = 0.0
    _settle()

    for comp in (cf, cr, cs, cc):
        for i in range(comp.sketches.count):
            comp.sketches.item(i).isVisible = False
    try:
        app.activeViewport.fit()
    except Exception:
        pass

    return message(s, g, dict(mode=mode, mode_n=mode_n, rev=sg_rev, slide=sg_sl,
                              step=moved * sg_rev, nut_twist=nut_twist))


def message(s, g, m):
    lines = [
        'Lead screw axis built. %s' % VERSION,
        'Tr %g x %g, %d start(s); axial load F = %g N; thread friction f = %g (assumed); flanks at 15 deg'
        % (s['d'], s['P'], s['starts'], s['F'], s['f']),
        'pitch diameter d2 = %.3f mm, screw minor d3 = %.3f mm, nut minor D1 = %.3f mm, nut major D4 = %.3f mm'
        % (s['d2'], s['d3'], s['D1'], s['D4']),
        'lead L = %.3f mm, lead angle lambda = %.3f deg' % (s['L'], math.degrees(s['lam'])),
        'torque to move the load against it T = %.2f N.mm, efficiency %.3f' % (s['T'], s['eff']),
    ]
    if s['locks']:
        lines.append('self-locking (pi f d2 sec(alpha) = %.3f mm > L = %.3f mm): torque to lower the load %.2f N.mm'
                     % (s['friction_lead'], s['L'], s['T_low']))
    else:
        lines.append('NOT self-locking (pi f d2 sec(alpha) = %.3f mm <= L = %.3f mm): the load turns the screw back; '
                     'torque to hold it %.2f N.mm' % (s['friction_lead'], s['L'], -s['T_low']))
    lines += [
        'carriage: %.3f mm per screw turn, towards -x for a positive turn about +x (right-hand); travel %.2f to '
        '+%.2f mm, 2 mm clear of the end blocks' % (s['L'], -g['travel'], g['travel']),
        'threads: screw %d leads (%.1f deg of twist), nut %.0f mm (%.1f deg); 0.25 mm at crests and roots, '
        '0.2 mm along the axis' % (LEADS, LEADS * 360.0, NUT_LEN, m['nut_twist']),
        'measured: revolute sign %+d, slider sign %+d, 30 deg of the screw moved the carriage %.4f mm'
        % (m['rev'], m['slide'], m['step']),
    ]
    return '\n'.join(lines)


# ---------------------------------------------------------------------------------------------- the dialog
class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addValueInput('d', 'Screw major diameter d', 'mm', VS('12 mm'))
            ins.addValueInput('P', 'Pitch P', 'mm', VS('3 mm'))
            ins.addIntegerSpinnerCommandInput('starts', 'Number of starts', 1, 8, 1, 1)
            ins.addValueInput('F', 'Axial load on the nut F (N)', '', VI(500.0))
            ins.addValueInput('f', 'Thread friction coefficient f (assumed)', '', VI(0.15))
            on_exec = _Execute()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('LeadScrewAxis dialog failed:\n%s' % traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            d = ins.itemById('d').value * 10.0
            P = ins.itemById('P').value * 10.0
            starts = ins.itemById('starts').value
            F = ins.itemById('F').value
            f = ins.itemById('f').value
            ui.messageBox(build(d=d, P=P, starts=starts, F=F, f=f))
        except ValueError as e:
            ui.messageBox('LeadScrewAxis refused: %s\n%s' % (e, VERSION))
        except Exception:
            ui.messageBox('LeadScrewAxis failed:\n%s\n%s' % (traceback.format_exc(), VERSION))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        adsk.terminate()


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Lead screw axis',
                                                         'A trapezoidal lead screw, its nut in a carriage on a guide rod')
        on_created = _Created()
        cdef.commandCreated.add(on_created)
        _handlers.append(on_created)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('LeadScrewAxis failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
    except Exception:
        pass
