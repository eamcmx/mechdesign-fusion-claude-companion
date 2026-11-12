# -*- coding: utf-8 -*-
# =============================================================================
# BallScrewAxis.py  -  a linear axis on a ball screw, written from its prompt alone (prompt test).
#
#   Frame (grounded): a base plate and two bearing blocks, 150 + 40 = 190 mm apart (centre to centre)
#   Guide rod: 12 mm, through both blocks
#   Screw: 12 mm journals end to end, the grooved part (150 mm) between the blocks
#   Nut: flange 54 x 12 mm and a body of 30 mm, 46 mm long, its own groove
#   Balls: three circuits of one turn, every ball centred on the helix, one body
#   Carriage: a plate bolted to the nut's flange, sliding on the guide rod
#
# The screw's axis is the line y = 0, z = H (z is up, as in the parts library). The balls' helix is right-hand
# and crosses the angle 0 (the +y direction from the axis) at x = 0; angles are measured from +y towards +z.
# The groove is the set of points closer than R_g to that helix: its section square to the axis is computed
# exactly (the envelope of the circle of radius R_g square to the helix) and swept along the axis, turning
# once per lead; the same tool cuts the screw and the nut.
# All sizes in mm here; the API works in cm and radians (CM converts).
# =============================================================================
import math
import os
import sys
import traceback
import importlib

import adsk.core
import adsk.fusion


def _companion_common():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % os.path.dirname(os.path.abspath(__file__)))
        d = up


_COMMON = _companion_common()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP          # noqa: E402  (puts every lib/ folder on sys.path)
import mdfc_parts as MP               # noqa: E402
importlib.reload(MP)

VERSION = 'BallScrewAxis prompt-test v1 (2026-11-12)'
CMD_ID = 'mdfcBallScrewAxisPromptTest'

CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

DEFAULTS = dict(d_s=16.0, P=5.0, D_w=3.175, F=500.0, C_dyn=7310.0, n=1000.0)

# ---------- the layout (mm) ----------
H = 30.0                         # height of the screw's axis above the base plate's top face (z = 0)
GROOVED = 150.0                  # the grooved part of the screw, centred on x = 0
X_G0, X_G1 = -GROOVED / 2.0, GROOVED / 2.0
L_S = 150.0 + 40.0               # distance between the bearing blocks (centre to centre) = l_s
BLOCK_W = 20.0                   # block thickness along x
X_BLOCK = L_S / 2.0              # the blocks' centres at x = +/- 95
D_JOURNAL = 12.0
X_END = X_BLOCK + BLOCK_W / 2.0 + 5.0       # the screw's ends, 5 mm out of the blocks
D_ROD = 12.0
Y_ROD = 40.0                     # guide rod at (y, z) = (40, H), clear of the 54 mm flange
D_FLANGE, T_FLANGE = 54.0, 12.0
D_NUT, L_NUT = 30.0, 46.0
X_NUT0 = -L_NUT / 2.0            # nut from -23 to +23, flange at the -x end
T_PLATE = 12.0                   # carriage plate, bolted to the flange's face on the body side
PCD_BOLTS, D_BOLT_HOLE = 42.0, 5.5
CLEAR_JOURNAL = 0.2              # diametral clearances so that nothing touches but the bolted faces
CLEAR_ROD_FIXED = 0.1
CLEAR_ROD_SLIDE = 0.2
CLEAR_NUT_BODY = 1.0


# ---------- design calculations ----------
def _thousands(v, fmt='%.0f'):
    s = fmt % v
    head, dot, tail = s.partition('.')
    sign = '-' if head.startswith('-') else ''
    head = head.lstrip('-')
    groups = []
    while len(head) > 3:
        groups.insert(0, head[-3:])
        head = head[:-3]
    groups.insert(0, head)
    return sign + ' '.join(groups) + (dot + tail if dot else '')


def _sci(v):
    m, e = ('%.4e' % v).split('e')
    return '%se%d' % (m, int(e))


def ballscrew(d_s=16.0, P=5.0, D_w=3.175, F=500.0, C_dyn=7310.0, n=1000.0):
    """Every derived value of the prompt, from the six inputs (mm, N, rpm). Raises ValueError when a hard
    constraint is broken."""
    for name, v in (('d_s', d_s), ('P', P), ('D_w', D_w), ('F', F), ('C_dyn', C_dyn), ('n', n)):
        if not v > 0:
            raise ValueError('%s must be positive (got %g)' % (name, v))
    d_m = d_s + 0.6
    R_g = 0.52 * D_w
    d_k = d_m - 2.0 * R_g
    turn = math.hypot(math.pi * d_m, P)                          # length of one turn of the balls' helix
    per_turn = int(math.floor(0.97 * turn / D_w + 1e-9))         # at least 3 % of the turn to spare
    l_s = L_S
    k = dict(d_s=d_s, P=P, D_w=D_w, F=F, C_dyn=C_dyn, n=n,
             d_m=d_m, R_g=R_g, d_k=d_k, land_d=d_s - 0.4, bore=d_s + 1.6, turn=turn,
             per_turn=per_turn, circuits=3, balls=3 * per_turn,
             T_a=F * P / (2.0 * math.pi * 0.90), T_c=F * P * 0.80 / (2.0 * math.pi),
             l_s=l_s, land=P - 2.0 * R_g, ratio=d_s / D_w)
    k['L'] = (C_dyn / (1.2 * F)) ** 3 * 1e6
    k['L_h'] = k['L'] / (60.0 * n)
    k['F_k'] = 4.072e5 * 0.25 * d_k ** 4 / l_s ** 2
    k['n_k'] = 2.71e8 * 0.446 * d_k / l_s ** 2
    if not 3.0 <= k['ratio'] <= 10.0:
        raise ValueError('d_s / D_w = %.2f is outside 3 to 10' % k['ratio'])
    if k['land'] < 1.0:
        why = 'the grooves overlap' if k['land'] < 0 else 'the land between the grooves is under 1 mm'
        raise ValueError('land P - 2 R_g = %.2f mm: %s' % (k['land'], why))
    if k['L_h'] < 10000.0:
        raise ValueError('L_h %s h, below 10 000 h' % _thousands(k['L_h']))
    if 0.5 * k['F_k'] < 2.0 * F:
        raise ValueError('F_k %s N: 0.5 F_k = %s N is below 2 F = %s N'
                         % (_thousands(k['F_k']), _thousands(0.5 * k['F_k']), _thousands(2.0 * F)))
    if per_turn < 1:
        raise ValueError('no ball of %g mm fits round one turn' % D_w)
    return k


def helix(x, k):
    """The point of the balls' helix at x: (x, y, z), z from the base plate (right-hand, angle 0 = +y at x = 0)."""
    r, a = k['d_m'] / 2.0, 2.0 * math.pi * x / k['P']
    return (x, r * math.cos(a), H + r * math.sin(a))


def groove_section(x0, k, n=240):
    """The boundary of the points closer than R_g to the helix, in the plane x = x0 (mm, z from the base plate).
    It is where the tube's surface h(s) + R_g (cos a N(s) + sin a B(s)) crosses the plane: for each a, s is
    found from the x component, so the points lie exactly at R_g from the helix."""
    r, Rg = k['d_m'] / 2.0, k['R_g']
    w = 2.0 * math.pi / k['P']
    c = math.sqrt(1.0 + (r * w) ** 2)
    pts = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        s = x0 - Rg * math.sin(a) * r * w / c
        ws = w * s
        y = r * math.cos(ws) - Rg * math.cos(a) * math.cos(ws) + Rg * math.sin(a) * math.sin(ws) / c
        z = r * math.sin(ws) - Rg * math.cos(a) * math.sin(ws) - Rg * math.sin(a) * math.cos(ws) / c
        pts.append((x0, y, H + z))
    return pts


def ball_centres(k, x_mid=0.0):
    """Three circuits of one turn each, side by side round x_mid, every ball centred on the helix."""
    m = k['per_turn']
    total = k['circuits'] * m
    j0 = -(total // 2)
    return [helix(x_mid + k['P'] * (j0 + j) / float(m), k) for j in range(total)]


# ---------- Fusion helpers ----------
def _delete(objs):
    for o in reversed(objs):
        try:
            if o is not None and o.isValid:
                o.deleteMe()
        except Exception:
            pass


def _yz_plane(comp, x):
    """A plane square to x at x (mm), with a sketch on it; the offset's sense is checked on the sketch."""
    for off in (x, -x):
        ci = comp.constructionPlanes.createInput()
        ci.setByOffset(comp.yZConstructionPlane, VI(off * CM))
        pl = comp.constructionPlanes.add(ci)
        sk = comp.sketches.add(pl)
        if abs(sk.modelToSketchSpace(P3(x * CM, 0.0, H * CM)).z) < 1e-7:
            return pl, sk
        _delete([pl, sk])
    raise RuntimeError('could not place a plane at x = %.3f mm' % x)


def _closed_profile(sk, pts):
    """A closed fitted spline through the points (a polyline if the spline gives no profile)."""
    model = [sk.modelToSketchSpace(P3(x * CM, y * CM, z * CM)) for x, y, z in pts]
    col = adsk.core.ObjectCollection.create()
    for q in model:
        col.add(q)
    spl = None
    try:
        spl = sk.sketchCurves.sketchFittedSplines.add(col)
        spl.isClosed = True
        if sk.profiles.count == 1:
            return sk.profiles.item(0)
    except Exception:
        pass
    _delete([spl])
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(model[0], model[1])
    prev = first.endSketchPoint
    for q in model[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    if sk.profiles.count != 1:
        raise RuntimeError('the groove section gives %d profiles' % sk.profiles.count)
    return sk.profiles.item(0)


def _groove_tool(comp, xa, xb, sense, k):
    """The groove from xa to xb as a new body: the computed section at xa swept along the axis, turning once per
    lead. sense = +1 or -1 is the sign of Fusion's twist angle that gives a right-hand groove (found by testing)."""
    made = []
    try:
        pl, sk = _yz_plane(comp, xa)
        made += [pl, sk]
        prof = _closed_profile(sk, groove_section(xa, k))
        skp = comp.sketches.add(comp.xZConstructionPlane)
        made.append(skp)
        line = skp.sketchCurves.sketchLines.addByTwoPoints(skp.modelToSketchSpace(P3(xa * CM, 0.0, H * CM)),
                                                           skp.modelToSketchSpace(P3(xb * CM, 0.0, H * CM)))
        path = comp.features.createPath(line, False)
        sweeps = comp.features.sweepFeatures
        inp = sweeps.createInput(prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        inp.twistAngle = VI(sense * 2.0 * math.pi * (xb - xa) / k['P'])
        f = sweeps.add(inp)
        made.append(f)
        return f.bodies.item(0), made
    except Exception:
        _delete(made)
        raise


def _inside(body, p):
    return body.pointContainment(P3(p[0] * CM, p[1] * CM, p[2] * CM)) == \
        adsk.fusion.PointContainment.PointInsidePointContainment


def _follows_helix(tool, xa, xb, k, samples=6):
    """(follows, exact, tested): the tool holds the helix and not the point half a turn round (a quarter lead into
    each tested lead, where a groove of the opposite hand would be half a turn off); exact: its surface lies
    within 0.02 to 0.03 mm of R_g on both sides; tested: how many places were tried."""
    P, r, Rg = k['P'], k['d_m'] / 2.0, k['R_g']
    leads = max(1, int((xb - xa - 0.5) // P))
    ms = sorted(set(int(round(i * (leads - 1) / float(max(1, samples - 1)))) for i in range(samples)))
    follows, exact, tested = True, True, 0
    for m in ms:
        x = xa + P * (m + 0.25)
        if x > xb - 0.5:
            continue
        tested += 1
        a = 2.0 * math.pi * x / P
        e = (math.cos(a), math.sin(a))
        if not _inside(tool, helix(x, k)) or _inside(tool, (x, -r * e[0], H - r * e[1])):
            follows = False
            break
        for t, want in ((Rg - 0.02, True), (Rg + 0.03, False)):
            for sg in (1.0, -1.0):
                rr = r + sg * t
                if _inside(tool, (x, rr * e[0], H + rr * e[1])) != want:
                    exact = False
    return follows, exact, tested


def cut_groove(occ, body, xa, xb, k):
    """Cuts the helical groove from xa to xb into `body`. Tries one sweep over the whole length, then pieces of
    four leads. Returns (twist sense, exact, pieces)."""
    comp = occ.component
    last = None
    for piece in (xb - xa, 4.0 * k['P']):
        made, tools, sense, exact = [], [], None, True
        try:
            bounds, x = [], xa
            while x < xb - 1e-9:
                bounds.append((x, min(xb, x + piece)))
                x += piece
            for a, b in bounds:
                for s in ((sense,) if sense else (1.0, -1.0)):
                    tool, m = _groove_tool(comp, a, b, s, k)
                    ok, ex, tested = _follows_helix(tool, a, b, k)
                    if not tested:                 # a short last piece: nothing to test, the sense is known
                        ok = sense is not None
                    if ok:
                        made += m
                        tools.append(tool)
                        sense, exact = s, exact and ex
                        break
                    _delete(m)
                else:
                    raise RuntimeError('the swept groove does not follow the helix from x = %.1f mm' % a)
            col = adsk.core.ObjectCollection.create()
            for t in tools:
                col.add(t)
            cf = comp.features.combineFeatures
            ci = cf.createInput(body, col)
            ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
            ci.isKeepToolBodies = False
            cf.add(ci)
            for o in made:
                if isinstance(o, adsk.fusion.Sketch):
                    o.isVisible = False
                elif isinstance(o, adsk.fusion.ConstructionPlane):
                    o.isLightBulbOn = False
            return sense, exact, len(bounds)
        except Exception as e:
            _delete(made)
            last = e
    raise RuntimeError('could not cut the groove: %s' % last)


def groove_in_part(body, k, radius):
    """At x = P/4 the groove (at the helix's angle) is cut away at `radius`, the land half a turn round is not."""
    x = k['P'] / 4.0
    a = 2.0 * math.pi * x / k['P']
    groove = (x, radius * math.cos(a), H + radius * math.sin(a))
    land = (x, -radius * math.cos(a), H - radius * math.sin(a))
    return (not _inside(body, groove)) and _inside(body, land)


# ---------- joints ----------
JD = adsk.fusion.JointDirections
JG = adsk.fusion.JointGeometry


def _x_direction(geom):
    """The joint direction of a joint geometry that lies along the world x axis, and its sense."""
    for d, v in ((JD.XAxisJointDirection, geom.secondaryAxisVector), (JD.YAxisJointDirection, geom.thirdAxisVector),
                 (JD.ZAxisJointDirection, geom.primaryAxisVector)):
        if v is not None and abs(v.x) > 0.999:
            return d, (1 if v.x > 0 else -1)
    return None, 0


def _axis_line(root, forward):
    """A sketch line on the screw's axis, pointing +x (forward) or -x, for a custom joint direction."""
    sk = root.sketches.add(root.xZConstructionPlane)
    sk.name = 'Screw axis +x' if forward else 'Screw axis -x'
    a, b = (-100.0, 100.0) if forward else (100.0, -100.0)
    ln = sk.sketchCurves.sketchLines.addByTwoPoints(sk.modelToSketchSpace(P3(a * CM, 0.0, H * CM)),
                                                    sk.modelToSketchSpace(P3(b * CM, 0.0, H * CM)))
    sk.isVisible = False
    return ln


def _rigid(root, name, one, two, point):
    j = None
    for geom in (lambda: JG.createByPoint(point), lambda: None):
        try:
            inp = root.asBuiltJoints.createInput(one, two, geom())
            inp.setAsRigidJointMotion()
            j = root.asBuiltJoints.add(inp)
            break
        except Exception:
            j = None
    if j is None:
        raise RuntimeError('could not make the rigid joint %s' % name)
    j.name = name
    return j


def _axis_kept(occ):
    """True when the occurrence's transform leaves two points of the screw's axis where they were."""
    t = occ.transform2
    for x in (-50.0, 50.0):
        p = P3(x * CM, 0.0, H * CM)
        q = p.copy()
        q.transformBy(t)
        if p.distanceTo(q) > 1e-6:
            return False
    return True


def _turn_of(occ):
    t = occ.transform2
    return math.atan2(t.getCell(2, 1) - t.getCell(1, 2), t.getCell(1, 1) + t.getCell(2, 2))


def _revolute(root, name, one, two, point):
    """A revolute As-built Joint whose positive rotation turns `one` about +x (right hand) on the screw's axis."""
    d, sense = _x_direction(JG.createByPoint(point))
    options = [(d, None)] if (d is not None and sense > 0) else []
    options += [(JD.CustomJointDirection, True), (JD.CustomJointDirection, False)]
    for d, fwd in options:
        try:
            inp = root.asBuiltJoints.createInput(one, two, JG.createByPoint(point))
            if fwd is None:
                inp.setAsRevoluteJointMotion(d)
            else:
                inp.setAsRevoluteJointMotion(d, _axis_line(root, fwd))
            j = root.asBuiltJoints.add(inp)
        except Exception:
            continue
        m = j.jointMotion
        m.rotationValue = 0.5
        a, kept = _turn_of(one), _axis_kept(one)
        m.rotationValue = 0.0
        if abs(a) < 0.1 or (a > 0 and kept):          # not measurable here, or right: keep it
            j.name = name
            return j
        j.deleteMe()
    raise RuntimeError('could not make %s turn the screw about +x' % name)


def _slider(root, name, one, two, point):
    """A slider As-built Joint along x; returns (joint, sense of +slide along world x)."""
    d, sense = _x_direction(JG.createByPoint(point))
    options = [(d, None)] if d is not None else []
    options += [(JD.CustomJointDirection, True)]
    for d, fwd in options:
        try:
            inp = root.asBuiltJoints.createInput(one, two, JG.createByPoint(point))
            if fwd is None:
                inp.setAsSliderJointMotion(d)
            else:
                inp.setAsSliderJointMotion(d, _axis_line(root, fwd))
            j = root.asBuiltJoints.add(inp)
        except Exception:
            continue
        m = j.jointMotion
        m.slideValue = 1.0
        t = one.transform2.translation
        m.slideValue = 0.0
        if abs(t.y) < 1e-6 and abs(t.z) < 1e-6 and abs(abs(t.x) - 1.0) < 1e-6:
            j.name = name
            return j, (1 if t.x > 0 else -1)
        if abs(t.x) < 1e-9 and abs(t.y) < 1e-9 and abs(t.z) < 1e-9:   # not measurable here: trust the geometry
            j.name = name
            return j, (sense if fwd is None and sense else 1)
        j.deleteMe()
    raise RuntimeError('could not make %s slide along x' % name)


def _carriage_dx(js, carriage, angle):
    js.jointMotion.rotationValue = angle
    dx = carriage.transform2.translation.x / CM
    js.jointMotion.rotationValue = 0.0
    return dx


def _motion_link(root, js, jc, sense, P, carriage):
    """360 deg of the screw for one lead of the carriage, towards -x for a right-hand turn about +x (the nut's
    groove then stays on the screw's). Checked by turning the screw 30 deg."""
    def make(rev):
        inp = root.motionLinks.createInput(js, jc)
        try:
            inp.motionOne = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
            inp.motionTwo = adsk.fusion.JointMotionTypes.SliderJointSlideMotionType
        except Exception:
            pass
        inp.valueOne = VI(2.0 * math.pi)
        inp.valueTwo = VI(P * CM)
        inp.isReversed = rev
        ml = root.motionLinks.add(inp)
        ml.name = 'One lead per turn'
        return ml
    want = -P / 12.0
    rev = sense > 0                     # slide value must fall when the screw turns forward
    ml = make(rev)
    dx = _carriage_dx(js, carriage, math.pi / 6.0)
    if abs(dx + want) < 1e-4:            # it went the other way
        try:
            ml.isReversed = not rev
        except Exception:
            pass
        dx = _carriage_dx(js, carriage, math.pi / 6.0)
        if abs(dx + want) < 1e-4:
            ml.deleteMe()
            ml = make(not rev)
            dx = _carriage_dx(js, carriage, math.pi / 6.0)
    return ml, dx


# ---------- the build ----------
def build(d_s=16.0, P=5.0, D_w=3.175, F=500.0, C_dyn=7310.0, n=1000.0):
    """Checks the inputs (ValueError when refused), opens a new design and builds the axis. Returns the message."""
    k = ballscrew(d_s, P, D_w, F, C_dyn, n)
    r, Rg = k['d_m'] / 2.0, k['R_g']
    if r + Rg > D_NUT / 2.0 - 1.0:
        raise ValueError('the nut body of %g mm leaves under 1 mm round a groove of %.2f mm' % (D_NUT, 2 * (r + Rg)))
    if k['land_d'] / 2.0 <= D_JOURNAL / 2.0:
        raise ValueError('the land of %.1f mm is not larger than the 12 mm journals' % k['land_d'])

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent

    frame = MP.new_occ(root, 'Frame')
    rod = MP.new_occ(root, 'Guide rod')
    screw = MP.new_occ(root, 'Screw')
    nut = MP.new_occ(root, 'Nut')
    balls = MP.new_occ(root, 'Balls')
    carriage = MP.new_occ(root, 'Carriage')
    frame.isGrounded = True

    # Frame: base plate (top face at z = 0) and two bearing blocks, one body
    xb0, xb1 = X_BLOCK - BLOCK_W / 2.0, X_BLOCK + BLOCK_W / 2.0
    fb = MP.box(-X_END - 5.0, X_END + 5.0, -35.0, Y_ROD + 20.0, -12.0, 0.0)
    MP.union(fb, MP.box(-xb1, -xb0, -25.0, Y_ROD + 15.0, -6.0, H + 20.0),
             MP.box(xb0, xb1, -25.0, Y_ROD + 15.0, -6.0, H + 20.0))
    MP.cut(fb, MP.cyl((-X_END - 10, 0.0, H), (X_END + 10, 0.0, H), (D_JOURNAL + CLEAR_JOURNAL) / 2.0),
           MP.cyl((-X_END - 10, Y_ROD, H), (X_END + 10, Y_ROD, H), (D_ROD + CLEAR_ROD_FIXED) / 2.0))
    b_frame = MP.add_body(frame, fb, 'Frame')

    # Guide rod, flush with the blocks' outer faces
    b_rod = MP.add_body(rod, MP.cyl((-xb1, Y_ROD, H), (xb1, Y_ROD, H), D_ROD / 2.0), 'Guide rod')

    # Screw: journals end to end, the land on the grooved part, then the groove
    sb = MP.cyl((-X_END, 0.0, H), (X_END, 0.0, H), D_JOURNAL / 2.0)
    MP.union(sb, MP.cyl((X_G0, 0.0, H), (X_G1, 0.0, H), k['land_d'] / 2.0))
    b_screw = MP.add_body(screw, sb, 'Screw')

    # Nut: flange at the -x end, body, bore, four bolt holes in the flange
    x_n1 = X_NUT0 + L_NUT
    xf = X_NUT0 + T_FLANGE
    nb = MP.cyl((X_NUT0, 0.0, H), (xf, 0.0, H), D_FLANGE / 2.0)
    MP.union(nb, MP.cyl((xf - 1.0, 0.0, H), (x_n1, 0.0, H), D_NUT / 2.0))
    MP.cut(nb, MP.cyl((X_NUT0 - 1.0, 0.0, H), (x_n1 + 1.0, 0.0, H), k['bore'] / 2.0))
    holes = []
    for i in range(4):
        a = math.radians(45.0 + 90.0 * i)
        holes.append((PCD_BOLTS / 2.0 * math.cos(a), H + PCD_BOLTS / 2.0 * math.sin(a)))
    for y, z in holes:
        MP.cut(nb, MP.cyl((X_NUT0 - 1.0, y, z), (xf + 1.0, y, z), D_BOLT_HOLE / 2.0))
    b_nut = MP.add_body(nut, nb, 'Nut')

    # Carriage: a plate against the flange's body-side face, round the nut's body, sliding on the rod
    xc0, xc1 = xf, xf + T_PLATE
    cb = MP.box(xc0, xc1, -30.0, Y_ROD + 12.0, 4.0, 2 * H - 4.0)
    MP.cut(cb, MP.cyl((xc0 - 1, 0.0, H), (xc1 + 1, 0.0, H), (D_NUT + CLEAR_NUT_BODY) / 2.0),
           MP.cyl((xc0 - 1, Y_ROD, H), (xc1 + 1, Y_ROD, H), (D_ROD + CLEAR_ROD_SLIDE) / 2.0))
    for y, z in holes:
        MP.cut(cb, MP.cyl((xc0 - 1.0, y, z), (xc1 + 1.0, y, z), D_BOLT_HOLE / 2.0))
    b_car = MP.add_body(carriage, cb, 'Carriage')

    # Balls: three circuits of one turn, centred in the nut, one body
    x_mid = (X_NUT0 + x_n1) / 2.0
    cs = ball_centres(k, x_mid)
    bb = None
    for c in cs:
        s = MP.sphere(c, D_w / 2.0)
        bb = s if bb is None else MP.union(bb, s)
    b_balls = MP.add_body(balls, bb, 'Balls')

    # the groove in the screw (running out past both shoulders) and in the nut
    s_sense, s_exact, s_pieces = cut_groove(screw, b_screw, X_G0 - 1.0, X_G1 + 1.0, k)
    n_sense, n_exact, n_pieces = cut_groove(nut, b_nut, X_NUT0 - 2.0, x_n1 + 2.0, k)
    b_screw = screw.component.bRepBodies.item(0)
    b_nut = nut.component.bRepBodies.item(0)
    ok_screw = groove_in_part(b_screw, k, (k['d_k'] / 2.0 + k['land_d'] / 2.0) / 2.0)
    ok_nut = groove_in_part(b_nut, k, (k['bore'] / 2.0 + r + Rg) / 2.0)

    for body, mat in ((b_frame, 'Aluminum 6061'), (b_car, 'Aluminum 6061'), (b_rod, 'Steel'), (b_screw, 'Steel'),
                      (b_nut, 'Steel'), (b_balls, 'Steel')):
        try:
            MP.set_material(body, mat)
        except Exception:
            pass
    for occ, ap in ((frame, 'Aluminum - Satin'), (carriage, 'Paint - Enamel Glossy (Blue)'),
                    (rod, 'Steel - Satin'), (screw, 'Steel - Satin'), (nut, 'Steel - Satin'), (balls, 'Chrome')):
        MP.colour(des, occ, ap)

    # As-built Joints and the Motion Link
    jsk = root.sketches.add(root.xZConstructionPlane)
    jsk.name = 'Joint positions'
    pt = lambda x: jsk.sketchPoints.add(jsk.modelToSketchSpace(P3(x * CM, 0.0, H * CM)))
    _rigid(root, 'Guide rod fixed', rod, frame, pt(-X_BLOCK))
    js = _revolute(root, 'Screw turns', screw, frame, pt(0.0))
    jc, sense = _slider(root, 'Carriage slides', carriage, frame, pt(0.0))
    _rigid(root, 'Nut bolted to carriage', nut, carriage, pt(xf))
    _rigid(root, 'Balls in nut', balls, nut, pt(x_mid))
    jsk.isVisible = False

    lo, hi = X_G0 - X_NUT0, X_G1 - x_n1          # carriage travel that keeps the nut on the grooved part
    if sense < 0:
        lo, hi = -hi, -lo
    lim = jc.jointMotion.slideLimits
    lim.minimumValue = lo * CM
    lim.maximumValue = hi * CM
    lim.isMinimumValueEnabled = True
    lim.isMaximumValueEnabled = True

    ml, dx30 = _motion_link(root, js, jc, sense, P, carriage)

    try:
        app.activeViewport.fit()
    except Exception:
        pass

    link_ok = abs(dx30 + P / 12.0) < 1e-4
    lines = [
        'Ball screw axis  [%s]' % VERSION,
        'Inputs: d_s %g mm, P %g mm, D_w %g mm, F %g N, C_dyn %g N, n %g rpm' % (d_s, P, D_w, F, C_dyn, n),
        'd_m = %.3f mm, R_g = %.3f mm, d_k = %.3f mm (screw land %.1f mm, nut bore %.1f mm)'
        % (k['d_m'], k['R_g'], k['d_k'], k['land_d'], k['bore']),
        'Land between grooves P - 2 R_g = %.3f mm; d_s / D_w = %.2f' % (k['land'], k['ratio']),
        'Balls: %d a turn, %d circuits of one turn, %d in all' % (k['per_turn'], k['circuits'], k['balls']),
        'T_a = %.2f N mm (driving), T_c = %.2f N mm (back-driving)' % (k['T_a'], k['T_c']),
        'L = %s revolutions, L_h = %s h' % (_sci(k['L']), _thousands(k['L_h'])),
        'l_s = %.0f mm: F_k = %s N (0.5 F_k against 2 F = %s N), n_k = %s rpm'
        % (k['l_s'], _thousands(k['F_k']), _thousands(2 * F), _thousands(k['n_k'])),
        'Carriage travel %.0f to %.0f mm; 30 deg of the screw moved it %.4f mm (%s)'
        % (X_G0 - X_NUT0, X_G1 - x_n1, dx30, 'one lead a turn' if link_ok else 'CHECK THE MOTION LINK'),
        'Groove swept in %d + %d piece(s), checked in screw: %s, nut: %s%s'
        % (s_pieces, n_pieces, 'yes' if ok_screw else 'NO', 'yes' if ok_nut else 'NO',
           '' if (s_exact and n_exact) else '; WARNING: the swept surface is off R_g by more than 0.02 mm'),
    ]
    return '\n'.join(lines)


# ---------- the dialog ----------
_handlers = []
_OWN = [False]       # True when Fusion ran this script (it then keeps it alive until the dialog closes)


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = dict(d_s=ins.itemById('d_s').value / CM, P=ins.itemById('P').value / CM,
                        D_w=ins.itemById('D_w').value / CM, F=ins.itemById('F').value,
                        C_dyn=ins.itemById('C_dyn').value, n=ins.itemById('n').value)
            msg = build(**vals)
        except ValueError as e:
            msg = 'Refused: %s\n[%s]' % (e, VERSION)
        except Exception:
            msg = 'Failed:\n%s\n[%s]' % (traceback.format_exc(), VERSION)
        ui.messageBox(msg, 'Ball screw axis')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _OWN[0]:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('d_s', 'Nominal diameter d_s', 'mm', VS('%g mm' % DEFAULTS['d_s']))
        ins.addValueInput('P', 'Lead P', 'mm', VS('%g mm' % DEFAULTS['P']))
        ins.addValueInput('D_w', 'Ball diameter D_w', 'mm', VS('%g mm' % DEFAULTS['D_w']))
        ins.addValueInput('F', 'Axial load F (N)', '', VI(DEFAULTS['F']))
        ins.addValueInput('C_dyn', 'Dynamic load rating C_dyn (N)', '', VI(DEFAULTS['C_dyn']))
        ins.addValueInput('n', 'Average speed n (rpm)', '', VI(DEFAULTS['n']))
        on_exec = _Execute()
        cmd.execute.add(on_exec)
        _handlers.append(on_exec)
        on_destroy = _Destroy()
        cmd.destroy.add(on_destroy)
        _handlers.append(on_destroy)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        _OWN[0] = context is not None
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Ball screw axis',
                                                       'A linear axis on a ball screw, built in a new design.')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        if _OWN[0]:
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n%s\n[%s]' % (traceback.format_exc(), VERSION))
