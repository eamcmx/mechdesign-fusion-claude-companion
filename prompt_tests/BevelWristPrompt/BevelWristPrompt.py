# =============================================================================
# BevelWristPrompt.py  -  the bevel-gear differential of a robot wrist as one jointed assembly, written from the
# prompt projects/bevel_differential_wrist/BevelWristPrompt/PROMPT.txt alone.
#
#   Frame (grounded)  <- Yoke turns on it about world Y            ("Pitch")
#   Yoke              <- Output gear turns on it about Z           ("Roll")
#                     <- Idler gear about Z, Side gear L and Side gear R about Y
#   Motion Links, 1:1, from "Roll" to the idler and to the two side gears (Fusion has no tooth contact).
#
# Four miter gears (pitch cone angle 45 deg) share their apex at the origin. Each tooth is Tredgold's tooth: the
# tooth of a virtual spur gear of pitch radius R tan(delta), drawn at the heel in the plane tangent to the back cone,
# lofted to the same section scaled by k = (R - b)/R at the toe, joined to the blank and patterned z times about the
# gear's axis. Sizes are in mm and degrees here; the API gets cm and radians.
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

VERSION = 'BevelWristPrompt v1.0 (2026-11-11)'


def _find_companion():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py, put common/ on
    sys.path and import it (every lib/ folder of the companion is then on sys.path as well)."""
    here = os.path.dirname(os.path.abspath(__file__))
    d = here
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            common = os.path.join(d, 'common')
            if common not in sys.path:
                sys.path.insert(0, common)
            import companion_paths
            return companion_paths
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


CP = _find_companion()

CM = 0.1                          # mm to cm
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
WORLD_Y = (0.0, 1.0, 0.0)
WORLD_Z = (0.0, 0.0, 1.0)
TEST_TURN = 0.2                   # rad: the turn used to find each joint's direction and sign
CMD_ID = 'mdfc_BevelWristPrompt'

_handlers = []                    # event handlers kept alive while the dialog is open
_own = False                      # True when Fusion started the script (run() got a context), not another script


# ---------- the design numbers (mm, deg) ----------
def _inv(a):
    return math.tan(a) - a


def tooth_uv(m, rv, alpha, s, n=8):
    """The virtual spur tooth at the heel as a closed outline of (u, v) points: u along the tooth's width, v outward
    from the virtual pitch circle (radius rv). Tip rv + m, root rv - 1.25 m, base rv cos(alpha), thickness s on the
    virtual pitch circle; below the base circle the flank is a radial line. The outline is closed a little below the
    root, so that the loft reaches into the blank."""
    rb = rv * math.cos(alpha)
    ra = rv + m
    rf = rv - 1.25 * m
    psi_b = s / (2.0 * rv) + _inv(alpha)          # half-angle of the tooth at (and below) the base circle

    def half(rho):
        return psi_b if rho <= rb else psi_b - _inv(math.acos(rb / rho))

    th_a = half(ra)
    if th_a <= 0.0:
        raise ValueError('The teeth come to a point below the tip circle (tip half-angle %.4f rad): reduce the '
                         'backlash or the pressure angle.' % th_a)
    r0 = max(rf, rb)
    flank = [r0 + (ra - r0) * i / float(n) for i in range(n + 1)]
    rc = rf - min(0.3 * m, 1.0)                    # the closing edge, below the root
    pol = [(rc, half(rf))]
    pol += [(rho, half(rho)) for rho in flank]
    pol += [(ra, th_a * f) for f in (0.5, 0.0, -0.5)]
    pol += [(rho, -half(rho)) for rho in reversed(flank)]
    pol.append((rc, -half(rf)))
    return [(rho * math.sin(ph), rho * math.cos(ph) - rv) for rho, ph in pol]


def numbers(m, z, alpha, b, backlash, d):
    """Everything derived from the six inputs. Refuses an odd z and a face wider than R / 3."""
    if abs(z - round(z)) > 1e-9:
        raise ValueError('z = %s is not a whole number of teeth.' % z)
    z = int(round(z))
    if z % 2:
        raise ValueError('z = %d is odd: each gear meets two others 180 deg apart, so z must be even.' % z)
    if z < 6:
        raise ValueError('z = %d is too few teeth.' % z)
    if m <= 0 or b <= 0 or d <= 0 or backlash < 0:
        raise ValueError('m, b and d must be more than 0 and the backlash not less than 0.')
    if not 0.0 < alpha < 45.0:
        raise ValueError('The pressure angle must lie between 0 and 45 deg.')
    delta = math.radians(45.0)                    # miter gears
    sd, cd = math.sin(delta), math.cos(delta)
    r = m * z / 2.0
    R = r / sd
    if b > R / 3.0 + 1e-9:
        raise ValueError('b = %g mm is more than R / 3 = %.2f mm (z = %d, m = %g mm): the face width is refused.'
                         % (b, R / 3.0, z, m))
    k = (R - b) / R
    rv = R * math.tan(delta)                      # Tredgold: the virtual spur gear's pitch radius
    a = math.radians(alpha)
    s = math.pi * m / 2.0 - backlash / 2.0
    tooth = tooth_uv(m, rv, a, s)
    rh, hh = R * sd - 1.25 * m * cd, R * cd + 1.25 * m * sd      # heel root (radius, height from the apex)
    rt, ht = k * rh, k * hh                                      # toe root
    zb = hh + 2.0                                                # back face
    r_back = (R - zb * cd) / sd                                  # the back cone's radius at the back face
    r_hub = d / 2.0 + 6.2
    h_end = zb + 8.0
    S = math.sqrt((h_end + 5.0) ** 2 + 18.0 ** 2)
    r_bore = (d + 0.4) / 2.0
    if r_hub >= r_back:
        raise ValueError('The hub (radius %.2f mm) is as large as the back face (radius %.2f mm): use a smaller '
                         'pin or more teeth.' % (r_hub, r_back))
    if r_bore >= rt:
        raise ValueError('The bore (radius %.2f mm) reaches the toe root (radius %.2f mm).' % (r_bore, rt))
    return dict(m=m, z=z, alpha=alpha, b=b, backlash=backlash, d=d, sd=sd, cd=cd, r=r, R=R, k=k, rv=rv,
                tooth=tooth, rh=rh, hh=hh, rt=rt, ht=ht, zb=zb, r_back=r_back, r_hub=r_hub, h_end=h_end, S=S,
                r_bore=r_bore)


# ---------- temporary bodies (mm in, cm out) ----------
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def _cyl(p0, p1, r0, r1=None):
    r1 = r0 if r1 is None else r1
    return _tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r0 * CM, P3(*[v * CM for v in p1]), r1 * CM)


def _box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2.0 * CM, (y0 + y1) / 2.0 * CM, (z0 + z1) / 2.0 * CM)
    obb = adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0),
                                                 (x1 - x0) * CM, (y1 - y0) * CM, (z1 - z0) * CM)
    return _tbm().createBox(obb)


def _bool(target, tool, kind):
    if not _tbm().booleanOperation(target, tool, kind):
        raise RuntimeError('A boolean of temporary bodies failed.')
    return target


UNION = adsk.fusion.BooleanTypes.UnionBooleanType
CUT = adsk.fusion.BooleanTypes.DifferenceBooleanType
COMMON = adsk.fusion.BooleanTypes.IntersectionBooleanType


def gear_blank(G, flange):
    """The blank in the gear's own frame (apex at the origin, axis along z): the root cone from the toe root to the
    heel root, the back cone r sin(delta) + h cos(delta) = R from the heel root up to z_back, a flat back face, the
    hub to h_end and the bore right through. The toe end is a flat face at the toe root's height."""
    sd, cd, R = G['sd'], G['cd'], G['R']

    def back_r(h):
        return (R - h * cd) / sd

    ht, zb = G['ht'], G['zb']
    root_cone = _cyl((0, 0, ht), (0, 0, zb), G['rt'], zb * G['rh'] / G['hh'])     # the root cone's frustum
    back_cone = _cyl((0, 0, ht - 1.0), (0, 0, zb + 1.0), back_r(ht - 1.0), back_r(zb + 1.0))
    blank = _bool(root_cone, back_cone, COMMON)        # inside both: root cone up to the heel, back cone above it
    _bool(blank, _cyl((0, 0, zb - 1.0), (0, 0, G['h_end']), G['r_hub']), UNION)
    top = G['h_end']
    if flange:                                         # the output gear's flange on its hub end
        _bool(blank, _cyl((0, 0, G['h_end']), (0, 0, G['h_end'] + 5.0), 18.0), UNION)
        top = G['h_end'] + 5.0
    _bool(blank, _cyl((0, 0, ht - 1.0), (0, 0, top + 1.0), G['r_bore']), CUT)
    return blank


def frame_body(G):
    """Base plate and two cheeks as one body, each cheek bored on the pitch axis."""
    S, he = G['S'], G['h_end']
    top = -(S + 15.0)
    body = _box(-S, S, -(he + 12.0), he + 12.0, top - 6.0, top)
    for y0, y1 in ((he + 4.0, he + 12.0), (-(he + 12.0), -(he + 4.0))):
        _bool(body, _box(-25.0, 25.0, y0, y1, top - 3.0, 25.0), UNION)
    _bool(body, _cyl((0, -(he + 13.0), 0), (0, he + 13.0, 0), G['r_bore']), CUT)
    return body


def yoke_body(G):
    """A cross of two pins through the apex: roll pin along Z, pitch pin along Y to 6 mm outside the cheeks."""
    he, r = G['h_end'], G['d'] / 2.0
    body = _cyl((0, 0, -he), (0, 0, he + 5.0), r)
    _bool(body, _cyl((0, -(he + 18.0), 0), (0, he + 18.0, 0), r), UNION)
    return body


# ---------- components ----------
def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def new_component(root, name, x=(1.0, 0.0, 0.0), zax=(0.0, 0.0, 1.0)):
    """A component made where it belongs: origin at the apex, local x and z as given, y = z x x."""
    y = _cross(zax, x)
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P3(0, 0, 0), V3(*x), V3(*y), V3(*zax))
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def add_body(comp, tb, name):
    """A temporary body, already in the component's own frame, added through a base feature."""
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = comp.bRepBodies.item(comp.bRepBodies.count - 1)
    body.name = name
    return body


def gear_frames(z):
    """(name, local x = where tooth 0 points, local z = the gear's axis away from the apex, flange). The side gears
    point tooth 0 at +Z; the output and the idler point a gap at +Y, so their tooth 0 is pi/z from +Y."""
    c, s = math.cos(math.pi / z), math.sin(math.pi / z)
    return [('Output gear', (-s, c, 0.0), (0.0, 0.0, 1.0), True),       # +Y turned pi/z about +Z
            ('Idler gear', (s, c, 0.0), (0.0, 0.0, -1.0), False),       # +Y turned pi/z about -Z
            ('Side gear L', (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), False),
            ('Side gear R', (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), False)]


# ---------- teeth ----------
def _inverse(mat):
    m = mat.copy()
    m.invert()
    return m


def _mapped(T, p_mm):
    p = P3(p_mm[0] * CM, p_mm[1] * CM, p_mm[2] * CM)
    p.transformBy(T)
    return p


def _section_points(sk, sec):
    """The section's points in the sketch's space, through the inverse of the sketch transform."""
    T = _inverse(sk.transform)
    pts = [_mapped(T, p) for p in sec]
    if max(abs(p.z) for p in pts) > 1e-5:              # off the plane: try the sketch's own mapping
        pts = [sk.modelToSketchSpace(P3(p[0] * CM, p[1] * CM, p[2] * CM)) for p in sec]
        if max(abs(p.z) for p in pts) > 1e-5:
            raise RuntimeError('The tooth section does not lie on its sketch plane in %s.' % sk.parentComponent.name)
    return [P3(p.x, p.y, 0.0) for p in pts]


def add_teeth(comp, body, G):
    """Heel and toe sections of tooth 0, each a closed polyline on a plane through three of its points, lofted heel
    to toe and joined to the blank, then patterned z times about the gear's axis (the component's z)."""
    sd, cd, R, k = G['sd'], G['cd'], G['R'], G['k']
    heel = [(R * sd + v * cd, u, R * cd - v * sd) for u, v in G['tooth']]
    toe = [(k * x, k * y, k * zz) for x, y, zz in heel]
    pick = (0, len(heel) // 2, len(heel) - 1)            # the two closing corners and the middle of the tip
    helper = comp.sketches.add(comp.xYConstructionPlane)
    helper.name = 'Section plane points'
    Th = _inverse(helper.transform)
    profiles, planes, sketches = [], [], [helper]
    for label, sec in (('Heel', heel), ('Toe', toe)):
        sp = [helper.sketchPoints.add(_mapped(Th, sec[i])) for i in pick]
        ci = comp.constructionPlanes.createInput()
        ci.setByThreePoints(sp[0], sp[1], sp[2])
        plane = comp.constructionPlanes.add(ci)
        plane.name = label + ' section plane'
        sk = comp.sketches.add(plane)
        sk.name = label + ' section'
        pts = _section_points(sk, sec)
        lines = sk.sketchCurves.sketchLines
        first = lines.addByTwoPoints(pts[0], pts[1])
        prev = first.endSketchPoint
        for q in pts[2:]:
            prev = lines.addByTwoPoints(prev, q).endSketchPoint
        lines.addByTwoPoints(prev, first.startSketchPoint)
        if sk.profiles.count != 1:
            raise RuntimeError('The %s section of %s gave %d profiles.' % (label.lower(), comp.name, sk.profiles.count))
        profiles.append(sk.profiles.item(0))
        planes.append(plane)
        sketches.append(sk)
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.JoinFeatureOperation)
    li.loftSections.add(profiles[0])
    li.loftSections.add(profiles[1])
    li.isSolid = True
    try:
        li.participantBodies = [body]
    except Exception:
        pass
    loft = lofts.add(li)
    feats = adsk.core.ObjectCollection.create()
    feats.add(loft)
    pats = comp.features.circularPatternFeatures

    def pattern_input():
        pi = pats.createInput(feats, comp.zConstructionAxis)
        pi.quantity = adsk.core.ValueInput.createByReal(G['z'])
        pi.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        return pi
    try:
        pats.add(pattern_input())
    except Exception:
        pi = pattern_input()
        pi.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
        pats.add(pi)
    for plane in planes:
        plane.isLightBulbOn = False
    for sk in sketches:
        sk.isVisible = False
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s has %d bodies after its teeth.' % (comp.name, comp.bRepBodies.count))


# ---------- joints ----------
def _rot(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def _turn(R1, R0, axis):
    """Angle (rad) of the rotation R1 R0^T about a world axis, right-handed."""
    D = [[sum(R1[a][k] * R0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
    s = sum(w[i] * axis[i] for i in range(3)) / 2.0
    c = (D[0][0] + D[1][1] + D[2][2] - 1.0) / 2.0
    return math.atan2(s, c)


def apex_point(occ, sketches):
    """A sketch point at the part's apex (its component's origin), in the root's context."""
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Apex'
    sketches.append(sk)
    return sk.originPoint.createForAssemblyContext(occ)


def revolute(root, moving, base, point, axis, name):
    """An As-built revolute joint of `moving` on `base` at the apex. Tries the X, Y and Z joint directions and keeps
    the one that turns the part about the wanted world axis; returns the joint and its sign (+1 if a positive value
    turns the part right-handed about that axis)."""
    dirs = (adsk.fusion.JointDirections.XAxisJointDirection, adsk.fusion.JointDirections.YAxisJointDirection,
            adsk.fusion.JointDirections.ZAxisJointDirection)
    seen = []
    for d in dirs:
        geo = adsk.fusion.JointGeometry.createByPoint(point)
        ji = root.asBuiltJoints.createInput(moving, base, geo)
        ji.setAsRevoluteJointMotion(d)
        j = root.asBuiltJoints.add(ji)
        R0 = _rot(moving)
        j.jointMotion.rotationValue = TEST_TURN
        a = _turn(_rot(moving), R0, axis)
        j.jointMotion.rotationValue = 0.0
        seen.append(round(a, 4))
        if abs(abs(a) - TEST_TURN) < 1e-3:
            j.name = name
            return j, (1 if a > 0 else -1)
        j.deleteMe()
    raise RuntimeError('No joint direction turns %s about %s (turns seen: %s).' % (moving.component.name, axis, seen))


def _add_link(root, j1, j2, reverse, name):
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = adsk.core.ValueInput.createByString('360 deg')
    li.valueTwo = adsk.core.ValueInput.createByString('360 deg')
    li.isReversed = reverse
    ml = root.motionLinks.add(li)
    try:
        ml.name = name
    except Exception:
        pass
    return ml


def _driven_turn(roll, s_roll, occ, axis):
    """How far `occ` turns about `axis` (rad) when the output turns +TEST_TURN on the yoke (yoke at rest)."""
    R0 = _rot(occ)
    roll.jointMotion.rotationValue = s_roll * TEST_TURN
    adsk.doEvents()
    a = _turn(_rot(occ), R0, axis)
    roll.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    return a


def link(root, roll, s_roll, joint, s_gear, occ, axis, want, name):
    """A 1:1 Motion Link from "Roll" to a gear on the yoke that must turn `want` (+1 or -1) times the roll about
    `axis`. Reverse comes from the signs; then the link is driven once and flipped if the gear went the wrong way."""
    reverse = want * s_roll * s_gear < 0
    ml = _add_link(root, roll, joint, reverse, name)
    got = _driven_turn(roll, s_roll, occ, axis)
    if got * want < -1e-6:
        ml.deleteMe()
        reverse = not reverse
        ml = _add_link(root, roll, joint, reverse, name)
        got = _driven_turn(roll, s_roll, occ, axis)
    return abs(got - want * TEST_TURN) < 1e-3


# ---------- the assembly ----------
def build(m=2.0, z=40, alpha=20.0, b=16.0, backlash=0.2, d=8.0):
    """Builds the wrist in a new design and returns the closing message. Sizes in mm, the pressure angle in deg."""
    G = numbers(m, z, alpha, b, backlash, d)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent

    frame = new_component(root, 'Frame')
    add_body(frame.component, frame_body(G), 'Frame')
    frame.isGrounded = True
    yoke = new_component(root, 'Yoke')
    add_body(yoke.component, yoke_body(G), 'Yoke')
    gears = {}
    for name, x, zax, flange in gear_frames(G['z']):
        occ = new_component(root, name, x, zax)
        body = add_body(occ.component, gear_blank(G, flange), name)
        add_teeth(occ.component, body, G)
        gears[name] = occ

    sketches = []
    pitch, s_pitch = revolute(root, yoke, frame, apex_point(yoke, sketches), WORLD_Y, 'Pitch')
    out = gears['Output gear']
    roll, s_roll = revolute(root, out, yoke, apex_point(out, sketches), WORLD_Z, 'Roll')
    # relative to the yoke, about +Y side gear L turns +roll and side gear R -roll; about +Z the idler turns -roll
    followers = (('Idler gear', 'Idler', WORLD_Z, -1), ('Side gear L', 'Side L', WORLD_Y, 1),
                 ('Side gear R', 'Side R', WORLD_Y, -1))
    bad = []
    for part, jname, axis, want in followers:
        occ = gears[part]
        j, s_g = revolute(root, occ, yoke, apex_point(occ, sketches), axis, jname)
        if not link(root, roll, s_roll, j, s_g, occ, axis, want, 'Roll to ' + jname):
            bad.append(jname)
    for sk in sketches:
        sk.isVisible = False
    pitch.jointMotion.rotationValue = 0.0
    roll.jointMotion.rotationValue = 0.0

    msg = ('Bevel-gear wrist differential built.\n'
           'z = %d teeth on each of the four gears, m = %g mm, R = %.3f mm (pressure angle %g deg, face width %g mm, '
           'k = %.4f, sweep radius S = %.2f mm).\n'
           'Joints: "Pitch" turns the yoke on the frame about Y, "Roll" turns the output gear on the yoke about Z; '
           'the idler and the side gears follow "Roll" through 1:1 Motion Links.\n%s'
           % (G['z'], G['m'], G['R'], G['alpha'], G['b'], G['k'], G['S'], VERSION))
    if bad:
        msg += '\nThese Motion Links did not turn their gear as expected: %s' % ', '.join(bad)
    return msg


# ---------- the dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            msg = build(m=ins.itemById('module').value / CM,
                        z=ins.itemById('teeth').value,
                        alpha=math.degrees(ins.itemById('alpha').value),
                        b=ins.itemById('face').value / CM,
                        backlash=ins.itemById('backlash').value / CM,
                        d=ins.itemById('pin').value / CM)
            ui.messageBox(msg, 'Bevel wrist differential')
        except ValueError as e:
            ui.messageBox('Refused: %s' % e, 'Bevel wrist differential')
        except Exception:
            ui.messageBox('Failed:\n%s' % traceback.format_exc(), 'Bevel wrist differential')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _own:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        VS = adsk.core.ValueInput.createByString
        ins.addValueInput('module', 'Module m at the heel', 'mm', VS('2 mm'))
        ins.addIntegerSpinnerCommandInput('teeth', 'Teeth z, all four gears (even)', 6, 400, 1, 40)
        ins.addValueInput('alpha', 'Pressure angle', 'deg', VS('20 deg'))
        ins.addValueInput('face', 'Face width b', 'mm', VS('16 mm'))
        ins.addValueInput('backlash', 'Backlash on the pitch circle', 'mm', VS('0.2 mm'))
        ins.addValueInput('pin', 'Yoke pin diameter d', 'mm', VS('8 mm'))
        on_execute = _Execute()
        cmd.execute.add(on_execute)
        _handlers.append(on_execute)
        on_destroy = _Destroy()
        cmd.destroy.add(on_destroy)
        _handlers.append(on_destroy)


def run(context):
    global _own
    ui = adsk.core.Application.get().userInterface
    try:
        _own = context is not None
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, 'Bevel wrist differential', 'The bevel-gear differential of a robot wrist, jointed.')
        on_created = _Created()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)
        cmd_def.execute()
        if _own:
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        old = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
    except Exception:
        pass
