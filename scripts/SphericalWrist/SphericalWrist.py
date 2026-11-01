# =============================================================================
# SphericalWrist.py  -  Chapter 35 of "Mechanical Design in Autodesk Fusion with Claude": a roll-pitch-roll wrist.
# Three revolute joints whose axes meet at one point turn the tool to any orientation: roll about the forearm's axis,
# pitch about an axis square to it, roll about the tool's axis. Their angles for an orientation (two solutions), and
# the singularity where the two rolls line up. Written from PROMPT.txt beside it; checked by
# checks/spherical_wrist_check.py.
#
#   INPUTS        the dialog: the pitch limit, the tool's length
#   DERIVED       xyx(): the orientation for three joint angles; ik(): the angles for an orientation, both
#                 solutions within the limits, or one at the singularity
#   HARD RULES    build() refuses a pitch limit above 110 deg (the flange would reach the roll housing) or below
#                 90 deg (the tool could not point square to the forearm), and a tool shorter than 20 mm
#   CONVENTIONS   the wrist's centre at the origin, the forearm along -x; components created with the joint's axis
#                 as their z; As-built revolute joints, the direction tried and the sign measured
#   ACCEPTANCE    the message; the flange's orientation measured against the target for each solution; the
#                 singularity measured; a path past it; no interference at the limits; four refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'SphericalWrist 1.0 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
D_FLANGE, R_FLANGE, T_FLANGE = 71.0, 30.0, 10.0           # the flange: its distance from the centre, radius, thickness


def rot(axis, deg):
    """The 3 x 3 rotation by deg about the x or y axis."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == 'x':
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def xyx(a, b, c):
    """The tool's orientation for roll a, pitch b, roll c (deg)."""
    return mul(mul(rot('x', a), rot('y', b)), rot('x', c))


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def ik(R, pmax, a_now=0.0):
    """Joint angles (deg) for an orientation: both solutions inside
    the pitch limit; at the singularity, the roll a kept at a_now."""
    if R[0][0] > 1 - 1e-12:                    # the two rolls in line
        total = math.degrees(math.atan2(R[2][1], R[1][1]))
        return [(a_now, 0.0, wrap(total - a_now))]
    out = []
    for s in (1, -1):
        b = s * math.degrees(math.acos(max(-1.0, min(1.0, R[0][0]))))
        a = math.degrees(math.atan2(s * R[1][0], -s * R[2][0]))
        c = math.degrees(math.atan2(s * R[0][1], s * R[0][2]))
        if abs(b) <= pmax:
            out.append((wrap(a), b, wrap(c)))
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


def boolean(a, kind, *bs):
    for b in bs:
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


def add_body(occ, tb, name):
    """A body given in world coordinates into a component placed with its own axes."""
    m = occ.transform2.copy()
    m.invert()
    tbm().transform(tb, m)
    comp = occ.component
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def place(root, name, x_axis, z_axis):
    """A component at the wrist's centre, its local z along z_axis (its joint's axis) and local x along x_axis."""
    zx, zy, zz = z_axis
    xx, xy, xz = x_axis
    y_axis = (zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx)
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P3(0, 0, 0), V3(*x_axis), V3(*y_axis), V3(*z_axis))
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def origin_point(occ):
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(occ)


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_about(root, moving, fixed, axis, name, limit=None):
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
            if limit is not None:
                lim = j.jointMotion.rotationLimits
                lim.isMinimumValueEnabled, lim.minimumValue = True, -math.radians(limit)
                lim.isMaximumValueEnabled, lim.maximumValue = True, math.radians(limit)
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


# ---------- the wrist ----------
def build(pmax=110.0, tool=60.0):
    if pmax > 110.0:
        raise ValueError('A pitch limit of %.0f deg: beyond 110 deg the flange reaches the roll housing.' % pmax)
    if pmax < 90.0:
        raise ValueError('A pitch limit of %.0f deg: below 90 deg the tool cannot point square to the forearm.'
                         % pmax)
    if tool < 20.0:
        raise ValueError('A tool of %.0f mm: at least 20 mm, so that it shows the flange\'s axis.' % tool)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)

    fore = place(root, 'Forearm', Y, X)
    add_body(fore, cyl((-300, 0, 0), (-121, 0, 0), 45.0), 'Forearm')
    fore.isGrounded = True
    roll = place(root, 'Roll housing', Y, X)
    rb = cyl((-120, 0, 0), (-50, 0, 0), 45.0)
    for s in (1, -1):
        union(rb, box(-55.0, 30.0, min(s * 36.0, s * 46.0), max(s * 36.0, s * 46.0), -30.0, 30.0))
    add_body(roll, rb, 'Roll housing')
    pitch = place(root, 'Pitch housing', X, Y)
    pb = cyl((0, -35, 0), (0, 35, 0), 25.0)
    union(pb, cyl((0, 0, 0), (D_FLANGE - 1.0, 0, 0), 18.0))
    add_body(pitch, pb, 'Pitch housing')
    flange = place(root, 'Flange', Y, X)
    fb = cyl((D_FLANGE, 0, 0), (D_FLANGE + T_FLANGE, 0, 0), R_FLANGE)
    union(fb, cyl((D_FLANGE + T_FLANGE - 0.001, 0, 0), (D_FLANGE + T_FLANGE + tool, 0, 0), 5.0))
    union(fb, box(D_FLANGE, D_FLANGE + T_FLANGE, -4.0, 4.0, R_FLANGE - 0.5, R_FLANGE + 10.0))   # a mark on top
    add_body(flange, fb, 'Flange')

    j4, s4 = revolute_about(root, roll, fore, X, 'J4 roll', 180.0)
    j5, s5 = revolute_about(root, pitch, roll, Y, 'J5 pitch', pmax)
    j6, s6 = revolute_about(root, flange, pitch, X, 'J6 roll', 180.0)
    for j, s in ((j4, s4), (j5, s5), (j6, s6)):
        j.name += ' (%s)' % ('+' if s > 0 else '-')            # the sign: a positive value turns it about +axis

    colour(design, fore, 'Paint - Enamel Glossy (Grey)')
    colour(design, roll, 'Paint - Enamel Glossy (Yellow)')
    colour(design, pitch, 'Paint - Enamel Glossy (Blue)')
    colour(design, flange, 'Paint - Enamel Glossy (Grey)')
    app.activeViewport.fit()
    return ('Spherical wrist built: roll J4 about x, pitch J5 about y (limit +-%.0f deg), roll J6 about the flange\'s '
            'axis; the three axes meet at the origin.\nThe flange %.0f mm from the centre, the tool %.0f mm long, its '
            'tip %.0f mm from the centre.\nJoint signs (a positive value turns about +axis): J4 %+d, J5 %+d, J6 %+d.\n\n'
            'At J5 = 0 the two rolls are in line: only J4 + J6 sets the orientation.\n\n[%s]'
            % (pmax, D_FLANGE, tool, D_FLANGE + T_FLANGE + tool, s4, s5, s6, VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('pmax', 'Pitch limit', 'deg', 110.0), ('tool', 'Tool length', 'mm', 60.0)]


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
        cid = 'SphericalWristCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Spherical wrist', 'A roll-pitch-roll robot wrist')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('SphericalWrist could not start:\n' + traceback.format_exc())
