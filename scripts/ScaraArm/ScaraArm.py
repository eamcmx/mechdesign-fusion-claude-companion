# =============================================================================
# ScaraArm.py  -  Chapter 33 of "Mechanical Design in Autodesk Fusion with Claude": a SCARA robot. Two arms turn in
# the horizontal plane, a quill slides up and down at the end of the second and turns the tool. Forward and inverse
# kinematics, the workspace, the joint limits and the payload's torque. Written from PROMPT.txt beside it; checked
# by checks/scara_check.py.
#
#   INPUTS        the dialog: L1, L2, column height H, stroke, joint limits q1max and q2max, payload m, reach speed
#   DERIVED       scara(): reach, inner radius, the tool's position for joint values (fk) and the joint values for
#                 a position (ik, both elbows), the resolution of a 0.1 deg step, the payload's inertia torque
#   HARD RULES    build() refuses L2 >= L1, limits beyond 170 deg, and a stroke longer than the column
#   CONVENTIONS   z up; base axis at the origin; bodies from TemporaryBRepManager; As-built Joints with limits:
#                 revolute J1, J2, J4 about z, slider J3 along z
#   ACCEPTANCE    the message; the tool where fk puts it for the joint values Fusion holds, at every target; within
#                 the 0.1 deg resolution of each target; no interference; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'ScaraArm 1.0 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
W1, W2, T1, T2, GAP = 80.0, 60.0, 40.0, 30.0, 1.0        # link widths and thicknesses, the gap between them (mm)
R_COL, R_QUILL = 60.0, 12.0
STEP = 0.1                                                # deg: the resolution Fusion keeps


def scara(L1=250.0, L2=200.0, H=400.0, stroke=150.0, q1max=132.0, q2max=150.0, m=2.0, alpha=10.0):
    reach = L1 + L2
    inner = math.sqrt(L1 * L1 + L2 * L2 + 2 * L1 * L2 * math.cos(math.radians(q2max)))
    z_l1 = H + GAP
    z_l2 = z_l1 + T1 + GAP
    z_flange = z_l2 - 110.0                               # the tool's flange, the quill up
    return dict(L1=L1, L2=L2, H=H, stroke=stroke, q1max=q1max, q2max=q2max, reach=reach, inner=inner, z_l1=z_l1,
                z_l2=z_l2, z_flange=z_flange, res=reach * math.radians(STEP), T_payload=m * (reach / 1000.0) ** 2 * alpha,
                m=m, alpha=alpha)


def fk(s, q1, q2, d3, q4):
    """The tool flange's centre and its angle for joint values (deg, mm; d3 down from the top)."""
    a1, a12 = math.radians(q1), math.radians(q1 + q2)
    return (s['L1'] * math.cos(a1) + s['L2'] * math.cos(a12), s['L1'] * math.sin(a1) + s['L2'] * math.sin(a12),
            s['z_flange'] - d3, q1 + q2 + q4)


def ik(s, x, y, z, phi):
    """Joint values for a tool position and angle: both elbows, those inside the limits."""
    L1, L2 = s['L1'], s['L2']
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1:
        return []
    out = []
    for sgn in (1, -1):
        q2 = sgn * math.degrees(math.acos(c2))
        q1 = math.degrees(math.atan2(y, x) - math.atan2(L2 * math.sin(math.radians(q2)),
                                                         L1 + L2 * math.cos(math.radians(q2))))
        q1 = (q1 + 180.0) % 360.0 - 180.0
        d3 = s['z_flange'] - z
        q4 = phi - q1 - q2
        if abs(q1) <= s['q1max'] and abs(q2) <= s['q2max'] and 0 <= d3 <= s['stroke']:
            out.append((q1, q2, d3, q4))
    return out


# ---------- Fusion building blocks (mm at the interface) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cyl(p0, p1, r):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def link(L, w, z0, t, hole_end):
    """A flat link from (0, 0) to (L, 0) in the xy plane, w wide, round ends, from z0 to z0 + t; a hole at its far
    end if hole_end > 0."""
    c = P3(L / 2 * CM, 0, (z0 + t / 2) * CM)
    b = tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), L * CM, w * CM, t * CM))
    union(b, cyl((0, 0, z0), (0, 0, z0 + t), w / 2), cyl((L, 0, z0), (L, 0, z0 + t), w / 2))
    if hole_end > 0:
        cut(b, cyl((L, 0, z0 - 1), (L, 0, z0 + t + 1), hole_end))
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


def place(root, name, x=0.0, y=0.0, z=0.0):
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def joint(root, moving, fixed, kind, name, lo=None, hi=None):
    """An As-built joint at the moving part's origin: revolute about z or slider along z, with limits (deg or mm)."""
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    J = adsk.fusion.JointDirections
    if kind == 'revolute':
        inp.setAsRevoluteJointMotion(J.ZAxisJointDirection)
    else:
        inp.setAsSliderJointMotion(J.ZAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    if lo is not None:
        lim = j.jointMotion.rotationLimits if kind == 'revolute' else j.jointMotion.slideLimits
        k = math.radians if kind == 'revolute' else (lambda v: v * CM)
        lim.isMinimumValueEnabled = True
        lim.minimumValue = k(lo)
        lim.isMaximumValueEnabled = True
        lim.maximumValue = k(hi)
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


# ---------- the robot ----------
def build(L1=250.0, L2=200.0, H=400.0, stroke=150.0, q1max=132.0, q2max=150.0, m=2.0):
    if L2 >= L1:
        raise ValueError('L2 = %.0f mm is not shorter than L1 = %.0f mm: the second arm would reach back past the '
                         'column.' % (L2, L1))
    if not (0 < q1max <= 170 and 0 < q2max <= 170):
        raise ValueError('Joint limits of %.0f and %.0f deg: keep them within 170 deg, where the cables still '
                         'reach.' % (q1max, q2max))
    if stroke >= H - 100.0:
        raise ValueError('A stroke of %.0f mm would bring the tool into the floor: keep it below H - 100 = %.0f mm.'
                         % (stroke, H - 100.0))
    s = scara(L1, L2, H, stroke, q1max, q2max, m)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    base = place(root, 'Base')
    bb = cyl((0, 0, 0), (0, 0, 40), 120)
    union(bb, cyl((0, 0, 39.999), (0, 0, H), R_COL))
    add_body(base.component, bb, 'Base')
    base.isGrounded = True

    a1 = place(root, 'Arm 1')
    add_body(a1.component, link(L1, W1, s['z_l1'], T1, 0), 'Arm 1')
    a2 = place(root, 'Arm 2', L1, 0, 0)
    add_body(a2.component, link(L2, W2, s['z_l2'], T2, R_QUILL + 0.5), 'Arm 2')
    quill = place(root, 'Quill', L1 + L2, 0, 0)
    add_body(quill.component, cyl((0, 0, s['z_flange'] + 0.001), (0, 0, s['z_l2'] + T2 + 120), R_QUILL), 'Quill')
    tool = place(root, 'Tool', L1 + L2, 0, s['z_flange'])
    tb = cyl((0, 0, -10), (0, 0, 0), 25.0)
    union(tb, cyl((0, 0, -60), (0, 0, -9.999), 6.0))                         # a pointer on the flange
    add_body(tool.component, tb, 'Tool')

    joint(root, a1, base, 'revolute', 'J1', -q1max, q1max)
    joint(root, a2, a1, 'revolute', 'J2', -q2max, q2max)
    joint(root, quill, a2, 'slider', 'J3', -stroke, 0.0)
    joint(root, tool, quill, 'revolute', 'J4', -360.0, 360.0)

    colour(design, base, 'Paint - Enamel Glossy (Grey)')
    colour(design, a1, 'Paint - Enamel Glossy (Yellow)')
    colour(design, a2, 'Paint - Enamel Glossy (Yellow)')
    colour(design, quill, 'Paint - Enamel Glossy (Grey)')
    colour(design, tool, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    return ('SCARA built: arms %.0f and %.0f mm, column %.0f mm, stroke %.0f mm; limits J1 +-%.0f deg, J2 +-%.0f deg.\n\n'
            'Reach %.1f mm; inner radius at the J2 limit %.3f mm. The tool flange at z = %.1f mm with the quill up.\n'
            'A step of 0.1 deg at J1 moves the tool %.3f mm at full reach.\n'
            'Accelerating %.1f kg at full reach at %.0f rad/s2 takes %.3f N m at J1 (the payload alone).\n\n'
            'Drive Joints on J1, J2, J3 and J4; the check sets them from the inverse kinematics.\n\n[%s]'
            % (L1, L2, H, stroke, q1max, q2max, s['reach'], s['inner'], s['z_flange'], s['res'], m, s['alpha'],
               s['T_payload'], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('L1', 'Arm 1', 'mm', 250.0), ('L2', 'Arm 2', 'mm', 200.0), ('H', 'Column height', 'mm', 400.0),
          ('stroke', 'Quill stroke', 'mm', 150.0), ('q1max', 'J1 limit', 'deg', 132.0),
          ('q2max', 'J2 limit', 'deg', 150.0)]


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
        cid = 'ScaraArmCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'SCARA robot', 'A SCARA arm with its kinematics')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('SCARA could not start:\n' + traceback.format_exc())
