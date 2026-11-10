# =============================================================================
# JointDrive.py  -  Chapter 34 of "Mechanical Design in Autodesk Fusion with Claude": the drive of a SCARA's first
# joint. The SCARA of Chapter 33, with aluminium arms and a steel quill and tool, gets a servo motor and a harmonic
# gearhead inside its hollow column. The load's inertia about the joint is measured on the model, then a move gives the
# acceleration, and the motor's torque and speed choose the gearhead's ratio. Written from PROMPT.txt beside it;
# checked by checks/joint_drive_check.py.
#
#   INPUTS        the dialog: the move (angle, time, dwell), the payload, the motor (rotor inertia, rated and peak
#                 torque, top speed) and the gearhead's efficiency
#   DERIVED       drive(): the speed and acceleration of a move in thirds, the best ratio for the inertias, the
#                 largest ratio the motor's speed allows, the motor's peak and RMS torque for each standard ratio,
#                 the ratio chosen; build() measures the load's inertia about J1 on the bodies, stretched and folded
#   HARD RULES    build() refuses an efficiency outside (0, 100] %, and a drive no standard ratio can make
#   CONVENTIONS   z up; bodies from TemporaryBRepManager; As-built Joints J1 to J4 as in Chapter 33 and the motor
#                 shaft's revolute; a Motion Link, reversed, at the ratio: the flexspline turns against the input
#   ACCEPTANCE    the message; the inertias against a hand calculation; the arm turning -1/n of the motor shaft;
#                 no interference; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'JointDrive 1.0 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
L1, L2, H, STROKE, Q1MAX, Q2MAX = 250.0, 200.0, 400.0, 150.0, 132.0, 150.0     # the SCARA of Chapter 33
W1, W2, T1, T2, GAP = 80.0, 60.0, 40.0, 30.0, 1.0
R_COL, R_BORE, R_QUILL, R_FLANGE, R_PLATE, T_PLATE, TIP = 60.0, 46.0, 12.0, 25.0, 120.0, 40.0, 60.0
R_HUB, R_MOTOR, L_MOTOR, R_HD, R_SHAFT = 40.0, 30.0, 110.0, 44.0, 6.0
RATIOS = (30, 50, 80, 100, 120, 160)                     # the standard ratios of harmonic gearheads


def profile(theta, t):
    """A move of theta (deg) in t (s) in thirds: accelerate, cruise, brake.
    Top speed (rad/s) and acceleration (rad/s2)."""
    w = math.radians(theta) / (2 * t / 3)
    return w, w / (t / 3)


def motor_torque(J_L, Jm, n, eta, alpha):
    """The motor's torque (N m) to accelerate its rotor and, through the
    gearhead, the load (kg m2)."""
    return (Jm * n + J_L / (eta * n)) * alpha


def drive(J_L, theta=90.0, t=0.4, dwell=0.2, Jm=0.3e-4, T_rated=1.27,
          T_peak=3.82, n_max=3000.0, eta=0.8):
    w, alpha = profile(theta, t)
    d = dict(J_L=J_L, w=w, alpha=alpha, T_load=J_L * alpha,
             n_opt=math.sqrt(J_L / (eta * Jm)),
             n_speed=n_max * 2 * math.pi / 60 / w,
             duty=math.sqrt((2 * t / 3) / (t + dwell)), rows=[])
    for n in RATIOS:
        Tp = motor_torque(J_L, Jm, n, eta, alpha)
        ok = n <= d['n_speed'] and Tp <= T_peak and Tp * d['duty'] <= T_rated
        d['rows'].append((n, Tp, Tp * d['duty'], ok))
    good = [r for r in d['rows'] if r[3]]
    best = min(good, key=lambda r: r[2]) if good else (None,) * 3
    d['n'], d['T_m'], d['T_rms'] = best[:3]
    return d


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


def add_body(comp, tb, name, material=None):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    if material:
        lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
        body.material = lib.materials.itemByName(material)
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


def inertia_z(occs):
    """The moment of inertia (kg m2) of the occurrences about the world z
    axis, the axis of J1."""
    m = [o.physicalProperties.getXYZMomentsOfInertia()[3] for o in occs]
    return sum(m) * 1e-4                                  # from kg cm2


# ---------- the robot and its drive ----------
def build(theta=90.0, t=0.4, dwell=0.2, m=2.0, Jm=0.3, T_rated=1.27, T_peak=3.82, n_max=3000.0, eta=80.0):
    if not 0 < eta <= 100:
        raise ValueError('An efficiency of %.0f %%: it lies between 0 and 100 %%.' % eta)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    z_l1 = H + GAP
    z_l2 = z_l1 + T1 + GAP
    z_flange = z_l2 - 110.0
    z_hd = H - 10.0                                           # the gearhead's top, 1 mm below the arm's hub

    base = place(root, 'Base')
    bb = cyl((0, 0, 0), (0, 0, T_PLATE), R_PLATE)
    tube = cyl((0, 0, T_PLATE - 0.001), (0, 0, H), R_COL)
    cut(tube, cyl((0, 0, T_PLATE), (0, 0, H + 1), R_BORE))
    add_body(base.component, union(bb, tube), 'Base')
    base.isGrounded = True
    motor = place(root, 'Motor')
    add_body(motor.component, cyl((0, 0, T_PLATE), (0, 0, T_PLATE + L_MOTOR), R_MOTOR), 'Motor')
    motor.isGrounded = True
    hd = place(root, 'Gearhead')
    hb = cyl((0, 0, z_hd - 60), (0, 0, z_hd), R_HD)
    cut(hb, cyl((0, 0, z_hd - 61), (0, 0, z_hd + 1), R_SHAFT + 2))
    add_body(hd.component, hb, 'Gearhead')
    hd.isGrounded = True
    shaft = place(root, 'Motor shaft')
    add_body(shaft.component, cyl((0, 0, T_PLATE + L_MOTOR + 0.5), (0, 0, z_hd - 10), R_SHAFT), 'Motor shaft')

    a1 = place(root, 'Arm 1')
    ab = link(L1, W1, z_l1, T1, 0)
    union(ab, cyl((0, 0, z_hd + 1), (0, 0, z_l1 + 0.001), R_HUB))           # the hub on the gearhead's output
    add_body(a1.component, ab, 'Arm 1', 'Aluminum')
    a2 = place(root, 'Arm 2', L1, 0, 0)
    add_body(a2.component, link(L2, W2, z_l2, T2, R_QUILL + 0.5), 'Arm 2', 'Aluminum')
    quill = place(root, 'Quill', L1 + L2, 0, 0)
    add_body(quill.component, cyl((0, 0, z_flange + 0.001), (0, 0, z_l2 + T2 + STROKE + 20.0), R_QUILL), 'Quill',
             'Steel')
    tool = place(root, 'Tool', L1 + L2, 0, z_flange)
    tb = cyl((0, 0, -10), (0, 0, 0), R_FLANGE)
    union(tb, cyl((0, 0, -TIP), (0, 0, -9.999), 6.0))
    add_body(tool.component, tb, 'Tool', 'Steel')

    j1 = joint(root, a1, base, 'revolute', 'J1', -Q1MAX, Q1MAX)
    j2 = joint(root, a2, a1, 'revolute', 'J2', -Q2MAX, Q2MAX)
    joint(root, quill, a2, 'slider', 'J3', -STROKE, 0.0)
    joint(root, tool, quill, 'revolute', 'J4', -180.0, 180.0)
    js = joint(root, shaft, motor, 'revolute', 'Motor')

    arm = [a1, a2, quill, tool]
    J_arm = inertia_z(arm)
    j2.jointMotion.rotationValue = math.radians(Q2MAX)
    adsk.doEvents()
    J_fold = inertia_z(arm)
    r_fold = math.sqrt(L1 * L1 + L2 * L2 + 2 * L1 * L2 * math.cos(math.radians(Q2MAX)))
    j2.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    J_L = J_arm + m * ((L1 + L2) / 1000.0) ** 2              # the payload, a point at full reach
    d = drive(J_L, theta, t, dwell, Jm * 1e-4, T_rated, T_peak, n_max, eta / 100.0)
    if d['n'] is None:
        n_fast = [r[0] for r in d['rows'] if r[0] <= d['n_speed']]
        if not n_fast:
            raise ValueError('The move needs %.1f rpm at the arm: the motor\'s %.0f rpm allow a ratio of %.1f at most, '
                             'below every standard ratio.' % (d['w'] * 60 / (2 * math.pi), n_max, d['n_speed']))
        raise ValueError('No standard ratio up to %.1f keeps the motor within %.2f N m peak and %.2f N m RMS: at '
                         '%d it needs %.2f N m peak and %.2f N m RMS.'
                         % (d['n_speed'], T_peak, T_rated, n_fast[-1], d['rows'][len(n_fast) - 1][1],
                            d['rows'][len(n_fast) - 1][2]))

    li = root.motionLinks.createInput(js, j1)
    li.valueOne = VS('%d deg' % (360 * d['n']))              # n turns of the motor ...
    li.valueTwo = VS('360 deg')                              # ... turn the arm once
    li.isReversed = True                                     # the flexspline turns against the wave generator
    root.motionLinks.add(li).name = 'Harmonic gearhead %d:1' % d['n']

    colour(design, base, 'Paint - Enamel Glossy (Grey)')
    colour(design, motor, 'Paint - Enamel Glossy (Black)')
    colour(design, shaft, 'Paint - Enamel Glossy (Grey)')
    colour(design, hd, 'Paint - Enamel Glossy (Blue)')
    colour(design, a1, 'Paint - Enamel Glossy (Yellow)')
    colour(design, a2, 'Paint - Enamel Glossy (Yellow)')
    colour(design, quill, 'Paint - Enamel Glossy (Grey)')
    colour(design, tool, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    table = '\n'.join('  %3d:1   %.3f N m peak, %.3f N m RMS%s' % (n, Tp, Tr, '' if ok else '   (no)')
                      for n, Tp, Tr, ok in d['rows'])
    return ('SCARA joint 1 drive built: a %.0f deg move in %.2f s, then %.2f s still; %.1f kg payload.\n\n'
            'Inertia about J1, measured on the bodies: arm %.5f kg m2 stretched, %.5f kg m2 folded to %.0f deg '
            '(the tool %.1f mm from the axis); with the payload %.5f kg m2.\n'
            'The move: %.4f rad/s at the top, %.3f rad/s2; %.3f N m at the joint.\n'
            'Best ratio for the inertias %.1f; the motor\'s %.0f rpm allow %.2f at most.\n%s\n'
            'Chosen: %d:1, %.3f N m peak and %.3f N m RMS at the motor.\n\n[%s]'
            % (theta, t, dwell, m, J_arm, J_fold, Q2MAX, r_fold, J_L, d['w'], d['alpha'], d['T_load'], d['n_opt'],
               n_max, d['n_speed'], table, d['n'], d['T_m'], d['T_rms'], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('theta', 'Move angle', 'deg', 90.0), ('t', 'Move time (s)', '', 0.4), ('dwell', 'Dwell (s)', '', 0.2),
          ('m', 'Payload (kg)', '', 2.0), ('Jm', 'Rotor inertia (kg cm2)', '', 0.3),
          ('T_rated', 'Rated torque (N m)', '', 1.27), ('T_peak', 'Peak torque (N m)', '', 3.82),
          ('n_max', 'Top speed (rpm)', '', 3000.0), ('eta', 'Gearhead efficiency (%)', '', 80.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            v = {}
            for key, _, unit, _ in FIELDS:
                x = ins.itemById(key).value
                v[key] = math.degrees(x) if unit == 'deg' else x
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
            ins.addValueInput(key, label, unit, VI(math.radians(d) if unit == 'deg' else d))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'JointDriveCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'SCARA joint drive', 'Size the drive of a SCARA joint')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('JointDrive could not start:\n' + traceback.format_exc())
