# -*- coding: utf-8 -*-
# =============================================================================
# KneeToggle.py  -  a knee toggle: two equal links from a fixed pivot T down to a ram on a straight guide, one closed
# loop of As-built Joints, so that Drive Joints on the upper link straightens the knee and drives the ram.
#
#   T = (0, 0), the ram's line is the -y axis; at angle alpha (of the links from the ram's line)
#       knee K = (l sin(alpha), -l cos(alpha)),   ram pin R = (0, -2 l cos(alpha))
#   virtual work: P dx_K = F dy_R  ->  F = P / (2 tan(alpha)),  mechanical advantage F / P = 1 / (2 tan(alpha))
#   stroke from alpha0 to straight: 2 l (1 - cos(alpha0))
#
# Sizes in mm and angles in degrees in this file; the API gets cm and radians.
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _find_common():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py in any folder above %s' % __file__)
        d = up


_COMMON = _find_common()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402,F401  (puts every lib/ folder on sys.path)
import mdfc_parts as MP  # noqa: E402

VERSION = 'KneeToggle v1 (prompt test 2026-11-11)'
CMD_ID = 'mdfc_KneeToggle_dialog'
CM = 0.1                      # mm to cm
WIDTH = 8.0                   # links' width, round ends of radius WIDTH / 2
HOLE = 3.0                    # pin holes at both ends of each link
RAM_X, RAM_Y = 16.0, 12.0     # the ram's block round R
ANGLES = (30.0, 10.0, 5.0, 2.0, 1.0)   # where F is reported
OPEN_MORE = 15.0              # the upper link may open to alpha0 + 15 deg

JD = adsk.fusion.JointDirections
KP_CENTRE = adsk.fusion.JointKeyPointTypes.CenterKeyPoint

_handlers = []


class Refused(ValueError):
    """Inputs outside the hard constraints."""


# ---------- the design calculation ----------
def toggle(l, alpha, P=1.0):
    """The toggle at angle alpha (deg) for links of length l (mm): knee, ram pin, F / P and F for a sideways P (N)."""
    a = math.radians(alpha)
    t = math.tan(a)
    ma = 1.0 / (2.0 * t) if abs(t) > 1e-12 else float('inf')
    return {'alpha': alpha,
            'K': (l * math.sin(a), -l * math.cos(a)),
            'R': (0.0, -2.0 * l * math.cos(a)),
            'ma': ma,
            'F': P * ma}


def stroke(l, alpha0):
    """The ram's travel from alpha0 to the straight position (mm)."""
    return 2.0 * l * (1.0 - math.cos(math.radians(alpha0)))


def check_inputs(l, alpha0, over, P):
    eps = 1e-9
    if not l > 0.0:
        raise Refused('the link length l must be more than 0 mm (got %g)' % l)
    if not 5.0 - eps <= alpha0 <= 60.0 + eps:
        raise Refused('alpha0 must be from 5 to 60 deg (got %g)' % alpha0)
    if not 1.0 - eps <= over <= 10.0 + eps:
        raise Refused('the over-centre angle must be from 1 to 10 deg, so that the toggle locks against its stop '
                      'just past straight (got %g)' % over)
    if not P > 0.0:
        raise Refused('the sideways force P must be more than 0 N (got %g)' % P)


def report(l, alpha0, over, P):
    lines = ['Knee toggle: l = %g mm, alpha0 = %g deg, over-centre %g deg, P = %g N' % (l, alpha0, over, P),
             '',
             'Stroke from alpha0 to straight, 2 l (1 - cos alpha0) = %.3f mm' % stroke(l, alpha0),
             'Mechanical advantage F / P = 1 / (2 tan alpha); at alpha0: %.3f' % toggle(l, alpha0)['ma'],
             '',
             'Ram force F = P / (2 tan alpha):']
    for a in ANGLES:
        t = toggle(l, a, P)
        lines.append('   alpha = %g deg:  F = %.2f N   (F / P = %.3f)' % (a, t['F'], t['ma']))
    lines += ['',
              'Drive Joints on "Upper link turns": 0 is alpha0; limited from alpha = -%g deg (past straight, '
              'the stop) to alpha = %g deg.' % (over, alpha0 + OPEN_MORE),
              '',
              VERSION]
    return '\n'.join(lines)


# ---------- the new design ----------
def _new_design():
    """A new Fusion design document, parametric, whose intent allows components (not a part design)."""
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    try:
        hybrid = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
        if design.designIntent != hybrid:
            design.designIntent = hybrid
    except Exception:
        pass
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    return doc


def _design_to_build_in():
    """The empty design run() opened before the dialog (a document cannot be made inside a command's events), or,
    when build() is called on its own, a new one."""
    app = adsk.core.Application.get()
    doc = app.activeDocument
    mark = doc.attributes.itemByName('KneeToggle', 'fresh') if doc is not None else None
    if mark is None:
        _new_design()
    else:
        mark.deleteMe()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    if root.occurrences.count or root.bRepBodies.count:
        _new_design()
        design = adsk.fusion.Design.cast(app.activeProduct)
    return design


# ---------- bodies ----------
def _link_body(p, q, z0, z1):
    """A link from p to q (x, y in mm), z0 to z1: WIDTH wide, round ends, a HOLE at each end."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    c = adsk.core.Point3D.create((p[0] + q[0]) / 2.0 * CM, (p[1] + q[1]) / 2.0 * CM, (z0 + z1) / 2.0 * CM)
    obb = adsk.core.OrientedBoundingBox3D.create(c, adsk.core.Vector3D.create(ux, uy, 0.0),
                                                 adsk.core.Vector3D.create(-uy, ux, 0.0),
                                                 L * CM, WIDTH * CM, (z1 - z0) * CM)
    b = MP.tbm().createBox(obb)
    MP.union(b, MP.cyl((p[0], p[1], z0), (p[0], p[1], z1), WIDTH / 2.0),
             MP.cyl((q[0], q[1], z0), (q[0], q[1], z1), WIDTH / 2.0))
    MP.cut(b, MP.cyl((p[0], p[1], z0 - 1.0), (p[0], p[1], z1 + 1.0), HOLE / 2.0),
           MP.cyl((q[0], q[1], z0 - 1.0), (q[0], q[1], z1 + 1.0), HOLE / 2.0))
    return b


# ---------- joint geometry ----------
def _flat_face(occ, z):
    """The planar face of the occurrence's body that lies square to z at height z (mm)."""
    body = occ.bRepBodies.item(0)
    for f in body.faces:
        g = f.geometry
        if g.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType and abs(abs(g.normal.z) - 1.0) < 1e-9 \
                and abs(f.pointOnFace.z - z * CM) < 1e-6:
            return f
    raise RuntimeError('no flat face at z = %g mm on %s' % (z, occ.component.name))


def _hole_edge(face, x, y):
    """The pin hole's edge on the face, centred at (x, y) (mm)."""
    for e in face.edges:
        g = e.geometry
        if g.curveType in (adsk.core.Curve3DTypes.Circle3DCurveType, adsk.core.Curve3DTypes.Arc3DCurveType) \
                and abs(g.radius - HOLE / 2.0 * CM) < 1e-6 \
                and math.hypot(g.center.x - x * CM, g.center.y - y * CM) < 1e-5:
            return e
    raise RuntimeError('no %g mm hole at (%.3f, %.3f)' % (HOLE, x, y))


def _pin_geometry(occ, z, x, y):
    """Joint geometry at the centre of the hole at (x, y) on the face at height z."""
    face = _flat_face(occ, z)
    edge = _hole_edge(face, x, y)
    if edge.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
        return adsk.fusion.JointGeometry.createByPlanarFace(face, edge, KP_CENTRE)
    return adsk.fusion.JointGeometry.createByCurve(edge, KP_CENTRE)


# ---------- joints ----------
def _revolute(root, one, two, geo, name, custom_axis=None):
    ji = root.asBuiltJoints.createInput(one, two, geo)
    done = ji.setAsRevoluteJointMotion(JD.ZAxisJointDirection) if custom_axis is None \
        else ji.setAsRevoluteJointMotion(JD.CustomJointDirection, custom_axis)
    if not done:
        raise RuntimeError('could not make %s revolute' % name)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def _turn_sign(j, occ):
    """+1 if a positive rotation value turns occ anticlockwise about +z, -1 if clockwise, 0 if it cannot be told."""
    m = j.jointMotion
    m.rotationValue = math.radians(2.0)
    s = occ.transform2.getCell(1, 0)
    if abs(s) < 1e-6:
        adsk.doEvents()
        s = occ.transform2.getCell(1, 0)
    m.rotationValue = 0.0
    if abs(occ.transform2.getCell(1, 0)) > 1e-9:
        adsk.doEvents()
    return 1 if s > 1e-6 else (-1 if s < -1e-6 else 0)


def _upper_joint(root, frame, upper, alpha0, over):
    """Upper link revolute about z on Frame at T, turning anticlockwise as alpha grows, limited from
    alpha = -over to alpha = alpha0 + 15 deg (rotation value 0 = alpha0)."""
    tries = ((4.0, None), (0.0, None), (4.0, root.zConstructionAxis))
    j = None
    for z, custom in tries:
        j = _revolute(root, upper, frame, _pin_geometry(upper, z, 0.0, 0.0), 'Upper link turns', custom)
        sign = _turn_sign(j, upper)
        if sign >= 0:
            break
        j.deleteMe()
        j = None
    if j is None:
        raise RuntimeError('the upper link would not turn anticlockwise about +z as alpha grows')
    lim = j.jointMotion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = math.radians(-over - alpha0)
    lim.isMaximumValueEnabled = True
    lim.maximumValue = math.radians(OPEN_MORE)
    j.jointMotion.rotationValue = 0.0
    return j


def _slider(root, ram, frame):
    """Ram slider along y on Frame: the joint that closes the loop."""
    face = _flat_face(ram, 12.0)
    for direction, entity in ((JD.CustomJointDirection, root.yConstructionAxis), (JD.YAxisJointDirection, None),
                              (JD.XAxisJointDirection, None), (JD.ZAxisJointDirection, None)):
        geo = adsk.fusion.JointGeometry.createByPlanarFace(face, None, KP_CENTRE)
        ji = root.asBuiltJoints.createInput(ram, frame, geo)
        try:
            done = ji.setAsSliderJointMotion(direction, entity) if entity is not None \
                else ji.setAsSliderJointMotion(direction)
        except Exception:
            done = False
        if not done:
            continue
        j = root.asBuiltJoints.add(ji)
        v = j.jointMotion.slideDirectionVector
        if abs(abs(v.y) - 1.0) < 1e-6:
            j.name = 'Ram slides'
            return j
        j.deleteMe()
    raise RuntimeError('could not make the ram slide along y')


# ---------- the build ----------
def build(l=40.0, alpha0=30.0, over=3.0, P=100.0):
    """Builds the knee toggle drawn at alpha0 and returns the report. Raises Refused (a ValueError) outside the
    hard constraints, before anything is made."""
    check_inputs(l, alpha0, over, P)
    design = _design_to_build_in()
    root = design.rootComponent
    t0 = toggle(l, alpha0)
    K0, R0 = t0['K'], t0['R']

    frame = MP.new_occ(root, 'Frame')
    upper = MP.new_occ(root, 'Upper link')
    lower = MP.new_occ(root, 'Lower link')
    ram = MP.new_occ(root, 'Ram')
    frame.isGrounded = True
    for o in (upper, lower, ram):
        o.isGrounded = False

    # the Frame's plate covers every place the links and the ram reach (its outline is not given: chosen here)
    x_lo = -max(12.0, l * math.sin(math.radians(over)) + WIDTH)
    x_hi = l * math.sin(math.radians(min(alpha0 + OPEN_MORE, 90.0))) + 12.0
    MP.add_body(frame, MP.box(x_lo, x_hi, -2.0 * l - 14.0, 12.0, -6.0, 0.0), 'Frame')
    MP.add_body(upper, _link_body((0.0, 0.0), K0, 0.0, 4.0), 'Upper link')
    MP.add_body(lower, _link_body(K0, R0, 4.0, 8.0), 'Lower link')
    MP.add_body(ram, MP.box(-RAM_X / 2.0, RAM_X / 2.0, R0[1] - RAM_Y / 2.0, R0[1] + RAM_Y / 2.0, 8.0, 12.0), 'Ram')

    ju = _upper_joint(root, frame, upper, alpha0, over)
    _revolute(root, lower, upper, _pin_geometry(lower, 4.0, K0[0], K0[1]), 'Lower link turns on upper link')
    _revolute(root, lower, ram, _pin_geometry(lower, 8.0, R0[0], R0[1]), 'Lower link turns on ram')
    _slider(root, ram, frame)
    ju.jointMotion.rotationValue = 0.0

    try:
        adsk.core.Application.get().activeViewport.fit()
    except Exception:
        pass
    return report(l, alpha0, over, P)


# ---------- the dialog ----------
class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addValueInput('l', 'Link length l', 'mm', adsk.core.ValueInput.createByString('40 mm'))
            ins.addValueInput('alpha0', 'Starting angle from the ram line alpha0', 'deg',
                              adsk.core.ValueInput.createByString('30 deg'))
            ins.addValueInput('over', 'Over-centre angle past straight', 'deg',
                              adsk.core.ValueInput.createByString('3 deg'))
            ins.addValueInput('P', 'Force pushing the knee sideways P (N)', '', adsk.core.ValueInput.createByReal(100.0))
            on_execute = _Execute()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('KneeToggle: the dialog failed\n' + traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            l = ins.itemById('l').value / CM
            alpha0 = math.degrees(ins.itemById('alpha0').value)
            over = math.degrees(ins.itemById('over').value)
            P = ins.itemById('P').value
            ui.messageBox(build(l=l, alpha0=alpha0, over=over, P=P), 'Knee toggle')
        except Refused as e:
            ui.messageBox('Refused: %s\n\n%s' % (e, VERSION), 'Knee toggle')
        except Exception:
            ui.messageBox('KneeToggle failed\n' + traceback.format_exc() + '\n' + VERSION, 'Knee toggle')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            adsk.terminate()
        except Exception:
            pass


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        doc = _new_design()                      # made here: a document cannot be created inside a command's events
        doc.attributes.add('KneeToggle', 'fresh', 'yes')
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Knee toggle',
                                                       'Two equal links from a fixed pivot to a ram on a guide')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        try:
            adsk.autoTerminate(False)
        except Exception:
            pass
    except Exception:
        if ui:
            ui.messageBox('KneeToggle failed\n' + traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
