# =============================================================================
# FirstAssembly.py  -  a block with a shaft standing in a hole through it, the shaft on a revolute As-built Joint
# so that Drive Joints turns it about z.
#
#   Block  60 x 40 x 20 mm by default, centred on the origin in x and y, top face at z = 0, bottom at z = -H_b,
#          with a hole of d + 0.2 mm (0.1 mm clearance on the radius) through it at (x, y). Grounded.
#   Shaft  d x L_s, drawn in its own component at its own origin; the component is created at (x, y, 0) and never
#          moved. The shaft runs from z = -H_b to z = L_s - H_b.
#   Joint  "Shaft turns": As-built Joint, revolute about the shaft's own axis (z through the shaft's centre).
#
# Every input is in millimetres; the Fusion API works in centimetres, so every length is multiplied by CM.
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

VERSION = 'FirstAssembly 1.0 (prompt test)'


# ---------- the companion's libraries, through common/companion_paths.py ----------
def _companion_paths():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py, put common/ on
    sys.path and import it. This script needs no library, so a missing companion is not an error."""
    here = os.path.dirname(os.path.abspath(__file__))
    while True:
        common = os.path.join(here, 'common')
        if os.path.isfile(os.path.join(common, 'companion_paths.py')):
            if common not in sys.path:
                sys.path.insert(0, common)
            import companion_paths
            return companion_paths
        up = os.path.dirname(here)
        if up == here:
            return None
        here = up


CP = _companion_paths()

app = adsk.core.Application.get()
ui = app.userInterface

CM = 0.1                       # mm to cm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

CLEARANCE = 0.1                # on the radius, mm: the hole is d + 0.2
MIN_WALL = 3.0                 # block material all round the hole, mm

# the dialog's inputs: id, label, default (mm)
INPUTS = [
    ('L_b', 'Block length L_b', 60.0),
    ('W_b', 'Block width W_b', 40.0),
    ('H_b', 'Block height H_b', 20.0),
    ('d', 'Shaft diameter d', 10.0),
    ('L_s', 'Shaft length L_s', 50.0),
    ('x', 'Shaft position x', 15.0),
    ('y', 'Shaft position y', 0.0),
]
DEFAULTS = {k: v for k, _, v in INPUTS}

JOINT_NAME = 'Shaft turns'

handlers = []                  # keeps the event handlers alive while the dialog is open
_keep_alive = False            # True when Fusion started the script and was asked to keep it loaded


def _num(v):
    return '%g' % round(v, 3)


# ---------- the rules, checked before anything is drawn ----------
def derive(L_b, W_b, H_b, d, L_s, x, y):
    """The derived sizes, or a ValueError that says why the inputs are refused."""
    for name, v in (('L_b', L_b), ('W_b', W_b), ('H_b', H_b), ('d', d), ('L_s', L_s)):
        if not v > 0:
            raise ValueError('%s must be greater than 0 mm (it is %s mm).' % (name, _num(v)))
    if not L_s > H_b:
        raise ValueError('The shaft must be longer than the block is high: L_s = %s mm is not more than H_b = %s mm.'
                         % (_num(L_s), _num(H_b)))
    d_hole = d + 2 * CLEARANCE
    wall_x = L_b / 2.0 - abs(x) - d_hole / 2.0
    wall_y = W_b / 2.0 - abs(y) - d_hole / 2.0
    for axis, wall, half, pos in (('x', wall_x, L_b / 2.0, x), ('y', wall_y, W_b / 2.0, y)):
        if wall < MIN_WALL - 1e-9:
            raise ValueError('Only %s mm of block material is left round the hole in %s (%s - %s - %s = %s mm); at '
                             'least %s mm is needed. Move the shaft towards the centre or make the block bigger.'
                             % (_num(wall), axis, _num(half), _num(abs(pos)), _num(d_hole / 2.0), _num(wall),
                                _num(MIN_WALL)))
    return dict(L_b=L_b, W_b=W_b, H_b=H_b, d=d, L_s=L_s, x=x, y=y, d_hole=d_hole,
                z_bottom=-H_b, z_top=L_s - H_b, wall_x=wall_x, wall_y=wall_y)


# ---------- geometry helpers ----------
def _profile_with_loops(sketch, n):
    for i in range(sketch.profiles.count):
        p = sketch.profiles.item(i)
        if p.profileLoops.count == n:
            return p
    raise RuntimeError('no profile with %d loops in sketch %s' % (n, sketch.name))


def _z_range_mm(body):
    """The lowest and highest z (mm, in the body's own component) of the body's faces square to z."""
    zs = []
    for f in body.faces:
        pl = adsk.core.Plane.cast(f.geometry)
        if pl and abs(abs(pl.normal.z) - 1.0) < 1e-9:
            zs.append(f.pointOnFace.z / CM)
    return min(zs), max(zs)


def _cylinder_face(body):
    for f in body.faces:
        if adsk.core.Cylinder.cast(f.geometry):
            return f
    raise RuntimeError('the shaft has no cylindrical face')


def _expect(cond, msg):
    if not cond:
        raise RuntimeError('built wrongly: ' + msg)


# ---------- the build ----------
def build(L_b=DEFAULTS['L_b'], W_b=DEFAULTS['W_b'], H_b=DEFAULTS['H_b'], d=DEFAULTS['d'], L_s=DEFAULTS['L_s'],
          x=DEFAULTS['x'], y=DEFAULTS['y']):
    """Builds the assembly in a new design and returns the closing message. All sizes in mm. Raises ValueError,
    before any document is made, if the inputs break a rule."""
    g = derive(float(L_b), float(W_b), float(H_b), float(d), float(L_s), float(x), float(y))

    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent

    # Block: its own component at the origin, one body
    occ_b = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp_b = occ_b.component
    comp_b.name = 'Block'
    sk = comp_b.sketches.add(comp_b.xYConstructionPlane)
    sk.name = 'Block outline and hole'
    sk.sketchCurves.sketchLines.addCenterPointRectangle(P3(0, 0, 0), P3(g['L_b'] / 2 * CM, g['W_b'] / 2 * CM, 0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(g['x'] * CM, g['y'] * CM, 0), g['d_hole'] / 2 * CM)
    ext = comp_b.features.extrudeFeatures
    inp = ext.createInput(_profile_with_loops(sk, 2), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(g['H_b'] * CM)),
                         adsk.fusion.ExtentDirections.NegativeExtentDirection)
    block_body = ext.add(inp).bodies.item(0)
    block_body.name = 'Block'
    occ_b.isGrounded = True

    # Shaft: its own component created where it belongs, at (x, y, 0); drawn at its own origin
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(g['x'] * CM, g['y'] * CM, 0.0)
    occ_s = root.occurrences.addNewComponent(m)
    comp_s = occ_s.component
    comp_s.name = 'Shaft'
    sk = comp_s.sketches.add(comp_s.xYConstructionPlane)
    sk.name = 'Shaft section'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), g['d'] / 2 * CM)
    ext = comp_s.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.setTwoSidesExtent(adsk.fusion.DistanceExtentDefinition.create(VI(g['z_top'] * CM)),        # up, along +z
                          adsk.fusion.DistanceExtentDefinition.create(VI(-g['z_bottom'] * CM)))    # down to -H_b
    shaft_body = ext.add(inp).bodies.item(0)
    shaft_body.name = 'Shaft'

    # what was built must be what the prompt asked for
    zb = _z_range_mm(block_body)
    zs = _z_range_mm(shaft_body)
    _expect(abs(zb[0] - g['z_bottom']) < 1e-6 and abs(zb[1]) < 1e-6,
            'block from z = %.4f to %.4f mm, wanted %s to 0' % (zb[0], zb[1], _num(g['z_bottom'])))
    _expect(abs(zs[0] - g['z_bottom']) < 1e-6 and abs(zs[1] - g['z_top']) < 1e-6,
            'shaft from z = %.4f to %.4f mm, wanted %s to %s' % (zs[0], zs[1], _num(g['z_bottom']), _num(g['z_top'])))

    # As-built Joint: revolute, the Shaft turning on the Block about the shaft's own axis (z through its centre)
    face = _cylinder_face(shaft_body).createForAssemblyContext(occ_s)
    geo = adsk.fusion.JointGeometry.createByNonPlanarFace(face, adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
    jin = root.asBuiltJoints.createInput(occ_s, occ_b, geo)
    jin.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    joint = root.asBuiltJoints.add(jin)
    joint.name = JOINT_NAME

    try:
        app.activeViewport.fit()
    except Exception:
        pass

    t = occ_s.transform2.translation
    sx, sy, sz = t.x / CM, t.y / CM, t.z / CM
    v_s = shaft_body.volume * 1000.0
    v_b = block_body.volume * 1000.0
    w_s = math.pi * (g['d'] / 2) ** 2 * g['L_s']
    w_b = g['L_b'] * g['W_b'] * g['H_b'] - math.pi * (g['d_hole'] / 2) ** 2 * g['H_b']
    return ('%s\n\n'
            'Shaft at (%.3f, %.3f) mm, z = %.3f mm; it runs from z = %s to %s mm.\n'
            'Shaft volume %.1f mm^3 (pi * %s^2 * %s = %.1f).\n'
            'Block volume %.1f mm^3 (%s * %s * %s - pi * %s^2 * %s = %.1f).\n'
            'Hole %s mm; wall left round it %s mm in x, %s mm in y.\n'
            'Joint "%s": As-built, revolute about z through the shaft\'s centre; the Block is grounded.'
            % (VERSION, sx, sy, sz, _num(g['z_bottom']), _num(g['z_top']),
               v_s, _num(g['d'] / 2), _num(g['L_s']), w_s,
               v_b, _num(g['L_b']), _num(g['W_b']), _num(g['H_b']), _num(g['d_hole'] / 2), _num(g['H_b']), w_b,
               _num(g['d_hole']), _num(g['wall_x']), _num(g['wall_y']), JOINT_NAME))


# ---------- the dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.command.commandInputs
            vals = {k: ins.itemById(k).value / CM for k, _, _ in INPUTS}     # the inputs give cm
            try:
                msg = build(**vals)
            except ValueError as e:
                ui.messageBox('%s\n\nRefused, nothing was drawn:\n%s' % (VERSION, e))
                return
            ui.messageBox(msg)
        except Exception:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _keep_alive:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            for k, label, v in INPUTS:
                ins.addValueInput(k, label + ' (mm)', 'mm', VS('%s mm' % _num(v)))
            h = _Execute()
            cmd.execute.add(h)
            handlers.append(h)
            h = _Destroy()
            cmd.destroy.add(h)
            handlers.append(h)
        except Exception:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


def run(context):
    """Opens the dialog. Fusion passes a context; a caller that drives the dialog itself passes None and keeps the
    handlers alive on its own, so only a run started by Fusion asks to stay loaded until the dialog closes."""
    global _keep_alive
    try:
        cmd_id = 'mdfcFirstAssembly'
        cd = ui.commandDefinitions.itemById(cmd_id)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(cmd_id, 'First assembly',
                                                        'A block with a shaft in a hole, on a revolute As-built Joint')
        h = _Created()
        cd.commandCreated.add(h)
        handlers.append(h)
        cd.execute()
        if context is not None:
            _keep_alive = True
            adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


def stop(context):
    pass
