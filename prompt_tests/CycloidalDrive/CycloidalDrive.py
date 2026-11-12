# =============================================================================
# CycloidalDrive.py  -  a single-disc cycloidal drive, assembled so that Drive Joints on the input shaft turns the
# disc on its eccentric and the output flange at the reduced speed.
#
#   Housing (grounded)  plate z = -6 to 0 with a bore for the input; N_p ring pins on radius R up to z = t + 2
#   Input               8 mm shaft z = -6 to t + 8; eccentric r = 10 mm about (e, 0), z = 1 to 1 + t
#   Disc                cycloidal outline (one closed polyline), bore 10.1 mm about its centre, 6 output holes
#   Output              flange z = t + 2 to t + 6 with 6 pins of 5 mm on radius 17 mm down to z = 1
#
#   As-built joints: Input on Housing, Disc on Input at the eccentric's centre, Output on Housing.
#   Motion links:    Input to Disc  N_d x 360 deg : (N_d + 1) x 360 deg, reversed
#                    Input to Output N_d x 360 deg : 360 deg, reversed
#
# Sizes are in millimetres here; the API works in centimetres and radians, so every length goes in times CM.
# =============================================================================
import math
import os
import sys
import traceback

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
            raise RuntimeError('common/companion_paths.py not found above %s' % __file__)
        d = up


_COMMON = _companion_common()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP          # noqa: E402  puts every lib/ folder on sys.path
import mdfc_parts as MP               # noqa: E402  the parts library: cyl, union, cut, new_occ, add_body

VERSION = 'CycloidalDrive v1.0 (prompt test)'
CMD_ID = 'mdfcCycloidalDriveCmd'

CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

DEFAULTS = dict(n_p=11, R=30.0, d_pin=6.0, e=1.5, t=8.0)

GAP = 0.1                  # disc outline to ring pins (mm)
PTS_PER_TURN = 36          # outline points per input turn
SHAFT_D = 8.0              # input shaft diameter
ECC_R = 10.0               # eccentric radius
BORE_GAP = 0.1             # disc bore over the eccentric, in radius
OUT_N, OUT_D, OUT_RC = 6, 5.0, 17.0     # output pins: count, diameter, circle radius
SHAFT_GAP = 0.1            # housing and flange bores over the input shaft, in radius (not given by the prompt)

_handlers = []
_standalone = False


# ---------- the design calculation (no Fusion needed) ----------
def cycloid(n_p, R, d_pin, e):
    """The derived numbers of the drive, after the hard constraints. Raises ValueError to refuse."""
    if int(n_p) != n_p or not 6 <= n_p <= 40:
        raise ValueError('N_p = %s is outside 6 to 40' % n_p)
    n_p = int(n_p)
    if R <= 0 or d_pin <= 0 or e <= 0:
        raise ValueError('R, the pin diameter and e must all be more than 0')
    spacing = 2.0 * R * math.sin(math.pi / n_p)
    if not spacing > d_pin + 1.0:
        raise ValueError('the ring pins overlap on their circle: 2 R sin(pi / N_p) = %.2f mm, not more than the pin '
                         'diameter + 1 mm = %.2f mm' % (spacing, d_pin + 1.0))
    K1 = e * n_p / R
    if not K1 < 1.0:
        raise ValueError('the pin centre curve loops: K1 = e N_p / R = %.3f, it must be less than 1' % K1)
    n_d = n_p - 1
    r_pin = d_pin / 2.0
    return dict(n_p=n_p, n_d=n_d, ratio=-n_d, K1=K1, r_pin=r_pin, gap=GAP, spacing=spacing,
                r_min=R - r_pin - GAP - e, r_max=R - r_pin - GAP + e)


def _pin_centre(n_p, R, e, theta):
    """Ring pin 0, at (R, 0), seen from the disc's frame when the input is at theta: the disc's centre is at
    e (cos theta, sin theta) and the disc has turned -theta / N_d. Returns the point and the unit normal of the
    curve that points towards the disc's centre."""
    n_d = n_p - 1
    a = theta / n_d
    c, s = math.cos(a), math.sin(a)
    vx, vy = R - e * math.cos(theta), -e * math.sin(theta)          # pin minus disc centre, world axes
    qx, qy = c * vx - s * vy, s * vx + c * vy                        # turned by +theta / N_d into the disc's frame
    # d q / d theta = Rot(a) (e N_p sin theta, R - e N_p cos theta) / N_d; the curve runs anticlockwise (K1 < 1),
    # so its left-hand normal points inwards
    tx, ty = e * n_p * math.sin(theta), R - e * n_p * math.cos(theta)
    L = math.hypot(tx, ty)
    nx, ny = -ty / L, tx / L
    return (qx, qy), (c * nx - s * ny, s * nx + c * ny)


def disc_outline(n_p, R, d_pin, e):
    """The disc's outline in its own frame (mm, centred on the disc's centre): the ring pin's centre curve over N_d
    input turns, offset towards the centre by the pin radius plus the gap; 36 points per input turn."""
    k = cycloid(n_p, R, d_pin, e)
    off = k['r_pin'] + GAP
    n = PTS_PER_TURN * k['n_d']
    pts = []
    for i in range(n):
        th = 2.0 * math.pi * i / PTS_PER_TURN
        (qx, qy), (nx, ny) = _pin_centre(k['n_p'], R, e, th)
        pts.append((qx + off * nx, qy + off * ny))
    return pts


# ---------- Fusion ----------
def _new_design():
    """A new Fusion design document, parametric, able to hold components."""
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        if des.designIntent == adsk.fusion.DesignIntentTypes.PartDesignIntentType:
            des.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    except Exception:
        pass
    return des


def _point(occ, x, y):
    """A sketch point at (x, y, 0) mm in the occurrence's component, as a proxy in the assembly."""
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'joint point'
    sp = sk.sketchPoints.add(P3(x * CM, y * CM, 0.0))
    return sp.createForAssemblyContext(occ)


def _turn_deg(occ):
    """The occurrence's turn about z, in degrees."""
    m = occ.transform2
    return math.degrees(math.atan2(m.getCell(1, 0), m.getCell(0, 0)))


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _revolute(root, one, two, point, name):
    geo = adsk.fusion.JointGeometry.createByPoint(point)
    ji = root.asBuiltJoints.createInput(one, two, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def _link(root, j1, j2, deg1, deg2, name):
    mi = root.motionLinks.createInput(j1, j2)
    mi.valueOne = VS('%d deg' % deg1)
    mi.valueTwo = VS('%d deg' % deg2)
    mi.isReversed = True
    link = root.motionLinks.add(mi)
    try:
        link.name = name
    except Exception:
        pass
    return link


def _disc_body(occ, outline, n_p, R, d_pin, e, t):
    """The disc from z = 1 to 1 + t, drawn in place for theta = 0 (its centre at (e, 0), not turned)."""
    comp = occ.component
    pi_ = comp.constructionPlanes.createInput()
    pi_.setByOffset(comp.xYConstructionPlane, VI(1.0 * CM))
    plane = comp.constructionPlanes.add(pi_)
    sk = comp.sketches.add(plane)
    sk.name = 'disc outline'
    sk.isComputeDeferred = True
    lines = sk.sketchCurves.sketchLines
    pts = [sk.modelToSketchSpace(P3((x + e) * CM, y * CM, 1.0 * CM)) for x, y in outline]
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    circles = sk.sketchCurves.sketchCircles
    circles.addByCenterRadius(sk.modelToSketchSpace(P3(e * CM, 0.0, 1.0 * CM)), (ECC_R + BORE_GAP) * CM)
    hole_r = (OUT_D + 2.0 * e + 0.2) / 2.0
    for i in range(OUT_N):
        a = 2.0 * math.pi * i / OUT_N
        c = P3((e + OUT_RC * math.cos(a)) * CM, OUT_RC * math.sin(a) * CM, 1.0 * CM)
        circles.addByCenterRadius(sk.modelToSketchSpace(c), hole_r * CM)
    sk.isComputeDeferred = False
    prof = max((sk.profiles.item(i) for i in range(sk.profiles.count)), key=lambda p: p.profileLoops.count)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    normal = sk.xDirection.crossProduct(sk.yDirection)
    inp.setDistanceExtent(False, VI(t * CM * (1 if normal.z > 0 else -1)))
    body = ext.add(inp).bodies.item(0)
    body.name = 'Disc'
    return body


def build(n_p=DEFAULTS['n_p'], R=DEFAULTS['R'], d_pin=DEFAULTS['d_pin'], e=DEFAULTS['e'], t=DEFAULTS['t'],
          new_document=True):
    """Builds the drive and returns the message. Refuses (ValueError) before touching Fusion. new_document=False
    builds into the active design (the dialog's command, where a document cannot be created)."""
    k = cycloid(n_p, R, d_pin, e)
    if t <= 0:
        raise ValueError('the disc thickness t must be more than 0')
    n_p, n_d, r_pin = k['n_p'], k['n_d'], k['r_pin']
    outline = disc_outline(n_p, R, d_pin, e)
    rs = [math.hypot(x, y) for x, y in outline]

    app = adsk.core.Application.get()
    des = _new_design() if new_document else adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    r_shaft = SHAFT_D / 2.0

    # Housing: the plate with its bore, the ring pins standing on it (started 1 mm inside the plate so they join)
    housing = MP.new_occ(root, 'Housing')
    plate = MP.cyl((0, 0, -6.0), (0, 0, 0.0), R + d_pin)
    MP.cut(plate, MP.cyl((0, 0, -7.0), (0, 0, 1.0), r_shaft + SHAFT_GAP))
    for i in range(n_p):
        a = 2.0 * math.pi * i / n_p
        x, y = R * math.cos(a), R * math.sin(a)
        MP.union(plate, MP.cyl((x, y, -1.0), (x, y, t + 2.0), r_pin))
    MP.add_body(housing, plate, 'Housing')
    housing.isGrounded = True

    # Input: the shaft and the eccentric about (e, 0)
    inp = MP.new_occ(root, 'Input')
    shaft = MP.cyl((0, 0, -6.0), (0, 0, t + 8.0), r_shaft)
    MP.union(shaft, MP.cyl((e, 0, 1.0), (e, 0, 1.0 + t), ECC_R))
    MP.add_body(inp, shaft, 'Input')

    # Disc
    disc = MP.new_occ(root, 'Disc')
    _disc_body(disc, outline, n_p, R, d_pin, e, t)

    # Output: the flange (clear of the ring pins, bored for the shaft) and its pins (started inside the flange)
    out = MP.new_occ(root, 'Output')
    flange = MP.cyl((0, 0, t + 2.0), (0, 0, t + 6.0), R - r_pin - 1.0)
    MP.cut(flange, MP.cyl((0, 0, t + 1.0), (0, 0, t + 7.0), r_shaft + SHAFT_GAP))
    for i in range(OUT_N):
        a = 2.0 * math.pi * i / OUT_N
        x, y = OUT_RC * math.cos(a), OUT_RC * math.sin(a)
        MP.union(flange, MP.cyl((x, y, 1.0), (x, y, t + 3.0), OUT_D / 2.0))
    MP.add_body(out, flange, 'Output')

    # Joints. The input's turn is measured once: if a positive joint value turns the Input clockwise, every joint
    # is made with its two occurrences the other way round.
    p_h = _point(housing, 0.0, 0.0)
    p_ecc = _point(inp, e, 0.0)
    order = 1
    j_in = _revolute(root, inp, housing, p_h, 'Input turns')
    j_in.jointMotion.rotationValue = math.radians(90.0)
    if _turn_deg(inp) < -45.0:
        j_in.jointMotion.rotationValue = 0.0
        j_in.deleteMe()
        order = -1
        j_in = _revolute(root, housing, inp, _point(housing, 0.0, 0.0), 'Input turns')
    j_in.jointMotion.rotationValue = 0.0

    def pair(moving, base):
        return (moving, base) if order == 1 else (base, moving)

    j_disc = _revolute(root, *pair(disc, inp), p_ecc, 'Disc on eccentric')
    j_out = _revolute(root, *pair(out, housing), _point(housing, 0.0, 0.0), 'Output turns')
    l_disc = _link(root, j_in, j_disc, n_d * 360, (n_d + 1) * 360, 'Input to Disc')
    l_out = _link(root, j_in, j_out, n_d * 360, 360, 'Input to Output')

    # One measured step of 30 deg: where a link plainly turned its part the mirrored way, reverse it.
    j_in.jointMotion.rotationValue = math.radians(30.0)
    if abs(_turn_deg(inp) - 30.0) < 0.5:
        if abs(_wrap(_turn_deg(out) - 30.0 / n_d)) < 0.5:
            l_out.isReversed = not l_out.isReversed
        if abs(_wrap(_turn_deg(disc) - (30.0 + 30.0 * (n_d + 1) / n_d))) < 0.5:
            l_disc.isReversed = not l_disc.isReversed
    j_in.jointMotion.rotationValue = 0.0

    app.activeViewport.fit()
    return ('%s\n\nN_d = %d lobes, ratio %d (the output turns the other way, 1/%d of the input)\n'
            'K1 = e N_p / R = %.3f\n'
            'disc outline from %.2f to %.2f mm from its centre (%d points), gap %.1f mm to the ring pins\n'
            'disc turns -theta/%d, -theta x %d/%d on the eccentric\n'
            'output holes %.1f mm on radius %.0f mm, disc bore %.1f mm in radius'
            % (VERSION, n_d, k['ratio'], n_d, k['K1'], min(rs), max(rs), len(outline), GAP, n_d, n_d + 1, n_d,
               OUT_D + 2.0 * e + 0.2, OUT_RC, ECC_R + BORE_GAP))


# ---------- the dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = dict(n_p=ins.itemById('n_p').value,
                        R=ins.itemById('R').value / CM,
                        d_pin=ins.itemById('d_pin').value / CM,
                        e=ins.itemById('e').value / CM,
                        t=ins.itemById('t').value / CM)
            msg = build(new_document=False, **vals)
            ui.messageBox(msg)
        except ValueError as ex:
            ui.messageBox('Refused: %s\n\n%s' % (ex, VERSION))
        except Exception:
            ui.messageBox('CycloidalDrive failed:\n%s' % traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _standalone:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addIntegerSpinnerCommandInput('n_p', 'Number of ring pins N_p', 1, 200, 1, DEFAULTS['n_p'])
        ins.addValueInput('R', 'Pin circle radius R', 'mm', VI(DEFAULTS['R'] * CM))
        ins.addValueInput('d_pin', 'Ring pin diameter', 'mm', VI(DEFAULTS['d_pin'] * CM))
        ins.addValueInput('e', 'Eccentricity e', 'mm', VI(DEFAULTS['e'] * CM))
        ins.addValueInput('t', 'Disc thickness t', 'mm', VI(DEFAULTS['t'] * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        d = _Destroy()
        cmd.destroy.add(d)
        _handlers.append(d)


def run(_context):
    """Opens a new design, then the dialog; OK builds the drive into that design (Fusion does not allow a document
    to be created inside a command's events)."""
    global _standalone
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        _standalone = _context is not None          # called by another script with None: that script keeps control
        _new_design()
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Cycloidal drive', 'A single-disc cycloidal drive')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if _standalone:
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('CycloidalDrive failed:\n%s' % traceback.format_exc())


def stop(_context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
