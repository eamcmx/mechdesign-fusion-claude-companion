# =============================================================================
# BeltDrive.py  -  a GT2 timing-belt drive: two toothed pulleys on shafts and the belt round them. The centre
# distance is found by bisection so that a belt of a whole number of teeth fits; the parts are put together with
# As-built Joints and a Motion Link, so that Drive Joints on pulley 1 turns pulley 2 the same way at z1/z2.
#
#   GT2: pitch 2 mm, PD = z p / pi, OD = PD - 0.508 mm, belt back 0.63 mm outside the OD (teeth not drawn)
#   L = 2 C cos(phi) + pi (a + b) + 2 phi (b - a), sin(phi) = (b - a) / C, solved for L = N_b p
#
#   Every size below is in millimetres; the API works in centimetres and radians (CM converts).
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


# ---------- the companion's libraries (walk up to the folder that holds common/companion_paths.py) ----------
def _companion_common():
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


_COMMON = _companion_common()
if _COMMON and _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
if _COMMON:
    import companion_paths as CP  # noqa: F401  (puts every lib/ on sys.path; this script needs none of them)

VERSION = 'BeltDrive v1.0 (prompt test, 12 Nov 2026)'

CM = 0.1                    # mm to cm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal

PITCH = 2.0                 # GT2 pitch, mm
OD_LESS = 0.508             # OD = PD - 0.508 mm
BACK = 0.63                 # belt thickness outside the pulleys' OD, mm
GROOVE_R = 0.6              # radius of the groove circles cut on the OD, mm
BORE = 5.2                  # pulley bore, mm
SHAFT_D = 5.0               # shaft diameter, mm
COLLAR_D = 8.0              # collar under each pulley, mm (not given by the prompt)
COLLAR_TOP = 1.0            # collar from the plate's top (z = 0) up to the pulley's underside (z = 1)
PULLEY_Z0 = 1.0             # pulleys from z = 1 to z = 1 + width + 2
BELT_Z0 = 2.0               # belt from z = 2 to z = 2 + width
SHAFT_ABOVE = 3.0           # shaft end above the pulley's top face, mm (not given)
PLATE_T = 5.0               # base plate thickness, mm (not given)
PLATE_MARGIN = 10.0         # base plate margin round the belt, mm (not given)
CLEAR = 2.0                 # the pulleys' outside diameters at least 2 mm apart
MIN_MESH = 6.0              # teeth in mesh on pulley 1, at least

DEFAULTS = dict(z1=20, z2=40, n_b=90, width=6.0)


# ---------- the drive's numbers (pure Python, no Fusion) ----------
def pitch_length(C, a, b):
    """Pitch length of an open belt round pulleys of pitch radii a and b whose centres are C apart (mm)."""
    s = 0.0 if a == b else (b - a) / C
    phi = math.asin(max(-1.0, min(1.0, s)))
    return 2.0 * C * math.cos(phi) + math.pi * (a + b) + 2.0 * phi * (b - a)


def _whole(v, name):
    if abs(v - round(v)) > 1e-9 or round(v) < 1:
        raise ValueError('%s must be a whole number of at least 1 (got %s)' % (name, v))
    return int(round(v))


def belt(z1, z2, n_b):
    """The drive's numbers for pulleys of z1 and z2 teeth and a belt of n_b teeth: a dict of mm and radians.
    Raises ValueError when the drive cannot be built."""
    z1, z2, n_b = _whole(z1, 'z1'), _whole(z2, 'z2'), _whole(n_b, 'N_b')
    pd1, pd2 = z1 * PITCH / math.pi, z2 * PITCH / math.pi
    od1, od2 = pd1 - OD_LESS, pd2 - OD_LESS
    for z, od in ((z1, od1), (z2, od2)):
        if od / 2.0 - GROOVE_R <= BORE / 2.0:
            raise ValueError('A pulley of %d teeth is too small: OD %.3f mm, its grooves would cut into the %.1f mm '
                             'bore' % (z, od, BORE))
    a, b = pd1 / 2.0, pd2 / 2.0
    L_want = n_b * PITCH
    # L(C) rises with C (dL/dC = 2 cos(phi) > 0); its least value, at C = |b - a|, is 2 pi max(a, b)
    L_min = 2.0 * math.pi * max(a, b)
    if L_want <= L_min + 1e-9:
        raise ValueError('The belt is too short: %d teeth (%.3f mm) do not reach round the pulleys; even with the '
                         'centres |b - a| apart the belt would be %.3f mm' % (n_b, L_want, L_min))
    lo, hi = abs(b - a), max(L_want, 1.0)
    while pitch_length(hi, a, b) < L_want:
        hi *= 2.0
    for _ in range(400):
        mid = 0.5 * (lo + hi)
        if pitch_length(mid, a, b) < L_want:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-13:
            break
    C = 0.5 * (lo + hi)
    phi = math.asin((b - a) / C)
    c_min = (od1 + od2) / 2.0 + CLEAR
    if not C > c_min:
        raise ValueError('The pulleys would touch: C = %.3f mm, it must be more than (OD1 + OD2)/2 + %.0f mm = %.3f mm'
                         % (C, CLEAR, c_min))
    wrap1, wrap2 = math.pi - 2.0 * phi, math.pi + 2.0 * phi
    mesh1 = z1 * wrap1 / (2.0 * math.pi)
    if mesh1 < MIN_MESH:
        raise ValueError('Only %.2f teeth in mesh on pulley 1; at least %d are needed' % (mesh1, int(MIN_MESH)))
    rm1, rm2 = od1 / 2.0 + BACK / 2.0, od2 / 2.0 + BACK / 2.0       # the belt's centre line
    return dict(z1=z1, z2=z2, n_b=n_b, pd1=pd1, pd2=pd2, od1=od1, od2=od2, a=a, b=b, C=C, phi=phi,
                wrap1=wrap1, wrap2=wrap2, mesh1=mesh1, ratio=z2 / float(z1), L=L_want,
                centre_line=pitch_length(C, rm1, rm2))


# ---------- Fusion helpers ----------
def _new_occ(root, name, x, y, z):
    """A new component in the root, its occurrence moved by its transform so its own origin is at (x, y, z) mm."""
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(m)
    occ.component.name = name
    return occ


def _origin_point(comp):
    """A sketch point at the component's own origin, for its joint."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Joint origin'
    return sk.sketchPoints.add(P3(0, 0, 0))


def _profiles(sk, loops):
    return [sk.profiles.item(i) for i in range(sk.profiles.count) if sk.profiles.item(i).profileLoops.count == loops]


def _one_body(comp, name):
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s has %d bodies, not one' % (name, comp.bRepBodies.count))
    comp.bRepBodies.item(0).name = name
    return comp.bRepBodies.item(0)


def _plate(root, k):
    occ = _new_occ(root, 'Base plate', 0, 0, 0)
    comp = occ.component
    r1 = k['od1'] / 2.0 + BACK + PLATE_MARGIN
    r2 = k['od2'] / 2.0 + BACK + PLATE_MARGIN
    ry = max(r1, r2)
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Plate'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-r1 * CM, -ry * CM, 0), P3((k['C'] + r2) * CM, ry * CM, 0))
    comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), VI(-PLATE_T * CM),
                                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)  # top at z = 0
    _one_body(comp, 'Base plate')
    occ.isGrounded = True
    return occ


def _shaft(root, name, x, top):
    """A 5 mm shaft standing on the plate (z = 0 to `top`), a collar under its pulley from z = 0 to z = 1."""
    occ = _new_occ(root, name, x, 0, 0)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Shaft and collar'
    circles = sk.sketchCurves.sketchCircles
    circles.addByCenterRadius(P3(0, 0, 0), SHAFT_D / 2.0 * CM)
    circles.addByCenterRadius(P3(0, 0, 0), COLLAR_D / 2.0 * CM)
    core, ring = _profiles(sk, 1)[0], _profiles(sk, 2)[0]
    ext = comp.features.extrudeFeatures
    body = ext.addSimple(core, VI(top * CM), adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    inp = ext.createInput(ring, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(COLLAR_TOP * CM)),
                         adsk.fusion.ExtentDirections.PositiveExtentDirection)
    inp.participantBodies = [body]
    ext.add(inp)
    _one_body(comp, name)
    return occ, _origin_point(comp)


def _pulley(root, name, x, z, od, height):
    """A pulley from z = 1 to z = 1 + height: its outside diameter, z grooves cut as circles of radius 0.6 mm
    centred on it, and the bore."""
    occ = _new_occ(root, name, x, 0, PULLEY_Z0)
    comp = occ.component
    ext = comp.features.extrudeFeatures
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Outside diameter and bore'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), od / 2.0 * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), BORE / 2.0 * CM)
    body = ext.addSimple(_profiles(sk, 2)[0], VI(height * CM),
                         adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    sg = comp.sketches.add(comp.xYConstructionPlane)
    sg.name = 'Grooves'
    for i in range(z):
        t = 2.0 * math.pi * i / z
        sg.sketchCurves.sketchCircles.addByCenterRadius(P3(od / 2.0 * math.cos(t) * CM, od / 2.0 * math.sin(t) * CM, 0),
                                                        GROOVE_R * CM)
    grooves = adsk.core.ObjectCollection.create()
    for i in range(sg.profiles.count):
        grooves.add(sg.profiles.item(i))
    inp = ext.createInput(grooves, adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setTwoSidesExtent(adsk.fusion.DistanceExtentDefinition.create(VI((height + 1.0) * CM)),
                          adsk.fusion.DistanceExtentDefinition.create(VI(1.0 * CM)))
    inp.participantBodies = [body]
    ext.add(inp)
    _one_body(comp, name)
    return occ, _origin_point(comp)


def _near(arc, x, y):
    """The end of the arc nearer to (x, y) mm."""
    best = None
    for sp in (arc.startSketchPoint, arc.endSketchPoint):
        g = sp.geometry
        d = math.hypot(g.x - x * CM, g.y - y * CM)
        if best is None or d < best[0]:
            best = (d, sp)
    return best[1]


def _loop(sk, r1, r2, C, phi):
    """One closed outline of the belt: an arc round each pulley and the two outside tangents, the lines drawn
    between the arcs' own end points. Tangent points: centre + r (-sin(phi), +-cos(phi))."""
    s, c = math.sin(phi), math.cos(phi)
    arcs, lines = sk.sketchCurves.sketchArcs, sk.sketchCurves.sketchLines
    t1u, t1l = (-r1 * s, r1 * c), (-r1 * s, -r1 * c)
    t2u, t2l = (C - r2 * s, r2 * c), (C - r2 * s, -r2 * c)
    a1 = arcs.addByCenterStartSweep(P3(0, 0, 0), P3(t1u[0] * CM, t1u[1] * CM, 0), math.pi - 2.0 * phi)
    a2 = arcs.addByCenterStartSweep(P3(C * CM, 0, 0), P3(t2l[0] * CM, t2l[1] * CM, 0), math.pi + 2.0 * phi)
    lines.addByTwoPoints(_near(a1, *t1l), _near(a2, *t2l))
    lines.addByTwoPoints(_near(a2, *t2u), _near(a1, *t1u))


def _belt(root, k, width):
    """The belt as a band 0.63 mm thick resting on the pulleys' outside diameters, from z = 2 to z = 2 + width."""
    occ = _new_occ(root, 'Belt', 0, 0, BELT_Z0)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Belt outline'
    ri1, ri2 = k['od1'] / 2.0, k['od2'] / 2.0
    _loop(sk, ri1, ri2, k['C'], k['phi'])                       # inside
    _loop(sk, ri1 + BACK, ri2 + BACK, k['C'], k['phi'])         # outside
    band = _profiles(sk, 2)
    if len(band) != 1:
        raise RuntimeError('the belt outline gave %d band profiles, not one' % len(band))
    comp.features.extrudeFeatures.addSimple(band[0], VI(width * CM),
                                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = _one_body(comp, 'Belt')
    return occ, _origin_point(comp), body


def _joint(root, moving, fixed, point, name, revolute=False):
    """An As-built Joint on a sketch point at the moving part's own origin, the moving occurrence first."""
    geo = adsk.fusion.JointGeometry.createByPoint(point.createForAssemblyContext(moving))
    inp = root.asBuiltJoints.createInput(moving, fixed, geo)
    if revolute:
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    if j is None:
        raise RuntimeError('the joint %s was not made' % name)
    j.name = name
    return j


# ---------- the build ----------
def build(z1=DEFAULTS['z1'], z2=DEFAULTS['z2'], n_b=DEFAULTS['n_b'], width=DEFAULTS['width']):
    """Builds the drive in a new design and returns the message. Raises ValueError (before any document is made)
    when the drive cannot be built."""
    k = belt(z1, z2, n_b)
    width = float(width)
    if not width > 0.0:
        raise ValueError('The belt width must be more than 0 mm (got %s)' % width)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    height = width + 2.0                                     # pulleys from z = 1 to z = 1 + width + 2
    top = PULLEY_Z0 + height + SHAFT_ABOVE
    plate = _plate(root, k)
    s1, s1p = _shaft(root, 'Shaft 1', 0.0, top)
    s2, s2p = _shaft(root, 'Shaft 2', k['C'], top)
    p1, p1p = _pulley(root, 'Pulley 1', 0.0, k['z1'], k['od1'], height)
    p2, p2p = _pulley(root, 'Pulley 2', k['C'], k['z2'], k['od2'], height)
    bo, bop, body = _belt(root, k, width)

    _joint(root, s1, plate, s1p, 'Shaft 1 fixed')
    _joint(root, s2, plate, s2p, 'Shaft 2 fixed')
    _joint(root, bo, plate, bop, 'Belt fixed')
    j1 = _joint(root, p1, s1, p1p, 'Pulley 1 turns', revolute=True)
    j2 = _joint(root, p2, s2, p2p, 'Pulley 2 turns', revolute=True)

    ml = root.motionLinks.createInput(j1, j2)
    ml.valueOne = VI(math.radians(k['z2'] * 360.0))           # z2 turns of pulley 1 ...
    ml.valueTwo = VI(math.radians(k['z1'] * 360.0))           # ... for z1 turns of pulley 2
    ml.isReversed = False                                     # a belt keeps the direction
    link = root.motionLinks.add(ml)
    if link is None:
        raise RuntimeError('the Motion Link was not made')
    try:
        link.name = 'Belt ratio'
    except Exception:
        pass

    v_body = body.volume * 1000.0
    v_band = width * BACK * k['centre_line']
    app.activeViewport.fit()
    return ('GT2 belt drive: z1 = %d, z2 = %d, belt %d teeth (pitch length %.1f mm), %.1f mm wide\n'
            'PD1 = %.4f mm, PD2 = %.4f mm (OD %.4f and %.4f mm)\n'
            'C = %.3f mm\n'
            'wrap angles: pulley 1 %.3f deg, pulley 2 %.3f deg\n'
            'teeth in mesh on pulley 1: %.2f\n'
            'ratio z2/z1 = %.3f, both pulleys turning the same way\n'
            'belt volume %.3f mm^3 (width x thickness x centre line %.3f mm^3)\n'
            '%s'
            % (k['z1'], k['z2'], k['n_b'], k['L'], width, k['pd1'], k['pd2'], k['od1'], k['od2'], k['C'],
               math.degrees(k['wrap1']), math.degrees(k['wrap2']), k['mesh1'], k['ratio'], v_body, v_band, VERSION))


# ---------- the dialog ----------
CMD_ID = 'mdfcBeltDriveCommand'
_handlers = []


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addIntegerSpinnerCommandInput('z1', 'Pulley 1 teeth z1', 1, 500, 1, DEFAULTS['z1'])
            ins.addIntegerSpinnerCommandInput('z2', 'Pulley 2 teeth z2', 1, 500, 1, DEFAULTS['z2'])
            ins.addIntegerSpinnerCommandInput('n_b', 'Belt teeth N_b (count)', 1, 5000, 1, DEFAULTS['n_b'])
            ins.addValueInput('width', 'Belt width', 'mm', VI(DEFAULTS['width'] * CM))
            on_execute = _Execute()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui.messageBox('Belt drive dialog failed:\n%s' % traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            msg = build(z1=ins.itemById('z1').value, z2=ins.itemById('z2').value, n_b=ins.itemById('n_b').value,
                        width=ins.itemById('width').value / CM)
            ui.messageBox(msg, 'Belt drive')
        except ValueError as e:
            ui.messageBox('Refused: %s\n%s' % (e, VERSION), 'Belt drive')
        except Exception:
            ui.messageBox('Belt drive failed:\n%s\n%s' % (traceback.format_exc(), VERSION))


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
        del _handlers[:]
        cdef = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'GT2 belt drive',
                                                         'Two GT2 pulleys on shafts and the belt round them')
        on_created = _Created()
        cdef.commandCreated.add(on_created)
        _handlers.append(on_created)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('Belt drive failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cdef = ui.commandDefinitions.itemById(CMD_ID)
        if cdef:
            cdef.deleteMe()
    except Exception:
        pass
