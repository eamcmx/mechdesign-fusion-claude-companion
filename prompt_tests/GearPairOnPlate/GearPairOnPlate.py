# =============================================================================
# GearPairOnPlate.py  -  a parametric spur gear pair on a base plate, assembled with As-built Joints and a Motion
# Link so that it turns with Drive Joints.
#
#   Run: a dialog asks for the module, the two tooth counts, the pressure angle, the face width, the backlash, the
#   shaft diameter and the plate thickness. OK builds a new design:
#     Base plate (grounded), Shaft 1 and Shaft 2 standing on it, Gear 1 at the origin and Gear 2 at (a, 0) on the
#     shafts above the plate; rigid As-built Joints plate-shaft, revolute As-built Joints gear-shaft about the shaft
#     axis, and a Motion Link: gear 1 turned by an angle turns gear 2 by z1/z2 of it, the other way.
#
#   Every size the user types is in mm and degrees; the API works in cm and radians (CM, math.radians).
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_paths():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py, put common/ on
    sys.path and import it. This script needs no library of the companion; the import keeps it in the same frame as
    every other script of the book."""
    d = os.path.dirname(os.path.abspath(__file__))
    while not os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up
    common = os.path.join(d, 'common')
    if common not in sys.path:
        sys.path.insert(0, common)
    import companion_paths
    return companion_paths


try:
    CP = _companion_paths()
except Exception:
    CP = None

VERSION = 'GearPairOnPlate pt-1 (2026-11-11)'
CMD_ID = 'mdfc_GearPairOnPlate_prompt_test'

CM = 0.1                                   # mm -> cm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

# Layout the prompt leaves open (mm): the gap from the plate's top to a gear's lower face, how far a shaft stands
# above its gear, and the plate's margin round the tip circles.
GEAR_GAP = 5.0
SHAFT_ABOVE = 5.0
PLATE_MARGIN = 10.0

DEFAULTS = dict(m=2.0, z1=20, z2=40, alpha=20.0, b=10.0, backlash=0.2, d_s=8.0, plate_t=6.0)

_handlers = []
_own_lifetime = [False]


# ---------- the gear calculations (mm, radians) ----------
def inv(a):
    """The involute function, inv(a) = tan a - a."""
    return math.tan(a) - a


def z_min(alpha_deg):
    """The fewest teeth without undercut, 2 / sin^2(alpha), rounded up."""
    return int(math.ceil(2.0 / math.sin(math.radians(alpha_deg)) ** 2 - 1e-9))


def gear_sizes(m, z, alpha_deg, backlash):
    alpha = math.radians(alpha_deg)
    r = m * z / 2.0                                  # pitch radius
    rb = r * math.cos(alpha)                         # base radius
    ra = r + m                                       # tip radius
    rf = r - 1.25 * m                                # root radius
    s = math.pi * m / 2.0 - backlash / 2.0           # tooth thickness on the pitch circle
    theta_b = s / (2.0 * r) + inv(alpha)             # half the tooth's angle at the base circle

    def half(rho):
        """Half the tooth's angle at radius rho (rho >= rb on the involute; below rb the flank is radial)."""
        return theta_b - inv(math.acos(rb / rho)) if rho > rb else theta_b
    r_lo = max(rb, rf)                               # where the involute flank starts
    return dict(r=r, rb=rb, ra=ra, rf=rf, s=s, theta_b=theta_b, half=half, r_lo=r_lo,
                r_in=rf - 0.25 * m)                  # the outline closes a little inside the root circle


def tooth_points(m, z, alpha_deg, backlash, phi, n=10, n_tip=4):
    """One tooth centred on the angle phi, as a closed list of (x, y) points in mm: from inside the root circle a
    radial line out to where the involute starts (the base circle, or the root circle when that is higher), the
    involute flank to the tip circle, the tip, the other flank down and the radial line back. The closing line lies
    inside the root circle, so the tooth overlaps the gear blank and joins to it; the gear's root is the blank's
    root circle."""
    g = gear_sizes(m, z, alpha_deg, backlash)
    rb, ra, r_lo, r_in, half = g['rb'], g['ra'], g['r_lo'], g['r_in'], g['half']
    t_lo = math.sqrt(max((r_lo / rb) ** 2 - 1.0, 0.0))     # roll angle of the involute at r_lo and at the tip
    t_a = math.sqrt((ra / rb) ** 2 - 1.0)
    flank = [rb * math.sqrt(1.0 + (t_lo + (t_a - t_lo) * i / n) ** 2) for i in range(n + 1)]
    flank[0], flank[-1] = r_lo, ra
    psi_a = half(ra)
    polar = [(r_in, -half(r_lo))]
    polar += [(rho, -half(rho)) for rho in flank]
    polar += [(ra, -psi_a + 2.0 * psi_a * j / n_tip) for j in range(1, n_tip)]
    polar += [(rho, half(rho)) for rho in reversed(flank)]
    polar += [(r_in, half(r_lo))]
    pts = [(rho * math.cos(phi + th), rho * math.sin(phi + th)) for rho, th in polar]
    clean = [pts[0]]
    for q in pts[1:]:
        if math.hypot(q[0] - clean[-1][0], q[1] - clean[-1][1]) > 1e-6:
            clean.append(q)
    return clean


def check_inputs(m, z1, z2, alpha, b, backlash, d_s, plate_t):
    """Refuse what cannot be built, with a message saying why (ValueError)."""
    if not 0.0 < alpha < 45.0:
        raise ValueError('The pressure angle must be between 0 and 45 deg (it is %g deg).' % alpha)
    zm = z_min(alpha)
    for name, z in (('z1', z1), ('z2', z2)):
        if abs(z - round(z)) > 1e-9:
            raise ValueError('%s = %g: a gear has a whole number of teeth.' % (name, z))
        if int(round(z)) < zm:
            raise ValueError('%s = %d is refused: fewer than %d teeth undercut at a pressure angle of %g deg '
                             '(z_min = 2/sin^2(alpha) = %.2f, rounded up to %d).'
                             % (name, int(round(z)), zm, alpha, 2.0 / math.sin(math.radians(alpha)) ** 2, zm))
    for name, v in (('module m', m), ('face width b', b), ('shaft diameter d_s', d_s),
                    ('base plate thickness', plate_t)):
        if v <= 0:
            raise ValueError('The %s must be more than 0 mm (it is %g mm).' % (name, v))
    if backlash < 0 or backlash >= math.pi * m / 2.0:
        raise ValueError('The backlash must be at least 0 and less than pi*m/2 = %.3f mm (it is %g mm).'
                         % (math.pi * m / 2.0, backlash))
    for name, z in (('gear 1', int(round(z1))), ('gear 2', int(round(z2)))):
        g = gear_sizes(m, z, alpha, backlash)
        if g['half'](g['ra']) <= 0:
            raise ValueError('The teeth of %s come to a point below the tip circle: lower the backlash.' % name)
        if d_s / 2.0 >= g['r_in'] * math.cos(g['half'](g['r_lo'])):
            raise ValueError('A %g mm shaft does not fit inside the root circle of %s (root diameter %.2f mm).'
                             % (d_s, name, 2 * g['rf']))


# ---------- Fusion helpers (positions in mm, converted to cm) ----------
def _polyline(sk, pts):
    """One closed polyline of sketch lines that share their SketchPoints (points in cm)."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def _new_occ(root, name, x, y, z):
    mat = adsk.core.Matrix3D.create()
    mat.translation = adsk.core.Vector3D.create(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def _profile_with_loops(sk, loops):
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        if p.profileLoops.count == loops:
            return p
    raise RuntimeError('no profile with %d loops in sketch %s' % (loops, sk.name))


def _plate(occ, x0, x1, y0, y1, t):
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Plate outline'
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(x0 * CM, y0 * CM, 0), P3(x1 * CM, y1 * CM, 0))
    ext = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), VI(t * CM),
                                                  adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.bodies.item(0).name = comp.name


def _shaft(occ, d, length):
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Shaft section'
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), d / 2.0 * CM)
    ext = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), VI(length * CM),
                                                  adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.bodies.item(0).name = comp.name


def _gear(occ, m, z, alpha, backlash, b, d_s, phase):
    """The blank (root circle with the bore), one tooth as one closed polyline joined to it, patterned z times about
    the component's z axis. The gear's lower face is on the component's xy plane, its axis the component's z axis."""
    comp = occ.component
    g = gear_sizes(m, z, alpha, backlash)
    ext = comp.features.extrudeFeatures
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Blank'
    circles = sk.sketchCurves.sketchCircles
    circles.addByCenterRadius(P3(0, 0, 0), g['rf'] * CM)
    circles.addByCenterRadius(P3(0, 0, 0), d_s / 2.0 * CM)
    blank = ext.addSimple(_profile_with_loops(sk, 2), VI(b * CM),
                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = blank.bodies.item(0)
    body.name = comp.name

    tsk = comp.sketches.add(comp.xYConstructionPlane)
    tsk.name = 'Tooth'
    _polyline(tsk, [(x * CM, y * CM) for x, y in tooth_points(m, z, alpha, backlash, phase)])
    if tsk.profiles.count != 1:
        raise RuntimeError('the tooth outline of %s gave %d profiles, not one' % (comp.name, tsk.profiles.count))
    tooth = ext.addSimple(tsk.profiles.item(0), VI(b * CM), adsk.fusion.FeatureOperations.JoinFeatureOperation)

    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth)
    pats = comp.features.circularPatternFeatures
    pin = pats.createInput(feats, comp.zConstructionAxis)
    pin.quantity = VI(float(z))
    pin.totalAngle = VS('360 deg')
    pin.isSymmetric = False
    pats.add(pin)
    return body


def _axis_point(occ):
    """Joint geometry at the occurrence's origin (on the shaft axis), its axes those of the world."""
    pt = occ.component.originConstructionPoint.createForAssemblyContext(occ)
    return adsk.fusion.JointGeometry.createByPoint(pt)


def _as_built(root, one, two, geo, revolute, name):
    inp = root.asBuiltJoints.createInput(one, two, geo)
    if revolute:
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def _turn_about_z(occ):
    t = occ.transform2
    return math.atan2(t.getCell(1, 0), t.getCell(0, 0))


def _revolute(root, gear, shaft, name):
    """A revolute As-built Joint about the shaft axis whose positive value turns the gear right-handed about +z
    (so that the joint's value and the gear's real turn about z agree). Made with the gear as the moving
    occurrence; if a small test turn goes the wrong way, it is made again with the occurrences the other way round."""
    j = _as_built(root, gear, shaft, _axis_point(shaft), True, name)
    try:
        r0 = _turn_about_z(gear)
        j.jointMotion.rotationValue = 0.1
        turned = _turn_about_z(gear) - r0
        j.jointMotion.rotationValue = 0.0
    except Exception:
        return j
    if turned < -0.05:
        j.deleteMe()
        j = _as_built(root, shaft, gear, _axis_point(shaft), True, name)
    return j


# ---------- the build ----------
def build(m=DEFAULTS['m'], z1=DEFAULTS['z1'], z2=DEFAULTS['z2'], alpha=DEFAULTS['alpha'], b=DEFAULTS['b'],
          backlash=DEFAULTS['backlash'], d_s=DEFAULTS['d_s'], plate_t=DEFAULTS['plate_t']):
    """Build the gear pair in a new design (sizes in mm, the pressure angle in degrees). Returns the message the
    dialog shows; raises ValueError for inputs that cannot be built (fewer than z_min teeth undercut)."""
    check_inputs(m, z1, z2, alpha, b, backlash, d_s, plate_t)
    z1, z2 = int(round(z1)), int(round(z2))
    a = m * (z1 + z2) / 2.0                                # centre distance
    g1, g2 = gear_sizes(m, z1, alpha, backlash), gear_sizes(m, z2, alpha, backlash)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = des.rootComponent

    z_shaft = plate_t                                      # the shafts stand on the plate's top face
    z_gear = plate_t + GEAR_GAP                            # the gears' lower faces
    plate = _new_occ(root, 'Base plate', 0.0, 0.0, 0.0)
    shaft1 = _new_occ(root, 'Shaft 1', 0.0, 0.0, z_shaft)
    shaft2 = _new_occ(root, 'Shaft 2', a, 0.0, z_shaft)
    gear1 = _new_occ(root, 'Gear 1', 0.0, 0.0, z_gear)
    gear2 = _new_occ(root, 'Gear 2', a, 0.0, z_gear)

    reach = max(g1['ra'], g2['ra']) + PLATE_MARGIN
    _plate(plate, -g1['ra'] - PLATE_MARGIN, a + g2['ra'] + PLATE_MARGIN, -reach, reach, plate_t)
    shaft_len = GEAR_GAP + b + SHAFT_ABOVE
    _shaft(shaft1, d_s, shaft_len)
    _shaft(shaft2, d_s, shaft_len)
    # Gear 1 has a tooth on the line of centres (+x); gear 2 is turned half a pitch from a tooth on the line, so that
    # a gap of gear 2 faces that tooth.
    _gear(gear1, m, z1, alpha, backlash, b, d_s, 0.0)
    _gear(gear2, m, z2, alpha, backlash, b, d_s, math.pi + math.pi / z2)

    plate.isGrounded = True
    for o in (shaft1, shaft2, gear1, gear2):
        o.isGrounded = False

    _as_built(root, shaft1, plate, _axis_point(shaft1), False, 'Shaft 1 on plate')
    _as_built(root, shaft2, plate, _axis_point(shaft2), False, 'Shaft 2 on plate')
    j1 = _revolute(root, gear1, shaft1, 'Gear 1 turns')
    j2 = _revolute(root, gear2, shaft2, 'Gear 2 turns')

    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('360 deg')
    li.valueTwo = VS('%.12g deg' % (360.0 * z1 / z2))       # gear 2 turns z1/z2 of gear 1's angle ...
    li.isReversed = True                                    # ... the other way
    link = root.motionLinks.add(li)
    try:
        link.name = 'Gear 1 drives gear 2'
    except Exception:
        pass

    zm = z_min(alpha)
    return '\n'.join([
        'Spur gear pair on a base plate, %s' % VERSION,
        'm = %g mm, z1 = %d, z2 = %d, alpha = %g deg, b = %g mm, backlash %g mm, shaft %g mm, plate %g mm'
        % (m, z1, z2, alpha, b, backlash, d_s, plate_t),
        'Centre distance a = %.2f mm' % a,
        'Ratio z2/z1 = %d/%d = %.3f: gear 2 turns %.4g deg the other way for each degree of gear 1'
        % (z2, z1, z2 / float(z1), z1 / float(z2)),
        'Pitch radii %.3f / %.3f mm, base %.3f / %.3f, tip %.3f / %.3f, root %.3f / %.3f'
        % (g1['r'], g2['r'], g1['rb'], g2['rb'], g1['ra'], g2['ra'], g1['rf'], g2['rf']),
        'Tooth thickness on the pitch circle %.4f mm each; fewest teeth without undercut at %g deg: %d'
        % (g1['s'], alpha, zm),
        'Drive "Gear 1 turns" (Drive Joints) to turn the pair.',
    ])


# ---------- the dialog ----------
def _read(inputs):
    def v(i):
        return inputs.itemById(i).value
    return dict(m=round(v('m') / CM, 9), z1=int(v('z1')), z2=int(v('z2')), alpha=round(math.degrees(v('alpha')), 9),
                b=round(v('b') / CM, 9), backlash=round(v('backlash') / CM, 9), d_s=round(v('d_s') / CM, 9),
                plate_t=round(v('plate_t') / CM, 9))


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            msg = build(**_read(args.command.commandInputs))
            ui.messageBox(msg)
        except ValueError as e:
            ui.messageBox('Not built: %s' % e)
        except Exception:
            ui.messageBox('GearPairOnPlate failed:\n%s' % traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _own_lifetime[0]:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            ins = cmd.commandInputs
            d = DEFAULTS
            ins.addValueInput('m', 'Module m', 'mm', VS('%g mm' % d['m']))
            ins.addIntegerSpinnerCommandInput('z1', 'Driver teeth z1', 3, 1000, 1, d['z1'])
            ins.addIntegerSpinnerCommandInput('z2', 'Driven teeth z2', 3, 1000, 1, d['z2'])
            ins.addValueInput('alpha', 'Pressure angle alpha', 'deg', VS('%g deg' % d['alpha']))
            ins.addValueInput('b', 'Face width b', 'mm', VS('%g mm' % d['b']))
            ins.addValueInput('backlash', 'Backlash on the pitch circle', 'mm', VS('%g mm' % d['backlash']))
            ins.addValueInput('d_s', 'Shaft diameter d_s', 'mm', VS('%g mm' % d['d_s']))
            ins.addValueInput('plate_t', 'Base plate thickness', 'mm', VS('%g mm' % d['plate_t']))
            on_execute = _Execute()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui.messageBox('GearPairOnPlate failed:\n%s' % traceback.format_exc())


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Spur gear pair on a plate',
                                                       'A parametric spur gear pair on a base plate, with joints')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        if context is not None:            # started by Fusion: stay alive until the dialog closes
            _own_lifetime[0] = True
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('GearPairOnPlate failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
    del _handlers[:]
