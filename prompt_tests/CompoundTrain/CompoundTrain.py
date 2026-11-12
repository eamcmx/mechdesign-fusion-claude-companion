# =============================================================================
# CompoundTrain.py  -  a two-stage reverted compound spur gear train on a base plate. The input gear (Gear 1) and the
# output gear (Gear 4) turn on one fixed spindle at the origin; the cluster on a countershaft at (a, 0) carries the
# two middle gears. The train is put together with As-built Joints and two Motion Links, ready to turn with Drive
# Joints.
#
#   Stage 1 (module m1): Gear 1 (z1) drives the cluster's gear z2,  z = 1 ... 1 + b above the plate
#   Stage 2 (module m2): the cluster's gear z3 drives Gear 4 (z4),  z = 3 + b ... 3 + 2b
#   a = m1 (z1 + z2) / 2,  z4 = 2 a / m2 - z3 (same centre distance: input and output on one axis)
#   ratio = (z2 / z1) (z4 / z3), the output turning the same way as the input (two external meshes)
#
# Sizes in millimetres; the Fusion API works in centimetres and radians (CM = 0.1 turns one into the other).
# =============================================================================
import importlib
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    """The first folder above this script that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('common/companion_paths.py not found above %s' % __file__)
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP        # puts every lib/ folder on sys.path
import mdfc_parts as MP             # the parts library: bodies, components, the involute tooth outline
importlib.reload(MP)                # Fusion keeps modules loaded; pick up edits to the library

VERSION = 'CompoundTrain v1.0 (prompt test, 2026-11-11)'

CM = 0.1                    # mm to cm
ALPHA_DEG = 20.0            # pressure angle
MIN_TEETH = 18              # fewer undercut at 20 degrees
BORE_CLEAR = 0.2            # every gear bore is d_s + 0.2 mm
BACKLASH = 0.1              # mm of circular backlash per mesh (each gear thinned by half of it at the pitch circle)
PLATE_T = 10.0              # base plate thickness, mm
PLATE_MARGIN = 10.0         # plate edge beyond the largest tip circle, mm
SHAFT_EXTRA = 3.0           # shafts stand 3 mm above stage 2

DEFAULTS = dict(m1=1.5, m2=2.0, z1=20, z2=60, z3=20, b=10.0, d_s=8.0)


# ---------- the train's numbers ----------
def contact_ratio(m, za, zb, alpha_deg=ALPHA_DEG):
    """Length of action over the base pitch for two standard external gears (addendum m) at standard centres."""
    al = math.radians(alpha_deg)
    ra, rb = m * za / 2.0, m * zb / 2.0                     # pitch radii
    la = math.sqrt((ra + m) ** 2 - (ra * math.cos(al)) ** 2)
    lb = math.sqrt((rb + m) ** 2 - (rb * math.cos(al)) ** 2)
    return (la + lb - (ra + rb) * math.sin(al)) / (math.pi * m * math.cos(al))


def _whole(v, name):
    if abs(v - round(v)) > 1e-9:
        raise ValueError('%s = %g is not a whole number of teeth' % (name, v))
    return int(round(v))


def train(m1, m2, z1, z2, z3):
    """The derived numbers of the train, or ValueError when a hard constraint is broken."""
    if m1 <= 0 or m2 <= 0:
        raise ValueError('the modules must be positive (m1 = %g, m2 = %g)' % (m1, m2))
    z1, z2, z3 = _whole(z1, 'z1'), _whole(z2, 'z2'), _whole(z3, 'z3')
    a = m1 * (z1 + z2) / 2.0
    z34 = m1 * (z1 + z2) / m2                                # z3 + z4 = 2 a / m2
    if abs(z34 - round(z34)) > 1e-9:
        raise ValueError('z3 + z4 = m1 (z1 + z2) / m2 = %.2f teeth, not a whole number, so stage 2 cannot have the '
                         'centre distance a = %.2f mm of stage 1' % (z34, a))
    z4 = int(round(z34)) - z3
    few = ['%s = %d' % (n, z) for n, z in (('z1', z1), ('z2', z2), ('z3', z3), ('z4', z4)) if z < MIN_TEETH]
    if few:
        raise ValueError('%s teeth: every gear needs at least %d teeth, fewer are undercut at %g degrees'
                         % (', '.join(few), MIN_TEETH, ALPHA_DEG))
    i1, i2 = z2 / float(z1), z4 / float(z3)
    return dict(a=a, z1=z1, z2=z2, z3=z3, z4=z4, z34=z34, i1=i1, i2=i2, ratio=i1 * i2,
                eps1=contact_ratio(m1, z1, z2), eps2=contact_ratio(m2, z3, z4))


def _radii(m, z):
    """Pitch, tip and root radii of a standard gear (addendum m, dedendum 1.25 m)."""
    r = m * z / 2.0
    return r, r + m, r - 1.25 * m


def _layout(m1, m2, b, d_s, t):
    """Levels, radii and the checks of fit that the numbers alone do not make."""
    if b <= 0 or d_s <= 0:
        raise ValueError('the face width and the shaft diameter must be positive (b = %g, d_s = %g)' % (b, d_s))
    L = dict(s1=(1.0, 1.0 + b), s2=(3.0 + b, 3.0 + 2.0 * b), H=3.0 + 2.0 * b + SHAFT_EXTRA,
             bore=d_s + BORE_CLEAR, hub=1.6 * d_s)
    L['g1'], L['g2'] = _radii(m1, t['z1']), _radii(m1, t['z2'])
    L['g3'], L['g4'] = _radii(m2, t['z3']), _radii(m2, t['z4'])
    for key, z in (('g1', t['z1']), ('g2', t['z2']), ('g3', t['z3']), ('g4', t['z4'])):
        if L['bore'] / 2.0 > L[key][2] - 1.0:
            raise ValueError('a bore of %.1f mm leaves less than 1 mm under the teeth of the %d-tooth gear'
                             % (L['bore'], z))
    a = t['a']
    # a gear on one shaft must clear the other shaft
    for key, z in (('g1', t['z1']), ('g4', t['z4']), ('g2', t['z2']), ('g3', t['z3'])):
        if a - L[key][1] < d_s / 2.0 + 0.5:
            raise ValueError('the %d-tooth gear reaches the other shaft: a - r_tip = %.2f mm for a shaft of %g mm'
                             % (z, a - L[key][1], d_s))
    if a < L['hub'] + 0.5:
        raise ValueError('the spacer and the cluster hub (1.6 d_s = %.1f mm) would touch at a = %.2f mm'
                         % (L['hub'], a))
    return L


# ---------- geometry ----------
def _teeth(occ, body, m, z, phase, z0, z1, name):
    """The teeth of a spur gear about the component's own z axis, from z0 to z1 (mm, component frame), joined to
    `body`: one tooth outline as one closed polyline of sketch lines sharing their points, extruded, then a
    circular pattern of z. `phase` is the angle of one tooth's centre line from +x towards +y."""
    comp = occ.component
    pts, rf = MP.tooth_outline(m, z, math.radians(ALPHA_DEG), BACKLASH, phase, 8)
    if abs(z0) < 1e-9:
        plane = comp.xYConstructionPlane
    else:
        ci = comp.constructionPlanes.createInput()
        ci.setByOffset(comp.xYConstructionPlane, MP.VI(z0 * CM))
        plane = comp.constructionPlanes.add(ci)
    sk = comp.sketches.add(plane)
    sk.name = name
    lines = sk.sketchCurves.sketchLines
    q = [sk.modelToSketchSpace(MP.P3(px * CM, py * CM, z0 * CM)) for px, py in pts]
    first = lines.addByTwoPoints(q[0], q[1])
    prev = first.endSketchPoint
    for p in q[2:]:
        prev = lines.addByTwoPoints(prev, p).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    if sk.profiles.count != 1:
        raise RuntimeError('%s: the tooth outline gave %d profiles, not one' % (name, sk.profiles.count))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    normal = sk.xDirection.crossProduct(sk.yDirection)
    inp.setDistanceExtent(False, MP.VI((z1 - z0) * CM * (1 if normal.z > 0 else -1)))
    inp.participantBodies = [body]
    tooth = ext.add(inp)
    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth)
    pats = comp.features.circularPatternFeatures
    pin = pats.createInput(feats, comp.zConstructionAxis)
    pin.quantity = MP.VI(z)
    pin.totalAngle = MP.VS('360 deg')
    pats.add(pin)
    return rf


def _gear(root, name, x, m, z, phase, lv, L):
    occ = MP.new_occ(root, name, (x, 0.0, 0.0))
    r, ra, rf = _radii(m, z)
    body = MP.add_body(occ, MP.tube((x, 0.0, lv[0]), (x, 0.0, lv[1]), rf, L['bore'] / 2.0), name)
    _teeth(occ, body, m, z, phase, lv[0], lv[1], '%s teeth' % name)
    return occ


# ---------- joints ----------
def _turn_z(occ):
    t = occ.transform2
    return math.atan2(t.getCell(1, 0), t.getCell(0, 0))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _rigid(root, one, two, name):
    inp = root.asBuiltJoints.createInput(one, two, None)
    inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def _revolute_once(root, one, two, moving, name):
    pt = moving.component.originConstructionPoint.createForAssemblyContext(moving)
    geo = adsk.fusion.JointGeometry.createByPoint(pt)
    inp = root.asBuiltJoints.createInput(one, two, geo)
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def _revolute(root, moving, shaft, name):
    """A revolute As-built Joint about z through the moving part's origin, made so that a positive joint angle
    turns the moving part right-handed about +z (checked by driving it a little and reading the body back; a
    joint that gives no reading is kept as made)."""
    j = _revolute_once(root, moving, shaft, moving, name)
    d = 0.0
    try:
        r0 = _turn_z(moving)
        j.jointMotion.rotationValue = 0.25
        d = _wrap(_turn_z(moving) - r0)
        j.jointMotion.rotationValue = 0.0
    except Exception:
        d = 0.0
    if d < -0.1:                                        # the joint turns the part the other way: swap its sides
        j.deleteMe()
        j = _revolute_once(root, shaft, moving, moving, name)
    return j


_REV = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType


def _link(root, driver, driven, deg_driver, deg_driven, name):
    """A Motion Link: when `driver` turns deg_driver, `driven` turns deg_driven the opposite way (an external
    mesh, the angles in the ratio of the teeth)."""
    inp = root.motionLinks.createInput(driver, driven)
    for attr in ('motionOne', 'motionTwo'):
        try:
            setattr(inp, attr, _REV)
        except Exception:
            pass
    inp.valueOne = MP.VS('%r deg' % float(deg_driver))
    inp.valueTwo = MP.VS('%r deg' % float(deg_driven))
    inp.isReversed = True
    ml = root.motionLinks.add(inp)
    try:
        ml.name = name
    except Exception:
        pass
    return dict(link=ml, one=float(deg_driver), two=float(deg_driven))


def _set_sense(j1, j_a, j_b, k):
    """Drive Gear 1 a little and compare the turns of the link's own two joints, j_a (its driver) and j_b: when j_b
    turns the wrong way, or at the inverse of the ratio, the link is set again the right way. A link that gives no
    reading at once is left as it was."""
    try:
        j1.jointMotion.rotationValue = 0.3
        va, vb = j_a.jointMotion.rotationValue, j_b.jointMotion.rotationValue
        j1.jointMotion.rotationValue = 0.0
    except Exception:
        return
    if abs(va) < 1e-6 or abs(vb) < 1e-6:
        return
    r, want = vb / va, -k['two'] / k['one']
    ml = k['link']
    near = lambda x, y: abs(x - y) < 0.02 * abs(y)
    try:
        if near(r, want):
            return
        if near(r, -want):                                  # the right ratio, the wrong way
            ml.isReversed = not ml.isReversed
        elif near(r, 1.0 / want) or near(r, -1.0 / want):  # the two values read the other way round
            rev = ml.isReversed if near(r, 1.0 / want) else not ml.isReversed
            ml.setMotionData(_REV, MP.VS('%r deg' % k['two']), _REV, MP.VS('%r deg' % k['one']), rev)
    except Exception:
        pass
    try:
        j1.jointMotion.rotationValue = 0.0
    except Exception:
        pass


# ---------- the build ----------
def build(m1=DEFAULTS['m1'], m2=DEFAULTS['m2'], z1=DEFAULTS['z1'], z2=DEFAULTS['z2'], z3=DEFAULTS['z3'],
          b=DEFAULTS['b'], d_s=DEFAULTS['d_s']):
    """Builds the train in a new design and returns the message; ValueError (nothing built) when refused."""
    t = train(m1, m2, z1, z2, z3)
    L = _layout(m1, m2, b, d_s, t)
    a, z1, z2, z3, z4 = t['a'], t['z1'], t['z2'], t['z3'], t['z4']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = design.rootComponent

    # base plate, top face at z = 0, grounded
    R0 = max(L['g1'][1], L['g4'][1])                    # tips on the spindle
    Ra = max(L['g2'][1], L['g3'][1])                    # tips on the countershaft
    ym = max(R0, Ra) + PLATE_MARGIN
    plate = MP.new_occ(root, 'Base plate')
    MP.add_body(plate, MP.box(-R0 - PLATE_MARGIN, a + Ra + PLATE_MARGIN, -ym, ym, -PLATE_T, 0.0), 'Base plate')
    plate.isGrounded = True

    # the two shafts standing on the plate, 3 mm taller than stage 2
    spindle = MP.new_occ(root, 'Spindle')
    MP.add_body(spindle, MP.cyl((0.0, 0.0, 0.0), (0.0, 0.0, L['H']), d_s / 2.0), 'Spindle')
    counter = MP.new_occ(root, 'Countershaft', (a, 0.0, 0.0))
    MP.add_body(counter, MP.cyl((a, 0.0, 0.0), (a, 0.0, L['H']), d_s / 2.0), 'Countershaft')

    s1, s2 = L['s1'], L['s2']
    # Gear 1, the input: one tooth on the line of centres, pointing at the countershaft (+x)
    g1 = _gear(root, 'Gear 1', 0.0, m1, z1, 0.0, s1, L)

    # Cluster: gear z2 at stage 1 (a gap facing the spindle, to take Gear 1's tooth), the hub between the stages,
    # gear z3 at stage 2 (a tooth facing the spindle, the driver of Gear 4); one body
    cl = MP.new_occ(root, 'Cluster', (a, 0.0, 0.0))
    blank = MP.union(MP.cyl((a, 0.0, s1[0]), (a, 0.0, s1[1]), L['g2'][2]),
                     MP.cyl((a, 0.0, s1[1]), (a, 0.0, s2[0]), L['hub'] / 2.0),
                     MP.cyl((a, 0.0, s2[0]), (a, 0.0, s2[1]), L['g3'][2]))
    MP.cut(blank, MP.cyl((a, 0.0, s1[0] - 1.0), (a, 0.0, s2[1] + 1.0), L['bore'] / 2.0))
    body = MP.add_body(cl, blank, 'Cluster')
    _teeth(cl, body, m1, z2, math.pi + math.pi / z2, s1[0], s1[1], 'Cluster z2 teeth')
    _teeth(cl, cl.component.bRepBodies.item(0), m2, z3, math.pi, s2[0], s2[1], 'Cluster z3 teeth')

    # Spacer on the spindle, filling the gap between the stages
    spacer = MP.new_occ(root, 'Spacer')
    MP.add_body(spacer, MP.tube((0.0, 0.0, s1[1]), (0.0, 0.0, s2[0]), L['hub'] / 2.0, L['bore'] / 2.0), 'Spacer')

    # Gear 4, the output, at stage 2 on the spindle: a gap facing the countershaft (+x), to take z3's tooth
    g4 = _gear(root, 'Gear 4', 0.0, m2, z4, math.pi / z4, s2, L)

    # As-built Joints only
    _rigid(root, spindle, plate, 'Spindle fixed')
    _rigid(root, counter, plate, 'Countershaft fixed')
    _rigid(root, spacer, spindle, 'Spacer fixed')
    j1 = _revolute(root, g1, spindle, 'Gear 1 turns')
    j2 = _revolute(root, cl, counter, 'Cluster turns')
    j4 = _revolute(root, g4, spindle, 'Gear 4 turns')

    # Motion Links, one per mesh, each the opposite way at the ratio of its teeth:
    # Gear 1 turning z2 degrees turns the cluster z1 degrees back; the cluster turning z4 turns Gear 4 z3 back
    k1 = _link(root, j1, j2, z2, z1, 'Mesh 1 (z1 : z2)')
    k2 = _link(root, j2, j4, z4, z3, 'Mesh 2 (z3 : z4)')
    _set_sense(j1, j1, j2, k1)
    _set_sense(j1, j2, j4, k2)
    for j in (j1, j2, j4):
        j.jointMotion.rotationValue = 0.0

    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return _message(m1, m2, b, d_s, t)


def _message(m1, m2, b, d_s, t):
    return ('%s\n'
            'Reverted compound spur train, input and output on one spindle\n'
            'Centre distance a = m1 (z1 + z2) / 2 = %.2f mm\n'
            'Stage 1: m1 = %g mm, %d : %d teeth, contact ratio %.3f\n'
            'Stage 2: m2 = %g mm, %d : %d teeth (z4 = 2 a / m2 - z3 = %d), contact ratio %.3f\n'
            'Ratio (z2 / z1) (z4 / z3) = %.3f x %.3f = %.3f, the output turning the same way as the input\n'
            'Face width %g mm, shafts %g mm, gear bores %g mm'
            % (VERSION, t['a'], m1, t['z1'], t['z2'], t['eps1'], m2, t['z3'], t['z4'], t['z4'], t['eps2'],
               t['i1'], t['i2'], t['ratio'], b, d_s, d_s + BORE_CLEAR))


# ---------- the dialog ----------
CMD_ID = 'mdfc_CompoundTrain'
_handlers = []
_state = dict(terminate=False)
_MM = ('m1', 'm2', 'b', 'd_s')
_TEETH = ('z1', 'z2', 'z3')


def _read(ins):
    v = {}
    for k in _MM:
        v[k] = round(ins.itemById(k).value / CM, 6)          # the inputs hold cm
    for k in _TEETH:
        v[k] = int(ins.itemById(k).value)
    return v


def _derived_text(v):
    try:
        t = train(v['m1'], v['m2'], v['z1'], v['z2'], v['z3'])
        _layout(v['m1'], v['m2'], v['b'], v['d_s'], t)
        return ('a = %.2f mm, z4 = %d<br>ratio %.3f (%.3f x %.3f)<br>contact ratios %.3f and %.3f'
                % (t['a'], t['z4'], t['ratio'], t['i1'], t['i2'], t['eps1'], t['eps2']))
    except ValueError as e:
        return 'Refused: %s' % e


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addValueInput('m1', 'Module of stage 1, m1', 'mm', MP.VI(DEFAULTS['m1'] * CM))
            ins.addValueInput('m2', 'Module of stage 2, m2', 'mm', MP.VI(DEFAULTS['m2'] * CM))
            ins.addIntegerSpinnerCommandInput('z1', 'Input gear teeth, z1', 1, 500, 1, DEFAULTS['z1'])
            ins.addIntegerSpinnerCommandInput('z2', 'First cluster gear teeth, z2', 1, 500, 1, DEFAULTS['z2'])
            ins.addIntegerSpinnerCommandInput('z3', 'Second cluster gear teeth, z3', 1, 500, 1, DEFAULTS['z3'])
            ins.addValueInput('b', 'Face width, b', 'mm', MP.VI(DEFAULTS['b'] * CM))
            ins.addValueInput('d_s', 'Shaft diameter, d_s', 'mm', MP.VI(DEFAULTS['d_s'] * CM))
            ins.addTextBoxCommandInput('derived', 'Derived', _derived_text(dict(DEFAULTS)), 4, True)
            h_exe, h_chg, h_end = _Execute(), _Changed(), _Destroy()
            cmd.execute.add(h_exe)
            cmd.inputChanged.add(h_chg)
            cmd.destroy.add(h_end)
            _handlers.extend([h_exe, h_chg, h_end])
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('CompoundTrain: the dialog failed\n' + traceback.format_exc())


class _Changed(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.inputs
            box = ins.itemById('derived')
            if box is not None and args.input.id != 'derived':
                box.formattedText = _derived_text(_read(ins))
        except Exception:
            pass


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            msg = build(**_read(args.command.commandInputs))
            ui.messageBox(msg)
        except ValueError as e:
            ui.messageBox('%s\nRefused: %s' % (VERSION, e))
        except Exception:
            ui.messageBox('%s\nFailed:\n%s' % (VERSION, traceback.format_exc()))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _state['terminate']:
            adsk.terminate()


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Compound train',
                                                       'A two-stage reverted compound spur gear train on a base plate')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if context is not None:                 # run by Fusion as a script: stay alive until the dialog closes
            _state['terminate'] = True
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('CompoundTrain failed:\n' + traceback.format_exc())


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
