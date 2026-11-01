# =============================================================================
# GearPairOnPlate.py  -  what the five-part prompt of Topic 5 asks for: a parametric spur gear pair on a base
# plate, assembled and ready to turn with Drive Joints.
#
# The prompt, part by part, and where this script answers it:
#   1. Goal            a new design with a plate, two shafts and two gears, jointed and linked      build()
#   2. Inputs          m, z1, z2, pressure angle, face width, backlash, shaft diameter, plate        FIELDS
#   3. Hard rules      involute flanks from the base circle (radial line below it), z >= 18,
#                      the driven gear turned so a tooth meets a gap, ratio z2 / z1             tooth_outline, mesh_phase
#   4. Conventions     FusionDesignDocumentType; one body per component; each tooth outline one closed polyline
#                      with shared sketch points; cm inside the API; parts placed where they belong WHEN they are
#                      created; As-built Joints; a Motion Link between the revolute joints
#   5. Acceptance      message with the centre distance, the ratio and the version; gear 1 driven 90 degrees turns
#                      gear 2 by 45 degrees the other way; no interference at rest; z2 = 50 gives a = 70 mm
#
# Tested in Fusion: workshop/fusion_toolkit/_dev/cases/gearpair_check.py
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'GearPairOnPlate 1.1 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
GAP = 1.0                                  # mm, the collar that lifts each gear off the plate
BORE_CLEAR = 0.2                           # mm, the gear's bore is this much larger than the shaft
Z_MIN = 18                                 # fewer teeth undercut at 20 degrees (Topic 4.1)
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

# (id, label, default, unit): 'mm' and 'deg' become value inputs, 'n' an integer
FIELDS = [('m', 'Module m', 2.0, 'mm'), ('z1', 'Driver teeth z1', 20, 'n'), ('z2', 'Driven teeth z2', 40, 'n'),
          ('alpha', 'Pressure angle', 20.0, 'deg'), ('b', 'Face width b', 10.0, 'mm'),
          ('backlash', 'Backlash on the pitch circle', 0.2, 'mm'), ('ds', 'Shaft diameter', 8.0, 'mm'),
          ('plate', 'Base plate thickness', 6.0, 'mm')]


# ---------- the tooth: involute flanks from the base circle (all in cm and radians) ----------
def inv(a):
    return math.tan(a) - a


def involute_pt(rb, t):
    return (rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t)))


def rotate(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def tooth_outline(m, z, alpha, backlash, phase, n=8):
    """One tooth, centred on angle `phase`, as a closed list of points: up the left flank, over the tip, down the
    right flank, closed just below the root circle so it fuses with the root disc. Where the base circle is above
    the root circle the flank starts with a radial line (there is no involute inside the base circle)."""
    r = m * z / 2.0                         # pitch radius
    rb = r * math.cos(alpha)                # base radius
    ra = r + m                              # tip radius
    rf = r - 1.25 * m                       # root radius
    r_lo = max(rb, rf)
    t_at = lambda rr: 0.0 if rr <= rb else math.sqrt((rr / rb) ** 2 - 1.0)
    half = (math.pi * m / 2.0 - backlash / 2.0) / (2.0 * r)       # half the tooth thickness, as an angle at r
    rot = half + inv(alpha)                  # where the involute leaves the base circle, from the tooth centre
    right, left = [], []
    for i in range(n + 1):
        t = t_at(r_lo) + (t_at(ra) - t_at(r_lo)) * i / n
        x, y = involute_pt(rb, t)
        right.append(rotate([(x, -y)], rot)[0])                  # mirrored, then turned by +rot
        left.append(rotate([(x, y)], -rot)[0])                   # turned by -rot
    a_l, a_r = math.atan2(left[0][1], left[0][0]), math.atan2(right[0][1], right[0][0])
    r_in = rf * 0.985
    pts = [(r_in * math.cos(a_r), r_in * math.sin(a_r)), (r_in * math.cos(a_l), r_in * math.sin(a_l))]
    if r_lo > r_in + 1e-9:
        pts.append((r_lo * math.cos(a_l), r_lo * math.sin(a_l)))   # the radial line up to where the flank starts
    pts.extend(left[1:] if r_lo <= r_in + 1e-9 else left)
    a0, a1 = math.atan2(left[-1][1], left[-1][0]), math.atan2(right[-1][1], right[-1][0])
    pts.extend((ra * math.cos(a0 + (a1 - a0) * j / 5), ra * math.sin(a0 + (a1 - a0) * j / 5)) for j in range(1, 5))
    pts.extend(reversed(right))
    clean = [pts[0]]                         # no zero-length lines: the radial line ends where the flank starts
    for q in pts[1:]:
        if math.hypot(q[0] - clean[-1][0], q[1] - clean[-1][1]) > 1e-7:
            clean.append(q)
    return rotate(clean, phase), rf


def mesh_phase(z1, z2):
    """Turn of the driven gear (centre on +x of the driver) so that the driver's tooth 0, pointing along +x, meets
    a GAP of the driven gear: the driven gear's tooth 0 then points along -x shifted by half a tooth."""
    return math.pi - math.pi / z2


# ---------- Fusion building blocks (mm at the interface) ----------
def place(root, name, x, y, z=0.0):
    """A new component, created where it belongs. Moving an occurrence AFTER it is created is undone by the next
    feature in a parametric design, so the position goes into the creation matrix."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, prof, length_mm, op=adsk.fusion.FeatureOperations.NewBodyFeatureOperation, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def polyline(sk, pts):
    """One closed polyline whose lines share their sketch points (closed splines are unreliable as profiles)."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def gear_body(comp, m, z, alpha_deg, b, backlash, phase, bore, name):
    """Root disc with a bore, one tooth joined to it, then a circular pattern of that tooth: one body."""
    pts, rf = tooth_outline(m * CM, z, math.radians(alpha_deg), backlash * CM, phase)
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), rf)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), bore * CM / 2)
    disc = extrude(comp, max(list(sk.profiles), key=lambda p: p.areaProperties().area), b)
    body = disc.bodies.item(0)
    body.name = name
    st = comp.sketches.add(comp.xYConstructionPlane)
    polyline(st, pts)
    tooth = extrude(comp, st.profiles.item(0), b, adsk.fusion.FeatureOperations.JoinFeatureOperation, body)
    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(z)
    pi.totalAngle = VS('360 deg')
    pats.add(pi)
    return body


def shaft_body(comp, ds, height):
    """A collar GAP high that the gear sits on, and the shaft standing up through it: one body."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), 1.6 * ds * CM / 2)
    body = extrude(comp, sk.profiles.item(0), GAP).bodies.item(0)
    s2 = comp.sketches.add(plane_at(comp, GAP))
    s2.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), ds * CM / 2)
    extrude(comp, s2.profiles.item(0), height - GAP, adsk.fusion.FeatureOperations.JoinFeatureOperation, body)
    body.name = comp.name
    return body


def colour(design, occ, name):
    """An appearance from Fusion's library, as in the RDM toolkit; the build does not depend on it."""
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


def origin_point(occ):
    """The component's origin as a joint point, in the assembly's context."""
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(occ)


def as_built(root, moving, fixed, motion, name):
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(origin_point(moving)))
    if motion == 'revolute':
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


# ---------- the assembly ----------
def build(m=2.0, z1=20, z2=40, alpha=20.0, b=10.0, backlash=0.2, ds=8.0, plate=6.0):
    z1, z2 = int(round(z1)), int(round(z2))
    if min(z1, z2) < Z_MIN:
        raise ValueError('%d teeth: below %d teeth at 20 degrees the involute is undercut. Choose %d or more.'
                         % (min(z1, z2), Z_MIN, Z_MIN))
    a = m * (z1 + z2) / 2.0                                       # centre distance, mm
    bore = ds + BORE_CLEAR
    if bore >= 2 * (m * min(z1, z2) / 2.0 - 1.25 * m) - 2 * m:
        raise ValueError('A %.1f mm shaft leaves too little metal under the teeth of a %d-tooth gear.' % (ds, min(z1, z2)))

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    # the base plate: top face at z = 0, big enough for both gears plus a margin
    ra2 = m * z2 / 2.0 + m
    margin = 2 * m + 6
    plate_occ = place(root, 'Base plate', 0, 0, 0)
    pc = plate_occ.component
    sk = pc.sketches.add(plane_at(pc, -plate))
    r1 = m * z1 / 2.0 + m
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3((-r1 - margin) * CM, (-ra2 - margin) * CM, 0),
                                                     P3((a + ra2 + margin) * CM, (ra2 + margin) * CM, 0))
    extrude(pc, sk.profiles.item(0), plate).bodies.item(0).name = 'Base plate'
    plate_occ.isGrounded = True

    # the shafts stand on the plate; each gear sits on its shaft's collar, GAP above the plate
    height = GAP + b + 3.0
    shaft1 = place(root, 'Shaft 1', 0, 0, 0)
    shaft_body(shaft1.component, ds, height)
    shaft2 = place(root, 'Shaft 2', a, 0, 0)
    shaft_body(shaft2.component, ds, height)
    gear1 = place(root, 'Gear 1', 0, 0, GAP)
    gear_body(gear1.component, m, z1, alpha, b, backlash, 0.0, bore, 'Gear 1')
    gear2 = place(root, 'Gear 2', a, 0, GAP)
    gear_body(gear2.component, m, z2, alpha, b, backlash, mesh_phase(z1, z2), bore, 'Gear 2')

    colour(design, plate_occ, 'Paint - Enamel Glossy (Grey)')
    colour(design, gear1, 'Paint - Enamel Glossy (Blue)')
    colour(design, gear2, 'Paint - Enamel Glossy (Yellow)')

    # joints: shafts rigid to the plate, each gear revolute on its shaft; the Motion Link is the mesh
    as_built(root, shaft1, plate_occ, 'rigid', 'Shaft 1 on plate')
    as_built(root, shaft2, plate_occ, 'rigid', 'Shaft 2 on plate')
    j1 = as_built(root, gear1, shaft1, 'revolute', 'Gear 1 turns')
    j2 = as_built(root, gear2, shaft2, 'revolute', 'Gear 2 turns')
    g = math.gcd(z1, z2)
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % (360 * z2 // g))                  # z2/z1 turns of gear 1 ...
    li.valueTwo = VS('%d deg' % (360 * z1 // g))                  # ... for one turn of gear 2
    li.isReversed = True                                          # two outside gears turn opposite ways
    root.motionLinks.add(li).name = 'Mesh %d:%d' % (z1, z2)
    app.activeViewport.fit()

    return ('Gear pair built.\n\n'
            'Centre distance a = m · (z1 + z2) / 2 = %.2f · (%d + %d) / 2 = %.2f mm\n'
            'Ratio z2 / z1 = %d / %d = %.3f, gear 2 turning the other way\n\n'
            'Drive Joints on "Gear 1 turns": 90 deg turns gear 2 by %.1f deg the other way.\n\n[%s]'
            % (m, z1, z2, a, z2, z1, z2 / z1, 90.0 * z1 / z2, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            p = {}
            for key, _, _, unit in FIELDS:
                v = inputs.itemById(key).value
                p[key] = v / CM if unit == 'mm' else (math.degrees(v) if unit == 'deg' else v)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        cmd.setDialogInitialSize(420, 330)
        inputs = cmd.commandInputs
        for key, label, d, unit in FIELDS:
            if unit == 'n':
                inputs.addIntegerSpinnerCommandInput(key, label, 6, 300, 1, int(d))
            elif unit == 'deg':
                inputs.addValueInput(key, label, 'deg', VS('%g deg' % d))
            else:
                inputs.addValueInput(key, label, 'mm', VI(d * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'GearPairOnPlateCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Gear pair on a plate', 'A spur gear pair, jointed and linked')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Gear pair could not start:\n' + traceback.format_exc())
