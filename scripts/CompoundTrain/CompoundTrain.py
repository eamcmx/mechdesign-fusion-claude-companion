# =============================================================================
# CompoundTrain.py  -  Chapter 13 of "Mechanical Design in Autodesk Fusion with Claude": a two-stage reverted compound
# spur gear train. Input and output gears turn on one fixed spindle; a cluster gear on a countershaft carries the two
# middle gears. Written from PROMPT.txt beside it; checked by checks/compound_train_check.py.
#
#   INPUTS        the dialog: m1, m2, z1, z2, z3, b, d_s
#   DERIVED       train(): a, z4 from the reverted condition, the ratio, each stage's contact ratio
#   HARD RULES    build() refuses a z4 that is not a whole number and any gear under 18 teeth; every driven gear
#                 turned so that a tooth of its driver meets a gap
#   CONVENTIONS   a new design; parts created where they belong; tooth outlines as closed polylines; cm and radians
#                 inside the API; As-built Joints; one Motion Link per mesh
#   ACCEPTANCE    the message with the defaults; the turns of the cluster and the output; no collision; z3 = 24;
#                 two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'CompoundTrain 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
GAP = 1.0                                  # mm, the shafts' collar that lifts stage 1 off the plate
LEVEL_GAP = 2.0                            # mm between the stages: the spacer and the cluster's hub
BORE_CLEAR = 0.2                           # mm, a gear's bore is this much larger than its shaft
Z_MIN = 18                                 # fewer teeth undercut at 20 degrees
ALPHA = math.radians(20.0)
BACKLASH = 0.2                             # mm on the pitch circle
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation


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



def polyline(sk, pts):
    """One closed polyline whose lines share their sketch points (closed splines are unreliable as profiles)."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def contact_ratio(m, za, zb):
    """Chapter 12: the length of the path of contact over the base pitch."""
    rba, rbb = m * za / 2 * math.cos(ALPHA), m * zb / 2 * math.cos(ALPHA)
    raa, rab = m * za / 2 + m, m * zb / 2 + m
    a = m * (za + zb) / 2
    return (math.sqrt(raa ** 2 - rba ** 2) + math.sqrt(rab ** 2 - rbb ** 2) - a * math.sin(ALPHA)) / (math.pi * m * math.cos(ALPHA))


def train(m1, m2, z1, z2, z3):
    a = m1 * (z1 + z2) / 2.0
    z4 = 2.0 * a / m2 - z3                  # the reverted condition: the second stage on the same centre distance
    return dict(a=a, z4=z4, ratio=(z2 / z1) * (z4 / z3), eps1=contact_ratio(m1, z1, z2), eps2=contact_ratio(m2, z3, z4))


# ---------- Fusion building blocks (mm at the interface) ----------
def place(root, name, x=0.0, y=0.0):
    """A new component created where it belongs: the position goes in the creation matrix."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(x * CM, y * CM, 0)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, prof, length_mm, op=NEW, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def annulus(comp, z_mm, r_out, r_in):
    sk = comp.sketches.add(plane_at(comp, z_mm))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), r_out * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), r_in * CM)
    return [p for p in sk.profiles if p.profileLoops.count == 2][0]


def gear(comp, z0, m, z, b, phase, bore, body=None):
    """A gear from z0 to z0 + b: root disc with a bore, one tooth, a circular pattern of that tooth. With `body`, the
    gear is joined to it (the cluster); otherwise it is a new body."""
    pts, rf = tooth_outline(m * CM, z, ALPHA, BACKLASH * CM, phase)
    disc = extrude(comp, annulus(comp, z0, rf / CM, bore / 2), b, JOIN if body else NEW, body)
    body = body or disc.bodies.item(0)
    st = comp.sketches.add(plane_at(comp, z0))
    polyline(st, pts)
    tooth = extrude(comp, st.profiles.item(0), b, JOIN, body)
    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(z)
    pi.totalAngle = VS('360 deg')
    pats.add(pi)
    return body


def shaft(comp, ds, height):
    """A collar GAP high under stage 1 and the shaft standing up through it: one body."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), 1.6 * ds / 2 * CM)
    body = extrude(comp, sk.profiles.item(0), GAP).bodies.item(0)
    s2 = comp.sketches.add(plane_at(comp, GAP))
    s2.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), ds / 2 * CM)
    extrude(comp, s2.profiles.item(0), height - GAP, JOIN, body)
    body.name = comp.name
    return body


def as_built(root, moving, fixed, motion, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    if motion == 'revolute':
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def mesh_link(root, j_driver, j_driven, z_driver, z_driven):
    """While the driver turns z_driven / z_driver turns, the driven gear turns one, the other way."""
    g = math.gcd(z_driver, z_driven)
    li = root.motionLinks.createInput(j_driver, j_driven)
    li.valueOne = VS('%d deg' % (360 * z_driven // g))
    li.valueTwo = VS('%d deg' % (360 * z_driver // g))
    li.isReversed = True
    root.motionLinks.add(li).name = 'Mesh %d:%d' % (z_driver, z_driven)


def colour(design, occ, name):
    """An appearance from Fusion's library; the build does not depend on it."""
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the assembly ----------
def build(m1=1.5, m2=2.0, z1=20, z2=60, z3=20, b=10.0, ds=8.0):
    z1, z2, z3 = int(round(z1)), int(round(z2)), int(round(z3))
    t = train(m1, m2, z1, z2, z3)
    if abs(t['z4'] - round(t['z4'])) > 1e-9:
        raise ValueError('The second stage needs z3 + z4 = m1 (z1 + z2) / m2 = %.2f teeth: not a whole number. '
                         'Change m2 or the first stage.' % (2 * t['a'] / m2))
    z4 = int(round(t['z4']))
    if min(z1, z2, z3, z4) < Z_MIN:
        raise ValueError('%d teeth: below %d teeth at 20 degrees the involute is undercut.' % (min(z1, z2, z3, z4), Z_MIN))
    a = t['a']
    s1, s2 = GAP, GAP + b + LEVEL_GAP                               # the bottoms of stage 1 and stage 2
    height = s2 + b + 3.0
    bore = ds + BORE_CLEAR

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    big = max(m1 * z2 / 2 + m1, m2 * z4 / 2 + m2) + 6
    plate = place(root, 'Base plate')
    pc = plate.component
    sk = pc.sketches.add(plane_at(pc, -6.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-big * CM, -big * CM, 0), P3((a + big) * CM, big * CM, 0))
    extrude(pc, sk.profiles.item(0), 6.0).bodies.item(0).name = 'Base plate'
    plate.isGrounded = True

    spindle = place(root, 'Spindle')
    shaft(spindle.component, ds, height)
    counter = place(root, 'Countershaft', a, 0)
    shaft(counter.component, ds, height)

    g1 = place(root, 'Gear 1')
    gear(g1.component, s1, m1, z1, b, 0.0, bore).name = 'Gear 1'                     # tooth 0 along +x
    cluster = place(root, 'Cluster', a, 0)
    c = cluster.component
    body = gear(c, s1, m1, z2, b, mesh_phase(z1, z2), bore)                         # a gap meets gear 1's tooth 0
    extrude(c, annulus(c, s1 + b, 1.6 * ds / 2, bore / 2), LEVEL_GAP, JOIN, body)     # the hub between the stages
    gear(c, s2, m2, z3, b, math.pi, bore, body)                                     # tooth 0 along -x, at gear 4
    body.name = 'Cluster'
    spacer = place(root, 'Spacer')
    extrude(spacer.component, annulus(spacer.component, s1 + b, 1.6 * ds / 2, ds / 2), LEVEL_GAP).bodies.item(0).name = 'Spacer'
    g4 = place(root, 'Gear 4')
    gear(g4.component, s2, m2, z4, b, math.pi / z4, bore).name = 'Gear 4'            # a gap meets the cluster's tooth

    as_built(root, spindle, plate, 'rigid', 'Spindle on plate')
    as_built(root, counter, plate, 'rigid', 'Countershaft on plate')
    as_built(root, spacer, spindle, 'rigid', 'Spacer on spindle')
    j1 = as_built(root, g1, spindle, 'revolute', 'Gear 1 turns')
    j2 = as_built(root, cluster, counter, 'revolute', 'Cluster turns')
    j4 = as_built(root, g4, spindle, 'revolute', 'Gear 4 turns')
    mesh_link(root, j1, j2, z1, z2)
    mesh_link(root, j2, j4, z3, z4)
    colour(design, plate, 'Paint - Enamel Glossy (Grey)')
    colour(design, g1, 'Paint - Enamel Glossy (Blue)')
    colour(design, cluster, 'Paint - Enamel Glossy (Yellow)')
    colour(design, g4, 'Paint - Enamel Glossy (Red)')
    app.activeViewport.fit()

    return ('Compound train built: two stages on a centre distance a = m1 (z1 + z2) / 2 = %.2f mm.\n\n'
            'Stage 1: module %.2f, %d : %d teeth, contact ratio %.3f\n'
            'Stage 2: module %.2f, %d : %d teeth (z4 = 2 a / m2 - z3), contact ratio %.3f\n'
            'Ratio (z2 / z1) (z4 / z3) = %.3f, the output turning the same way as the input\n\n'
            'Drive Joints on "Gear 1 turns": 90 deg turns the cluster %.1f deg and gear 4 %.1f deg.\n\n[%s]'
            % (a, m1, z1, z2, t['eps1'], m2, z3, z4, t['eps2'], t['ratio'], -90.0 * z1 / z2, 90.0 / t['ratio'], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('m1', 'Module, stage 1', 1.5, 'mm'), ('m2', 'Module, stage 2', 2.0, 'mm'), ('z1', 'Input gear teeth z1', 20, 'n'),
          ('z2', 'Cluster teeth z2 (stage 1)', 60, 'n'), ('z3', 'Cluster teeth z3 (stage 2)', 20, 'n'),
          ('b', 'Face width', 10.0, 'mm'), ('ds', 'Shaft diameter', 8.0, 'mm')]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = {}
            for fid, _label, _d, unit in FIELDS:
                v = ins.itemById(fid).value
                p[fid] = v / CM if unit == 'mm' else v
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for fid, label, d, unit in FIELDS:
            if unit == 'n':
                ins.addIntegerSpinnerCommandInput(fid, label, 6, 400, 1, d)
            else:
                ins.addValueInput(fid, label, 'mm', VI(d * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'CompoundTrainCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Compound train', 'A two-stage reverted gear train')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Compound train could not start:\n' + traceback.format_exc())
