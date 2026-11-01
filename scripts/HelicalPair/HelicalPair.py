# =============================================================================
# HelicalPair.py  -  Chapter 13 of "Mechanical Design in Autodesk Fusion with Claude": a pair of helical (or
# herringbone) gears on a base plate, assembled and ready to turn with Drive Joints. Each tooth is the transverse
# involute outline swept along the gear's axis with a twist. Written from PROMPT.txt beside it; checked by
# checks/helical_pair_check.py.
#
#   INPUTS        the dialog: m_n, z1, z2, beta, alpha_n, b, herringbone
#   DERIVED       helical(): m_t, alpha_t, a, the twist of each gear, z_min, the transverse and overlap ratios
#   HARD RULES    opposite hands; involute flanks in the transverse plane; z >= z_min; 5 <= beta <= 45; the
#                 driven gear turned so that a tooth meets a gap; herringbone halves of opposite hand
#   CONVENTIONS   a new design; tooth outlines as closed polylines swept with a twist; cm and radians inside the
#                 API; As-built Joints; a Motion Link for the mesh
#   ACCEPTANCE    the message; the twist measured on the teeth's tips; no collision; herringbone; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'HelicalPair 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
GAP = 1.0                                  # mm, the collar that lifts each gear off the plate
BORE_CLEAR = 0.2                           # mm
BACKLASH = 0.2                             # mm on the pitch circle, transverse
DS = 8.0                                   # mm, shaft diameter
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation


def helical(mn, z1, z2, beta_deg, alpha_n_deg, b):
    beta, an = math.radians(beta_deg), math.radians(alpha_n_deg)
    mt = mn / math.cos(beta)
    at = math.atan(math.tan(an) / math.cos(beta))
    r1, r2 = mt * z1 / 2, mt * z2 / 2
    a = r1 + r2
    rb1, rb2 = r1 * math.cos(at), r2 * math.cos(at)
    ra1, ra2 = r1 + mn, r2 + mn
    eps_a = (math.sqrt(ra1 ** 2 - rb1 ** 2) + math.sqrt(ra2 ** 2 - rb2 ** 2) - a * math.sin(at)) / (math.pi * mt * math.cos(at))
    return dict(mt=mt, at=at, a=a, th1=b * math.tan(beta) / r1, th2=b * math.tan(beta) / r2,
                zmin=2 * math.cos(beta) / math.sin(at) ** 2, eps_a=eps_a, eps_b=b * math.sin(beta) / (math.pi * mn))


# ---------- the tooth, in the transverse plane (all in cm and radians) ----------
def inv(a):
    return math.tan(a) - a


def involute_pt(rb, t):
    return (rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t)))


def rotate(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def transverse_tooth(mt, mn, z, at, backlash, phase, n=8):
    """Chapter 11's tooth drawn in the transverse plane: pitch circle and pressure angle from the transverse module
    and angle, addendum and dedendum from the normal module."""
    r = mt * z / 2.0                        # pitch radius
    rb = r * math.cos(at)                   # base radius
    ra = r + mn                             # tip radius
    rf = r - 1.25 * mn                      # root radius
    r_lo = max(rb, rf)
    t_at = lambda rr: 0.0 if rr <= rb else math.sqrt((rr / rb) ** 2 - 1.0)
    half = (math.pi * mt / 2.0 - backlash / 2.0) / (2.0 * r)      # half the tooth thickness, as an angle at r
    rot = half + inv(at)
    right, left = [], []
    for i in range(n + 1):
        t = t_at(r_lo) + (t_at(ra) - t_at(r_lo)) * i / n
        x, y = involute_pt(rb, t)
        right.append(rotate([(x, -y)], rot)[0])
        left.append(rotate([(x, y)], -rot)[0])
    a_l, a_r = math.atan2(left[0][1], left[0][0]), math.atan2(right[0][1], right[0][0])
    r_in = rf * 0.985
    pts = [(r_in * math.cos(a_r), r_in * math.sin(a_r)), (r_in * math.cos(a_l), r_in * math.sin(a_l))]
    if r_lo > r_in + 1e-9:
        pts.append((r_lo * math.cos(a_l), r_lo * math.sin(a_l)))
    pts.extend(left[1:] if r_lo <= r_in + 1e-9 else left)
    a0, a1 = math.atan2(left[-1][1], left[-1][0]), math.atan2(right[-1][1], right[-1][0])
    pts.extend((ra * math.cos(a0 + (a1 - a0) * j / 5), ra * math.sin(a0 + (a1 - a0) * j / 5)) for j in range(1, 5))
    pts.extend(reversed(right))
    clean = [pts[0]]
    for q in pts[1:]:
        if math.hypot(q[0] - clean[-1][0], q[1] - clean[-1][1]) > 1e-7:
            clean.append(q)
    return rotate(clean, phase), rf


def polyline(sk, pts):
    """One closed polyline whose lines share their sketch points."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


# ---------- Fusion building blocks (mm at the interface) ----------
def place(root, name, x=0.0, y=0.0):
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


def twisted_tooth(comp, body, outline, z0, length, twist):
    """The tooth outline at height z0 swept up the gear's axis for `length` mm, turning by `twist` radians
    (positive: anticlockwise seen from above, a right-hand tooth)."""
    sk = comp.sketches.add(plane_at(comp, z0))
    polyline(sk, outline)
    ps = comp.sketches.add(comp.xYConstructionPlane)
    axis = ps.sketchCurves.sketchLines.addByTwoPoints(P3(0, 0, z0 * CM), P3(0, 0, (z0 + length) * CM))
    ps.isVisible = False
    sw = comp.features.sweepFeatures
    inp = sw.createInput(sk.profiles.item(0), comp.features.createPath(axis, False), JOIN)
    inp.twistAngle = VI(twist)
    inp.participantBodies = [body]
    return sw.add(inp)


def gear(comp, name, z0, h, z, b, phase, theta, herringbone):
    """Root disc with a bore, the twisted tooth (two halves for a herringbone), a circular pattern: one body.
    theta = the tooth's turn over the face width, positive for a right-hand gear. Returns the body and the twist
    Fusion actually stored: it keeps a sweep's twist in tenths of a degree."""
    pts, rf = transverse_tooth(h['mt'] * CM, h['mn'] * CM, z, h['at'], BACKLASH * CM, phase)
    sk = comp.sketches.add(plane_at(comp, z0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), rf)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (DS + BORE_CLEAR) / 2 * CM)
    body = extrude(comp, [p for p in sk.profiles if p.profileLoops.count == 2][0], b).bodies.item(0)
    body.name = name
    feats = adsk.core.ObjectCollection.create()
    if herringbone:
        lower = twisted_tooth(comp, body, pts, z0, b / 2, theta / 2)
        stored = lower.twistAngle.value                                # read back what Fusion kept
        upper, _ = transverse_tooth(h['mt'] * CM, h['mn'] * CM, z, h['at'], BACKLASH * CM, phase + stored)
        feats.add(lower)
        feats.add(twisted_tooth(comp, body, upper, z0 + b / 2, b / 2, -stored))
    else:
        sweep = twisted_tooth(comp, body, pts, z0, b, theta)
        stored = sweep.twistAngle.value
        feats.add(sweep)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(z)
    pi.totalAngle = VS('360 deg')
    pats.add(pi)
    return body, stored


def shaft(comp, height):
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), 1.6 * DS / 2 * CM)
    body = extrude(comp, sk.profiles.item(0), GAP).bodies.item(0)
    s2 = comp.sketches.add(plane_at(comp, GAP))
    s2.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), DS / 2 * CM)
    extrude(comp, s2.profiles.item(0), height - GAP, JOIN, body)
    body.name = comp.name


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


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the assembly ----------
def build(mn=2.0, z1=20, z2=40, beta=20.0, alpha_n=20.0, b=12.0, herringbone=False):
    z1, z2 = int(round(z1)), int(round(z2))
    if not 5.0 <= beta <= 45.0:
        raise ValueError('Helix angle %.1f deg: keep it from 5 to 45 degrees.' % beta)
    h = helical(mn, z1, z2, beta, alpha_n, b)
    if min(z1, z2) < h['zmin']:
        raise ValueError('%d teeth: below z_min = 2 cos(beta) / sin^2(alpha_t) = %.2f the involute is undercut.'
                         % (min(z1, z2), h['zmin']))
    # Fusion keeps a sweep's twist in tenths of a degree: make gear 1's twist a whole number of tenths (of two tenths,
    # each half of a herringbone) by adjusting the face width; gear 2's twist then follows from the ratio
    k = 2 if herringbone else 1
    step = math.radians(0.1 * k)
    b_asked, b = b, round(h['th1'] / step) * step * (h['mt'] * z1 / 2) / math.tan(math.radians(beta))
    h = helical(mn, z1, z2, beta, alpha_n, b)
    sweep2 = math.degrees(h['th2']) / k                             # gear 2's twist per sweep, before Fusion rounds it
    tip_err = abs(sweep2 - round(sweep2, 1)) * math.pi / 180 * (h['mt'] * z2 / 2 + mn)
    if tip_err > BACKLASH / 4:
        raise ValueError('Gear 2\'s twist of %.3f deg per sweep, kept in tenths of a degree, would move its tips by '
                         '%.3f mm: more than a quarter of the backlash. Change the face width.' % (sweep2, tip_err))
    h['mn'], h['beta'] = mn, math.radians(beta)
    a = h['a']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    ra1, ra2 = h['mt'] * z1 / 2 + mn, h['mt'] * z2 / 2 + mn
    plate = place(root, 'Base plate')
    pc = plate.component
    sk = pc.sketches.add(plane_at(pc, -6.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3((-ra1 - 6) * CM, (-ra2 - 6) * CM, 0), P3((a + ra2 + 6) * CM, (ra2 + 6) * CM, 0))
    extrude(pc, sk.profiles.item(0), 6.0).bodies.item(0).name = 'Base plate'
    plate.isGrounded = True
    s1 = place(root, 'Shaft 1')
    shaft(s1.component, GAP + b + 3.0)
    s2 = place(root, 'Shaft 2', a, 0)
    shaft(s2.component, GAP + b + 3.0)
    g1 = place(root, 'Gear 1')
    _, t1 = gear(g1.component, 'Gear 1', GAP, h, z1, b, 0.0, h['th1'], herringbone)                   # right hand
    g2 = place(root, 'Gear 2', a, 0)
    _, t2 = gear(g2.component, 'Gear 2', GAP, h, z2, b, math.pi - math.pi / z2, -h['th2'], herringbone)  # left hand

    as_built(root, s1, plate, 'rigid', 'Shaft 1 on plate')
    as_built(root, s2, plate, 'rigid', 'Shaft 2 on plate')
    j1 = as_built(root, g1, s1, 'revolute', 'Gear 1 turns')
    j2 = as_built(root, g2, s2, 'revolute', 'Gear 2 turns')
    g = math.gcd(z1, z2)
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % (360 * z2 // g))
    li.valueTwo = VS('%d deg' % (360 * z1 // g))
    li.isReversed = True
    root.motionLinks.add(li).name = 'Mesh %d:%d' % (z1, z2)
    colour(design, plate, 'Paint - Enamel Glossy (Grey)')
    colour(design, g1, 'Paint - Enamel Glossy (Blue)')
    colour(design, g2, 'Paint - Enamel Glossy (Yellow)')
    app.activeViewport.fit()

    return ('%s pair built: normal module %.2f, %d : %d teeth, helix angle %.1f deg (gear 1 right hand).\n\n'
            'm_t = %.5f mm, alpha_t = %.3f deg, centre distance a = %.3f mm\n'
            'Face width %.3f mm (asked %.3f): twist over it, gear 1 %.3f deg, gear 2 %.3f deg\n'
            'Twist per sweep kept by Fusion: gear 1 %.1f deg, gear 2 %.1f deg\n'
            'z_min = %.2f; transverse contact ratio %.3f, overlap ratio %.3f\n\n[%s]'
            % ('Herringbone' if herringbone else 'Helical', mn, z1, z2, beta, h['mt'], math.degrees(h['at']), a,
               b, b_asked, math.degrees(h['th1']), -math.degrees(h['th2']), math.degrees(t1), math.degrees(t2),
               h['zmin'], h['eps_a'], h['eps_b'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(mn=ins.itemById('mn').value / CM, z1=ins.itemById('z1').value, z2=ins.itemById('z2').value,
                     beta=math.degrees(ins.itemById('beta').value), alpha_n=math.degrees(ins.itemById('alpha_n').value),
                     b=ins.itemById('b').value / CM, herringbone=ins.itemById('herringbone').selectedItem.name == 'yes')
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('mn', 'Normal module', 'mm', VI(2.0 * CM))
        ins.addIntegerSpinnerCommandInput('z1', 'Driver teeth z1', 6, 400, 1, 20)
        ins.addIntegerSpinnerCommandInput('z2', 'Driven teeth z2', 6, 400, 1, 40)
        ins.addValueInput('beta', 'Helix angle', 'deg', VI(math.radians(20.0)))
        ins.addValueInput('alpha_n', 'Normal pressure angle', 'deg', VI(math.radians(20.0)))
        ins.addValueInput('b', 'Face width', 'mm', VI(12.0 * CM))
        dd = ins.addDropDownCommandInput('herringbone', 'Herringbone', adsk.core.DropDownStyles.TextListDropDownStyle)
        dd.listItems.add('no', True)
        dd.listItems.add('yes', False)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'HelicalPairCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Helical pair', 'A pair of helical or herringbone gears')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Helical pair could not start:\n' + traceback.format_exc())
