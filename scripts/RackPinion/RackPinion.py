# =============================================================================
# RackPinion.py  -  Chapter 15 of "Mechanical Design in Autodesk Fusion with Claude": a spur pinion on a fixed shaft
# and a straight rack sliding under it, assembled so that Drive Joints on the pinion moves the rack. Written from
# PROMPT.txt beside it; checked by checks/rack_pinion_check.py.
#
#   INPUTS        the dialog: m, z, N_r, alpha, b
#   DERIVED       rack_pinion(): r, r_b, r_a, p, travel per turn, contact ratio, travel limits
#   HARD RULES    build() refuses an undercut pinion and a rack too short to travel; a rack gap under the pinion's
#                 tooth 0; the rack's pitch line tangent to the pinion's pitch circle
#   CONVENTIONS   a new design; parts created where they belong; the pinion's tooth and the whole rack each one
#                 closed polyline; cm and radians inside the API; As-built Joints; a revolute-to-slider Motion Link
#   ACCEPTANCE    the message; 90 deg moves the rack 31.416 mm; no collision through one pitch; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'RackPinion 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
GAP = 1.0                                  # mm, the collar that lifts the pinion off the plate
BORE_CLEAR = 0.2                           # mm
BACKLASH = 0.2                             # mm on the pitch circle / pitch line
DS = 8.0                                   # mm, shaft diameter
RACK_BODY = 8.0                            # mm of rack below the tooth roots
Z_MIN = 18
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation


def rack_pinion(m, z, n_r, alpha_deg):
    al = math.radians(alpha_deg)
    r = m * z / 2.0
    rb, ra = r * math.cos(al), r + m
    p = math.pi * m
    eps = (math.sqrt(ra ** 2 - rb ** 2) - r * math.sin(al) + m / math.sin(al)) / (math.pi * m * math.cos(al))
    return dict(r=r, rb=rb, ra=ra, p=p, per_turn=math.pi * m * z, per_deg=r * math.pi / 180, eps=eps,
                limit=(n_r / 2.0 - 2) * p)


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




def polyline(sk, pts):
    """One closed polyline whose lines share their sketch points (closed splines are unreliable as profiles)."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def rack_outline(m, n_r, alpha, backlash):
    """The whole rack, teeth and body, as one closed list of points in cm: pitch line on y = 0, teeth pointing up,
    tooth k centred at x = (k + 1/2) p - (n_r // 2) p, so that a GAP is centred on x = 0."""
    p = math.pi * m
    half = p / 4.0 - backlash / 4.0                       # half the tooth thickness on the pitch line
    top, root = half - m * math.tan(alpha), half + 1.25 * m * math.tan(alpha)
    y_top, y_root, y_bot = m, -1.25 * m, -1.25 * m - RACK_BODY * CM
    xs = [(k + 0.5) * p - (n_r // 2) * p for k in range(n_r)]
    x_left, x_right = xs[0] - p / 2, xs[-1] + p / 2
    pts = [(x_left, y_bot), (x_left, y_root)]
    for x in xs:
        pts += [(x - root, y_root), (x - top, y_top), (x + top, y_top), (x + root, y_root)]
    pts += [(x_right, y_root), (x_right, y_bot)]
    return pts


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


def gear_body(comp, m, z, alpha_deg, b, z0, phase, name):
    """Chapter 11's gear: root disc with a bore, one tooth joined to it, a circular pattern of that tooth."""
    pts, rf = tooth_outline(m * CM, z, math.radians(alpha_deg), BACKLASH * CM, phase)
    sk = comp.sketches.add(plane_at(comp, z0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), rf)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (DS + BORE_CLEAR) / 2 * CM)
    body = extrude(comp, [q for q in sk.profiles if q.profileLoops.count == 2][0], b).bodies.item(0)
    body.name = name
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


def as_built(root, moving, fixed, motion, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    if motion == 'revolute':
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif motion == 'slider':
        inp.setAsSliderJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
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
def build(m=2.0, z=20, n_r=16, alpha=20.0, b=10.0):
    z, n_r = int(round(z)), int(round(n_r))
    if z < Z_MIN:
        raise ValueError('%d teeth: below %d teeth at 20 degrees the involute is undercut.' % (z, Z_MIN))
    if n_r < 8:
        raise ValueError('%d rack teeth: at least 8, or the rack cannot travel and stay in mesh.' % n_r)
    k = rack_pinion(m, z, n_r, alpha)
    r = k['r']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    half_len = n_r // 2 * k['p'] + k['p']
    plate = place(root, 'Base plate')
    pc = plate.component
    sk = pc.sketches.add(plane_at(pc, -6.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3((-half_len - k['limit']) * CM, (-1.25 * m - RACK_BODY - 6) * CM, 0),
                                                     P3((half_len + k['limit']) * CM, (2 * r + m + 6) * CM, 0))
    extrude(pc, sk.profiles.item(0), 6.0).bodies.item(0).name = 'Base plate'
    plate.isGrounded = True

    shaft = place(root, 'Shaft', 0, r)
    c = shaft.component
    sk = c.sketches.add(c.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), 1.6 * DS / 2 * CM)
    body = extrude(c, sk.profiles.item(0), GAP).bodies.item(0)
    s2 = c.sketches.add(plane_at(c, GAP))
    s2.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), DS / 2 * CM)
    extrude(c, s2.profiles.item(0), b + 3.0, JOIN, body)
    body.name = 'Shaft'

    pinion = place(root, 'Pinion', 0, r)
    gear_body(pinion.component, m, z, alpha, b, GAP, -math.pi / 2, 'Pinion')          # tooth 0 straight down

    rack = place(root, 'Rack')
    c = rack.component
    sk = c.sketches.add(plane_at(c, GAP))
    polyline(sk, rack_outline(m * CM, n_r, math.radians(alpha), BACKLASH * CM))
    extrude(c, sk.profiles.item(0), b).bodies.item(0).name = 'Rack'

    as_built(root, shaft, plate, 'rigid', 'Shaft on plate')
    jp = as_built(root, pinion, shaft, 'revolute', 'Pinion turns')
    jr = as_built(root, rack, plate, 'slider', 'Rack slides')
    lim = jr.jointMotion.slideLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = -k['limit'] * CM
    lim.isMaximumValueEnabled = True
    lim.maximumValue = k['limit'] * CM
    li = root.motionLinks.createInput(jp, jr)
    li.valueOne = VS('360 deg')                                   # one turn of the pinion ...
    li.valueTwo = VI(k['per_turn'] * CM)                          # ... moves the rack pi m z
    root.motionLinks.add(li).name = 'Rack and pinion'
    colour(design, plate, 'Paint - Enamel Glossy (Grey)')
    colour(design, pinion, 'Paint - Enamel Glossy (Blue)')
    colour(design, rack, 'Paint - Enamel Glossy (Yellow)')
    app.activeViewport.fit()

    return ('Rack and pinion built: module %.2f, pinion %d teeth, rack %d teeth.\n\n'
            'Pinion pitch radius r = m z / 2 = %.2f mm; rack pitch p = pi m = %.3f mm\n'
            'Rack travel per pinion turn pi m z = %.3f mm (%.4f mm per degree)\n'
            'Contact ratio %.3f; rack limits plus and minus %.3f mm\n\n'
            'Drive Joints on "Pinion turns": 90 deg moves the rack %.3f mm towards +x.\n\n[%s]'
            % (m, z, n_r, r, k['p'], k['per_turn'], k['per_deg'], k['eps'], k['limit'], 90 * k['per_deg'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(m=ins.itemById('m').value / CM, z=ins.itemById('z').value, n_r=ins.itemById('n_r').value,
                     alpha=math.degrees(ins.itemById('alpha').value), b=ins.itemById('b').value / CM)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('m', 'Module', 'mm', VI(2.0 * CM))
        ins.addIntegerSpinnerCommandInput('z', 'Pinion teeth', 6, 200, 1, 20)
        ins.addIntegerSpinnerCommandInput('n_r', 'Rack teeth', 2, 400, 1, 16)
        ins.addValueInput('alpha', 'Pressure angle', 'deg', VI(math.radians(20.0)))
        ins.addValueInput('b', 'Face width', 'mm', VI(10.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'RackPinionCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Rack and pinion', 'A pinion driving a sliding rack')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Rack and pinion could not start:\n' + traceback.format_exc())
