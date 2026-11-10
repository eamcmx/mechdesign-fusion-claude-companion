# =============================================================================
# PlanetaryPrompt.py  -  Chapter 15 of "Mechanical Design in Autodesk Fusion with Claude": the planetary gearbox
# that its prompt asks for (PROMPT.txt beside it),
# written from that prompt alone: every size, rule, joint and link below is a line of the prompt.
#
#   INPUTS        FIELDS (the dialog)
#   DERIVED       build(): Zr, a, theta_i, psi_i, phi_r, the three ratios
#   HARD RULES    build() refuses uneven spacing and touching planets; tooth_outline(): involute + radial line;
#                 gear_body(): disc + one tooth + pattern; ring_body(): annulus + one space cut + pattern
#   CONVENTIONS   FusionDesignDocumentType; components created where they belong; cm inside the API;
#                 As-built Joints; ring grounded; planets on the carrier; meshes as Motion Links
#   ACCEPTANCE    the message; sun 360 deg -> carrier 80 deg, planets -224 deg on the carrier; no interference;
#                 Zs 24 / Zp 30 / N 4 builds, Zs 20 / Zp 25 / N 4 refused
#
# Checked by checks/planetary_check.py.
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'PlanetaryPrompt 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

# (id, label, default, unit): 'mm' and 'deg' are value inputs, 'n' an integer
FIELDS = [('m', 'Module m', 1.5, 'mm'), ('zs', 'Sun teeth Zs', 16, 'n'), ('zp', 'Planet teeth Zp', 20, 'n'),
          ('n', 'Number of planets N', 3, 'n'), ('alpha', 'Pressure angle', 20.0, 'deg'),
          ('b', 'Face width b', 12.0, 'mm'), ('backlash', 'Backlash on the pitch circle', 0.3, 'mm'),
          ('d_pin', 'Planet pin diameter', 6.0, 'mm'), ('d_in', 'Sun shaft diameter (input)', 8.0, 'mm'),
          ('d_out', 'Carrier shaft diameter (output)', 12.0, 'mm'), ('ct', 'Carrier plate thickness', 6.0, 'mm'),
          ('wall', 'Ring wall', 6.0, 'mm')]
SHAFT_L = 20.0                             # mm, both shafts stick out 20 mm


# ---------- the tooth (cm and radians) ----------
def inv(a):
    return math.tan(a) - a


def involute_pt(rb, t):
    return (rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t)))


def rotate(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def tooth_outline(m, z, alpha, backlash, phase, r_top=None, r_bot=None, n=8):
    """One tooth of an outside gear, centred on angle `phase`, as a closed list of points: up the left involute,
    over the tip, down the right involute, closed just below the root circle. Below the base circle there is no
    involute: the flank starts with a radial line. A negative backlash and other tip and root radii give the tooth
    SPACE of the ring (the same involutes, a little wider)."""
    r = m * z / 2.0
    rb = r * math.cos(alpha)
    ra = r + m if r_top is None else r_top
    rf = r - 1.25 * m if r_bot is None else r_bot
    r_lo = max(rb, rf)
    t_at = lambda rr: 0.0 if rr <= rb else math.sqrt((rr / rb) ** 2 - 1.0)
    half = (math.pi * m / 2.0 - backlash / 2.0) / (2.0 * r)
    rot = half + inv(alpha)
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
    pts.extend(left)
    a0, a1 = math.atan2(left[-1][1], left[-1][0]), math.atan2(right[-1][1], right[-1][0])
    pts.extend((ra * math.cos(a0 + (a1 - a0) * j / 5), ra * math.sin(a0 + (a1 - a0) * j / 5)) for j in range(1, 5))
    pts.extend(reversed(right))
    clean = [pts[0]]                         # no zero-length lines
    for q in pts[1:]:
        if math.hypot(q[0] - clean[-1][0], q[1] - clean[-1][1]) > 1e-7:
            clean.append(q)
    return rotate(clean, phase)


# ---------- Fusion building blocks (mm at the interface) ----------
def place(root, name, x=0.0, y=0.0):
    """A component created where it belongs: a move made after creation is undone by the next feature."""
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


def circles(comp, z_mm, *specs):
    """A sketch at height z_mm with circles (x, y, diameter) in mm."""
    sk = comp.sketches.add(plane_at(comp, z_mm))
    for x, y, d in specs:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), d * CM / 2)
    return sk


def extrude(comp, prof, length_mm, op=NEW, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def polyline(sk, pts):
    """One closed polyline whose lines share their sketch points."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def pattern(comp, feature, count):
    feats = adsk.core.ObjectCollection.create()
    feats.add(feature)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(count)
    pi.totalAngle = VS('360 deg')
    return pats.add(pi)


def ring_profile(sk):
    """The profile with two loops: the ring between two circles, not the hole inside."""
    return [p for p in sk.profiles if p.profileLoops.count == 2][0]


def gear_body(comp, m, z, alpha, b, backlash, phase, bore, name):
    """An outside gear from z = 0 to b: a root disc (with a bore if bore > 0), one tooth joined, a pattern of z."""
    rf = m * z / 2.0 - 1.25 * m
    sk = circles(comp, 0, (0, 0, 2 * rf), *([(0, 0, bore)] if bore > 0 else []))
    prof = ring_profile(sk) if bore > 0 else sk.profiles.item(0)
    body = extrude(comp, prof, b).bodies.item(0)
    body.name = name
    st = comp.sketches.add(comp.xYConstructionPlane)
    polyline(st, tooth_outline(m * CM, z, math.radians(alpha), backlash * CM, phase))
    pattern(comp, extrude(comp, st.profiles.item(0), b, JOIN, body), z)
    return body


def ring_body(comp, m, zr, alpha, b, backlash, wall, phi_r):
    """The ring: an annulus from Rr - m to Rr + 1.25 m + wall, one tooth space cut, the cut patterned Zr times."""
    Rr = m * zr / 2.0
    sk = circles(comp, 0, (0, 0, 2 * (Rr + 1.25 * m + wall)), (0, 0, 2 * (Rr - m)))
    body = extrude(comp, ring_profile(sk), b).bodies.item(0)
    body.name = 'Ring'
    st = comp.sketches.add(comp.xYConstructionPlane)
    polyline(st, tooth_outline(m * CM, zr, math.radians(alpha), -backlash * CM, phi_r,
                               r_top=(Rr + 1.25 * m) * CM, r_bot=(Rr - m) * CM))
    pattern(comp, extrude(comp, st.profiles.item(0), b, CUT, body), zr)
    return body


def origin_point(occ):
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(occ)


def as_built(root, moving, fixed, revolute, name):
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(origin_point(moving)))
    if revolute:
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def motion_link(root, j1, j2, turns1, turns2, reverse, name):
    """While j1 turns turns1 x 360 deg, j2 turns turns2 x 360 deg (reduced by their common factor)."""
    g = math.gcd(int(turns1), int(turns2))
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % (360 * int(turns1) // g))
    li.valueTwo = VS('%d deg' % (360 * int(turns2) // g))
    li.isReversed = reverse
    ml = root.motionLinks.add(li)
    ml.name = name
    return ml


# ---------- the gearbox ----------
def build(m=1.5, zs=16, zp=20, n=3, alpha=20.0, b=12.0, backlash=0.3, d_pin=6.0, d_in=8.0, d_out=12.0, ct=6.0,
          wall=6.0):
    zs, zp, n = int(round(zs)), int(round(zp)), int(round(n))
    # DERIVED
    zr = zs + 2 * zp
    a = m * (zs + zp) / 2.0
    theta = [2 * math.pi * i / n for i in range(n)]
    pitch_p = 2 * math.pi / zp
    psi = [((t * (zs + zp) + math.pi * (zp - 1)) / zp) % pitch_p for t in theta]
    phi_r = 0.0 if zp % 2 else math.pi / zr
    ratios = (1 + zr / zs, -zr / zs, 1 + zs / zr)
    # HARD CONSTRAINTS
    if (zs + zr) % n:
        raise ValueError('(Zs + Zr) / N = (%d + %d) / %d = %.2f is not whole: %d planets cannot be spaced evenly.'
                         % (zs, zr, n, (zs + zr) / n, n))
    if 2 * a * math.sin(math.pi / n) <= m * (zp + 2):
        raise ValueError('Neighbouring planets touch: 2 · a · sin(180 / N) = %.1f mm is not more than their tip '
                         'diameter m · (Zp + 2) = %.1f mm.' % (2 * a * math.sin(math.pi / n), m * (zp + 2)))

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    ring = place(root, 'Ring')
    ring_body(ring.component, m, zr, alpha, b, backlash, wall, phi_r)

    sun = place(root, 'Sun')
    sun_b = gear_body(sun.component, m, zs, alpha, b, backlash, 0.0, 0, 'Sun')
    top_in = b + 1 + ct / 2.0 - 0.5                     # the sun shaft ends 0.5 mm short of the bore's end
    sk = circles(sun.component, -SHAFT_L, (0, 0, d_in))
    extrude(sun.component, sk.profiles.item(0), SHAFT_L + top_in, JOIN, sun_b)

    planets = []
    for i in range(n):
        x, y = a * math.cos(theta[i]), a * math.sin(theta[i])
        p = place(root, 'Planet %d' % (i + 1), x, y)
        gear_body(p.component, m, zp, alpha, b, backlash, psi[i], d_pin + 0.2, 'Planet %d' % (i + 1))
        planets.append(p)

    carrier = place(root, 'Carrier')
    cc = carrier.component
    z0 = b + 1.0
    disc = extrude(cc, circles(cc, z0, (0, 0, 2 * (a + d_pin))).profiles.item(0), ct).bodies.item(0)
    disc.name = 'Carrier'
    pins = circles(cc, 0, *[(a * math.cos(t), a * math.sin(t), d_pin) for t in theta])
    for k in range(pins.profiles.count):
        extrude(cc, pins.profiles.item(k), z0, JOIN, disc)
    extrude(cc, circles(cc, z0, (0, 0, d_in + 4.4)).profiles.item(0), ct / 2.0, CUT, disc)
    extrude(cc, circles(cc, z0 + ct, (0, 0, d_out)).profiles.item(0), SHAFT_L, JOIN, disc)

    bearing = place(root, 'Sun bearing')
    bc = bearing.component
    extrude(bc, ring_profile(circles(bc, z0, (0, 0, d_in + 0.2), (0, 0, d_in + 4))), ct / 2.0).bodies.item(0).name = 'Sun bearing'

    # JOINTS and the meshes as MOTION LINKS
    ring.isGrounded = True
    j_sun = as_built(root, sun, ring, True, 'Sun (input)')
    j_car = as_built(root, carrier, ring, True, 'Carrier (output)')
    as_built(root, bearing, carrier, False, 'Sun bearing in carrier')
    motion_link(root, j_sun, j_car, zs + zr, zs, False, 'Sun to carrier')
    for i, p in enumerate(planets):
        jp = as_built(root, p, carrier, True, 'Planet %d on carrier' % (i + 1))
        motion_link(root, j_car, jp, zp, zr, True, 'Planet %d spin' % (i + 1))
    app.activeViewport.fit()

    return ('Planetary gearbox built.\n\n'
            'Zs = %d, Zp = %d, Zr = Zs + 2 · Zp = %d, %d planets, module %.2f mm\n'
            'Ring held (sun in, carrier out):    1 + Zr / Zs = %.4f\n'
            'Carrier held (sun in, ring out):    -Zr / Zs = %.4f\n'
            'Sun held (ring in, carrier out):    1 + Zs / Zr = %.4f\n\n'
            'Drive Joints on "Sun (input)": one turn moves the carrier %.2f deg the same way.\n\n[%s]'
            % (zs, zp, zr, n, m, ratios[0], ratios[1], ratios[2], 360.0 / ratios[0], VERSION))


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
        cmd.setDialogInitialSize(440, 440)
        inputs = cmd.commandInputs
        for key, label, d, unit in FIELDS:
            if unit == 'n':
                inputs.addIntegerSpinnerCommandInput(key, label, 2, 300, 1, int(d))
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
        cid = 'PlanetaryPromptCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Planetary gearbox (from the prompt)',
                                                         'A planetary gearbox, jointed and linked')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Planetary gearbox could not start:\n' + traceback.format_exc())
