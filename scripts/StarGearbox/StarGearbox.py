# =============================================================================
# StarGearbox.py  -  Chapter 32 of "Mechanical Design in Autodesk Fusion with Claude": the star gearbox of a geared
# turbofan. A planetary gear set with its carrier held: the low-pressure turbine drives the sun, the sun turns N
# star gears on fixed pins, and the stars turn the ring, which drives the fan the other way at zs / zr of the speed.
# Written from PROMPT.txt beside it; checked by checks/star_gearbox_check.py.
#
#   INPUTS        FIELDS (the dialog): m, zs, zp, N, alpha, b, backlash, pin, shafts, carrier plate, ring wall
#   DERIVED       star(): zr, a, the stars' angles and phases, the ring's phase, the ratio, speeds and torques
#   HARD RULES    build() refuses uneven spacing, touching stars and stars whose phases would differ; the
#                 phases of Chapter 15; the ring turning against the sun at zs / zr
#   CONVENTIONS   FusionDesignDocumentType; components created where they belong; the carrier grounded; gears
#                 from sketches (Chapter 15); As-built Joints; the meshes as Motion Links
#   ACCEPTANCE    the message; sun 90 deg -> ring -29.388 deg, stars -87.273 deg, measured on the bodies; no tooth
#                 collision over one sun tooth; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'StarGearbox 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

# (id, label, default, unit): 'mm' and 'deg' are value inputs, 'n' an integer
FIELDS = [('m', 'Module m', 3.0, 'mm'), ('zs', 'Sun teeth Zs', 32, 'n'), ('zp', 'Star teeth Zp', 33, 'n'),
          ('n', 'Number of stars N', 5, 'n'), ('alpha', 'Pressure angle', 20.0, 'deg'),
          ('b', 'Face width b', 40.0, 'mm'), ('backlash', 'Backlash on the pitch circle', 0.3, 'mm'),
          ('d_pin', 'Star pin diameter', 20.0, 'mm'), ('d_in', 'Sun shaft diameter (from the turbine)', 30.0, 'mm'),
          ('d_out', 'Fan shaft diameter', 60.0, 'mm'), ('ct', 'Carrier and ring plates', 8.0, 'mm'),
          ('wall', 'Ring wall', 10.0, 'mm')]
SHAFT_L = 60.0                             # mm, both shafts stick out 60 mm
POWER, N_TURBINE, LOSS = 20e6, 10000.0, 0.005     # W, rpm, the share of the power lost in the meshes


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



def star(m, zs, zp, n, alpha):
    """Everything worked out from the inputs."""
    zr = zs + 2 * zp
    a = m * (zs + zp) / 2.0
    theta = [2 * math.pi * i / n for i in range(n)]
    pitch_p = 2 * math.pi / zp
    psi = [((t * (zs + zp) + math.pi * (zp - 1)) / zp) % pitch_p for t in theta]
    phi_r = 0.0 if zp % 2 else math.pi / zr
    w_s = N_TURBINE
    w_r = -N_TURBINE * zs / zr
    T_s = POWER / (w_s * 2 * math.pi / 60.0)
    T_r = T_s * zr / zs
    return dict(zr=zr, a=a, theta=theta, psi=psi, phi_r=phi_r, ratio=zr / zs, n_fan=w_r, T_s=T_s, T_r=T_r,
                T_c=T_s + T_r, Ft=T_s / (n * m * zs / 2.0 / 1000.0), heat=LOSS * POWER,
                spacing=(zs + zr) / n, phases_equal=(zs + zp) % n == 0, gap=2 * a * math.sin(math.pi / n) - m * (zp + 2))


# ---------- the gearbox ----------
def build(m=3.0, zs=32, zp=33, n=5, alpha=20.0, b=40.0, backlash=0.3, d_pin=20.0, d_in=30.0, d_out=60.0, ct=8.0,
          wall=10.0):
    zs, zp, n = int(round(zs)), int(round(zp)), int(round(n))
    g = star(m, zs, zp, n, alpha)
    zr, a, theta, psi, phi_r = g['zr'], g['a'], g['theta'], g['psi'], g['phi_r']
    if min(zs, zp) < 18:
        raise ValueError('A gear of %d teeth: below 18 an outside gear is undercut at 20 deg (Chapter 12).'
                         % min(zs, zp))
    if (zs + zr) % n:
        raise ValueError('(Zs + Zr) / N = (%d + %d) / %d = %.2f is not whole: %d stars cannot be spaced evenly.'
                         % (zs, zr, n, (zs + zr) / n, n))
    if g['gap'] <= 0:
        raise ValueError('Neighbouring stars touch: 2 a sin(180 / N) = %.2f mm is not more than their tip diameter '
                         '%.1f mm.' % (2 * a * math.sin(math.pi / n), m * (zp + 2)))

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    carrier = place(root, 'Star carrier')                  # held: the stars' pins and the gearbox's mounting
    cc = carrier.component
    plate = extrude(cc, circles(cc, -1.0 - ct, (0, 0, 2 * (a + d_pin))).profiles.item(0), ct).bodies.item(0)
    plate.name = 'Star carrier'
    pins = circles(cc, -1.0, *[(a * math.cos(t), a * math.sin(t), d_pin) for t in theta])
    for k in range(pins.profiles.count):
        extrude(cc, pins.profiles.item(k), 1.0 + b, JOIN, plate)
    extrude(cc, circles(cc, -1.0 - ct, (0, 0, d_in + 4.4)).profiles.item(0), ct, CUT, plate)
    carrier.isGrounded = True

    ring = place(root, 'Ring')
    rb = ring_body(ring.component, m, zr, alpha, b, backlash, wall, phi_r)
    rc = ring.component
    Ro = m * zr / 2.0 + 1.25 * m + wall
    extrude(rc, circles(rc, b + 1.0, (0, 0, 2 * Ro)).profiles.item(0), ct, NEW)      # the plate to the fan shaft
    extrude(rc, circles(rc, b + 1.0 + ct, (0, 0, d_out)).profiles.item(0), SHAFT_L, NEW)
    extrude(rc, ring_profile(circles(rc, b, (0, 0, 2 * Ro), (0, 0, 2 * (Ro - wall / 2.0)))), 1.0, NEW)  # joins them
    combine = rc.features.combineFeatures
    tools = adsk.core.ObjectCollection.create()
    for k in range(1, rc.bRepBodies.count):
        tools.add(rc.bRepBodies.item(k))
    ci = combine.createInput(rb, tools)
    ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    combine.add(ci)
    rc.bRepBodies.item(0).name = 'Ring'

    sun = place(root, 'Sun')
    sun_b = gear_body(sun.component, m, zs, alpha, b, backlash, 0.0, 0, 'Sun')
    sk = circles(sun.component, -1.0 - ct - SHAFT_L, (0, 0, d_in))
    extrude(sun.component, sk.profiles.item(0), SHAFT_L + ct + 1.0, JOIN, sun_b)

    stars = []
    for i in range(n):
        x, y = a * math.cos(theta[i]), a * math.sin(theta[i])
        p = place(root, 'Star %d' % (i + 1), x, y)
        gear_body(p.component, m, zp, alpha, b, backlash, psi[i], d_pin + 0.4, 'Star %d' % (i + 1))
        stars.append(p)

    # JOINTS and the meshes as MOTION LINKS
    j_sun = as_built(root, sun, carrier, True, 'Sun (from the turbine)')
    j_ring = as_built(root, ring, carrier, True, 'Ring (to the fan)')
    motion_link(root, j_sun, j_ring, zr, zs, True, 'Sun to ring')                   # ring = -(zs / zr) sun
    for i, p in enumerate(stars):
        jp = as_built(root, p, carrier, True, 'Star %d on its pin' % (i + 1))
        motion_link(root, j_sun, jp, zp, zs, True, 'Sun to star %d' % (i + 1))      # star = -(zs / zp) sun
    app.activeViewport.fit()

    return ('Star gearbox built: sun %d, %d stars of %d, ring %d teeth (Zs + 2 Zp), module %.2f mm, centre distance '
            '%.3f mm.\n\n'
            'Carrier held: the ring turns against the sun at Zs / Zr; ratio %.4f : 1. A turbine at %.0f rpm turns the '
            'fan at %.1f rpm.\n'
            'At %.0f MW: sun %.1f N m, ring %.1f N m, carrier reaction %.1f N m; %.0f N tangential at each sun mesh.\n'
            'Losing %.1f %% of the power heats the oil with %.0f kW.\n'
            '(Zs + Zr) / N = %d; (Zs + Zp) / N = %.2f: the stars %s.\n\n'
            'Drive Joints on "Sun (from the turbine)": the stars and the ring follow.\n\n[%s]'
            % (zs, n, zp, zr, m, a, g['ratio'], N_TURBINE, g['n_fan'], POWER / 1e6, g['T_s'], g['T_r'], g['T_c'],
               g['Ft'], 100 * LOSS, g['heat'] / 1000.0, int(g['spacing']), (zs + zp) / n,
               'all have the same phase' if g['phases_equal'] else 'need phases of their own', VERSION))


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
        cid = 'StarGearboxCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Star gearbox', 'The carrier-held gearbox of a geared turbofan')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Star gearbox could not start:\n' + traceback.format_exc())
