# =============================================================================
# SingleCylinderEngine.py  -  Chapter 28 of "Mechanical Design in Autodesk Fusion with Claude": a four-stroke,
# side-valve single-cylinder engine. A crankshaft, connecting rod and piston (a slider-crank); a camshaft beside the
# crankshaft, geared 2:1; an inlet and an exhaust valve lifted by cams through roller followers. Written from
# PROMPT.txt beside it; checked by checks/engine_check.py.
#
#   INPUTS        the dialog: bore B, stroke S, rod length l, valve lift h, cam base circle Rb, roller Rr, valve
#                 timing IVO, IVC, EVO, EVC (crank degrees from the dead centres)
#   DERIVED       engine(): r, lambda, swept volume, the valve events in crank degrees from the start of the inlet
#                 stroke; valve_lift(): the lift of each valve at a crank angle; cam(): the lobe and its limits
#   HARD RULES    build() refuses a rod shorter than 3 r, a pressure angle above 30 deg and an undercut lobe; the
#                 camshaft at half the crank's speed, the other way; built at top dead centre of compression
#   CONVENTIONS   crank axis = world Z, cylinder axis = world +Y, the crank turning anticlockwise seen from +Z;
#                 bodies from TemporaryBRepManager, gears and lobes from sketches; As-built Joints closing the
#                 crank-rod-piston loop; one Motion Link for the timing gears
#   ACCEPTANCE    the message; the piston on x(theta) at every 30 deg of two turns; the camshaft at -theta/2; each
#                 roller 0.05 mm from its lobe at every step; no interference; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'SingleCylinderEngine 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
GAP = 0.05                                 # mm between each roller and its lobe
CAM_X, LOBE_Z = 60.0, (12.0, 32.0)         # camshaft at (60, 0); inlet and exhaust lobes from these z, 8 mm thick
LOBE_T = 8.0
M_G, Z_CRANK, Z_CAM, B_G = 2.0, 20, 40, 10.0   # timing gears: module, teeth, face width; centre distance 60 mm
PHI_MAX = 30.0
REST = 360.0                               # built at top dead centre of compression: both valves shut


def engine(B, S, l, IVO, IVC, EVO, EVC):
    """Engine numbers; valve events in crank degrees from TDC at the start of the inlet stroke."""
    r = S / 2.0
    ev = dict(io=-IVO, ic=180.0 + IVC, eo=540.0 - EVO, ec=720.0 + EVC)
    return dict(r=r, lam=r / l, Vd=math.pi / 4 * B * B * S, ev=ev,
                d_in=ev['ic'] - ev['io'], d_ex=ev['ec'] - ev['eo'], overlap=ev['ec'] - 720.0 - ev['io'])


def piston_x(r, l, theta):
    """Crank axis to piston pin, theta from top dead centre (radians)."""
    return r * math.cos(theta) + math.sqrt(l * l - (r * math.sin(theta)) ** 2)


def lobe(h, duration_crank):
    """Lift against cam angle from the valve's opening (radians): cycloidal rise over a quarter of the open
    duration in crank degrees, cycloidal return over the next quarter, shut for the rest of the turn."""
    b = math.radians(duration_crank / 4.0)
    def s(t):
        t = t % (2 * math.pi)
        if t < b:
            u = t / b
            return h * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
        if t < 2 * b:
            u = (t - b) / b
            return h * (1 - (u - math.sin(2 * math.pi * u) / (2 * math.pi)))
        return 0.0
    return s


def valve_lift(e, h, which, theta_deg):
    """The lift of a valve at a crank angle (degrees from TDC at the start of the inlet stroke)."""
    opens, dur = (e['ev']['io'], e['d_in']) if which == 'in' else (e['ev']['eo'], e['d_ex'])
    return lobe(h, dur)(math.radians((theta_deg - opens) / 2.0))


def cam(Rb, Rr, h, duration_crank):
    """Pressure angle and the pitch curve's smallest radius of curvature for one lobe."""
    s = lobe(h, duration_crank)
    Rp = Rb + Rr
    phi, rho = 0.0, float('inf')
    for i in range(36000):
        t = 2 * math.pi * i / 36000
        rr = Rp + s(t)
        r1 = (s(t + 1e-6) - s(t - 1e-6)) / 2e-6
        r2 = (s(t + 1e-4) - 2 * s(t) + s(t - 1e-4)) / 1e-8
        phi = max(phi, abs(math.degrees(math.atan2(r1, rr))))
        den = rr * rr + 2 * r1 * r1 - rr * r2
        if abs(den) > 1e-12:
            q = (rr * rr + r1 * r1) ** 1.5 / den
            if q > 0:
                rho = min(rho, q)
    return dict(phi=phi, rho=rho, Rp=Rp)


def lobe_outline(e, h, Rb, Rr, which):
    """The lobe in the camshaft's own frame, as built at REST. The camshaft turns clockwise (seen from +Z) by half
    the crank's angle, and the roller sits on +Y: the point of the pitch curve at anticlockwise angle k from +Y comes
    under the roller after a clockwise turn of k, at crank angle REST + 2k. Offset towards the centre by Rr + GAP;
    one point per degree."""
    Rp = Rb + Rr
    pitch = []
    for k in range(360):
        R = Rp + valve_lift(e, h, which, REST + 2.0 * k)
        a = math.radians(k)
        pitch.append((-R * math.sin(a), R * math.cos(a)))
    n = len(pitch)
    area = sum(pitch[i][0] * pitch[(i + 1) % n][1] - pitch[(i + 1) % n][0] * pitch[i][1] for i in range(n)) / 2
    d = Rr + GAP
    out = []
    for i in range(n):
        (x0, y0), (x1, y1) = pitch[i - 1], pitch[(i + 1) % n]
        L = math.hypot(x1 - x0, y1 - y0)
        nx, ny = -(y1 - y0) / L, (x1 - x0) / L                      # left normal: inward for an anticlockwise curve
        if area < 0:
            nx, ny = -nx, -ny
        out.append((pitch[i][0] + d * nx, pitch[i][1] + d * ny))
    return out


# ---------- the timing gears (as in Chapter 12) ----------
def inv(a):
    return math.tan(a) - a


def involute_pt(rb, t):
    return (rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t)))


def rotate(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def tooth_outline(m, z, alpha, backlash, phase, n=8):
    """One tooth centred on angle `phase` (lengths in cm), closed just below the root circle."""
    r = m * z / 2.0
    rb = r * math.cos(alpha)
    ra = r + m
    rf = r - 1.25 * m
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
    pts.extend(left[1:] if r_lo <= r_in + 1e-9 else left)
    a0, a1 = math.atan2(left[-1][1], left[-1][0]), math.atan2(right[-1][1], right[-1][0])
    pts.extend((ra * math.cos(a0 + (a1 - a0) * j / 5), ra * math.sin(a0 + (a1 - a0) * j / 5)) for j in range(1, 5))
    pts.extend(reversed(right))
    clean = [pts[0]]
    for q in pts[1:]:
        if math.hypot(q[0] - clean[-1][0], q[1] - clean[-1][1]) > 1e-7:
            clean.append(q)
    return rotate(clean, phase), rf


# ---------- Fusion building blocks (mm at the interface) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cyl(p0, p1, r):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    obb = adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM, (y1 - y0) * CM,
                                                 (z1 - z0) * CM)
    return tbm().createBox(obb)


def boolean(a, kind, *bs):
    for b in bs:
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


def cut(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.DifferenceBooleanType, *bs)


def add_body(comp, tb, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def place(root, name, x=0.0, y=0.0, z=0.0):
    """A component created where it belongs: its origin at (x, y, z), axes as the world's."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = V3(x * CM, y * CM, z * CM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def polyline(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def extrude(comp, prof, z0, length_mm, op, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def gear_body(comp, z, phase, bore, name):
    """Root disc with a bore, one tooth joined, patterned z times; in the component's XY plane, B_G thick."""
    NEW, JOIN = adsk.fusion.FeatureOperations.NewBodyFeatureOperation, adsk.fusion.FeatureOperations.JoinFeatureOperation
    pts, rf = tooth_outline(M_G * CM, z, math.radians(20.0), 0.2 * CM, phase)
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), rf)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), bore * CM / 2)
    body = extrude(comp, max(list(sk.profiles), key=lambda p: p.areaProperties().area), 0, B_G, NEW).bodies.item(0)
    body.name = name
    st = comp.sketches.add(comp.xYConstructionPlane)
    polyline(st, pts)
    feats = adsk.core.ObjectCollection.create()
    feats.add(extrude(comp, st.profiles.item(0), 0, B_G, JOIN, body))
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(z)
    pi.totalAngle = VS('360 deg')
    pats.add(pi)
    return body


def joint(root, moving, fixed, x, y, z, kind, name):
    """An As-built joint at the world point (x, y, z): revolute about Z, slider along Y, or rigid."""
    t = moving.transform2.translation
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(x * CM - t.x, y * CM - t.y, z * CM - t.z)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    if kind == 'revolute':
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif kind == 'slider':
        inp.setAsSliderJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def turn_sign(occ, j):
    """+1 if a positive joint value turns the part anticlockwise about +Z, else -1."""
    t0 = occ.transform2
    j.jointMotion.rotationValue = 0.2
    t1 = occ.transform2
    j.jointMotion.rotationValue = 0.0
    a0 = math.atan2(t0.getCell(1, 0), t0.getCell(0, 0))
    a1 = math.atan2(t1.getCell(1, 0), t1.getCell(0, 0))
    return 1 if math.sin(a1 - a0) > 0 else -1


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the engine ----------
def build(B=60.0, S=54.0, l=90.0, h=6.0, Rb=15.0, Rr=5.0, IVO=10.0, IVC=50.0, EVO=50.0, EVC=10.0):
    e = engine(B, S, l, IVO, IVC, EVO, EVC)
    r = e['r']
    if l < 3 * r:
        raise ValueError('A rod of %.1f mm is shorter than 3 r = %.1f mm: the piston would be pushed hard against '
                         'the wall (lambda = %.3f).' % (l, 3 * r, e['lam']))
    for name, dur in (('inlet', e['d_in']), ('exhaust', e['d_ex'])):
        c = cam(Rb, Rr, h, dur)
        if c['phi'] > PHI_MAX:
            raise ValueError('The %s lobe (open %.0f crank deg) reaches a pressure angle of %.1f deg: above %.0f deg.'
                             % (name, dur, c['phi'], PHI_MAX))
        if c['rho'] <= Rr:
            raise ValueError('The %s lobe\'s pitch curve bends to %.2f mm, not more than the roller\'s %.1f mm: '
                             'undercut.' % (name, c['rho'], Rr))
    x_tdc = r + l                                                  # the piston pin at top dead centre
    top = x_tdc + 20.0                                             # the piston crown at TDC
    deck = top + 13.0                                              # the head's underside
    vz = [z0 + LOBE_T / 2 for z0 in LOBE_Z]                        # the valves' planes
    v_top = deck - 7.0                                             # a shut valve's head top: h + 1 mm below the head

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    blk = place(root, 'Block')
    tube = cyl((0, 50, 0), (0, deck, 0), B / 2 + 6)
    cut(tube, cyl((0, 49, 0), (0, deck, 0), B / 2))
    union(tube, box(-B / 2 - 10, CAM_X + 12, deck, deck + 10, -B / 2 - 10, vz[1] + 12))   # the head
    guide = box(B / 2 + 4, CAM_X + 12, 70, deck - 25, LOBE_Z[0] - 4, vz[1] + 12)          # the valve guides
    union(tube, guide)
    for z in vz:
        cut(tube, cyl((CAM_X, 60, z), (CAM_X, deck - 20, z), 4.2))
    add_body(blk.component, tube, 'Block')
    blk.isGrounded = True

    crank = place(root, 'Crankshaft')
    cs = cyl((0, 0, -48), (0, 0, -12), 10)
    union(cs, cyl((0, 0, -12), (0, 0, -6), 16), cyl((0, r, -12), (0, r, -6), 13), box(-13, 13, 0, r, -12, -6))
    union(cs, cyl((0, r, -6.001), (0, r, 6), 8))
    add_body(crank.component, cs, 'Crankshaft')

    cg = place(root, 'Crank gear', 0, 0, -40)
    gear_body(cg.component, Z_CRANK, 0.0, 20.4, 'Crank gear')     # tooth 0 towards the cam gear (+x)

    rod = place(root, 'Connecting rod')
    rb_ = cyl((0, r, -5), (0, r, 5), 15)
    union(rb_, cyl((0, x_tdc, -5), (0, x_tdc, 5), 9), box(-5, 5, r, x_tdc, -5, 5))
    cut(rb_, cyl((0, r, -6), (0, r, 6), 8.2), cyl((0, x_tdc, -6), (0, x_tdc, 6), 5.2))
    add_body(rod.component, rb_, 'Connecting rod')

    pis = place(root, 'Piston')
    pb = cyl((0, x_tdc - 25, 0), (0, top, 0), B / 2 - 0.2)
    cut(pb, box(-16, 16, x_tdc - 26, x_tdc + 11, -6, 6))
    union(pb, cyl((0, x_tdc, -(B / 2 - 2)), (0, x_tdc, B / 2 - 2), 5))
    add_body(pis.component, pb, 'Piston')

    cam_occ = place(root, 'Camshaft', CAM_X, 0, 0)
    k = cam_occ.component
    shaft = add_body(k, cyl((0, 0, -45), (0, 0, vz[1] + 8), 6), 'Camshaft')
    JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
    for z0, which in zip(LOBE_Z, ('in', 'ex')):
        sk = k.sketches.add(plane_at(k, z0))
        polyline(sk, [(x * CM, y * CM) for (x, y) in lobe_outline(e, h, Rb, Rr, which)])
        extrude(k, sk.profiles.item(0), z0, LOBE_T, JOIN, shaft)

    camg = place(root, 'Cam gear', CAM_X, 0, -40)
    gear_body(camg.component, Z_CAM, math.pi - math.pi / Z_CAM, 12.4, 'Cam gear')   # a gap meets the crank gear's tooth 0

    valves = []
    for z, z0, name in zip(vz, LOBE_Z, ('Inlet valve', 'Exhaust valve')):
        v = place(root, name)
        Rp = Rb + Rr
        vb = cyl((CAM_X, Rp, z0), (CAM_X, Rp, z0 + LOBE_T), Rr)                        # the roller
        union(vb, cyl((CAM_X, Rp, z), (CAM_X, v_top - 3, z), 4.0))                       # the stem
        union(vb, cyl((CAM_X, v_top - 3, z), (CAM_X, v_top, z), 9.0))                    # the head
        add_body(v.component, vb, name)
        valves.append(v)

    jc = joint(root, crank, blk, 0, 0, 0, 'revolute', 'Crank turns')
    joint(root, rod, crank, 0, r, 0, 'revolute', 'Rod on crankpin')
    joint(root, pis, rod, 0, x_tdc, 0, 'revolute', 'Piston on pin')
    joint(root, pis, blk, 0, x_tdc, 0, 'slider', 'Piston slides')                   # closes the loop
    joint(root, cg, crank, 0, 0, -40, 'rigid', 'Crank gear on crankshaft')
    jk = joint(root, cam_occ, blk, CAM_X, 0, 0, 'revolute', 'Camshaft turns')
    joint(root, camg, cam_occ, CAM_X, 0, -40, 'rigid', 'Cam gear on camshaft')
    for v, z, name in zip(valves, vz, ('Inlet valve slides', 'Exhaust valve slides')):
        joint(root, v, blk, CAM_X, Rb + Rr, z, 'slider', name)
    sc, sk_ = turn_sign(crank, jc), turn_sign(cam_occ, jk)
    li = root.motionLinks.createInput(jc, jk)
    li.valueOne = VS('720 deg')                                    # two turns of the crank ...
    li.valueTwo = VS('360 deg')                                    # ... for one of the camshaft
    li.isReversed = sc * sk_ > 0                                   # the camshaft turns the other way
    root.motionLinks.add(li).name = 'Timing gears 20:40'

    colour(design, blk, 'Paint - Enamel Glossy (Grey)')
    colour(design, pis, 'Paint - Enamel Glossy (Blue)')
    colour(design, rod, 'Paint - Enamel Glossy (Yellow)')
    for v in valves:
        colour(design, v, 'Paint - Enamel Glossy (Red)')
    app.activeViewport.fit()

    return ('Single-cylinder engine built: bore %.1f mm, stroke %.1f mm, rod %.1f mm (lambda = r / l = %.4f), '
            'swept volume %.2f cm3.\n\n'
            'Valves, crank degrees from TDC at the start of the inlet stroke: inlet %.0f to %.0f (%.0f deg), '
            'exhaust %.0f to %.0f (%.0f deg), overlap %.0f deg; lift %.1f mm.\n'
            'Lobes: base circle %.1f mm, roller %.1f mm; largest pressure angle %.2f deg (inlet), %.2f deg (exhaust).\n\n'
            'Built at top dead centre of compression, both valves shut. Drive Joints on "Crank turns" moves the '
            'piston and the camshaft; set each valve with its slider: Fusion cannot link a cam.\n\n[%s]'
            % (B, S, l, e['lam'], e['Vd'] / 1000.0, e['ev']['io'], e['ev']['ic'], e['d_in'], e['ev']['eo'],
               e['ev']['ec'], e['d_ex'], e['overlap'], h, Rb, Rr, cam(Rb, Rr, h, e['d_in'])['phi'],
               cam(Rb, Rr, h, e['d_ex'])['phi'], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('B', 'Bore', 'mm', 60.0), ('S', 'Stroke', 'mm', 54.0), ('l', 'Rod length', 'mm', 90.0),
          ('h', 'Valve lift', 'mm', 6.0), ('Rb', 'Cam base circle radius', 'mm', 15.0),
          ('Rr', 'Roller radius', 'mm', 5.0), ('IVO', 'Inlet opens before TDC', 'deg', 10.0),
          ('IVC', 'Inlet closes after BDC', 'deg', 50.0), ('EVO', 'Exhaust opens before BDC', 'deg', 50.0),
          ('EVC', 'Exhaust closes after TDC', 'deg', 10.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = {}
            for key, _, unit, _ in FIELDS:
                v = ins.itemById(key).value
                p[key] = v / CM if unit == 'mm' else math.degrees(v)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, unit, d in FIELDS:
            ins.addValueInput(key, label, unit, VI(d * CM if unit == 'mm' else math.radians(d)))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'SingleCylinderEngineCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Single-cylinder engine', 'A four-stroke side-valve engine')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Single-cylinder engine could not start:\n' + traceback.format_exc())
