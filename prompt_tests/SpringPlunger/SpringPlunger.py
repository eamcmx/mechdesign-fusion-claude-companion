# =============================================================================
# SpringPlunger.py  -  a spring plunger: a sleeve that slides down a rod against a helical compression spring and is
# held up by a DIN 471 retaining ring, the rod fixed in a base block by a cross pin. The script sizes the spring
# (music wire ASTM A228, plain and ground ends, Shigley Chapter 10), refuses a design outside the hard constraints,
# builds the six components with As-built joints and reports the spring's rate, forces, stresses and safety factors.
#
#   build(...)   the whole job from the inputs (mm); raises ValueError when a constraint refuses the design
#   spring(...)  the numbers only, as a dict
#   run()        the command dialog
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

VERSION = 'SpringPlunger v1 (prompt test)'
CMD_ID = 'mdfcSpringPlungerPromptTest'
CM = 0.1                                   # the API works in cm: every mm is multiplied by this
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

_handlers = []


def _companion():
    """Walk up from this script's folder to the first one holding common/companion_paths.py and import it, so that
    every lib/ of the companion is on sys.path. This script needs no library; outside the companion it returns None."""
    try:
        d = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return None
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            common = os.path.join(d, 'common')
            if common not in sys.path:
                sys.path.insert(0, common)
            import companion_paths
            return companion_paths
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


CP = _companion()

# ---------- data ----------
A_MW, M_MW = 2211.0, 0.145                 # music wire A228: S_ut = A / d^m, MPa and mm (Shigley Table 10-4)
WIRE_MIN, WIRE_MAX = 0.84, 3.175           # the music wire rows used, mm

# DIN 471 rings for a rod d_r, mm: ring thickness s, lug width a, back width b, lug hole d5, groove diameter d2,
# groove width m
RING = {6: (0.7, 2.7, 1.3, 1.2, 5.7, 0.8),
        8: (0.8, 3.2, 1.5, 1.2, 7.6, 0.9),
        10: (1.0, 3.3, 1.8, 1.5, 9.6, 1.1),
        12: (1.0, 3.3, 1.8, 1.7, 11.5, 1.1)}

SLEEVE_H = 8.0                             # the sleeve's height, mm
ROD_ABOVE_GROOVE = 2.0                     # the rod's top above the groove's upper flank, mm
RING_GAP_DEG = 30.0                        # the ring's gap, centred on +x
LUG_DEG = 30.0                             # each lug's angular width, beside the gap


# ---------- the spring ----------
def spring(d, D, Nt, L0, L1, st, dp):
    """The spring's numbers (mm, N, MPa) from wire d, mean coil D, total coils Nt, free length L0, installed length L1,
    stroke st and cross pin dp."""
    G = 81700.0 if d <= 1.60 else 81000.0          # MPa (Table 10-5)
    Sut = A_MW / d ** M_MW
    Ssy = 0.45 * Sut                               # Table 10-6
    C = D / d
    KB = (4.0 * C + 2.0) / (4.0 * C - 3.0)
    Na = Nt - 1                                    # plain and ground ends (Table 10-1)
    if Na < 1:
        raise ValueError('N_a = N_t - 1 = %d is outside Shigley\'s recommended 3 .. 15' % Na)
    Ls = d * Nt
    k = d ** 4 * G / (8.0 * D ** 3 * Na)
    L2 = L1 - st
    F1 = k * (L0 - L1)
    F2 = k * (L0 - L2)
    Fs = k * (L0 - Ls)

    def tau(F):
        return KB * 8.0 * F * D / (math.pi * d ** 3)

    tau1, tau2, taus = tau(F1), tau(F2), tau(Fs)
    n2 = Ssy / tau2 if tau2 > 0 else float('inf')
    ns = Ssy / taus if taus > 0 else float('inf')
    xi = Fs / F2 - 1.0 if F2 > 0 else float('-inf')
    taup = F2 / (2.0 * math.pi * dp ** 2 / 4.0)
    return dict(G=G, Sut=Sut, Ssy=Ssy, C=C, KB=KB, Na=Na, Ls=Ls, k=k, L2=L2, F1=F1, F2=F2, Fs=Fs,
                tau1=tau1, tau2=tau2, taus=taus, n2=n2, ns=ns, xi=xi, taup=taup)


def refuse(dr, d, D, Nt, L0, L1, stroke, dp, s):
    """Raises ValueError naming the first hard constraint the design breaks."""
    if not 4.0 <= s['C'] <= 12.0:
        raise ValueError('C = D / d = %.1f is outside Shigley\'s recommended 4 .. 12' % s['C'])
    if not 3 <= s['Na'] <= 15:
        raise ValueError('N_a = N_t - 1 = %d is outside Shigley\'s recommended 3 .. 15' % s['Na'])
    if not WIRE_MIN <= d <= WIRE_MAX:
        raise ValueError('wire diameter d = %.3f mm is outside the music wire rows used, %.2f .. %.3f mm'
                         % (d, WIRE_MIN, WIRE_MAX))
    if not L1 < L0:
        raise ValueError('installed length L1 = %.2f mm is not less than the free length L0 = %.2f mm: the spring '
                         'would not be preloaded' % (L1, L0))
    if not s['xi'] >= 0.15:
        raise ValueError('xi = F_s / F2 - 1 = %.3f is below 0.15: the spring is not robustly linear over the stroke'
                         % s['xi'])
    if not s['ns'] >= 1.2:
        raise ValueError('n_s = S_sy / tau(F_s) = %.3f is below 1.2: the spring is not solid-safe' % s['ns'])
    if not (D - d) - dr >= 1.0:
        raise ValueError('the spring\'s inside diameter D - d = %.2f mm does not clear the rod d_r = %d mm by 1 mm'
                         % (D - d, dr))
    if not dp <= 0.4 * dr:
        raise ValueError('cross pin d_p = %.2f mm is more than 0.4 d_r = %.2f mm' % (dp, 0.4 * dr))


# ---------- small geometry helpers (world mm in, cm to the API) ----------
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def _cyl(p0, p1, r):
    return _tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def _box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    obb = adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0),
                                                 (x1 - x0) * CM, (y1 - y0) * CM, (z1 - z0) * CM)
    return _tbm().createBox(obb)


def _cut(a, *tools):
    for b in tools:
        if not _tbm().booleanOperation(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def _add_body(comp, tb, name):
    """A temporary body into the component (its occurrence sits at the origin, so world = component frame)."""
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def _new_occ(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def _offset_plane(comp, z):
    if abs(z) < 1e-12:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z * CM))
    return comp.constructionPlanes.add(ci)


def _extrude_z(comp, sk, prof, dz, op, body=None):
    """Extrudes a profile by dz mm along world z (negative = down), whichever way the sketch's normal points."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    n = sk.xDirection.crossProduct(sk.yDirection)
    inp.setDistanceExtent(False, VI(dz * CM * (1.0 if n.z > 0 else -1.0)))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def _tick():
    app = adsk.core.Application.get()
    try:
        app.activeViewport.refresh()
        adsk.doEvents()
    except Exception:
        pass


# ---------- the parts ----------
def _spring_body(comp, d, D, Nt, L1):
    """A circle of diameter d swept along a 3D spline through the helix (24 points per turn, Nt turns from z = 0 to
    L1), then ground: cut flat at z = 0 and z = L1."""
    R = D / 2.0
    n = 24 * int(Nt)
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Helix'
    pts = adsk.core.ObjectCollection.create()
    for i in range(n + 1):
        th = 2.0 * math.pi * i / 24.0
        pts.add(sk.modelToSketchSpace(P3(R * math.cos(th) * CM, R * math.sin(th) * CM, L1 * i / n * CM)))
    helix = sk.sketchCurves.sketchFittedSplines.add(pts)

    start = P3(R * CM, 0.0, 0.0)
    try:                                           # the wire's section square to the helix at its start
        ci = comp.constructionPlanes.createInput()
        ci.setByDistanceOnPath(helix, VI(0.0))
        plane = comp.constructionPlanes.add(ci)
    except Exception:                              # nearly square to it: the helix starts along +y
        plane = comp.xZConstructionPlane
    sk2 = comp.sketches.add(plane)
    sk2.name = 'Wire'
    c = sk2.modelToSketchSpace(start)
    c.z = 0.0
    sk2.sketchCurves.sketchCircles.addByCenterRadius(c, d / 2.0 * CM)

    sweeps = comp.features.sweepFeatures
    si = sweeps.createInput(sk2.profiles.item(0), comp.features.createPath(helix, False),
                            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    sweeps.add(si)

    h = R + d + 2.0                                # the cutting squares, larger than the coil
    cut = adsk.fusion.FeatureOperations.CutFeatureOperation
    for z, dz, name in ((0.0, -(d + 2.0), 'Grind bottom'), (L1, d + 2.0, 'Grind top')):
        skc = comp.sketches.add(_offset_plane(comp, z))
        skc.name = name
        skc.sketchCurves.sketchLines.addTwoPointRectangle(skc.modelToSketchSpace(P3(-h * CM, -h * CM, z * CM)),
                                                          skc.modelToSketchSpace(P3(h * CM, h * CM, z * CM)))
        _extrude_z(comp, skc, skc.profiles.item(0), dz, cut, comp.bRepBodies.item(0))
    body = comp.bRepBodies.item(0)
    body.name = 'Spring'
    return body


def _ring_body(comp, z0, ri, s, a, b, d5):
    """The DIN 471 ring on the plane z = z0, s thick: inside radius ri, a 30 degree gap centred on +x, two lugs of
    radial width a and 30 degrees each beside it with a hole d5 in each, the back of radial width b. True arcs."""
    sk = comp.sketches.add(_offset_plane(comp, z0))
    sk.name = 'Ring'
    ra, rb = ri + a, ri + b
    g = RING_GAP_DEG / 2.0
    lg = g + LUG_DEG

    def sp(r, deg):
        t = math.radians(deg)
        q = sk.modelToSketchSpace(P3(r * math.cos(t) * CM, r * math.sin(t) * CM, z0 * CM))
        q.z = 0.0
        return q

    arcs = sk.sketchCurves.sketchArcs
    lines = sk.sketchCurves.sketchLines
    inner = arcs.addByThreePoints(sp(ri, g), sp(ri, 180.0), sp(ri, 360.0 - g))
    lug2 = arcs.addByThreePoints(sp(ra, 360.0 - g), sp(ra, 360.0 - g - LUG_DEG / 2.0), sp(ra, 360.0 - lg))
    back = arcs.addByThreePoints(sp(rb, 360.0 - lg), sp(rb, 180.0), sp(rb, lg))
    lug1 = arcs.addByThreePoints(sp(ra, lg), sp(ra, g + LUG_DEG / 2.0), sp(ra, g))

    def end(arc, r, deg):                          # the arc's own end point at that place, so the loop is shared
        t = sp(r, deg)
        p, q = arc.startSketchPoint, arc.endSketchPoint
        return p if p.geometry.distanceTo(t) < q.geometry.distanceTo(t) else q

    lines.addByTwoPoints(end(inner, ri, 360.0 - g), end(lug2, ra, 360.0 - g))
    lines.addByTwoPoints(end(lug2, ra, 360.0 - lg), end(back, rb, 360.0 - lg))
    lines.addByTwoPoints(end(back, rb, lg), end(lug1, ra, lg))
    lines.addByTwoPoints(end(lug1, ra, g), end(inner, ri, g))
    for deg in (g + LUG_DEG / 2.0, 360.0 - g - LUG_DEG / 2.0):
        sk.sketchCurves.sketchCircles.addByCenterRadius(sp(ri + a / 2.0, deg), d5 / 2.0 * CM)

    acc = adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy
    prof = max(list(sk.profiles), key=lambda p: p.areaProperties(acc).area)     # the ring less its two holes
    feat = _extrude_z(comp, sk, prof, s, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = feat.bodies.item(0)
    body.name = 'Retaining ring'
    return body


def _origin_geo(occ):
    """Joint geometry at the occurrence's origin, its axes those of the component (z up), in the assembly's context."""
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Joint origin'
    pt = sk.sketchPoints.add(P3(0.0, 0.0, 0.0))
    sk.isVisible = False
    try:
        pt = pt.createForAssemblyContext(occ)
    except Exception:
        pass
    return adsk.fusion.JointGeometry.createByPoint(pt)


def _rigid(root, one, two, name):
    ab = root.asBuiltJoints
    inp = ab.createInput(one, two, _origin_geo(one))
    inp.setAsRigidJointMotion()
    j = ab.add(inp)
    j.name = name
    return j


# ---------- the whole job ----------
def build(dr=8, d=1.5, D=12.0, Nt=10, L0=36.0, L1=30.0, stroke=8.0, dp=3.0):
    """Builds the plunger in a new design (mm in). Returns the message; raises ValueError if refused."""
    dr = int(round(dr))
    Nt = int(round(Nt))
    if dr not in RING:
        raise ValueError('rod diameter d_r = %d mm: one of 6, 8, 10, 12 mm (the DIN 471 rows given)' % dr)
    for name, v in (('d', d), ('D', D), ('L0', L0), ('L1', L1), ('stroke', stroke), ('d_p', dp)):
        if not v > 0:
            raise ValueError('%s = %g mm must be positive' % (name, v))
    s = spring(d, D, Nt, L0, L1, stroke, dp)
    refuse(dr, d, D, Nt, L0, L1, stroke, dp, s)

    rs, ra, rb, d5, d2, rm = RING[dr]
    S = max(30.0, D + d + 12.0)                   # base side
    H = 1.5 * dr + 4.0                             # base height, top face at z = 0
    bd = 1.5 * dr                                  # rod bore depth
    zp = -bd / 2.0                                 # cross pin axis
    z_sl0, z_sl1 = L1, L1 + SLEEVE_H               # sleeve
    z_r0, z_r1 = z_sl1, z_sl1 + rs                 # ring, on top of the sleeve
    z_g0, z_g1 = z_r1 - rm, z_r1                   # groove: upper flank level with the ring's top face
    z_top = z_g1 + ROD_ABOVE_GROOVE                # rod top
    sl_od, sl_id = D + d + 4.0, dr + 0.2

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = des.rootComponent

    # Base: grounded, top face at z = 0, a blind bore for the rod and a hole for the cross pin
    o_base = _new_occ(root, 'Base')
    tb = _box(-S / 2, S / 2, -S / 2, S / 2, -H, 0.0)
    _cut(tb, _cyl((0, 0, -bd), (0, 0, 1.0), dr / 2.0),
         _cyl((-S / 2 - 1.0, 0, zp), (S / 2 + 1.0, 0, zp), dp / 2.0))
    _add_body(o_base.component, tb, 'Base')
    o_base.isGrounded = True
    _tick()

    # Rod: from the bottom of the bore to 2 mm above the groove; the groove d2 x m and the cross pin hole
    o_rod = _new_occ(root, 'Rod')
    tb = _cyl((0, 0, -bd), (0, 0, z_top), dr / 2.0)
    groove = _cut(_cyl((0, 0, z_g0), (0, 0, z_g1), dr / 2.0 + 1.0),
                  _cyl((0, 0, z_g0 - 1.0), (0, 0, z_g1 + 1.0), d2 / 2.0))
    _cut(tb, groove, _cyl((-dr, 0, zp), (dr, 0, zp), dp / 2.0))
    _add_body(o_rod.component, tb, 'Rod')
    _tick()

    # Pin: along x through base and rod at half the bore depth, as long as the base is wide
    o_pin = _new_occ(root, 'Pin')
    _add_body(o_pin.component, _cyl((-S / 2, 0, zp), (S / 2, 0, zp), dp / 2.0), 'Pin')
    _tick()

    # Spring: standing on the base, drawn at its installed length
    o_spr = _new_occ(root, 'Spring')
    _spring_body(o_spr.component, d, D, Nt, L1)
    _tick()

    # Sleeve: on the spring, from z = L1 to L1 + 8
    o_sl = _new_occ(root, 'Sleeve')
    tb = _cut(_cyl((0, 0, z_sl0), (0, 0, z_sl1), sl_od / 2.0),
              _cyl((0, 0, z_sl0 - 1.0), (0, 0, z_sl1 + 1.0), sl_id / 2.0))
    _add_body(o_sl.component, tb, 'Sleeve')
    _tick()

    # Retaining ring: on top of the sleeve, in the groove
    o_ring = _new_occ(root, 'Retaining ring')
    _ring_body(o_ring.component, z_r0, d2 / 2.0, rs, ra, rb, d5)
    _tick()

    # Joints, As-built only
    _rigid(root, o_rod, o_base, 'Rod in base')
    _rigid(root, o_pin, o_base, 'Pin in base')
    _rigid(root, o_spr, o_base, 'Spring on base')
    _rigid(root, o_ring, o_rod, 'Ring on rod')
    ab = root.asBuiltJoints
    inp = ab.createInput(o_sl, o_rod, _origin_geo(o_sl))
    inp.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = ab.add(inp)
    j.name = 'Sleeve slides'
    lim = j.jointMotion.slideLimits
    lim.minimumValue = -stroke * CM
    lim.maximumValue = 0.0
    lim.isMinimumValueEnabled = True
    lim.isMaximumValueEnabled = True

    try:
        app.activeViewport.fit()
    except Exception:
        pass

    return ('Spring plunger built  [%s]\n\n'
            'Spring, music wire ASTM A228, plain and ground ends: d = %.2f mm, D = %.2f mm, N_t = %d, '
            'L0 = %.2f mm, L1 = %.2f mm, stroke %.2f mm\n'
            'C = %.3f, K_B = %.5f, N_a = %d, L_s = %.2f mm\n'
            'k = %.5f N/mm (G = %.1f GPa)\n'
            'F1 = %.2f N, F2 = %.2f N (at L2 = %.2f mm), F_s = %.2f N\n'
            'tau at F1 = %.1f MPa, at F2 = %.1f MPa, at solid = %.1f MPa\n'
            'S_ut = %.1f MPa, S_sy = %.1f MPa\n'
            'n2 = %.3f, n_s = %.3f, xi = %.3f\n'
            'Cross pin d_p = %.2f mm in double shear: tau_p = %.2f MPa\n'
            'Retaining ring DIN 471-%d: s = %.1f mm, groove d2 = %.1f mm x m = %.1f mm\n'
            'Base %.1f x %.1f x %.1f mm, sleeve OD %.1f mm, bore %.1f mm'
            % (VERSION, d, D, Nt, L0, L1, stroke,
               s['C'], s['KB'], s['Na'], s['Ls'], s['k'], s['G'] / 1000.0,
               s['F1'], s['F2'], s['L2'], s['Fs'],
               s['tau1'], s['tau2'], s['taus'], s['Sut'], s['Ssy'],
               s['n2'], s['ns'], s['xi'], dp, s['taup'],
               dr, rs, d2, rm, S, S, H, sl_od, sl_id))


# ---------- the command dialog ----------
class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('dr', 'Rod diameter d_r (mm)',
                                         adsk.core.DropDownStyles.TextListDropDownStyle)
        for v in (6, 8, 10, 12):
            dd.listItems.add(str(v), v == 8, '')
        ins.addValueInput('d', 'Spring wire diameter d', 'mm', VI(1.5 * CM))
        ins.addValueInput('D', 'Mean coil diameter D', 'mm', VI(12.0 * CM))
        ins.addIntegerSpinnerCommandInput('Nt', 'Total coils N_t', 2, 60, 1, 10)
        ins.addValueInput('L0', 'Free length L0', 'mm', VI(36.0 * CM))
        ins.addValueInput('L1', 'Installed length L1', 'mm', VI(30.0 * CM))
        ins.addValueInput('stroke', 'Stroke of the sleeve', 'mm', VI(8.0 * CM))
        ins.addValueInput('dp', 'Cross pin diameter d_p', 'mm', VI(3.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs

            def mm(i):
                return round(ins.itemById(i).value / CM, 9)

            msg = build(dr=int(ins.itemById('dr').selectedItem.name), d=mm('d'), D=mm('D'),
                        Nt=ins.itemById('Nt').value, L0=mm('L0'), L1=mm('L1'), stroke=mm('stroke'), dp=mm('dp'))
            ui.messageBox(msg)
        except ValueError as e:
            ui.messageBox('Spring plunger refused  [%s]\n\n%s' % (VERSION, e))
        except Exception:
            ui.messageBox('Spring plunger failed  [%s]\n\n%s' % (VERSION, traceback.format_exc()))


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Spring plunger',
                                                       'Builds a spring plunger and sizes its spring')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Spring plunger failed  [%s]\n\n%s' % (VERSION, traceback.format_exc()))


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
