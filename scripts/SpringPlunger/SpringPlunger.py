# =============================================================================
# SpringPlunger.py  -  Chapter 9 of "Mechanical Design in Autodesk Fusion with Claude": a sleeve sliding down a rod
# against a helical compression spring, held up by a DIN 471 retaining ring, the rod fixed in a base block by a
# cross pin; the spring's rate, forces, stresses and safety factors. Written from PROMPT.txt beside it; checked by
# checks/spring_plunger_check.py.
#
#   INPUTS        the dialog: d_r, d, D, N_t, L0, L1, stroke, d_p
#   DERIVED       spring(): G, S_ut, S_sy, C, K_B, N_a, L_s, k, F1, F2, F_s, the stresses, n2, n_s, xi, tau_p
#   HARD RULES    build() refuses C or N_a out of range, a wire outside the table, no preload, xi < 0.15,
#                 n_s < 1.2, a spring that does not clear the rod, a pin too large for the rod
#   CONVENTIONS   a new design; every part created where it belongs; the spring swept along a helical spline and
#                 ground flat; the ring drawn with true arcs; cm inside the API; As-built joints
#   ACCEPTANCE    the message with the defaults; six components, no interference; the sleeve's travel; refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'SpringPlunger 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
A_MW, M_MW = 2211.0, 0.145                 # music wire A228: S_ut = A / d^m, MPa and mm (Shigley Table 10-4)
SLEEVE_H = 8.0                             # mm
GAP, LUG = math.radians(30), math.radians(30)

# DIN 471, rod diameter: (ring thickness s, lug width a, back width b, lug hole d5, groove diameter d2, groove width m)
DIN471 = {6: (0.7, 2.7, 1.3, 1.2, 5.7, 0.8), 8: (0.8, 3.2, 1.5, 1.2, 7.6, 0.9),
          10: (1.0, 3.3, 1.8, 1.5, 9.6, 1.1), 12: (1.0, 3.3, 1.8, 1.7, 11.5, 1.1)}


def spring(d, D, Nt, L0, L1, stroke, dp):
    G = 81700.0 if d <= 1.60 else 81000.0               # MPa, music wire (Shigley Table 10-5)
    Sut = A_MW / d ** M_MW
    Ssy = 0.45 * Sut
    C = D / d
    KB = (4 * C + 2) / (4 * C - 3)
    Na = Nt - 1                                         # plain and ground ends
    Ls = d * Nt
    k = d ** 4 * G / (8 * D ** 3 * Na)
    L2 = L1 - stroke
    F1, F2, Fs = k * (L0 - L1), k * (L0 - L2), k * (L0 - Ls)
    tau = lambda F: KB * 8 * F * D / (math.pi * d ** 3)
    return dict(G=G, Sut=Sut, Ssy=Ssy, C=C, KB=KB, Na=Na, Ls=Ls, k=k, L2=L2, F1=F1, F2=F2, Fs=Fs,
                tau1=tau(F1), tau2=tau(F2), taus=tau(Fs), n2=Ssy / tau(F2), ns=Ssy / tau(Fs),
                xi=Fs / F2 - 1 if F2 > 0 else float('inf'), taup=F2 / (2 * math.pi * dp ** 2 / 4))


def plane_at(comp, z_mm):
    if abs(z_mm) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z_mm * CM))
    return comp.constructionPlanes.add(ci)


def new_part(root, name):
    """A new component at the origin; its geometry is drawn where it belongs in the assembly."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def extrude(comp, profile, length_mm, op=NEW, body=None, down=False):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, op)
    side = adsk.fusion.ExtentDirections.NegativeExtentDirection if down else adsk.fusion.ExtentDirections.PositiveExtentDirection
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(length_mm * CM)), side)
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def cross(comp, z_mm, dia, length_mm, op=NEW, body=None):
    """A cylinder (or a hole) of diameter dia along x through the origin at height z, length_mm long."""
    sk = comp.sketches.add(comp.yZConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P3(0, 0, z_mm * CM)), dia / 2 * CM)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), op)
    inp.setSymmetricExtent(VI(length_mm * CM), True)
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def annulus(sk, r_out, r_in):
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), r_out * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), r_in * CM)
    return [p for p in sk.profiles if p.profileLoops.count == 2][0]


def ring_outline(sk, r_in, r_back, r_lug):
    """The retaining ring seen from above, gap centred on +x: lines and true arcs sharing their sketch points."""
    pt = lambda r, t: P3(r * math.cos(t) * CM, r * math.sin(t) * CM, 0)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs

    def arc(start, r, ta, tb, end=None):
        a = arcs.addByThreePoints(start, pt(r, (ta + tb) / 2), end if end is not None else pt(r, tb))
        s = start.geometry
        return a.endSketchPoint if a.startSketchPoint.geometry.distanceTo(s) < 1e-9 else a.startSketchPoint

    t1, t2 = GAP / 2, 2 * math.pi - GAP / 2
    first = lines.addByTwoPoints(pt(r_in, t1), pt(r_lug, t1))
    p = arc(first.endSketchPoint, r_lug, t1, t1 + LUG)
    p = lines.addByTwoPoints(p, pt(r_back, t1 + LUG)).endSketchPoint
    p = arc(p, r_back, t1 + LUG, t2 - LUG)
    p = lines.addByTwoPoints(p, pt(r_lug, t2 - LUG)).endSketchPoint
    p = arc(p, r_lug, t2 - LUG, t2)
    p = lines.addByTwoPoints(p, pt(r_in, t2)).endSketchPoint
    arc(p, r_in, t2, t1, first.startSketchPoint)                     # back round the inside, through 180 degrees


def helix_spring(comp, d, D, Nt, L1):
    """Fusion's API cannot create a coil: sweep the wire along a 3D spline through the helix, then grind the ends."""
    pts = adsk.core.ObjectCollection.create()
    n = 24 * Nt
    for i in range(n + 1):
        t = 2 * math.pi * i / 24
        pts.add(P3(D / 2 * math.cos(t) * CM, D / 2 * math.sin(t) * CM, L1 * i / n * CM))
    sk = comp.sketches.add(comp.xYConstructionPlane)
    spline = sk.sketchCurves.sketchFittedSplines.add(pts)
    ci = comp.constructionPlanes.createInput()
    ci.setByDistanceOnPath(spline, VI(0))
    wire = comp.sketches.add(comp.constructionPlanes.add(ci))
    start = wire.modelToSketchSpace(P3(D / 2 * CM, 0, 0))            # the helix starts at (D/2, 0, 0)
    wire.sketchCurves.sketchCircles.addByCenterRadius(start, d / 2 * CM)
    sw = comp.features.sweepFeatures
    inp = sw.createInput(wire.profiles.item(0), comp.features.createPath(spline, False), NEW)
    inp.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    body = sw.add(inp).bodies.item(0)
    for z, down in ((0.0, True), (L1, False)):                       # ground ends: flat at z = 0 and z = L1
        g = comp.sketches.add(plane_at(comp, z))
        g.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), D * CM)
        extrude(comp, g.profiles.item(0), d, CUT, body, down)
    return body


def as_built(root, moving, fixed, name, slider=False):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    if slider:
        inp.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


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


def build(dr=8, d=1.5, D=12.0, Nt=10, L0=36.0, L1=30.0, stroke=8.0, dp=3.0):
    dr, Nt = int(round(dr)), int(round(Nt))
    if dr not in DIN471:
        raise ValueError('Rod diameter %d mm: the DIN 471 table here has %s mm.' % (dr, ', '.join(map(str, DIN471))))
    if not 0.84 <= d <= 3.17:
        raise ValueError('Wire diameter %.2f mm is outside the music wire rows used (0.84 to 3.17 mm).' % d)
    s = spring(d, D, Nt, L0, L1, stroke, dp)
    if not 4 <= s['C'] <= 12:
        raise ValueError('Spring index C = D / d = %.1f: keep it between 4 and 12.' % s['C'])
    if not 3 <= s['Na'] <= 15:
        raise ValueError('N_a = %d active coils: keep it between 3 and 15.' % s['Na'])
    if L1 >= L0:
        raise ValueError('Installed length %.1f mm is not shorter than the free length %.1f mm: no preload.' % (L1, L0))
    if s['xi'] < 0.15:
        raise ValueError('Robust linearity: xi = F_s / F2 - 1 = %.3f, below 0.15. Shorten the stroke or add coils.' % s['xi'])
    if s['ns'] < 1.2:
        raise ValueError('Not solid-safe: n_s = %.3f, below 1.2.' % s['ns'])
    if D - d - dr < 1.0:
        raise ValueError('The spring\'s inside diameter %.1f mm does not clear the %d mm rod by 1 mm.' % (D - d, dr))
    if dp > 0.4 * dr:
        raise ValueError('Cross pin %.1f mm is larger than 0.4 d_r = %.1f mm.' % (dp, 0.4 * dr))
    rs, ra, rb, d5, d2, m = DIN471[dr]
    B = max(30.0, D + d + 12.0)                    # base block side
    e = 1.5 * dr                                   # bore depth
    H = e + 4.0                                    # base height
    z_pin = -e / 2.0
    z_sleeve_top = L1 + SLEEVE_H
    z_groove_top = z_sleeve_top + rs               # the ring's top face is the groove's upper flank
    z_rod_top = z_groove_top + 2.0

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    base = new_part(root, 'Base')
    c = base.component
    sk = c.sketches.add(plane_at(c, -H))
    sk.sketchCurves.sketchLines.addCenterPointRectangle(P3(0, 0, 0), P3(B / 2 * CM, B / 2 * CM, 0))
    body = extrude(c, sk.profiles.item(0), H).bodies.item(0)
    body.name = 'Base'
    sk = c.sketches.add(c.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), dr / 2 * CM)
    extrude(c, sk.profiles.item(0), e, CUT, body, down=True)                      # the rod's bore
    cross(c, z_pin, dp, B + 2.0, CUT, body)                                        # the pin's hole
    base.isGrounded = True

    rod = new_part(root, 'Rod')
    c = rod.component
    sk = c.sketches.add(plane_at(c, -e))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), dr / 2 * CM)
    body = extrude(c, sk.profiles.item(0), z_rod_top + e).bodies.item(0)
    body.name = 'Rod'
    cross(c, z_pin, dp, dr + 2.0, CUT, body)
    sk = c.sketches.add(plane_at(c, z_groove_top - m))
    extrude(c, annulus(sk, dr / 2 + 1.0, d2 / 2), m, CUT, body)                    # the groove

    pin = new_part(root, 'Pin')
    cross(pin.component, z_pin, dp, B).bodies.item(0).name = 'Pin'

    spr = new_part(root, 'Spring')
    helix_spring(spr.component, d, D, Nt, L1).name = 'Spring'

    sleeve = new_part(root, 'Sleeve')
    c = sleeve.component
    sk = c.sketches.add(plane_at(c, L1))
    extrude(c, annulus(sk, (D + d + 4.0) / 2, (dr + 0.2) / 2), SLEEVE_H).bodies.item(0).name = 'Sleeve'

    ring = new_part(root, 'Retaining ring')
    c = ring.component
    sk = c.sketches.add(plane_at(c, z_sleeve_top))
    ring_outline(sk, d2 / 2, d2 / 2 + rb, d2 / 2 + ra)
    for t in (GAP / 2 + LUG / 2, -GAP / 2 - LUG / 2):
        r = d2 / 2 + ra / 2
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(r * math.cos(t) * CM, r * math.sin(t) * CM, 0), d5 / 2 * CM)
    prof = [p for p in sk.profiles if p.profileLoops.count == 3][0]
    extrude(c, prof, rs).bodies.item(0).name = 'Retaining ring'

    as_built(root, rod, base, 'Rod in base')
    as_built(root, pin, base, 'Pin in base')
    as_built(root, spr, base, 'Spring on base')
    as_built(root, ring, rod, 'Ring in groove')
    j = as_built(root, sleeve, rod, 'Sleeve slides', slider=True)
    lim = j.jointMotion.slideLimits
    lim.isMaximumValueEnabled = True
    lim.maximumValue = 0.0
    lim.isMinimumValueEnabled = True
    lim.minimumValue = -stroke * CM
    for occ, ap in ((base, 'Paint - Enamel Glossy (Grey)'), (rod, 'Steel - Satin'), (pin, 'Steel - Satin'),
                    (spr, 'Paint - Enamel Glossy (Yellow)'), (sleeve, 'Paint - Enamel Glossy (Blue)'),
                    (ring, 'Paint - Enamel Glossy (Black)')):
        colour(design, occ, ap)
    app.activeViewport.fit()

    return ('Spring plunger built: rod %d mm, DIN 471 ring %d x %.1f, cross pin %.1f mm.\n\n'
            'Spring (music wire, plain and ground ends): d = %.2f mm, D = %.1f mm, N_t = %d\n'
            'C = %.3f, K_B = %.5f, N_a = %d, L_s = %.2f mm, k = %.5f N/mm\n'
            'F1 = %.2f N at L1 = %.1f mm, F2 = %.2f N at L2 = %.1f mm, F_s = %.2f N at solid\n'
            'tau(F2) = %.1f MPa, tau(F_s) = %.1f MPa, S_sy = %.1f MPa\n'
            'n2 = %.3f, n_s = %.3f, xi = %.3f\n'
            'Cross pin in double shear: tau_p = %.2f MPa\n\n[%s]'
            % (dr, dr, rs, dp, d, D, Nt, s['C'], s['KB'], s['Na'], s['Ls'], s['k'], s['F1'], L1, s['F2'], s['L2'],
               s['Fs'], s['tau2'], s['taus'], s['Ssy'], s['n2'], s['ns'], s['xi'], s['taup'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(dr=int(ins.itemById('dr').selectedItem.name), d=ins.itemById('d').value / CM,
                     D=ins.itemById('D').value / CM, Nt=ins.itemById('Nt').value, L0=ins.itemById('L0').value / CM,
                     L1=ins.itemById('L1').value / CM, stroke=ins.itemById('stroke').value / CM,
                     dp=ins.itemById('dp').value / CM)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('dr', 'Rod diameter (mm)', adsk.core.DropDownStyles.TextListDropDownStyle)
        for size in DIN471:
            dd.listItems.add(str(size), size == 8)
        ins.addValueInput('d', 'Spring wire diameter', 'mm', VI(1.5 * CM))
        ins.addValueInput('D', 'Mean coil diameter', 'mm', VI(12.0 * CM))
        ins.addIntegerSpinnerCommandInput('Nt', 'Total coils', 4, 30, 1, 10)
        ins.addValueInput('L0', 'Free length', 'mm', VI(36.0 * CM))
        ins.addValueInput('L1', 'Installed length', 'mm', VI(30.0 * CM))
        ins.addValueInput('stroke', 'Stroke of the sleeve', 'mm', VI(8.0 * CM))
        ins.addValueInput('dp', 'Cross pin diameter', 'mm', VI(3.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'SpringPlungerCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Spring plunger', 'A sleeve on a spring, a ring and a pin')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Spring plunger could not start:\n' + traceback.format_exc())
