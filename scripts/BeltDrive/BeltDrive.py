# =============================================================================
# BeltDrive.py  -  Chapter 19 of "Mechanical Design in Autodesk Fusion with Claude": a GT2 timing-belt drive. The
# centre distance is solved so that a belt of a whole number of teeth fits; the belt is drawn as the band it is at
# rest, round the pulleys' outside diameters. Written from PROMPT.txt beside it; checked by checks/belt_drive_check.py.
#
#   INPUTS        the dialog: z1, z2, N_b, width
#   DERIVED       belt(): PD, OD, C (bisection), phi, the wrap angles, the teeth in mesh, the ratio
#   HARD RULES    build() refuses a belt too short, pulleys that touch, fewer than 6 teeth in mesh
#   CONVENTIONS   a new design; parts created where they belong; the belt two loops of arcs and tangent lines with
#                 shared sketch points; cm inside the API; As-built Joints; a Motion Link that keeps the direction
#   ACCEPTANCE    the message; the belt's volume; pulley 2 +15 deg per 30 deg step; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'BeltDrive 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
PITCH = 2.0                                # mm, GT2
PLD = 0.254                                # mm, pitch line to the pulley's outside diameter, each side
BACK = 0.63                                # mm, the belt's back outside the pulleys
DS, GAP = 5.0, 1.0                         # mm, shaft diameter and the collar under each pulley
GROOVE_R = 0.6                             # mm, the grooves cut in each pulley


def pitch_length(C, a, b):
    """Pitch length of an open belt round pulleys of pitch radii a <= b, centres C apart."""
    phi = math.asin((b - a) / C)
    return 2 * C * math.cos(phi) + math.pi * (a + b) + 2 * phi * (b - a)


def belt(z1, z2, n_b):
    pd1, pd2 = z1 * PITCH / math.pi, z2 * PITCH / math.pi
    a, b = min(pd1, pd2) / 2, max(pd1, pd2) / 2
    L = n_b * PITCH
    lo, hi = b - a + 1e-9, 10 * L
    if pitch_length(lo, a, b) > L:
        return dict(pd1=pd1, pd2=pd2, C=None)
    for _ in range(200):                                        # bisection: L grows with C
        mid = (lo + hi) / 2
        if pitch_length(mid, a, b) < L:
            lo = mid
        else:
            hi = mid
    C = (lo + hi) / 2
    phi = math.asin((pd2 - pd1) / 2 / C)                        # positive when pulley 2 is the larger
    w1, w2 = math.pi - 2 * phi, math.pi + 2 * phi
    return dict(pd1=pd1, pd2=pd2, od1=pd1 - 2 * PLD, od2=pd2 - 2 * PLD, C=C, phi=phi, wrap1=w1, wrap2=w2,
                mesh1=z1 * w1 / (2 * math.pi), ratio=z2 / z1)


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


def extrude(comp, profs, length_mm, op=NEW, body=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, VI(length_mm * CM))
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def belt_loop(sk, a, b, C, phi):
    """One loop round circles of radii a (at the origin) and b (at (C, 0)): the upper tangent, the arc round circle 2
    through angle 0, the lower tangent, the arc round circle 1 through angle pi. Lines and arcs share their points."""
    pt = lambda cx, r, t: P3((cx + r * math.cos(t)) * CM, r * math.sin(t) * CM, 0)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    psi = math.pi / 2 + phi                                      # the tangents' normals
    top = lines.addByTwoPoints(pt(0, a, psi), pt(C, b, psi))

    def arc(start, cx, r, mid, end):
        q = arcs.addByThreePoints(start, pt(cx, r, mid), end)
        return q.endSketchPoint if q.startSketchPoint.geometry.distanceTo(start.geometry) < 1e-9 else q.startSketchPoint

    p = arc(top.endSketchPoint, C, b, 0.0, pt(C, b, -psi))
    p = lines.addByTwoPoints(p, pt(0, a, -psi)).endSketchPoint
    arc(p, 0, a, math.pi, top.startSketchPoint)


def pulley(occ, z, od, width):
    c = occ.component
    sk = c.sketches.add(plane_at(c, GAP))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), od / 2 * CM)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), (DS + 0.2) / 2 * CM)
    body = extrude(c, [q for q in sk.profiles if q.profileLoops.count == 2][0], width + 2).bodies.item(0)
    body.name = c.name
    g = c.sketches.add(plane_at(c, GAP))
    for k in range(z):
        t = 2 * math.pi * k / z
        g.sketchCurves.sketchCircles.addByCenterRadius(P3(od / 2 * math.cos(t) * CM, od / 2 * math.sin(t) * CM, 0),
                                                       GROOVE_R * CM)
    col = adsk.core.ObjectCollection.create()
    for q in g.profiles:
        col.add(q)
    extrude(c, col, width + 2, CUT, body)
    return body


def shaft(occ, height):
    c = occ.component
    sk = c.sketches.add(c.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), 1.6 * DS / 2 * CM)
    body = extrude(c, sk.profiles.item(0), GAP).bodies.item(0)
    s2 = c.sketches.add(plane_at(c, GAP))
    s2.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), DS / 2 * CM)
    extrude(c, s2.profiles.item(0), height - GAP, JOIN, body)
    body.name = c.name


def as_built(root, moving, fixed, revolute, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    if revolute:
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


# ---------- the drive ----------
def build(z1=20, z2=40, n_b=90, width=6.0):
    z1, z2, n_b = int(round(z1)), int(round(z2)), int(round(n_b))
    k = belt(z1, z2, n_b)
    if k['C'] is None:
        raise ValueError('A belt of %d teeth (%.0f mm) is too short to go round these pulleys.' % (n_b, n_b * PITCH))
    C = k['C']
    if C <= (k['od1'] + k['od2']) / 2 + 2.0:
        raise ValueError('Centre distance %.2f mm: the pulleys would touch.' % C)
    if k['mesh1'] < 6.0:
        raise ValueError('Only %.2f teeth of pulley 1 in mesh: at least 6. Use a longer belt or closer ratios.' % k['mesh1'])

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    r_big = max(k['od1'], k['od2']) / 2 + BACK + 6
    plate = place(root, 'Base plate')
    pc = plate.component
    sk = pc.sketches.add(plane_at(pc, -6.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-r_big * CM, -r_big * CM, 0), P3((C + r_big) * CM, r_big * CM, 0))
    extrude(pc, sk.profiles.item(0), 6.0).bodies.item(0).name = 'Base plate'
    plate.isGrounded = True
    s1, s2 = place(root, 'Shaft 1'), place(root, 'Shaft 2', C, 0)
    for s in (s1, s2):
        shaft(s, GAP + width + 2 + 3)
    p1, p2 = place(root, 'Pulley 1'), place(root, 'Pulley 2', C, 0)
    pulley(p1, z1, k['od1'], width)
    pulley(p2, z2, k['od2'], width)

    bt = place(root, 'Belt')
    c = bt.component
    sk = c.sketches.add(plane_at(c, GAP + 1))
    a_in, b_in = k['od1'] / 2, k['od2'] / 2
    belt_loop(sk, a_in, b_in, C, k['phi'])                      # the inside, on the outside diameters
    belt_loop(sk, a_in + BACK, b_in + BACK, C, k['phi'])        # the back
    prof = [q for q in sk.profiles if q.profileLoops.count == 2][0]
    extrude(c, prof, width).bodies.item(0).name = 'Belt'

    for s in (s1, s2, bt):
        as_built(root, s, plate, False, '%s fixed' % s.component.name)
    j1 = as_built(root, p1, s1, True, 'Pulley 1 turns')
    j2 = as_built(root, p2, s2, True, 'Pulley 2 turns')
    g = math.gcd(z1, z2)
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % (360 * z2 // g))                  # z2 / z1 turns of pulley 1 ...
    li.valueTwo = VS('%d deg' % (360 * z1 // g))                  # ... for one turn of pulley 2
    li.isReversed = False                                         # a belt keeps the direction
    root.motionLinks.add(li).name = 'Belt %d:%d' % (z1, z2)
    colour(design, plate, 'Paint - Enamel Glossy (Grey)')
    colour(design, p1, 'Aluminum - Satin')
    colour(design, p2, 'Aluminum - Satin')
    colour(design, bt, 'Paint - Enamel Glossy (Black)')
    app.activeViewport.fit()

    return ('GT2 belt drive built: pulleys of %d and %d teeth, a belt of %d teeth (%.0f mm).\n\n'
            'Pitch diameters %.4f and %.4f mm; outside diameters %.4f and %.4f mm\n'
            'Centre distance C = %.3f mm (solved for a pitch length of %.0f mm)\n'
            'Wrap: pulley 1 %.3f deg, pulley 2 %.3f deg; %.2f teeth of pulley 1 in mesh\n'
            'Ratio %.3f, both pulleys turning the same way\n\n[%s]'
            % (z1, z2, n_b, n_b * PITCH, k['pd1'], k['pd2'], k['od1'], k['od2'], C, n_b * PITCH,
               math.degrees(k['wrap1']), math.degrees(k['wrap2']), k['mesh1'], k['ratio'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(z1=ins.itemById('z1').value, z2=ins.itemById('z2').value, n_b=ins.itemById('n_b').value,
                     width=ins.itemById('width').value / CM)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addIntegerSpinnerCommandInput('z1', 'Pulley 1 teeth', 8, 200, 1, 20)
        ins.addIntegerSpinnerCommandInput('z2', 'Pulley 2 teeth', 8, 200, 1, 40)
        ins.addIntegerSpinnerCommandInput('n_b', 'Belt teeth', 20, 2000, 1, 90)
        ins.addValueInput('width', 'Belt width', 'mm', VI(6.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'BeltDriveCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Belt drive', 'A GT2 timing-belt drive')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Belt drive could not start:\n' + traceback.format_exc())
