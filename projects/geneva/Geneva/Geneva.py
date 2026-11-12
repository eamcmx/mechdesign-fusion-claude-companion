# =============================================================================
# Geneva.py  -  Chapter 25 of "Mechanical Design in Autodesk Fusion with Claude": an external Geneva drive. A driver
# with a pin and a locking disc indexes a slotted wheel by one slot per turn and holds it still between. Written from
# PROMPT.txt beside it; checked by checks/geneva_check.py.
#
#   INPUTS        the dialog: n, a, the pin diameter
#   DERIVED       geneva(): C, b, alpha, the indexing and dwell angles, the peak speed ratio, the gap sizes
#   HARD RULES    build() refuses n outside 3-12, a pin larger than a / 3 and slots whose bottoms reach the bore;
#                 built at dwell, the pin away
#   CONVENTIONS   a new design; parts at the origin, drawn in place; cm inside the API; two As-built revolute
#                 joints and no Motion Link (the wheel's motion is not a constant ratio)
#   ACCEPTANCE    the message; the driver stepped 10 deg in the dwell and 1 deg in the index, the wheel set by
#                 the law: no interference, 0.15 mm clear,
#                 the wheel -90 deg after the turn
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'Geneva 1.1 (2026-11-12)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
GAP = 0.2                                  # mm, the running gap
BORE = 8.0                                 # mm, the wheel's bore
RELIEF = 0.5                               # mm over the running gap: the slots' mouth corners swing out of the wheel's
                                           # circle against the relief between mid-index and the steps either side of it
WALL = 2.0                                 # mm, the least wall between a slot's bottom and the bore


def geneva(n, a, dp):
    C = a / math.sin(math.pi / n)
    alpha = math.pi / 2 - math.pi / n
    return dict(C=C, b=math.sqrt(C * C - a * a), alpha=alpha, index=2 * alpha, dwell=2 * math.pi - 2 * alpha,
                peak=a / (C - a), r_lock=a - dp, w=dp / 2 + GAP, r_bot=C - a - dp / 2 - 1.0)


# ---------- Fusion building blocks (mm at the interface) ----------
def new_part(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
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


def all_profiles(sk):
    col = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        col.add(p)
    return col


def circle(sk, x, y, r):
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), r * CM)


def polygon(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0] * CM, pts[0][1] * CM, 0), P3(pts[1][0] * CM, pts[1][1] * CM, 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0] * CM, q[1] * CM, 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def revolute(root, moving, fixed, x, y, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(x * CM, y * CM, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
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
def build(n=4, a=30.0, dp=8.0):
    n = int(round(n))
    if not 3 <= n <= 12:
        raise ValueError('%d slots: choose from 3 to 12.' % n)
    if dp > a / 3:
        raise ValueError('A pin of %.1f mm is too large for a = %.1f mm: at most a / 3 = %.1f mm.' % (dp, a, a / 3))
    g = geneva(n, a, dp)
    if g['r_bot'] < BORE / 2 + WALL:
        raise ValueError("The slots' bottoms come %.2f mm from the wheel's centre, inside its %.0f mm bore and a %.0f mm "
                         'wall: the wheel would fall apart. Make a larger.' % (g['r_bot'], BORE, WALL))
    C, b = g['C'], g['b']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    ground = new_part(root, 'Ground')
    k = ground.component
    sk = k.sketches.add(plane_at(k, -6.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-(a + 12) * CM, -(b + 12) * CM, 0), P3((C + b + 12) * CM, (b + 12) * CM, 0))
    extrude(k, sk.profiles.item(0), 6.0).bodies.item(0).name = 'Ground'
    ground.isGrounded = True

    drv = new_part(root, 'Driver')
    k = drv.component
    sk = k.sketches.add(k.xYConstructionPlane)
    circle(sk, 0, 0, a + 6)
    body = extrude(k, sk.profiles.item(0), 3.5).bodies.item(0)                     # the crank disc, 0.5 under the wheel
    body.name = 'Driver'
    sk = k.sketches.add(k.xYConstructionPlane)
    circle(sk, -a, 0, dp / 2)
    extrude(k, sk.profiles.item(0), 10.0, JOIN, body)                              # the pin, pointing away (theta = pi)
    sk = k.sketches.add(plane_at(k, 3.5))
    circle(sk, 0, 0, g['r_lock'])
    circle(sk, -C, 0, b + GAP + RELIEF)  # the relief, towards the pin
    crescent = [p for p in sk.profiles if p.areaProperties().centroid.x > 0][0]
    extrude(k, crescent, 6.5, JOIN, body)                                          # the locking disc

    whl = new_part(root, 'Wheel')
    k = whl.component
    sk = k.sketches.add(plane_at(k, 4.0))
    circle(sk, C, 0, b)
    circle(sk, C, 0, BORE / 2)
    body = extrude(k, [p for p in sk.profiles if p.profileLoops.count == 2][0], 6.0).bodies.item(0)
    body.name = 'Wheel'
    r_c = g['r_bot'] + g['w']                                                      # centre of each slot's round end
    for i in range(n):
        psi = math.pi + math.pi / n + 2 * math.pi * i / n                          # slot i points this way
        ux, uy, vx, vy = math.cos(psi), math.sin(psi), -math.sin(psi), math.cos(psi)
        r_out = b + 2.0
        sk = k.sketches.add(plane_at(k, 4.0))
        polygon(sk, [(C + r_c * ux + g['w'] * vx, r_c * uy + g['w'] * vy), (C + r_out * ux + g['w'] * vx, r_out * uy + g['w'] * vy),
                     (C + r_out * ux - g['w'] * vx, r_out * uy - g['w'] * vy), (C + r_c * ux - g['w'] * vx, r_c * uy - g['w'] * vy)])
        circle(sk, C + r_c * ux, r_c * uy, g['w'])
        extrude(k, all_profiles(sk), 6.0, CUT, body)
        lam = math.pi + 2 * math.pi * i / n                                         # a locking arc between two slots
        sk = k.sketches.add(plane_at(k, 4.0))
        circle(sk, C + C * math.cos(lam), C * math.sin(lam), g['r_lock'] + GAP)
        extrude(k, all_profiles(sk), 6.0, CUT, body)

    revolute(root, drv, ground, 0.0, 0.0, 'Driver turns')
    revolute(root, whl, ground, C, 0.0, 'Wheel turns')
    colour(design, ground, 'Paint - Enamel Glossy (Grey)')
    colour(design, drv, 'Paint - Enamel Glossy (Blue)')
    colour(design, whl, 'Paint - Enamel Glossy (Yellow)')
    app.activeViewport.fit()

    return ('Geneva drive built: %d slots, pin circle a = %.1f mm, pin %.1f mm.\n\n'
            'Centre distance C = a / sin(pi / n) = %.3f mm; wheel radius b = %.3f mm\n'
            'Each driver turn: %.1f deg indexing (the wheel turns %.1f deg the other way), %.1f deg dwell\n'
            'Peak wheel speed at mid-index: %.3f times the driver\'s\n\n'
            'Turn "Driver turns" and set "Wheel turns" by the Geneva law: Fusion cannot link them.\n\n[%s]'
            % (n, a, dp, C, b, math.degrees(g['index']), 360.0 / n, math.degrees(g['dwell']), g['peak'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(n=ins.itemById('n').value, a=ins.itemById('a').value / CM, dp=ins.itemById('dp').value / CM)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addIntegerSpinnerCommandInput('n', 'Slots', 2, 24, 1, 4)
        ins.addValueInput('a', 'Pin circle radius', 'mm', VI(30.0 * CM))
        ins.addValueInput('dp', 'Pin diameter', 'mm', VI(8.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'GenevaCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Geneva drive', 'A Geneva indexing drive')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Geneva drive could not start:\n' + traceback.format_exc())
