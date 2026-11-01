# =============================================================================
# FourBar.py  -  Chapter 21 of "Mechanical Design in Autodesk Fusion with Claude": a four-bar crank-rocker linkage,
# assembled as one closed loop of four revolute joints. Written from PROMPT.txt beside it; checked by
# checks/four_bar_check.py.
#
#   INPUTS        the dialog: a, b, c, d
#   DERIVED       position(): B and C for a crank angle, by intersecting two circles; linkage(): the start, the
#                 rocker's swing, the transmission angle's range
#   HARD RULES    build() refuses a linkage whose crank cannot turn fully and a transmission angle below 40 deg
#   CONVENTIONS   a new design; parts at the origin, drawn in place, on four levels; each bar one closed outline of
#                 arcs and lines; cm inside the API; four As-built revolute joints closing one loop
#   ACCEPTANCE    the message; the rocker against Freudenstein's equation at 12 steps; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'FourBar 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
WIDTH, HOLE, LAYER = 8.0, 3.0, 4.0         # mm: bar width, hole diameter, bar thickness
MU_MIN = 40.0                              # deg, the smallest transmission angle accepted


def position(a, b, c, d, t2):
    """B and C (mm) for the crank angle t2: C where the circle of radius b round B meets the circle of radius c round
    D = (d, 0), on the upper branch."""
    bx, by = a * math.cos(t2), a * math.sin(t2)
    dx, dy = d - bx, -by
    L = math.hypot(dx, dy)
    x = (b * b - c * c + L * L) / (2 * L)                        # along BD from B
    h = math.sqrt(b * b - x * x)                                 # square to BD
    ux, uy = dx / L, dy / L
    return (bx, by), (bx + x * ux - h * uy, by + x * uy + h * ux)


def linkage(a, b, c, d):
    t4 = []
    for k in range(360):
        _, (cx, cy) = position(a, b, c, d, math.radians(k))
        t4.append(math.degrees(math.atan2(cy, cx - d)))
    mu = lambda t2: math.degrees(math.acos((b * b + c * c - (a * a + d * d - 2 * a * d * math.cos(t2))) / (2 * b * c)))
    return dict(t4_0=t4[0], t4_min=min(t4), t4_max=max(t4), swing=max(t4) - min(t4), mu_min=mu(0.0), mu_max=mu(math.pi))


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


def bar(occ, p, q, z0):
    """A bar from p to q (mm), WIDTH wide with round ends and a hole at each end, from z0 to z0 + LAYER."""
    c = occ.component
    sk = c.sketches.add(plane_at(c, z0))
    ux, uy = q[0] - p[0], q[1] - p[1]
    n = math.hypot(ux, uy)
    ux, uy = ux / n, uy / n
    nx, ny = -uy, ux
    r = WIDTH / 2
    P = lambda x, y: P3(x * CM, y * CM, 0)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    side = lines.addByTwoPoints(P(p[0] + r * nx, p[1] + r * ny), P(q[0] + r * nx, q[1] + r * ny))
    a1 = arcs.addByThreePoints(side.endSketchPoint, P(q[0] + r * ux, q[1] + r * uy), P(q[0] - r * nx, q[1] - r * ny))
    e1 = a1.endSketchPoint if a1.startSketchPoint.geometry.distanceTo(side.endSketchPoint.geometry) < 1e-9 else a1.startSketchPoint
    back = lines.addByTwoPoints(e1, P(p[0] - r * nx, p[1] - r * ny))
    arcs.addByThreePoints(back.endSketchPoint, P(p[0] - r * ux, p[1] - r * uy), side.startSketchPoint)
    for x, y in (p, q):
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), HOLE / 2 * CM)
    prof = [pr for pr in sk.profiles if pr.profileLoops.count == 3][0]
    ext = c.features.extrudeFeatures
    inp = ext.createInput(prof, NEW)
    inp.setDistanceExtent(False, VI(LAYER * CM))
    ext.add(inp).bodies.item(0).name = c.name


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


# ---------- the linkage ----------
def build(a=15.0, b=55.0, c=50.0, d=55.0):
    links = sorted([a, b, c, d])
    if links[0] + links[3] > links[1] + links[2] or a > min(b, c, d):
        raise ValueError('Not a crank-rocker: Grashof s + l = %.1f against p + q = %.1f, and the crank (%.1f mm) must be '
                         'the shortest link.' % (links[0] + links[3], links[1] + links[2], a))
    k = linkage(a, b, c, d)
    if k['mu_min'] < MU_MIN:
        raise ValueError('The transmission angle falls to %.2f deg: below %.0f deg the linkage binds.' % (k['mu_min'], MU_MIN))
    B, C = position(a, b, c, d, 0.0)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    ground = new_part(root, 'Ground')
    g = ground.component
    sk = g.sketches.add(plane_at(g, -6.0))
    m = WIDTH + 6
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-(a + m) * CM, -(a + m) * CM, 0), P3((d + c + m) * CM, (b + c + m) * CM, 0))
    ext = g.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), NEW)
    inp.setDistanceExtent(False, VI(6.0 * CM))
    ext.add(inp).bodies.item(0).name = 'Ground'
    ground.isGrounded = True

    crank, coupler, rocker = new_part(root, 'Crank'), new_part(root, 'Coupler'), new_part(root, 'Rocker')
    bar(crank, (0.0, 0.0), B, 0.0)
    bar(coupler, B, C, LAYER)
    bar(rocker, (d, 0.0), C, 2 * LAYER)

    revolute(root, crank, ground, 0.0, 0.0, 'Crank turns')
    revolute(root, rocker, ground, d, 0.0, 'Rocker on ground')
    revolute(root, coupler, crank, B[0], B[1], 'Coupler on crank')
    revolute(root, coupler, rocker, C[0], C[1], 'Coupler on rocker')        # closes the loop
    colour(design, ground, 'Paint - Enamel Glossy (Grey)')
    colour(design, crank, 'Paint - Enamel Glossy (Red)')
    colour(design, coupler, 'Paint - Enamel Glossy (Yellow)')
    colour(design, rocker, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    return ('Four-bar crank-rocker built: a = %.1f, b = %.1f, c = %.1f, d = %.1f mm.\n\n'
            'At theta2 = 0: B = (%.3f, %.3f), C = (%.3f, %.3f), theta4 = %.3f deg\n'
            'The rocker swings %.3f deg, from %.3f to %.3f deg\n'
            'Transmission angle from %.2f to %.2f deg\n\n'
            'Step "Crank turns" round (30 deg at a time) with Drive Joints.\n\n[%s]'
            % (a, b, c, d, B[0], B[1], C[0], C[1], k['t4_0'], k['swing'], k['t4_min'], k['t4_max'], k['mu_min'],
               k['mu_max'], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = {k: ins.itemById(k).value / CM for k in ('a', 'b', 'c', 'd')}
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for k, label, v in (('a', 'Crank a', 15.0), ('b', 'Coupler b', 55.0), ('c', 'Rocker c', 50.0), ('d', 'Ground d', 55.0)):
            ins.addValueInput(k, label, 'mm', VI(v * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'FourBarCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Four-bar linkage', 'A crank-rocker four-bar linkage')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Four-bar linkage could not start:\n' + traceback.format_exc())
