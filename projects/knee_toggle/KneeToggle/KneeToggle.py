# =============================================================================
# KneeToggle.py  -  Chapter 23 of "Mechanical Design in Autodesk Fusion with Claude": a knee toggle. Two equal links
# from a fixed pivot down to a ram on a straight guide, one closed loop of three revolute joints and a slider. Written
# from PROMPT.txt beside it; checked by checks/knee_toggle_check.py.
#
#   INPUTS        the dialog: l, alpha0, the over-centre angle, P
#   DERIVED       toggle(): the knee and the ram at an angle, the mechanical advantage, the stroke, F at five angles
#   HARD RULES    build() refuses a starting angle outside 5-60 deg and an over-centre angle outside 1-10 deg; the
#                 upper link limited to just past straight
#   CONVENTIONS   a new design; parts at the origin, drawn in place at alpha0, on four levels; links with round ends
#                 (Chapter 22); cm inside the API; As-built Joints closing the loop with a slider
#   ACCEPTANCE    the message; the ram's position and the virtual-work ratio at every 2 deg step; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'KneeToggle 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
WIDTH, HOLE, LAYER = 8.0, 3.0, 4.0         # mm: bar width, hole diameter, bar thickness
ANGLES = (30, 10, 5, 2, 1)                 # deg, where the ram's force is reported


def toggle(l, alpha_deg):
    """The knee K and the ram's pin R (mm) at the angle alpha of the links from the ram's line, and F / P there."""
    a = math.radians(alpha_deg)
    return dict(K=(l * math.sin(a), -l * math.cos(a)), R=(0.0, -2 * l * math.cos(a)),
                ma=1.0 / (2 * math.tan(a)) if a > 0 else float('inf'))


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




def slider_y(root, moving, fixed, x, y, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(x * CM, y * CM, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    inp.setAsSliderJointMotion(adsk.fusion.JointDirections.YAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def block(occ, cx, cy, w, h, z0):
    """The ram: a block w x h round (cx, cy) with a hole for its pin, from z0 to z0 + LAYER."""
    c = occ.component
    sk = c.sketches.add(plane_at(c, z0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3((cx - w / 2) * CM, (cy - h / 2) * CM, 0),
                                                     P3((cx + w / 2) * CM, (cy + h / 2) * CM, 0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(cx * CM, cy * CM, 0), HOLE / 2 * CM)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    ext = c.features.extrudeFeatures
    inp = ext.createInput(prof, NEW)
    inp.setDistanceExtent(False, VI(LAYER * CM))
    ext.add(inp).bodies.item(0).name = c.name


# ---------- the toggle ----------
def build(l=40.0, alpha0=30.0, over=3.0, P=100.0):
    if not 5.0 <= alpha0 <= 60.0:
        raise ValueError('Starting angle %.1f deg: choose from 5 to 60 deg.' % alpha0)
    if not 1.0 <= over <= 10.0:
        raise ValueError('Over-centre angle %.1f deg: choose from 1 to 10 deg.' % over)
    t = toggle(l, alpha0)
    K, R = t['K'], t['R']
    stroke = 2 * l * (1 - math.cos(math.radians(alpha0)))
    forces = [P * toggle(l, a)['ma'] for a in ANGLES]

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    frame = new_part(root, 'Frame')
    g = frame.component
    sk = g.sketches.add(plane_at(g, -6.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(-20 * CM, (-2 * l - 20) * CM, 0), P3((l + 15) * CM, 15 * CM, 0))
    ext = g.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), NEW)
    inp.setDistanceExtent(False, VI(6.0 * CM))
    ext.add(inp).bodies.item(0).name = 'Frame'
    frame.isGrounded = True

    upper, lower, ram = new_part(root, 'Upper link'), new_part(root, 'Lower link'), new_part(root, 'Ram')
    bar(upper, (0.0, 0.0), K, 0.0)
    bar(lower, K, R, LAYER)
    block(ram, R[0], R[1], 16.0, 12.0, 2 * LAYER)

    ju = revolute(root, upper, frame, 0.0, 0.0, 'Upper link turns')
    lim = ju.jointMotion.rotationLimits
    lim.isMinimumValueEnabled, lim.minimumValue = True, -math.radians(alpha0 + over)  # just past straight
    lim.isMaximumValueEnabled, lim.maximumValue = True, math.radians(15.0)
    revolute(root, lower, upper, K[0], K[1], 'Knee')
    revolute(root, lower, ram, R[0], R[1], 'Ram pin')
    slider_y(root, ram, frame, R[0], R[1], 'Ram slides')                                # closes the loop
    colour(design, frame, 'Paint - Enamel Glossy (Grey)')
    colour(design, upper, 'Paint - Enamel Glossy (Red)')
    colour(design, lower, 'Paint - Enamel Glossy (Yellow)')
    colour(design, ram, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    return ('Knee toggle built: links of %.1f mm at %.1f deg from the ram\'s line; %.1f deg over centre allowed.\n\n'
            'Mechanical advantage F / P = 1 / (2 tan(alpha)); stroke to straight 2 l (1 - cos(alpha0)) = %.3f mm\n'
            'With P = %.0f N: %s\n\n'
            'Drive "Upper link turns" towards negative angles (2 deg at a time) to straighten the knee.\n\n[%s]'
            % (l, alpha0, over, stroke, P, ', '.join('F = %.2f N at %d deg' % (f, a) for f, a in zip(forces, ANGLES)), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(l=ins.itemById('l').value / CM, alpha0=math.degrees(ins.itemById('alpha0').value),
                     over=math.degrees(ins.itemById('over').value), P=ins.itemById('P').value)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('l', 'Link length', 'mm', VI(40.0 * CM))
        ins.addValueInput('alpha0', 'Starting angle from the ram\'s line', 'deg', VI(math.radians(30.0)))
        ins.addValueInput('over', 'Over-centre angle allowed', 'deg', VI(math.radians(3.0)))
        ins.addValueInput('P', 'Force on the knee (N)', '', VI(100.0))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'KneeToggleCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Knee toggle', 'Two links and a ram: a toggle')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Knee toggle could not start:\n' + traceback.format_exc())
