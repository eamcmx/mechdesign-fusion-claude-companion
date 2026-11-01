# =============================================================================
# CycloidalDrive.py  -  Chapter 18 of "Mechanical Design in Autodesk Fusion with Claude": a single-disc cycloidal
# drive. The input's eccentric rolls a disc of N_p - 1 lobes round N_p ring pins; six output pins in the disc's holes
# take off its slow turn. Written from PROMPT.txt beside it; checked by checks/cycloidal_check.py.
#
#   INPUTS        the dialog: N_p, R, ring pin diameter, e, t
#   DERIVED       cycloid(): N_d, the ratio, K1; disc_outline(): the pin centre's curve in the disc's frame, offset
#   HARD RULES    build() refuses a pin count out of range, pins that overlap, and a looping curve (K1 >= 1)
#   CONVENTIONS   a new design; parts at the origin, drawn in place; the outline one closed polyline; cm inside the
#                 API; As-built Joints, the disc's on the eccentric's centre; two Motion Links
#   ACCEPTANCE    the message; the output -3 deg per 30 deg step of the input; no interference at 12 steps
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'CycloidalDrive 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
GAP = 0.1                                  # mm between the disc and the pins, and in every hole
PER_TURN = 36                              # points of the outline per input turn
N_OUT, D_OUT, R_OUT = 6, 5.0, 17.0         # output pins: number, diameter, circle radius (mm)
R_CAM, DS = 10.0, 8.0                      # eccentric radius and input shaft diameter (mm)


def cycloid(n_p, R, d_pin, e):
    n_d = n_p - 1
    rp = d_pin / 2.0
    return dict(n_d=n_d, ratio=-n_d, K1=e * n_p / R, r_min=R - rp - GAP - e, r_max=R - rp - GAP + e)


def disc_outline(n_p, R, d_pin, e):
    """In the disc's own frame (its centre at the origin, not turned): the curve the centre of ring pin 0 traces while
    the input turns N_d times, the disc's centre at e (cos th, sin th) and the disc turned by -th / N_d; then offset
    towards the disc's centre by the pin radius plus GAP. Points in mm."""
    n_d = n_p - 1
    pts = []
    for i in range(PER_TURN * n_d):
        th = 2 * math.pi * i / PER_TURN
        cx, cy = e * math.cos(th), e * math.sin(th)
        phi = -th / n_d
        px, py = R - cx, -cy                                       # pin 0 seen from the disc's centre ...
        c, s = math.cos(-phi), math.sin(-phi)
        pts.append((px * c - py * s, px * s + py * c))             # ... in the disc's turned frame
    n = len(pts)
    area = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1] for i in range(n)) / 2
    d = d_pin / 2.0 + GAP
    out = []
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i - 1], pts[(i + 1) % n]
        L = math.hypot(x1 - x0, y1 - y0)
        nx, ny = -(y1 - y0) / L, (x1 - x0) / L                     # left normal: inward for an anticlockwise curve
        if area < 0:
            nx, ny = -nx, -ny
        out.append((pts[i][0] + d * nx, pts[i][1] + d * ny))
    return out


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


def circles(comp, z_mm, items):
    """A sketch at height z with circles (x, y, radius) in mm; returns the sketch."""
    sk = comp.sketches.add(plane_at(comp, z_mm))
    for x, y, r in items:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), r * CM)
    return sk


def all_profiles(sk):
    col = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        col.add(p)
    return col


def polyline(sk, pts):
    """One closed polyline whose lines share their sketch points."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0] * CM, pts[0][1] * CM, 0), P3(pts[1][0] * CM, pts[1][1] * CM, 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0] * CM, q[1] * CM, 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def revolute(root, moving, fixed, x, y, name):
    """An As-built revolute joint about z through the world point (x, y, 0)."""
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(x * CM, y * CM, 0)).createForAssemblyContext(moving)
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def link(root, j1, j2, deg1, deg2, name):
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % deg1)
    li.valueTwo = VS('%d deg' % deg2)
    li.isReversed = True
    root.motionLinks.add(li).name = name


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
def build(n_p=11, R=30.0, d_pin=6.0, e=1.5, t=8.0):
    n_p = int(round(n_p))
    if not 6 <= n_p <= 40:
        raise ValueError('%d ring pins: choose from 6 to 40.' % n_p)
    if 2 * R * math.sin(math.pi / n_p) <= d_pin + 1.0:
        raise ValueError('The pins overlap: 2 R sin(pi / N_p) = %.2f mm is not more than the pin diameter + 1 mm.'
                         % (2 * R * math.sin(math.pi / n_p)))
    k = cycloid(n_p, R, d_pin, e)
    if k['K1'] >= 1.0:
        raise ValueError('K1 = e N_p / R = %.3f: the curve of the pin centre loops. Use a smaller eccentricity.' % k['K1'])
    n_d = k['n_d']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    housing = new_part(root, 'Housing')
    c = housing.component
    sk = circles(c, -6.0, [(0, 0, R + d_pin / 2 + 6), (0, 0, (DS + 0.2) / 2)])
    body = extrude(c, [q for q in sk.profiles if q.profileLoops.count == 2][0], 6.0).bodies.item(0)
    body.name = 'Housing'
    sk = circles(c, 0.0, [(R * math.cos(2 * math.pi * i / n_p), R * math.sin(2 * math.pi * i / n_p), d_pin / 2)
                          for i in range(n_p)])
    extrude(c, all_profiles(sk), t + 2.0, JOIN, body)
    housing.isGrounded = True

    inp_ = new_part(root, 'Input')
    c = inp_.component
    body = extrude(c, circles(c, -6.0, [(0, 0, DS / 2)]).profiles.item(0), t + 14.0).bodies.item(0)
    body.name = 'Input'
    extrude(c, circles(c, 1.0, [(e, 0, R_CAM)]).profiles.item(0), t, JOIN, body)           # the eccentric

    disc = new_part(root, 'Disc')
    c = disc.component
    sk = c.sketches.add(plane_at(c, 1.0))
    polyline(sk, [(x + e, y) for x, y in disc_outline(n_p, R, d_pin, e)])
    body = extrude(c, sk.profiles.item(0), t).bodies.item(0)
    body.name = 'Disc'
    holes = [(e, 0, R_CAM + GAP)] + [(e + R_OUT * math.cos(2 * math.pi * i / N_OUT), R_OUT * math.sin(2 * math.pi * i / N_OUT),
                                      D_OUT / 2 + e + GAP) for i in range(N_OUT)]
    extrude(c, all_profiles(circles(c, 1.0, holes)), t, CUT, body)

    out = new_part(root, 'Output')
    c = out.component
    sk = circles(c, t + 2.0, [(0, 0, R_OUT + D_OUT / 2 + 3), (0, 0, (DS + 0.2) / 2)])
    body = extrude(c, [q for q in sk.profiles if q.profileLoops.count == 2][0], 4.0).bodies.item(0)
    body.name = 'Output'
    sk = circles(c, 1.0, [(R_OUT * math.cos(2 * math.pi * i / N_OUT), R_OUT * math.sin(2 * math.pi * i / N_OUT), D_OUT / 2)
                          for i in range(N_OUT)])
    extrude(c, all_profiles(sk), t + 1.0, JOIN, body)

    j_in = revolute(root, inp_, housing, 0, 0, 'Input turns')
    j_d = revolute(root, disc, inp_, e, 0, 'Disc on eccentric')
    j_out = revolute(root, out, housing, 0, 0, 'Output turns')
    link(root, j_in, j_d, 360 * n_d, 360 * (n_d + 1), 'Disc rolls')       # -(N_d + 1) / N_d relative to the eccentric
    link(root, j_in, j_out, 360 * n_d, 360, 'Output')                     # -1 / N_d
    colour(design, housing, 'Paint - Enamel Glossy (Grey)')
    colour(design, disc, 'Paint - Enamel Glossy (Yellow)')
    colour(design, out, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    return ('Cycloidal drive built: %d ring pins on R = %.1f mm, a disc of %d lobes, eccentricity %.2f mm.\n\n'
            'Ratio %d (the output turns the other way, 1/%d of the input)\n'
            'K1 = e N_p / R = %.3f; the disc outline runs from %.3f to %.3f mm; gap %.2f mm\n\n'
            'Step the input 30 deg at a time: the output turns %.1f deg per step.\n\n[%s]'
            % (n_p, R, n_d, e, k['ratio'], n_d, k['K1'], k['r_min'], k['r_max'], GAP, -30.0 / n_d, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(n_p=ins.itemById('n_p').value, R=ins.itemById('R').value / CM, d_pin=ins.itemById('d_pin').value / CM,
                     e=ins.itemById('e').value / CM, t=ins.itemById('t').value / CM)
            ui.messageBox(build(**p), VERSION)
        except ValueError as ex:
            ui.messageBox(str(ex), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addIntegerSpinnerCommandInput('n_p', 'Ring pins', 3, 60, 1, 11)
        ins.addValueInput('R', 'Pin circle radius', 'mm', VI(30.0 * CM))
        ins.addValueInput('d_pin', 'Ring pin diameter', 'mm', VI(6.0 * CM))
        ins.addValueInput('e', 'Eccentricity', 'mm', VI(1.5 * CM))
        ins.addValueInput('t', 'Disc thickness', 'mm', VI(8.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'CycloidalDriveCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Cycloidal drive', 'A single-disc cycloidal reducer')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Cycloidal drive could not start:\n' + traceback.format_exc())
