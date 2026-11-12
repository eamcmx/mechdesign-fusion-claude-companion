# -*- coding: utf-8 -*-
# =============================================================================
# FourBar.py  -  a four-bar crank-rocker linkage on a base plate, assembled as one closed loop of revolute
# As-built Joints, so that Drive Joints on the crank swings the rocker.
#
#   inputs     crank a, coupler b, rocker c, ground d (mm), in a command dialog
#   derived    the rocker's end C on the upper branch at every crank angle, theta4, the rocker's swing,
#              the transmission angle mu and its smallest and largest values
#   refused    not Grashof, the crank not the shortest link, the acute transmission angle below 40 deg
#   built      Ground (plate z -6..0, grounded), Crank (z 0..4), Coupler (z 4..8), Rocker (z 8..12),
#              at theta2 = 0; four revolute As-built Joints about z
#
# Fusion's API works in cm and radians; every size here is in mm and converted with CM.
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    """The first folder above this script that holds common/companion_paths.py."""
    here = os.path.dirname(os.path.abspath(__file__))
    d = here
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % here)
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402,F401  (puts every lib/ on sys.path; this script needs no library)

VERSION = 'FourBar pt-1 (2026-11-11)'

CM = 0.1                                  # mm to cm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal

WIDTH = 8.0                               # every link a bar 8 mm wide with round ends
HOLE = 3.0                                # a 3 mm hole at each end
PLATE = (-6.0, 0.0)                       # the ground plate, z from -6 to 0
LAYERS = {'Crank': (0.0, 4.0), 'Coupler': (4.0, 8.0), 'Rocker': (8.0, 12.0)}
MU_LIMIT = 40.0                           # the smallest acute transmission angle allowed (deg)

CMD_ID = 'mdfcFourBarCrankRocker'
INPUTS = (('a', 'Crank length a', 15.0),
          ('b', 'Coupler length b', 55.0),
          ('c', 'Rocker length c', 50.0),
          ('d', 'Ground length d', 55.0))

_handlers = []
_own_script = False


# ---------- kinematics, in mm and radians ----------
def _wrap(x):
    """An angle brought into (-pi, pi]."""
    return math.atan2(math.sin(x), math.cos(x))


def positions(a, b, c, d, t2):
    """B and C at crank angle t2: B = a (cos t2, sin t2); C where the circle of radius b round B meets the circle
    of radius c round D = (d, 0), on the upper branch (to the left of the line from B to D, the open linkage).
    None if the two circles do not meet."""
    bx, by = a * math.cos(t2), a * math.sin(t2)
    ex, ey = d - bx, -by
    e = math.hypot(ex, ey)
    if e < 1e-12:
        return None
    x = (b * b - c * c + e * e) / (2.0 * e)          # from B along BD to the foot of C
    h2 = b * b - x * x
    if h2 < 0.0:
        return None
    h = math.sqrt(h2)
    ux, uy = ex / e, ey / e
    return (bx, by), (bx + x * ux - h * uy, by + x * uy + h * ux)


def _golden(f, lo, hi, tol=1e-12):
    """The argument of the largest value of f on [lo, hi] (golden-section search)."""
    g = (math.sqrt(5.0) - 1.0) / 2.0
    x1, x2 = hi - g * (hi - lo), lo + g * (hi - lo)
    f1, f2 = f(x1), f(x2)
    while hi - lo > tol:
        if f1 < f2:
            lo, x1, f1 = x1, x2, f2
            x2 = lo + g * (hi - lo)
            f2 = f(x2)
        else:
            hi, x2, f2 = x2, x1, f1
            x1 = hi - g * (hi - lo)
            f1 = f(x1)
    return (lo + hi) / 2.0


def linkage(a, b, c, d):
    """Everything derived from the four lengths (mm), angles in degrees:
    t4_0 the rocker angle theta4 = atan2(C_y, C_x - d) at theta2 = 0; t4_min, t4_max and swing over a full turn of
    the crank; mu_min = mu(0) and mu_max = mu(180 deg), the transmission angle between coupler and rocker."""
    def t4(t2):
        p = positions(a, b, c, d, t2)
        if p is None:
            raise ValueError('Not a crank-rocker: the coupler and the rocker cannot meet at theta2 = %.1f deg'
                             % math.degrees(t2))
        cx, cy = p[1]
        return math.atan2(cy, cx - d)

    t40 = t4(0.0)

    def f(t2):                                         # theta4 kept continuous round theta4(0): a rocker swings < 180
        return t40 + _wrap(t4(t2) - t40)

    n = 720
    step = 2.0 * math.pi / n
    ts = [i * step for i in range(n)]
    vs = [f(t) for t in ts]
    i_hi = max(range(n), key=lambda i: vs[i])
    i_lo = min(range(n), key=lambda i: vs[i])
    t_hi = _golden(f, ts[i_hi] - step, ts[i_hi] + step)
    t_lo = _golden(lambda t: -f(t), ts[i_lo] - step, ts[i_lo] + step)
    t4_hi, t4_lo = f(t_hi), f(t_lo)

    def mu(t2):
        bd2 = a * a + d * d - 2.0 * a * d * math.cos(t2)
        return math.degrees(math.acos(max(-1.0, min(1.0, (b * b + c * c - bd2) / (2.0 * b * c)))))

    B0, C0 = positions(a, b, c, d, 0.0)
    return dict(t4_0=math.degrees(t40), t4_min=math.degrees(t4_lo), t4_max=math.degrees(t4_hi),
                swing=math.degrees(t4_hi - t4_lo), t2_at_min=math.degrees(t_lo) % 360.0,
                t2_at_max=math.degrees(t_hi) % 360.0, mu_min=mu(0.0), mu_max=mu(math.pi),
                A=(0.0, 0.0), B=B0, C=C0, D=(d, 0.0))


def grashof(a, b, c, d):
    """Refuses (ValueError) unless s + l <= p + q and the crank is the shortest link, so that it turns fully."""
    for name, v in (('a', a), ('b', b), ('c', c), ('d', d)):
        if not v > 0.0:
            raise ValueError('Not a crank-rocker: the length %s = %g mm must be positive' % (name, v))
    s, p, q, l = sorted((a, b, c, d))
    if s + l > p + q + 1e-9:
        raise ValueError('Not a crank-rocker: s + l = %g + %g = %g mm is more than p + q = %g + %g = %g mm '
                         '(Grashof), so no link turns fully' % (s, l, s + l, p, q, p + q))
    if a > s + 1e-9:
        raise ValueError('Not a crank-rocker: the crank a = %g mm is not the shortest link (%g mm), so it '
                         'does not turn fully' % (a, s))


def transmission(k):
    """Refuses (ValueError) if the acute transmission angle falls below 40 deg: mu(0) and 180 deg - mu(180 deg)."""
    at0, at180 = k['mu_min'], 180.0 - k['mu_max']
    worst, where = (at0, 0) if at0 <= at180 else (at180, 180)
    if worst < MU_LIMIT:
        raise ValueError('The transmission angle falls to %.2f deg at theta2 = %d deg; it must stay at least '
                         '%.0f deg (mu(0) = %.2f deg, 180 deg - mu(180 deg) = %.2f deg)'
                         % (worst, where, MU_LIMIT, at0, at180))


# ---------- geometry in Fusion ----------
def _new_occ(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())     # at the origin, drawn in place
    occ.component.name = name
    return occ


def _plane(comp, z):
    """The plane z (mm) of the component's own frame, square to z."""
    if abs(z) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z * CM))
    return comp.constructionPlanes.add(ci)


def _sketch(comp, z, name):
    sk = comp.sketches.add(_plane(comp, z))
    sk.name = name
    up = sk.xDirection.crossProduct(sk.yDirection).z > 0.0     # the sketch's normal along +z?
    return sk, up


def _to_sketch(sk, x, y, z):
    return sk.modelToSketchSpace(P3(x * CM, y * CM, z * CM))


def _largest_profile(sk):
    best, area = None, -1.0
    for i in range(sk.profiles.count):
        pr = sk.profiles.item(i)
        ar = pr.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area
        if ar > area:
            best, area = pr, ar
    return best


def _extrude(comp, profile, length, along_plus_z, sketch_up, name):
    """A new body from the profile, `length` mm along +z (along_plus_z) or -z."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    positive = along_plus_z == sketch_up
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(length * CM)),
                         adsk.fusion.ExtentDirections.PositiveExtentDirection if positive
                         else adsk.fusion.ExtentDirections.NegativeExtentDirection)
    feat = ext.add(inp)
    body = feat.bodies.item(0)
    body.name = name
    return body


def _link(occ, p, q, z0, z1, name):
    """A bar from p to q (mm), WIDTH wide with round ends: two arcs and two lines sharing their sketch points, a
    HOLE at each end on the arcs' centres, from z0 to z1. Returns the sketch points at p and q."""
    comp = occ.component
    sk, up = _sketch(comp, z0, name)
    sweep = math.pi if up else -math.pi                 # half a turn anticlockwise about +z
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    ux, uy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    nx, ny = -uy, ux                                    # the left normal of p -> q
    r = WIDTH / 2.0
    arcs = sk.sketchCurves.sketchArcs
    lines = sk.sketchCurves.sketchLines
    circles = sk.sketchCurves.sketchCircles
    # round the end at p: from p + r n through p - r u to p - r n; round the end at q: from q - r n through
    # q + r u to q + r n
    arc_p = arcs.addByCenterStartSweep(_to_sketch(sk, p[0], p[1], z0),
                                       _to_sketch(sk, p[0] + r * nx, p[1] + r * ny, z0), sweep)
    arc_q = arcs.addByCenterStartSweep(_to_sketch(sk, q[0], q[1], z0),
                                       _to_sketch(sk, q[0] - r * nx, q[1] - r * ny, z0), sweep)
    lines.addByTwoPoints(arc_p.endSketchPoint, arc_q.startSketchPoint)
    lines.addByTwoPoints(arc_q.endSketchPoint, arc_p.startSketchPoint)
    circles.addByCenterRadius(arc_p.centerSketchPoint, HOLE / 2.0 * CM)
    circles.addByCenterRadius(arc_q.centerSketchPoint, HOLE / 2.0 * CM)
    _extrude(comp, _largest_profile(sk), z1 - z0, True, up, name)
    return arc_p.centerSketchPoint, arc_q.centerSketchPoint


def _plate(occ, a, b, c, d):
    """The ground plate under the whole motion (A, D and every place B and C reach, 14 mm beyond them, out to
    whole 5 mm), z -6 to 0, with a 3 mm hole at A and at D for the pins."""
    xs, ys = [0.0, d], [0.0, 0.0]
    for i in range(360):
        pb, pc = positions(a, b, c, d, math.radians(i))
        xs += [pb[0], pc[0]]
        ys += [pb[1], pc[1]]
    m = WIDTH / 2.0 + 10.0
    x0, x1 = 5.0 * math.floor((min(xs) - m) / 5.0), 5.0 * math.ceil((max(xs) + m) / 5.0)
    y0, y1 = 5.0 * math.floor((min(ys) - m) / 5.0), 5.0 * math.ceil((max(ys) + m) / 5.0)
    comp = occ.component
    sk, up = _sketch(comp, PLATE[1], 'Plate')
    sk.sketchCurves.sketchLines.addTwoPointRectangle(_to_sketch(sk, x0, y0, PLATE[1]), _to_sketch(sk, x1, y1, PLATE[1]))
    for px, py in ((0.0, 0.0), (d, 0.0)):
        sk.sketchCurves.sketchCircles.addByCenterRadius(_to_sketch(sk, px, py, PLATE[1]), HOLE / 2.0 * CM)
    _extrude(comp, _largest_profile(sk), PLATE[1] - PLATE[0], False, up, 'Plate')
    return (x0, x1, y0, y1)


def _revolute(root, name, one, two, point, point_occ):
    """An As-built Joint, revolute about z, of occurrence `one` on occurrence `two` at a sketch point of
    `point_occ`."""
    geo = adsk.fusion.JointGeometry.createByPoint(point.createForAssemblyContext(point_occ))
    inp = root.asBuiltJoints.createInput(one, two, geo)
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def message(a, b, c, d, k):
    return ('%s\n'
            'Four-bar crank-rocker, a = %g, b = %g, c = %g, d = %g mm, built at theta2 = 0 (upper branch).\n'
            'theta4 = %.3f deg at the start.\n'
            'The rocker swings %.3f deg, from %.3f to %.3f deg.\n'
            'The transmission angle runs from %.2f to %.2f deg (mu(0) to mu(180 deg)).'
            % (VERSION, a, b, c, d, k['t4_0'], k['swing'], k['t4_min'], k['t4_max'], k['mu_min'], k['mu_max']))


def build(a=15.0, b=55.0, c=50.0, d=55.0):
    """Checks the design, then builds it in a new Fusion design; returns the message. ValueError if refused."""
    a, b, c, d = float(a), float(b), float(c), float(d)
    grashof(a, b, c, d)
    k = linkage(a, b, c, d)
    transmission(k)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent

    ground = _new_occ(root, 'Ground')
    ground.isGrounded = True
    _plate(ground, a, b, c, d)

    A, B, C, D = k['A'], k['B'], k['C'], k['D']
    crank = _new_occ(root, 'Crank')
    coupler = _new_occ(root, 'Coupler')
    rocker = _new_occ(root, 'Rocker')
    crank_A, crank_B = _link(crank, A, B, *LAYERS['Crank'], name='Crank')
    coupler_B, coupler_C = _link(coupler, B, C, *LAYERS['Coupler'], name='Coupler')
    rocker_D, rocker_C = _link(rocker, D, C, *LAYERS['Rocker'], name='Rocker')

    _revolute(root, 'Crank turns', crank, ground, crank_A, crank)
    _revolute(root, 'Rocker turns', rocker, ground, rocker_D, rocker)
    _revolute(root, 'Coupler on crank', coupler, crank, coupler_B, coupler)
    _revolute(root, 'Coupler on rocker', coupler, rocker, coupler_C, coupler)      # closes the loop

    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return message(a, b, c, d, k)


# ---------- the command dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            v = {key: ins.itemById(key).value / CM for key, _, _ in INPUTS}        # cm to mm
            ui.messageBox(build(**v))
        except ValueError as e:
            ui.messageBox('%s\nRefused: %s' % (VERSION, e))
        except Exception:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _own_script:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            ins = cmd.commandInputs
            for key, label, mm in INPUTS:
                ins.addValueInput(key, label + ' (mm)', 'mm', VI(mm * CM))
            on_exec = _Execute()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


def run(context):
    global _own_script
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        _own_script = context is not None        # run by Fusion as a script, not called by another script
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Four-bar crank-rocker',
                                                       'A crank-rocker on a base plate, one loop of revolute joints')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        if _own_script:
            adsk.autoTerminate(False)            # keep the script alive while its dialog is open
    except Exception:
        if ui:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
