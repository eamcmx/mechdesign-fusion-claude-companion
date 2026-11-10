# =============================================================================
# HarmonicDrive.py  -  Chapter 19 of "Mechanical Design in Autodesk Fusion with Claude": a cup-type harmonic drive.
# The wave generator bends the flexspline into an oval whose long axis meshes with the rigid circular spline; z_c - z_f
# = 2 teeth of difference give a ratio of -z_f / 2. A rigid body cannot bend, so the drive is turned by turn_to(phi),
# which builds the flexspline bent at the new wave generator angle and turned as a whole by -phi (z_c - z_f) / z_f.
# Written from PROMPT.txt beside it; checked by checks/harmonic_check.py.
#
#   INPUTS        the dialog: m, z_f, z_c - z_f, b, t, phi
#   DERIVED       harmonic(): z_c, the ratio, r_f, r_c, w0, the tooth form; bent(): the flexspline's points
#   HARD RULES    harmonic() refuses a difference other than 2, z_f out of range and pointed teeth
#   CONVENTIONS   a new design; parts at the origin, drawn in place; each toothed outline one closed polyline; cm
#                 inside the API; two As-built revolute joints, no Motion Link; turn_to() replaces two bodies
#   ACCEPTANCE    the message; tips measured at the long and short axes; no interference over one mesh period
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'HarmonicDrive 1.2 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
ALPHA = 30.0                               # flank angle of the straight-sided teeth (deg)
HA, HF = 0.75, 0.90                        # addendum and dedendum, in modules, on both parts
JB = 0.06                                  # backlash on the pitch line (mm)
WG_GAP = 0.03                              # wave generator inside the flexspline's bore (mm)
N_CURVE = 360                              # points of the bore, the root line and the wave generator
OVERLAP = 0.05                             # mm each joined part reaches into the last, so the join never rests on a touch
BUILT = 'mdfc build'                       # the name of every sketch and plane the builders make
CUP, HUB = 20.0, 10.0                      # the flexspline's wall below the teeth, the output hub's radius (mm)
_STATE = {}


def harmonic(m=0.3, z_f=200, dz=2, b=8.0, t=0.6):
    z_f, dz = int(round(z_f)), int(round(dz))
    if dz != 2:
        raise ValueError('z_c - z_f = %d: a two-lobed wave generator needs a difference of 2 teeth.' % dz)
    if not 60 <= z_f <= 320:
        raise ValueError('%d flexspline teeth: choose from 60 to 320.' % z_f)
    z_c = z_f + dz
    ta = math.tan(math.radians(ALPHA))
    s = math.pi * m / 2.0 - JB / 2.0
    tip = s / 2.0 - HA * m * ta
    if tip <= 0:
        raise ValueError('The teeth come to a point: tip half-width %.4f mm.' % tip)
    return dict(m=m, z_f=z_f, z_c=z_c, ratio=-z_f / dz, r_f=m * z_f / 2.0, r_c=m * z_c / 2.0, w0=m * dz / 2.0, b=b,
                t=t, ha=HA * m, hf=HF * m, s=s, tip=tip, root=s / 2.0 + HF * m * ta)


def rho_of(k, phi):
    """The flexspline's turn as a whole (radians) when the wave generator stands at phi (radians)."""
    return -phi * (k['z_c'] - k['z_f']) / k['z_f']


def neutral(k, theta, phi):
    """The bent pitch line at material angle theta: out by w = w0 cos 2psi, along by v = -(w0/2) sin 2psi."""
    psi = theta - phi
    r = k['r_f'] + k['w0'] * math.cos(2 * psi)
    a = theta - k['w0'] / 2.0 * math.sin(2 * psi) / k['r_f']
    return r * math.cos(a), r * math.sin(a)


def bent(k, theta, h, phi, d=1e-6):
    """The point h (mm) out from the bent pitch line, on its normal, at material angle theta."""
    x0, y0 = neutral(k, theta, phi)
    x1, y1 = neutral(k, theta + d, phi)
    L = math.hypot(x1 - x0, y1 - y0)
    return x0 + h * (y1 - y0) / L, y0 - h * (x1 - x0) / L


def flex_outline(k, phi):
    rho = rho_of(k, phi)
    pts = []
    for i in range(k['z_f']):
        th = 2 * math.pi * i / k['z_f'] + rho
        for s, h in ((-k['root'], -k['hf']), (-k['tip'], k['ha']), (k['tip'], k['ha']), (k['root'], -k['hf'])):
            pts.append(bent(k, th + s / k['r_f'], h, phi))
        pts.append(bent(k, th + math.pi / k['z_f'], -k['hf'], phi))      # the middle of the root land
    return pts


def flex_curve(k, h, phi):
    rho = rho_of(k, phi)
    return [bent(k, 2 * math.pi * i / N_CURVE + rho, h, phi) for i in range(N_CURVE)]


def cs_outline(k):
    ta = math.tan(math.radians(ALPHA))
    tip, root = k['s'] / 2 - k['ha'] * ta, k['s'] / 2 + k['hf'] * ta
    rt, rr = k['r_c'] - k['ha'], k['r_c'] + k['hf']
    pts = []
    for i in range(k['z_c']):
        c = 2 * math.pi * (i + 0.5) / k['z_c']
        for s, r in ((-root, rr), (-tip, rt), (tip, rt), (root, rr)):
            pts.append((r * math.cos(c + s / k['r_c']), r * math.sin(c + s / k['r_c'])))
    return pts


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
    plane = comp.constructionPlanes.add(ci)
    plane.name = BUILT
    return plane


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
    first = lines.addByTwoPoints(P3(pts[0][0] * CM, pts[0][1] * CM, 0), P3(pts[1][0] * CM, pts[1][1] * CM, 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0] * CM, q[1] * CM, 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def sketch_of(comp, z_mm, curves=(), circles=()):
    """A sketch at height z with closed polylines and circles (x, y, r); returns (sketch, the profile with the most
    loops: the ring between them, or the one region)."""
    sk = comp.sketches.add(plane_at(comp, z_mm))
    sk.name = BUILT
    sk.isComputeDeferred = True
    for pts in curves:
        polyline(sk, pts)
    for x, y, r in circles:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(x * CM, y * CM, 0), r * CM)
    sk.isComputeDeferred = False
    best = max(sk.profiles, key=lambda q: q.profileLoops.count)
    return sk, best


def revolute(root, moving, fixed, name):
    sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(moving)
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


def _occ(root, name):
    for o in root.occurrences:
        if o.component.name == name:
            return o
    raise RuntimeError('no component %r' % name)


# ---------- the bent parts ----------
def _flexspline(comp, k, phi):
    rim = extrude(comp, sketch_of(comp, 0.0, [flex_outline(k, phi), flex_curve(k, -k['hf'] - k['t'], phi)])[1],
                  k['b']).bodies.item(0)
    rim.name = 'Flexspline'
    extrude(comp, sketch_of(comp, -CUP, [flex_curve(k, -k['hf'], phi), flex_curve(k, -k['hf'] - k['t'], phi)])[1],
            CUP + OVERLAP, JOIN, rim)
    r_d = k['r_f'] - k['hf'] + k['w0'] + 0.5     # the diaphragm covers the cup's bent end, so the join meets a whole face
    extrude(comp, sketch_of(comp, -CUP - 2.0, circles=[(0, 0, r_d)])[1], 2.0 + OVERLAP, JOIN, rim)  # the diaphragm
    extrude(comp, sketch_of(comp, -CUP - 6.0, circles=[(0, 0, HUB)])[1], 4.0 + OVERLAP, JOIN, rim)  # the output hub


def _wave_generator(comp, k, phi):
    plug = extrude(comp, sketch_of(comp, 0.0, [flex_curve(k, -k['hf'] - k['t'] - WG_GAP, phi)])[1], k['b']).bodies.item(0)
    plug.name = 'WaveGenerator'
    extrude(comp, sketch_of(comp, k['b'] - OVERLAP, circles=[(0, 0, 5.0)])[1], 12.0 + OVERLAP, JOIN, plug)


def _clear(comp):
    """Delete what the builders made: the extrusions (and with them the bodies), then their sketches and planes. Deleting
    only the bodies would leave every rebuild's features in the timeline, which grows and slows each turn."""
    ext = comp.features.extrudeFeatures
    for i in reversed(range(ext.count)):
        ext.item(i).deleteMe()
    for coll in (comp.sketches, comp.constructionPlanes):
        for i in reversed(range(coll.count)):
            if coll.item(i).name == BUILT:
                coll.item(i).deleteMe()


def turn_to(phi_deg):
    """Turn the drive: the wave generator to phi, the flexspline bent there and turned by rho. Returns rho in deg."""
    k = _STATE['k']
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent
    phi = math.radians(phi_deg)
    for name, make in (('Flexspline', _flexspline), ('WaveGenerator', _wave_generator)):
        comp = _occ(root, name).component
        _clear(comp)
        make(comp, k, phi)
        if comp.bRepBodies.count != 1:
            raise RuntimeError('%s rebuilt at %.4f deg as %d bodies' % (name, phi_deg, comp.bRepBodies.count))
    _STATE['phi'] = phi_deg
    return math.degrees(rho_of(k, phi))


# ---------- the drive ----------
def build(m=0.3, z_f=200, dz=2, b=8.0, t=0.6, phi=0.0):
    k = harmonic(m, z_f, dz, b, t)
    _STATE.clear()
    _STATE['k'] = k

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    cs = new_part(root, 'CircularSpline')
    c = cs.component
    r_out = k['r_c'] + k['hf'] + 4.0
    body = extrude(c, sketch_of(c, 0.0, [cs_outline(k)], [(0, 0, r_out)])[1], b).bodies.item(0)
    body.name = 'CircularSpline'
    cs.isGrounded = True

    fl = new_part(root, 'Flexspline')
    wg = new_part(root, 'WaveGenerator')
    _flexspline(fl.component, k, math.radians(phi))
    _wave_generator(wg.component, k, math.radians(phi))
    _STATE['phi'] = phi

    revolute(root, wg, cs, 'Wave generator turns')
    revolute(root, fl, cs, 'Flexspline turns')
    colour(design, cs, 'Paint - Enamel Glossy (Grey)')
    colour(design, fl, 'Paint - Enamel Glossy (Yellow)')
    colour(design, wg, 'Paint - Enamel Glossy (Blue)')
    app.activeViewport.fit()

    return ('Harmonic drive built: m = %.2f mm, z_f = %d, z_c = %d, the wave generator at %.1f deg.\n\n'
            'Ratio %.0f with the circular spline held (the flexspline turns the other way, 1/%d of the input)\n'
            'r_f = %.3f, r_c = %.3f, w0 = %.3f mm\n'
            'Teeth: straight flanks at %.0f deg, addendum %.3f, dedendum %.3f, pitch thickness %.4f mm\n\n'
            'Turn it with turn_to(phi): the flexspline is built bent at phi and turned by %.4f deg per input deg.\n\n[%s]'
            % (m, k['z_f'], k['z_c'], phi, k['ratio'], k['z_f'] // 2, k['r_f'], k['r_c'], k['w0'], ALPHA, k['ha'],
               k['hf'], k['s'], -1.0 / (k['z_f'] / 2.0), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(m=ins.itemById('m').value / CM, z_f=ins.itemById('z_f').value, dz=ins.itemById('dz').value,
                     b=ins.itemById('b').value / CM, t=ins.itemById('t').value / CM,
                     phi=math.degrees(ins.itemById('phi').value))
            ui.messageBox(build(**p), VERSION)
        except ValueError as ex:
            ui.messageBox(str(ex), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('m', 'Module', 'mm', VI(0.3 * CM))
        ins.addIntegerSpinnerCommandInput('z_f', 'Flexspline teeth', 20, 400, 1, 200)
        ins.addIntegerSpinnerCommandInput('dz', 'Tooth difference z_c - z_f', 1, 4, 1, 2)
        ins.addValueInput('b', 'Face width', 'mm', VI(8.0 * CM))
        ins.addValueInput('t', 'Flexspline wall', 'mm', VI(0.6 * CM))
        ins.addValueInput('phi', 'Wave generator angle', 'deg', VI(0.0))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'HarmonicDriveCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Harmonic drive', 'A cup-type harmonic drive')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Harmonic drive could not start:\n' + traceback.format_exc())
