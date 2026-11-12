# =============================================================================
# HarmonicDrive.py - a cup-type harmonic drive (strain wave gear) for Autodesk Fusion: the rigid circular spline,
# the flexspline bent into an oval by the wave generator at the angle phi, and the wave generator, drawn with its
# flexible bearing as one body.
#
# A rigid body cannot bend, so the drive is not turned by a Motion Link: turn_to(phi) deletes the extrusions,
# sketches and planes of the flexspline and the wave generator and builds them again, bent at the new angle, the
# flexspline turned as a whole by rho = -phi (z_c - z_f) / z_f.
#
#   harmonic(...)   the derived sizes (no Fusion needed); refuses what the prompt refuses
#   build(...)      a new design with the three components and their two as-built revolute joints
#   turn_to(phi)    the flexspline and wave generator rebuilt at phi; returns rho in degrees
#   run(context)    the dialog
#
# Sizes in mm and degrees here; the Fusion API works in cm and radians (CM below).
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
            raise RuntimeError('no common/companion_paths.py in any folder above ' + here)
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402,F401  (every lib/ on sys.path; this script needs no library)

VERSION = 'HarmonicDrive 1.0 (2026-11-12)'

CM = 0.1                       # mm to cm
FLANK_DEG = 30.0               # straight flanks at 30 deg
ADD = 0.75                     # addendum, in modules
DED = 0.90                     # dedendum, in modules: 0.15 m beyond the partner's tip
BACKLASH = 0.06                # mm: thickness on the pitch line pi m / 2 - 0.03
WG_GAP = 0.03                  # mm: the wave generator's outline inside the flexspline's bore
RING = 4.0                     # mm of circular spline outside its root line
Z_WALL = -20.0                 # the flexspline's bent wall runs down to here
Z_FLOOR = -22.0                # the underside of the diaphragm and the output hub
SHAFT_D = 10.0                 # the wave generator's input shaft
SHAFT_L = 12.0
N_ROUND = 360                  # points on each round curve (bore, root line, wave generator)
DEFAULTS = dict(m=0.3, z_f=200, dz=2, b=8.0, t=0.6, phi=0.0)

_handlers = []
_last = {}                     # what the last build made: the design, occurrences, inputs, and its timeline items
_standalone = False            # run by Fusion (with a context) rather than called by another script


# ---------------------------------------------------------------- design calculations (no Fusion) ------------
def harmonic(m=0.3, z_f=200, dz=2, b=8.0, t=0.6, phi=0.0):
    """Every derived size of the drive, in mm and degrees. Raises ValueError for what the prompt refuses."""
    if dz != 2:
        raise ValueError('Refused: a two-lobed wave generator needs a tooth difference of 2 '
                         '(z_c - z_f = %s given).' % dz)
    if z_f != int(z_f) or not 60 <= z_f <= 320:
        raise ValueError('Refused: the flexspline teeth z_f must be a count from 60 to 320 (%s given).' % z_f)
    for name, val in (('module m', m), ('face width b', b), ('wall under the teeth t', t)):
        if not val > 0:
            raise ValueError('Refused: the %s must be positive (%s mm given).' % (name, val))
    z_f, dz = int(z_f), int(dz)
    z_c = z_f + dz
    r_f = m * z_f / 2.0
    r_c = m * z_c / 2.0
    w0 = m * dz / 2.0                          # the push, so that the bent pitch line reaches r_c on the long axis
    ha, hf = ADD * m, DED * m
    p = math.pi * m                            # pitch on the pitch line (the same on both parts)
    thick = math.pi * m / 2.0 - BACKLASH / 2.0
    tan_a = math.tan(math.radians(FLANK_DEG))
    half_tip = thick / 2.0 - ha * tan_a        # half widths of a tooth at its tip and at its root
    half_root = thick / 2.0 + hf * tan_a
    tip_w = 2.0 * half_tip
    land = p - 2.0 * half_root                 # root land between two teeth
    if not tip_w > 0:
        raise ValueError('Refused: the flexspline teeth come to a point (tip width %.4f mm at m = %s mm).'
                         % (tip_w, m))
    if not land > 0:
        raise ValueError('Refused: the flanks of neighbouring teeth meet below the root (land %.4f mm).' % land)
    rho = -phi * dz / z_f + 0.0                # the flexspline turned as a whole, degrees (+ 0.0: no -0)
    return dict(m=m, z_f=z_f, dz=dz, b=b, t=t, phi=phi, z_c=z_c,
                ratio=-z_f / dz, out_ratio=dz / z_f,
                r_f=r_f, r_c=r_c, w0=w0, ha=ha, hf=hf, p=p, thick=thick, half_tip=half_tip,
                half_root=half_root, tip_w=tip_w, land=land, rho=rho,
                r_out=r_c + hf + RING,                   # circular spline's outside radius
                r_dia=r_f - hf,                          # round diaphragm: the flexspline's root line before bending
                r_hub=(r_f - hf) / 3.0,                  # output hub, in the middle of the diaphragm
                phi_rad=math.radians(phi), rho_rad=math.radians(rho))


def bent(k, theta, n):
    """The point at material angle theta (rad, on the flexspline already turned by rho), carried n mm along the
    outward normal of the bent pitch line. With psi = theta - phi the pitch line moves out by w = w0 cos 2 psi and
    along by v = -(w0 / 2) sin 2 psi (inextensible: dv/dtheta = -w)."""
    psi = theta - k['phi_rad']
    s2, c2 = math.sin(2.0 * psi), math.cos(2.0 * psi)
    r_f, w0 = k['r_f'], k['w0']
    w = w0 * c2
    v = -0.5 * w0 * s2
    a = -2.0 * w0 * s2 - v                     # radial part of d(point)/d(theta); its tangential part is r_f
    c, s = math.cos(theta), math.sin(theta)
    L = math.hypot(r_f, a)
    x = (r_f + w) * c - v * s
    y = (r_f + w) * s + v * c
    nx = (r_f * c + a * s) / L
    ny = (r_f * s - a * c) / L
    return x + n * nx, y + n * ny


def flexspline_outline(k):
    """The bent rim's toothed outline, five points a tooth, counter-clockwise: root corner, two tip corners, root
    corner, middle of the root land. Tooth 0 is centred at material angle 0 (turned by rho)."""
    ha, hf, ht, hr, p, r_f = k['ha'], k['hf'], k['half_tip'], k['half_root'], k['p'], k['r_f']
    form = ((-hr, -hf), (-ht, ha), (ht, ha), (hr, -hf), (p / 2.0, -hf))      # (along the pitch line, out of it)
    pts = []
    for j in range(k['z_f']):
        th = 2.0 * math.pi * j / k['z_f'] + k['rho_rad']
        for s_, n_ in form:
            pts.append(bent(k, th + s_ / r_f, n_))
    return pts


def circular_outline(k):
    """The circular spline's internal teeth, five points a tooth, counter-clockwise: tooth k centred at
    360 (k + 1/2) / z_c deg, so that a space faces 0 deg."""
    ha, hf, ht, hr, p, r_c = k['ha'], k['hf'], k['half_tip'], k['half_root'], k['p'], k['r_c']
    form = ((-hr, hf), (-ht, -ha), (ht, -ha), (hr, hf), (p / 2.0, hf))       # n outward: the tips point inward
    pts = []
    for i in range(k['z_c']):
        c = 2.0 * math.pi * (i + 0.5) / k['z_c']
        for s_, n_ in form:
            a, r = c + s_ / r_c, r_c + n_
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def round_curve(k, n, count=N_ROUND):
    """A curve bent the same way as the pitch line, n mm off it along its normal (bore, root line, wave generator)."""
    return [bent(k, k['phi_rad'] + 2.0 * math.pi * i / count, n) for i in range(count)]


def bore(k):
    return round_curve(k, -(k['hf'] + k['t']))


def root_line(k):
    return round_curve(k, -k['hf'])


def wave_generator_outline(k):
    return round_curve(k, -(k['hf'] + k['t'] + WG_GAP))


def message(k):
    return ('Harmonic drive (cup type), %s\n'
            'z_f = %d, z_c = %d (difference %d, two-lobed wave generator)\n'
            'ratio %.0f with the circular spline held (the flexspline turns %.4f of the wave generator, '
            'the other way)\n'
            'r_f = %.3f mm, r_c = %.3f mm, w0 = %.3f mm\n'
            'tooth form: straight flanks at %.0f deg, addendum %.2f m = %.3f mm, dedendum %.2f m = %.3f mm, '
            'thickness on the pitch line %.4f mm (backlash %.2f mm), tip width %.4f mm\n'
            'face width %.2f mm, wall under the teeth %.2f mm\n'
            'wave generator at phi = %.4f deg, flexspline turned rho = %.4f deg'
            % (VERSION, k['z_f'], k['z_c'], k['dz'], k['ratio'], k['out_ratio'], k['r_f'], k['r_c'], k['w0'],
               FLANK_DEG, ADD, k['ha'], DED, k['hf'], k['thick'], BACKLASH, k['tip_w'], k['b'], k['t'],
               k['phi'], k['rho']))


# ---------------------------------------------------------------- Fusion ---------------------------------------
def _P(x, y, z=0.0):
    return adsk.core.Point3D.create(x * CM, y * CM, z * CM)


def _VI(v_cm):
    return adsk.core.ValueInput.createByReal(v_cm)


def _plane(comp, z, made):
    """The component's XY plane, or a plane offset from it to z (mm), remembered for deletion."""
    if abs(z) < 1e-12:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, _VI(z * CM))
    pl = comp.constructionPlanes.add(ci)
    made.append(pl)
    return pl


def _sketch(comp, plane, name, made):
    sk = comp.sketches.add(plane)
    sk.name = name
    sk.isComputeDeferred = True
    made.append(sk)
    return sk


def _polyline(sk, pts):
    """One closed polyline whose lines share their points (the sketch is on an XY plane at z = 0)."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(_P(*pts[0]), _P(*pts[1]))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, _P(*q)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def _circle(sk, z, r):
    c = sk.modelToSketchSpace(_P(0.0, 0.0, z))
    sk.sketchCurves.sketchCircles.addByCenterRadius(c, r * CM)


def _profiles(sk, loops):
    """The sketch's profiles bounded by this many loops (2: a ring between two curves; 1: inside a single one)."""
    out = [p for p in sk.profiles if p.profileLoops.count == loops]
    if not out:
        raise RuntimeError('sketch %s: no profile with %d loops' % (sk.name, loops))
    return out


def _extrude(comp, sk, profs, z0, z1, made, join=False):
    """Extrudes the profiles of a sketch lying at z0 to z1 (mm): a new body, or joined to the component's body."""
    ext = comp.features.extrudeFeatures
    if len(profs) == 1:
        prof = profs[0]
    else:
        prof = adsk.core.ObjectCollection.create()
        for p in profs:
            prof.add(p)
    op = (adsk.fusion.FeatureOperations.JoinFeatureOperation if join
          else adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp = ext.createInput(prof, op)
    normal = sk.xDirection.crossProduct(sk.yDirection)          # an extrusion goes along the sketch's normal
    inp.setDistanceExtent(False, _VI((z1 - z0) * CM * (1.0 if normal.z > 0 else -1.0)))
    if join:
        inp.participantBodies = [comp.bRepBodies.item(0)]
    f = ext.add(inp)
    made.append(f)
    return f


def _build_circular_spline(comp, k, made):
    sk = _sketch(comp, comp.xYConstructionPlane, 'Circular spline teeth', made)
    _circle(sk, 0.0, k['r_out'])
    _polyline(sk, circular_outline(k))
    sk.isComputeDeferred = False
    f = _extrude(comp, sk, _profiles(sk, 2), 0.0, k['b'], made)
    f.bodies.item(0).name = 'CircularSpline'


def _build_flexspline(comp, k, made):
    xy = comp.xYConstructionPlane
    hole = bore(k)
    # the whole bent rim: all its teeth in one closed polyline, the bore in another, z = 0 to b
    sk = _sketch(comp, xy, 'Flexspline rim at phi %.4f' % k['phi'], made)
    _polyline(sk, flexspline_outline(k))
    _polyline(sk, hole)
    sk.isComputeDeferred = False
    f = _extrude(comp, sk, _profiles(sk, 2), 0.0, k['b'], made)
    f.bodies.item(0).name = 'Flexspline'
    # the wall, from the root line to the bore, bent the same way, down to z = -20
    sk = _sketch(comp, xy, 'Flexspline wall', made)
    _polyline(sk, root_line(k))
    _polyline(sk, hole)
    sk.isComputeDeferred = False
    _extrude(comp, sk, _profiles(sk, 2), 0.0, Z_WALL, made, join=True)
    # the round diaphragm and the output hub, z = -20 to -22
    pl = _plane(comp, Z_WALL, made)
    sk = _sketch(comp, pl, 'Diaphragm and hub', made)
    _circle(sk, Z_WALL, k['r_dia'])
    _circle(sk, Z_WALL, k['r_hub'])
    sk.isComputeDeferred = False
    _extrude(comp, sk, _profiles(sk, 2), Z_WALL, Z_FLOOR, made, join=True)       # diaphragm
    _extrude(comp, sk, _profiles(sk, 1), Z_WALL, Z_FLOOR, made, join=True)       # hub


def _build_wave_generator(comp, k, made):
    sk = _sketch(comp, comp.xYConstructionPlane, 'Wave generator at phi %.4f' % k['phi'], made)
    _polyline(sk, wave_generator_outline(k))
    sk.isComputeDeferred = False
    f = _extrude(comp, sk, _profiles(sk, 1), 0.0, k['b'], made)
    f.bodies.item(0).name = 'WaveGenerator'
    pl = _plane(comp, k['b'], made)
    sk = _sketch(comp, pl, 'Input shaft', made)
    _circle(sk, k['b'], SHAFT_D / 2.0)
    sk.isComputeDeferred = False
    _extrude(comp, sk, _profiles(sk, 1), k['b'], k['b'] + SHAFT_L, made, join=True)


def _new_occ(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())      # at the origin, drawn in place
    occ.component.name = name
    return occ


def _revolute(root, occ, on, name):
    """An as-built revolute joint about z between occ and on, at the origin of `on`."""
    pt = on.component.originConstructionPoint.createForAssemblyContext(on)
    geo = adsk.fusion.JointGeometry.createByPoint(pt)
    inp = root.asBuiltJoints.createInput(occ, on, geo)
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def build(m=0.3, z_f=200, dz=2, b=8.0, t=0.6, phi=0.0):
    """A new design with the drive bent at phi. Returns the message the dialog shows."""
    k = harmonic(m, z_f, dz, b, t, phi)                 # refuses before anything is made
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    cs = _new_occ(root, 'CircularSpline')
    fs = _new_occ(root, 'Flexspline')
    wg = _new_occ(root, 'WaveGenerator')
    cs.isGrounded = True
    _build_circular_spline(cs.component, k, [])
    made = []
    _build_flexspline(fs.component, k, made)
    _build_wave_generator(wg.component, k, made)
    _revolute(root, wg, cs, 'WaveGenerator on CircularSpline')
    _revolute(root, fs, cs, 'Flexspline on CircularSpline')
    _last.clear()
    _last.update(design=des, cs=cs, fs=fs, wg=wg, made=made,
                 args=dict(m=m, z_f=z_f, dz=dz, b=b, t=t, phi=phi))
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return message(k)


def turn_to(phi):
    """Rebuilds the flexspline and the wave generator bent at phi (deg), the flexspline turned by rho; deletes the
    extrusions, sketches and planes the last build made first, so the timeline does not grow. Returns rho (deg)."""
    if not _last:
        raise RuntimeError('turn_to: build the drive first')
    args = dict(_last['args'], phi=phi)
    k = harmonic(**args)
    for item in reversed(_last['made']):
        if item is not None and item.isValid:
            item.deleteMe()
    made = []
    _build_flexspline(_last['fs'].component, k, made)
    _build_wave_generator(_last['wg'].component, k, made)
    _last['made'] = made
    _last['args'] = args
    return k['rho']


# ---------------------------------------------------------------- the dialog -----------------------------------
CMD_ID = 'mdfcHarmonicDriveCmd'


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        VS = adsk.core.ValueInput.createByString
        d = DEFAULTS
        ins.addValueInput('m', 'Module m', 'mm', VS('%s mm' % d['m']))
        ins.addIntegerSpinnerCommandInput('z_f', 'Flexspline teeth z_f', 1, 1000, 1, d['z_f'])
        ins.addIntegerSpinnerCommandInput('dz', 'Tooth difference z_c - z_f', 0, 20, 1, d['dz'])
        ins.addValueInput('b', 'Face width b', 'mm', VS('%s mm' % d['b']))
        ins.addValueInput('t', 'Flexspline wall under the teeth t', 'mm', VS('%s mm' % d['t']))
        ins.addValueInput('phi', 'Wave generator angle phi', 'deg', VS('%s deg' % d['phi']))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        h = _Destroy()
        cmd.destroy.add(h)
        _handlers.append(h)


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = dict(m=round(ins.itemById('m').value / CM, 9),
                        z_f=ins.itemById('z_f').value,
                        dz=ins.itemById('dz').value,
                        b=round(ins.itemById('b').value / CM, 9),
                        t=round(ins.itemById('t').value / CM, 9),
                        phi=round(math.degrees(ins.itemById('phi').value), 9))
            ui.messageBox(build(**vals))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('HarmonicDrive failed:\n' + traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _standalone:
            adsk.terminate()


def run(context):
    global _standalone
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        _standalone = context is not None
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Harmonic drive',
                                                       'A cup-type harmonic drive (strain wave gear)')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if _standalone:
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('HarmonicDrive failed:\n' + traceback.format_exc())


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
