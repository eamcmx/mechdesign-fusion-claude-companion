# =============================================================================
# WormWheel.py  -  a worm driving a worm wheel on shafts at right angles, assembled and ready to turn with Drive
# Joints. A command dialog takes the axial module m, the diameter factor q = d1 / m, the wheel's teeth z2, the axial
# pressure angle alpha_x, the wheel's face width b and the worm's number of starts; everything else is derived.
#
#   worm    Archimedean thread, straight-sided in the axial section, its section square to the axis (a lobe between
#           two spirals, one closed polyline) swept along the axis with a twist of 5 turns; 5 leads long
#   wheel   involute spur teeth in the transverse plane at alpha_x, one tooth swept with the twist theta over the face
#           width (a helical gear of the worm's lead angle), then patterned z2 times
#   joints  As-built revolute joints worm/worm shaft and wheel/wheel shaft, and a reversed Motion Link at the ratio
#
# World frame: the wheel's axis is +Z, its mid-plane z = 0; the worm's axis is along +X at y = -a, in that mid-plane.
# Sizes in mm and angles in degrees in this file; the Fusion API is given cm and radians.
# =============================================================================
import importlib
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    """The first folder above this script that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % os.path.dirname(os.path.abspath(__file__)))
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP          # noqa: E402  (puts every lib/ folder on sys.path)
import mdfc_parts as MP               # noqa: E402  (the parts library: temporary bodies into components)
importlib.reload(MP)

VERSION = 'WormWheel v1.0 (prompt test 2026-11-12)'
CMD_ID = 'mdfcWormWheelCmd'
TITLE = 'Worm and worm wheel'

CM = 0.1                 # mm to cm
LEADS = 5                # the worm is 5 leads long and its thread twists 5 turns
SHAFT_D = 8.0            # both shafts
BORE_D = SHAFT_D + 0.2   # each bore 0.2 mm larger than its shaft
SHAFT_OUT = 10.0         # how far each shaft stands out of its part, each side
BACKLASH = 0.1           # the worm's thread is pi m / 2 - 0.1 thick on its pitch line
SINK = 0.25              # sections close this far below the root, inside the core they join
F_LOW, F_HIGH = 0.05, 0.10

P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VS = adsk.core.ValueInput.createByString

_handlers = []
_state = {}
_STANDALONE = __name__ != 'mdfc_check'     # a check loads this file as a namespace: it must not end the check


def inv(a):
    return math.tan(a) - a


# ---------------------------------------------------------------- the design calculation (mm, radians)
def worm(m=2.0, q=10.0, z2=30, alpha_deg=20.0, starts=1, b=8.0):
    """Every derived value of the pair. Raises ValueError for a pair that must be refused."""
    m, q, alpha_deg, b = float(m), float(q), float(alpha_deg), float(b)
    z2, starts = int(round(z2)), int(round(starts))
    if m <= 0 or q <= 0 or b <= 0:
        raise ValueError('m, q and b must be positive')
    if starts < 1:
        raise ValueError('the worm needs at least one start')
    if not 0.0 < alpha_deg < 45.0:
        raise ValueError('alpha_x = %g deg: it must lie between 0 and 45 deg' % alpha_deg)
    if z2 < 18:
        raise ValueError('z2 = %d: a wheel of fewer than 18 teeth would be undercut' % z2)
    lam = math.atan(starts / q)
    if math.degrees(lam) > 25.0:
        raise ValueError('Lead angle %.1f deg is above 25 deg (starts / q = %d / %g)' % (math.degrees(lam), starts, q))
    ax = math.radians(alpha_deg)
    d1 = q * m
    r1 = d1 / 2.0
    px = math.pi * m                      # axial pitch
    L = starts * math.pi * m              # lead
    d2 = m * z2
    r2 = d2 / 2.0
    a = (d1 + d2) / 2.0
    ratio = z2 / starts
    pn = math.atan(math.tan(ax) * math.cos(lam))
    cp, tl = math.cos(pn), math.tan(lam)

    def eff(f):                            # driving from the worm
        return (cp - f * tl) / (cp + f / tl)

    def back(f):                           # back-driving from the wheel
        return (cp - f / tl) / (cp + f * tl)

    theta = b * tl / r2                    # the wheel tooth's turn over the face width
    theta_deg = round(math.degrees(theta), 1)
    if theta_deg <= 0.0:
        theta_deg = 0.1
    bp = math.radians(theta_deg) * r2 / tl  # the face width that makes theta a whole number of tenths
    w = dict(m=m, q=q, z2=z2, alpha_deg=alpha_deg, ax=ax, starts=starts, b=b,
             d1=d1, r1=r1, px=px, L=L, lam=lam, d2=d2, r2=r2, a=a, ratio=ratio, phi_n=pn,
             eff05=eff(F_LOW), eff10=eff(F_HIGH), back05=back(F_LOW), back10=back(F_HIGH), f_lock=cp * tl,
             theta_deg=theta_deg, bp=bp, length=LEADS * L,
             ra1=r1 + m, rf1=r1 - 1.25 * m, s1=px / 2.0 - BACKLASH,
             ra2=r2 + m, rf2=r2 - 1.25 * m, rb2=r2 * math.cos(ax))
    # the geometry must be buildable
    if hw(w, w['ra1']) <= 0.05:
        raise ValueError('the worm thread would come to a point below its tip (alpha_x too large for m)')
    if px - 2.0 * hw(w, w['rf1'] - SINK) <= 0.05:
        raise ValueError('the worm thread would fill its own root (alpha_x too large for m)')
    if w['rf1'] - SINK <= BORE_D / 2.0 + 0.5:
        raise ValueError('the worm root diameter %.2f mm leaves no wall round the %.1f mm bore'
                         % (2 * w['rf1'], BORE_D))
    if tooth_half(w, w['ra2']) <= 0.0:
        raise ValueError('the wheel teeth would come to a point below their tip')
    return w


def hw(w, rho):
    """Half the thickness of the worm's axial profile at the radius rho (straight flanks at alpha_x)."""
    return w['s1'] / 2.0 - (rho - w['r1']) * math.tan(w['ax'])


def tooth_half(w, r):
    """Half the angular thickness of a wheel tooth at the radius r (involute above the base circle, radial below)."""
    rb = w['rb2']
    rr = max(r, rb)
    half_p = (math.pi * w['m'] / 2.0) / (2.0 * w['r2'])
    return half_p + inv(w['ax']) - inv(math.acos(rb / rr))


def _polar(r, t):
    return (r * math.cos(t), r * math.sin(t))


def _dedupe(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-6:
            out.append(p)
    if math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= 1e-6:
        out.pop()
    return out


def lobe_points(w, n=16, n_tip=6, n_root=14):
    """The worm thread's section square to its axis, centred on the angle 0: at radius rho it fills
    |phi| <= 2 pi hw(rho) / L. Two spirals, the tip arc and an arc just inside the root; one closed polyline."""
    L, ra = w['L'], w['ra1']
    r_in = w['rf1'] - SINK

    def c(rho):
        return 2.0 * math.pi * hw(w, rho) / L
    pts = []
    for i in range(n + 1):                                   # one flank, outwards
        rho = r_in + (ra - r_in) * i / n
        pts.append(_polar(rho, c(rho)))
    for j in range(1, n_tip):                                # across the tip
        pts.append(_polar(ra, c(ra) * (1.0 - 2.0 * j / n_tip)))
    for i in range(n, -1, -1):                               # the other flank, inwards
        rho = r_in + (ra - r_in) * i / n
        pts.append(_polar(rho, -c(rho)))
    for j in range(1, n_root):                               # back under the root, through the angle 0
        pts.append(_polar(r_in, -c(r_in) + 2.0 * c(r_in) * j / n_root))
    return _dedupe(pts)


def tooth_points(w, n=12, n_tip=5):
    """One wheel tooth in the transverse plane, centred on the angle 0, closed just below the root circle."""
    ra, rb = w['ra2'], w['rb2']
    r_in = w['rf2'] - SINK
    r_lo = max(rb, r_in)
    pts = [_polar(r_in, -tooth_half(w, r_lo))]
    for i in range(n + 1):                                   # one flank, outwards (radial below the base circle)
        r = r_lo + (ra - r_lo) * i / n
        pts.append(_polar(r, -tooth_half(w, r)))
    for j in range(1, n_tip):                                # across the tip
        h = tooth_half(w, ra)
        pts.append(_polar(ra, -h + 2.0 * h * j / n_tip))
    for i in range(n, -1, -1):                               # the other flank, inwards
        r = r_lo + (ra - r_lo) * i / n
        pts.append(_polar(r, tooth_half(w, r)))
    pts.append(_polar(r_in, tooth_half(w, r_lo)))
    return _dedupe(pts)


def _rotated(pts, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def message(w):
    def lock(v):
        return ' (self-locking)' if v <= 0 else ''
    return '\n'.join([
        '%s  -  %s' % (TITLE, VERSION),
        'Worm: d1 = %.3f mm, lead L = %.4f mm, lead angle lambda = %.4f deg, %d start%s, %.3f mm long (%d leads)'
        % (w['d1'], w['L'], math.degrees(w['lam']), w['starts'], '' if w['starts'] == 1 else 's', w['length'], LEADS),
        'Wheel: d2 = %.3f mm, z2 = %d, face width b\' = %.3f mm (b = %.3f mm asked; twist theta = %.1f deg)'
        % (w['d2'], w['z2'], w['bp'], w['b'], w['theta_deg']),
        'Centre distance a = %.3f mm, ratio %g : 1' % (w['a'], w['ratio']),
        'Normal pressure angle phi_n = %.3f deg' % math.degrees(w['phi_n']),
        'Efficiency driving from the worm: %.3f at f = %.2f, %.3f at f = %.2f'
        % (w['eff05'], F_LOW, w['eff10'], F_HIGH),
        'Back-driving from the wheel: %.3f at f = %.2f%s, %.3f at f = %.2f%s'
        % (w['back05'], F_LOW, lock(w['back05']), w['back10'], F_HIGH, lock(w['back10'])),
        'Self-locking above f = %.4f' % w['f_lock'],
        'One worm turn about +X turns the wheel %.4f deg about +Z.' % (-360.0 / w['ratio']),
    ])


# ---------------------------------------------------------------- Fusion
def new_design():
    """A new Fusion design document (not a part design), parametric, shown in mm."""
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    try:
        if design.designIntent == adsk.fusion.DesignIntentTypes.PartDesignIntentType:
            design.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    except Exception:
        pass
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    return design


def _frame(origin, xdir, zdir):
    """A transform whose local z lies along `zdir` (the part's axis), its origin at `origin` (mm)."""
    z = V3(*zdir)
    x = V3(*xdir)
    y = z.crossProduct(x)
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P3(*[v * CM for v in origin]), x, y, z)
    return mat


def _occurrence(root, name, mat, grounded):
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    occ.isGrounded = grounded
    return occ


def _one_body(comp):
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s has %d bodies, not one' % (comp.name, comp.bRepBodies.count))
    return comp.bRepBodies.item(0)


def _inside(comp, p):
    """Is the point p (mm, in the component's own frame) inside the component's body?"""
    pc = _one_body(comp).pointContainment(P3(p[0] * CM, p[1] * CM, p[2] * CM))
    return pc == adsk.fusion.PointContainment.PointInsidePointContainment


def _twisted_sweep(comp, pts, z0, z1, twist_deg, body, name):
    """A closed polyline (x, y in mm) on the plane z = z0 square to the component's z axis, swept along that axis to
    z1 with a twist, joined to `body`."""
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z0 * CM))
    plane = comp.constructionPlanes.add(ci)
    sk = comp.sketches.add(plane)
    sk.name = name + ' section'
    sk.isComputeDeferred = True
    model = [sk.modelToSketchSpace(P3(x * CM, y * CM, z0 * CM)) for x, y in pts]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(model[0], model[1])
    prev = first.endSketchPoint
    for p in model[2:]:
        prev = lines.addByTwoPoints(prev, p).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    if sk.profiles.count < 1:
        raise RuntimeError('the section of %s did not close' % name)
    prof = max((sk.profiles.item(i) for i in range(sk.profiles.count)), key=lambda pr: pr.areaProperties().area)
    sp = comp.sketches.add(comp.xZConstructionPlane)
    sp.name = name + ' path'
    axis_line = sp.sketchCurves.sketchLines.addByTwoPoints(sp.modelToSketchSpace(P3(0, 0, z0 * CM)),
                                                           sp.modelToSketchSpace(P3(0, 0, z1 * CM)))
    path = comp.features.createPath(axis_line, False)
    sweeps = comp.features.sweepFeatures
    si = sweeps.createInput(prof, path, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    si.twistAngle = VS('%s deg' % _num(twist_deg))
    si.participantBodies = [body]
    feat = sweeps.add(si)
    feat.name = name
    return feat


def _num(v):
    return ('%.6f' % v).rstrip('0').rstrip('.')


def _set_hand(comp, feat, twist_deg, probe):
    """Fusion's sign for a twist is checked on the body, not assumed: `probe` (mm, component frame) lies inside the
    swept section only when it turns the right-handed way. If it does not, the twist is reversed."""
    if _inside(comp, probe):
        return
    feat.twistAngle.expression = '%s deg' % _num(-twist_deg)
    if not _inside(comp, probe):
        raise RuntimeError('could not make %s right-handed' % feat.name)


def _pattern(comp, feat, n):
    col = adsk.core.ObjectCollection.create()
    col.add(feat)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(col, comp.zConstructionAxis)
    pi.quantity = adsk.core.ValueInput.createByReal(n)
    pi.totalAngle = VS('360 deg')
    pi.isSymmetric = False
    return pats.add(pi)


def _revolute(root, occ_move, occ_shaft, name):
    """An as-built revolute joint on a sketch point at the moving part's own origin, about its own z axis."""
    comp = occ_move.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = 'Joint origin'
    pt = sk.sketchPoints.add(sk.modelToSketchSpace(P3(0, 0, 0)))
    geo = adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(occ_move))
    ji = root.asBuiltJoints.createInput(occ_move, occ_shaft, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def build(m=2.0, q=10.0, z2=30, alpha=20.0, b=8.0, starts=1, design=None):
    """Build the pair in `design` (a new design document when None) and return the message."""
    w = worm(m, q, z2, alpha, starts, b)              # refuses before anything is made
    if design is None:
        design = new_design()
    root = design.rootComponent
    a, length, bp = w['a'], w['length'], w['bp']
    z2, starts = w['z2'], w['starts']

    worm_frame = _frame((0.0, -a, 0.0), (0, 1, 0), (1, 0, 0))     # local z = world +X, local x = world +Y
    wheel_frame = _frame((0.0, 0.0, 0.0), (1, 0, 0), (0, 0, 1))   # local z = world +Z
    occ_ws = _occurrence(root, 'Worm shaft', worm_frame, True)
    occ_gs = _occurrence(root, 'Wheel shaft', wheel_frame, True)
    occ_w = _occurrence(root, 'Worm', worm_frame, False)
    occ_g = _occurrence(root, 'Wheel', wheel_frame, False)

    # shafts and the cores of the two parts (world mm, moved into each component's frame by the parts library)
    MP.add_body(occ_ws, MP.cyl((-length / 2 - SHAFT_OUT, -a, 0.0), (length / 2 + SHAFT_OUT, -a, 0.0), SHAFT_D / 2),
                'Worm shaft')
    MP.add_body(occ_gs, MP.cyl((0.0, 0.0, -bp / 2 - SHAFT_OUT), (0.0, 0.0, bp / 2 + SHAFT_OUT), SHAFT_D / 2),
                'Wheel shaft')
    core_w = MP.add_body(occ_w, MP.tube((-length / 2, -a, 0.0), (length / 2, -a, 0.0), w['rf1'], BORE_D / 2), 'Worm')
    core_g = MP.add_body(occ_g, MP.tube((0.0, 0.0, -bp / 2), (0.0, 0.0, bp / 2), w['rf2'], BORE_D / 2), 'Wheel')

    # the worm's thread: in the worm's frame the lobe's centre lies at the angle 2 pi z / L (right hand), so at z = 0
    # it faces the wheel (local +x = world +Y)
    cw = occ_w.component
    L, r1 = w['L'], w['r1']
    z0 = -length / 2.0
    thread = _twisted_sweep(cw, _rotated(lobe_points(w), 2 * math.pi * z0 / L), z0, -z0, LEADS * 360.0, core_w,
                            'Thread')
    zt = z0 + w['px'] / 4.0
    _set_hand(cw, thread, LEADS * 360.0, (r1 * math.cos(2 * math.pi * zt / L), r1 * math.sin(2 * math.pi * zt / L), zt))
    if starts > 1:
        _pattern(cw, thread, starts)
    for k in range(starts):                                  # each start's thread at z = 0, its gaps between
        t = 2 * math.pi * k / starts
        if not _inside(cw, (r1 * math.cos(t), r1 * math.sin(t), 0.0)):
            raise RuntimeError('the worm thread is not where it should be')
        if _inside(cw, (r1 * math.cos(t + math.pi / starts), r1 * math.sin(t + math.pi / starts), 0.0)):
            raise RuntimeError('the worm thread fills its own gap')

    # the wheel's tooth: at the mid-plane a gap faces the worm (-Y), so a tooth is centred half a pitch from it; over
    # the face width the tooth turns by +theta about +Z (right hand), its centre at tm + theta z / b'
    cg = occ_g.component
    r2 = w['r2']
    th = math.radians(w['theta_deg'])
    tm = -math.pi / 2 + math.pi / z2
    tooth = _twisted_sweep(cg, _rotated(tooth_points(w), tm - th / 2), -bp / 2, bp / 2, w['theta_deg'], core_g,
                           'Tooth')
    zt = bp / 2 - min(0.1, bp / 4)
    f = (zt + bp / 2) / bp
    hp = math.pi / (2 * z2)
    t = tm + th * zt / bp + hp - min(0.5 * f * th, hp)       # inside if right-handed, outside if not
    _set_hand(cg, tooth, w['theta_deg'], (r2 * math.cos(t), r2 * math.sin(t), zt))
    _pattern(cg, tooth, z2)
    for k in range(z2):
        t = tm + 2 * math.pi * k / z2
        if not _inside(cg, (r2 * math.cos(t), r2 * math.sin(t), 0.0)):
            raise RuntimeError('wheel tooth %d is missing' % (k + 1))
    if _inside(cg, (0.0, -r2, 0.0)):
        raise RuntimeError('the wheel has no gap facing the worm')

    # joints: positive values turn the worm anticlockwise about +X and the wheel about +Z; the right-hand worm turning
    # about +X pushes the wheel about -Z, hence the reversed link
    jw = _revolute(root, occ_w, occ_ws, 'Worm turns')
    jg = _revolute(root, occ_g, occ_gs, 'Wheel turns')
    ml = root.motionLinks.createInput(jw, jg)
    try:
        ml.motionOne = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
        ml.motionTwo = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
    except Exception:
        pass
    ml.valueOne = VS('360 deg')
    ml.valueTwo = VS('%.10g deg' % (360.0 / w['ratio']))
    ml.isReversed = True
    link = root.motionLinks.add(ml)
    try:
        link.name = 'Worm drives wheel'
    except Exception:
        pass
    try:
        adsk.core.Application.get().activeViewport.fit()
    except Exception:
        pass
    return message(w)


# ---------------------------------------------------------------- the dialog
class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            ins = cmd.commandInputs
            ins.addValueInput('m', 'Axial module m', 'mm', VS('2 mm'))
            ins.addValueInput('q', 'Diameter factor q = d1 / m', '', VS('10'))
            ins.addIntegerSpinnerCommandInput('z2', 'Wheel teeth z2', 1, 1000, 1, 30)
            ins.addValueInput('alpha', 'Axial pressure angle alpha_x', 'deg', VS('20 deg'))
            ins.addValueInput('b', 'Wheel face width b', 'mm', VS('8 mm'))
            ins.addIntegerSpinnerCommandInput('starts', 'Number of starts of the worm', 1, 10, 1, 1)
            on_execute = _Execute()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            on_destroy = _Destroy()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            ui.messageBox('Failed:\n%s' % traceback.format_exc(), TITLE)


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            ins = args.command.commandInputs
            values = dict(m=ins.itemById('m').value / CM,
                          q=ins.itemById('q').value,
                          z2=int(ins.itemById('z2').value),
                          alpha=math.degrees(ins.itemById('alpha').value),
                          b=ins.itemById('b').value / CM,
                          starts=int(ins.itemById('starts').value))
            design = _state.get('design')
            if design is None or not design.isValid:
                design = adsk.fusion.Design.cast(app.activeProduct)
            msg = build(design=design, **values)
            ui.messageBox(msg, TITLE)
        except ValueError as e:
            ui.messageBox('Refused: %s' % e, TITLE)
        except Exception:
            ui.messageBox('Failed:\n%s' % traceback.format_exc(), TITLE)


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _STANDALONE:
            adsk.terminate()


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        # A document cannot be created inside a command's events, so the new design is opened first and the dialog's
        # OK builds into it.
        _state['design'] = new_design()
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, TITLE, 'A worm driving a worm wheel, ready to turn')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        if _STANDALONE:
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n%s' % traceback.format_exc(), TITLE)


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
