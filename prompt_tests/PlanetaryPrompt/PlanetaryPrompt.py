# -*- coding: utf-8 -*-
# =============================================================================
# PlanetaryPrompt.py  -  a parametric planetary gearbox for Autodesk Fusion, written from its prompt.
#
# Run it: a dialog asks for the module, the tooth counts, the number of planets and the sizes of the shafts,
# pins and carrier. OK opens a new design with Ring (grounded), Sun, Planet 1 .. Planet N, Carrier and Sun
# bearing, one body each, joined by As-built Joints and tied by Motion Links, so that ASSEMBLE > Drive Joints on
# "Sun (input)" turns the planets round the ring and turns the carrier.
#
# Every size below is in millimetres and every angle in degrees until it reaches the API, which takes cm and
# radians (MM converts).
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _find_companion():
    """Walk up from this script's folder to the first one holding common/companion_paths.py, put common/ on
    sys.path and import it. This script needs no library of the companion; the walk is kept so that it finds
    them the way every companion script does."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            common = os.path.join(d, 'common')
            if common not in sys.path:
                sys.path.insert(0, common)
            import companion_paths
            return companion_paths
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


try:
    CP = _find_companion()
except Exception:
    CP = None

VERSION = 'PlanetaryPrompt v1.0'
TITLE = 'Planetary gearbox'
CMD_ID = 'mdfc_PlanetaryPrompt_cmd'

MM = 0.1                                  # cm in one mm
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

DEFAULTS = dict(m=1.5, zs=16, zp=20, n=3, alpha=20.0, b=12.0, backlash=0.3, pin_d=6.0, sun_d=8.0, out_d=12.0,
                plate=6.0, wall=6.0)

SHAFT_OUT = 20.0                          # mm: the input shaft leaves 20 mm below the gears, the output 20 mm above
_handlers = []


# ----------------------------------------------------------------------------- design calculations (mm, rad)
def _inv(x):
    return math.tan(x) - x


def _polar(r, ang):
    return (r * math.cos(ang), r * math.sin(ang))


def tooth_outline(m, z, alpha, s, phase, ra, rf, n_flank=12, n_tip=5):
    """One outside tooth of a z-tooth gear (module m, pressure angle alpha), centred on the angle `phase`: arc
    thickness s on the pitch circle, tip radius ra, root radius rf. True involute flanks from the base circle (from
    the root circle where that is above the base circle); where the base circle is above the root circle the flank
    goes on as a radial line down to the root circle. The radial line is carried 0.25 m below the root circle and
    the outline closed there, so that a tooth overlaps the disc it is joined to and a tooth space runs out into
    the ring's hole. Returns the closed outline as a list of (x, y) in mm, the closing line implied."""
    r = m * z / 2.0
    rb = r * math.cos(alpha)
    half_base = s / (2.0 * r) + _inv(alpha)          # half the tooth's angle at the base circle

    def beta(rho):                                   # half the tooth's angle at the radius rho
        return half_base if rho <= rb else half_base - _inv(math.acos(rb / rho))

    if beta(ra) <= 0.0:
        raise ValueError('the teeth of a %d-tooth gear come to a point below their tip circle' % z)
    r_lo = max(rb, rf)
    r_close = rf - 0.25 * m
    t_lo = math.sqrt(max((r_lo / rb) ** 2 - 1.0, 0.0))
    t_hi = math.sqrt((ra / rb) ** 2 - 1.0)
    radii = [rb * math.sqrt(1.0 + (t_lo + (t_hi - t_lo) * k / n_flank) ** 2) for k in range(n_flank + 1)]
    radii[0], radii[-1] = r_lo, ra
    b_lo, b_tip = beta(r_lo), beta(ra)
    pts = [_polar(r_close, -b_lo)]                                   # radial line up to the flank
    pts += [_polar(rho, -beta(rho)) for rho in radii]                # one flank, root to tip
    pts += [_polar(ra, -b_tip + 2.0 * b_tip * j / (n_tip + 1)) for j in range(1, n_tip + 1)]   # the tip
    pts += [_polar(rho, beta(rho)) for rho in reversed(radii)]       # the other flank, tip to root
    pts += [_polar(r_close, b_lo)]                                   # radial line down; the chord closes it
    c, sn = math.cos(phase), math.sin(phase)
    return [(x * c - y * sn, x * sn + y * c) for x, y in pts]


def derive(m=1.5, zs=16, zp=20, n=3, alpha=20.0, b=12.0, backlash=0.3, pin_d=6.0, sun_d=8.0, out_d=12.0,
           plate=6.0, wall=6.0):
    """Everything computed from the inputs, or ValueError when the gearbox cannot be built."""
    zs, zp, n = int(round(zs)), int(round(zp)), int(round(n))
    if min(m, b, pin_d, sun_d, out_d, plate, wall) <= 0 or backlash < 0:
        raise ValueError('every size must be positive (the backlash may be zero)')
    if zs < 6 or zp < 6 or n < 1:
        raise ValueError('the sun and the planets need at least 6 teeth, and there must be at least one planet')
    zr = zs + 2 * zp
    if (zs + zr) % n:
        raise ValueError('(Zs + Zr)/N = %d/%d = %.3f is not whole: %d planets cannot be spaced evenly'
                         % (zs + zr, n, (zs + zr) / float(n), n))
    a = m * (zs + zp) / 2.0
    if n > 1 and not 2.0 * a * math.sin(math.pi / n) > m * (zp + 2):
        raise ValueError('neighbouring planets touch: 2*a*sin(pi/N) = %.3f mm is not more than m*(Zp + 2) = %.3f mm'
                         % (2.0 * a * math.sin(math.pi / n), m * (zp + 2)))
    al = math.radians(alpha)
    g = dict(m=m, zs=zs, zp=zp, zr=zr, n=n, alpha=alpha, al=al, b=b, backlash=backlash, pin_d=pin_d,
             sun_d=sun_d, out_d=out_d, plate=plate, wall=wall, a=a)
    g['theta'] = [2.0 * math.pi * i / n for i in range(n)]
    pp = 2.0 * math.pi / zp
    g['psi'] = [((t * (zs + zp) + math.pi * (zp - 1)) / zp) % pp for t in g['theta']]
    g['phi_r'] = 0.0 if zp % 2 else math.pi / zr
    g['ratio_ring_held'] = 1.0 + zr / float(zs)
    g['ratio_carrier_held'] = -zr / float(zs)
    g['ratio_sun_held'] = 1.0 + zs / float(zr)
    s_out = math.pi * m / 2.0 - backlash / 2.0       # outside teeth: half the backlash off each tooth
    s_ring = math.pi * m / 2.0 + backlash / 2.0      # the ring's tooth space: half the backlash added
    g['s_out'], g['s_ring'] = s_out, s_ring
    for key, z in (('sun', zs), ('planet', zp)):
        r = m * z / 2.0
        g[key] = dict(z=z, r=r, rb=r * math.cos(al), ra=r + m, rf=r - 1.25 * m)
    rr = m * zr / 2.0
    g['ring'] = dict(z=zr, r=rr, rb=rr * math.cos(al), r_in=rr - m, r_root=rr + 1.25 * m,
                     r_out=rr + 1.25 * m + wall)
    if s_out <= 0:
        raise ValueError('the backlash is larger than a tooth')
    if pin_d + 0.2 >= 2.0 * g['planet']['rf']:
        raise ValueError('the planet bore (pin + 0.2 mm) is wider than the planet root circle')
    if sun_d >= 2.0 * g['sun']['rf']:
        raise ValueError('the sun shaft is wider than the sun root circle')
    # the outlines (mm), which also refuse pointed teeth
    g['sun_tooth'] = tooth_outline(m, zs, al, s_out, 0.0, g['sun']['ra'], g['sun']['rf'])
    g['planet_teeth'] = [tooth_outline(m, zp, al, s_out, p, g['planet']['ra'], g['planet']['rf']) for p in g['psi']]
    g['ring_space'] = tooth_outline(m, zr, al, s_ring, g['phi_r'], g['ring']['r_root'], g['ring']['r_in'])
    # the axial layout
    g['z_car0'] = b + 1.0                             # the carrier's bottom face
    g['z_car1'] = b + 1.0 + plate                     # its top face
    g['bore_depth'] = plate / 2.0                     # the blind bore from its bottom face
    g['bore_d'] = sun_d + 4.4
    g['sun_top'] = b + 1.0 + plate / 2.0 - 0.5        # the sun shaft ends 0.5 mm short of the bore's end
    g['brg_id'], g['brg_od'] = sun_d + 0.2, sun_d + 4.0
    g['car_r'] = a + pin_d
    return g


def message(g):
    """The closing message: line 0 the version tag, line 1 the gears, line 2 the three ratios."""
    c_turn = 360.0 / g['ratio_ring_held']
    return '\n'.join([
        '%s: planetary gearbox built' % VERSION,
        'm %g mm, Zs %d, Zp %d, Zr %d, N %d, centre distance a %.3f mm'
        % (g['m'], g['zs'], g['zp'], g['zr'], g['n'], g['a']),
        'Ratios: ring held 1 + Zr/Zs = %.4f, carrier held -Zr/Zs = %.4f, sun held 1 + Zs/Zr = %.4f'
        % (g['ratio_ring_held'], g['ratio_carrier_held'], g['ratio_sun_held']),
        'One sun turn: carrier %.3f deg the same way, each planet %.3f deg relative to the carrier'
        % (c_turn, -g['zr'] / float(g['zp']) * c_turn),
        'Drive Joints on "Sun (input)" to watch it turn.',
    ])


# ----------------------------------------------------------------------------- Fusion helpers (mm in, cm out)
def _polyline(sk, pts):
    """The prompt's polyline: one closed loop of sketch lines sharing their SketchPoints (points in cm)."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def _new_occ(root, name, t=(0.0, 0.0, 0.0)):
    """A component made where it belongs: its origin at the translation t (mm), never moved afterwards."""
    mat = adsk.core.Matrix3D.create()
    mat.translation = adsk.core.Vector3D.create(t[0] * MM, t[1] * MM, t[2] * MM)
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def _plane(comp, z):
    """The component's XY plane, or a plane parallel to it at z (mm)."""
    if abs(z) < 1e-9:
        return comp.xYConstructionPlane
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, VI(z * MM))
    return comp.constructionPlanes.add(ci)


def _sketch(comp, plane, name):
    sk = comp.sketches.add(plane)
    sk.name = name
    return sk


def _circle(sk, z, x, y, r):
    """A circle of radius r (mm) centred at (x, y, z) in the component's frame. On the XY plane sketch space is
    the component's own frame; on a parallel plane the point is converted."""
    if abs(z) < 1e-9:
        c = P3(x * MM, y * MM, 0)
    else:
        c = sk.modelToSketchSpace(P3(x * MM, y * MM, z * MM))
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(c.x, c.y, 0), r * MM)


def _annulus(sk):
    for p in sk.profiles:
        if p.profileLoops.count == 2:
            return p
    raise RuntimeError('no annular profile in sketch %s' % sk.name)


def _all_profiles(sk):
    col = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        col.add(p)
    return col


def _extrude(sk, profiles, length, op, body=None):
    """Extrude from the sketch plane by `length` mm: along +z when positive, -z when negative."""
    comp = sk.parentComponent
    normal = sk.xDirection.crossProduct(sk.yDirection)
    up = (length > 0) == (normal.z > 0)
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, op)
    d = adsk.fusion.ExtentDirections.PositiveExtentDirection if up else \
        adsk.fusion.ExtentDirections.NegativeExtentDirection
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(abs(length) * MM)), d)
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def _pattern(comp, feature, count):
    """A circular pattern of one feature, `count` times round the component's own z axis."""
    feats = adsk.core.ObjectCollection.create()
    feats.add(feature)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(count)
    pi.totalAngle = VS('360 deg')
    pi.isSymmetric = False
    return pats.add(pi)


def _cm(pts):
    return [(x * MM, y * MM) for x, y in pts]


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


# ----------------------------------------------------------------------------- the parts
def _ring(root, g):
    occ = _new_occ(root, 'Ring')
    comp = occ.component
    R = g['ring']
    sk = _sketch(comp, comp.xYConstructionPlane, 'Ring annulus')
    _circle(sk, 0, 0, 0, R['r_in'])
    _circle(sk, 0, 0, 0, R['r_out'])
    body = _extrude(sk, _annulus(sk), g['b'], NEW).bodies.item(0)
    body.name = 'Ring'
    sk = _sketch(comp, comp.xYConstructionPlane, 'Ring tooth space 0')
    _polyline(sk, _cm(g['ring_space']))
    space = _extrude(sk, sk.profiles.item(0), g['b'], CUT, comp.bRepBodies.item(0))
    _pattern(comp, space, g['zr'])
    return occ


def _gear_teeth(comp, g, outline, z, name):
    sk = _sketch(comp, comp.xYConstructionPlane, name + ' tooth 0')
    _polyline(sk, _cm(outline))
    tooth = _extrude(sk, sk.profiles.item(0), g['b'], JOIN, comp.bRepBodies.item(0))
    _pattern(comp, tooth, z)


def _sun(root, g):
    occ = _new_occ(root, 'Sun')
    comp = occ.component
    sk = _sketch(comp, comp.xYConstructionPlane, 'Sun root disc')
    _circle(sk, 0, 0, 0, g['sun']['rf'])
    body = _extrude(sk, sk.profiles.item(0), g['b'], NEW).bodies.item(0)
    body.name = 'Sun'
    sk = _sketch(comp, _plane(comp, -SHAFT_OUT), 'Sun shaft (input)')
    _circle(sk, -SHAFT_OUT, 0, 0, g['sun_d'] / 2.0)
    _extrude(sk, sk.profiles.item(0), g['sun_top'] + SHAFT_OUT, JOIN, comp.bRepBodies.item(0))
    _gear_teeth(comp, g, g['sun_tooth'], g['zs'], 'Sun')
    return occ


def _planet(root, g, i):
    th = g['theta'][i]
    occ = _new_occ(root, 'Planet %d' % (i + 1), (g['a'] * math.cos(th), g['a'] * math.sin(th), 0.0))
    comp = occ.component
    sk = _sketch(comp, comp.xYConstructionPlane, 'Planet %d root disc' % (i + 1))
    _circle(sk, 0, 0, 0, g['planet']['rf'])
    _circle(sk, 0, 0, 0, (g['pin_d'] + 0.2) / 2.0)
    body = _extrude(sk, _annulus(sk), g['b'], NEW).bodies.item(0)
    body.name = 'Planet %d' % (i + 1)
    _gear_teeth(comp, g, g['planet_teeth'][i], g['zp'], 'Planet %d' % (i + 1))
    return occ


def _carrier(root, g):
    occ = _new_occ(root, 'Carrier')
    comp = occ.component
    z0, z1 = g['z_car0'], g['z_car1']
    bottom = _plane(comp, z0)
    sk = _sketch(comp, bottom, 'Carrier disc')
    _circle(sk, z0, 0, 0, g['car_r'])
    body = _extrude(sk, sk.profiles.item(0), g['plate'], NEW).bodies.item(0)
    body.name = 'Carrier'
    sk = _sketch(comp, bottom, 'Planet pins')
    for th in g['theta']:
        _circle(sk, z0, g['a'] * math.cos(th), g['a'] * math.sin(th), g['pin_d'] / 2.0)
    _extrude(sk, _all_profiles(sk), -z0, JOIN, comp.bRepBodies.item(0))          # down to z = 0
    sk = _sketch(comp, bottom, 'Bearing bore')
    _circle(sk, z0, 0, 0, g['bore_d'] / 2.0)
    _extrude(sk, sk.profiles.item(0), g['bore_depth'], CUT, comp.bRepBodies.item(0))
    sk = _sketch(comp, _plane(comp, z1), 'Output shaft')
    _circle(sk, z1, 0, 0, g['out_d'] / 2.0)
    _extrude(sk, sk.profiles.item(0), SHAFT_OUT, JOIN, comp.bRepBodies.item(0))
    return occ


def _sun_bearing(root, g):
    occ = _new_occ(root, 'Sun bearing')
    comp = occ.component
    z0 = g['z_car0']
    sk = _sketch(comp, _plane(comp, z0), 'Sun bearing sleeve')
    _circle(sk, z0, 0, 0, g['brg_id'] / 2.0)
    _circle(sk, z0, 0, 0, g['brg_od'] / 2.0)
    body = _extrude(sk, _annulus(sk), g['bore_depth'], NEW).bodies.item(0)
    body.name = 'Sun bearing'
    return occ


# ----------------------------------------------------------------------------- joints and motion links
def _joint_point(occ):
    """The origin of the occurrence's component, in the assembly: the joint's origin, its axes the component's."""
    comp = occ.component
    try:
        return adsk.fusion.JointGeometry.createByPoint(comp.originConstructionPoint.createForAssemblyContext(occ))
    except Exception:
        sk = _sketch(comp, comp.xYConstructionPlane, 'Joint origin')
        pt = sk.sketchPoints.add(P3(0, 0, 0))
        return adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(occ))


def _as_built(root, moving, base, name, revolute=True):
    joints = root.asBuiltJoints
    inp = joints.createInput(moving, base, _joint_point(moving))
    if revolute:
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        inp.setAsRigidJointMotion()
    j = joints.add(inp)
    j.name = name
    return j


def _value(deg):
    return VS('%d deg' % deg)


def _motion_link(root, one, two, deg_one, deg_two, reverse, name):
    links = root.motionLinks
    inp = links.createInput(one, two)
    try:
        inp.valueOne = _value(deg_one)
        inp.valueTwo = _value(deg_two)
    except Exception:
        inp.valueOne = VI(math.radians(deg_one))
        inp.valueTwo = VI(math.radians(deg_two))
    inp.isReversed = reverse
    link = links.add(inp)
    try:
        link.name = name
    except Exception:
        pass
    return link


# ----------------------------------------------------------------------------- build
def build(**kwargs):
    """Build the gearbox in a new design from the inputs (mm, deg; DEFAULTS for any left out). Returns the closing
    message; raises ValueError, before any document is opened, when the gearbox cannot be built."""
    p = dict(DEFAULTS)
    p.update(kwargs)
    g = derive(**p)
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    ring = _ring(root, g)
    ring.isGrounded = True
    sun = _sun(root, g)
    planets = [_planet(root, g, i) for i in range(g['n'])]
    carrier = _carrier(root, g)
    bearing = _sun_bearing(root, g)
    j_sun = _as_built(root, sun, ring, 'Sun (input)')
    j_car = _as_built(root, carrier, ring, 'Carrier (output)')
    j_pl = [_as_built(root, pl, carrier, 'Planet %d on carrier' % (i + 1)) for i, pl in enumerate(planets)]
    _as_built(root, bearing, carrier, 'Sun bearing in carrier', revolute=False)
    zs, zp, zr = g['zs'], g['zp'], g['zr']
    _motion_link(root, j_sun, j_car, (zs + zr) * 360, zs * 360, False, 'Sun to carrier')
    for i, j in enumerate(j_pl):
        _motion_link(root, j_car, j, zp * 360, zr * 360, True, 'Carrier to planet %d' % (i + 1))
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return message(g)


# ----------------------------------------------------------------------------- the dialog
_MM_INPUTS = [('m', 'Module m', 'm'), ('b', 'Face width b', 'b'), ('backlash', 'Backlash on the pitch circle', 'backlash'),
              ('pin_d', 'Planet pin diameter', 'pin_d'), ('sun_d', 'Sun shaft diameter (input)', 'sun_d'),
              ('out_d', 'Carrier shaft diameter (output)', 'out_d'), ('plate', 'Carrier plate thickness', 'plate'),
              ('wall', 'Ring wall outside the tooth spaces', 'wall')]
_INT_INPUTS = [('zs', 'Sun teeth Zs', 6, 400), ('zp', 'Planet teeth Zp', 6, 400), ('n', 'Number of planets N', 1, 24)]


def _read(inputs):
    p = {}
    for cid, _label, key in _MM_INPUTS:
        p[key] = inputs.itemById(cid).value / MM                  # cm to mm
    for cid, _label, _lo, _hi in _INT_INPUTS:
        p[cid] = int(inputs.itemById(cid).value)
    p['alpha'] = math.degrees(inputs.itemById('alpha').value)    # rad to deg
    return p


def _derived_text(inputs):
    try:
        g = derive(**_read(inputs))
        return ('Zr = %d, a = %.3f mm<br>ring held %.4f, carrier held %.4f, sun held %.4f'
                % (g['zr'], g['a'], g['ratio_ring_held'], g['ratio_carrier_held'], g['ratio_sun_held']))
    except Exception as e:
        return 'Cannot be built: %s' % e


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            cmd.okButtonText = 'Build'
            ins = cmd.commandInputs
            d = DEFAULTS
            ins.addValueInput('m', 'Module m', 'mm', VS('%g mm' % d['m']))
            for cid, label, lo, hi in _INT_INPUTS:
                ins.addIntegerSpinnerCommandInput(cid, label, lo, hi, 1, int(d[cid]))
            ins.addValueInput('alpha', 'Pressure angle', 'deg', VS('%g deg' % d['alpha']))
            for cid, label, key in _MM_INPUTS[1:]:
                ins.addValueInput(cid, label, 'mm', VS('%g mm' % d[key]))
            ins.addTextBoxCommandInput('derived', 'Derived', _derived_text(ins), 3, True)
            for ev, h in ((cmd.execute, _Execute()), (cmd.inputChanged, _Changed()), (cmd.destroy, _Destroy())):
                ev.add(h)
                _handlers.append(h)
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('The dialog failed:\n%s' % traceback.format_exc(), TITLE)


class _Changed(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.inputs
            box = ins.itemById('derived')
            if box is not None and args.input.id != 'derived':
                box.formattedText = _derived_text(ins)
        except Exception:
            pass


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            msg = build(**_read(args.command.commandInputs))
            ui.messageBox(msg, TITLE)
        except ValueError as e:
            ui.messageBox('%s: refused, %s.' % (VERSION, e), TITLE)
        except Exception:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()), TITLE)


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        adsk.terminate()


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(CMD_ID, TITLE, 'Builds a parametric planetary gearbox.')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()), TITLE)


def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
    except Exception:
        pass
